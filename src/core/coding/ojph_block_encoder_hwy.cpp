//***************************************************************************/
// This software is released under the 2-Clause BSD license, included
// below.
//
// Copyright (c) 2019, Aous Naman
// Copyright (c) 2019, Kakadu Software Pty Ltd, Australia
// Copyright (c) 2019, The University of New South Wales, Australia
// Copyright (c) 2024, Intel Corporation
// Copyright (c) 2026, Osamu Watanabe
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// 1. Redistributions of source code must retain the above copyright
// notice, this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
// IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
// TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
// PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
// TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//***************************************************************************/
// This file is part of the OpenJPH software implementation.
// File: ojph_block_encoder_hwy.cpp
//
// A port of the AVX2 HT cleanup-pass block encoder to Google Highway,
// compiled once per x86 target through hwy's foreach_target mechanism
// and selected at run time (dynamic dispatch).  The vector part
// (per-quad significance/exponent/magnitude computation) is written for
// a width-agnostic lane count: 8 x 32-bit lanes on 256-bit and wider
// targets, degrading to 4 lanes on 128-bit targets (SSE4).  The serial
// MEL/VLC/MagSgn bit-packing is identical to the AVX2 file, so the
// produced codestream is byte-identical for every target.
//***************************************************************************/

#include "ojph_arch.h"
#if defined(OJPH_ENABLE_HWY) \
    && (defined(OJPH_ARCH_I386) || defined(OJPH_ARCH_X86_64))

#include <cassert>
#include <cstring>
#include <cstdint>
#include <climits>
#include <mutex>

#include "ojph_mem.h"
#include "ojph_block_encoder.h"
#include "ojph_message.h"

#undef HWY_TARGET_INCLUDE
#define HWY_TARGET_INCLUDE "coding/ojph_block_encoder_hwy.cpp"
#include <hwy/foreach_target.h>
#include <hwy/highway.h>

//***************************************************************************/
// declarations shared by all compiled targets; the tables are pure data,
// so a single copy serves every target (definitions are at the bottom of
// this file, in the once-per-TU section)
#ifndef OJPH_BLOCK_ENCODER_HWY_SHARED
#define OJPH_BLOCK_ENCODER_HWY_SHARED
namespace ojph {
  namespace local {

    //VLC encoding
    // index is (c_q << 8) + (rho << 4) + eps
    // data is  (cwd << 8) + (cwd_len << 4) + eps
    // table 0 is for the initial line of quads
    extern ui32 enc_hwy_vlc_tbl0[2048];
    extern ui32 enc_hwy_vlc_tbl1[2048];

    //UVLC encoding, pair tables (see uvlc_init_pair_tables)
    extern ui32 enc_hwy_uvlc_tbl_pair1[33 * 33];
    extern ui32 enc_hwy_uvlc_tbl_pair2[33 * 33];

  } /* namespace local */
} /* namespace ojph */
#endif // OJPH_BLOCK_ENCODER_HWY_SHARED

//***************************************************************************/
// per-target section; compiled once for every enabled target, with the
// target's codegen attributes applied to everything in the region (the
// serial coders included, so they keep BMI2 & co. on the AVX2+ targets)
HWY_BEFORE_NAMESPACE();
namespace ojph {
  namespace local {
    namespace HWY_NAMESPACE {

    namespace hn = hwy::HWY_NAMESPACE;

#if HWY_TARGET == HWY_SCALAR
    // The single-lane SCALAR target (hwy's fallback when it considers the
    // vector-emulation fallback broken for the compiler at hand) cannot
    // express the multi-lane kernels below.  It can only be reached on
    // x86 CPUs without SSE4.1, which the run-time gate in
    // ojph_codeblock_fun.cpp keeps on the generic encoder anyway, so
    // simply forward to the generic encoder.
    void hwy_encode_codeblock(ui32* buf, ui32 missing_msbs,
                              ui32 num_passes, ui32 _width, ui32 height,
                              ui32 stride, ui32* lengths,
                              ojph::mem_elastic_allocator *elastic,
                              ojph::coded_lists *& coded)
    {
      ojph_encode_codeblock32(buf, missing_msbs, num_passes, _width,
                              height, stride, lengths, elastic, coded);
    }
#else

    /////////////////////////////////////////////////////////////////////////
    //
    /////////////////////////////////////////////////////////////////////////
    struct mel_struct {
      //storage
      ui8* buf;      //pointer to data buffer
      ui32 pos;      //position of next writing within buf
      ui32 buf_size; //size of buffer, which we must not exceed

      // all these can be replaced by bytes
      int remaining_bits; //number of empty bits in tmp
      int tmp;            //temporary storage of coded bits
      int run;            //number of 0 run
      int k;              //state
      int threshold;      //threshold where one bit must be coded
    };

    //////////////////////////////////////////////////////////////////////////
    static inline void
    mel_init(mel_struct* melp, ui32 buffer_size, ui8* data)
    {
      melp->buf = data;
      melp->pos = 0;
      melp->buf_size = buffer_size;
      melp->remaining_bits = 8;
      melp->tmp = 0;
      melp->run = 0;
      melp->k = 0;
      melp->threshold = 1; // this is 1 << mel_exp[melp->k];
    }

    static const int mel_exp[13] = {0,0,0,1,1,1,2,2,2,3,3,4,5};

    //////////////////////////////////////////////////////////////////////////
    static inline void
    mel_emit_bits(mel_struct* melp, ui32 bits, int num_bits)
    {
      melp->tmp = (melp->tmp << num_bits) | (int)bits;
      melp->remaining_bits -= num_bits;
      if (melp->remaining_bits <= 0) {
        int excess = -melp->remaining_bits;
        ui8 byte = (ui8)(melp->tmp >> excess);
        melp->buf[melp->pos++] = byte;
        melp->tmp &= (1 << excess) - 1;
        melp->remaining_bits += 8 - (byte == 0xFF);
      }
    }

    //////////////////////////////////////////////////////////////////////////
    static inline void
    mel_encode(mel_struct* melp, bool bit)
    {
      if (bit == false) {
        ++melp->run;
        if (melp->run >= melp->threshold) {
          mel_emit_bits(melp, 1, 1);
          melp->run = 0;
          melp->k = ojph_min(12, melp->k + 1);
          melp->threshold = 1 << mel_exp[melp->k];
        }
      } else {
        int t = mel_exp[melp->k];
        mel_emit_bits(melp, (ui32)melp->run & ((1u << t) - 1), t + 1);
        melp->run = 0;
        melp->k = ojph_max(0, melp->k - 1);
        melp->threshold = 1 << mel_exp[melp->k];
      }
    }

    //////////////////////////////////////////////////////////////////////////
    // exactly equivalent to n calls of mel_encode(melp, false); the run
    // counter is bumped in chunks, emitting a 1 whenever it reaches the
    // threshold (run < threshold is an invariant of mel_encode)
    static inline void
    mel_advance_run(mel_struct* melp, ui32 n)
    {
      ui32 remaining = n;
      while (remaining > 0) {
        ui32 space = (ui32)melp->threshold - (ui32)melp->run;
        if (remaining >= space) {
          remaining -= space;
          mel_emit_bits(melp, 1, 1);
          melp->run = 0;
          melp->k = ojph_min(12, melp->k + 1);
          melp->threshold = 1 << mel_exp[melp->k];
        } else {
          melp->run += (int)remaining;
          remaining = 0;
        }
      }
    }

    /////////////////////////////////////////////////////////////////////////
    //
    /////////////////////////////////////////////////////////////////////////

    struct vlc_struct {
      //storage
      ui8* buf;      //pointer to data buffer
      ui32 pos;      //position of next writing within buf
      ui32 buf_size; //size of buffer, which we must not exceed

      int used_bits; //number of occupied bits in tmp
      ui64 tmp;       //temporary storage of coded bits
      bool last_greater_than_8F; //true if last byte us greater than 0x8F
    };

    //////////////////////////////////////////////////////////////////////////
    static inline void
    vlc_init(vlc_struct* vlcp, ui32 buffer_size, ui8* data)
    {
      vlcp->buf = data + buffer_size - 1; //points to last byte
      vlcp->pos = 1;                      //locations will be all -pos
      vlcp->buf_size = buffer_size;

      vlcp->buf[0] = 0xFF;
      vlcp->used_bits = 4;
      vlcp->tmp = 0xF;
      vlcp->last_greater_than_8F = true;
    }

    //////////////////////////////////////////////////////////////////////////
    static inline void
    vlc_drain(vlc_struct* vlcp)
    {
      while (vlcp->used_bits >= 8) {
        int escape = (int)vlcp->last_greater_than_8F;
        int is_7f = (int)((vlcp->tmp & 0x7F) == 0x7F);
        int need_stuff = (escape & is_7f) != 0 ? 1 : 0;
        int bits = 8 - need_stuff;

        ui8 byte = (ui8)(vlcp->tmp & ((1u << bits) - 1));
        *(vlcp->buf - vlcp->pos) = byte;
        vlcp->pos++;
        vlcp->tmp >>= bits;
        vlcp->used_bits -= bits;
        vlcp->last_greater_than_8F = byte > 0x8F;
      }
    }

    //////////////////////////////////////////////////////////////////////////
    static inline void
    vlc_encode(vlc_struct* vlcp, ui64 cwd, int cwd_len)
    {
      while (true) {
        int avail = 64 - vlcp->used_bits;
        if (HWY_LIKELY(avail > 0 && cwd_len <= avail)) {
          vlcp->tmp |= cwd << vlcp->used_bits;
          vlcp->used_bits += cwd_len;
          return;
        }
        if (HWY_LIKELY(avail > 0)) // available space smaller than needed
          vlcp->tmp |= cwd << vlcp->used_bits;
        vlcp->used_bits = 64;
        vlc_drain(vlcp);
        cwd >>= avail;
        cwd_len -= avail;
      }
    }

    //////////////////////////////////////////////////////////////////////////
    //
    //////////////////////////////////////////////////////////////////////////
    static inline void
    terminate_mel_vlc(mel_struct* melp, vlc_struct* vlcp)
    {
      if (melp->run > 0)
        mel_emit_bits(melp, 1, 1);

      if (vlcp->last_greater_than_8F && (vlcp->tmp & 0x7f) == 0x7f) {
        *(vlcp->buf - vlcp->pos) = 0x7f;
        vlcp->pos++;
        vlcp->tmp >>= 7;
        vlcp->used_bits -= 7;
      }

      melp->tmp = melp->tmp << melp->remaining_bits;
      int mel_mask = (0xFF << melp->remaining_bits) & 0xFF;
      int vlc_mask = 0xFF >> (8 - vlcp->used_bits);
      if ((mel_mask | vlc_mask) == 0)
        return;  //last mel byte cannot be 0xFF, since then
                 //melp->remaining_bits would be < 8
      if (melp->pos >= melp->buf_size)
        OJPH_ERROR(0x00020011, "mel encoder's buffer is full");
      ui8 vlcp_tmp = (ui8)vlcp->tmp;
      int fuse = melp->tmp | vlcp_tmp;
      if ( ( ((fuse ^ melp->tmp) & mel_mask)
           | ((fuse ^ vlcp_tmp) & vlc_mask) ) == 0
          && (fuse != 0xFF) && vlcp->pos > 1)
      {
        melp->buf[melp->pos++] = (ui8)fuse;
      }
      else
      {
        if (vlcp->pos >= vlcp->buf_size)
          OJPH_ERROR(0x00020012, "vlc encoder's buffer is full");
        melp->buf[melp->pos++] = (ui8)melp->tmp; //melp->tmp cannot be 0xFF
        *(vlcp->buf - vlcp->pos) = (ui8)vlcp_tmp;
        vlcp->pos++;
      }
    }

/////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////

    struct ms_struct {
      //storage
      ui8* buf;        //pointer to data buffer
      ui32 pos;        //position of next writing within buf
      ui32 buf_size;   //size of buffer, which we must not exceed

      int used_bits;   //number of occupied bits in tmp
      ui64 tmp;        //temporary storage of coded bits (64-bit accumulator)
      bool last_was_ff;//true if the last written byte was 0xFF
    };

    //////////////////////////////////////////////////////////////////////////
    static inline void
    ms_init(ms_struct* msp, ui32 buffer_size, ui8* data)
    {
      msp->buf = data;
      msp->pos = 0;
      msp->buf_size = buffer_size;
      msp->used_bits = 0;
      msp->tmp = 0;
      msp->last_was_ff = false;
    }

    //////////////////////////////////////////////////////////////////////////
    static inline void
    ms_drain(ms_struct* msp)
    {
      if (msp->last_was_ff) {
        if (msp->used_bits < 7)
          return;
        msp->buf[msp->pos++] = (ui8)(msp->tmp & 0x7F);
        msp->tmp >>= 7;
        msp->used_bits -= 7;
        msp->last_was_ff = false;
      }

      while (msp->used_bits >= 8) {
        int n_bytes = msp->used_bits >> 3;
        if (n_bytes > 8) n_bytes = 8;

        ui64 word = msp->tmp;
        ui64 valid_mask = (n_bytes < 8)
                        ? (1ULL << (n_bytes * 8)) - 1 : ~(ui64)0;

        ui64 w = ~word;
        ui64 ff_detect = (w - 0x0101010101010101ULL) & ~w
                       & 0x8080808080808080ULL;
        ff_detect &= valid_mask;

        if (HWY_LIKELY(ff_detect == 0)) {
          memcpy(msp->buf + msp->pos, &word, (size_t)n_bytes);
          msp->pos += (ui32)n_bytes;
          if (n_bytes < 8)
            msp->tmp >>= (n_bytes * 8);
          else
            msp->tmp = 0;
          msp->used_bits -= n_bytes * 8;
        } else {
          int ff_pos = (int)(count_trailing_zeros(ff_detect) >> 3);
          int safe = ff_pos + 1;
          memcpy(msp->buf + msp->pos, &word, (size_t)safe);
          msp->pos += (ui32)safe;
          int bits = safe * 8;
          if (bits < 64)
            msp->tmp >>= bits;
          else
            msp->tmp = 0;
          msp->used_bits -= bits;

          if (msp->used_bits >= 7) {
            msp->buf[msp->pos++] = (ui8)(msp->tmp & 0x7F);
            msp->tmp >>= 7;
            msp->used_bits -= 7;
            msp->last_was_ff = false;
          } else {
            msp->last_was_ff = true;
            return;
          }
        }
      }
    }

    //////////////////////////////////////////////////////////////////////////
    static inline void
    ms_encode_nodefer(ms_struct* msp, ui64 cwd, int cwd_len)
    {
      while (true) {
        int avail = 64 - msp->used_bits;
        if (HWY_LIKELY(avail > 0 && cwd_len <= avail)) {
          msp->tmp |= cwd << msp->used_bits;
          msp->used_bits += cwd_len;
          return;
        }
        if (HWY_LIKELY(avail > 0)) // available space smaller than needed
          msp->tmp |= cwd << msp->used_bits;
        msp->used_bits = 64;
        ms_drain(msp);
        cwd >>= avail;
        cwd_len -= avail;
      }
    }

    //////////////////////////////////////////////////////////////////////////
    static inline void
    ms_terminate(ms_struct* msp)
    {
      ms_drain(msp);
      if (msp->used_bits)
      {
        int max_bits = msp->last_was_ff ? 7 : 8;
        int t = max_bits - msp->used_bits;
        ui32 byte = (ui32)(msp->tmp & ((1ULL << msp->used_bits) - 1));
        byte |= (0xFFu & ((1u << t) - 1)) << msp->used_bits;
        if (byte != 0xFF)
        {
          if (msp->pos >= msp->buf_size)
            OJPH_ERROR(0x00020013, "magnitude sign encoder's buffer is full");
          msp->buf[msp->pos++] = (ui8)byte;
        }
      }
      else if (msp->last_was_ff)
        msp->pos--;
    }

    //////////////////////////////////////////////////////////////////////////
    // vector helpers; the lane count VL is 8 x 32-bit lanes on 256-bit and
    // wider targets, and 4 on 128-bit targets.  Each x-iteration of the
    // encoder processes 2 * VL samples in each of 2 lines, i.e. VL quads.
    //////////////////////////////////////////////////////////////////////////

    using tag_u32 = hn::CappedTag<uint32_t, 8>;
    using tag_i32 = hn::RebindToSigned<tag_u32>;
    using vec_u32 = hn::Vec<tag_u32>;

    static constexpr tag_u32 du;
    static constexpr tag_i32 di;

    // lanes per vector = quads per x-iteration (a compile-time constant;
    // on x86 the capped tag always has exactly this many lanes)
    static constexpr ui32 VL = (ui32)hn::MaxLanes(tag_u32());

    // the widest codeblock is 1024 samples, i.e. this many x-iterations
    static constexpr ui32 max_n_loop = 1024 / (2 * VL);

    static inline vec_u32 v_zero() { return hn::Zero(du); }
    static inline vec_u32 v_one() { return hn::Set(du, 1); }

    // 0xFFFFFFFF per lane where a != b, else 0
    static inline vec_u32 v_cmpneq(vec_u32 a, vec_u32 b)
    {
      return hn::VecFromMask(du, hn::Ne(a, b));
    }

    // 0xFFFFFFFF per lane where a == b, else 0
    static inline vec_u32 v_cmpeq(vec_u32 a, vec_u32 b)
    {
      return hn::VecFromMask(du, hn::Eq(a, b));
    }

    // 0xFFFFFFFF per lane where a > b (signed), else 0
    static inline vec_u32 v_cmpgt(vec_u32 a, vec_u32 b)
    {
      return hn::BitCast(du, hn::VecFromMask(di,
        hn::Gt(hn::BitCast(di, a), hn::BitCast(di, b))));
    }

    static inline vec_u32 v_max(vec_u32 a, vec_u32 b)
    {
      return hn::BitCast(du, hn::Max(hn::BitCast(di, a),
                                     hn::BitCast(di, b)));
    }

    static inline vec_u32 v_min(vec_u32 a, vec_u32 b)
    {
      return hn::BitCast(du, hn::Min(hn::BitCast(di, a),
                                     hn::BitCast(di, b)));
    }

    // rotate all lanes down by one, injecting a scalar into the top lane
    // (lane i receives lane i + 1, lane VL - 1 receives the scalar)
    static inline vec_u32 v_shift_down(vec_u32 v, ui32 top)
    {
      return hn::InsertLane(hn::Slide1Down(du, v), VL - 1, top);
    }

    // rotate all lanes up by one, injecting a scalar into the bottom lane
    // (lane i receives lane i - 1, lane 0 receives the scalar)
    static inline vec_u32 v_shift_up(vec_u32 v, ui32 bottom)
    {
      return hn::InsertLane(hn::Slide1Up(du, v), 0, bottom);
    }

    static void proc_pixel(vec_u32 *src_vec, ui32 p,
                           vec_u32 *eq_vec, vec_u32 *s_vec,
                           vec_u32 &rho_vec, vec_u32 &e_qmax_vec)
    {
      vec_u32 val_vec[4];
      vec_u32 _eq_vec[4];
      vec_u32 _s_vec[4];
      vec_u32 _rho_vec[4];

      for (ui32 i = 0; i < 4; ++i) {
        /* val = t + t; //multiply by 2 and get rid of sign */
        val_vec[i] = hn::Add(src_vec[i], src_vec[i]);

        /* val >>= p;  // 2 \mu_p + x */
        val_vec[i] = hn::ShiftRightSame(val_vec[i], (int)p);

        /* val &= ~1u; // 2 \mu_p */
        val_vec[i] = hn::And(val_vec[i], hn::Set(du, ~1u));

        /* if (val) { */
        const vec_u32 val_notmask = v_cmpneq(val_vec[i], v_zero());

        /*   rho[i] = 1 << i;
         *   rho is processed below.
         */

        /*   e_q[i] = 32 - (int)count_leading_zeros(--val); //2\mu_p - 1 */
        val_vec[i] = hn::Sub(val_vec[i], v_one());
        _eq_vec[i] = hn::Sub(hn::Set(du, 32),
                             hn::LeadingZeroCount(val_vec[i]));

        /*   e_qmax[i] = ojph_max(e_qmax[i], e_q[j]);
         *   e_qmax is processed below
         */

        /*   s[0] = --val + (t >> 31); //v_n = 2(\mu_p-1) + s_n */
        val_vec[i] = hn::Sub(val_vec[i], v_one());
        _s_vec[i] = hn::ShiftRight<31>(src_vec[i]);
        _s_vec[i] = hn::Add(_s_vec[i], val_vec[i]);

        _eq_vec[i] = hn::And(_eq_vec[i], val_notmask);
        _s_vec[i] = hn::And(_s_vec[i], val_notmask);
        val_vec[i] = hn::ShiftRight<31>(val_notmask);
        /* } */
      }

      /* Deinterleave the even and odd columns; reorder from
       * *_vec[0]:[0, 0], [0, 1], ..., [0, VL-1]      (line 0, left half)
       * *_vec[1]:[1, 0], [1, 1], ..., [1, VL-1]      (line 1, left half)
       * *_vec[2]:[0, VL], ..., [0, 2*VL-1]           (line 0, right half)
       * *_vec[3]:[1, VL], ..., [1, 2*VL-1]           (line 1, right half)
       * to
       * *_vec[0]:[0, 0], [0, 2], ..., [0, 2*VL-2]    (line 0, even cols)
       * *_vec[1]:[1, 0], [1, 2], ..., [1, 2*VL-2]    (line 1, even cols)
       * *_vec[2]:[0, 1], [0, 3], ..., [0, 2*VL-1]    (line 0, odd cols)
       * *_vec[3]:[1, 1], [1, 3], ..., [1, 2*VL-1]    (line 1, odd cols)
       * so that eq_vec[j]/s_vec[j] hold sample j of the VL quads
       */
      for (ui32 i = 0; i < 2; ++i) {
        eq_vec[0 + i] = hn::ConcatEven(du, _eq_vec[2 + i], _eq_vec[0 + i]);
        eq_vec[2 + i] = hn::ConcatOdd(du, _eq_vec[2 + i], _eq_vec[0 + i]);

        s_vec[0 + i] = hn::ConcatEven(du, _s_vec[2 + i], _s_vec[0 + i]);
        s_vec[2 + i] = hn::ConcatOdd(du, _s_vec[2 + i], _s_vec[0 + i]);

        _rho_vec[0 + i] = hn::ConcatEven(du, val_vec[2 + i], val_vec[0 + i]);
        _rho_vec[2 + i] = hn::ConcatOdd(du, val_vec[2 + i], val_vec[0 + i]);
      }

      e_qmax_vec = v_max(eq_vec[0], eq_vec[1]);
      e_qmax_vec = v_max(e_qmax_vec, eq_vec[2]);
      e_qmax_vec = v_max(e_qmax_vec, eq_vec[3]);
      _rho_vec[1] = hn::ShiftLeft<1>(_rho_vec[1]);
      _rho_vec[2] = hn::ShiftLeft<2>(_rho_vec[2]);
      _rho_vec[3] = hn::ShiftLeft<3>(_rho_vec[3]);
      rho_vec = hn::Or(_rho_vec[0], _rho_vec[1]);
      rho_vec = hn::Or(rho_vec, _rho_vec[2]);
      rho_vec = hn::Or(rho_vec, _rho_vec[3]);
    }

    static void proc_ms_encode(ms_struct *msp,
                               vec_u32 &tuple_vec,
                               vec_u32 &uq_vec,
                               vec_u32 &rho_vec,
                               vec_u32 *s_vec)
    {
      vec_u32 m_vec[4];

      /* Prepare parameters for ms_encode */
      /* m = (rho[i] & 1) ? Uq[i] - ((tuple[i] & 1) >> 0) : 0; */
      auto tmp = hn::And(tuple_vec, v_one());
      tmp = hn::Sub(uq_vec, tmp);
      auto tmp1 = hn::And(rho_vec, v_one());
      auto mask = v_cmpneq(tmp1, v_zero());
      m_vec[0] = hn::And(mask, tmp);

      /* m = (rho[i] & 2) ? Uq[i] - ((tuple[i] & 2) >> 1) : 0; */
      tmp = hn::And(tuple_vec, hn::Set(du, 2));
      tmp = hn::ShiftRight<1>(tmp);
      tmp = hn::Sub(uq_vec, tmp);
      tmp1 = hn::And(rho_vec, hn::Set(du, 2));
      mask = v_cmpneq(tmp1, v_zero());
      m_vec[1] = hn::And(mask, tmp);

      /* m = (rho[i] & 4) ? Uq[i] - ((tuple[i] & 4) >> 2) : 0; */
      tmp = hn::And(tuple_vec, hn::Set(du, 4));
      tmp = hn::ShiftRight<2>(tmp);
      tmp = hn::Sub(uq_vec, tmp);
      tmp1 = hn::And(rho_vec, hn::Set(du, 4));
      mask = v_cmpneq(tmp1, v_zero());
      m_vec[2] = hn::And(mask, tmp);

      /* m = (rho[i] & 8) ? Uq[i] - ((tuple[i] & 8) >> 3) : 0; */
      tmp = hn::And(tuple_vec, hn::Set(du, 8));
      tmp = hn::ShiftRight<3>(tmp);
      tmp = hn::Sub(uq_vec, tmp);
      tmp1 = hn::And(rho_vec, hn::Set(du, 8));
      mask = v_cmpneq(tmp1, v_zero());
      m_vec[3] = hn::And(mask, tmp);

      /* cwd = s[i] & ((1U << m) - 1); cwd_len = m; computed on the
       * per-sample rows; the transpose to per-quad emission order
       * happens through the scalar indexing below, [sample][quad]
       */
      ui32 cwd[4][VL];
      ui32 cwd_len[4][VL];
      for (ui32 i = 0; i < 4; ++i) {
        hn::StoreU(m_vec[i], du, cwd_len[i]);
        tmp = hn::Shl(v_one(), m_vec[i]);
        tmp = hn::Sub(tmp, v_one());
        tmp = hn::And(tmp, s_vec[i]);
        hn::StoreU(tmp, du, cwd[i]);
      }

      /* per quad: emit samples 0..3; all four fused into one call when
       * they fit the 64-bit accumulator, otherwise two calls of two
       */
      for (ui32 q = 0; q < VL; ++q) {
        ui64 _cwd     = cwd[0][q];
        int  _cwd_len = (int)cwd_len[0][q];
        _cwd     |= ((ui64)cwd[1][q]) << _cwd_len;
        _cwd_len += (int)cwd_len[1][q];

        int len1 = (int)cwd_len[2][q] + (int)cwd_len[3][q];
        if (HWY_LIKELY(_cwd_len + len1 <= 64)) {
          _cwd     |= ((ui64)cwd[2][q]) << _cwd_len;
          _cwd_len += (int)cwd_len[2][q];
          _cwd     |= ((ui64)cwd[3][q]) << _cwd_len;
          _cwd_len += (int)cwd_len[3][q];
          ms_encode_nodefer(msp, _cwd, _cwd_len);
        } else {
          ms_encode_nodefer(msp, _cwd, _cwd_len);
          _cwd     = cwd[2][q];
          _cwd_len = (int)cwd_len[2][q];
          _cwd     |= ((ui64)cwd[3][q]) << _cwd_len;
          _cwd_len += (int)cwd_len[3][q];
          ms_encode_nodefer(msp, _cwd, _cwd_len);
        }
      }
      ms_drain(msp);
    }

    static vec_u32 cal_eps_vec(vec_u32 *eq_vec, vec_u32 &u_q_vec,
                               vec_u32 &e_qmax_vec)
    {
      /* if (u_q[i] > 0) {
       *     eps[i] |= (e_q[i * 4 + 0] == e_qmax[i]);
       *     eps[i] |= (e_q[i * 4 + 1] == e_qmax[i]) << 1;
       *     eps[i] |= (e_q[i * 4 + 2] == e_qmax[i]) << 2;
       *     eps[i] |= (e_q[i * 4 + 3] == e_qmax[i]) << 3;
       * }
       */
      auto u_q_mask = v_cmpgt(u_q_vec, v_zero());

      auto mask = v_cmpeq(eq_vec[0], e_qmax_vec);
      auto eps_vec = hn::ShiftRight<31>(mask);

      mask = v_cmpeq(eq_vec[1], e_qmax_vec);
      auto tmp = hn::ShiftRight<31>(mask);
      tmp = hn::ShiftLeft<1>(tmp);
      eps_vec = hn::Or(eps_vec, tmp);

      mask = v_cmpeq(eq_vec[2], e_qmax_vec);
      tmp = hn::ShiftRight<31>(mask);
      tmp = hn::ShiftLeft<2>(tmp);
      eps_vec = hn::Or(eps_vec, tmp);

      mask = v_cmpeq(eq_vec[3], e_qmax_vec);
      tmp = hn::ShiftRight<31>(mask);
      tmp = hn::ShiftLeft<3>(tmp);
      eps_vec = hn::Or(eps_vec, tmp);

      return hn::And(u_q_mask, eps_vec);
    }

    static void update_lep(ui32 x, vec_u32 &prev_e_val_vec,
                           vec_u32 *eq_vec, vec_u32 *e_val_vec)
    {
      /* lep[0] = ojph_max(lep[0], (ui8)e_q[1]); lep++;
       * lep[0] = (ui8)e_q[3];
       * Compare e_q[1] with e_q[3] of the previous round.
       */
      auto tmp = v_shift_up(eq_vec[3], hn::GetLane(prev_e_val_vec));
      prev_e_val_vec = hn::InsertLane(v_zero(), 0,
                                      hn::ExtractLane(eq_vec[3], VL - 1));
      e_val_vec[x] = v_max(eq_vec[1], tmp);
    }

    static void update_lcxp(ui32 x, vec_u32 &prev_cx_val_vec,
                            vec_u32 &rho_vec, vec_u32 *cx_val_vec)
    {
      /* lcxp[0] = (ui8)(lcxp[0] | (ui8)((rho[0] & 2) >> 1)); lcxp++;
       * lcxp[0] = (ui8)((rho[0] & 8) >> 3);
       * Or (rho[0] & 2) and (rho[0] of the previous round & 8).
       */
      auto tmp = v_shift_up(rho_vec, hn::GetLane(prev_cx_val_vec));
      prev_cx_val_vec = hn::InsertLane(v_zero(), 0,
                                       hn::ExtractLane(rho_vec, VL - 1));

      tmp = hn::And(tmp, hn::Set(du, 8));
      tmp = hn::ShiftRight<3>(tmp);

      auto tmp1 = hn::And(rho_vec, hn::Set(du, 2));
      tmp1 = hn::ShiftRight<1>(tmp1);
      cx_val_vec[x] = hn::Or(tmp, tmp1);
    }

    static vec_u32 cal_tuple(vec_u32 &cq_vec, vec_u32 &rho_vec,
                             vec_u32 &eps_vec, ui32 *vlc_tbl)
    {
      /* tuple[i] = enc_hwy_vlc_tbl1[(c_q[i] << 8) + (rho[i] << 4) + eps[i]]; */
      auto tmp = hn::ShiftLeft<8>(cq_vec);
      auto tmp1 = hn::ShiftLeft<4>(rho_vec);
      tmp = hn::Add(tmp, tmp1);
      tmp = hn::Add(tmp, eps_vec);
      return hn::GatherIndex(du, vlc_tbl, hn::BitCast(di, tmp));
    }

    static vec_u32 proc_cq1(ui32 x, vec_u32 *cx_val_vec, vec_u32 &rho_vec)
    {
      ojph_unused(x);
      ojph_unused(cx_val_vec);

      /* c_q[i + 1] = (rho[i] >> 1) | (rho[i] & 1); */
      auto tmp = hn::ShiftRight<1>(rho_vec);
      auto tmp1 = hn::And(rho_vec, v_one());
      return hn::Or(tmp, tmp1);
    }

    static vec_u32 proc_cq2(ui32 x, vec_u32 *cx_val_vec, vec_u32 &rho_vec)
    {
      // c_q[i + 1] = (lcxp[i + 1] + (lcxp[i + 2] << 2))
      //            | (((rho[i] & 4) >> 1) | ((rho[i] & 8) >> 2));

      // lcxp[i + 1]: the two top lanes come from the next iteration's
      // cx values, lanes 0 and 1
      auto lcxp1_vec = v_shift_down(cx_val_vec[x],
                                    hn::GetLane(cx_val_vec[x + 1]));
      auto tmp = v_shift_down(lcxp1_vec,
                              hn::ExtractLane(cx_val_vec[x + 1], 1));
      tmp = hn::ShiftLeft<2>(tmp);
      tmp = hn::Add(lcxp1_vec, tmp);

      auto tmp1 = hn::And(rho_vec, hn::Set(du, 4));
      tmp1 = hn::ShiftRight<1>(tmp1);
      tmp = hn::Or(tmp, tmp1);

      tmp1 = hn::And(rho_vec, hn::Set(du, 8));
      tmp1 = hn::ShiftRight<2>(tmp1);

      return hn::Or(tmp, tmp1);
    }

    static void proc_mel_encode1(mel_struct *melp, vec_u32 &cq_vec,
                                 vec_u32 &rho_vec, vec_u32 u_q_vec,
                                 ui32 ignore)
    {
      int32_t mel_need_encode[VL];
      int32_t mel_need_encode2[VL];
      int32_t mel_bit[VL];
      int32_t mel_bit2[VL];
      /* Prepare mel_encode params */
      /* if (c_q[i] == 0) { */
      hn::StoreU(hn::BitCast(di, v_cmpeq(cq_vec, v_zero())), di,
                 mel_need_encode);
      /*   mel_encode(&mel, rho[i] != 0); */
      hn::StoreU(hn::BitCast(di,
        hn::ShiftRight<31>(v_cmpneq(rho_vec, v_zero()))), di, mel_bit);
      /* } */

      /*   mel_encode(&mel, ojph_min(u_q[i], u_q[i + 1]) > 2); */
      // (only even i is read below, so the top lane's value is a don't-care)
      auto tmp = hn::Slide1Down(du, u_q_vec);
      auto tmp1 = v_min(u_q_vec, tmp);
      hn::StoreU(hn::BitCast(di,
        hn::ShiftRight<31>(v_cmpgt(tmp1, hn::Set(du, 2)))), di, mel_bit2);

      /* if (u_q[i] > 0 && u_q[i + 1] > 0) { } */
      auto need_encode2 = v_cmpgt(u_q_vec, v_zero());
      hn::StoreU(hn::BitCast(di,
        hn::And(need_encode2, v_cmpgt(tmp, v_zero()))), di,
        mel_need_encode2);

      ui32 i_max = VL - (ignore / 2);

      for (ui32 i = 0; i < i_max; i += 2) {
        if (mel_need_encode[i]) {
          mel_encode(melp, mel_bit[i]);
        }

        if (i + 1 < i_max) {
          if (mel_need_encode[i + 1]) {
            mel_encode(melp, mel_bit[i + 1]);
          }
        }

        if (mel_need_encode2[i]) {
          mel_encode(melp, mel_bit2[i]);
        }
      }
    }

    static void proc_mel_encode2(mel_struct *melp, vec_u32 &cq_vec,
                                 vec_u32 &rho_vec, vec_u32 u_q_vec,
                                 ui32 ignore)
    {
      ojph_unused(u_q_vec);

      ui32 mask = (ui32)hn::BitsFromMask(du, hn::Eq(cq_vec, v_zero()));

      ui32 i_max = VL - (ignore / 2);
      if (i_max < VL)
        mask &= (1u << i_max) - 1;

      if (mask == 0)
        return;

      int32_t mel_bit[VL];
      hn::StoreU(hn::BitCast(di,
        hn::ShiftRight<31>(v_cmpneq(rho_vec, v_zero()))), di, mel_bit);

      while (mask) {
        ui32 i = (ui32)count_trailing_zeros(mask);
        mel_encode(melp, mel_bit[i]);
        mask &= mask - 1;
      }
    }

    static inline void
    build_vlc_uvlc_pair(ui32 *tuple, ui32 *u_q, ui32 i,
                        const ui32 *uvlc_tbl, ui64 &val, int &size)
    {
      val = tuple[i + 0] >> 4;
      size = tuple[i + 0] & 7;

      val |= (ui64)(tuple[i + 1] >> 4) << size;
      size += tuple[i + 1] & 7;

      ui32 entry = uvlc_tbl[u_q[i] * 33 + u_q[i + 1]];
      val |= (ui64)(entry >> 5) << size;
      size += entry & 0x1F;
    }

    static void proc_vlc_encode(vlc_struct *vlcp, ui32 *tuple,
                                ui32 *u_q, ui32 ignore, const ui32 *uvlc_tbl)
    {
      ui32 i_max = VL - (ignore / 2);

      ui32 i = 0;
      for (; i + 2 < i_max; i += 4) {
        ui64 val1; int size1;
        build_vlc_uvlc_pair(tuple, u_q, i, uvlc_tbl, val1, size1);
        ui64 val2; int size2;
        build_vlc_uvlc_pair(tuple, u_q, i + 2, uvlc_tbl, val2, size2);
        vlc_encode(vlcp, val1 | (val2 << size1), size1 + size2);
      }
      if (i < i_max) {
        ui64 val; int size;
        build_vlc_uvlc_pair(tuple, u_q, i, uvlc_tbl, val, size);
        vlc_encode(vlcp, val, size);
      }
    }

    template<int PASS>
    OJPH_FORCE_INLINE void encode_x_loop(
        ui32 *sp, ui32 stride, ui32 height, ui32 y,
        ui32 n_loop, ui32 _width, ui32 ignore, ui32 p,
        mel_struct &mel, vlc_struct &vlc, ms_struct &ms,
        vec_u32 *e_val_vec, vec_u32 &prev_e_val_vec,
        vec_u32 *cx_val_vec, vec_u32 &prev_cx_val_vec,
        ui32 &prev_cq)
    {
      ui32 *vlc_tbl = (PASS == 1) ? enc_hwy_vlc_tbl0 : enc_hwy_vlc_tbl1;

      vec_u32 tmp, tmp1;
      vec_u32 eq_vec[4];
      vec_u32 s_vec[4];
      vec_u32 src_vec[4];

      /* 2 * VL samples per line per iteration */
      for (ui32 x = 0; x < n_loop; ++x) {

        /* t = sp[i]; */
        if ((x == (n_loop - 1)) && (_width % (2 * VL))) {
          ui32 tmp_buf[2 * VL] = { 0 };
          memcpy(tmp_buf, sp, (_width % (2 * VL)) * sizeof(ui32));
          src_vec[0] = hn::LoadU(du, tmp_buf);
          src_vec[2] = hn::LoadU(du, tmp_buf + VL);
          if (y + 1 < height) {
            memcpy(tmp_buf, sp + stride, (_width % (2 * VL)) * sizeof(ui32));
            src_vec[1] = hn::LoadU(du, tmp_buf);
            src_vec[3] = hn::LoadU(du, tmp_buf + VL);
          }
          else {
            src_vec[1] = v_zero();
            src_vec[3] = v_zero();
          }
        }
        else {
          src_vec[0] = hn::LoadU(du, sp);
          src_vec[2] = hn::LoadU(du, sp + VL);

          if (y + 1 < height) {
            src_vec[1] = hn::LoadU(du, sp + stride);
            src_vec[3] = hn::LoadU(du, sp + VL + stride);
          }
          else {
            src_vec[1] = v_zero();
            src_vec[3] = v_zero();
          }
          sp += 2 * VL;
        }

        /* Fast path: a chunk of 2*VL x 2 zero samples in a zero context
         * (incoming c_q all zero) emits no VLC/MagSgn bits at all and
         * only advances the MEL zero-run; this is the common case for
         * mask-like content, where significant codeblocks are still
         * mostly empty.  Everything it skips provably produces no
         * output: rho == 0 and c_q == 0 give zero-length VLC tuples,
         * u_q == 0 (so zero-length UVLC codewords), and m == 0 (so no
         * MagSgn bits); only the per-quad mel_encode(false) calls and
         * the line-state updates remain, and both are reproduced
         * exactly below.  Output is bit-identical to the slow path. */
        vec_u32 src_or = hn::Or(hn::Or(src_vec[0], src_vec[1]),
                                hn::Or(src_vec[2], src_vec[3]));
        if (hn::AllTrue(du, hn::Eq(src_or, v_zero()))) {
          vec_u32 rho0 = v_zero();
          vec_u32 t = (PASS == 1) ? v_zero()
                                  : proc_cq2(x, cx_val_vec, rho0);
          vec_u32 cq0_vec = v_shift_up(t, prev_cq);
          prev_cq = hn::ExtractLane(t, VL - 1);
          // update_lep / update_lcxp with all-zero e_q and rho
          e_val_vec[x] = hn::InsertLane(v_zero(), 0,
                                        hn::GetLane(prev_e_val_vec));
          prev_e_val_vec = v_zero();
          cx_val_vec[x] = hn::InsertLane(v_zero(), 0,
            (hn::GetLane(prev_cx_val_vec) & 8) >> 3);
          prev_cx_val_vec = v_zero();
          ui32 _ignore = ((n_loop - 1) == x) ? ignore : 0;
          ui32 i_max = VL - (_ignore / 2);
          // proc_mel_encode1/2 issue one mel_encode(&mel, rho != 0),
          // i.e. a zero bit, per quad whose c_q is zero; since all the
          // bits are zero runs may be batched regardless of position
          ui32 cq_zero_mask =
            (ui32)hn::BitsFromMask(du, hn::Eq(cq0_vec, v_zero()))
            & ((1u << i_max) - 1u);
          mel_advance_run(&mel, (ui32)population_count(cq_zero_mask));
          if (cq_zero_mask != ((1u << i_max) - 1u)) {
            // some incoming c_q != 0: their (rho == 0, eps == 0) VLC
            // tuples must still be emitted (u_q == 0, so the UVLC
            // codewords are zero-length, and m == 0 means no MagSgn
            // bits); everything else is as in the all-zero case
            vec_u32 tuple_vec = cal_tuple(cq0_vec, rho0, rho0,
              (PASS == 1) ? enc_hwy_vlc_tbl0 : enc_hwy_vlc_tbl1);
            ui32 u_q[VL + 2] = { 0 };
            ui32 tuple[VL + 2];
            tuple_vec = hn::ShiftRight<4>(tuple_vec);
            hn::StoreU(tuple_vec, du, tuple);
            if (i_max & 1) tuple[i_max] = 0;
            tuple[VL] = 0;
            proc_vlc_encode(&vlc, tuple, u_q, _ignore,
                (PASS == 1) ? enc_hwy_uvlc_tbl_pair1 : enc_hwy_uvlc_tbl_pair2);
          }
          continue;
        }

        vec_u32 rho_vec, e_qmax_vec;
        proc_pixel(src_vec, p, eq_vec, s_vec, rho_vec, e_qmax_vec);

        // max_e[(i + 1) % num] = ojph_max(lep[i + 1], lep[i + 2]) - 1;
        tmp = v_shift_down(e_val_vec[x], hn::GetLane(e_val_vec[x + 1]));

        auto max_e_vec = v_max(tmp, e_val_vec[x]);
        max_e_vec = hn::Sub(max_e_vec, v_one());

        // kappa[i] = (rho[i] & (rho[i] - 1)) ? ojph_max(1, max_e[i]) : 1;
        tmp = v_max(max_e_vec, v_one());
        tmp1 = hn::Sub(rho_vec, v_one());
        tmp1 = hn::And(rho_vec, tmp1);

        auto cmp = v_cmpeq(tmp1, v_zero());
        auto kappa_vec1_ = hn::And(cmp, v_one());
        auto kappa_vec2_ = hn::AndNot(cmp, tmp);
        const vec_u32 kappa_vec = v_max(kappa_vec1_, kappa_vec2_);

        if (PASS == 1)
          tmp = proc_cq1(x, cx_val_vec, rho_vec);
        else
          tmp = proc_cq2(x, cx_val_vec, rho_vec);

        auto cq_vec = v_shift_up(tmp, prev_cq);
        prev_cq = hn::ExtractLane(tmp, VL - 1);

        update_lep(x, prev_e_val_vec, eq_vec, e_val_vec);
        update_lcxp(x, prev_cx_val_vec, rho_vec, cx_val_vec);

        /* Uq[i] = ojph_max(e_qmax[i], kappa[i]); */
        /* u_q[i] = Uq[i] - kappa[i]; */
        auto uq_vec = v_max(kappa_vec, e_qmax_vec);
        auto u_q_vec = hn::Sub(uq_vec, kappa_vec);

        auto eps_vec = cal_eps_vec(eq_vec, u_q_vec, e_qmax_vec);
        vec_u32 tuple_vec = cal_tuple(cq_vec, rho_vec, eps_vec, vlc_tbl);
        ui32 _ignore = ((n_loop - 1) == x) ? ignore : 0;

        if (PASS == 1)
          proc_mel_encode1(&mel, cq_vec, rho_vec, u_q_vec, _ignore);
        else
          proc_mel_encode2(&mel, cq_vec, rho_vec, u_q_vec, _ignore);

        proc_ms_encode(&ms, tuple_vec, uq_vec, rho_vec, s_vec);

        ui32 u_q[VL + 2];
        ui32 tuple[VL + 2];
        tuple_vec = hn::ShiftRight<4>(tuple_vec);
        hn::StoreU(tuple_vec, du, tuple);
        hn::StoreU(u_q_vec, du, u_q);
        {
          ui32 i_max = VL - (_ignore / 2);
          if (i_max & 1) { tuple[i_max] = 0; u_q[i_max] = 0; }
          tuple[VL] = 0; u_q[VL] = 0;
        }
        proc_vlc_encode(&vlc, tuple, u_q, _ignore,
            (PASS == 1) ? enc_hwy_uvlc_tbl_pair1 : enc_hwy_uvlc_tbl_pair2);
      }
    }

    void hwy_encode_codeblock(ui32* buf, ui32 missing_msbs,
                              ui32 num_passes, ui32 _width, ui32 height,
                              ui32 stride, ui32* lengths,
                              ojph::mem_elastic_allocator *elastic,
                              ojph::coded_lists *& coded)
    {
      ojph_unused(num_passes);                      //currently not used

      ui32 width = (_width + 2 * VL - 1) & ~(2 * VL - 1);
      ui32 ignore = width - _width;
      const int ms_size = (16384 * 16 + 14) / 15; //more than enough
      const int mel_vlc_size = 3072;              //more than enough
      const int mel_size = 192;
      const int vlc_size = mel_vlc_size - mel_size;

      ui8 ms_buf[ms_size];
      ui8 mel_vlc_buf[mel_vlc_size];
      ui8 *mel_buf = mel_vlc_buf;
      ui8 *vlc_buf = mel_vlc_buf + mel_size;

      mel_struct mel;
      mel_init(&mel, mel_size, mel_buf);
      vlc_struct vlc;
      vlc_init(&vlc, vlc_size, vlc_buf);
      ms_struct ms;
      ms_init(&ms, ms_size, ms_buf);

      const ui32 p = 30 - missing_msbs;

      //e_val: E values for a line (these are the highest set bit)
      //cx_val: is the context values
      //Each byte stores the info for the 2 sample. For E, it is maximum
      // of the two samples, while for cx, it is the OR of these two samples.
      //The maximum is between the pixel at the bottom left of one quad
      // and the bottom right of the earlier quad. The same is true for cx.
      //For a 1024 pixels, we need 512 bytes, the 2 extra,
      // one for the non-existing earlier quad, and one for beyond the
      // the end
      ui32 n_loop = width / (2 * VL);

      vec_u32 e_val_vec[max_n_loop + 1];
      for (ui32 i = 0; i < ojph_min(max_n_loop, n_loop); ++i)
        e_val_vec[i] = v_zero();

      vec_u32 prev_e_val_vec = v_zero();

      vec_u32 cx_val_vec[max_n_loop + 1];
      vec_u32 prev_cx_val_vec = v_zero();

      ui32 prev_cq = 0;

      vec_u32 tmp;

      /* 2 lines per iteration */
      for (ui32 y = 0; y < height; y += 2)
      {
        e_val_vec[n_loop] = prev_e_val_vec;
        /* lcxp[0] = (ui8)((rho[0] & 8) >> 3); */
        tmp = hn::And(prev_cx_val_vec, hn::Set(du, 8));
        cx_val_vec[n_loop] = hn::ShiftRight<3>(tmp);

        prev_e_val_vec = v_zero();
        prev_cx_val_vec = v_zero();

        ui32 *sp = buf + y * stride;

        if (y == 0)
          encode_x_loop<1>(sp, stride, height, y, n_loop, _width,
                           ignore, p, mel, vlc, ms,
                           e_val_vec, prev_e_val_vec,
                           cx_val_vec, prev_cx_val_vec, prev_cq);
        else
          encode_x_loop<2>(sp, stride, height, y, n_loop, _width,
                           ignore, p, mel, vlc, ms,
                           e_val_vec, prev_e_val_vec,
                           cx_val_vec, prev_cx_val_vec, prev_cq);

        /* prev_cq = lcxp[0] + (lcxp[1] << 2); */
        prev_cq = hn::ExtractLane(cx_val_vec[0], 0)
                + (hn::ExtractLane(cx_val_vec[0], 1) << 2);
      }

      ms_terminate(&ms);
      vlc_drain(&vlc);
      terminate_mel_vlc(&mel, &vlc);

      //copy to elastic
      lengths[0] = mel.pos + vlc.pos + ms.pos;
      elastic->get_buffer(mel.pos + vlc.pos + ms.pos, coded);
      memcpy(coded->buf, ms.buf, ms.pos);
      memcpy(coded->buf + ms.pos, mel.buf, mel.pos);
      memcpy(coded->buf + ms.pos + mel.pos, vlc.buf - vlc.pos + 1, vlc.pos);

      // put in the interface locator word
      ui32 num_bytes = mel.pos + vlc.pos;
      coded->buf[lengths[0]-1] = (ui8)(num_bytes >> 4);
      coded->buf[lengths[0]-2] = coded->buf[lengths[0]-2] & 0xF0;
      coded->buf[lengths[0]-2] =
          (ui8)(coded->buf[lengths[0]-2] | (num_bytes & 0xF));

      coded->avail_size -= lengths[0];
    }

#endif // HWY_TARGET == HWY_SCALAR

    } /* namespace HWY_NAMESPACE */
  } /* namespace local */
} /* namespace ojph */
HWY_AFTER_NAMESPACE();

//***************************************************************************/
// once-per-TU section: the shared tables, their initialization, and the
// dynamic-dispatch entry point
#if HWY_ONCE
namespace ojph {
  namespace local {

    /////////////////////////////////////////////////////////////////////////
    // tables
    /////////////////////////////////////////////////////////////////////////

    //VLC encoding
    // index is (c_q << 8) + (rho << 4) + eps
    // data is  (cwd << 8) + (cwd_len << 4) + eps
    // table 0 is for the initial line of quads
    ui32 enc_hwy_vlc_tbl0[2048];
    ui32 enc_hwy_vlc_tbl1[2048];

    //UVLC encoding
    ui32 enc_hwy_uvlc_tbl_pair1[33 * 33];
    ui32 enc_hwy_uvlc_tbl_pair2[33 * 33];
    static ui32 ulvc_cwd_pre[33];
    static int ulvc_cwd_pre_len[33];
    static ui32 ulvc_cwd_suf[33];
    static int ulvc_cwd_suf_len[33];

    /////////////////////////////////////////////////////////////////////////
    static bool vlc_init_tables()
    {
      struct vlc_src_table { int c_q, rho, u_off, e_k, e_1, cwd, cwd_len; };
      vlc_src_table tbl0[] = {
    #include "table0.h"
      };
      size_t tbl0_size = sizeof(tbl0) / sizeof(vlc_src_table);

      si32 pattern_popcnt[16];
      for (ui32 i = 0; i < 16; ++i)
        pattern_popcnt[i] = (si32)population_count(i);

      vlc_src_table* src_tbl = tbl0;
      ui32 *tgt_tbl = enc_hwy_vlc_tbl0;
      size_t tbl_size = tbl0_size;
      for (int i = 0; i < 2048; ++i)
      {
        int c_q = i >> 8, rho = (i >> 4) & 0xF, emb = i & 0xF;
        if (((emb & rho) != emb) || (rho == 0 && c_q == 0))
          tgt_tbl[i] = 0;
        else
        {
          vlc_src_table *best_entry = NULL;
          if (emb) // u_off = 1
          {
            int best_e_k = -1;
            for (size_t j = 0; j < tbl_size; ++j)
            {
              if (src_tbl[j].c_q == c_q && src_tbl[j].rho == rho)
                if (src_tbl[j].u_off == 1)
                  if ((emb & src_tbl[j].e_k) == src_tbl[j].e_1)
                  {
                    //now we need to find the smallest cwd with the highest
                    // number of bits set in e_k
                    int ones_count = pattern_popcnt[src_tbl[j].e_k];
                    if (ones_count >= best_e_k)
                    {
                      best_entry = src_tbl + j;
                      best_e_k = ones_count;
                    }
                  }
            }
          }
          else // u_off = 0
          {
            for (size_t j = 0; j < tbl_size; ++j)
            {
              if (src_tbl[j].c_q == c_q && src_tbl[j].rho == rho)
                if (src_tbl[j].u_off == 0)
                {
                  best_entry = src_tbl + j;
                  break;
                }
            }
          }
          assert(best_entry);
          tgt_tbl[i] = (ui16)((best_entry->cwd<<8) + (best_entry->cwd_len<<4)
                             + best_entry->e_k);
        }
      }

      vlc_src_table tbl1[] = {
    #include "table1.h"
      };
      size_t tbl1_size = sizeof(tbl1) / sizeof(vlc_src_table);

      src_tbl = tbl1;
      tgt_tbl = enc_hwy_vlc_tbl1;
      tbl_size = tbl1_size;
      for (int i = 0; i < 2048; ++i)
      {
        int c_q = i >> 8, rho = (i >> 4) & 0xF, emb = i & 0xF;
        if (((emb & rho) != emb) || (rho == 0 && c_q == 0))
          tgt_tbl[i] = 0;
        else
        {
          vlc_src_table *best_entry = NULL;
          if (emb) // u_off = 1
          {
            int best_e_k = -1;
            for (size_t j = 0; j < tbl_size; ++j)
            {
              if (src_tbl[j].c_q == c_q && src_tbl[j].rho == rho)
                if (src_tbl[j].u_off == 1)
                  if ((emb & src_tbl[j].e_k) == src_tbl[j].e_1)
                  {
                    //now we need to find the smallest cwd with the highest
                    // number of bits set in e_k
                    int ones_count = pattern_popcnt[src_tbl[j].e_k];
                    if (ones_count >= best_e_k)
                    {
                      best_entry = src_tbl + j;
                      best_e_k = ones_count;
                    }
                  }
            }
          }
          else // u_off = 0
          {
            for (size_t j = 0; j < tbl_size; ++j)
            {
              if (src_tbl[j].c_q == c_q && src_tbl[j].rho == rho)
                if (src_tbl[j].u_off == 0)
                {
                  best_entry = src_tbl + j;
                  break;
                }
            }
          }
          assert(best_entry);
          tgt_tbl[i] = (ui16)((best_entry->cwd<<8) + (best_entry->cwd_len<<4)
                             + best_entry->e_k);
        }
      }


      return true;
    }

    /////////////////////////////////////////////////////////////////////////
    static bool uvlc_init_tables()
    {
      //code goes from 0 to 31, extension and 32 are not supported here
      ulvc_cwd_pre[0] = 0; ulvc_cwd_pre[1] = 1; ulvc_cwd_pre[2] = 2;
      ulvc_cwd_pre[3] = 4; ulvc_cwd_pre[4] = 4;
      ulvc_cwd_pre_len[0] = 0; ulvc_cwd_pre_len[1] = 1;
      ulvc_cwd_pre_len[2] = 2;
      ulvc_cwd_pre_len[3] = 3; ulvc_cwd_pre_len[4] = 3;
      ulvc_cwd_suf[0] = 0; ulvc_cwd_suf[1] = 0; ulvc_cwd_suf[2] = 0;
      ulvc_cwd_suf[3] = 0; ulvc_cwd_suf[4] = 1;
      ulvc_cwd_suf_len[0] = 0; ulvc_cwd_suf_len[1] = 0;
      ulvc_cwd_suf_len[2] = 0;
      ulvc_cwd_suf_len[3] = 1; ulvc_cwd_suf_len[4] = 1;
      for (int i = 5; i < 33; ++i)
      {
        ulvc_cwd_pre[i] = 0;
        ulvc_cwd_pre_len[i] = 3;
        ulvc_cwd_suf[i] = (ui32)(i-5);
        ulvc_cwd_suf_len[i] = 5;
      }
      return true;
    }

    /////////////////////////////////////////////////////////////////////////
    static void uvlc_init_pair_tables()
    {
      for (int uq0 = 0; uq0 < 33; ++uq0) {
        for (int uq1 = 0; uq1 < 33; ++uq1) {
          ui32 cwd; int len;

          cwd = 0; len = 0;
          if (uq0 > 2 && uq1 > 2) {
            cwd |= ulvc_cwd_pre[uq0 - 2];
            len += ulvc_cwd_pre_len[uq0 - 2];
            cwd |= ulvc_cwd_pre[uq1 - 2] << len;
            len += ulvc_cwd_pre_len[uq1 - 2];
            cwd |= ulvc_cwd_suf[uq0 - 2] << len;
            len += ulvc_cwd_suf_len[uq0 - 2];
            cwd |= ulvc_cwd_suf[uq1 - 2] << len;
            len += ulvc_cwd_suf_len[uq1 - 2];
          } else if (uq0 > 2 && uq1 > 0) {
            cwd |= ulvc_cwd_pre[uq0];
            len += ulvc_cwd_pre_len[uq0];
            cwd |= (ui32)(uq1 - 1) << len;
            len += 1;
            cwd |= ulvc_cwd_suf[uq0] << len;
            len += ulvc_cwd_suf_len[uq0];
          } else {
            cwd |= ulvc_cwd_pre[uq0];
            len += ulvc_cwd_pre_len[uq0];
            cwd |= ulvc_cwd_pre[uq1] << len;
            len += ulvc_cwd_pre_len[uq1];
            cwd |= ulvc_cwd_suf[uq0] << len;
            len += ulvc_cwd_suf_len[uq0];
            cwd |= ulvc_cwd_suf[uq1] << len;
            len += ulvc_cwd_suf_len[uq1];
          }
          enc_hwy_uvlc_tbl_pair1[uq0 * 33 + uq1] = (cwd << 5) | (ui32)len;

          cwd = 0; len = 0;
          cwd |= ulvc_cwd_pre[uq0];
          len += ulvc_cwd_pre_len[uq0];
          cwd |= ulvc_cwd_pre[uq1] << len;
          len += ulvc_cwd_pre_len[uq1];
          cwd |= ulvc_cwd_suf[uq0] << len;
          len += ulvc_cwd_suf_len[uq0];
          cwd |= ulvc_cwd_suf[uq1] << len;
          len += ulvc_cwd_suf_len[uq1];
          enc_hwy_uvlc_tbl_pair2[uq0 * 33 + uq1] = (cwd << 5) | (ui32)len;
        }
      }
    }

    /////////////////////////////////////////////////////////////////////////
    bool initialize_block_encoder_tables_simd() {
      static bool tables_initialized = false;
      static std::once_flag tables_initialized_flag;
      std::call_once(tables_initialized_flag, []() {
        memset(enc_hwy_vlc_tbl0, 0, 2048 * sizeof(ui32));
        memset(enc_hwy_vlc_tbl1, 0, 2048 * sizeof(ui32));
        tables_initialized = vlc_init_tables();
        tables_initialized = tables_initialized && uvlc_init_tables();
        uvlc_init_pair_tables();
      });
      return tables_initialized;
    }

    /////////////////////////////////////////////////////////////////////////
    // True when at least one of the SIMD targets compiled into this file
    // is available at run time; ojph_codeblock_fun.cpp installs the
    // encoder only in that case.  Mirrors hwy_tx_kernels_available() in
    // ojph_codestream_hwy.cpp: hwy assumes its baseline target is
    // supported without checking, so when the baseline needs more than
    // the architecture guarantees (the MSVC static /arch:AVX2 build; GCC
    // and clang keep the baseline at portable EMU128), verify it with
    // our own CPU detection.
    bool hwy_encoder_available()
    {
#if defined(OJPH_ARCH_X86_64) || defined(OJPH_ARCH_I386)
  #if HWY_STATIC_TARGET <= HWY_AVX3
      if (get_cpu_ext_level() < X86_CPU_EXT_LEVEL_AVX512)
        return false;
  #elif HWY_STATIC_TARGET <= HWY_AVX2
      if (get_cpu_ext_level() < X86_CPU_EXT_LEVEL_AVX2FMA)
        return false;
  #elif HWY_STATIC_TARGET <= HWY_SSE4
      if (get_cpu_ext_level() < X86_CPU_EXT_LEVEL_SSE42)
        return false;
  #endif
#endif
      const int64_t sup = hwy::SupportedTargets();
      // SupportedTargets() re-initializes hwy's chosen dispatch target
      // with the full detected set, counting on its caller to narrow it
      // to the returned (possibly DisableTargets-masked) set; do so, or
      // a preceding hwy::DisableTargets() would be ignored
      hwy::GetChosenTarget().Update(sup);
      return (sup & HWY_TARGETS & ~(HWY_EMU128 | HWY_SCALAR)) != 0;
    }

    /////////////////////////////////////////////////////////////////////////
    // dynamic dispatch: one entry in the table per compiled target; the
    // first call selects the best target the CPU supports
    HWY_EXPORT(hwy_encode_codeblock);

    void ojph_encode_codeblock_simd(ui32* buf, ui32 missing_msbs,
                                   ui32 num_passes, ui32 _width, ui32 height,
                                   ui32 stride, ui32* lengths,
                                   ojph::mem_elastic_allocator *elastic,
                                   ojph::coded_lists *& coded)
    {
      HWY_DYNAMIC_DISPATCH(hwy_encode_codeblock)(buf, missing_msbs,
        num_passes, _width, height, stride, lengths, elastic, coded);
    }

  } /* namespace local */
} /* namespace ojph */
#endif // HWY_ONCE

#endif
