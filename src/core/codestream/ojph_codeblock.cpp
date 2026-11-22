
//***************************************************************************/
// This software is released under the 2-Clause BSD license, included
// below.
//
// Copyright (c) 2019, Aous Naman
// Copyright (c) 2019, Kakadu Software Pty Ltd, Australia
// Copyright (c) 2019, The University of New South Wales, Australia
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
// File: ojph_codeblock.cpp
// Author: Aous Naman
// Date: 28 August 2019
//***************************************************************************/


#include <climits>
#include <cstring>
#include "ojph_arch.h"
#if defined(OJPH_ARCH_X86_64) && !defined(OJPH_DISABLE_SIMD)
#include <emmintrin.h>
#endif
#include <cmath>

#include "ojph_mem.h"
#include "ojph_params.h"
#include "ojph_codestream_local.h"
#include "ojph_codeblock.h"
#include "ojph_subband.h"
#include "ojph_resolution.h"

namespace ojph {

  namespace local
  {

    //////////////////////////////////////////////////////////////////////////
    void codeblock::pre_alloc(codestream *codestream, const size& nominal,
                              ui32 precision)
    {
      mem_fixed_allocator* allocator = codestream->get_allocator();

      assert(byte_alignment / sizeof(ui32) > 1);
      const ui32 f = byte_alignment / sizeof(ui32) - 1;
      ui32 stride = (nominal.w + f) & ~f; // a multiple of 8

      if (precision <= 32)
        allocator->pre_alloc_data<ui32>(nominal.h * (size_t)stride, 0);
      else
        allocator->pre_alloc_data<ui64>(nominal.h * (size_t)stride, 0);
    }

    //////////////////////////////////////////////////////////////////////////
    void codeblock::finalize_alloc(codestream *codestream,
                                   subband *parent, const size& nominal,
                                   const size& cb_size,
                                   coded_cb_header* coded_cb,
                                   ui32 K_max, int line_offset,
                                   ui32 precision, ui32 comp_idx)
    {
      mem_fixed_allocator* allocator = codestream->get_allocator();

      const ui32 f = byte_alignment / sizeof(ui32) - 1;
      this->stride = (nominal.w + f) & ~f; // a multiple of 8
      this->buf_size = this->stride * nominal.h;

      if (precision <= 32) {
        this->precision = BUF32;
        this->buf32 = allocator->post_alloc_data<ui32>(this->buf_size, 0);
      }
      else {
        this->precision = BUF64;
        this->buf64 = allocator->post_alloc_data<ui64>(this->buf_size, 0);
      }

      this->nominal_size = nominal;
      this->cb_size = cb_size;
      this->parent = parent;
      this->line_offset = line_offset;
      this->cur_line = 0;
      this->zero_prefix_lines = 0;
      this->delta = parent->get_delta();
      this->delta_inv = 1.0f / this->delta;
      this->K_max = K_max;
      for (int i = 0; i < 4; ++i)
        this->max_val64[i] = 0;
      const param_cod* coc = codestream->get_coc(comp_idx);
      this->reversible = coc->is_reversible();
      this->resilient = codestream->is_resilient();
      this->stripe_causal = coc->get_block_vertical_causality();
      this->zero_block = false;
      this->coded_cb = coded_cb;

      this->codeblock_functions.init(reversible);
    }

    //////////////////////////////////////////////////////////////////////////
    // true when count words, starting at sp, are all zero; this is used on
    // both integer and float data -- for floats, a bitwise-zero test treats
    // -0.0f as nonzero, which is safely conservative
    template <typename T>
    static bool is_all_zero(const T *sp, ui32 count)
    {
      T acc = 0;
      for (ui32 i = 0; i < count; ++i)
        acc |= sp[i];
      return acc == 0;
    }

    //////////////////////////////////////////////////////////////////////////
    // transfer samples between a 16-bit line and a 32-bit codeblock,
    // converting to/from sign and magnitude; equivalent to
    // gen_rev_tx_to_cb32/gen_rev_tx_from_cb32 with a 16-bit line
    static void rev_tx_to_cb16(const si16 *sp, ui32 *dp, ui32 K_max,
                               ui32 count, ui32* max_val)
    {
      ui32 shift = 31 - K_max;
      ui32 tmax = *max_val;
      for (ui32 i = count; i > 0; --i)
      {
        si32 v = *sp++;
        ui32 sign = v >= 0 ? 0U : 0x80000000U;
        ui32 val = (ui32)(v >= 0 ? v : -v);
        val <<= shift;
        *dp++ = sign | val;
        tmax |= val; // it is more efficient to use or than max
      }
      *max_val = tmax;
    }

    //////////////////////////////////////////////////////////////////////////
    void codeblock::push(line_buf *line)
    {
      // Most codeblocks of mask-like or smooth images are entirely zero;
      // for those, the buffer is never written and codeblock::encode
      // skips them through max_val, so pushing costs only a read-only
      // scan.  Once a nonzero line is seen, the skipped all-zero prefix
      // is zeroed in the buffer, and later lines are written normally.
      //
      // convert to sign and magnitude and keep max_val
      if (precision == BUF32)
      {
        if (line->flags & line_buf::LFT_16BIT)
        { // 16-bit lines feed 32-bit codeblocks
          const si16 *sp = line->i16 + line_offset;
          if (zero_prefix_lines == cur_line &&
              is_all_zero(sp, cb_size.w))
          {
            ++zero_prefix_lines;
            ++cur_line;
            return;
          }
          if (zero_prefix_lines > 0 && zero_prefix_lines == cur_line)
            memset(buf32, 0,
                   (size_t)zero_prefix_lines * stride * sizeof(ui32));
          ui32 *dp = buf32 + cur_line * stride;
          rev_tx_to_cb16(sp, dp, K_max, cb_size.w, max_val32);
          ++cur_line;
          return;
        }
        assert(line->flags & line_buf::LFT_32BIT);
        const void *sp = (line->flags & line_buf::LFT_INTEGER)
          ? (const void*)(line->i32 + line_offset)
          : (const void*)(line->f32 + line_offset);
        if (zero_prefix_lines == cur_line &&
            is_all_zero((const ui32*)sp, cb_size.w))
        {
          ++zero_prefix_lines;
          ++cur_line;
          return;
        }
        if (zero_prefix_lines > 0 && zero_prefix_lines == cur_line)
          memset(buf32, 0, (size_t)zero_prefix_lines * stride * sizeof(ui32));
        ui32 *dp = buf32 + cur_line * stride;
        this->codeblock_functions.tx_to_cb32(sp, dp, K_max, delta_inv,
                                             cb_size.w, max_val32);
        ++cur_line;
      }
      else
      {
        assert(precision == BUF64);
        assert(line->flags & line_buf::LFT_64BIT);
        const si64 *sp = line->i64 + line_offset;
        if (zero_prefix_lines == cur_line &&
            is_all_zero((const ui64*)sp, cb_size.w))
        {
          ++zero_prefix_lines;
          ++cur_line;
          return;
        }
        if (zero_prefix_lines > 0 && zero_prefix_lines == cur_line)
          memset(buf64, 0, (size_t)zero_prefix_lines * stride * sizeof(ui64));
        ui64 *dp = buf64 + cur_line * stride;
        this->codeblock_functions.tx_to_cb64(sp, dp, K_max, delta_inv,
                                             cb_size.w, max_val64);
        ++cur_line;
      }
    }

    //////////////////////////////////////////////////////////////////////////
    void codeblock::encode(mem_elastic_allocator *elastic)
    {
      if (precision == BUF32)
      {
        ui32 mv = this->codeblock_functions.find_max_val32(max_val32);
        if (mv >= 1u << (31 - K_max))
        {
          coded_cb->missing_msbs = K_max - 1;
          assert(coded_cb->missing_msbs > 0);
          assert(coded_cb->missing_msbs < K_max);
          coded_cb->num_passes = 1;

          this->codeblock_functions.encode_cb32(buf32, K_max-1, 1,
            cb_size.w, cb_size.h, stride, coded_cb->pass_length,
            elastic, coded_cb->next_coded);
        }
      }
      else
      {
        assert(precision == BUF64);
        ui64 mv = this->codeblock_functions.find_max_val64(max_val64);
        if (mv >= 1ULL << (63 - K_max))
        {
          coded_cb->missing_msbs = K_max - 1;
          assert(coded_cb->missing_msbs > 0);
          assert(coded_cb->missing_msbs < K_max);
          coded_cb->num_passes = 1;

          this->codeblock_functions.encode_cb64(buf64, K_max-1, 1,
            cb_size.w, cb_size.h, stride, coded_cb->pass_length,
            elastic, coded_cb->next_coded);
        }
      }
    }

    //////////////////////////////////////////////////////////////////////////
    void codeblock::recreate(const size &cb_size, coded_cb_header* coded_cb)
    {
      assert(cb_size.h * stride <= buf_size && cb_size.w <= stride);
      this->cb_size = cb_size;
      this->coded_cb = coded_cb;
      this->cur_line = 0;
      this->zero_prefix_lines = 0;
      for (int i = 0; i < 4; ++i)
        this->max_val64[i] = 0;
      this->zero_block = false;
    }

    //////////////////////////////////////////////////////////////////////////
    void codeblock::decode()
    {
      if (coded_cb->pass_length[0] > 0 && coded_cb->num_passes > 0 &&
          coded_cb->next_coded != NULL)
      {
        bool result;
        if (precision == BUF32)
        {
          result = this->codeblock_functions.decode_cb32(
            coded_cb->next_coded->buf + coded_cb_header::prefix_buf_size,
            buf32, coded_cb->missing_msbs, coded_cb->num_passes,
            coded_cb->pass_length[0], coded_cb->pass_length[1],
            cb_size.w, cb_size.h, stride, stripe_causal);
        }
        else
        {
          assert(precision == BUF64);
          // The decoders place a sample's bits relative to the cleanup
          // pass's least significant bitplane, p = 62 - missing_msbs here
          // and p = 30 - missing_msbs for 32-bit codeblocks, and fail when
          // a sample needs more bits than missing_msbs + 2.  So a 64-bit
          // codeblock decodes as a 32-bit one with min(missing_msbs, 28)
          // missing MSBs (28 is the most that still gives the SPP and MRP
          // passes their bitplanes), and its magnitudes shifted up by the
          // difference in p, as long as no sample needs more than 30 bits;
          // the 32-bit decoders are much faster.  The 32-bit decoder
          // writes into the second half of buf64, which is then widened
          // into buf64 from the start; a widened group of samples never
          // overwrites samples that are yet to be read.  When the 32-bit
          // decoder fails, the 64-bit decoder decides.  Since a failed
          // attempt costs nearly as much as a successful one, the next 32
          // 64-bit codeblocks after a failure skip it; images that need
          // more than 30 bits tend to need them in most codeblocks.
          static thread_local ui32 skip_32bit_attempts = 0;
          result = false;
          ui32 m = coded_cb->missing_msbs;
          if (skip_32bit_attempts > 0)
            --skip_32bit_attempts;
          else if (m <= 32 + 28)
          {
            ui32 m32 = m < 28 ? m : 28;
            ui32 shift = 32 + m32 - m;
            ui32 *half = (ui32*)(buf64 + buf_size / 2);
            result = this->codeblock_functions.decode_cb32(
              coded_cb->next_coded->buf + coded_cb_header::prefix_buf_size,
              half, m32, coded_cb->num_passes,
              coded_cb->pass_length[0], coded_cb->pass_length[1],
              cb_size.w, cb_size.h, stride, stripe_causal);
            if (!result)
              skip_32bit_attempts = 32;
            else
              for (ui32 y = 0; y < cb_size.h; ++y)
              {
                const ui32 *sp = half + (size_t)y * stride;
                ui64 *dp = buf64 + (size_t)y * stride;
#if defined(OJPH_ARCH_X86_64) && !defined(OJPH_DISABLE_SIMD)
                const __m128i mag_mask = _mm_set1_epi32(0x7FFFFFFF);
                const __m128i sign_mask = _mm_set1_epi32((int)0x80000000u);
                const __m128i zero = _mm_setzero_si128();
                const __m128i count = _mm_cvtsi32_si128((int)shift);
                for (ui32 x = 0; x < cb_size.w; x += 4)
                {
                  __m128i v = _mm_loadu_si128((const __m128i*)(sp + x));
                  __m128i mag = _mm_and_si128(v, mag_mask);
                  __m128i sign = _mm_and_si128(v, sign_mask);
                  __m128i lo = _mm_sll_epi64(_mm_unpacklo_epi32(mag, zero),
                                             count);
                  __m128i hi = _mm_sll_epi64(_mm_unpackhi_epi32(mag, zero),
                                             count);
                  lo = _mm_or_si128(lo, _mm_unpacklo_epi32(zero, sign));
                  hi = _mm_or_si128(hi, _mm_unpackhi_epi32(zero, sign));
                  _mm_storeu_si128((__m128i*)(dp + x), lo);
                  _mm_storeu_si128((__m128i*)(dp + x + 2), hi);
                }
#else
                for (ui32 x = 0; x < cb_size.w; x += 4)
                {
                  ui32 v[4];
                  ui64 w[4];
                  memcpy(v, sp + x, sizeof(v));
                  for (int k = 0; k < 4; ++k)
                    w[k] = ((ui64)(v[k] & 0x7FFFFFFFu) << shift)
                         | ((ui64)(v[k] & 0x80000000u) << 32);
                  memcpy(dp + x, w, sizeof(w));
                }
#endif
              }
          }
          if (result == false)
            result = this->codeblock_functions.decode_cb64(
              coded_cb->next_coded->buf + coded_cb_header::prefix_buf_size,
              buf64, coded_cb->missing_msbs, coded_cb->num_passes,
              coded_cb->pass_length[0], coded_cb->pass_length[1],
              cb_size.w, cb_size.h, stride, stripe_causal);
        }

        if (result == false)
        {
          if (resilient == true) {
            OJPH_INFO(0x000300A1, "Error decoding a codeblock.");
            zero_block = true;
          }
          else
            OJPH_ERROR(0x000300A1, "Error decoding a codeblock.");
        }
      }
      else
        zero_block = true;
    }


    //////////////////////////////////////////////////////////////////////////
    void codeblock::pull_line(line_buf *line)
    {
      //convert to sign and magnitude
      if (precision == BUF32)
      {
        if (line->flags & line_buf::LFT_16BIT)
        { // 16-bit lines are fed from 32-bit codeblocks
          si16 *dp = line->i16 + line_offset;
          if (!zero_block)
          {
            const ui32 *sp = buf32 + cur_line * stride;
            this->codeblock_functions.tx_from_cb16(sp, dp, K_max, cb_size.w);
          }
          else
            memset(dp, 0, (size_t)cb_size.w * sizeof(si16));
          ++cur_line;
          return;
        }
        assert(line->flags & line_buf::LFT_32BIT);
        void *dp = (line->flags & line_buf::LFT_INTEGER)
          ? (void*)(line->i32 + line_offset)
          : (void*)(line->f32 + line_offset);
        if (!zero_block)
        {
          const ui32 *sp = buf32 + cur_line * stride;
          this->codeblock_functions.tx_from_cb32(sp, dp, K_max, delta,
                                                 cb_size.w);
        }
        else
          this->codeblock_functions.mem_clear(dp, cb_size.w * sizeof(ui32));
      }
      else
      {
        assert(precision == BUF64);
        assert((reversible && (line->flags & line_buf::LFT_64BIT))
               || (!reversible && (line->flags & line_buf::LFT_32BIT)));
        si64 *dp = line->i64 + line_offset;
        if (!zero_block)
        {
          const ui64 *sp = buf64 + cur_line * stride;
          this->codeblock_functions.tx_from_cb64(sp, dp, K_max, delta,
                                                 cb_size.w);
        }
        else
          this->codeblock_functions.mem_clear(dp, cb_size.w * sizeof(*dp));
      }

      ++cur_line;
      assert(cur_line <= cb_size.h);
    }

  }
}