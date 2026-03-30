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
// File: ojph_block_encoder.cpp
// Author: Aous Naman
// Date: 17 September 2019
//***************************************************************************/

//***************************************************************************/
/** @file ojph_block_encoder.cpp
 *  @brief implements HTJ2K block encoder
 */

#include <cassert>
#include <cstring>
#include <cstdint>
#include <climits>
#include <mutex>

#include "ojph_mem.h"
#include "ojph_arch.h"
#include "ojph_block_encoder.h"
#include "ojph_message.h"

#ifdef OJPH_COMPILER_MSVC
#include <malloc.h>
#define OJPH_MS_STACK_ALLOC(sz) ((ui8*)_alloca(sz))
#else
#include <alloca.h>
#define OJPH_MS_STACK_ALLOC(sz) ((ui8*)alloca(sz))
#endif

namespace ojph {
  namespace local {

    static ui32 ht_ms32_scratch_bytes(ui32 width, ui32 height)
    {
      ui64 n = (ui64)width * height;
      assert(n > 0 && n <= 4096u);
      return (ui32)((n * 64ull + 14ull) / 15ull);
    }

    static ui32 ht_ms64_scratch_bytes(ui32 width, ui32 height)
    {
      ui64 n = (ui64)width * height;
      assert(n > 0 && n <= 4096u);
      return (ui32)((n * 24030ull + 4095ull) / 4096ull);
    }

    static const ui32 k_ht_mel_scratch_bytes = 192u;

    static ui32 ht_vlc_payload_scratch_bytes(ui32 width, ui32 height)
    {
      ui64 qcols = ((ui64)width + 1u) / 2u;
      ui64 qrows = ((ui64)height + 1u) / 2u;
      ui64 num_quads = qcols * qrows;
      assert(num_quads > 0 && num_quads <= 1280u);
      ui64 raw_bits = num_quads * 19ull;
      ui64 raw_bytes = (raw_bits + 7ull) / 8ull;
      return (ui32)((raw_bytes * 16ull + 14ull) / 15ull) + 64u;
    }

    static ui32 ht_mel_vlc_scratch_bytes(ui32 width, ui32 height)
    {
      return k_ht_mel_scratch_bytes + ht_vlc_payload_scratch_bytes(width, height);
    }

    static ui32 ht_ctx_row_slots(ui32 width)
    {
      ui32 n = (width + 1u) / 2u + 16u;
      if (n < 32u)
        n = 32u;
      if (n > 513u)
        n = 513u;
      return n;
    }

    /////////////////////////////////////////////////////////////////////////
    // tables
    /////////////////////////////////////////////////////////////////////////

    //VLC encoding
    // index is (c_q << 8) + (rho << 4) + eps
    // data is  (cwd << 8) + (cwd_len << 4) + eps
    // table 0 is for the initial line of quads
    static ui16 vlc_tbl0[2048] = { 0 };
    static ui16 vlc_tbl1[2048] = { 0 };
    static ui16 vlc_tbl0_alt[2048] = { 0 };
    static ui16 vlc_tbl1_alt[2048] = { 0 };

    struct vlc_src_row { int c_q, rho, u_off, e_k, e_1, cwd, cwd_len; };

    static void fill_vlc_alt_ui16(ui16 *primary, ui16 *alt,
                                const vlc_src_row *src_tbl, size_t tbl_size)
    {
      si32 pattern_popcnt[16];
      for (ui32 i = 0; i < 16; ++i)
        pattern_popcnt[i] = (si32)population_count(i);
      for (int i = 0; i < 2048; ++i)
      {
        if (primary[i] == 0)
        {
          alt[i] = 0;
          continue;
        }
        int c_q = i >> 8, rho = (i >> 4) & 0xF, emb = i & 0xF;
        ui16 p = primary[i];
        int p_cwd = (int)p >> 8;
        int p_len = ((int)p >> 4) & 7;
        int p_ek = (int)p & 15;
        const vlc_src_row *best2 = NULL;
        int br = -1, be = -1, bc = INT_MAX;
        if (emb)
        {
          for (size_t j = 0; j < tbl_size; ++j)
          {
            if (src_tbl[j].c_q != c_q || src_tbl[j].rho != rho)
              continue;
            if (src_tbl[j].u_off != 1)
              continue;
            if ((emb & src_tbl[j].e_k) != src_tbl[j].e_1)
              continue;
            if (src_tbl[j].cwd_len != p_len)
              continue;
            if (src_tbl[j].cwd == p_cwd && src_tbl[j].e_k == p_ek)
              continue;
            int cr = pattern_popcnt[src_tbl[j].e_k & rho];
            int ce = pattern_popcnt[src_tbl[j].e_k];
            int cc = src_tbl[j].cwd;
            bool rep = best2 == NULL;
            if (!rep && cr > br)
              rep = true;
            else if (!rep && cr == br && ce > be)
              rep = true;
            else if (!rep && cr == br && ce == be && cc < bc)
              rep = true;
            if (rep)
            {
              best2 = src_tbl + j;
              br = cr;
              be = ce;
              bc = cc;
            }
          }
        }
        else
        {
          for (size_t j = 0; j < tbl_size; ++j)
          {
            if (src_tbl[j].c_q != c_q || src_tbl[j].rho != rho)
              continue;
            if (src_tbl[j].u_off != 0)
              continue;
            if (src_tbl[j].cwd_len != p_len)
              continue;
            if (src_tbl[j].cwd == p_cwd && src_tbl[j].e_k == p_ek)
              continue;
            int cr = pattern_popcnt[src_tbl[j].e_k & rho];
            int ce = pattern_popcnt[src_tbl[j].e_k];
            int cc = src_tbl[j].cwd;
            bool rep = best2 == NULL;
            if (!rep && cr > br)
              rep = true;
            else if (!rep && cr == br && ce > be)
              rep = true;
            else if (!rep && cr == br && ce == be && cc < bc)
              rep = true;
            if (rep)
            {
              best2 = src_tbl + j;
              br = cr;
              be = ce;
              bc = cc;
            }
          }
        }
        if (best2)
          alt[i] = (ui16)((best2->cwd << 8) + (best2->cwd_len << 4) + best2->e_k);
        else
          alt[i] = p;
      }
    }

    static inline int ht_ms_msum_Uq(int e_k, int Uq, int rho)
    {
      int s = 0;
      for (int b = 0; b < 4; ++b)
      {
        if (rho & (1 << b))
          s += Uq - ((e_k >> b) & 1);
      }
      return s;
    }

    static inline ui16 ht_pick_vlc_tuple_u16(ui16 pri, ui16 alt, int Uq, int rho)
    {
      int len_pri = (pri >> 4) & 7;
      int len_alt = (alt >> 4) & 7;
      int sp = len_pri + ht_ms_msum_Uq(pri & 15, Uq, rho);
      int sa = len_alt + ht_ms_msum_Uq(alt & 15, Uq, rho);
      return sa < sp ? alt : pri;
    }

    //UVLC encoding
    const int num_uvlc_entries = 75;
    struct uvlc_tbl_struct {
      ui8 pre, pre_len, suf, suf_len, ext, ext_len;
    };
    static uvlc_tbl_struct uvlc_tbl[num_uvlc_entries];
    
    /////////////////////////////////////////////////////////////////////////
    static bool vlc_init_tables()
    {
      vlc_src_row tbl0[] = {
    #include "table0.h"
      };
      size_t tbl0_size = sizeof(tbl0) / sizeof(vlc_src_row);

      si32 pattern_popcnt[16];
      for (ui32 i = 0; i < 16; ++i)
        pattern_popcnt[i] = (si32)population_count(i);

      vlc_src_row* src_tbl = tbl0;
      ui16 *tgt_tbl = vlc_tbl0;
      size_t tbl_size = tbl0_size;
      for (int i = 0; i < 2048; ++i)
      {
        int c_q = i >> 8, rho = (i >> 4) & 0xF, emb = i & 0xF;
        if (((emb & rho) != emb) || (rho == 0 && c_q == 0))
          tgt_tbl[i] = 0;
        else
        {
          vlc_src_row *best_entry = NULL;
          if (emb)
          {
            int best_cwd_len = INT_MAX;
            int best_rho_pop = -1;
            int best_e_k_pop = -1;
            int best_cwd = INT_MAX;
            for (size_t j = 0; j < tbl_size; ++j)
            {
              if (src_tbl[j].c_q == c_q && src_tbl[j].rho == rho)
                if (src_tbl[j].u_off == 1)
                  if ((emb & src_tbl[j].e_k) == src_tbl[j].e_1)
                  {
                    int cand_len = src_tbl[j].cwd_len;
                    int cand_rho_pop =
                      pattern_popcnt[src_tbl[j].e_k & rho];
                    int cand_e_pop = pattern_popcnt[src_tbl[j].e_k];
                    int cand_cwd = src_tbl[j].cwd;
                    bool replace = best_entry == NULL;
                    if (!replace && cand_len < best_cwd_len)
                      replace = true;
                    else if (!replace && cand_len == best_cwd_len)
                    {
                      if (cand_rho_pop > best_rho_pop)
                        replace = true;
                      else if (cand_rho_pop == best_rho_pop)
                      {
                        if (cand_e_pop > best_e_k_pop)
                          replace = true;
                        else if (cand_e_pop == best_e_k_pop && cand_cwd < best_cwd)
                          replace = true;
                      }
                    }
                    if (replace)
                    {
                      best_entry = src_tbl + j;
                      best_cwd_len = cand_len;
                      best_rho_pop = cand_rho_pop;
                      best_e_k_pop = cand_e_pop;
                      best_cwd = cand_cwd;
                    }
                  }
            }
          }
          else
          {
            int best_cwd_len = INT_MAX;
            int best_rho_pop = -1;
            int best_e_k_pop = -1;
            int best_cwd = INT_MAX;
            for (size_t j = 0; j < tbl_size; ++j)
            {
              if (src_tbl[j].c_q == c_q && src_tbl[j].rho == rho)
                if (src_tbl[j].u_off == 0)
                {
                  int cand_len = src_tbl[j].cwd_len;
                  int cand_rho_pop =
                    pattern_popcnt[src_tbl[j].e_k & rho];
                  int cand_e_pop = pattern_popcnt[src_tbl[j].e_k];
                  int cand_cwd = src_tbl[j].cwd;
                  bool replace = best_entry == NULL;
                  if (!replace && cand_len < best_cwd_len)
                    replace = true;
                  else if (!replace && cand_len == best_cwd_len)
                  {
                    if (cand_rho_pop > best_rho_pop)
                      replace = true;
                    else if (cand_rho_pop == best_rho_pop)
                    {
                      if (cand_e_pop > best_e_k_pop)
                        replace = true;
                      else if (cand_e_pop == best_e_k_pop && cand_cwd < best_cwd)
                        replace = true;
                    }
                  }
                  if (replace)
                  {
                    best_entry = src_tbl + j;
                    best_cwd_len = cand_len;
                    best_rho_pop = cand_rho_pop;
                    best_e_k_pop = cand_e_pop;
                    best_cwd = cand_cwd;
                  }
                }
            }
          }
          assert(best_entry);
          tgt_tbl[i] = (ui16)((best_entry->cwd<<8) + (best_entry->cwd_len<<4)
                             + best_entry->e_k);
        }
      }

      fill_vlc_alt_ui16(vlc_tbl0, vlc_tbl0_alt, tbl0, tbl0_size);

      vlc_src_row tbl1[] = {
    #include "table1.h"
      };
      size_t tbl1_size = sizeof(tbl1) / sizeof(vlc_src_row);

      src_tbl = tbl1;
      tgt_tbl = vlc_tbl1;
      tbl_size = tbl1_size;
      for (int i = 0; i < 2048; ++i)
      {
        int c_q = i >> 8, rho = (i >> 4) & 0xF, emb = i & 0xF;
        if (((emb & rho) != emb) || (rho == 0 && c_q == 0))
          tgt_tbl[i] = 0;
        else
        {
          vlc_src_row *best_entry = NULL;
          if (emb)
          {
            int best_cwd_len = INT_MAX;
            int best_rho_pop = -1;
            int best_e_k_pop = -1;
            int best_cwd = INT_MAX;
            for (size_t j = 0; j < tbl_size; ++j)
            {
              if (src_tbl[j].c_q == c_q && src_tbl[j].rho == rho)
                if (src_tbl[j].u_off == 1)
                  if ((emb & src_tbl[j].e_k) == src_tbl[j].e_1)
                  {
                    int cand_len = src_tbl[j].cwd_len;
                    int cand_rho_pop =
                      pattern_popcnt[src_tbl[j].e_k & rho];
                    int cand_e_pop = pattern_popcnt[src_tbl[j].e_k];
                    int cand_cwd = src_tbl[j].cwd;
                    bool replace = best_entry == NULL;
                    if (!replace && cand_len < best_cwd_len)
                      replace = true;
                    else if (!replace && cand_len == best_cwd_len)
                    {
                      if (cand_rho_pop > best_rho_pop)
                        replace = true;
                      else if (cand_rho_pop == best_rho_pop)
                      {
                        if (cand_e_pop > best_e_k_pop)
                          replace = true;
                        else if (cand_e_pop == best_e_k_pop && cand_cwd < best_cwd)
                          replace = true;
                      }
                    }
                    if (replace)
                    {
                      best_entry = src_tbl + j;
                      best_cwd_len = cand_len;
                      best_rho_pop = cand_rho_pop;
                      best_e_k_pop = cand_e_pop;
                      best_cwd = cand_cwd;
                    }
                  }
            }
          }
          else
          {
            int best_cwd_len = INT_MAX;
            int best_rho_pop = -1;
            int best_e_k_pop = -1;
            int best_cwd = INT_MAX;
            for (size_t j = 0; j < tbl_size; ++j)
            {
              if (src_tbl[j].c_q == c_q && src_tbl[j].rho == rho)
                if (src_tbl[j].u_off == 0)
                {
                  int cand_len = src_tbl[j].cwd_len;
                  int cand_rho_pop =
                    pattern_popcnt[src_tbl[j].e_k & rho];
                  int cand_e_pop = pattern_popcnt[src_tbl[j].e_k];
                  int cand_cwd = src_tbl[j].cwd;
                  bool replace = best_entry == NULL;
                  if (!replace && cand_len < best_cwd_len)
                    replace = true;
                  else if (!replace && cand_len == best_cwd_len)
                  {
                    if (cand_rho_pop > best_rho_pop)
                      replace = true;
                    else if (cand_rho_pop == best_rho_pop)
                    {
                      if (cand_e_pop > best_e_k_pop)
                        replace = true;
                      else if (cand_e_pop == best_e_k_pop && cand_cwd < best_cwd)
                        replace = true;
                    }
                  }
                  if (replace)
                  {
                    best_entry = src_tbl + j;
                    best_cwd_len = cand_len;
                    best_rho_pop = cand_rho_pop;
                    best_e_k_pop = cand_e_pop;
                    best_cwd = cand_cwd;
                  }
                }
            }
          }
          assert(best_entry);
          tgt_tbl[i] = (ui16)((best_entry->cwd<<8) + (best_entry->cwd_len<<4)
                             + best_entry->e_k);
        }
      }

      fill_vlc_alt_ui16(vlc_tbl1, vlc_tbl1_alt, tbl1, tbl1_size);

      return true;
    }

    /////////////////////////////////////////////////////////////////////////
    static bool uvlc_init_tables()
    {
      //code goes from 0 to 31, extension and 32 are not supported here
      uvlc_tbl[0].pre = 0;
      uvlc_tbl[0].pre_len = 0;
      uvlc_tbl[0].suf = 0;
      uvlc_tbl[0].suf_len = 0;
      uvlc_tbl[0].ext = 0;
      uvlc_tbl[0].ext_len = 0;

      uvlc_tbl[1].pre = 1;
      uvlc_tbl[1].pre_len = 1;
      uvlc_tbl[1].suf = 0;
      uvlc_tbl[1].suf_len = 0;
      uvlc_tbl[1].ext = 0;
      uvlc_tbl[1].ext_len = 0;

      uvlc_tbl[2].pre = 2;
      uvlc_tbl[2].pre_len = 2;
      uvlc_tbl[2].suf = 0;
      uvlc_tbl[2].suf_len = 0;
      uvlc_tbl[2].ext = 0;
      uvlc_tbl[2].ext_len = 0;

      uvlc_tbl[3].pre = 4;
      uvlc_tbl[3].pre_len = 3;
      uvlc_tbl[3].suf = 0;
      uvlc_tbl[3].suf_len = 1;
      uvlc_tbl[3].ext = 0;
      uvlc_tbl[3].ext_len = 0;

      uvlc_tbl[4].pre = 4;
      uvlc_tbl[4].pre_len = 3;
      uvlc_tbl[4].suf = 1;
      uvlc_tbl[4].suf_len = 1;
      uvlc_tbl[4].ext = 0;
      uvlc_tbl[4].ext_len = 0;

      for (int i = 5; i < 33; ++i)
      {
        uvlc_tbl[i].pre = 0;
        uvlc_tbl[i].pre_len = 3;
        uvlc_tbl[i].suf = (ui8)(i - 5);
        uvlc_tbl[i].suf_len = 5;
        uvlc_tbl[i].ext = 0;
        uvlc_tbl[i].ext_len = 0;
      }

      for (int i = 33; i < num_uvlc_entries; ++i)
      {
        uvlc_tbl[i].pre = 0;
        uvlc_tbl[i].pre_len = 3;
        uvlc_tbl[i].suf = (ui8)(28 + (i - 33) % 4);
        uvlc_tbl[i].suf_len = 5;
        uvlc_tbl[i].ext = (ui8)((i - 33) / 4);
        uvlc_tbl[i].ext_len = 4;
      }

      return true;
    }

    /////////////////////////////////////////////////////////////////////////
    bool initialize_block_encoder_tables() {
      static bool tables_initialized = false;
      static std::once_flag tables_initialized_flag;
      std::call_once(tables_initialized_flag, []() {
        memset(vlc_tbl0, 0, 2048 * sizeof(ui16));
        memset(vlc_tbl1, 0, 2048 * sizeof(ui16));
        memset(vlc_tbl0_alt, 0, 2048 * sizeof(ui16));
        memset(vlc_tbl1_alt, 0, 2048 * sizeof(ui16));
        tables_initialized = vlc_init_tables();
        tables_initialized = tables_initialized && uvlc_init_tables();
      });
      return tables_initialized;
    }

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

    //////////////////////////////////////////////////////////////////////////
    static inline void
    mel_emit_bit(mel_struct* melp, int v)
    {
      assert(v == 0 || v == 1);
      melp->tmp = (melp->tmp << 1) + v;
      melp->remaining_bits--;
      if (melp->remaining_bits == 0)
      {
        if (melp->pos >= melp->buf_size)
          OJPH_ERROR(0x00020001, "mel encoder's buffer is full");

        melp->buf[melp->pos++] = (ui8)melp->tmp;
        melp->remaining_bits = (melp->tmp == 0xFF ? 7 : 8);
        melp->tmp = 0;
      }
    }

    //////////////////////////////////////////////////////////////////////////
    static inline void
    mel_encode(mel_struct* melp, bool bit)
    {
      //MEL exponent
      static const int mel_exp[13] = {0,0,0,1,1,1,2,2,2,3,3,4,5};

      if (bit == false)
      {
        ++melp->run;
        if (melp->run >= melp->threshold)
        {
          mel_emit_bit(melp, 1);
          melp->run = 0;
          melp->k = ojph_min(12, melp->k + 1);
          melp->threshold = 1 << mel_exp[melp->k];
        }
      }
      else
      {
        mel_emit_bit(melp, 0);
        int t = mel_exp[melp->k];
        while (t > 0)
          mel_emit_bit(melp, (melp->run >> --t) & 1);
        melp->run = 0;
        melp->k = ojph_max(0, melp->k - 1);
        melp->threshold = 1 << mel_exp[melp->k];
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
      int tmp;       //temporary storage of coded bits
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
    vlc_encode(vlc_struct* vlcp, int cwd, int cwd_len)
    {
      while (cwd_len > 0)
      {
        if (vlcp->pos >= vlcp->buf_size)
          OJPH_ERROR(0x00020002, "vlc encoder's buffer is full");

        int avail_bits = 8 - vlcp->last_greater_than_8F - vlcp->used_bits;
        int t = ojph_min(avail_bits, cwd_len);
        vlcp->tmp |= (cwd & ((1 << t) - 1)) << vlcp->used_bits;
        vlcp->used_bits += t;
        avail_bits -= t;
        cwd_len -= t;
        cwd >>= t;
        if (avail_bits == 0)
        {
          if (vlcp->last_greater_than_8F && vlcp->tmp != 0x7F)
          {
            vlcp->last_greater_than_8F = false;
            continue; //one empty bit remaining
          }
          *(vlcp->buf - vlcp->pos) = (ui8)(vlcp->tmp);
          vlcp->pos++;
          vlcp->last_greater_than_8F = vlcp->tmp > 0x8F;
          vlcp->tmp = 0;
          vlcp->used_bits = 0;
        }
      }
    }

    //////////////////////////////////////////////////////////////////////////
    //
    //////////////////////////////////////////////////////////////////////////
    static inline void
    terminate_mel_vlc(mel_struct* melp, vlc_struct* vlcp)
    {
      if (melp->run > 0)
        mel_emit_bit(melp, 1);

      melp->tmp = melp->tmp << melp->remaining_bits;
      int mel_mask = (0xFF << melp->remaining_bits) & 0xFF;
      int vlc_mask = 0xFF >> (8 - vlcp->used_bits);
      if ((mel_mask | vlc_mask) == 0)
        return;  //last mel byte cannot be 0xFF, since then
                 //melp->remaining_bits would be < 8
      if (melp->pos >= melp->buf_size)
        OJPH_ERROR(0x00020003, "mel encoder's buffer is full");
      int fuse = melp->tmp | vlcp->tmp;
      if ( ( ((fuse ^ melp->tmp) & mel_mask)
           | ((fuse ^ vlcp->tmp) & vlc_mask) ) == 0
          && (fuse != 0xFF) && vlcp->pos > 1)
      {
        melp->buf[melp->pos++] = (ui8)fuse;
      }
      else
      {
        if (vlcp->pos >= vlcp->buf_size)
          OJPH_ERROR(0x00020004, "vlc encoder's buffer is full");
        melp->buf[melp->pos++] = (ui8)melp->tmp; //melp->tmp cannot be 0xFF
        *(vlcp->buf - vlcp->pos) = (ui8)vlcp->tmp;
        vlcp->pos++;
      }
    }

    /////////////////////////////////////////////////////////////////////////
    //
    /////////////////////////////////////////////////////////////////////////
    struct ms_struct {
      //storage
      ui8* buf;      //pointer to data buffer
      ui32 pos;      //position of next writing within buf
      ui32 buf_size; //size of buffer, which we must not exceed

      int max_bits;  //maximum number of bits that can be store in tmp
      int used_bits; //number of occupied bits in tmp
      ui32 tmp;      //temporary storage of coded bits
    };

    //////////////////////////////////////////////////////////////////////////
    static inline void
    ms_init(ms_struct* msp, ui32 buffer_size, ui8* data)
    {
      msp->buf = data;
      msp->pos = 0;
      msp->buf_size = buffer_size;
      msp->max_bits = 8;
      msp->used_bits = 0;
      msp->tmp = 0;
    }

    //////////////////////////////////////////////////////////////////////////
    static inline void
    ms_encode(ms_struct* msp, ui32 cwd, int cwd_len)
    {
      while (cwd_len > 0)
      {
        if (msp->pos >= msp->buf_size)
          OJPH_ERROR(0x00020005, "magnitude sign encoder's buffer is full");
        int t = ojph_min(msp->max_bits - msp->used_bits, cwd_len);
        msp->tmp |= (cwd & ((1U << t) - 1)) << msp->used_bits;
        msp->used_bits += t;
        cwd >>= t;
        cwd_len -= t;
        if (msp->used_bits >= msp->max_bits)
        {
          msp->buf[msp->pos++] = (ui8)msp->tmp;
          msp->max_bits = (msp->tmp == 0xFF) ? 7 : 8;
          msp->tmp = 0;
          msp->used_bits = 0;
        }
      }
    }

    //////////////////////////////////////////////////////////////////////////
    static inline void
    ms_encode64(ms_struct* msp, ui64 cwd, int cwd_len)
    {
      while (cwd_len > 0)
      {
        if (msp->pos >= msp->buf_size)
          OJPH_ERROR(0x00020005, "magnitude sign encoder's buffer is full");
        int t = ojph_min(msp->max_bits - msp->used_bits, cwd_len);
        msp->tmp |= (ui32)((cwd & ((1ULL << t) - 1)) << msp->used_bits);
        msp->used_bits += t;
        cwd >>= t;
        cwd_len -= t;
        if (msp->used_bits >= msp->max_bits)
        {
          msp->buf[msp->pos++] = (ui8)msp->tmp;
          msp->max_bits = (msp->tmp == 0xFF) ? 7 : 8;
          msp->tmp = 0;
          msp->used_bits = 0;
        }
      }
    }    

    //////////////////////////////////////////////////////////////////////////
    static inline void
    ms_terminate(ms_struct* msp)
    {
      if (msp->used_bits)
      {
        int t = msp->max_bits - msp->used_bits; //unused bits
        msp->tmp |= (0xFF & ((1U << t) - 1)) << msp->used_bits;
        msp->used_bits += t;
        if (msp->tmp != 0xFF)
        {
          if (msp->pos >= msp->buf_size)
            OJPH_ERROR(0x00020006, "magnitude sign encoder's buffer is full");
          msp->buf[msp->pos++] = (ui8)msp->tmp;
        }
      }
      else if (msp->max_bits == 7)
        msp->pos--;
    }

    //////////////////////////////////////////////////////////////////////////
    //
    //
    //
    //
    //
    //////////////////////////////////////////////////////////////////////////
    void ojph_encode_codeblock32(ui32* buf, ui32 missing_msbs, ui32 num_passes,
                                 ui32 width, ui32 height, ui32 stride,
                                 ui32* lengths,
                                 ojph::mem_elastic_allocator *elastic,
                                 ojph::coded_lists *& coded)
    {
      assert(num_passes == 1);
      (void)num_passes;                      //currently not used
      const ui32 ctx_row_slots = ht_ctx_row_slots(width);
      const ui32 ms_size = ht_ms32_scratch_bytes(width, height);
      const ui32 mel_vlc_size = ht_mel_vlc_scratch_bytes(width, height);
      const ui32 mel_size = k_ht_mel_scratch_bytes;
      const ui32 vlc_size = mel_vlc_size - mel_size;
      const ui32 ht_slab_bytes = ms_size + mel_vlc_size + ctx_row_slots * 2u;
      ui8* const ht_slab = OJPH_MS_STACK_ALLOC(ht_slab_bytes);
      ui8* const ms_buf = ht_slab;
      ui8* const mel_vlc_buf = ht_slab + ms_size;
      ui8* const mel_buf = mel_vlc_buf;
      ui8* const vlc_buf = mel_vlc_buf + mel_size;
      ui8* const e_val = mel_vlc_buf + mel_vlc_size;
      ui8* const cx_val = e_val + ctx_row_slots;
      memset(e_val, 0, ctx_row_slots * 2u);

      mel_struct mel;
      mel_init(&mel, mel_size, mel_buf);
      vlc_struct vlc;
      vlc_init(&vlc, vlc_size, vlc_buf);
      ms_struct ms;
      ms_init(&ms, ms_size, ms_buf);

      ui32 p = 30 - missing_msbs;

      ui8* lep = e_val;
      ui8* lcxp = cx_val;

      //initial row of quads
      int e_qmax[2] = {0,0}, e_q[8] = {0,0,0,0,0,0,0,0};
      int rho[2] = {0,0};
      int c_q0 = 0;
      ui32 s[8] = {0,0,0,0,0,0,0,0}, val, t;
      ui32 y = 0;
      ui32 *sp = buf;
      for (ui32 x = 0; x < width; x += 4)
      {
        //prepare two quads
        t = sp[0];
        val = t + t; //multiply by 2 and get rid of sign
        val >>= p;  // 2 \mu_p + x
        val &= ~1u; // 2 \mu_p
        if (val)
        {
          rho[0] = 1;
          e_q[0] = 32 - (int)count_leading_zeros(--val); //2\mu_p - 1
          e_qmax[0] = e_q[0];
          s[0] = --val + (t >> 31); //v_n = 2(\mu_p-1) + s_n
        }

        t = height > 1 ? sp[stride] : 0;
        ++sp;
        val = t + t; //multiply by 2 and get rid of sign
        val >>= p; // 2 \mu_p + x
        val &= ~1u;// 2 \mu_p
        if (val)
        {
          rho[0] += 2;
          e_q[1] = 32 - (int)count_leading_zeros(--val); //2\mu_p - 1
          e_qmax[0] = ojph_max(e_qmax[0], e_q[1]);
          s[1] = --val + (t >> 31); //v_n = 2(\mu_p-1) + s_n
        }

        if (x+1 < width)
        {
          t = sp[0];
          val = t + t; //multiply by 2 and get rid of sign
          val >>= p; // 2 \mu_p + x
          val &= ~1u;// 2 \mu_p
          if (val)
          {
            rho[0] += 4;
            e_q[2] = 32 - (int)count_leading_zeros(--val); //2\mu_p - 1
            e_qmax[0] = ojph_max(e_qmax[0], e_q[2]);
            s[2] = --val + (t >> 31); //v_n = 2(\mu_p-1) + s_n
          }

          t = height > 1 ? sp[stride] : 0;
          ++sp;
          val = t + t; //multiply by 2 and get rid of sign
          val >>= p; // 2 \mu_p + x
          val &= ~1u;// 2 \mu_p
          if (val)
          {
            rho[0] += 8;
            e_q[3] = 32 - (int)count_leading_zeros(--val); //2\mu_p - 1
            e_qmax[0] = ojph_max(e_qmax[0], e_q[3]);
            s[3] = --val + (t >> 31); //v_n = 2(\mu_p-1) + s_n
          }
        }

        int Uq0 = ojph_max(e_qmax[0], 1); //kappa_q = 1
        int u_q0 = Uq0 - 1, u_q1 = 0; //kappa_q = 1

        int eps0 = 0;
        if (u_q0 > 0)
        {
          eps0 |= (e_q[0] == e_qmax[0]);
          eps0 |= (e_q[1] == e_qmax[0]) << 1;
          eps0 |= (e_q[2] == e_qmax[0]) << 2;
          eps0 |= (e_q[3] == e_qmax[0]) << 3;
        }
        lep[0] = ojph_max(lep[0], (ui8)e_q[1]); lep++;
        lep[0] = (ui8)e_q[3];
        lcxp[0] = (ui8)(lcxp[0] | (ui8)((rho[0] & 2) >> 1)); lcxp++;
        lcxp[0] = (ui8)((rho[0] & 8) >> 3);

        int ti0 = (c_q0 << 8) + (rho[0] << 4) + eps0;
        ui16 tuple0 = ht_pick_vlc_tuple_u16(vlc_tbl0[ti0], vlc_tbl0_alt[ti0],
                                            Uq0, rho[0]);
        vlc_encode(&vlc, tuple0 >> 8, (tuple0 >> 4) & 7);

        if (c_q0 == 0)
            mel_encode(&mel, rho[0] != 0);

        int m = (rho[0] & 1) ? Uq0 - (tuple0 & 1) : 0;
        ms_encode(&ms, s[0] & ((1U<<m)-1), m);
        m = (rho[0] & 2) ? Uq0 - ((tuple0 & 2) >> 1) : 0;
        ms_encode(&ms, s[1] & ((1U<<m)-1), m);
        m = (rho[0] & 4) ? Uq0 - ((tuple0 & 4) >> 2) : 0;
        ms_encode(&ms, s[2] & ((1U<<m)-1), m);
        m = (rho[0] & 8) ? Uq0 - ((tuple0 & 8) >> 3) : 0;
        ms_encode(&ms, s[3] & ((1U<<m)-1), m);

        if (x+2 < width)
        {
          t = sp[0];
          val = t + t; //multiply by 2 and get rid of sign
          val >>= p; // 2 \mu_p + x
          val &= ~1u;// 2 \mu_p
          if (val)
          {
            rho[1] = 1;
            e_q[4] = 32 - (int)count_leading_zeros(--val); //2\mu_p - 1
            e_qmax[1] = e_q[4];
            s[4] = --val + (t >> 31); //v_n = 2(\mu_p-1) + s_n
          }

          t = height > 1 ? sp[stride] : 0;
          ++sp;
          val = t + t; //multiply by 2 and get rid of sign
          val >>= p; // 2 \mu_p + x
          val &= ~1u;// 2 \mu_p
          if (val)
          {
            rho[1] += 2;
            e_q[5] = 32 - (int)count_leading_zeros(--val); //2\mu_p - 1
            e_qmax[1] = ojph_max(e_qmax[1], e_q[5]);
            s[5] = --val + (t >> 31); //v_n = 2(\mu_p-1) + s_n
          }

          if (x+3 < width)
          {
            t = sp[0];
            val = t + t; //multiply by 2 and get rid of sign
            val >>= p; // 2 \mu_p + x
            val &= ~1u;// 2 \mu_p
            if (val)
            {
              rho[1] += 4;
              e_q[6] = 32 - (int)count_leading_zeros(--val); //2\mu_p - 1
              e_qmax[1] = ojph_max(e_qmax[1], e_q[6]);
              s[6] = --val + (t >> 31); //v_n = 2(\mu_p-1) + s_n
            }

            t = height > 1 ? sp[stride] : 0;
            ++sp;
            val = t + t; //multiply by 2 and get rid of sign
            val >>= p; // 2 \mu_p + x
            val &= ~1u;// 2 \mu_p
            if (val)
            {
              rho[1] += 8;
              e_q[7] = 32 - (int)count_leading_zeros(--val); //2\mu_p - 1
              e_qmax[1] = ojph_max(e_qmax[1], e_q[7]);
              s[7] = --val + (t >> 31); //v_n = 2(\mu_p-1) + s_n
            }
          }

          int c_q1 = (rho[0] >> 1) | (rho[0] & 1);
          int Uq1 = ojph_max(e_qmax[1], 1); //kappa_q = 1
          u_q1 = Uq1 - 1; //kappa_q = 1

          int eps1 = 0;
          if (u_q1 > 0)
          {
            eps1 |= (e_q[4] == e_qmax[1]);
            eps1 |= (e_q[5] == e_qmax[1]) << 1;
            eps1 |= (e_q[6] == e_qmax[1]) << 2;
            eps1 |= (e_q[7] == e_qmax[1]) << 3;
          }
          lep[0] = ojph_max(lep[0], (ui8)e_q[5]); lep++;
          lep[0] = (ui8)e_q[7];
          lcxp[0] |= (ui8)(lcxp[0] | (ui8)((rho[1] & 2) >> 1)); lcxp++;
          lcxp[0] = (ui8)((rho[1] & 8) >> 3);
          int ti1a = (c_q1 << 8) + (rho[1] << 4) + eps1;
          ui16 tuple1 = ht_pick_vlc_tuple_u16(vlc_tbl0[ti1a], vlc_tbl0_alt[ti1a],
                                              Uq1, rho[1]);
          vlc_encode(&vlc, tuple1 >> 8, (tuple1 >> 4) & 7);

          if (c_q1 == 0)
            mel_encode(&mel, rho[1] != 0);

          int m = (rho[1] & 1) ? Uq1 - (tuple1 & 1) : 0;
          ms_encode(&ms, s[4] & ((1U<<m)-1), m);
          m = (rho[1] & 2) ? Uq1 - ((tuple1 & 2) >> 1) : 0;
          ms_encode(&ms, s[5] & ((1U<<m)-1), m);
          m = (rho[1] & 4) ? Uq1 - ((tuple1 & 4) >> 2) : 0;
          ms_encode(&ms, s[6] & ((1U<<m)-1), m);
          m = (rho[1] & 8) ? Uq1 - ((tuple1 & 8) >> 3) : 0;
          ms_encode(&ms, s[7] & ((1U<<m)-1), m);
        }

        if (u_q0 > 0 && u_q1 > 0)
          mel_encode(&mel, ojph_min(u_q0, u_q1) > 2);

        if (u_q0 > 2 && u_q1 > 2)
        {
          vlc_encode(&vlc, uvlc_tbl[u_q0-2].pre, uvlc_tbl[u_q0-2].pre_len);
          vlc_encode(&vlc, uvlc_tbl[u_q1-2].pre, uvlc_tbl[u_q1-2].pre_len);
          vlc_encode(&vlc, uvlc_tbl[u_q0-2].suf, uvlc_tbl[u_q0-2].suf_len);
          vlc_encode(&vlc, uvlc_tbl[u_q1-2].suf, uvlc_tbl[u_q1-2].suf_len);
        }
        else if (u_q0 > 2 && u_q1 > 0)
        {
          vlc_encode(&vlc, uvlc_tbl[u_q0].pre, uvlc_tbl[u_q0].pre_len);
          vlc_encode(&vlc, u_q1 - 1, 1);
          vlc_encode(&vlc, uvlc_tbl[u_q0].suf, uvlc_tbl[u_q0].suf_len);
        }
        else
        {
          vlc_encode(&vlc, uvlc_tbl[u_q0].pre, uvlc_tbl[u_q0].pre_len);
          vlc_encode(&vlc, uvlc_tbl[u_q1].pre, uvlc_tbl[u_q1].pre_len);
          vlc_encode(&vlc, uvlc_tbl[u_q0].suf, uvlc_tbl[u_q0].suf_len);
          vlc_encode(&vlc, uvlc_tbl[u_q1].suf, uvlc_tbl[u_q1].suf_len);
        }

        //prepare for next iteration
        c_q0 = (rho[1] >> 1) | (rho[1] & 1);
        s[0] = s[1] = s[2] = s[3] = s[4] = s[5] = s[6] = s[7] = 0;
        e_q[0]=e_q[1]=e_q[2]=e_q[3]=e_q[4]=e_q[5]=e_q[6]=e_q[7]=0;
        rho[0] = rho[1] = 0; e_qmax[0] = e_qmax[1] = 0;
      }

      lep[1] = 0;

      for (y = 2; y < height; y += 2)
      {
        lep = e_val;
        int max_e = ojph_max(lep[0], lep[1]) - 1;
        lep[0] = 0;
        lcxp = cx_val;
        c_q0 = lcxp[0] + (lcxp[1] << 2);
        lcxp[0] = 0;

        sp = buf + y * stride;
        for (ui32 x = 0; x < width; x += 4)
        {
          //prepare two quads
          t = sp[0];
          val = t + t; //multiply by 2 and get rid of sign
          val >>= p; // 2 \mu_p + x
          val &= ~1u;// 2 \mu_p
          if (val)
          {
            rho[0] = 1;
            e_q[0] = 32 - (int)count_leading_zeros(--val); //2\mu_p - 1
            e_qmax[0] = e_q[0];
            s[0] = --val + (t >> 31); //v_n = 2(\mu_p-1) + s_n
          }

          t = y + 1 < height ? sp[stride] : 0;
          ++sp;
          val = t + t; //multiply by 2 and get rid of sign
          val >>= p; // 2 \mu_p + x
          val &= ~1u;// 2 \mu_p
          if (val)
          {
            rho[0] += 2;
            e_q[1] = 32 - (int)count_leading_zeros(--val); //2\mu_p - 1
            e_qmax[0] = ojph_max(e_qmax[0], e_q[1]);
            s[1] = --val + (t >> 31); //v_n = 2(\mu_p-1) + s_n
          }

          if (x+1 < width)
          {
            t = sp[0];
            val = t + t; //multiply by 2 and get rid of sign
            val >>= p; // 2 \mu_p + x
            val &= ~1u;// 2 \mu_p
            if (val)
            {
              rho[0] += 4;
              e_q[2] = 32 - (int)count_leading_zeros(--val); //2\mu_p - 1
              e_qmax[0] = ojph_max(e_qmax[0], e_q[2]);
              s[2] = --val + (t >> 31); //v_n = 2(\mu_p-1) + s_n
            }

            t = y + 1 < height ? sp[stride] : 0;
            ++sp;
            val = t + t; //multiply by 2 and get rid of sign
            val >>= p; // 2 \mu_p + x
            val &= ~1u;// 2 \mu_p
            if (val)
            {
              rho[0] += 8;
              e_q[3] = 32 - (int)count_leading_zeros(--val); //2\mu_p - 1
              e_qmax[0] = ojph_max(e_qmax[0], e_q[3]);
              s[3] = --val + (t >> 31); //v_n = 2(\mu_p-1) + s_n
            }
          }

          int kappa = (rho[0] & (rho[0]-1)) ? ojph_max(1,max_e) : 1;
          int Uq0 = ojph_max(e_qmax[0], kappa);
          int u_q0 = Uq0 - kappa, u_q1 = 0;

          int eps0 = 0;
          if (u_q0 > 0)
          {
            eps0 |= (e_q[0] == e_qmax[0]);
            eps0 |= (e_q[1] == e_qmax[0]) << 1;
            eps0 |= (e_q[2] == e_qmax[0]) << 2;
            eps0 |= (e_q[3] == e_qmax[0]) << 3;
          }
          lep[0] = ojph_max(lep[0], (ui8)e_q[1]); lep++;
          max_e = ojph_max(lep[0], lep[1]) - 1;
          lep[0] = (ui8)e_q[3];
          lcxp[0] = (ui8)(lcxp[0] | (ui8)((rho[0] & 2) >> 1)); lcxp++;
          int c_q1 = lcxp[0] + (lcxp[1] << 2);
          lcxp[0] = (ui8)((rho[0] & 8) >> 3);
          int ti0b = (c_q0 << 8) + (rho[0] << 4) + eps0;
          ui16 tuple0 = ht_pick_vlc_tuple_u16(vlc_tbl1[ti0b], vlc_tbl1_alt[ti0b],
                                              Uq0, rho[0]);
          vlc_encode(&vlc, tuple0 >> 8, (tuple0 >> 4) & 7);

          if (c_q0 == 0)
              mel_encode(&mel, rho[0] != 0);

          int m = (rho[0] & 1) ? Uq0 - (tuple0 & 1) : 0;
          ms_encode(&ms, s[0] & ((1U<<m)-1), m);
          m = (rho[0] & 2) ? Uq0 - ((tuple0 & 2) >> 1) : 0;
          ms_encode(&ms, s[1] & ((1U<<m)-1), m);
          m = (rho[0] & 4) ? Uq0 - ((tuple0 & 4) >> 2) : 0;
          ms_encode(&ms, s[2] & ((1U<<m)-1), m);
          m = (rho[0] & 8) ? Uq0 - ((tuple0 & 8) >> 3) : 0;
          ms_encode(&ms, s[3] & ((1U<<m)-1), m);

          if (x+2 < width)
          {
            t = sp[0];
            val = t + t; //multiply by 2 and get rid of sign
            val >>= p; // 2 \mu_p + x
            val &= ~1u;// 2 \mu_p
            if (val)
            {
              rho[1] = 1;
              e_q[4] = 32 - (int)count_leading_zeros(--val); //2\mu_p - 1
              e_qmax[1] = e_q[4];
              s[4] = --val + (t >> 31); //v_n = 2(\mu_p-1) + s_n
            }

            t = y + 1 < height ? sp[stride] : 0;
            ++sp;
            val = t + t; //multiply by 2 and get rid of sign
            val >>= p; // 2 \mu_p + x
            val &= ~1u;// 2 \mu_p
            if (val)
            {
              rho[1] += 2;
              e_q[5] = 32 - (int)count_leading_zeros(--val); //2\mu_p - 1
              e_qmax[1] = ojph_max(e_qmax[1], e_q[5]);
              s[5] = --val + (t >> 31); //v_n = 2(\mu_p-1) + s_n
            }

            if (x+3 < width)
            {
              t = sp[0];
              val = t + t; //multiply by 2 and get rid of sign
              val >>= p; // 2 \mu_p + x
              val &= ~1u;// 2 \mu_p
              if (val)
              {
                rho[1] += 4;
                e_q[6] = 32 - (int)count_leading_zeros(--val); //2\mu_p - 1
                e_qmax[1] = ojph_max(e_qmax[1], e_q[6]);
                s[6] = --val + (t >> 31); //v_n = 2(\mu_p-1) + s_n
              }

              t = y + 1 < height ? sp[stride] : 0;
              ++sp;
              val = t + t; //multiply by 2 and get rid of sign
              val >>= p; // 2 \mu_p + x
              val &= ~1u;// 2 \mu_p
              if (val)
              {
                rho[1] += 8;
                e_q[7] = 32 - (int)count_leading_zeros(--val); //2\mu_p - 1
                e_qmax[1] = ojph_max(e_qmax[1], e_q[7]);
                s[7] = --val + (t >> 31); //v_n = 2(\mu_p-1) + s_n
              }
            }

            kappa = (rho[1] & (rho[1]-1)) ? ojph_max(1,max_e) : 1;
            c_q1 |= ((rho[0] & 4) >> 1) | ((rho[0] & 8) >> 2);
            int Uq1 = ojph_max(e_qmax[1], kappa);
            u_q1 = Uq1 - kappa;

            int eps1 = 0;
            if (u_q1 > 0)
            {
              eps1 |= (e_q[4] == e_qmax[1]);
              eps1 |= (e_q[5] == e_qmax[1]) << 1;
              eps1 |= (e_q[6] == e_qmax[1]) << 2;
              eps1 |= (e_q[7] == e_qmax[1]) << 3;
            }
            lep[0] = ojph_max(lep[0], (ui8)e_q[5]); lep++;
            max_e = ojph_max(lep[0], lep[1]) - 1;
            lep[0] = (ui8)e_q[7];
            lcxp[0] = (ui8)(lcxp[0] | (ui8)((rho[1] & 2) >> 1)); lcxp++;
            c_q0 = lcxp[0] + (lcxp[1] << 2);
            lcxp[0] = (ui8)((rho[1] & 8) >> 3);
            int ti1b = (c_q1 << 8) + (rho[1] << 4) + eps1;
            ui16 tuple1 = ht_pick_vlc_tuple_u16(vlc_tbl1[ti1b], vlc_tbl1_alt[ti1b],
                                                Uq1, rho[1]);
            vlc_encode(&vlc, tuple1 >> 8, (tuple1 >> 4) & 7);

            if (c_q1 == 0)
              mel_encode(&mel, rho[1] != 0);

            int m = (rho[1] & 1) ? Uq1 - (tuple1 & 1) : 0;
            ms_encode(&ms, s[4] & ((1U<<m)-1), m);
            m = (rho[1] & 2) ? Uq1 - ((tuple1 & 2) >> 1) : 0;
            ms_encode(&ms, s[5] & ((1U<<m)-1), m);
            m = (rho[1] & 4) ? Uq1 - ((tuple1 & 4) >> 2) : 0;
            ms_encode(&ms, s[6] & ((1U<<m)-1), m);
            m = (rho[1] & 8) ? Uq1 - ((tuple1 & 8) >> 3) : 0;
            ms_encode(&ms, s[7] & ((1U<<m)-1), m);
          }

          vlc_encode(&vlc, uvlc_tbl[u_q0].pre, uvlc_tbl[u_q0].pre_len);
          vlc_encode(&vlc, uvlc_tbl[u_q1].pre, uvlc_tbl[u_q1].pre_len);
          vlc_encode(&vlc, uvlc_tbl[u_q0].suf, uvlc_tbl[u_q0].suf_len);
          vlc_encode(&vlc, uvlc_tbl[u_q1].suf, uvlc_tbl[u_q1].suf_len);

          //prepare for next iteration
          c_q0 |= ((rho[1] & 4) >> 1) | ((rho[1] & 8) >> 2);
          s[0] = s[1] = s[2] = s[3] = s[4] = s[5] = s[6] = s[7] = 0;
          e_q[0]=e_q[1]=e_q[2]=e_q[3]=e_q[4]=e_q[5]=e_q[6]=e_q[7]=0;
          rho[0] = rho[1] = 0; e_qmax[0] = e_qmax[1] = 0;
        }
      }


      terminate_mel_vlc(&mel, &vlc);
      ms_terminate(&ms);

      //copy to elastic
      lengths[0] = mel.pos + vlc.pos + ms.pos;
      lengths[1] = 0;
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

    //////////////////////////////////////////////////////////////////////////
    //
    //
    //
    //
    //
    //////////////////////////////////////////////////////////////////////////
    void ojph_encode_codeblock64(ui64* buf, ui32 missing_msbs, ui32 num_passes,
                                 ui32 width, ui32 height, ui32 stride,
                                 ui32* lengths,
                                 ojph::mem_elastic_allocator *elastic,
                                 ojph::coded_lists *& coded)
    {
      assert(num_passes == 1);
      (void)num_passes;                      //currently not used
      const ui32 ctx_row_slots = ht_ctx_row_slots(width);
      const ui32 ms_size = ht_ms64_scratch_bytes(width, height);
      const ui32 mel_vlc_size = ht_mel_vlc_scratch_bytes(width, height);
      const ui32 mel_size = k_ht_mel_scratch_bytes;
      const ui32 vlc_size = mel_vlc_size - mel_size;
      const ui32 ht_slab_bytes = ms_size + mel_vlc_size + ctx_row_slots * 2u;
      ui8* const ht_slab = OJPH_MS_STACK_ALLOC(ht_slab_bytes);
      ui8* const ms_buf = ht_slab;
      ui8* const mel_vlc_buf = ht_slab + ms_size;
      ui8* const mel_buf = mel_vlc_buf;
      ui8* const vlc_buf = mel_vlc_buf + mel_size;
      ui8* const e_val = mel_vlc_buf + mel_vlc_size;
      ui8* const cx_val = e_val + ctx_row_slots;
      memset(e_val, 0, ctx_row_slots * 2u);

      mel_struct mel;
      mel_init(&mel, mel_size, mel_buf);
      vlc_struct vlc;
      vlc_init(&vlc, vlc_size, vlc_buf);
      ms_struct ms;
      ms_init(&ms, ms_size, ms_buf);

      ui32 p = 62 - missing_msbs;

      ui8* lep = e_val;
      ui8* lcxp = cx_val;

      //initial row of quads
      int e_qmax[2] = {0,0}, e_q[8] = {0,0,0,0,0,0,0,0};
      int rho[2] = {0,0};
      int c_q0 = 0;
      ui64 s[8] = {0,0,0,0,0,0,0,0}, val, t;
      ui32 y = 0;
      ui64 *sp = buf;
      for (ui32 x = 0; x < width; x += 4)
      {
        //prepare two quads
        t = sp[0];
        val = t + t; //multiply by 2 and get rid of sign
        val >>= p;  // 2 \mu_p + x
        val &= ~1ULL; // 2 \mu_p
        if (val)
        {
          rho[0] = 1;
          e_q[0] = 64 - (int)count_leading_zeros(--val); //2\mu_p - 1
          e_qmax[0] = e_q[0];
          s[0] = --val + (t >> 63); //v_n = 2(\mu_p-1) + s_n
        }

        t = height > 1 ? sp[stride] : 0;
        ++sp;
        val = t + t; //multiply by 2 and get rid of sign
        val >>= p; // 2 \mu_p + x
        val &= ~1ULL;// 2 \mu_p
        if (val)
        {
          rho[0] += 2;
          e_q[1] = 64 - (int)count_leading_zeros(--val); //2\mu_p - 1
          e_qmax[0] = ojph_max(e_qmax[0], e_q[1]);
          s[1] = --val + (t >> 63); //v_n = 2(\mu_p-1) + s_n
        }

        if (x + 1 < width)
        {
          t = sp[0];
          val = t + t; //multiply by 2 and get rid of sign
          val >>= p; // 2 \mu_p + x
          val &= ~1ULL;// 2 \mu_p
          if (val)
          {
            rho[0] += 4;
            e_q[2] = 64 - (int)count_leading_zeros(--val); //2\mu_p - 1
            e_qmax[0] = ojph_max(e_qmax[0], e_q[2]);
            s[2] = --val + (t >> 63); //v_n = 2(\mu_p-1) + s_n
          }

          t = height > 1 ? sp[stride] : 0;
          ++sp;
          val = t + t; //multiply by 2 and get rid of sign
          val >>= p; // 2 \mu_p + x
          val &= ~1ULL;// 2 \mu_p
          if (val)
          {
            rho[0] += 8;
            e_q[3] = 64 - (int)count_leading_zeros(--val); //2\mu_p - 1
            e_qmax[0] = ojph_max(e_qmax[0], e_q[3]);
            s[3] = --val + (t >> 63); //v_n = 2(\mu_p-1) + s_n
          }
        }

        int Uq0 = ojph_max(e_qmax[0], 1); //kappa_q = 1
        int u_q0 = Uq0 - 1, u_q1 = 0; //kappa_q = 1

        int eps0 = 0;
        if (u_q0 > 0)
        {
          eps0 |= (e_q[0] == e_qmax[0]);
          eps0 |= (e_q[1] == e_qmax[0]) << 1;
          eps0 |= (e_q[2] == e_qmax[0]) << 2;
          eps0 |= (e_q[3] == e_qmax[0]) << 3;
        }
        lep[0] = ojph_max(lep[0], (ui8)e_q[1]); lep++;
        lep[0] = (ui8)e_q[3];
        lcxp[0] = (ui8)(lcxp[0] | (ui8)((rho[0] & 2) >> 1)); lcxp++;
        lcxp[0] = (ui8)((rho[0] & 8) >> 3);

        int ti0c = (c_q0 << 8) + (rho[0] << 4) + eps0;
        ui16 tuple0 = ht_pick_vlc_tuple_u16(vlc_tbl0[ti0c], vlc_tbl0_alt[ti0c],
                                            Uq0, rho[0]);
        vlc_encode(&vlc, tuple0 >> 8, (tuple0 >> 4) & 7);

        if (c_q0 == 0)
          mel_encode(&mel, rho[0] != 0);

        int m = (rho[0] & 1) ? Uq0 - (tuple0 & 1) : 0;
        ms_encode64(&ms, s[0] & ((1ULL << m) - 1), m);
        m = (rho[0] & 2) ? Uq0 - ((tuple0 & 2) >> 1) : 0;
        ms_encode64(&ms, s[1] & ((1ULL << m) - 1), m);
        m = (rho[0] & 4) ? Uq0 - ((tuple0 & 4) >> 2) : 0;
        ms_encode64(&ms, s[2] & ((1ULL << m) - 1), m);
        m = (rho[0] & 8) ? Uq0 - ((tuple0 & 8) >> 3) : 0;
        ms_encode64(&ms, s[3] & ((1ULL << m) - 1), m);

        if (x + 2 < width)
        {
          t = sp[0];
          val = t + t; //multiply by 2 and get rid of sign
          val >>= p; // 2 \mu_p + x
          val &= ~1ULL;// 2 \mu_p
          if (val)
          {
            rho[1] = 1;
            e_q[4] = 64 - (int)count_leading_zeros(--val); //2\mu_p - 1
            e_qmax[1] = e_q[4];
            s[4] = --val + (t >> 63); //v_n = 2(\mu_p-1) + s_n
          }

          t = height > 1 ? sp[stride] : 0;
          ++sp;
          val = t + t; //multiply by 2 and get rid of sign
          val >>= p; // 2 \mu_p + x
          val &= ~1ULL;// 2 \mu_p
          if (val)
          {
            rho[1] += 2;
            e_q[5] = 64 - (int)count_leading_zeros(--val); //2\mu_p - 1
            e_qmax[1] = ojph_max(e_qmax[1], e_q[5]);
            s[5] = --val + (t >> 63); //v_n = 2(\mu_p-1) + s_n
          }

          if (x + 3 < width)
          {
            t = sp[0];
            val = t + t; //multiply by 2 and get rid of sign
            val >>= p; // 2 \mu_p + x
            val &= ~1ULL;// 2 \mu_p
            if (val)
            {
              rho[1] += 4;
              e_q[6] = 64 - (int)count_leading_zeros(--val); //2\mu_p - 1
              e_qmax[1] = ojph_max(e_qmax[1], e_q[6]);
              s[6] = --val + (t >> 63); //v_n = 2(\mu_p-1) + s_n
            }

            t = height > 1 ? sp[stride] : 0;
            ++sp;
            val = t + t; //multiply by 2 and get rid of sign
            val >>= p; // 2 \mu_p + x
            val &= ~1ULL;// 2 \mu_p
            if (val)
            {
              rho[1] += 8;
              e_q[7] = 64 - (int)count_leading_zeros(--val); //2\mu_p - 1
              e_qmax[1] = ojph_max(e_qmax[1], e_q[7]);
              s[7] = --val + (t >> 63); //v_n = 2(\mu_p-1) + s_n
            }
          }

          int c_q1 = (rho[0] >> 1) | (rho[0] & 1);
          int Uq1 = ojph_max(e_qmax[1], 1); //kappa_q = 1
          u_q1 = Uq1 - 1; //kappa_q = 1

          int eps1 = 0;
          if (u_q1 > 0)
          {
            eps1 |= (e_q[4] == e_qmax[1]);
            eps1 |= (e_q[5] == e_qmax[1]) << 1;
            eps1 |= (e_q[6] == e_qmax[1]) << 2;
            eps1 |= (e_q[7] == e_qmax[1]) << 3;
          }
          lep[0] = ojph_max(lep[0], (ui8)e_q[5]); lep++;
          lep[0] = (ui8)e_q[7];
          lcxp[0] |= (ui8)(lcxp[0] | (ui8)((rho[1] & 2) >> 1)); lcxp++;
          lcxp[0] = (ui8)((rho[1] & 8) >> 3);
          int ti1c = (c_q1 << 8) + (rho[1] << 4) + eps1;
          ui16 tuple1 = ht_pick_vlc_tuple_u16(vlc_tbl0[ti1c], vlc_tbl0_alt[ti1c],
                                              Uq1, rho[1]);
          vlc_encode(&vlc, tuple1 >> 8, (tuple1 >> 4) & 7);

          if (c_q1 == 0)
            mel_encode(&mel, rho[1] != 0);

          int m = (rho[1] & 1) ? Uq1 - (tuple1 & 1) : 0;
          ms_encode64(&ms, s[4] & ((1ULL << m) - 1), m);
          m = (rho[1] & 2) ? Uq1 - ((tuple1 & 2) >> 1) : 0;
          ms_encode64(&ms, s[5] & ((1ULL << m) - 1), m);
          m = (rho[1] & 4) ? Uq1 - ((tuple1 & 4) >> 2) : 0;
          ms_encode64(&ms, s[6] & ((1ULL << m) - 1), m);
          m = (rho[1] & 8) ? Uq1 - ((tuple1 & 8) >> 3) : 0;
          ms_encode64(&ms, s[7] & ((1ULL << m) - 1), m);
        }

        if (u_q0 > 0 && u_q1 > 0)
          mel_encode(&mel, ojph_min(u_q0, u_q1) > 2);

        if (u_q0 > 2 && u_q1 > 2)
        {
          vlc_encode(&vlc, uvlc_tbl[u_q0-2].pre, uvlc_tbl[u_q0-2].pre_len);
          vlc_encode(&vlc, uvlc_tbl[u_q1-2].pre, uvlc_tbl[u_q1-2].pre_len);
          vlc_encode(&vlc, uvlc_tbl[u_q0-2].suf, uvlc_tbl[u_q0-2].suf_len);
          vlc_encode(&vlc, uvlc_tbl[u_q1-2].suf, uvlc_tbl[u_q1-2].suf_len);
          vlc_encode(&vlc, uvlc_tbl[u_q0-2].ext, uvlc_tbl[u_q0-2].ext_len);
          vlc_encode(&vlc, uvlc_tbl[u_q1-2].ext, uvlc_tbl[u_q1-2].ext_len);
        }
        else if (u_q0 > 2 && u_q1 > 0)
        {
          vlc_encode(&vlc, uvlc_tbl[u_q0].pre, uvlc_tbl[u_q0].pre_len);
          vlc_encode(&vlc, u_q1 - 1, 1);
          vlc_encode(&vlc, uvlc_tbl[u_q0].suf, uvlc_tbl[u_q0].suf_len);
          vlc_encode(&vlc, uvlc_tbl[u_q0].ext, uvlc_tbl[u_q0].ext_len);
        }
        else
        {
          vlc_encode(&vlc, uvlc_tbl[u_q0].pre, uvlc_tbl[u_q0].pre_len);
          vlc_encode(&vlc, uvlc_tbl[u_q1].pre, uvlc_tbl[u_q1].pre_len);
          vlc_encode(&vlc, uvlc_tbl[u_q0].suf, uvlc_tbl[u_q0].suf_len);
          vlc_encode(&vlc, uvlc_tbl[u_q1].suf, uvlc_tbl[u_q1].suf_len);
          vlc_encode(&vlc, uvlc_tbl[u_q0].ext, uvlc_tbl[u_q0].ext_len);
          vlc_encode(&vlc, uvlc_tbl[u_q1].ext, uvlc_tbl[u_q1].ext_len);
        }

        //prepare for next iteration
        c_q0 = (rho[1] >> 1) | (rho[1] & 1);
        s[0] = s[1] = s[2] = s[3] = s[4] = s[5] = s[6] = s[7] = 0;
        e_q[0]=e_q[1]=e_q[2]=e_q[3]=e_q[4]=e_q[5]=e_q[6]=e_q[7]=0;
        rho[0] = rho[1] = 0; e_qmax[0] = e_qmax[1] = 0;
      }

      lep[1] = 0;

      for (y = 2; y < height; y += 2)
      {
        lep = e_val;
        int max_e = ojph_max(lep[0], lep[1]) - 1;
        lep[0] = 0;
        lcxp = cx_val;
        c_q0 = lcxp[0] + (lcxp[1] << 2);
        lcxp[0] = 0;

        sp = buf + y * stride;
        for (ui32 x = 0; x < width; x += 4)
        {
          //prepare two quads
          t = sp[0];
          val = t + t; //multiply by 2 and get rid of sign
          val >>= p; // 2 \mu_p + x
          val &= ~1ULL;// 2 \mu_p
          if (val)
          {
            rho[0] = 1;
            e_q[0] = 64 - (int)count_leading_zeros(--val); //2\mu_p - 1
            e_qmax[0] = e_q[0];
            s[0] = --val + (t >> 63); //v_n = 2(\mu_p-1) + s_n
          }

          t = y + 1 < height ? sp[stride] : 0;
          ++sp;
          val = t + t; //multiply by 2 and get rid of sign
          val >>= p; // 2 \mu_p + x
          val &= ~1ULL;// 2 \mu_p
          if (val)
          {
            rho[0] += 2;
            e_q[1] = 64 - (int)count_leading_zeros(--val); //2\mu_p - 1
            e_qmax[0] = ojph_max(e_qmax[0], e_q[1]);
            s[1] = --val + (t >> 63); //v_n = 2(\mu_p-1) + s_n
          }

          if (x + 1 < width)
          {
            t = sp[0];
            val = t + t; //multiply by 2 and get rid of sign
            val >>= p; // 2 \mu_p + x
            val &= ~1ULL;// 2 \mu_p
            if (val)
            {
              rho[0] += 4;
              e_q[2] = 64 - (int)count_leading_zeros(--val); //2\mu_p - 1
              e_qmax[0] = ojph_max(e_qmax[0], e_q[2]);
              s[2] = --val + (t >> 63); //v_n = 2(\mu_p-1) + s_n
            }

            t = y + 1 < height ? sp[stride] : 0;
            ++sp;
            val = t + t; //multiply by 2 and get rid of sign
            val >>= p; // 2 \mu_p + x
            val &= ~1ULL;// 2 \mu_p
            if (val)
            {
              rho[0] += 8;
              e_q[3] = 64 - (int)count_leading_zeros(--val); //2\mu_p - 1
              e_qmax[0] = ojph_max(e_qmax[0], e_q[3]);
              s[3] = --val + (t >> 63); //v_n = 2(\mu_p-1) + s_n
            }
          }

          int kappa = (rho[0] & (rho[0]-1)) ? ojph_max(1,max_e) : 1;
          int Uq0 = ojph_max(e_qmax[0], kappa);
          int u_q0 = Uq0 - kappa, u_q1 = 0;

          int eps0 = 0;
          if (u_q0 > 0)
          {
            eps0 |= (e_q[0] == e_qmax[0]);
            eps0 |= (e_q[1] == e_qmax[0]) << 1;
            eps0 |= (e_q[2] == e_qmax[0]) << 2;
            eps0 |= (e_q[3] == e_qmax[0]) << 3;
          }
          lep[0] = ojph_max(lep[0], (ui8)e_q[1]); lep++;
          max_e = ojph_max(lep[0], lep[1]) - 1;
          lep[0] = (ui8)e_q[3];
          lcxp[0] = (ui8)(lcxp[0] | (ui8)((rho[0] & 2) >> 1)); lcxp++;
          int c_q1 = lcxp[0] + (lcxp[1] << 2);
          lcxp[0] = (ui8)((rho[0] & 8) >> 3);
          int ti0d = (c_q0 << 8) + (rho[0] << 4) + eps0;
          ui16 tuple0 = ht_pick_vlc_tuple_u16(vlc_tbl1[ti0d], vlc_tbl1_alt[ti0d],
                                              Uq0, rho[0]);
          vlc_encode(&vlc, tuple0 >> 8, (tuple0 >> 4) & 7);

          if (c_q0 == 0)
              mel_encode(&mel, rho[0] != 0);

          int m = (rho[0] & 1) ? Uq0 - (tuple0 & 1) : 0;
          ms_encode64(&ms, s[0] & ((1ULL << m) - 1), m);
          m = (rho[0] & 2) ? Uq0 - ((tuple0 & 2) >> 1) : 0;
          ms_encode64(&ms, s[1] & ((1ULL << m) - 1), m);
          m = (rho[0] & 4) ? Uq0 - ((tuple0 & 4) >> 2) : 0;
          ms_encode64(&ms, s[2] & ((1ULL << m) - 1), m);
          m = (rho[0] & 8) ? Uq0 - ((tuple0 & 8) >> 3) : 0;
          ms_encode64(&ms, s[3] & ((1ULL << m) - 1), m);

          if (x + 2 < width)
          {
            t = sp[0];
            val = t + t; //multiply by 2 and get rid of sign
            val >>= p; // 2 \mu_p + x
            val &= ~1ULL;// 2 \mu_p
            if (val)
            {
              rho[1] = 1;
              e_q[4] = 64 - (int)count_leading_zeros(--val); //2\mu_p - 1
              e_qmax[1] = e_q[4];
              s[4] = --val + (t >> 63); //v_n = 2(\mu_p-1) + s_n
            }

            t = y + 1 < height ? sp[stride] : 0;
            ++sp;
            val = t + t; //multiply by 2 and get rid of sign
            val >>= p; // 2 \mu_p + x
            val &= ~1ULL;// 2 \mu_p
            if (val)
            {
              rho[1] += 2;
              e_q[5] = 64 - (int)count_leading_zeros(--val); //2\mu_p - 1
              e_qmax[1] = ojph_max(e_qmax[1], e_q[5]);
              s[5] = --val + (t >> 63); //v_n = 2(\mu_p-1) + s_n
            }

            if (x + 3 < width)
            {
              t = sp[0];
              val = t + t; //multiply by 2 and get rid of sign
              val >>= p; // 2 \mu_p + x
              val &= ~1ULL;// 2 \mu_p
              if (val)
              {
                rho[1] += 4;
                e_q[6] = 64 - (int)count_leading_zeros(--val); //2\mu_p - 1
                e_qmax[1] = ojph_max(e_qmax[1], e_q[6]);
                s[6] = --val + (t >> 63); //v_n = 2(\mu_p-1) + s_n
              }

              t = y + 1 < height ? sp[stride] : 0;
              ++sp;
              val = t + t; //multiply by 2 and get rid of sign
              val >>= p; // 2 \mu_p + x
              val &= ~1ULL;// 2 \mu_p
              if (val)
              {
                rho[1] += 8;
                e_q[7] = 64 - (int)count_leading_zeros(--val); //2\mu_p - 1
                e_qmax[1] = ojph_max(e_qmax[1], e_q[7]);
                s[7] = --val + (t >> 63); //v_n = 2(\mu_p-1) + s_n
              }
            }

            kappa = (rho[1] & (rho[1]-1)) ? ojph_max(1,max_e) : 1;
            c_q1 |= ((rho[0] & 4) >> 1) | ((rho[0] & 8) >> 2);
            int Uq1 = ojph_max(e_qmax[1], kappa);
            u_q1 = Uq1 - kappa;

            int eps1 = 0;
            if (u_q1 > 0)
            {
              eps1 |= (e_q[4] == e_qmax[1]);
              eps1 |= (e_q[5] == e_qmax[1]) << 1;
              eps1 |= (e_q[6] == e_qmax[1]) << 2;
              eps1 |= (e_q[7] == e_qmax[1]) << 3;
            }
            lep[0] = ojph_max(lep[0], (ui8)e_q[5]); lep++;
            max_e = ojph_max(lep[0], lep[1]) - 1;
            lep[0] = (ui8)e_q[7];
            lcxp[0] = (ui8)(lcxp[0] | (ui8)((rho[1] & 2) >> 1)); lcxp++;
            c_q0 = lcxp[0] + (lcxp[1] << 2);
            lcxp[0] = (ui8)((rho[1] & 8) >> 3);
            int ti1d = (c_q1 << 8) + (rho[1] << 4) + eps1;
            ui16 tuple1 = ht_pick_vlc_tuple_u16(vlc_tbl1[ti1d], vlc_tbl1_alt[ti1d],
                                                Uq1, rho[1]);
            vlc_encode(&vlc, tuple1 >> 8, (tuple1 >> 4) & 7);

            if (c_q1 == 0)
              mel_encode(&mel, rho[1] != 0);

            int m = (rho[1] & 1) ? Uq1 - (tuple1 & 1) : 0;
            ms_encode64(&ms, s[4] & ((1ULL << m) - 1), m);
            m = (rho[1] & 2) ? Uq1 - ((tuple1 & 2) >> 1) : 0;
            ms_encode64(&ms, s[5] & ((1ULL << m) - 1), m);
            m = (rho[1] & 4) ? Uq1 - ((tuple1 & 4) >> 2) : 0;
            ms_encode64(&ms, s[6] & ((1ULL << m) - 1), m);
            m = (rho[1] & 8) ? Uq1 - ((tuple1 & 8) >> 3) : 0;
            ms_encode64(&ms, s[7] & ((1ULL << m) - 1), m);
          }

          vlc_encode(&vlc, uvlc_tbl[u_q0].pre, uvlc_tbl[u_q0].pre_len);
          vlc_encode(&vlc, uvlc_tbl[u_q1].pre, uvlc_tbl[u_q1].pre_len);
          vlc_encode(&vlc, uvlc_tbl[u_q0].suf, uvlc_tbl[u_q0].suf_len);
          vlc_encode(&vlc, uvlc_tbl[u_q1].suf, uvlc_tbl[u_q1].suf_len);
          vlc_encode(&vlc, uvlc_tbl[u_q0].ext, uvlc_tbl[u_q0].ext_len);
          vlc_encode(&vlc, uvlc_tbl[u_q1].ext, uvlc_tbl[u_q1].ext_len);

          //prepare for next iteration
          c_q0 |= ((rho[1] & 4) >> 1) | ((rho[1] & 8) >> 2);
          s[0] = s[1] = s[2] = s[3] = s[4] = s[5] = s[6] = s[7] = 0;
          e_q[0]=e_q[1]=e_q[2]=e_q[3]=e_q[4]=e_q[5]=e_q[6]=e_q[7]=0;
          rho[0] = rho[1] = 0; e_qmax[0] = e_qmax[1] = 0;
        }
      }


      terminate_mel_vlc(&mel, &vlc);
      ms_terminate(&ms);

      //copy to elastic
      lengths[0] = mel.pos + vlc.pos + ms.pos;
      lengths[1] = 0;
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
  }
}
