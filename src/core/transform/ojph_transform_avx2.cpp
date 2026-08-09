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
// File: ojph_transform_avx2.cpp
// Author: Aous Naman
// Date: 28 August 2019
//***************************************************************************/

#include "ojph_arch.h"
#if defined(OJPH_ARCH_I386) || defined(OJPH_ARCH_X86_64)

#include <climits>
#include <cstdio>

#include "ojph_defs.h"
#include "ojph_mem.h"
#include "ojph_params.h"
#include "../codestream/ojph_params_local.h"

#include "ojph_transform.h"
#include "ojph_transform_local.h"

#include <immintrin.h>

namespace ojph {
  namespace local {

    /////////////////////////////////////////////////////////////////////////
    // https://github.com/seung-lab/dijkstra3d/blob/master/libdivide.h
    static inline
    __m256i avx2_mm256_srai_epi64(__m256i a, int amt, __m256i m)
    {
      // note than m must be obtained using
      // __m256i m = _mm256_set1_epi64x(1ULL << (63 - amt));
      __m256i x = _mm256_srli_epi64(a, amt);
      x = _mm256_xor_si256(x, m);
      __m256i result = _mm256_sub_epi64(x, m);
      return result;
    }

    /////////////////////////////////////////////////////////////////////////
    static
    void avx2_rev_vert_step32(const lifting_step* s, const line_buf* sig,
                              const line_buf* other, const line_buf* aug,
                              ui32 repeat, bool synthesis)
    {
      const si32 a = s->rev.Aatk;
      const si32 b = s->rev.Batk;
      const ui8 e = s->rev.Eatk;
      __m256i va = _mm256_set1_epi32(a);
      __m256i vb = _mm256_set1_epi32(b);

      si32* dst = aug->i32;
      const si32* src1 = sig->i32, * src2 = other->i32;
      // The general definition of the wavelet in Part 2 is slightly
      // different to part 2, although they are mathematically equivalent
      // here, we identify the simpler form from Part 1 and employ them
      if (a == 1)
      { // 5/3 update and any case with a == 1
        int i = (int)repeat;
        if (synthesis)
          for (; i > 0; i -= 8, dst += 8, src1 += 8, src2 += 8)
          {
            __m256i s1 = _mm256_load_si256((__m256i*)src1);
            __m256i s2 = _mm256_load_si256((__m256i*)src2);
            __m256i d = _mm256_load_si256((__m256i*)dst);
            __m256i t = _mm256_add_epi32(s1, s2);
            __m256i v = _mm256_add_epi32(vb, t);
            __m256i w = _mm256_srai_epi32(v, e);
            d = _mm256_sub_epi32(d, w);
            _mm256_store_si256((__m256i*)dst, d);
          }
        else
          for (; i > 0; i -= 8, dst += 8, src1 += 8, src2 += 8)
          {
            __m256i s1 = _mm256_load_si256((__m256i*)src1);
            __m256i s2 = _mm256_load_si256((__m256i*)src2);
            __m256i d = _mm256_load_si256((__m256i*)dst);
            __m256i t = _mm256_add_epi32(s1, s2);
            __m256i v = _mm256_add_epi32(vb, t);
            __m256i w = _mm256_srai_epi32(v, e);
            d = _mm256_add_epi32(d, w);
            _mm256_store_si256((__m256i*)dst, d);
          }
      }
      else if (a == -1 && b == 1 && e == 1)
      { // 5/3 predict
        int i = (int)repeat;
        if (synthesis)
          for (; i > 0; i -= 8, dst += 8, src1 += 8, src2 += 8)
          {
            __m256i s1 = _mm256_load_si256((__m256i*)src1);
            __m256i s2 = _mm256_load_si256((__m256i*)src2);
            __m256i d = _mm256_load_si256((__m256i*)dst);
            __m256i t = _mm256_add_epi32(s1, s2);
            __m256i w = _mm256_srai_epi32(t, e);
            d = _mm256_add_epi32(d, w);
            _mm256_store_si256((__m256i*)dst, d);
          }
        else
          for (; i > 0; i -= 8, dst += 8, src1 += 8, src2 += 8)
          {
            __m256i s1 = _mm256_load_si256((__m256i*)src1);
            __m256i s2 = _mm256_load_si256((__m256i*)src2);
            __m256i d = _mm256_load_si256((__m256i*)dst);
            __m256i t = _mm256_add_epi32(s1, s2);
            __m256i w = _mm256_srai_epi32(t, e);
            d = _mm256_sub_epi32(d, w);
            _mm256_store_si256((__m256i*)dst, d);
          }
      }
      else if (a == -1)
      { // any case with a == -1, which is not 5/3 predict
        int i = (int)repeat;
        if (synthesis)
          for (; i > 0; i -= 8, dst += 8, src1 += 8, src2 += 8)
          {
            __m256i s1 = _mm256_load_si256((__m256i*)src1);
            __m256i s2 = _mm256_load_si256((__m256i*)src2);
            __m256i d = _mm256_load_si256((__m256i*)dst);
            __m256i t = _mm256_add_epi32(s1, s2);
            __m256i v = _mm256_sub_epi32(vb, t);
            __m256i w = _mm256_srai_epi32(v, e);
            d = _mm256_sub_epi32(d, w);
            _mm256_store_si256((__m256i*)dst, d);
          }
        else
          for (; i > 0; i -= 8, dst += 8, src1 += 8, src2 += 8)
          {
            __m256i s1 = _mm256_load_si256((__m256i*)src1);
            __m256i s2 = _mm256_load_si256((__m256i*)src2);
            __m256i d = _mm256_load_si256((__m256i*)dst);
            __m256i t = _mm256_add_epi32(s1, s2);
            __m256i v = _mm256_sub_epi32(vb, t);
            __m256i w = _mm256_srai_epi32(v, e);
            d = _mm256_add_epi32(d, w);
            _mm256_store_si256((__m256i*)dst, d);
          }
      }
      else { // general case
        int i = (int)repeat;
        if (synthesis)
          for (; i > 0; i -= 8, dst += 8, src1 += 8, src2 += 8)
          {
            __m256i s1 = _mm256_load_si256((__m256i*)src1);
            __m256i s2 = _mm256_load_si256((__m256i*)src2);
            __m256i d = _mm256_load_si256((__m256i*)dst);
            __m256i t = _mm256_add_epi32(s1, s2);
            __m256i u = _mm256_mullo_epi32(va, t);
            __m256i v = _mm256_add_epi32(vb, u);
            __m256i w = _mm256_srai_epi32(v, e);
            d = _mm256_sub_epi32(d, w);
            _mm256_store_si256((__m256i*)dst, d);
          }
        else
          for (; i > 0; i -= 8, dst += 8, src1 += 8, src2 += 8)
          {
            __m256i s1 = _mm256_load_si256((__m256i*)src1);
            __m256i s2 = _mm256_load_si256((__m256i*)src2);
            __m256i d = _mm256_load_si256((__m256i*)dst);
            __m256i t = _mm256_add_epi32(s1, s2);
            __m256i u = _mm256_mullo_epi32(va, t);
            __m256i v = _mm256_add_epi32(vb, u);
            __m256i w = _mm256_srai_epi32(v, e);
            d = _mm256_add_epi32(d, w);
            _mm256_store_si256((__m256i*)dst, d);
          }
      }
    }

    /////////////////////////////////////////////////////////////////////////
    static
    void avx2_rev_vert_step64(const lifting_step* s, const line_buf* sig,
                              const line_buf* other, const line_buf* aug,
                              ui32 repeat, bool synthesis)
    {
      const si32 a = s->rev.Aatk;
      const si32 b = s->rev.Batk;
      const ui8 e = s->rev.Eatk;
      __m256i vb = _mm256_set1_epi64x(b);
      __m256i ve = _mm256_set1_epi64x(1LL << (63 - e));

      si64* dst = aug->i64;
      const si64* src1 = sig->i64, * src2 = other->i64;
      // The general definition of the wavelet in Part 2 is slightly
      // different to part 2, although they are mathematically equivalent
      // here, we identify the simpler form from Part 1 and employ them
      if (a == 1)
      { // 5/3 update and any case with a == 1
        int i = (int)repeat;
        if (synthesis)
          for (; i > 0; i -= 4, dst += 4, src1 += 4, src2 += 4)
          {
            __m256i s1 = _mm256_load_si256((__m256i*)src1);
            __m256i s2 = _mm256_load_si256((__m256i*)src2);
            __m256i d = _mm256_load_si256((__m256i*)dst);
            __m256i t = _mm256_add_epi64(s1, s2);
            __m256i v = _mm256_add_epi64(vb, t);
            __m256i w = avx2_mm256_srai_epi64(v, e, ve);
            d = _mm256_sub_epi64(d, w);
            _mm256_store_si256((__m256i*)dst, d);
          }
        else
          for (; i > 0; i -= 4, dst += 4, src1 += 4, src2 += 4)
          {
            __m256i s1 = _mm256_load_si256((__m256i*)src1);
            __m256i s2 = _mm256_load_si256((__m256i*)src2);
            __m256i d = _mm256_load_si256((__m256i*)dst);
            __m256i t = _mm256_add_epi64(s1, s2);
            __m256i v = _mm256_add_epi64(vb, t);
            __m256i w = avx2_mm256_srai_epi64(v, e, ve);
            d = _mm256_add_epi64(d, w);
            _mm256_store_si256((__m256i*)dst, d);
          }
      }
      else if (a == -1 && b == 1 && e == 1)
      { // 5/3 predict
        int i = (int)repeat;
        if (synthesis)
          for (; i > 0; i -= 4, dst += 4, src1 += 4, src2 += 4)
          {
            __m256i s1 = _mm256_load_si256((__m256i*)src1);
            __m256i s2 = _mm256_load_si256((__m256i*)src2);
            __m256i d = _mm256_load_si256((__m256i*)dst);
            __m256i t = _mm256_add_epi64(s1, s2);
            __m256i w = avx2_mm256_srai_epi64(t, e, ve);
            d = _mm256_add_epi64(d, w);
            _mm256_store_si256((__m256i*)dst, d);
          }
        else
          for (; i > 0; i -= 4, dst += 4, src1 += 4, src2 += 4)
          {
            __m256i s1 = _mm256_load_si256((__m256i*)src1);
            __m256i s2 = _mm256_load_si256((__m256i*)src2);
            __m256i d = _mm256_load_si256((__m256i*)dst);
            __m256i t = _mm256_add_epi64(s1, s2);
            __m256i w = avx2_mm256_srai_epi64(t, e, ve);
            d = _mm256_sub_epi64(d, w);
            _mm256_store_si256((__m256i*)dst, d);
          }
      }
      else if (a == -1)
      { // any case with a == -1, which is not 5/3 predict
        int i = (int)repeat;
        if (synthesis)
          for (; i > 0; i -= 4, dst += 4, src1 += 4, src2 += 4)
          {
            __m256i s1 = _mm256_load_si256((__m256i*)src1);
            __m256i s2 = _mm256_load_si256((__m256i*)src2);
            __m256i d = _mm256_load_si256((__m256i*)dst);
            __m256i t = _mm256_add_epi64(s1, s2);
            __m256i v = _mm256_sub_epi64(vb, t);
            __m256i w = avx2_mm256_srai_epi64(v, e, ve);
            d = _mm256_sub_epi64(d, w);
            _mm256_store_si256((__m256i*)dst, d);
          }
        else
          for (; i > 0; i -= 4, dst += 4, src1 += 4, src2 += 4)
          {
            __m256i s1 = _mm256_load_si256((__m256i*)src1);
            __m256i s2 = _mm256_load_si256((__m256i*)src2);
            __m256i d = _mm256_load_si256((__m256i*)dst);
            __m256i t = _mm256_add_epi64(s1, s2);
            __m256i v = _mm256_sub_epi64(vb, t);
            __m256i w = avx2_mm256_srai_epi64(v, e, ve);
            d = _mm256_add_epi64(d, w);
            _mm256_store_si256((__m256i*)dst, d);
          }
      }
      else { // general case
        // 64bit multiplication is not supported in avx2;
        // in particular, _mm256_mullo_epi64.
        if (synthesis)
          for (ui32 i = repeat; i > 0; --i)
            *dst++ -= (b + a * (*src1++ + *src2++)) >> e;
        else
          for (ui32 i = repeat; i > 0; --i)
            *dst++ += (b + a * (*src1++ + *src2++)) >> e;
      }
    }

    /////////////////////////////////////////////////////////////////////////
    void avx2_rev_vert_step(const lifting_step* s, const line_buf* sig,
                            const line_buf* other, const line_buf* aug,
                            ui32 repeat, bool synthesis)
    {
      // the step adds (Batk + Aatk * x) >> Eatk, which is identically zero
      // when Aatk == 0 and Batk >> Eatk == 0 (the rev13 update step is
      // such); skip the pass
      if (s->rev.Aatk == 0 && (s->rev.Batk >> s->rev.Eatk) == 0)
        return;

      if (((sig != NULL) && (sig->flags & line_buf::LFT_32BIT)) ||
          ((aug != NULL) && (aug->flags & line_buf::LFT_32BIT)) ||
          ((other != NULL) && (other->flags & line_buf::LFT_32BIT)))
      {
        assert((sig == NULL || sig->flags & line_buf::LFT_32BIT) &&
               (other == NULL || other->flags & line_buf::LFT_32BIT) &&
               (aug == NULL || aug->flags & line_buf::LFT_32BIT));
        avx2_rev_vert_step32(s, sig, other, aug, repeat, synthesis);
      }
      else
      {
        assert((sig == NULL || sig->flags & line_buf::LFT_64BIT) &&
               (other == NULL || other->flags & line_buf::LFT_64BIT) &&
               (aug == NULL || aug->flags & line_buf::LFT_64BIT));
        avx2_rev_vert_step64(s, sig, other, aug, repeat, synthesis);
      }
    }

  } // !local
} // !ojph

#endif
