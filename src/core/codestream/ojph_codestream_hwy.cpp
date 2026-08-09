//***************************************************************************/
// This software is released under the 2-Clause BSD license, included
// below.
//
// Copyright (c) 2026, Aous Naman
// Copyright (c) 2026, Kakadu Software Pty Ltd, Australia
// Copyright (c) 2026, The University of New South Wales, Australia
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
// File: ojph_codestream_hwy.cpp
//***************************************************************************/

// Decode-side data-movement kernels written with Google Highway (libhwy).
// The file is compiled once for the best statically selected target (the
// build adds the target flags, e.g. -mavx2); the caller must gate calls
// on a matching run-time CPU check (see ojph_codeblock_fun.cpp).

#include "ojph_defs.h"
#include "ojph_arch.h"

#if defined(OJPH_ENABLE_HWY)

#include <hwy/highway.h>

namespace hn = hwy::HWY_NAMESPACE;

namespace ojph {
  namespace local {

    //////////////////////////////////////////////////////////////////////////
    // sign-magnitude ui32 codeblock samples to a 16-bit line; the hwy
    // equivalent of rev_tx_from_cb16 (see ojph_codestream_gen.cpp)
    void hwy_rev_tx_from_cb16(const ui32 *sp, si16 *dp, ui32 K_max,
                              ui32 count)
    {
      const int shift = (int)(31 - K_max);
      const hn::ScalableTag<si32> d32;
      const hn::Repartition<si16, decltype(d32)> d16;
      const size_t N = hn::Lanes(d32);
      const auto mag_mask = hn::Set(d32, 0x7FFFFFFF);

      ui32 i = 0;
      for ( ; i + 2 * N <= count; i += (ui32)(2 * N))
      {
        auto v0 = hn::LoadU(d32, (const si32*)sp + i);
        auto v1 = hn::LoadU(d32, (const si32*)sp + i + N);
        // (v & 0x7FFFFFFF) >> shift; the value is non-negative, so an
        // arithmetic shift equals the logical shift of the generic code
        auto m0 = hn::ShiftRightSame(hn::And(v0, mag_mask), shift);
        auto m1 = hn::ShiftRightSame(hn::And(v1, mag_mask), shift);
        // negate when v is negative; a zero magnitude stays zero for
        // either sign, so the undef-if-zero case is harmless
        m0 = hn::IfNegativeThenNegOrUndefIfZero(v0, m0);
        m1 = hn::IfNegativeThenNegOrUndefIfZero(v1, m1);
        hn::StoreU(hn::OrderedDemote2To(d16, m0, m1), d16, dp + i);
      }
      for ( ; i < count; ++i)
      {
        ui32 v = sp[i];
        si32 val = (si32)((v & 0x7FFFFFFFU) >> shift);
        dp[i] = (si16)((v & 0x80000000U) ? -val : val);
      }
    }

    //////////////////////////////////////////////////////////////////////////
    // sign-magnitude ui32 codeblock samples to a 32-bit line; the hwy
    // equivalent of gen_rev_tx_from_cb32.  Not dispatched: it measured
    // slightly slower than the hand-written avx2_rev_tx_from_cb32, which
    // rounds count up to whole vectors instead of running a scalar tail;
    // kept for non-AVX2 targets and future re-evaluation
    void hwy_rev_tx_from_cb32(const ui32 *sp, void *dp, ui32 K_max,
                              float delta, ui32 count)
    {
      ojph_unused(delta);
      const int shift = (int)(31 - K_max);
      const hn::ScalableTag<si32> d32;
      const size_t N = hn::Lanes(d32);
      const auto mag_mask = hn::Set(d32, 0x7FFFFFFF);
      si32 *p = (si32*)dp;

      ui32 i = 0;
      for ( ; i + N <= count; i += (ui32)N)
      {
        auto v = hn::LoadU(d32, (const si32*)sp + i);
        auto m = hn::ShiftRightSame(hn::And(v, mag_mask), shift);
        hn::StoreU(hn::IfNegativeThenNegOrUndefIfZero(v, m), d32, p + i);
      }
      for ( ; i < count; ++i)
      {
        ui32 v = sp[i];
        si32 val = (si32)((v & 0x7FFFFFFFU) >> shift);
        p[i] = (v & 0x80000000U) ? -val : val;
      }
    }

    //////////////////////////////////////////////////////////////////////////
    // fold a vector or-accumulator into the caller's max_val array; the
    // array holds 8 ui32 (see codeblock::max_val32), which matches the
    // AVX2 vector width this file is compiled for; other widths fold to
    // a scalar in element 0, which every find_max_val32 handles too
    static inline
    void hwy_fold_max_val(hn::Vec<hn::ScalableTag<si32> > tmax,
                          ui32 *max_val)
    {
      const hn::ScalableTag<si32> d;
      const size_t N = hn::Lanes(d);
      if (N == 8)
      {
        auto m = hn::LoadU(d, (si32*)max_val);
        hn::StoreU(hn::Or(m, tmax), d, (si32*)max_val);
      }
      else
      {
        HWY_ALIGN si32 tmp[HWY_MAX_LANES_D(hn::ScalableTag<si32>)];
        hn::Store(tmax, d, tmp);
        ui32 t = 0;
        for (size_t i = 0; i < N; ++i)
          t |= (ui32)tmp[i];
        max_val[0] |= t;
      }
    }

    //////////////////////////////////////////////////////////////////////////
    // 32-bit line samples to sign-magnitude ui32 codeblock samples,
    // or-accumulating the magnitudes into max_val; the hwy equivalent of
    // gen_rev_tx_to_cb32.  Like the AVX2 implementation, the last
    // iteration loads and stores a whole vector (the buffers are padded)
    // and masks the extra lanes out of the accumulator only.
    void hwy_rev_tx_to_cb32(const void *sp, ui32 *dp, ui32 K_max,
                            float delta_inv, ui32 count, ui32* max_val)
    {
      ojph_unused(delta_inv);
      const int shift = (int)(31 - K_max);
      const hn::ScalableTag<si32> d;
      const size_t N = hn::Lanes(d);
      const auto sign_mask = hn::Set(d, (si32)0x80000000);
      auto tmax = hn::Zero(d);
      const si32 *p = (const si32*)sp;

      ui32 i = 0;
      for ( ; i + N <= count; i += (ui32)N)
      {
        auto v = hn::LoadU(d, p + i);
        auto sign = hn::And(v, sign_mask);
        auto val = hn::ShiftLeftSame(hn::Abs(v), shift);
        tmax = hn::Or(tmax, val);
        hn::StoreU(hn::Or(val, sign), d, (si32*)dp + i);
      }
      if (i < count)
      {
        auto v = hn::LoadU(d, p + i);
        auto sign = hn::And(v, sign_mask);
        auto val = hn::ShiftLeftSame(hn::Abs(v), shift);
        tmax = hn::Or(tmax,
          hn::IfThenElseZero(hn::FirstN(d, count - i), val));
        hn::StoreU(hn::Or(val, sign), d, (si32*)dp + i);
      }
      hwy_fold_max_val(tmax, max_val);
    }

    //////////////////////////////////////////////////////////////////////////
    // quantize float line samples to sign-magnitude ui32 codeblock
    // samples, or-accumulating the magnitudes into max_val; the hwy
    // equivalent of avx2_irv_tx_to_cb32 (round to nearest, like the
    // SSE2/AVX2 implementations; the generic implementation truncates).
    // NearestIntInRange is a bare cvt (values are in range by design,
    // as the other implementations also assume)
    void hwy_irv_tx_to_cb32(const void *sp, ui32 *dp, ui32 K_max,
                            float delta_inv, ui32 count, ui32* max_val)
    {
      ojph_unused(K_max);
      const hn::ScalableTag<si32> d;
      const hn::RebindToFloat<decltype(d)> df;
      const size_t N = hn::Lanes(d);
      const auto sign_mask = hn::Set(d, (si32)0x80000000);
      const auto vdelta_inv = hn::Set(df, delta_inv);
      auto tmax = hn::Zero(d);
      const float *p = (const float*)sp;

      ui32 i = 0;
      for ( ; i + N <= count; i += (ui32)N)
      {
        auto vf = hn::Mul(hn::LoadU(df, p + i), vdelta_inv);
        auto t = hn::NearestIntInRange(d, vf);
        auto sign = hn::And(t, sign_mask);
        auto val = hn::Abs(t);
        tmax = hn::Or(tmax, val);
        hn::StoreU(hn::Or(val, sign), d, (si32*)dp + i);
      }
      if (i < count)
      {
        auto vf = hn::Mul(hn::LoadU(df, p + i), vdelta_inv);
        auto t = hn::NearestIntInRange(d, vf);
        auto sign = hn::And(t, sign_mask);
        auto val = hn::Abs(t);
        tmax = hn::Or(tmax,
          hn::IfThenElseZero(hn::FirstN(d, count - i), val));
        hn::StoreU(hn::Or(val, sign), d, (si32*)dp + i);
      }
      hwy_fold_max_val(tmax, max_val);
    }

    //////////////////////////////////////////////////////////////////////////
    // sign-magnitude ui32 codeblock samples to a dequantized float line;
    // the hwy equivalent of gen_irv_tx_from_cb32
    void hwy_irv_tx_from_cb32(const ui32 *sp, void *dp, ui32 K_max,
                              float delta, ui32 count)
    {
      ojph_unused(K_max);
      const hn::ScalableTag<si32> d;
      const hn::RebindToFloat<decltype(d)> df;
      const size_t N = hn::Lanes(d);
      const auto mag_mask = hn::Set(d, 0x7FFFFFFF);
      const auto vdelta = hn::Set(df, delta);
      float *p = (float*)dp;

      // the loop overruns count to whole vectors, as the AVX2
      // implementation does; the buffers are padded
      for (ui32 i = 0; i < count; i += (ui32)N)
      {
        auto v = hn::LoadU(d, (const si32*)sp + i);
        auto mag = hn::And(v, mag_mask);
        auto valf = hn::Mul(hn::ConvertTo(df, mag), vdelta);
        auto sign = hn::AndNot(mag_mask, v);
        valf = hn::Or(valf, hn::BitCast(df, sign));
        hn::StoreU(valf, df, p + i);
      }
    }

    //////////////////////////////////////////////////////////////////////////
    // widen a 16-bit component line to 32 bits, undoing the level shift;
    // the hwy equivalent of the scalar loop in tile::pull
    void hwy_rev_convert16(const si16 *sp, si32 *dp, si32 shift, ui32 count)
    {
      const hn::ScalableTag<si32> d32;
      const hn::Rebind<si16, decltype(d32)> d16;
      const size_t N = hn::Lanes(d32);
      const auto vshift = hn::Set(d32, shift);

      ui32 i = 0;
      for ( ; i + N <= count; i += (ui32)N)
      {
        auto v = hn::PromoteTo(d32, hn::LoadU(d16, sp + i));
        hn::StoreU(hn::Add(v, vshift), d32, dp + i);
      }
      for ( ; i < count; ++i)
        dp[i] = (si32)sp[i] + shift;
    }

  }
}

#endif // OJPH_ENABLE_HWY
