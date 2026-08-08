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
