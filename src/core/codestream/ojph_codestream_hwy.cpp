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

// Data-movement kernels written with Google Highway (libhwy).  The file
// is compiled once per Highway target (SSE4, AVX2, AVX3, ...) through
// foreach_target.h, and every public function dispatches to the best
// target the CPU supports at run time; the caller must still gate calls
// on hwy_tx_kernels_available() (see ojph_codeblock_fun.cpp), which
// verifies that at least one of the compiled SIMD targets is available.

#undef HWY_TARGET_INCLUDE
#define HWY_TARGET_INCLUDE "codestream/ojph_codestream_hwy.cpp"
#include <hwy/foreach_target.h>
#include <hwy/highway.h>

#include "ojph_defs.h"
#include "ojph_arch.h"

#if defined(OJPH_ENABLE_HWY)

HWY_BEFORE_NAMESPACE();
namespace ojph {
  namespace local {
    namespace HWY_NAMESPACE {

    namespace hn = hwy::HWY_NAMESPACE;

    //////////////////////////////////////////////////////////////////////////
    // sign-magnitude ui32 codeblock samples to a 16-bit line; the hwy
    // equivalent of rev_tx_from_cb16 (see ojph_codestream_gen.cpp)
    void rev_tx_from_cb16(const ui32 *sp, si16 *dp, ui32 K_max,
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
    // equivalent of gen_rev_tx_from_cb32.  At AVX2 the hand-written
    // avx2_rev_tx_from_cb32, which rounds count up to whole vectors
    // instead of running a scalar tail, measured slightly faster and is
    // dispatched instead; this serves the other targets
    void rev_tx_from_cb32(const ui32 *sp, void *dp, ui32 K_max,
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
    // AVX2 vector width; other widths fold entry-wise into the first
    // min(N, 8) entries, which every find_max_val32 handles too
    static inline
    void fold_max_val(hn::Vec<hn::ScalableTag<si32> > tmax,
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
        for (size_t i = 0; i < N; ++i)
          max_val[i & 7] |= (ui32)tmp[i];
      }
    }

    //////////////////////////////////////////////////////////////////////////
    // 32-bit line samples to sign-magnitude ui32 codeblock samples,
    // or-accumulating the magnitudes into max_val; the hwy equivalent of
    // gen_rev_tx_to_cb32.  Like the AVX2 implementation, the last
    // iteration loads and stores a whole vector (the buffers are padded)
    // and masks the extra lanes out of the accumulator only.
    void rev_tx_to_cb32(const void *sp, ui32 *dp, ui32 K_max,
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
      for ( ; i + 2 * N <= count; i += 2 * (ui32)N)
      { // unrolled twice; large codeblock widths benefit measurably
        auto v0 = hn::LoadU(d, p + i);
        auto v1 = hn::LoadU(d, p + i + N);
        auto sign0 = hn::And(v0, sign_mask);
        auto sign1 = hn::And(v1, sign_mask);
        auto val0 = hn::ShiftLeftSame(hn::Abs(v0), shift);
        auto val1 = hn::ShiftLeftSame(hn::Abs(v1), shift);
        tmax = hn::Or(tmax, hn::Or(val0, val1));
        hn::StoreU(hn::Or(val0, sign0), d, (si32*)dp + i);
        hn::StoreU(hn::Or(val1, sign1), d, (si32*)dp + i + N);
      }
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
      fold_max_val(tmax, max_val);
    }

    //////////////////////////////////////////////////////////////////////////
    // round to nearest; NearestIntInRange (a bare cvt) exists only for
    // the x86 SIMD targets, so the portable fallback target uses
    // NearestInt, which is equivalent for the in-range values here
    static inline hn::Vec<hn::ScalableTag<si32> >
    nearest_int(hn::ScalableTag<si32> d,
                    hn::Vec<hn::ScalableTag<float> > v)
    {
#if HWY_TARGET == HWY_EMU128 || HWY_TARGET == HWY_SCALAR
      (void)d;
      return hn::NearestInt(v);
#else
      return hn::NearestIntInRange(d, v);
#endif
    }

    //////////////////////////////////////////////////////////////////////////
    // quantize float line samples to sign-magnitude ui32 codeblock
    // samples, or-accumulating the magnitudes into max_val; the hwy
    // equivalent of avx2_irv_tx_to_cb32 (round to nearest, like the
    // SSE2/AVX2 implementations; the generic implementation truncates).
    // NearestIntInRange is a bare cvt (values are in range by design,
    // as the other implementations also assume)
    void irv_tx_to_cb32(const void *sp, ui32 *dp, ui32 K_max,
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
      si32 *q = (si32*)dp;

      // pointer-bumping loop; gcc compiles it faster than an indexed one
      ui32 rem = count;
      for ( ; rem >= N; rem -= (ui32)N, p += N, q += N)
      {
        auto vf = hn::Mul(hn::LoadU(df, p), vdelta_inv);
        auto t = nearest_int(d, vf);
        auto sign = hn::And(t, sign_mask);
        auto val = hn::Abs(t);
        tmax = hn::Or(tmax, val);
        hn::StoreU(hn::Or(val, sign), d, q);
      }
      if (rem)
      {
        auto vf = hn::Mul(hn::LoadU(df, p), vdelta_inv);
        auto t = nearest_int(d, vf);
        auto sign = hn::And(t, sign_mask);
        auto val = hn::Abs(t);
        tmax = hn::Or(tmax,
          hn::IfThenElseZero(hn::FirstN(d, rem), val));
        hn::StoreU(hn::Or(val, sign), d, q);
      }
      fold_max_val(tmax, max_val);
    }

    //////////////////////////////////////////////////////////////////////////
    // sign-magnitude ui32 codeblock samples to a dequantized float line;
    // the hwy equivalent of gen_irv_tx_from_cb32
    void irv_tx_from_cb32(const ui32 *sp, void *dp, ui32 K_max,
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
    void rev_convert16(const si16 *sp, si32 *dp, si32 shift, ui32 count)
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

    } // !HWY_NAMESPACE
  }
}
HWY_AFTER_NAMESPACE();

#if HWY_ONCE

namespace ojph {
  namespace local {

    HWY_EXPORT(rev_tx_from_cb16);
    HWY_EXPORT(rev_tx_from_cb32);
    HWY_EXPORT(rev_tx_to_cb32);
    HWY_EXPORT(irv_tx_to_cb32);
    HWY_EXPORT(irv_tx_from_cb32);
    HWY_EXPORT(rev_convert16);

    //////////////////////////////////////////////////////////////////////////
    // True when at least one of the SIMD targets compiled into this file
    // is available at run time; the dispatch sites install the functions
    // below only in that case.  hwy assumes its baseline target is
    // supported without checking, so when the baseline needs more than
    // the architecture guarantees (an MSVC /arch:AVX2 build; the GCC and
    // clang builds keep the baseline at portable EMU128), verify it with
    // our own CPU detection.
    bool hwy_tx_kernels_available()
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

    //////////////////////////////////////////////////////////////////////////
    // True when dispatch will resolve to an AVX-512 class target (the
    // target bits are ordered so that smaller values are newer); used to
    // prefer the hwy kernels over the hand-written AVX2 survivors where
    // the 512-bit versions measured faster
    bool hwy_tx_kernels_use_avx3()
    {
      if (!hwy_tx_kernels_available())
        return false;
      const int64_t sup = hwy::SupportedTargets();
      hwy::GetChosenTarget().Update(sup); // see above
      const int64_t simd = sup & HWY_TARGETS & ~(HWY_EMU128 | HWY_SCALAR);
      const int64_t best = simd & (-simd);
      return best != 0 && best <= HWY_AVX3;
    }

    //////////////////////////////////////////////////////////////////////////
    // or-reduce the 8-entry max_val accumulator kept by the tx_to_cb32
    // kernels above (the generic kernels use entry 0 only, so this works
    // for them too); called once per codeblock
    ui32 find_max_val32(ui32* address)
    {
      ui32 t = address[0];
      for (int i = 1; i < 8; ++i)
        t |= address[i];
      return t;
    }

    //////////////////////////////////////////////////////////////////////////
    void rev_tx_from_cb16(const ui32 *sp, si16 *dp, ui32 K_max,
                              ui32 count)
    {
      HWY_DYNAMIC_DISPATCH(rev_tx_from_cb16)(sp, dp, K_max, count);
    }

    //////////////////////////////////////////////////////////////////////////
    void rev_tx_from_cb32(const ui32 *sp, void *dp, ui32 K_max,
                              float delta, ui32 count)
    {
      HWY_DYNAMIC_DISPATCH(rev_tx_from_cb32)(sp, dp, K_max, delta, count);
    }

    //////////////////////////////////////////////////////////////////////////
    void rev_tx_to_cb32(const void *sp, ui32 *dp, ui32 K_max,
                            float delta_inv, ui32 count, ui32* max_val)
    {
      HWY_DYNAMIC_DISPATCH(rev_tx_to_cb32)(sp, dp, K_max, delta_inv,
                                           count, max_val);
    }

    //////////////////////////////////////////////////////////////////////////
    void irv_tx_to_cb32(const void *sp, ui32 *dp, ui32 K_max,
                            float delta_inv, ui32 count, ui32* max_val)
    {
      HWY_DYNAMIC_DISPATCH(irv_tx_to_cb32)(sp, dp, K_max, delta_inv,
                                           count, max_val);
    }

    //////////////////////////////////////////////////////////////////////////
    void irv_tx_from_cb32(const ui32 *sp, void *dp, ui32 K_max,
                              float delta, ui32 count)
    {
      HWY_DYNAMIC_DISPATCH(irv_tx_from_cb32)(sp, dp, K_max, delta, count);
    }

    //////////////////////////////////////////////////////////////////////////
    void rev_convert16(const si16 *sp, si32 *dp, si32 shift, ui32 count)
    {
      HWY_DYNAMIC_DISPATCH(rev_convert16)(sp, dp, shift, count);
    }

  }
}

#endif // HWY_ONCE

#endif // OJPH_ENABLE_HWY
