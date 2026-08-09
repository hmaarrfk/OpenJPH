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
// File: ojph_transform_hwy.cpp
//***************************************************************************/

// Google Highway implementations of the reversible DWT functions.  This
// translation unit is compiled once per Highway target (SSE4, AVX2,
// AVX3, ...) through foreach_target.h, and every installed function
// dispatches to the best target the CPU supports at run time;
// install_rev_transforms() installs the functions only when one of the
// compiled SIMD targets is available.
//
// The functions here compute values identical to those of the generic
// (and SSE2/AVX2) implementations; codestreams they produce are
// byte-identical.  Cases they do not handle (64-bit lines, uncommon
// lifting kernels) are forwarded to the functions that were selected
// before installation.

// Highway must be included before any ojph header, because ojph_defs.h
// renames the ojph namespace token.
#undef HWY_TARGET_INCLUDE
#define HWY_TARGET_INCLUDE "transform/ojph_transform_hwy.cpp"
#include <hwy/foreach_target.h>
#include <hwy/highway.h>

#include "ojph_arch.h"

#ifdef OJPH_ENABLE_HWY

#include <cassert>

#include "ojph_defs.h"
#include "ojph_mem.h"
#include "ojph_params.h"
#include "../codestream/ojph_params_local.h"

#include "ojph_transform.h"
#include "ojph_transform_local.h"

HWY_BEFORE_NAMESPACE();
namespace ojph {
  namespace local {
    namespace HWY_NAMESPACE {

    namespace hn = hwy::HWY_NAMESPACE;

    //////////////////////////////////////////////////////////////////////////
    //
    //                    Vertical lifting steps
    //
    //////////////////////////////////////////////////////////////////////////

    //////////////////////////////////////////////////////////////////////////
    // One vertical lifting pass on 32-bit lines; CASE selects how the
    // update term w is computed (mirroring the cases of
    // gen_rev_vert_step32) and SUB whether it is subtracted from or added
    // to the line being updated; both are compile-time constants, so each
    // instantiation is a single tight loop.
    template <int CASE, bool SUB>
    static
    void rev_vert_pass32(const si32* src1, const si32* src2, si32* dst,
                             ui32 repeat, si32 a, si32 b, ui8 e)
    {
      const hn::ScalableTag<si32> d;
      const ui32 L = (ui32)hn::Lanes(d);
      const auto va = hn::Set(d, a);
      const auto vb = hn::Set(d, b);

      // the loop overruns the line ends by less than one vector; all lines
      // come from mem_fixed_allocator, whose allocations are rounded up
      // and aligned (the SSE2/AVX2 implementations rely on the same)
      for (ui32 i = 0; i < repeat; i += L)
      {
        auto s1 = hn::LoadU(d, src1 + i);
        auto s2 = hn::LoadU(d, src2 + i);
        auto dv = hn::LoadU(d, dst + i);
        auto t = hn::Add(s1, s2);
        hn::Vec<hn::ScalableTag<si32> > w;
        if (CASE == 0)      // 5/3 update and any case with a == 1
          w = hn::ShiftRightSame(hn::Add(vb, t), e);
        else if (CASE == 1) // 5/3 predict
          w = hn::ShiftRight<1>(t);
        else if (CASE == 2) // a == -1, but not 5/3 predict
          w = hn::ShiftRightSame(hn::Sub(vb, t), e);
        else                // general case
          w = hn::ShiftRightSame(hn::Add(vb, hn::Mul(va, t)), e);
        dv = SUB ? hn::Sub(dv, w) : hn::Add(dv, w);
        hn::StoreU(dv, d, dst + i);
      }
    }

    //////////////////////////////////////////////////////////////////////////
    // One vertical lifting step of a whole-sample symmetric kernel on
    // 32-bit lines; the cases mirror those of gen_rev_vert_step32.
    void rev_vert_step32(const lifting_step* s, const si32* src1,
                             const si32* src2, si32* dst, ui32 repeat,
                             bool synthesis)
    {
      const si32 a = s->rev.Aatk;
      const si32 b = s->rev.Batk;
      const ui8 e = s->rev.Eatk;

      if (a == 1) {
        if (synthesis)
          rev_vert_pass32<0, true>(src1, src2, dst, repeat, a, b, e);
        else
          rev_vert_pass32<0, false>(src1, src2, dst, repeat, a, b, e);
      }
      else if (a == -1 && b == 1 && e == 1) {
        if (synthesis)
          rev_vert_pass32<1, false>(src1, src2, dst, repeat, a, b, e);
        else
          rev_vert_pass32<1, true>(src1, src2, dst, repeat, a, b, e);
      }
      else if (a == -1) {
        if (synthesis)
          rev_vert_pass32<2, true>(src1, src2, dst, repeat, a, b, e);
        else
          rev_vert_pass32<2, false>(src1, src2, dst, repeat, a, b, e);
      }
      else {
        if (synthesis)
          rev_vert_pass32<3, true>(src1, src2, dst, repeat, a, b, e);
        else
          rev_vert_pass32<3, false>(src1, src2, dst, repeat, a, b, e);
      }
    }

    //////////////////////////////////////////////////////////////////////////
    // One vertical lifting step of an arbitrary kernel with a single
    // lifting coefficient; the cases mirror gen_rev_vert_step_one_tap_T.
    // Only the cases whose scalar arithmetic is performed in the lane type
    // are implemented; the rest are forwarded (return false), so that
    // results stay identical to the generic implementation.
    template <typename T>
    static
    bool rev_vert_step_one_tap_T(const lifting_step* s, const T* sp,
                                     T* dst, ui32 repeat, bool synthesis)
    {
      const T a = (T)s->rev.Aatk;
      const T b = (T)s->rev.Batk;
      const ui8 e = s->rev.Eatk;

      const hn::ScalableTag<T> d;
      const ui32 L = (ui32)hn::Lanes(d);

      if (a == -1 && b == 0 && e == 0)
      { // previous-sample predict
        if (synthesis)
          for (ui32 i = 0; i < repeat; i += L)
          {
            auto dv = hn::Add(hn::LoadU(d, dst + i), hn::LoadU(d, sp + i));
            hn::StoreU(dv, d, dst + i);
          }
        else
          for (ui32 i = 0; i < repeat; i += L)
          {
            auto dv = hn::Sub(hn::LoadU(d, dst + i), hn::LoadU(d, sp + i));
            hn::StoreU(dv, d, dst + i);
          }
        return true;
      }
      else if (a == 0)
      { // null step; only the constant contributes, if anything
        const T v = (T)(b >> e);
        if (v != 0)
        {
          const auto vv = hn::Set(d, synthesis ? (T)-v : v);
          for (ui32 i = 0; i < repeat; i += L)
            hn::StoreU(hn::Add(hn::LoadU(d, dst + i), vv), d, dst + i);
        }
        return true;
      }
      return false; // general case; forwarded to the fallback
    }

    //////////////////////////////////////////////////////////////////////////
    bool rev_vert_one_tap16(const lifting_step* s, const si16* sp,
                                si16* dst, ui32 repeat, bool synthesis)
    {
      return rev_vert_step_one_tap_T<si16>(s, sp, dst, repeat, synthesis);
    }

    //////////////////////////////////////////////////////////////////////////
    bool rev_vert_one_tap32(const lifting_step* s, const si32* sp,
                                si32* dst, ui32 repeat, bool synthesis)
    {
      return rev_vert_step_one_tap_T<si32>(s, sp, dst, repeat, synthesis);
    }

    //////////////////////////////////////////////////////////////////////////
    //
    //     Fused horizontal transforms for the rev13 kernel (WS)
    //
    //////////////////////////////////////////////////////////////////////////

    //////////////////////////////////////////////////////////////////////////
    // Analysis: deinterleave the input and apply the 5/3 predict step in a
    // single pass; the null update step of the kernel changes nothing.
    // Values are identical to the two-pass (deinterleave, then lift)
    // implementations.
    void rev13_horz_ana32(const si32* sp, si32* lp, si32* hp,
                              ui32 width, bool even)
    {
      const hn::ScalableTag<si32> d;
      const ui32 L = (ui32)hn::Lanes(d);
      hn::Vec<hn::ScalableTag<si32> > e0, o0, e1, o1;

      if (even)
      { // first sample is low-pass;
        // L[i] = x[2i], H[i] = x[2i+1] - ((x[2i] + x[2i+2]) >> 1),
        // where the last x[2i+2] is replaced, for even width, by the
        // constant extension x[width-2]
        ui32 l_width = (width + 1) >> 1;
        ui32 h_width = width >> 1;
        ui32 safe = (width & 1) ? h_width : (h_width ? h_width - 1 : 0);
        ui32 i = 0;
        for (; i + L <= safe; i += L)
        {
          hn::LoadInterleaved2(d, sp + 2 * i, e0, o0);
          hn::LoadInterleaved2(d, sp + 2 * i + 2, e1, o1);
          auto h = hn::Sub(o0, hn::ShiftRight<1>(hn::Add(e0, e1)));
          hn::StoreU(e0, d, lp + i);
          hn::StoreU(h, d, hp + i);
        }
        for (; i < h_width; ++i)
        {
          si32 ev = sp[2 * i], od = sp[2 * i + 1];
          si32 en = (i + 1 < l_width) ? sp[2 * i + 2] : sp[width - 2];
          lp[i] = ev;
          hp[i] = od - ((ev + en) >> 1);
        }
        if (l_width > h_width)
          lp[l_width - 1] = sp[width - 1];
      }
      else
      { // first sample is high-pass;
        // H[0] = x[0] - x[1], L[i-1] = x[2i-1], and
        // H[i] = x[2i] - ((x[2i-1] + x[2i+1]) >> 1), where the last
        // x[2i+1] is replaced, for odd width, by the extension x[width-2]
        ui32 h_width = (width + 1) >> 1;
        ui32 l_width = width >> 1;
        hp[0] = sp[0] - sp[1];
        ui32 safe = (width & 1) ? (h_width - 1) : h_width;
        ui32 i = 1;
        for (; i + L <= safe; i += L)
        {
          hn::LoadInterleaved2(d, sp + 2 * i - 1, e0, o0);
          hn::LoadInterleaved2(d, sp + 2 * i + 1, e1, o1);
          auto h = hn::Sub(o0, hn::ShiftRight<1>(hn::Add(e0, e1)));
          hn::StoreU(e0, d, lp + i - 1);
          hn::StoreU(h, d, hp + i);
        }
        for (; i < h_width; ++i)
        {
          si32 l0 = sp[2 * i - 1];
          si32 l1 = (i < l_width) ? sp[2 * i + 1] : sp[width - 2];
          lp[i - 1] = l0;
          hp[i] = sp[2 * i] - ((l0 + l1) >> 1);
        }
        if ((width & 1) == 0)
          lp[l_width - 1] = sp[width - 1];
      }
    }

    //////////////////////////////////////////////////////////////////////////
    // Synthesis: apply the inverse 5/3 predict step and interleave into
    // the output in a single pass; the inverse of rev13_horz_ana32.
    void rev13_horz_syn32(si32* dp, const si32* lp, const si32* hp,
                              ui32 width, bool even)
    {
      const hn::ScalableTag<si32> d;
      const ui32 L = (ui32)hn::Lanes(d);

      if (even)
      { // x[2i] = L[i], x[2i+1] = H[i] + ((L[i] + L[i+1]) >> 1)
        ui32 l_width = (width + 1) >> 1;
        ui32 h_width = width >> 1;
        ui32 safe = (width & 1) ? h_width : (h_width ? h_width - 1 : 0);
        ui32 i = 0;
        for (; i + L <= safe; i += L)
        {
          auto l0 = hn::LoadU(d, lp + i);
          auto l1 = hn::LoadU(d, lp + i + 1);
          auto h = hn::LoadU(d, hp + i);
          auto od = hn::Add(h, hn::ShiftRight<1>(hn::Add(l0, l1)));
          hn::StoreInterleaved2(l0, od, d, dp + 2 * i);
        }
        for (; i < h_width; ++i)
        {
          si32 l0 = lp[i];
          si32 l1 = (i + 1 < l_width) ? lp[i + 1] : lp[l_width - 1];
          dp[2 * i] = l0;
          dp[2 * i + 1] = hp[i] + ((l0 + l1) >> 1);
        }
        if (l_width > h_width)
          dp[width - 1] = lp[l_width - 1];
      }
      else
      { // x[0] = H[0] + L[0], x[2i-1] = L[i-1], and
        // x[2i] = H[i] + ((L[i-1] + L[i]) >> 1)
        ui32 h_width = (width + 1) >> 1;
        ui32 l_width = width >> 1;
        dp[0] = hp[0] + lp[0];
        ui32 safe = (width & 1) ? (h_width - 1) : h_width;
        ui32 i = 1;
        for (; i + L <= safe; i += L)
        {
          auto l0 = hn::LoadU(d, lp + i - 1);
          auto l1 = hn::LoadU(d, lp + i);
          auto h = hn::LoadU(d, hp + i);
          auto od = hn::Add(h, hn::ShiftRight<1>(hn::Add(l0, l1)));
          hn::StoreInterleaved2(l0, od, d, dp + 2 * i - 1);
        }
        for (; i < h_width; ++i)
        {
          si32 l0 = lp[i - 1];
          si32 l1 = (i < l_width) ? lp[i] : lp[l_width - 1];
          dp[2 * i - 1] = l0;
          dp[2 * i] = hp[i] + ((l0 + l1) >> 1);
        }
        if ((width & 1) == 0)
          dp[width - 1] = lp[l_width - 1];
      }
    }

    //////////////////////////////////////////////////////////////////////////
    //
    //     Fused horizontal transforms for the rev53 kernel (WS)
    //
    //////////////////////////////////////////////////////////////////////////

    //////////////////////////////////////////////////////////////////////////
    // returns [prev[L-1], v[0], ..., v[L-2]], i.e. v shifted up one lane
    // with the last lane of prev shifted in
    static inline hn::Vec<hn::ScalableTag<si32> >
    shift_in_prev(hn::Vec<hn::ScalableTag<si32> > v,
                      hn::Vec<hn::ScalableTag<si32> > prev)
    {
      const hn::ScalableTag<si32> d;
#if HWY_TARGET == HWY_AVX2
      // [prev[4..7], v[0..3]], then a per-block byte-wise combine
      auto t = hn::ConcatLowerUpper(d, v, prev);
      return hn::CombineShiftRightBytes<16 - sizeof(si32)>(d, v, t);
#else
      const size_t L = hn::Lanes(d);
      auto last = hn::Set(d, hn::ExtractLane(prev, L - 1));
      return hn::IfThenElse(hn::FirstN(d, 1), last, hn::Slide1Up(d, v));
#endif
    }

    //////////////////////////////////////////////////////////////////////////
    // Analysis: deinterleave the input and apply the 5/3 predict and
    // update steps in a single pass; the update term H[i-1] is obtained
    // by combining the previous block's H values with the current ones
    // (lane 0 of the first block gets the boundary extension), so values
    // are identical to the two-pass implementations.
    void rev53_horz_ana32(const si32* sp, si32* lp, si32* hp,
                              ui32 width, bool even)
    {
      const hn::ScalableTag<si32> d;
      const ui32 L = (ui32)hn::Lanes(d);
      const auto two = hn::Set(d, 2);
      hn::Vec<hn::ScalableTag<si32> > e0, o0, e1, o1, prev;

      if (even)
      { // first sample is low-pass;
        // H[i] = x[2i+1] - ((x[2i] + x[2i+2]) >> 1), then
        // L[i] = x[2i] + ((H[i-1] + H[i] + 2) >> 2), with the constant
        // extensions x[2i+2] -> x[width-2], H[-1] -> H[0], and
        // H[h_width] -> H[h_width-1] at the boundaries
        ui32 l_width = (width + 1) >> 1;
        ui32 h_width = width >> 1;
        ui32 safe = (width & 1) ? h_width : (h_width ? h_width - 1 : 0);
        ui32 i = 0;
        if (L <= safe)
        { // first block; H[-1] = H[0], older lanes are H[k - 1]
          hn::LoadInterleaved2(d, sp, e0, o0);
          hn::LoadInterleaved2(d, sp + 2, e1, o1);
          auto h = hn::Sub(o0, hn::ShiftRight<1>(hn::Add(e0, e1)));
          auto hm1 =
            hn::IfThenElse(hn::FirstN(d, 1), h, hn::Slide1Up(d, h));
          auto l = hn::Add(e0,
            hn::ShiftRight<2>(hn::Add(hn::Add(hm1, h), two)));
          hn::StoreU(l, d, lp);
          hn::StoreU(h, d, hp);
          prev = h;
          for (i = L; i + L <= safe; i += L)
          {
            hn::LoadInterleaved2(d, sp + 2 * i, e0, o0);
            hn::LoadInterleaved2(d, sp + 2 * i + 2, e1, o1);
            h = hn::Sub(o0, hn::ShiftRight<1>(hn::Add(e0, e1)));
            hm1 = shift_in_prev(h, prev);
            l = hn::Add(e0,
              hn::ShiftRight<2>(hn::Add(hn::Add(hm1, h), two)));
            hn::StoreU(l, d, lp + i);
            hn::StoreU(h, d, hp + i);
            prev = h;
          }
        }
        for (; i < h_width; ++i)
        {
          si32 ev = sp[2 * i], od = sp[2 * i + 1];
          si32 en = (i + 1 < l_width) ? sp[2 * i + 2] : sp[width - 2];
          si32 h = od - ((ev + en) >> 1);
          si32 hm1 = (i > 0) ? hp[i - 1] : h;
          hp[i] = h;
          lp[i] = ev + ((hm1 + h + 2) >> 2);
        }
        if (l_width > h_width)
          lp[l_width - 1] =
            sp[width - 1] + ((2 * hp[h_width - 1] + 2) >> 2);
      }
      else
      { // first sample is high-pass;
        // H[0] = x[0] - x[1], H[i] = x[2i] - ((x[2i-1] + x[2i+1]) >> 1),
        // then L[i-1] = x[2i-1] + ((H[i-1] + H[i] + 2) >> 2), with the
        // extensions x[2i+1] -> x[width-2] and H[h_width] -> H[h_width-1]
        ui32 h_width = (width + 1) >> 1;
        ui32 l_width = width >> 1;
        hp[0] = sp[0] - sp[1];
        ui32 safe = (width & 1) ? (h_width - 1) : h_width;
        prev = hn::Set(d, hp[0]); // only its last lane is used
        ui32 i = 1;
        for (; i + L <= safe; i += L)
        {
          hn::LoadInterleaved2(d, sp + 2 * i - 1, e0, o0);
          hn::LoadInterleaved2(d, sp + 2 * i + 1, e1, o1);
          auto h = hn::Sub(o0, hn::ShiftRight<1>(hn::Add(e0, e1)));
          auto hm1 = shift_in_prev(h, prev);
          auto l = hn::Add(e0,
            hn::ShiftRight<2>(hn::Add(hn::Add(hm1, h), two)));
          hn::StoreU(l, d, lp + i - 1);
          hn::StoreU(h, d, hp + i);
          prev = h;
        }
        for (; i < h_width; ++i)
        {
          si32 l0 = sp[2 * i - 1];
          si32 l1 = (i < l_width) ? sp[2 * i + 1] : sp[width - 2];
          si32 h = sp[2 * i] - ((l0 + l1) >> 1);
          hp[i] = h;
          lp[i - 1] = l0 + ((hp[i - 1] + h + 2) >> 2);
        }
        if ((width & 1) == 0)
          lp[l_width - 1] =
            sp[width - 1] + ((2 * hp[h_width - 1] + 2) >> 2);
      }
    }

    //////////////////////////////////////////////////////////////////////////
    // Synthesis: apply the inverse 5/3 update and predict steps and
    // interleave into the output in a single pass; the inverse of
    // rev53_horz_ana32.  The updated low-pass values L' are
    // recomputed where a lane needs its neighbour, so values are
    // identical to the two-pass implementations.
    void rev53_horz_syn32(si32* dp, const si32* lp, const si32* hp,
                              ui32 width, bool even)
    {
      const hn::ScalableTag<si32> d;
      const ui32 L = (ui32)hn::Lanes(d);
      const auto two = hn::Set(d, 2);

      if (even)
      { // L'[i] = L[i] - ((H[i-1] + H[i] + 2) >> 2), then
        // x[2i] = L'[i], x[2i+1] = H[i] + ((L'[i] + L'[i+1]) >> 1)
        ui32 l_width = (width + 1) >> 1;
        ui32 h_width = width >> 1;
        ui32 i = 0;
        if (h_width > 0)
        {
          for (; i + L + 1 <= h_width; i += L)
          {
            auto lv = hn::LoadU(d, lp + i);
            auto hv = hn::LoadU(d, hp + i);
            hn::Vec<hn::ScalableTag<si32> > hm1;
            if (i == 0) // H[-1] = H[0]; older lanes are H[k - 1]
              hm1 = hn::IfThenElse(hn::FirstN(d, 1), hv,
                                   hn::Slide1Up(d, hv));
            else
              hm1 = hn::LoadU(d, hp + i - 1);
            auto l0 = hn::Sub(lv,
              hn::ShiftRight<2>(hn::Add(hn::Add(hm1, hv), two)));
            // recompute L'[i+1 ..]
            auto lv1 = hn::LoadU(d, lp + i + 1);
            auto hv1 = hn::LoadU(d, hp + i + 1);
            auto l1 = hn::Sub(lv1,
              hn::ShiftRight<2>(hn::Add(hn::Add(hv, hv1), two)));
            auto od = hn::Add(hv, hn::ShiftRight<1>(hn::Add(l0, l1)));
            hn::StoreInterleaved2(l0, od, d, dp + 2 * i);
          }
          for (; i < h_width; ++i)
          {
            si32 hm1 = (i > 0) ? hp[i - 1] : hp[0];
            si32 l0 = lp[i] - ((hm1 + hp[i] + 2) >> 2);
            si32 l1;
            if (i + 1 < l_width)
            {
              si32 h1 = (i + 1 < h_width) ? hp[i + 1] : hp[h_width - 1];
              l1 = lp[i + 1] - ((hp[i] + h1 + 2) >> 2);
            }
            else
              l1 = l0; // L'[l_width] -> L'[l_width-1]
            dp[2 * i] = l0;
            dp[2 * i + 1] = hp[i] + ((l0 + l1) >> 1);
          }
        }
        if (l_width > h_width)
          dp[width - 1] =
            lp[l_width - 1] - ((2 * hp[h_width - 1] + 2) >> 2);
      }
      else
      { // L'[i] = L[i] - ((H[i] + H[i+1] + 2) >> 2), then x[0] = H[0] +
        // L'[0], x[2i-1] = L'[i-1], and
        // x[2i] = H[i] + ((L'[i-1] + L'[i]) >> 1)
        ui32 h_width = (width + 1) >> 1;
        ui32 l_width = width >> 1;
        // L'[j] with the extensions H[h_width] -> H[h_width-1] and
        // L'[l_width] -> L'[l_width-1]
        const si32 h_last = hp[h_width - 1];
        si32 lpr0; // L'[0]
        {
          si32 h1 = (1 < h_width) ? hp[1] : h_last;
          lpr0 = lp[0] - ((hp[0] + h1 + 2) >> 2);
        }
        dp[0] = hp[0] + lpr0;
        ui32 i = 1;
        for (; i + L + 1 <= h_width; i += L)
        {
          auto hv0 = hn::LoadU(d, hp + i - 1);
          auto hv = hn::LoadU(d, hp + i);
          auto hv1 = hn::LoadU(d, hp + i + 1);
          auto lm1 = hn::Sub(hn::LoadU(d, lp + i - 1),
            hn::ShiftRight<2>(hn::Add(hn::Add(hv0, hv), two)));
          auto l0 = hn::Sub(hn::LoadU(d, lp + i),
            hn::ShiftRight<2>(hn::Add(hn::Add(hv, hv1), two)));
          auto ev = hn::Add(hv, hn::ShiftRight<1>(hn::Add(lm1, l0)));
          hn::StoreInterleaved2(lm1, ev, d, dp + 2 * i - 1);
        }
        for (; i < h_width; ++i)
        {
          si32 lm1 = lp[i - 1] -
            ((hp[i - 1] + hp[i] + 2) >> 2); // L'[i-1]; i - 1 < l_width
          si32 l0;
          if (i < l_width)
          {
            si32 h1 = (i + 1 < h_width) ? hp[i + 1] : h_last;
            l0 = lp[i] - ((hp[i] + h1 + 2) >> 2);
          }
          else
            l0 = lm1; // L'[l_width] -> L'[l_width-1]
          dp[2 * i - 1] = lm1;
          dp[2 * i] = hp[i] + ((lm1 + l0) >> 1);
        }
        if ((width & 1) == 0)
          dp[width - 1] =
            lp[l_width - 1] - ((hp[h_width - 1] + h_last + 2) >> 2);
      }
    }

    //////////////////////////////////////////////////////////////////////////
    //
    //  Horizontal transforms for general whole-sample symmetric kernels
    //  on 32-bit lines: one deinterleave/interleave pass and one pass per
    //  lifting step, mirroring gen_rev_horz_ana32/gen_rev_horz_syn32
    //
    //////////////////////////////////////////////////////////////////////////

    //////////////////////////////////////////////////////////////////////////
    // One horizontal lifting pass; the two taps for dp[i] are sp[i - 1]
    // and sp[i], and CASE/SUB are as in rev_vert_pass32.  The loop
    // overruns count to whole vectors; the extension slots and the line
    // padding absorb the overrun (as in the SSE2/AVX2 implementations).
    template <int CASE, bool SUB>
    static
    void rev_horz_pass32(const si32* sp, si32* dp, ui32 count,
                             si32 a, si32 b, ui8 e)
    {
      const hn::ScalableTag<si32> d;
      const ui32 L = (ui32)hn::Lanes(d);
      const auto va = hn::Set(d, a);
      const auto vb = hn::Set(d, b);
      for (ui32 i = 0; i < count; i += L)
      {
        auto s1 = hn::LoadU(d, sp + i - 1);
        auto s2 = hn::LoadU(d, sp + i);
        auto dv = hn::LoadU(d, dp + i);
        auto t = hn::Add(s1, s2);
        hn::Vec<hn::ScalableTag<si32> > w;
        if (CASE == 0)      // 5/3 update and any case with a == 1
          w = hn::ShiftRightSame(hn::Add(vb, t), e);
        else if (CASE == 1) // 5/3 predict
          w = hn::ShiftRight<1>(t);
        else if (CASE == 2) // a == -1, but not 5/3 predict
          w = hn::ShiftRightSame(hn::Sub(vb, t), e);
        else                // general case
          w = hn::ShiftRightSame(hn::Add(vb, hn::Mul(va, t)), e);
        dv = SUB ? hn::Sub(dv, w) : hn::Add(dv, w);
        hn::StoreU(dv, d, dp + i);
      }
    }

    //////////////////////////////////////////////////////////////////////////
    // Analysis; values identical to gen_rev_horz_ana32
    void rev_horz_ws_ana32(const param_atk* atk, const line_buf* ldst,
                               const line_buf* hdst, const line_buf* src,
                               ui32 width, bool even)
    {
      const hn::ScalableTag<si32> d;
      const ui32 L = (ui32)hn::Lanes(d);

      // split src into ldst and hdst
      {
        si32* dpl = even ? ldst->i32 : hdst->i32;
        si32* dph = even ? hdst->i32 : ldst->i32;
        const si32* sp = src->i32;
        hn::Vec<hn::ScalableTag<si32> > ev, od;
        const ui32 half = (width + 1) >> 1;
        for (ui32 i = 0; i < half; i += L)
        {
          hn::LoadInterleaved2(d, sp + 2 * i, ev, od);
          hn::StoreU(ev, d, dpl + i);
          hn::StoreU(od, d, dph + i);
        }
      }

      si32* hp = hdst->i32, * lp = ldst->i32;
      ui32 l_width = (width + (even ? 1 : 0)) >> 1;  // low pass
      ui32 h_width = (width + (even ? 0 : 1)) >> 1;  // high pass
      ui32 num_steps = atk->get_num_steps();
      for (ui32 j = num_steps; j > 0; --j)
      {
        const lifting_step* s = atk->get_step(j - 1);
        const si32 a = s->rev.Aatk;
        const si32 b = s->rev.Batk;
        const ui8 e = s->rev.Eatk;

        // extension
        lp[-1] = lp[0];
        lp[l_width] = lp[l_width - 1];
        // lifting step
        const si32* sp = lp + (even ? 1 : 0);
        if (a == 1)
          rev_horz_pass32<0, false>(sp, hp, h_width, a, b, e);
        else if (a == -1 && b == 1 && e == 1)
          rev_horz_pass32<1, true>(sp, hp, h_width, a, b, e);
        else if (a == -1)
          rev_horz_pass32<2, false>(sp, hp, h_width, a, b, e);
        else
          rev_horz_pass32<3, false>(sp, hp, h_width, a, b, e);

        // swap buffers
        si32* t = lp; lp = hp; hp = t;
        even = !even;
        ui32 w = l_width; l_width = h_width; h_width = w;
      }
    }

    //////////////////////////////////////////////////////////////////////////
    // Synthesis; values identical to gen_rev_horz_syn32
    void rev_horz_ws_syn32(const param_atk* atk, const line_buf* dst,
                               const line_buf* lsrc, const line_buf* hsrc,
                               ui32 width, bool even)
    {
      const hn::ScalableTag<si32> d;
      const ui32 L = (ui32)hn::Lanes(d);

      bool ev = even;
      si32* oth = hsrc->i32, * aug = lsrc->i32;
      ui32 aug_width = (width + (even ? 1 : 0)) >> 1;  // low pass
      ui32 oth_width = (width + (even ? 0 : 1)) >> 1;  // high pass
      ui32 num_steps = atk->get_num_steps();
      for (ui32 j = 0; j < num_steps; ++j)
      {
        const lifting_step* s = atk->get_step(j);
        const si32 a = s->rev.Aatk;
        const si32 b = s->rev.Batk;
        const ui8 e = s->rev.Eatk;

        // extension
        oth[-1] = oth[0];
        oth[oth_width] = oth[oth_width - 1];
        // lifting step
        const si32* sp = oth + (ev ? 0 : 1);
        if (a == 1)
          rev_horz_pass32<0, true>(sp, aug, aug_width, a, b, e);
        else if (a == -1 && b == 1 && e == 1)
          rev_horz_pass32<1, false>(sp, aug, aug_width, a, b, e);
        else if (a == -1)
          rev_horz_pass32<2, true>(sp, aug, aug_width, a, b, e);
        else
          rev_horz_pass32<3, true>(sp, aug, aug_width, a, b, e);

        // swap buffers
        si32* t = aug; aug = oth; oth = t;
        ev = !ev;
        ui32 w = aug_width; aug_width = oth_width; oth_width = w;
      }

      // combine both lsrc and hsrc into dst
      {
        si32* dp = dst->i32;
        const si32* spl = even ? lsrc->i32 : hsrc->i32;
        const si32* sph = even ? hsrc->i32 : lsrc->i32;
        const ui32 half = (width + 1) >> 1;
        for (ui32 i = 0; i < half; i += L)
        {
          auto l = hn::LoadU(d, spl + i);
          auto h = hn::LoadU(d, sph + i);
          hn::StoreInterleaved2(l, h, d, dp + 2 * i);
        }
      }
    }

    //////////////////////////////////////////////////////////////////////////
    //
    //   Fused horizontal transforms for the previous-sample kernel (ARB)
    //
    //////////////////////////////////////////////////////////////////////////

    //////////////////////////////////////////////////////////////////////////
    // Analysis; values identical to gen_rev_horz_ana_prev_T.
    template <typename T>
    static
    void rev_horz_ana_prev_T(T* lp, T* hp, const T* sp,
                                 ui32 width, bool even)
    {
      const hn::ScalableTag<T> d;
      const ui32 L = (ui32)hn::Lanes(d);
      hn::Vec<hn::ScalableTag<T> > ev, od;

      if (even)
      { // L[i] = x[2i], H[i] = x[2i+1] - x[2i]
        ui32 l_width = (width + 1) >> 1;
        ui32 h_width = width >> 1;
        ui32 i = 0;
        for (; i + L <= h_width; i += L)
        {
          hn::LoadInterleaved2(d, sp + 2 * i, ev, od);
          hn::StoreU(ev, d, lp + i);
          hn::StoreU(hn::Sub(od, ev), d, hp + i);
        }
        for (; i < h_width; ++i)
        {
          lp[i] = sp[2 * i];
          hp[i] = (T)(sp[2 * i + 1] - sp[2 * i]);
        }
        if (l_width > h_width)
          lp[l_width - 1] = sp[width - 1];
      }
      else
      { // H[0] = x[0] - x[1], L[i-1] = x[2i-1], H[i] = x[2i] - x[2i-1]
        ui32 h_width = (width + 1) >> 1;
        hp[0] = (T)(sp[0] - sp[1]);
        ui32 i = 1;
        for (; i + L <= h_width; i += L)
        {
          hn::LoadInterleaved2(d, sp + 2 * i - 1, ev, od);
          hn::StoreU(ev, d, lp + i - 1);
          hn::StoreU(hn::Sub(od, ev), d, hp + i);
        }
        for (; i < h_width; ++i)
        {
          lp[i - 1] = sp[2 * i - 1];
          hp[i] = (T)(sp[2 * i] - sp[2 * i - 1]);
        }
        if ((width & 1) == 0)
          lp[(width >> 1) - 1] = sp[width - 1];
      }
    }

    //////////////////////////////////////////////////////////////////////////
    // Synthesis; values identical to gen_rev_horz_syn_prev_T.
    template <typename T>
    static
    void rev_horz_syn_prev_T(T* dp, const T* lp, const T* hp,
                                 ui32 width, bool even)
    {
      const hn::ScalableTag<T> d;
      const ui32 L = (ui32)hn::Lanes(d);

      if (even)
      { // x[2i] = L[i], x[2i+1] = H[i] + L[i]
        ui32 l_width = (width + 1) >> 1;
        ui32 h_width = width >> 1;
        ui32 i = 0;
        for (; i + L <= h_width; i += L)
        {
          auto l = hn::LoadU(d, lp + i);
          auto h = hn::LoadU(d, hp + i);
          hn::StoreInterleaved2(l, hn::Add(h, l), d, dp + 2 * i);
        }
        for (; i < h_width; ++i)
        {
          dp[2 * i] = lp[i];
          dp[2 * i + 1] = (T)(hp[i] + lp[i]);
        }
        if (l_width > h_width)
          dp[width - 1] = lp[l_width - 1];
      }
      else
      { // x[0] = H[0] + L[0], x[2i-1] = L[i-1], x[2i] = H[i] + L[i-1]
        ui32 h_width = (width + 1) >> 1;
        dp[0] = (T)(hp[0] + lp[0]);
        ui32 i = 1;
        for (; i + L <= h_width; i += L)
        {
          auto l = hn::LoadU(d, lp + i - 1);
          auto h = hn::LoadU(d, hp + i);
          hn::StoreInterleaved2(l, hn::Add(h, l), d, dp + 2 * i - 1);
        }
        for (; i < h_width; ++i)
        {
          dp[2 * i - 1] = lp[i - 1];
          dp[2 * i] = (T)(hp[i] + lp[i - 1]);
        }
        if ((width & 1) == 0)
          dp[width - 1] = lp[(width >> 1) - 1];
      }
    }

    //////////////////////////////////////////////////////////////////////////
    void rev_horz_ana_prev16(si16* lp, si16* hp, const si16* sp,
                                 ui32 width, bool even)
    {
      rev_horz_ana_prev_T<si16>(lp, hp, sp, width, even);
    }

    //////////////////////////////////////////////////////////////////////////
    void rev_horz_ana_prev32(si32* lp, si32* hp, const si32* sp,
                                 ui32 width, bool even)
    {
      rev_horz_ana_prev_T<si32>(lp, hp, sp, width, even);
    }

    //////////////////////////////////////////////////////////////////////////
    void rev_horz_syn_prev16(si16* dp, const si16* lp, const si16* hp,
                                 ui32 width, bool even)
    {
      rev_horz_syn_prev_T<si16>(dp, lp, hp, width, even);
    }

    //////////////////////////////////////////////////////////////////////////
    void rev_horz_syn_prev32(si32* dp, const si32* lp, const si32* hp,
                                 ui32 width, bool even)
    {
      rev_horz_syn_prev_T<si32>(dp, lp, hp, width, even);
    }

    //////////////////////////////////////////////////////////////////////////
    //
    //           Irreversible (9/7 float) transform functions
    //
    //////////////////////////////////////////////////////////////////////////

    //////////////////////////////////////////////////////////////////////////
    // multiply a line by a constant factor; like the lifting loops below,
    // the loop overruns the line end by less than one vector
    static inline void multiply_const(float* p, float f, ui32 width)
    {
      const hn::ScalableTag<float> d;
      const ui32 L = (ui32)hn::Lanes(d);
      const auto vf = hn::Set(d, f);
      for (ui32 i = 0; i < width; i += L)
        hn::StoreU(hn::Mul(vf, hn::LoadU(d, p + i)), d, p + i);
    }

    //////////////////////////////////////////////////////////////////////////
    // One vertical lifting step; values identical to gen_irv_vert_step
    // (this file is compiled with -ffp-contract=off, so Mul and Add are
    // not fused, keeping results identical to the generic and SSE/AVX
    // implementations)
    void simd_irv_vert_step(const lifting_step* s, const line_buf* sig,
                           const line_buf* other, const line_buf* aug,
                           ui32 repeat, bool synthesis)
    {
      float a = s->irv.Aatk;
      if (synthesis)
        a = -a;

      const hn::ScalableTag<float> d;
      const ui32 L = (ui32)hn::Lanes(d);
      const auto va = hn::Set(d, a);

      float* dst = aug->f32;
      const float* src1 = sig->f32, * src2 = other->f32;
      for (ui32 i = 0; i < repeat; i += L)
      {
        auto s1 = hn::LoadU(d, src1 + i);
        auto s2 = hn::LoadU(d, src2 + i);
        auto dv = hn::LoadU(d, dst + i);
        dv = hn::Add(dv, hn::Mul(va, hn::Add(s1, s2)));
        hn::StoreU(dv, d, dst + i);
      }
    }

    //////////////////////////////////////////////////////////////////////////
    void simd_irv_vert_times_K(float K, const line_buf* aug, ui32 repeat)
    {
      multiply_const(aug->f32, K, repeat);
    }

    //////////////////////////////////////////////////////////////////////////
    // Analysis; the structure mirrors gen_irv_horz_ana (deinterleave, one
    // pass per lifting step, then scale by K); values are identical
    void simd_irv_horz_ana(const param_atk* atk, const line_buf* ldst,
                          const line_buf* hdst, const line_buf* src,
                          ui32 width, bool even)
    {
      if (width > 1)
      {
        const hn::ScalableTag<float> d;
        const ui32 L = (ui32)hn::Lanes(d);

        // split src into ldst and hdst
        {
          float* dpl = even ? ldst->f32 : hdst->f32;
          float* dph = even ? hdst->f32 : ldst->f32;
          const float* sp = src->f32;
          hn::Vec<hn::ScalableTag<float> > ev, od;
          const ui32 half = (width + 1) >> 1;
          for (ui32 i = 0; i < half; i += L)
          {
            hn::LoadInterleaved2(d, sp + 2 * i, ev, od);
            hn::StoreU(ev, d, dpl + i);
            hn::StoreU(od, d, dph + i);
          }
        }

        // the actual horizontal transform
        float* hp = hdst->f32, * lp = ldst->f32;
        ui32 l_width = (width + (even ? 1 : 0)) >> 1;  // low pass
        ui32 h_width = (width + (even ? 0 : 1)) >> 1;  // high pass
        ui32 num_steps = atk->get_num_steps();
        for (ui32 j = num_steps; j > 0; --j)
        {
          const lifting_step* s = atk->get_step(j - 1);
          const auto va = hn::Set(d, s->irv.Aatk);

          // extension
          lp[-1] = lp[0];
          lp[l_width] = lp[l_width - 1];
          // lifting step
          const float* sp = lp + (even ? 1 : 0);
          float* dp = hp;
          for (ui32 i = 0; i < h_width; i += L)
          {
            auto m = hn::LoadU(d, sp + i - 1);
            auto n = hn::LoadU(d, sp + i);
            auto p = hn::LoadU(d, dp + i);
            p = hn::Add(p, hn::Mul(va, hn::Add(m, n)));
            hn::StoreU(p, d, dp + i);
          }

          // swap buffers
          float* t = lp; lp = hp; hp = t;
          even = !even;
          ui32 w = l_width; l_width = h_width; h_width = w;
        }

        { // multiply by K or 1/K
          float K = atk->get_K();
          multiply_const(lp, 1.0f / K, l_width);
          multiply_const(hp, K, h_width);
        }
      }
      else {
        if (even)
          ldst->f32[0] = src->f32[0];
        else
          hdst->f32[0] = src->f32[0] * 2.0f;
      }
    }

    //////////////////////////////////////////////////////////////////////////
    // Synthesis; the inverse of simd_irv_horz_ana, mirroring
    // gen_irv_horz_syn
    void simd_irv_horz_syn(const param_atk* atk, const line_buf* dst,
                          const line_buf* lsrc, const line_buf* hsrc,
                          ui32 width, bool even)
    {
      if (width > 1)
      {
        const hn::ScalableTag<float> d;
        const ui32 L = (ui32)hn::Lanes(d);
        bool ev = even;
        float* oth = hsrc->f32, * aug = lsrc->f32;
        ui32 aug_width = (width + (even ? 1 : 0)) >> 1;  // low pass
        ui32 oth_width = (width + (even ? 0 : 1)) >> 1;  // high pass

        { // multiply by K or 1/K
          float K = atk->get_K();
          multiply_const(aug, K, aug_width);
          multiply_const(oth, 1.0f / K, oth_width);
        }

        ui32 num_steps = atk->get_num_steps();
        for (ui32 j = 0; j < num_steps; ++j)
        {
          const lifting_step* s = atk->get_step(j);
          const auto va = hn::Set(d, s->irv.Aatk);

          // extension
          oth[-1] = oth[0];
          oth[oth_width] = oth[oth_width - 1];
          // lifting step
          const float* sp = oth + (ev ? 0 : 1);
          float* dp = aug;
          for (ui32 i = 0; i < aug_width; i += L)
          {
            auto m = hn::LoadU(d, sp + i - 1);
            auto n = hn::LoadU(d, sp + i);
            auto p = hn::LoadU(d, dp + i);
            p = hn::Sub(p, hn::Mul(va, hn::Add(m, n)));
            hn::StoreU(p, d, dp + i);
          }

          // swap buffers
          float* t = aug; aug = oth; oth = t;
          ev = !ev;
          ui32 w = aug_width; aug_width = oth_width; oth_width = w;
        }

        // combine both lsrc and hsrc into dst
        {
          float* dp = dst->f32;
          const float* spl = even ? lsrc->f32 : hsrc->f32;
          const float* sph = even ? hsrc->f32 : lsrc->f32;
          const ui32 half = (width + 1) >> 1;
          for (ui32 i = 0; i < half; i += L)
          {
            auto l = hn::LoadU(d, spl + i);
            auto h = hn::LoadU(d, sph + i);
            hn::StoreInterleaved2(l, h, d, dp + 2 * i);
          }
        }
      }
      else {
        if (even)
          dst->f32[0] = lsrc->f32[0];
        else
          dst->f32[0] = hsrc->f32[0] * 0.5f;
      }
    }

    } // !HWY_NAMESPACE namespace
  } // !local namespace
} // !ojph namespace
HWY_AFTER_NAMESPACE();

#if HWY_ONCE

namespace ojph {
  namespace local {

    HWY_EXPORT(rev_vert_step32);
    HWY_EXPORT(rev_vert_one_tap16);
    HWY_EXPORT(rev_vert_one_tap32);
    HWY_EXPORT(rev13_horz_ana32);
    HWY_EXPORT(rev13_horz_syn32);
    HWY_EXPORT(rev53_horz_ana32);
    HWY_EXPORT(rev53_horz_syn32);
    HWY_EXPORT(rev_horz_ws_ana32);
    HWY_EXPORT(rev_horz_ws_syn32);
    HWY_EXPORT(rev_horz_ana_prev16);
    HWY_EXPORT(rev_horz_ana_prev32);
    HWY_EXPORT(rev_horz_syn_prev16);
    HWY_EXPORT(rev_horz_syn_prev32);
    HWY_EXPORT(simd_irv_vert_step);
    HWY_EXPORT(simd_irv_vert_times_K);
    HWY_EXPORT(simd_irv_horz_ana);
    HWY_EXPORT(simd_irv_horz_syn);

    //////////////////////////////////////////////////////////////////////////
    // The functions selected before Highway installation; used for the
    // cases the Highway implementations do not handle.
    static void (*fb_rev_vert_step)
      (const lifting_step* s, const line_buf* sig, const line_buf* other,
        const line_buf* aug, ui32 repeat, bool synthesis) = NULL;
    static void (*fb_rev_horz_ana)
      (const param_atk* atk, const line_buf* ldst, const line_buf* hdst,
        const line_buf* src, ui32 width, bool even) = NULL;
    static void (*fb_rev_horz_syn)
      (const param_atk* atk, const line_buf* dst, const line_buf* lsrc,
        const line_buf* hsrc, ui32 width, bool even) = NULL;
    static void (*fb_rev_vert_step_one_tap)
      (const lifting_step* s, const line_buf* src, const line_buf* aug,
        ui32 repeat, bool synthesis) = NULL;
    static void (*fb_rev_horz_ana_arb)
      (const param_atk* atk, const line_buf* ldst, const line_buf* hdst,
        const line_buf* src, ui32 width, bool even) = NULL;
    static void (*fb_rev_horz_syn_arb)
      (const param_atk* atk, const line_buf* dst, const line_buf* lsrc,
        const line_buf* hsrc, ui32 width, bool even) = NULL;

    //////////////////////////////////////////////////////////////////////////
    // True when the lifting step is a no-op; it adds
    // (Batk + Aatk * x) >> Eatk, which is identically zero when Aatk == 0
    // and Batk >> Eatk == 0.
    static inline bool is_null_step(const lifting_step* s)
    {
      return s->rev.Aatk == 0 && (s->rev.Batk >> s->rev.Eatk) == 0;
    }

    //////////////////////////////////////////////////////////////////////////
    // True when the kernel is the 5/3 predict step (see the generic
    // implementations in ojph_transform.cpp).
    static inline bool is_53_predict_step(const lifting_step* s)
    {
      return s->rev.Aatk == -1 && s->rev.Batk == 1 && s->rev.Eatk == 1;
    }

    //////////////////////////////////////////////////////////////////////////
    // True when the kernel is the two-step previous-sample predict-only
    // kernel; same as is_fused_prev_sample_kernel in ojph_transform.cpp.
    static inline bool is_prev_sample_kernel(const param_atk* atk)
    {
      if (atk->get_num_steps() != 2)
        return false;
      const lifting_step* s1 = atk->get_step(1);
      return is_null_step(atk->get_step(0)) &&
             s1->rev.Aatk == -1 && s1->rev.Batk == 0 && s1->rev.Eatk == 0 &&
             s1->rev.Oatk == 0;
    }

    //////////////////////////////////////////////////////////////////////////
    // True when the kernel is the rev13 kernel (a null update step then
    // the 5/3 predict step), for which fused single-pass horizontal
    // transforms are provided above.
    static inline bool is_rev13_kernel(const param_atk* atk)
    {
      return atk->get_num_steps() == 2 && is_null_step(atk->get_step(0)) &&
             is_53_predict_step(atk->get_step(1));
    }

    //////////////////////////////////////////////////////////////////////////
    // True when the kernel is the classic 5/3 kernel (the 5/3 update step
    // then the 5/3 predict step), for which fused single-pass horizontal
    // transforms are provided above.
    static inline bool is_rev53_kernel(const param_atk* atk)
    {
      if (atk->get_num_steps() != 2)
        return false;
      const lifting_step* s0 = atk->get_step(0);
      return s0->rev.Aatk == 1 && s0->rev.Batk == 2 && s0->rev.Eatk == 2 &&
             is_53_predict_step(atk->get_step(1));
    }

    //////////////////////////////////////////////////////////////////////////
    // True when the hwy vertical step should be preferred over the
    // fallback; set at install time (see install_rev_transforms)
    static bool hwy_vert_step_wins = false;

    //////////////////////////////////////////////////////////////////////////
    static
    void simd_rev_vert_step(const lifting_step* s, const line_buf* sig,
                           const line_buf* other, const line_buf* aug,
                           ui32 repeat, bool synthesis)
    {
      if (is_null_step(s))
        return; // the step changes nothing; rev13 update steps are such

      // At AVX2 the hand-written SIMD vertical steps (SSE2/AVX2) measured
      // slightly faster than the Highway loop of the same width, so the
      // Highway loop runs only when the generic implementation would run
      // otherwise; at AVX-512 class targets the Highway loop is well
      // ahead (see install_rev_transforms) and is preferred.
      if (!hwy_vert_step_wins && fb_rev_vert_step != gen_rev_vert_step)
      {
        fb_rev_vert_step(s, sig, other, aug, repeat, synthesis);
        return;
      }

      if (((sig != NULL) && (sig->flags & line_buf::LFT_32BIT)) ||
          ((aug != NULL) && (aug->flags & line_buf::LFT_32BIT)) ||
          ((other != NULL) && (other->flags & line_buf::LFT_32BIT)))
      {
        assert((sig == NULL || sig->flags & line_buf::LFT_32BIT) &&
               (other == NULL || other->flags & line_buf::LFT_32BIT) &&
               (aug == NULL || aug->flags & line_buf::LFT_32BIT));
        HWY_DYNAMIC_DISPATCH(rev_vert_step32)(s, sig->i32, other->i32,
                                              aug->i32, repeat, synthesis);
      }
      else
        fb_rev_vert_step(s, sig, other, aug, repeat, synthesis);
    }

    //////////////////////////////////////////////////////////////////////////
    static
    void simd_rev_vert_step_one_tap(const lifting_step* s,
                                   const line_buf* src, const line_buf* aug,
                                   ui32 repeat, bool synthesis)
    {
      bool done = false;
      if (aug->flags & line_buf::LFT_16BIT)
      {
        assert(src == NULL || src->flags & line_buf::LFT_16BIT);
        done = HWY_DYNAMIC_DISPATCH(rev_vert_one_tap16)(s, src->i16,
                 aug->i16, repeat, synthesis);
      }
      else if (aug->flags & line_buf::LFT_32BIT)
      {
        assert(src == NULL || src->flags & line_buf::LFT_32BIT);
        done = HWY_DYNAMIC_DISPATCH(rev_vert_one_tap32)(s, src->i32,
                 aug->i32, repeat, synthesis);
      }
      if (!done)
        fb_rev_vert_step_one_tap(s, src, aug, repeat, synthesis);
    }

    //////////////////////////////////////////////////////////////////////////
    static
    void simd_rev_horz_ana(const param_atk* atk, const line_buf* ldst,
                          const line_buf* hdst, const line_buf* src,
                          ui32 width, bool even)
    {
      if (width > 1 && (src->flags & line_buf::LFT_32BIT))
      {
        if (is_rev13_kernel(atk))
          HWY_DYNAMIC_DISPATCH(rev13_horz_ana32)(src->i32, ldst->i32,
            hdst->i32, width, even);
        else if (is_rev53_kernel(atk))
          HWY_DYNAMIC_DISPATCH(rev53_horz_ana32)(src->i32, ldst->i32,
            hdst->i32, width, even);
        else
          HWY_DYNAMIC_DISPATCH(rev_horz_ws_ana32)(atk, ldst, hdst, src,
            width, even);
      }
      else
        fb_rev_horz_ana(atk, ldst, hdst, src, width, even);
    }

    //////////////////////////////////////////////////////////////////////////
    static
    void simd_rev_horz_syn(const param_atk* atk, const line_buf* dst,
                          const line_buf* lsrc, const line_buf* hsrc,
                          ui32 width, bool even)
    {
      if (width > 1 && (dst->flags & line_buf::LFT_32BIT))
      {
        if (is_rev13_kernel(atk))
          HWY_DYNAMIC_DISPATCH(rev13_horz_syn32)(dst->i32, lsrc->i32,
            hsrc->i32, width, even);
        else if (is_rev53_kernel(atk))
          HWY_DYNAMIC_DISPATCH(rev53_horz_syn32)(dst->i32, lsrc->i32,
            hsrc->i32, width, even);
        else
          HWY_DYNAMIC_DISPATCH(rev_horz_ws_syn32)(atk, dst, lsrc, hsrc,
            width, even);
      }
      else
        fb_rev_horz_syn(atk, dst, lsrc, hsrc, width, even);
    }

    //////////////////////////////////////////////////////////////////////////
    static
    void simd_rev_horz_ana_arb(const param_atk* atk, const line_buf* ldst,
                              const line_buf* hdst, const line_buf* src,
                              ui32 width, bool even)
    {
      if (width > 1 && is_prev_sample_kernel(atk))
      {
        if (src->flags & line_buf::LFT_16BIT)
        {
          HWY_DYNAMIC_DISPATCH(rev_horz_ana_prev16)(ldst->i16, hdst->i16,
            src->i16, width, even);
          return;
        }
        else if (src->flags & line_buf::LFT_32BIT)
        {
          HWY_DYNAMIC_DISPATCH(rev_horz_ana_prev32)(ldst->i32, hdst->i32,
            src->i32, width, even);
          return;
        }
      }
      fb_rev_horz_ana_arb(atk, ldst, hdst, src, width, even);
    }

    //////////////////////////////////////////////////////////////////////////
    static
    void simd_rev_horz_syn_arb(const param_atk* atk, const line_buf* dst,
                              const line_buf* lsrc, const line_buf* hsrc,
                              ui32 width, bool even)
    {
      if (width > 1 && is_prev_sample_kernel(atk))
      {
        if (dst->flags & line_buf::LFT_16BIT)
        {
          HWY_DYNAMIC_DISPATCH(rev_horz_syn_prev16)(dst->i16, lsrc->i16,
            hsrc->i16, width, even);
          return;
        }
        else if (dst->flags & line_buf::LFT_32BIT)
        {
          HWY_DYNAMIC_DISPATCH(rev_horz_syn_prev32)(dst->i32, lsrc->i32,
            hsrc->i32, width, even);
          return;
        }
      }
      fb_rev_horz_syn_arb(atk, dst, lsrc, hsrc, width, even);
    }

    //////////////////////////////////////////////////////////////////////////
    static
    void simd_irv_vert_step(const lifting_step* s, const line_buf* sig,
                           const line_buf* other, const line_buf* aug,
                           ui32 repeat, bool synthesis)
    {
      HWY_DYNAMIC_DISPATCH(simd_irv_vert_step)(s, sig, other, aug,
                                               repeat, synthesis);
    }

    //////////////////////////////////////////////////////////////////////////
    static
    void simd_irv_vert_times_K(float K, const line_buf* aug, ui32 repeat)
    {
      HWY_DYNAMIC_DISPATCH(simd_irv_vert_times_K)(K, aug, repeat);
    }

    //////////////////////////////////////////////////////////////////////////
    static
    void simd_irv_horz_ana(const param_atk* atk, const line_buf* ldst,
                          const line_buf* hdst, const line_buf* src,
                          ui32 width, bool even)
    {
      HWY_DYNAMIC_DISPATCH(simd_irv_horz_ana)(atk, ldst, hdst, src,
                                              width, even);
    }

    //////////////////////////////////////////////////////////////////////////
    static
    void simd_irv_horz_syn(const param_atk* atk, const line_buf* dst,
                          const line_buf* lsrc, const line_buf* hsrc,
                          ui32 width, bool even)
    {
      HWY_DYNAMIC_DISPATCH(simd_irv_horz_syn)(atk, dst, lsrc, hsrc,
                                              width, even);
    }

    //////////////////////////////////////////////////////////////////////////
    // The SIMD targets compiled into this file that are available at run
    // time (0 when none is).  hwy assumes its baseline target is
    // supported without checking, so when the baseline needs more than
    // the architecture guarantees (an MSVC /arch:AVX2 build; the GCC and
    // clang builds keep the baseline at portable EMU128), verify it with
    // our own CPU detection.
    static inline int64_t hwy_simd_targets()
    {
#if defined(OJPH_ARCH_X86_64) || defined(OJPH_ARCH_I386)
  #if HWY_STATIC_TARGET <= HWY_AVX3
      if (get_cpu_ext_level() < X86_CPU_EXT_LEVEL_AVX512)
        return 0;
  #elif HWY_STATIC_TARGET <= HWY_AVX2
      if (get_cpu_ext_level() < X86_CPU_EXT_LEVEL_AVX2FMA)
        return 0;
  #elif HWY_STATIC_TARGET <= HWY_SSE4
      if (get_cpu_ext_level() < X86_CPU_EXT_LEVEL_SSE42)
        return 0;
  #endif
#endif
      const int64_t sup = hwy::SupportedTargets();
      // SupportedTargets() re-initializes hwy's chosen dispatch target
      // with the full detected set, counting on its caller to narrow it
      // to the returned (possibly DisableTargets-masked) set; do so, or
      // a preceding hwy::DisableTargets() would be ignored
      hwy::GetChosenTarget().Update(sup);
      return sup & HWY_TARGETS & ~(HWY_EMU128 | HWY_SCALAR);
    }

    //////////////////////////////////////////////////////////////////////////
    void install_irv_transforms()
    {
      if (hwy_simd_targets() == 0)
        return;
      irv_vert_step    = simd_irv_vert_step;
      irv_vert_times_K = simd_irv_vert_times_K;
      irv_horz_ana     = simd_irv_horz_ana;
      irv_horz_syn     = simd_irv_horz_syn;
    }

    //////////////////////////////////////////////////////////////////////////
    void install_rev_transforms()
    {
      const int64_t simd = hwy_simd_targets();
      if (simd == 0)
        return;
      // dispatch resolves to the best target (smaller bit values are
      // newer); at AVX-512 class targets the hwy vertical step measured
      // well ahead of the hand-written AVX2 survivor (176 vs 303
      // ns/4096-sample step on a Sapphire Rapids Xeon w5-2445), while at
      // AVX2 the survivor keeps a small edge
      const int64_t best = simd & (-simd);
      hwy_vert_step_wins = best <= HWY_AVX3;

      fb_rev_vert_step         = rev_vert_step;
      fb_rev_horz_ana          = rev_horz_ana;
      fb_rev_horz_syn          = rev_horz_syn;
      fb_rev_vert_step_one_tap = rev_vert_step_one_tap;
      fb_rev_horz_ana_arb      = rev_horz_ana_arb;
      fb_rev_horz_syn_arb      = rev_horz_syn_arb;

      rev_vert_step            = simd_rev_vert_step;
      rev_horz_ana             = simd_rev_horz_ana;
      rev_horz_syn             = simd_rev_horz_syn;
      rev_vert_step_one_tap    = simd_rev_vert_step_one_tap;
      rev_horz_ana_arb         = simd_rev_horz_ana_arb;
      rev_horz_syn_arb         = simd_rev_horz_syn_arb;
    }

  } // !local namespace
} // !ojph namespace

#endif // HWY_ONCE

#endif // OJPH_ENABLE_HWY
