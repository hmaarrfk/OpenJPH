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
// translation unit is compiled for a single (static) Highway target,
// selected at compile time from the compiler flags the build system sets
// for it (see src/core/CMakeLists.txt); hwy_install_rev_transforms()
// installs the functions only when the CPU supports that target.
//
// The functions here compute values identical to those of the generic
// (and SSE2/AVX2) implementations; codestreams they produce are
// byte-identical.  Cases they do not handle (64-bit lines, uncommon
// lifting kernels) are forwarded to the functions that were selected
// before installation.

// Highway must be included before any ojph header, because ojph_defs.h
// renames the ojph namespace token.
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

namespace hn = hwy::HWY_NAMESPACE;

namespace ojph {
  namespace local {

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
    // transforms are provided below.
    static inline bool is_rev13_kernel(const param_atk* atk)
    {
      return atk->get_num_steps() == 2 && is_null_step(atk->get_step(0)) &&
             is_53_predict_step(atk->get_step(1));
    }

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
    void hwy_rev_vert_pass32(const si32* src1, const si32* src2, si32* dst,
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
        auto s1 = hn::Load(d, src1 + i);
        auto s2 = hn::Load(d, src2 + i);
        auto dv = hn::Load(d, dst + i);
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
    static
    void hwy_rev_vert_step32(const lifting_step* s, const si32* src1,
                             const si32* src2, si32* dst, ui32 repeat,
                             bool synthesis)
    {
      const si32 a = s->rev.Aatk;
      const si32 b = s->rev.Batk;
      const ui8 e = s->rev.Eatk;

      if (a == 1) {
        if (synthesis)
          hwy_rev_vert_pass32<0, true>(src1, src2, dst, repeat, a, b, e);
        else
          hwy_rev_vert_pass32<0, false>(src1, src2, dst, repeat, a, b, e);
      }
      else if (a == -1 && b == 1 && e == 1) {
        if (synthesis)
          hwy_rev_vert_pass32<1, false>(src1, src2, dst, repeat, a, b, e);
        else
          hwy_rev_vert_pass32<1, true>(src1, src2, dst, repeat, a, b, e);
      }
      else if (a == -1) {
        if (synthesis)
          hwy_rev_vert_pass32<2, true>(src1, src2, dst, repeat, a, b, e);
        else
          hwy_rev_vert_pass32<2, false>(src1, src2, dst, repeat, a, b, e);
      }
      else {
        if (synthesis)
          hwy_rev_vert_pass32<3, true>(src1, src2, dst, repeat, a, b, e);
        else
          hwy_rev_vert_pass32<3, false>(src1, src2, dst, repeat, a, b, e);
      }
    }

    //////////////////////////////////////////////////////////////////////////
    static
    void hwy_rev_vert_step(const lifting_step* s, const line_buf* sig,
                           const line_buf* other, const line_buf* aug,
                           ui32 repeat, bool synthesis)
    {
      if (is_null_step(s))
        return; // the step changes nothing; rev13 update steps are such

      // Measurements show the hand-written SIMD vertical steps (SSE2/AVX2)
      // are slightly faster than the Highway loop of the same width; use
      // the Highway loop only when the generic implementation would run
      // otherwise.
      if (fb_rev_vert_step != gen_rev_vert_step)
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
        hwy_rev_vert_step32(s, sig->i32, other->i32, aug->i32, repeat,
                            synthesis);
      }
      else
        fb_rev_vert_step(s, sig, other, aug, repeat, synthesis);
    }

    //////////////////////////////////////////////////////////////////////////
    // One vertical lifting step of an arbitrary kernel with a single
    // lifting coefficient; the cases mirror gen_rev_vert_step_one_tap_T.
    // Only the cases whose scalar arithmetic is performed in the lane type
    // are implemented; the rest are forwarded, so that results stay
    // identical to the generic implementation.
    template <typename T>
    static
    bool hwy_rev_vert_step_one_tap_T(const lifting_step* s, const T* sp,
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
    static
    void hwy_rev_vert_step_one_tap(const lifting_step* s,
                                   const line_buf* src, const line_buf* aug,
                                   ui32 repeat, bool synthesis)
    {
      bool done = false;
      if (aug->flags & line_buf::LFT_16BIT)
      {
        assert(src == NULL || src->flags & line_buf::LFT_16BIT);
        done = hwy_rev_vert_step_one_tap_T<si16>(s, src->i16, aug->i16,
                                                 repeat, synthesis);
      }
      else if (aug->flags & line_buf::LFT_32BIT)
      {
        assert(src == NULL || src->flags & line_buf::LFT_32BIT);
        done = hwy_rev_vert_step_one_tap_T<si32>(s, src->i32, aug->i32,
                                                 repeat, synthesis);
      }
      if (!done)
        fb_rev_vert_step_one_tap(s, src, aug, repeat, synthesis);
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
    static
    void hwy_rev13_horz_ana32(const si32* sp, si32* lp, si32* hp,
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
    // the output in a single pass; the inverse of hwy_rev13_horz_ana32.
    static
    void hwy_rev13_horz_syn32(si32* dp, const si32* lp, const si32* hp,
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
    static
    void hwy_rev_horz_ana(const param_atk* atk, const line_buf* ldst,
                          const line_buf* hdst, const line_buf* src,
                          ui32 width, bool even)
    {
      if (width > 1 && (src->flags & line_buf::LFT_32BIT) &&
          is_rev13_kernel(atk))
        hwy_rev13_horz_ana32(src->i32, ldst->i32, hdst->i32, width, even);
      else
        fb_rev_horz_ana(atk, ldst, hdst, src, width, even);
    }

    //////////////////////////////////////////////////////////////////////////
    static
    void hwy_rev_horz_syn(const param_atk* atk, const line_buf* dst,
                          const line_buf* lsrc, const line_buf* hsrc,
                          ui32 width, bool even)
    {
      if (width > 1 && (dst->flags & line_buf::LFT_32BIT) &&
          is_rev13_kernel(atk))
        hwy_rev13_horz_syn32(dst->i32, lsrc->i32, hsrc->i32, width, even);
      else
        fb_rev_horz_syn(atk, dst, lsrc, hsrc, width, even);
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
    void hwy_rev_horz_ana_prev_T(T* lp, T* hp, const T* sp,
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
    void hwy_rev_horz_syn_prev_T(T* dp, const T* lp, const T* hp,
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
    static
    void hwy_rev_horz_ana_arb(const param_atk* atk, const line_buf* ldst,
                              const line_buf* hdst, const line_buf* src,
                              ui32 width, bool even)
    {
      if (width > 1 && is_prev_sample_kernel(atk))
      {
        if (src->flags & line_buf::LFT_16BIT)
        {
          hwy_rev_horz_ana_prev_T<si16>(ldst->i16, hdst->i16, src->i16,
                                        width, even);
          return;
        }
        else if (src->flags & line_buf::LFT_32BIT)
        {
          hwy_rev_horz_ana_prev_T<si32>(ldst->i32, hdst->i32, src->i32,
                                        width, even);
          return;
        }
      }
      fb_rev_horz_ana_arb(atk, ldst, hdst, src, width, even);
    }

    //////////////////////////////////////////////////////////////////////////
    static
    void hwy_rev_horz_syn_arb(const param_atk* atk, const line_buf* dst,
                              const line_buf* lsrc, const line_buf* hsrc,
                              ui32 width, bool even)
    {
      if (width > 1 && is_prev_sample_kernel(atk))
      {
        if (dst->flags & line_buf::LFT_16BIT)
        {
          hwy_rev_horz_syn_prev_T<si16>(dst->i16, lsrc->i16, hsrc->i16,
                                        width, even);
          return;
        }
        else if (dst->flags & line_buf::LFT_32BIT)
        {
          hwy_rev_horz_syn_prev_T<si32>(dst->i32, lsrc->i32, hsrc->i32,
                                        width, even);
          return;
        }
      }
      fb_rev_horz_syn_arb(atk, dst, lsrc, hsrc, width, even);
    }

    //////////////////////////////////////////////////////////////////////////
    void hwy_install_rev_transforms()
    {
#if defined(OJPH_ARCH_X86_64) || defined(OJPH_ARCH_I386)
      // this file is compiled for a fixed Highway target; install only
      // when the CPU supports it (targets are bitflags; smaller is newer)
  #if HWY_STATIC_TARGET <= HWY_AVX3
      if (get_cpu_ext_level() < X86_CPU_EXT_LEVEL_AVX512)
        return;
  #elif HWY_STATIC_TARGET <= HWY_AVX2
      if (get_cpu_ext_level() < X86_CPU_EXT_LEVEL_AVX2FMA)
        return;
  #elif HWY_STATIC_TARGET <= HWY_SSE4
      if (get_cpu_ext_level() < X86_CPU_EXT_LEVEL_SSE42)
        return;
  #endif
#endif
      fb_rev_vert_step         = rev_vert_step;
      fb_rev_horz_ana          = rev_horz_ana;
      fb_rev_horz_syn          = rev_horz_syn;
      fb_rev_vert_step_one_tap = rev_vert_step_one_tap;
      fb_rev_horz_ana_arb      = rev_horz_ana_arb;
      fb_rev_horz_syn_arb      = rev_horz_syn_arb;

      rev_vert_step            = hwy_rev_vert_step;
      rev_horz_ana             = hwy_rev_horz_ana;
      rev_horz_syn             = hwy_rev_horz_syn;
      rev_vert_step_one_tap    = hwy_rev_vert_step_one_tap;
      rev_horz_ana_arb         = hwy_rev_horz_ana_arb;
      rev_horz_syn_arb         = hwy_rev_horz_syn_arb;
    }

  } // !local namespace
} // !ojph namespace

#endif // OJPH_ENABLE_HWY
