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
// File: ojph_colour_hwy.cpp
//***************************************************************************/

// Google Highway implementations of the colour transforms (RCT and
// ICT) and the rev_convert line conversion.  Like
// ojph_transform_hwy.cpp, this translation unit is compiled once per
// Highway target (SSE4, AVX2, AVX3, ...) through foreach_target.h and
// dispatches to the best target the CPU supports at run time; it is
// compiled with -ffp-contract=off, so the float (ICT) results are
// identical to the generic implementation.

// Highway must be included before any ojph header, because ojph_defs.h
// renames the ojph namespace token.
#undef HWY_TARGET_INCLUDE
#define HWY_TARGET_INCLUDE "transform/ojph_colour_hwy.cpp"
#include <hwy/foreach_target.h>
#include <hwy/highway.h>

#include "ojph_arch.h"

#ifdef OJPH_ENABLE_HWY

#include <cassert>
#include <climits>

#include "ojph_defs.h"
#include "ojph_mem.h"
#include "ojph_colour.h"
#include "ojph_colour_local.h"

HWY_BEFORE_NAMESPACE();
namespace ojph {
  namespace local {
    namespace HWY_NAMESPACE {

    namespace hn = hwy::HWY_NAMESPACE;

    //////////////////////////////////////////////////////////////////////////
    // add a shift while copying a line, converting between 32- and
    // 64-bit integers as needed; values identical to gen_rev_convert.
    // This is the hot per-line output conversion of tile::pull for
    // components with 32-bit lines.
    void simd_rev_convert(
      const line_buf *src_line, const ui32 src_line_offset,
      line_buf *dst_line, const ui32 dst_line_offset,
      si64 shift, ui32 width)
    {
      if (src_line->flags & line_buf::LFT_32BIT)
      {
        if (dst_line->flags & line_buf::LFT_32BIT)
        {
          const si32 *sp = src_line->i32 + src_line_offset;
          si32 *dp = dst_line->i32 + dst_line_offset;
          const hn::ScalableTag<si32> d;
          const ui32 L = (ui32)hn::Lanes(d);
          const auto vs = hn::Set(d, (si32)shift);
          for (ui32 i = 0; i < width; i += L)
            hn::StoreU(hn::Add(hn::LoadU(d, sp + i), vs), d, dp + i);
        }
        else
        {
          const si32 *sp = src_line->i32 + src_line_offset;
          si64 *dp = dst_line->i64 + dst_line_offset;
          const hn::ScalableTag<si64> d64;
          const hn::Rebind<si32, decltype(d64)> d32h; // half-width
          const ui32 L = (ui32)hn::Lanes(d64);
          const auto vs = hn::Set(d64, shift);
          for (ui32 i = 0; i < width; i += L)
          {
            auto v = hn::PromoteTo(d64, hn::LoadU(d32h, sp + i));
            hn::StoreU(hn::Add(v, vs), d64, dp + i);
          }
        }
      }
      else
      {
        assert(src_line->flags & line_buf::LFT_64BIT);
        assert(dst_line->flags & line_buf::LFT_32BIT);
        const si64 *sp = src_line->i64 + src_line_offset;
        si32 *dp = dst_line->i32 + dst_line_offset;
        const hn::ScalableTag<si64> d64;
        const hn::Rebind<ui32, decltype(d64)> du32h; // half-width
        const hn::RebindToUnsigned<decltype(d64)> du64;
        const ui32 L = (ui32)hn::Lanes(d64);
        const auto vs = hn::Set(d64, shift);
        for (ui32 i = 0; i < width; i += L)
        {
          auto v = hn::Add(hn::LoadU(d64, sp + i), vs);
          // truncate si64 to si32, as the generic implementation does
          hn::StoreU(hn::TruncateTo(du32h, hn::BitCast(du64, v)),
                     du32h, (ui32*)dp + i);
        }
      }
    }

    //////////////////////////////////////////////////////////////////////////
    // Round float lanes to the nearest integer, ties to even.  The x86
    // targets provide NearestIntInRange (a bare cvtps2dq); the portable
    // fallback targets in this hwy version do not, so use NearestInt
    // there -- the fallback only has to compile, since the install gate
    // in install_colour_transforms() never dispatches to it (the two
    // only differ for out-of-range lanes, which the callers overwrite
    // when clipping anyway).
    template <class DI32, class VF>
    static inline hn::VFromD<DI32> nearest_int(DI32 di32, VF v)
    {
#if HWY_TARGET == HWY_EMU128 || HWY_TARGET == HWY_SCALAR
      (void)di32;
      return hn::NearestInt(v);
#else
      return hn::NearestIntInRange(di32, v);
#endif
    }

    //////////////////////////////////////////////////////////////////////////
    // Convert the float line produced by the inverse irreversible
    // transform to integers, clipping values that exceed what bit_depth
    // supports; the vector equivalent of gen_irv_convert_to_integer.
    // Rounding uses the vector round-to-nearest-even conversion, like
    // the SSE2/AVX2 implementations this replaces (the generic
    // implementation rounds half away from zero, so results can differ
    // from it at exact .5 ties; this mirrors the historical behaviour).
    // Like the other loops in this file, the loops overrun the line
    // ends by less than one vector; the lines are padded.
    template<bool NLT_TYPE3>
    static inline
    void local_simd_irv_convert_to_integer(const line_buf *src_line,
      line_buf *dst_line, ui32 dst_line_offset,
      ui32 bit_depth, bool is_signed, ui32 width)
    {
      assert((src_line->flags & line_buf::LFT_32BIT) &&
             (src_line->flags & line_buf::LFT_INTEGER) == 0 &&
             (dst_line->flags & line_buf::LFT_32BIT) &&
             (dst_line->flags & line_buf::LFT_INTEGER));
      assert(bit_depth <= 32);

      const hn::ScalableTag<float> df;
      const hn::RebindToSigned<decltype(df)> d32;
      const int L = (int)hn::Lanes(df);

      // values outside the half-closed interval [-0.5f, 0.5f) are
      // clipped to the range bit_depth supports (see the generic
      // implementation for the reasoning)
      const si32 neg_limit = (si32)INT_MIN >> (32 - bit_depth);
      const auto mul = hn::Set(df, (float)(1ull << bit_depth));
      const auto fl_up_lim  = hn::Set(df, -(float)neg_limit); // val < up
      const auto fl_low_lim = hn::Set(df, (float)neg_limit);  // val >= low
      const auto s32_up_lim  = hn::Set(d32, INT_MAX >> (32 - bit_depth));
      const auto s32_low_lim = hn::Set(d32, INT_MIN >> (32 - bit_depth));

      const float* sp = src_line->f32;
      si32* dp = dst_line->i32 + dst_line_offset;
      if (is_signed)
      {
        const auto bias =
          hn::Set(d32, -(si32)((1ULL << (bit_depth - 1)) + 1));
        for (int i = (int)width; i > 0; i -= L, sp += L, dp += L)
        {
          auto t = hn::Mul(hn::LoadU(df, sp), mul);
          auto u = nearest_int(d32, t);
          u = hn::IfThenElse(hn::RebindMask(d32, hn::Ge(t, fl_low_lim)),
                             u, s32_low_lim);
          u = hn::IfThenElse(hn::RebindMask(d32, hn::Lt(t, fl_up_lim)),
                             u, s32_up_lim);
          if (NLT_TYPE3)
            u = hn::IfNegativeThenElse(u, hn::Sub(bias, u), u);
          hn::StoreU(u, d32, dp);
        }
      }
      else
      {
        const auto half = hn::Set(d32, (si32)(1ULL << (bit_depth - 1)));
        for (int i = (int)width; i > 0; i -= L, sp += L, dp += L)
        {
          auto t = hn::Mul(hn::LoadU(df, sp), mul);
          auto u = nearest_int(d32, t);
          u = hn::IfThenElse(hn::RebindMask(d32, hn::Ge(t, fl_low_lim)),
                             u, s32_low_lim);
          u = hn::IfThenElse(hn::RebindMask(d32, hn::Lt(t, fl_up_lim)),
                             u, s32_up_lim);
          hn::StoreU(hn::Add(u, half), d32, dp);
        }
      }
    }

    //////////////////////////////////////////////////////////////////////////
    void simd_irv_convert_to_integer(const line_buf *src_line,
      line_buf *dst_line, ui32 dst_line_offset,
      ui32 bit_depth, bool is_signed, ui32 width)
    {
      local_simd_irv_convert_to_integer<false>(src_line, dst_line,
        dst_line_offset, bit_depth, is_signed, width);
    }

    //////////////////////////////////////////////////////////////////////////
    void simd_irv_convert_to_integer_nlt_type3(const line_buf *src_line,
      line_buf *dst_line, ui32 dst_line_offset,
      ui32 bit_depth, bool is_signed, ui32 width)
    {
      local_simd_irv_convert_to_integer<true>(src_line, dst_line,
        dst_line_offset, bit_depth, is_signed, width);
    }

    //////////////////////////////////////////////////////////////////////////
    // Convert an integer line to the floats fed to the forward
    // irreversible transform, undoing the level shift; the vector
    // equivalent of gen_irv_convert_to_float (values identical)
    template<bool NLT_TYPE3>
    static inline
    void local_simd_irv_convert_to_float(const line_buf *src_line,
      ui32 src_line_offset, line_buf *dst_line,
      ui32 bit_depth, bool is_signed, ui32 width)
    {
      assert((src_line->flags & line_buf::LFT_32BIT) &&
             (src_line->flags & line_buf::LFT_INTEGER) &&
             (dst_line->flags & line_buf::LFT_32BIT) &&
             (dst_line->flags & line_buf::LFT_INTEGER) == 0);
      assert(bit_depth <= 32);

      const hn::ScalableTag<float> df;
      const hn::RebindToSigned<decltype(df)> d32;
      const int L = (int)hn::Lanes(df);
      const auto mul =
        hn::Set(df, (float)(1.0 / (double)(1ULL << bit_depth)));

      const si32* sp = src_line->i32 + src_line_offset;
      float* dp = dst_line->f32;
      if (is_signed)
      {
        const auto bias =
          hn::Set(d32, -(si32)((1ULL << (bit_depth - 1)) + 1));
        for (int i = (int)width; i > 0; i -= L, sp += L, dp += L)
        {
          auto t = hn::LoadU(d32, sp);
          if (NLT_TYPE3)
            t = hn::IfNegativeThenElse(t, hn::Sub(bias, t), t);
          hn::StoreU(hn::Mul(hn::ConvertTo(df, t), mul), df, dp);
        }
      }
      else
      {
        const auto half = hn::Set(d32, (si32)(1ULL << (bit_depth - 1)));
        for (int i = (int)width; i > 0; i -= L, sp += L, dp += L)
        {
          auto t = hn::Sub(hn::LoadU(d32, sp), half);
          hn::StoreU(hn::Mul(hn::ConvertTo(df, t), mul), df, dp);
        }
      }
    }

    //////////////////////////////////////////////////////////////////////////
    void simd_irv_convert_to_float(const line_buf *src_line,
      ui32 src_line_offset, line_buf *dst_line,
      ui32 bit_depth, bool is_signed, ui32 width)
    {
      local_simd_irv_convert_to_float<false>(src_line, src_line_offset,
        dst_line, bit_depth, is_signed, width);
    }

    //////////////////////////////////////////////////////////////////////////
    void simd_irv_convert_to_float_nlt_type3(const line_buf *src_line,
      ui32 src_line_offset, line_buf *dst_line,
      ui32 bit_depth, bool is_signed, ui32 width)
    {
      local_simd_irv_convert_to_float<true>(src_line, src_line_offset,
        dst_line, bit_depth, is_signed, width);
    }

    //////////////////////////////////////////////////////////////////////////
    // Forward RCT; values identical to gen_rct_forward.  Like all the
    // loops in this file, the loops overrun the line ends by less than
    // one vector; the lines are padded (the SSE2/AVX2 implementations
    // rely on the same).
    void simd_rct_forward(
      const line_buf *r, const line_buf *g, const line_buf *b,
      line_buf *y, line_buf *cb, line_buf *cr, ui32 repeat)
    {
      assert((y->flags  & line_buf::LFT_INTEGER) &&
             (cb->flags & line_buf::LFT_INTEGER) &&
             (cr->flags & line_buf::LFT_INTEGER) &&
             (r->flags  & line_buf::LFT_INTEGER) &&
             (g->flags  & line_buf::LFT_INTEGER) &&
             (b->flags  & line_buf::LFT_INTEGER));

      const hn::ScalableTag<si32> d32;
      const ui32 L = (ui32)hn::Lanes(d32);

      if (y->flags & line_buf::LFT_32BIT)
      {
        assert((y->flags  & line_buf::LFT_32BIT) &&
               (cb->flags & line_buf::LFT_32BIT) &&
               (cr->flags & line_buf::LFT_32BIT) &&
               (r->flags  & line_buf::LFT_32BIT) &&
               (g->flags  & line_buf::LFT_32BIT) &&
               (b->flags  & line_buf::LFT_32BIT));
        const si32 *rp = r->i32, * gp = g->i32, * bp = b->i32;
        si32 *yp = y->i32, * cbp = cb->i32, * crp = cr->i32;
        for (ui32 i = 0; i < repeat; i += L)
        {
          auto mr = hn::LoadU(d32, rp + i);
          auto mg = hn::LoadU(d32, gp + i);
          auto mb = hn::LoadU(d32, bp + i);
          auto t = hn::Add(hn::Add(mr, mb), hn::ShiftLeft<1>(mg));
          hn::StoreU(hn::ShiftRight<2>(t), d32, yp + i);
          hn::StoreU(hn::Sub(mb, mg), d32, cbp + i);
          hn::StoreU(hn::Sub(mr, mg), d32, crp + i);
        }
      }
      else
      {
        assert((y->flags  & line_buf::LFT_64BIT) &&
               (cb->flags & line_buf::LFT_64BIT) &&
               (cr->flags & line_buf::LFT_64BIT) &&
               (r->flags  & line_buf::LFT_32BIT) &&
               (g->flags  & line_buf::LFT_32BIT) &&
               (b->flags  & line_buf::LFT_32BIT));
        const hn::Repartition<si64, decltype(d32)> d64;
        const si32 *rp = r->i32, *gp = g->i32, *bp = b->i32;
        si64 *yp = y->i64, *cbp = cb->i64, *crp = cr->i64;
        for (ui32 i = 0; i < repeat; i += L)
        {
          auto mr32 = hn::LoadU(d32, rp + i);
          auto mg32 = hn::LoadU(d32, gp + i);
          auto mb32 = hn::LoadU(d32, bp + i);

          auto mr = hn::PromoteLowerTo(d64, mr32);
          auto mg = hn::PromoteLowerTo(d64, mg32);
          auto mb = hn::PromoteLowerTo(d64, mb32);
          auto t = hn::Add(hn::Add(mr, mb), hn::ShiftLeft<1>(mg));
          hn::StoreU(hn::ShiftRight<2>(t), d64, yp + i);
          hn::StoreU(hn::Sub(mb, mg), d64, cbp + i);
          hn::StoreU(hn::Sub(mr, mg), d64, crp + i);

          mr = hn::PromoteUpperTo(d64, mr32);
          mg = hn::PromoteUpperTo(d64, mg32);
          mb = hn::PromoteUpperTo(d64, mb32);
          t = hn::Add(hn::Add(mr, mb), hn::ShiftLeft<1>(mg));
          hn::StoreU(hn::ShiftRight<2>(t), d64, yp + i + L / 2);
          hn::StoreU(hn::Sub(mb, mg), d64, cbp + i + L / 2);
          hn::StoreU(hn::Sub(mr, mg), d64, crp + i + L / 2);
        }
      }
    }

    //////////////////////////////////////////////////////////////////////////
    // Backward RCT; values identical to gen_rct_backward
    void simd_rct_backward(
      const line_buf *y, const line_buf *cb, const line_buf *cr,
      line_buf *r, line_buf *g, line_buf *b, ui32 repeat)
    {
      assert((y->flags  & line_buf::LFT_INTEGER) &&
             (cb->flags & line_buf::LFT_INTEGER) &&
             (cr->flags & line_buf::LFT_INTEGER) &&
             (r->flags  & line_buf::LFT_INTEGER) &&
             (g->flags  & line_buf::LFT_INTEGER) &&
             (b->flags  & line_buf::LFT_INTEGER));

      const hn::ScalableTag<si32> d32;
      const ui32 L = (ui32)hn::Lanes(d32);

      if (y->flags & line_buf::LFT_32BIT)
      {
        assert((y->flags  & line_buf::LFT_32BIT) &&
               (cb->flags & line_buf::LFT_32BIT) &&
               (cr->flags & line_buf::LFT_32BIT) &&
               (r->flags  & line_buf::LFT_32BIT) &&
               (g->flags  & line_buf::LFT_32BIT) &&
               (b->flags  & line_buf::LFT_32BIT));
        const si32 *yp = y->i32, *cbp = cb->i32, *crp = cr->i32;
        si32 *rp = r->i32, *gp = g->i32, *bp = b->i32;
        for (ui32 i = 0; i < repeat; i += L)
        {
          auto my  = hn::LoadU(d32, yp + i);
          auto mcb = hn::LoadU(d32, cbp + i);
          auto mcr = hn::LoadU(d32, crp + i);
          auto t = hn::Add(mcb, mcr);
          auto mg = hn::Sub(my, hn::ShiftRight<2>(t));
          hn::StoreU(mg, d32, gp + i);
          hn::StoreU(hn::Add(mcb, mg), d32, bp + i);
          hn::StoreU(hn::Add(mcr, mg), d32, rp + i);
        }
      }
      else
      {
        assert((y->flags  & line_buf::LFT_64BIT) &&
               (cb->flags & line_buf::LFT_64BIT) &&
               (cr->flags & line_buf::LFT_64BIT) &&
               (r->flags  & line_buf::LFT_32BIT) &&
               (g->flags  & line_buf::LFT_32BIT) &&
               (b->flags  & line_buf::LFT_32BIT));
        const hn::Repartition<si64, decltype(d32)> d64;
        const hn::Rebind<ui32, decltype(d64)> du32h; // half-width
        const hn::RebindToUnsigned<decltype(d64)> du64;
        const si64 *yp = y->i64, *cbp = cb->i64, *crp = cr->i64;
        si32 *rp = r->i32, *gp = g->i32, *bp = b->i32;
        for (ui32 i = 0; i < repeat; i += L)
        {
          for (ui32 k = 0; k < 2; ++k)
          { // two half-vectors of si64 per full vector of si32 outputs
            ui32 j = i + k * (L / 2);
            auto my  = hn::LoadU(d64, yp + j);
            auto mcb = hn::LoadU(d64, cbp + j);
            auto mcr = hn::LoadU(d64, crp + j);
            auto t = hn::Add(mcb, mcr);
            auto mg = hn::Sub(my, hn::ShiftRight<2>(t));
            // truncate si64 to si32, as the generic implementation does
            hn::StoreU(hn::TruncateTo(du32h, hn::BitCast(du64, mg)),
                       du32h, (ui32*)gp + j);
            hn::StoreU(hn::TruncateTo(du32h,
              hn::BitCast(du64, hn::Add(mcb, mg))), du32h, (ui32*)bp + j);
            hn::StoreU(hn::TruncateTo(du32h,
              hn::BitCast(du64, hn::Add(mcr, mg))), du32h, (ui32*)rp + j);
          }
        }
      }
    }

    //////////////////////////////////////////////////////////////////////////
    // Forward ICT; values identical to gen_ict_forward (no fma; see the
    // note at the top of the file)
    void simd_ict_forward(const float *r, const float *g, const float *b,
                         float *y, float *cb, float *cr, ui32 repeat)
    {
      const hn::ScalableTag<float> d;
      const ui32 L = (ui32)hn::Lanes(d);
      const auto alpha_rf = hn::Set(d, CT_CNST::ALPHA_RF);
      const auto alpha_gf = hn::Set(d, CT_CNST::ALPHA_GF);
      const auto alpha_bf = hn::Set(d, CT_CNST::ALPHA_BF);
      const auto beta_cbf = hn::Set(d, CT_CNST::BETA_CbF);
      const auto beta_crf = hn::Set(d, CT_CNST::BETA_CrF);
      for (ui32 i = 0; i < repeat; i += L)
      {
        auto mr = hn::LoadU(d, r + i);
        auto mg = hn::LoadU(d, g + i);
        auto mb = hn::LoadU(d, b + i);
        auto my = hn::Mul(alpha_rf, mr);
        my = hn::Add(my, hn::Mul(alpha_gf, mg));
        my = hn::Add(my, hn::Mul(alpha_bf, mb));
        hn::StoreU(my, d, y + i);
        hn::StoreU(hn::Mul(beta_cbf, hn::Sub(mb, my)), d, cb + i);
        hn::StoreU(hn::Mul(beta_crf, hn::Sub(mr, my)), d, cr + i);
      }
    }

    //////////////////////////////////////////////////////////////////////////
    // Backward ICT; values identical to gen_ict_backward
    void simd_ict_backward(const float *y, const float *cb, const float *cr,
                          float *r, float *g, float *b, ui32 repeat)
    {
      const hn::ScalableTag<float> d;
      const ui32 L = (ui32)hn::Lanes(d);
      const auto gamma_cr2g = hn::Set(d, CT_CNST::GAMMA_CR2G);
      const auto gamma_cb2g = hn::Set(d, CT_CNST::GAMMA_CB2G);
      const auto gamma_cr2r = hn::Set(d, CT_CNST::GAMMA_CR2R);
      const auto gamma_cb2b = hn::Set(d, CT_CNST::GAMMA_CB2B);
      for (ui32 i = 0; i < repeat; i += L)
      {
        auto my  = hn::LoadU(d, y + i);
        auto mcb = hn::LoadU(d, cb + i);
        auto mcr = hn::LoadU(d, cr + i);
        auto mg = hn::Sub(my, hn::Mul(gamma_cr2g, mcr));
        hn::StoreU(hn::Sub(mg, hn::Mul(gamma_cb2g, mcb)), d, g + i);
        hn::StoreU(hn::Add(my, hn::Mul(gamma_cr2r, mcr)), d, r + i);
        hn::StoreU(hn::Add(my, hn::Mul(gamma_cb2b, mcb)), d, b + i);
      }
    }

    } // !HWY_NAMESPACE namespace
  } // !local namespace
} // !ojph namespace
HWY_AFTER_NAMESPACE();

#if HWY_ONCE

namespace ojph {
  namespace local {

    HWY_EXPORT(simd_rev_convert);
    HWY_EXPORT(simd_rct_forward);
    HWY_EXPORT(simd_rct_backward);
    HWY_EXPORT(simd_ict_forward);
    HWY_EXPORT(simd_ict_backward);
    HWY_EXPORT(simd_irv_convert_to_integer);
    HWY_EXPORT(simd_irv_convert_to_integer_nlt_type3);
    HWY_EXPORT(simd_irv_convert_to_float);
    HWY_EXPORT(simd_irv_convert_to_float_nlt_type3);

    //////////////////////////////////////////////////////////////////////////
    static
    void simd_rev_convert(
      const line_buf *src_line, const ui32 src_line_offset,
      line_buf *dst_line, const ui32 dst_line_offset,
      si64 shift, ui32 width)
    {
      HWY_DYNAMIC_DISPATCH(simd_rev_convert)(src_line, src_line_offset,
        dst_line, dst_line_offset, shift, width);
    }

    //////////////////////////////////////////////////////////////////////////
    static
    void simd_rct_forward(
      const line_buf *r, const line_buf *g, const line_buf *b,
      line_buf *y, line_buf *cb, line_buf *cr, ui32 repeat)
    {
      HWY_DYNAMIC_DISPATCH(simd_rct_forward)(r, g, b, y, cb, cr, repeat);
    }

    //////////////////////////////////////////////////////////////////////////
    static
    void simd_rct_backward(
      const line_buf *y, const line_buf *cb, const line_buf *cr,
      line_buf *r, line_buf *g, line_buf *b, ui32 repeat)
    {
      HWY_DYNAMIC_DISPATCH(simd_rct_backward)(y, cb, cr, r, g, b, repeat);
    }

    //////////////////////////////////////////////////////////////////////////
    static
    void simd_ict_forward(const float *r, const float *g, const float *b,
                         float *y, float *cb, float *cr, ui32 repeat)
    {
      HWY_DYNAMIC_DISPATCH(simd_ict_forward)(r, g, b, y, cb, cr, repeat);
    }

    //////////////////////////////////////////////////////////////////////////
    static
    void simd_ict_backward(const float *y, const float *cb, const float *cr,
                          float *r, float *g, float *b, ui32 repeat)
    {
      HWY_DYNAMIC_DISPATCH(simd_ict_backward)(y, cb, cr, r, g, b, repeat);
    }

    //////////////////////////////////////////////////////////////////////////
    static
    void simd_irv_convert_to_integer(const line_buf *src_line,
      line_buf *dst_line, ui32 dst_line_offset,
      ui32 bit_depth, bool is_signed, ui32 width)
    {
      HWY_DYNAMIC_DISPATCH(simd_irv_convert_to_integer)(src_line,
        dst_line, dst_line_offset, bit_depth, is_signed, width);
    }

    //////////////////////////////////////////////////////////////////////////
    static
    void simd_irv_convert_to_integer_nlt_type3(const line_buf *src_line,
      line_buf *dst_line, ui32 dst_line_offset,
      ui32 bit_depth, bool is_signed, ui32 width)
    {
      HWY_DYNAMIC_DISPATCH(simd_irv_convert_to_integer_nlt_type3)(src_line,
        dst_line, dst_line_offset, bit_depth, is_signed, width);
    }

    //////////////////////////////////////////////////////////////////////////
    static
    void simd_irv_convert_to_float(const line_buf *src_line,
      ui32 src_line_offset, line_buf *dst_line,
      ui32 bit_depth, bool is_signed, ui32 width)
    {
      HWY_DYNAMIC_DISPATCH(simd_irv_convert_to_float)(src_line,
        src_line_offset, dst_line, bit_depth, is_signed, width);
    }

    //////////////////////////////////////////////////////////////////////////
    static
    void simd_irv_convert_to_float_nlt_type3(const line_buf *src_line,
      ui32 src_line_offset, line_buf *dst_line,
      ui32 bit_depth, bool is_signed, ui32 width)
    {
      HWY_DYNAMIC_DISPATCH(simd_irv_convert_to_float_nlt_type3)(src_line,
        src_line_offset, dst_line, bit_depth, is_signed, width);
    }

    //////////////////////////////////////////////////////////////////////////
    void install_colour_transforms()
    {
#if defined(OJPH_ARCH_X86_64) || defined(OJPH_ARCH_I386)
      // dispatch resolves to the best compiled-in target the CPU
      // supports; install only when one of the SIMD targets is
      // available.  hwy assumes its baseline target is supported
      // without checking, so when the baseline needs more than the
      // architecture guarantees (an MSVC /arch:AVX2 build; the GCC and
      // clang builds keep the baseline at portable EMU128), verify it
      // with our own CPU detection.
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
      const int64_t sup = hwy::SupportedTargets();
      // SupportedTargets() re-initializes hwy's chosen dispatch target
      // with the full detected set, counting on its caller to narrow it
      // to the returned (possibly DisableTargets-masked) set; do so, or
      // a preceding hwy::DisableTargets() would be ignored (see
      // ojph_transform_hwy.cpp)
      hwy::GetChosenTarget().Update(sup);
      if ((sup & HWY_TARGETS & ~(HWY_EMU128 | HWY_SCALAR)) == 0)
        return;
      rev_convert  = simd_rev_convert;
      rct_forward  = simd_rct_forward;
      rct_backward = simd_rct_backward;
      ict_forward  = simd_ict_forward;
      ict_backward = simd_ict_backward;
      irv_convert_to_integer = simd_irv_convert_to_integer;
      irv_convert_to_float   = simd_irv_convert_to_float;
      irv_convert_to_integer_nlt_type3 =
        simd_irv_convert_to_integer_nlt_type3;
      irv_convert_to_float_nlt_type3 =
        simd_irv_convert_to_float_nlt_type3;
    }

  } // !local namespace
} // !ojph namespace

#endif // HWY_ONCE

#endif // OJPH_ENABLE_HWY
