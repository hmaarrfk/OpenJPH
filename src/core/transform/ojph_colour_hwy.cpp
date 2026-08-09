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
// ICT).  Like ojph_transform_hwy.cpp, this translation unit is compiled
// for a single (static) Highway target and installed only when the CPU
// supports it; it is compiled with -ffp-contract=off, so the float
// (ICT) results are identical to the generic implementation.

// Highway must be included before any ojph header, because ojph_defs.h
// renames the ojph namespace token.
#include <hwy/highway.h>

#include "ojph_arch.h"

#ifdef OJPH_ENABLE_HWY

#include <cassert>

#include "ojph_defs.h"
#include "ojph_mem.h"
#include "ojph_colour.h"
#include "ojph_colour_local.h"

namespace hn = hwy::HWY_NAMESPACE;

namespace ojph {
  namespace local {

    //////////////////////////////////////////////////////////////////////////
    // Forward RCT; values identical to gen_rct_forward.  Like all the
    // loops in this file, the loops overrun the line ends by less than
    // one vector; the lines are padded (the SSE2/AVX2 implementations
    // rely on the same).
    static
    void hwy_rct_forward(
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
    static
    void hwy_rct_backward(
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
    static
    void hwy_ict_forward(const float *r, const float *g, const float *b,
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
    static
    void hwy_ict_backward(const float *y, const float *cb, const float *cr,
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

    //////////////////////////////////////////////////////////////////////////
    void hwy_install_colour_transforms()
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
      rct_forward  = hwy_rct_forward;
      rct_backward = hwy_rct_backward;
      ict_forward  = hwy_ict_forward;
      ict_backward = hwy_ict_backward;
    }

  } // !local namespace
} // !ojph namespace

#endif // OJPH_ENABLE_HWY
