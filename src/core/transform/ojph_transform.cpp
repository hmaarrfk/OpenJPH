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
// File: ojph_transform.cpp
// Author: Aous Naman
// Date: 28 August 2019
//***************************************************************************/

#include <cstdio>
#include <mutex>

#include "ojph_arch.h"
#include "ojph_mem.h"
#include "ojph_transform.h"
#include "ojph_transform_local.h"
#include "ojph_params.h"
#include "../codestream/ojph_params_local.h"

namespace ojph {

  // defined elsewhere
  class line_buf;

  namespace local {

    /////////////////////////////////////////////////////////////////////////
    // Reversible functions
    /////////////////////////////////////////////////////////////////////////

    /////////////////////////////////////////////////////////////////////////
    void (*rev_vert_step)
      (const lifting_step* s, const line_buf* sig, const line_buf* other,
        const line_buf* aug, ui32 repeat, bool synthesis) = NULL;

    /////////////////////////////////////////////////////////////////////////
    void (*rev_horz_ana)
      (const param_atk* atk, const line_buf* ldst, const line_buf* hdst,
        const line_buf* src, ui32 width, bool even) = NULL;

    /////////////////////////////////////////////////////////////////////////
    void (*rev_horz_syn)
      (const param_atk* atk, const line_buf* dst, const line_buf* lsrc,
        const line_buf* hsrc, ui32 width, bool even) = NULL;
    
    /////////////////////////////////////////////////////////////////////////
    // Irreversible functions
    /////////////////////////////////////////////////////////////////////////

    /////////////////////////////////////////////////////////////////////////
    void (*irv_vert_step)
      (const lifting_step* s, const line_buf* sig, const line_buf* other,
        const line_buf* aug, ui32 repeat, bool synthesis) = NULL;

    /////////////////////////////////////////////////////////////////////////
    void (*irv_vert_times_K)
      (float K, const line_buf* aug, ui32 repeat) = NULL;

    /////////////////////////////////////////////////////////////////////////
    void (*irv_horz_ana)
      (const param_atk* atk, const line_buf* ldst, const line_buf* hdst,
        const line_buf* src, ui32 width, bool even) = NULL;

    /////////////////////////////////////////////////////////////////////////
    void (*irv_horz_syn)
      (const param_atk* atk, const line_buf* dst, const line_buf* lsrc,
        const line_buf* hsrc, ui32 width, bool even) = NULL;

    //////////////////////////////////////////////////////////////////////////
    void init_wavelet_transform_functions()
    {
      static std::once_flag wavelet_transform_functions_init_flag;
      std::call_once(wavelet_transform_functions_init_flag, [](){
#if !defined(OJPH_ENABLE_WASM_SIMD) || !defined(OJPH_EMSCRIPTEN)

        rev_vert_step             = gen_rev_vert_step;
        rev_horz_ana              = gen_rev_horz_ana;
        rev_horz_syn              = gen_rev_horz_syn;

        irv_vert_step             = gen_irv_vert_step;
        irv_vert_times_K          = gen_irv_vert_times_K;
        irv_horz_ana              = gen_irv_horz_ana;
        irv_horz_syn              = gen_irv_horz_syn;

  #ifndef OJPH_DISABLE_SIMD

    #if (defined(OJPH_ARCH_X86_64) || defined(OJPH_ARCH_I386))

      #ifndef OJPH_DISABLE_SSE
        if (get_cpu_ext_level() >= X86_CPU_EXT_LEVEL_SSE)
        {
          irv_vert_step             = sse_irv_vert_step;
          irv_vert_times_K          = sse_irv_vert_times_K;
          irv_horz_ana              = sse_irv_horz_ana;
          irv_horz_syn              = sse_irv_horz_syn;
        }
      #endif // !OJPH_DISABLE_SSE

      #ifndef OJPH_DISABLE_SSE2
        if (get_cpu_ext_level() >= X86_CPU_EXT_LEVEL_SSE2)
        {
          rev_vert_step             = sse2_rev_vert_step;
          rev_horz_ana              = sse2_rev_horz_ana;
          rev_horz_syn              = sse2_rev_horz_syn;
        }
      #endif // !OJPH_DISABLE_SSE2

      #ifndef OJPH_DISABLE_AVX
        if (get_cpu_ext_level() >= X86_CPU_EXT_LEVEL_AVX)
        {
          irv_vert_step             = avx_irv_vert_step;
          irv_vert_times_K          = avx_irv_vert_times_K;
          irv_horz_ana              = avx_irv_horz_ana;      
          irv_horz_syn              = avx_irv_horz_syn;
        }
      #endif // !OJPH_DISABLE_AVX

      #ifndef OJPH_DISABLE_AVX2
        if (get_cpu_ext_level() >= X86_CPU_EXT_LEVEL_AVX2)
        {
          rev_vert_step             = avx2_rev_vert_step;
          rev_horz_ana              = avx2_rev_horz_ana;
          rev_horz_syn              = avx2_rev_horz_syn;
        }
      #endif // !OJPH_DISABLE_AVX2

      #if (defined(OJPH_ARCH_X86_64) && !defined(OJPH_DISABLE_AVX512))
        if (get_cpu_ext_level() >= X86_CPU_EXT_LEVEL_AVX512)
        {
          // rev_vert_step             = avx512_rev_vert_step;
          // rev_horz_ana              = avx512_rev_horz_ana;
          // rev_horz_syn              = avx512_rev_horz_syn;

          irv_vert_step             = avx512_irv_vert_step;
          irv_vert_times_K          = avx512_irv_vert_times_K;
          irv_horz_ana              = avx512_irv_horz_ana;
          irv_horz_syn              = avx512_irv_horz_syn;
        }
      #endif // !OJPH_DISABLE_AVX512
    
    #elif defined(OJPH_ARCH_ARM)

    #elif defined(OJPH_ARCH_PPC64LE)

        if (get_cpu_ext_level() >= PPC_CPU_EXT_LEVEL_ARCH_3_00)
        {
          // 128-bit VSX kernels; see ojph_simd_vsx.h
          rev_vert_step             = vsx_rev_vert_step;
          rev_horz_ana              = vsx_rev_horz_ana;
          rev_horz_syn              = vsx_rev_horz_syn;

          irv_vert_step             = vsx_irv_vert_step;
          irv_vert_times_K          = vsx_irv_vert_times_K;
          irv_horz_ana              = vsx_irv_horz_ana;
          irv_horz_syn              = vsx_irv_horz_syn;
        }

    #endif // !(defined(OJPH_ARCH_X86_64) || defined(OJPH_ARCH_I386))

  #endif // !OJPH_DISABLE_SIMD

#else // OJPH_ENABLE_WASM_SIMD
        rev_vert_step             = wasm_rev_vert_step;
        rev_horz_ana              = wasm_rev_horz_ana;
        rev_horz_syn              = wasm_rev_horz_syn;
        
        irv_vert_step             = wasm_irv_vert_step;
        irv_vert_times_K          = wasm_irv_vert_times_K;
        irv_horz_ana              = wasm_irv_horz_ana;
        irv_horz_syn              = wasm_irv_horz_syn;
#endif // !OJPH_ENABLE_WASM_SIMD
      });
    }
    
    //////////////////////////////////////////////////////////////////////////

#if !defined(OJPH_ENABLE_WASM_SIMD) || !defined(OJPH_EMSCRIPTEN)

    /////////////////////////////////////////////////////////////////////////
    static
    void gen_rev_vert_step32(const lifting_step* s, const line_buf* sig, 
                             const line_buf* other, const line_buf* aug, 
                             ui32 repeat, bool synthesis)
    {
      const si32 a = s->rev.Aatk;
      const si32 b = s->rev.Batk;
      const ui8 e = s->rev.Eatk;

      si32* dst = aug->i32;
      const si32* src1 = sig->i32, * src2 = other->i32;
      // The general definition of the wavelet in Part 2 is slightly 
      // different to part 2, although they are mathematically equivalent
      // here, we identify the simpler form from Part 1 and employ them
      if (a == 1)
      { // 5/3 update and any case with a == 1
        if (synthesis)
          for (ui32 i = repeat; i > 0; --i)
            *dst++ -= (b + *src1++ + *src2++) >> e;
        else
          for (ui32 i = repeat; i > 0; --i)
            *dst++ += (b + *src1++ + *src2++) >> e;
      }
      else if (a == -1 && b == 1 && e == 1)
      { // 5/3 predict
        if (synthesis)
          for (ui32 i = repeat; i > 0; --i)
            *dst++ += (*src1++ + *src2++) >> e;
        else
          for (ui32 i = repeat; i > 0; --i)
            *dst++ -= (*src1++ + *src2++) >> e;
      }
      else if (a == -1)
      { // any case with a == -1, which is not 5/3 predict
        if (synthesis)
          for (ui32 i = repeat; i > 0; --i)
            *dst++ -= (b - (*src1++ + *src2++)) >> e;
        else
          for (ui32 i = repeat; i > 0; --i)
            *dst++ += (b - (*src1++ + *src2++)) >> e;
      }
      else { // general case
        if (synthesis)
          for (ui32 i = repeat; i > 0; --i)
            *dst++ -= (b + a * (*src1++ + *src2++)) >> e;
        else
          for (ui32 i = repeat; i > 0; --i)
            *dst++ += (b + a * (*src1++ + *src2++)) >> e;
      }
    }

    /////////////////////////////////////////////////////////////////////////
    static
    void gen_rev_vert_step64(const lifting_step* s, const line_buf* sig, 
                             const line_buf* other, const line_buf* aug, 
                             ui32 repeat, bool synthesis)
    {
      const si64 a = s->rev.Aatk;
      const si64 b = s->rev.Batk;
      const ui8 e = s->rev.Eatk;

      si64* dst = aug->i64;
      const si64* src1 = sig->i64, * src2 = other->i64;
      // The general definition of the wavelet in Part 2 is slightly 
      // different to part 2, although they are mathematically equivalent
      // here, we identify the simpler form from Part 1 and employ them
      if (a == 1)
      { // 5/3 update and any case with a == 1
        if (synthesis)
          for (ui32 i = repeat; i > 0; --i)
            *dst++ -= (b + *src1++ + *src2++) >> e;
        else
          for (ui32 i = repeat; i > 0; --i)
            *dst++ += (b + *src1++ + *src2++) >> e;
      }
      else if (a == -1 && b == 1 && e == 1)
      { // 5/3 predict
        if (synthesis)
          for (ui32 i = repeat; i > 0; --i)
            *dst++ += (*src1++ + *src2++) >> e;
        else
          for (ui32 i = repeat; i > 0; --i)
            *dst++ -= (*src1++ + *src2++) >> e;
      }
      else if (a == -1)
      { // any case with a == -1, which is not 5/3 predict
        if (synthesis)
          for (ui32 i = repeat; i > 0; --i)
            *dst++ -= (b - (*src1++ + *src2++)) >> e;
        else
          for (ui32 i = repeat; i > 0; --i)
            *dst++ += (b - (*src1++ + *src2++)) >> e;
      }
      else { // general case
        if (synthesis)
          for (ui32 i = repeat; i > 0; --i)
            *dst++ -= (b + a * (*src1++ + *src2++)) >> e;
        else
          for (ui32 i = repeat; i > 0; --i)
            *dst++ += (b + a * (*src1++ + *src2++)) >> e;
      }
    }

    /////////////////////////////////////////////////////////////////////////
    void gen_rev_vert_step(const lifting_step* s, const line_buf* sig, 
                           const line_buf* other, const line_buf* aug, 
                           ui32 repeat, bool synthesis)
    {
      if (((sig != NULL) && (sig->flags & line_buf::LFT_32BIT)) || 
          ((aug != NULL) && (aug->flags & line_buf::LFT_32BIT)) ||
          ((other != NULL) && (other->flags & line_buf::LFT_32BIT))) 
      {
        assert((sig == NULL || sig->flags & line_buf::LFT_32BIT) &&
               (other == NULL || other->flags & line_buf::LFT_32BIT) && 
               (aug == NULL || aug->flags & line_buf::LFT_32BIT));
        gen_rev_vert_step32(s, sig, other, aug, repeat, synthesis);
      }
      else 
      {
        assert((sig == NULL || sig->flags & line_buf::LFT_64BIT) &&
               (other == NULL || other->flags & line_buf::LFT_64BIT) && 
               (aug == NULL || aug->flags & line_buf::LFT_64BIT));
        gen_rev_vert_step64(s, sig, other, aug, repeat, synthesis);
      }
    }

    /////////////////////////////////////////////////////////////////////////
    static
    void gen_rev_horz_ana32(const param_atk* atk, const line_buf* ldst, 
                            const line_buf* hdst, const line_buf* src, 
                            ui32 width, bool even)
    {
      if (width > 1)
      {
        // combine both lsrc and hsrc into dst
        si32* dph = hdst->i32;
        si32* dpl = ldst->i32;
        si32* sp = src->i32;
        ui32 w = width;
        if (!even)
        {
          *dph++ = *sp++; --w;
        }
        for (; w > 1; w -= 2)
        {
          *dpl++ = *sp++; *dph++ = *sp++;
        }
        if (w)
        {
          *dpl++ = *sp++; --w;
        }

        si32* hp = hdst->i32, * lp = ldst->i32;
        ui32 l_width = (width + (even ? 1 : 0)) >> 1;  // low pass
        ui32 h_width = (width + (even ? 0 : 1)) >> 1;  // high pass
        ui32 num_steps = atk->get_num_steps();
        for (ui32 j = num_steps; j > 0; --j)
        {
          // first lifting step
          const lifting_step* s = atk->get_step(j - 1);
          const si32 a = s->rev.Aatk;
          const si32 b = s->rev.Batk;
          const ui8 e = s->rev.Eatk;

          // extension
          lp[-1] = lp[0];
          lp[l_width] = lp[l_width - 1];
          // lifting step
          const si32* sp = lp + (even ? 1 : 0);
          si32* dp = hp;
          if (a == 1) 
          { // 5/3 update and any case with a == 1
            for (ui32 i = h_width; i > 0; --i, sp++, dp++)
              *dp += (b + (sp[-1] + sp[0])) >> e;
          }
          else if (a == -1 && b == 1 && e == 1)
          {  // 5/3 predict
            for (ui32 i = h_width; i > 0; --i, sp++, dp++)
              *dp -= (sp[-1] + sp[0]) >> e;
          }
          else if (a == -1)
          { // any case with a == -1, which is not 5/3 predict
            for (ui32 i = h_width; i > 0; --i, sp++, dp++)
              *dp += (b - (sp[-1] + sp[0])) >> e;
          }
          else {
            // general case
            for (ui32 i = h_width; i > 0; --i, sp++, dp++)
              *dp += (b + a * (sp[-1] + sp[0])) >> e;
          }

          // swap buffers
          si32* t = lp; lp = hp; hp = t;
          even = !even;
          ui32 w = l_width; l_width = h_width; h_width = w;
        }
      }
      else {
        if (even)
          ldst->i32[0] = src->i32[0];
        else
          hdst->i32[0] = src->i32[0] << 1;
      }
    }

    /////////////////////////////////////////////////////////////////////////
    static
    void gen_rev_horz_ana64(const param_atk* atk, const line_buf* ldst, 
                            const line_buf* hdst, const line_buf* src, 
                            ui32 width, bool even)
    {
      if (width > 1)
      {
        // combine both lsrc and hsrc into dst
        si64* dph = hdst->i64;
        si64* dpl = ldst->i64;
        si64* sp = src->i64;
        ui32 w = width;
        if (!even)
        {
          *dph++ = *sp++; --w;
        }
        for (; w > 1; w -= 2)
        {
          *dpl++ = *sp++; *dph++ = *sp++;
        }
        if (w)
        {
          *dpl++ = *sp++; --w;
        }

        si64* hp = hdst->i64, * lp = ldst->i64;
        ui32 l_width = (width + (even ? 1 : 0)) >> 1;  // low pass
        ui32 h_width = (width + (even ? 0 : 1)) >> 1;  // high pass
        ui32 num_steps = atk->get_num_steps();
        for (ui32 j = num_steps; j > 0; --j)
        {
          // first lifting step
          const lifting_step* s = atk->get_step(j - 1);
          const si64 a = s->rev.Aatk;
          const si64 b = s->rev.Batk;
          const ui8 e = s->rev.Eatk;

          // extension
          lp[-1] = lp[0];
          lp[l_width] = lp[l_width - 1];
          // lifting step
          const si64* sp = lp + (even ? 1 : 0);
          si64* dp = hp;
          if (a == 1) 
          { // 5/3 update and any case with a == 1
            for (ui32 i = h_width; i > 0; --i, sp++, dp++)
              *dp += (b + (sp[-1] + sp[0])) >> e;
          }
          else if (a == -1 && b == 1 && e == 1)
          {  // 5/3 predict
            for (ui32 i = h_width; i > 0; --i, sp++, dp++)
              *dp -= (sp[-1] + sp[0]) >> e;
          }
          else if (a == -1)
          { // any case with a == -1, which is not 5/3 predict
            for (ui32 i = h_width; i > 0; --i, sp++, dp++)
              *dp += (b - (sp[-1] + sp[0])) >> e;
          }
          else {
            // general case
            for (ui32 i = h_width; i > 0; --i, sp++, dp++)
              *dp += (b + a * (sp[-1] + sp[0])) >> e;
          }

          // swap buffers
          si64* t = lp; lp = hp; hp = t;
          even = !even;
          ui32 w = l_width; l_width = h_width; h_width = w;
        }
      }
      else {
        if (even)
          ldst->i64[0] = src->i64[0];
        else
          hdst->i64[0] = src->i64[0] << 1;
      }
    }

    /////////////////////////////////////////////////////////////////////////
    void gen_rev_horz_ana(const param_atk* atk, const line_buf* ldst, 
                          const line_buf* hdst, const line_buf* src, 
                          ui32 width, bool even)
    {
      if (src->flags & line_buf::LFT_32BIT) 
      {
        assert((ldst == NULL || ldst->flags & line_buf::LFT_32BIT) &&
               (hdst == NULL || hdst->flags & line_buf::LFT_32BIT));
        gen_rev_horz_ana32(atk, ldst, hdst, src, width, even);
      }
      else 
      {
        assert((ldst == NULL || ldst->flags & line_buf::LFT_64BIT) &&
               (hdst == NULL || hdst->flags & line_buf::LFT_64BIT) && 
               (src == NULL || src->flags & line_buf::LFT_64BIT));
        gen_rev_horz_ana64(atk, ldst, hdst, src, width, even);
      }
    }

    //////////////////////////////////////////////////////////////////////////
    //
    // Reversible functions for arbitrary (ARB) kernels with a single
    // lifting coefficient per step.  The lifting equation implemented here
    // is that of T.801 (H-3); the source sample of the single tap for the
    // sample at index i of the updated subsequence is derived from the
    // step offset Oatk, the parity of the first sample of the segment,
    // and the parity of the subsequence the step updates.  Boundary
    // extension is constant (T.801 H.2.4.3), which replicates the nearest
    // sample of the same parity; on the subsequences used here, this is
    // the replication of the first or last sample of the subsequence.
    //
    //////////////////////////////////////////////////////////////////////////

    //////////////////////////////////////////////////////////////////////////
    template <typename T>
    static
    void gen_rev_horz_ana_arb_T(const param_atk* atk, T* lp, T* hp, T* srcp,
                                ui32 width, bool even)
    {
      // split src into low and high subsequences
      T* dph = hp;
      T* dpl = lp;
      T* sp = srcp;
      ui32 w = width;
      if (!even)
      {
        *dph++ = *sp++; --w;
      }
      for (; w > 1; w -= 2)
      {
        *dpl++ = *sp++; *dph++ = *sp++;
      }
      if (w)
      {
        *dpl++ = *sp++; --w;
      }

      ui32 l_width = (width + (even ? 1 : 0)) >> 1;  // low pass
      ui32 h_width = (width + (even ? 0 : 1)) >> 1;  // high pass
      ui32 num_steps = atk->get_num_steps();
      for (ui32 j = num_steps; j > 0; --j)
      {
        const lifting_step* s = atk->get_step(j - 1);
        const T a = s->rev.Aatk;
        const T b = s->rev.Batk;
        const ui8 e = s->rev.Eatk;
        // with m_init = 0, steps at odd positions (in the signalled,
        // reconstruction order) update the odd-indexed subsequence
        const bool updates_odd = ((j - 1) & 1) == 1;

        // constant extension of the source subsequence
        lp[-1] = lp[0];
        lp[l_width] = lp[l_width - 1];
        // one-tap lifting step
        const T* sp = lp + s->rev.Oatk
                    + (even ? 1 : 0) - (updates_odd ? 1 : 0);
        T* dp = hp;
        if (a == -1 && b == 0 && e == 0)
        { // previous-sample predict
          for (ui32 i = h_width; i > 0; --i, sp++, dp++)
            *dp -= sp[0];
        }
        else if (a == 0)
        { // null step; only the constant contributes, if anything
          const T v = (T)(b >> e);
          if (v != 0)
            for (ui32 i = h_width; i > 0; --i, dp++)
              *dp += v;
        }
        else
        { // general case
          for (ui32 i = h_width; i > 0; --i, sp++, dp++)
            *dp = (T)(*dp + ((b + a * sp[0]) >> e));
        }

        // swap buffers
        T* t = lp; lp = hp; hp = t;
        even = !even;
        ui32 w = l_width; l_width = h_width; h_width = w;
      }
    }

    //////////////////////////////////////////////////////////////////////////
    // True when the kernel is the two-step previous-sample predict-only
    // kernel (a null update step, then H = odd - preceding even), for which
    // fused single-pass transforms are provided below.
    static bool is_fused_prev_sample_kernel(const param_atk* atk)
    {
      if (atk->get_num_steps() != 2)
        return false;
      const lifting_step* s0 = atk->get_step(0);
      const lifting_step* s1 = atk->get_step(1);
      return s0->rev.Aatk == 0 && (s0->rev.Batk >> s0->rev.Eatk) == 0 &&
             s1->rev.Aatk == -1 && s1->rev.Batk == 0 && s1->rev.Eatk == 0 &&
             s1->rev.Oatk == 0;
    }

    //////////////////////////////////////////////////////////////////////////
    // Fused analysis for the previous-sample kernel: the deinterleaving and
    // the one-tap prediction are performed in a single pass over the input.
    template <typename T>
    static
    void gen_rev_horz_ana_prev_T(T* lp, T* hp, const T* sp,
                                 ui32 width, bool even)
    {
      if (even)
      { // first sample is low-pass: H[i] = src[2i+1] - src[2i]
        ui32 l_width = (width + 1) >> 1;
        ui32 h_width = width >> 1;
        for (ui32 i = 0; i < h_width; ++i) {
          lp[i] = sp[2 * i];
          hp[i] = sp[2 * i + 1] - sp[2 * i];
        }
        if (l_width > h_width)
          lp[l_width - 1] = sp[width - 1];
      }
      else
      { // first sample is high-pass; its preceding even sample is beyond
        // the boundary, and constant extension replicates the nearest
        // even-indexed sample, src[1]: H[0] = src[0] - src[1]
        ui32 h_width = (width + 1) >> 1;
        hp[0] = sp[0] - sp[1];
        for (ui32 i = 1; i < h_width; ++i) {
          lp[i - 1] = sp[2 * i - 1];
          hp[i] = sp[2 * i] - sp[2 * i - 1];
        }
        if ((width & 1) == 0)
          lp[(width >> 1) - 1] = sp[width - 1];
      }
    }

    //////////////////////////////////////////////////////////////////////////
    void rev_horz_ana_arb(const param_atk* atk, const line_buf* ldst,
                          const line_buf* hdst, const line_buf* src,
                          ui32 width, bool even)
    {
      if (width > 1)
      {
        if (is_fused_prev_sample_kernel(atk))
        {
          if (src->flags & line_buf::LFT_16BIT)
            gen_rev_horz_ana_prev_T<si16>(ldst->i16, hdst->i16, src->i16,
                                          width, even);
          else if (src->flags & line_buf::LFT_32BIT)
            gen_rev_horz_ana_prev_T<si32>(ldst->i32, hdst->i32, src->i32,
                                          width, even);
          else
            gen_rev_horz_ana_prev_T<si64>(ldst->i64, hdst->i64, src->i64,
                                          width, even);
        }
        else if (src->flags & line_buf::LFT_16BIT)
          gen_rev_horz_ana_arb_T<si16>(atk, ldst->i16, hdst->i16, src->i16,
                                       width, even);
        else if (src->flags & line_buf::LFT_32BIT)
          gen_rev_horz_ana_arb_T<si32>(atk, ldst->i32, hdst->i32, src->i32,
                                       width, even);
        else
          gen_rev_horz_ana_arb_T<si64>(atk, ldst->i64, hdst->i64, src->i64,
                                       width, even);
      }
      else
      {
        if (src->flags & line_buf::LFT_16BIT) {
          if (even)
            ldst->i16[0] = src->i16[0];
          else
            hdst->i16[0] = (si16)(src->i16[0] << 1);
        }
        else if (src->flags & line_buf::LFT_32BIT) {
          if (even)
            ldst->i32[0] = src->i32[0];
          else
            hdst->i32[0] = src->i32[0] << 1;
        }
        else {
          if (even)
            ldst->i64[0] = src->i64[0];
          else
            hdst->i64[0] = src->i64[0] << 1;
        }
      }
    }

    //////////////////////////////////////////////////////////////////////////
    template <typename T>
    static
    void gen_rev_horz_syn_arb_T(const param_atk* atk, T* dstp, T* lp, T* hp,
                                ui32 width, bool even)
    {
      bool ev = even;
      T* oth = hp, * aug = lp;
      ui32 aug_width = (width + (even ? 1 : 0)) >> 1;  // low pass
      ui32 oth_width = (width + (even ? 0 : 1)) >> 1;  // high pass
      ui32 num_steps = atk->get_num_steps();
      for (ui32 j = 0; j < num_steps; ++j)
      {
        const lifting_step* s = atk->get_step(j);
        const T a = s->rev.Aatk;
        const T b = s->rev.Batk;
        const ui8 e = s->rev.Eatk;
        // with m_init = 0, steps at odd positions (in the signalled,
        // reconstruction order) update the odd-indexed subsequence
        const bool updates_odd = (j & 1) == 1;

        // constant extension of the source subsequence
        oth[-1] = oth[0];
        oth[oth_width] = oth[oth_width - 1];
        // one-tap lifting step
        const T* sp = oth + s->rev.Oatk
                    + (updates_odd ? 0 : 1) - (ev ? 1 : 0);
        T* dp = aug;
        if (a == -1 && b == 0 && e == 0)
        { // previous-sample predict
          for (ui32 i = aug_width; i > 0; --i, sp++, dp++)
            *dp += sp[0];
        }
        else if (a == 0)
        { // null step; only the constant contributes, if anything
          const T v = (T)(b >> e);
          if (v != 0)
            for (ui32 i = aug_width; i > 0; --i, dp++)
              *dp -= v;
        }
        else
        { // general case
          for (ui32 i = aug_width; i > 0; --i, sp++, dp++)
            *dp = (T)(*dp - ((b + a * sp[0]) >> e));
        }

        // swap buffers
        T* t = aug; aug = oth; oth = t;
        ev = !ev;
        ui32 w = aug_width; aug_width = oth_width; oth_width = w;
      }

      // combine both low and high subsequences into dst
      T* sph = hp;
      T* spl = lp;
      T* dp = dstp;
      ui32 w = width;
      if (!even)
      {
        *dp++ = *sph++; --w;
      }
      for (; w > 1; w -= 2)
      {
        *dp++ = *spl++; *dp++ = *sph++;
      }
      if (w)
      {
        *dp++ = *spl++; --w;
      }
    }

    //////////////////////////////////////////////////////////////////////////
    // Fused synthesis for the previous-sample kernel; the inverse of
    // gen_rev_horz_ana_prev_T, interleaving in the same single pass.
    template <typename T>
    static
    void gen_rev_horz_syn_prev_T(T* dp, const T* lp, const T* hp,
                                 ui32 width, bool even)
    {
      if (even)
      {
        ui32 h_width = width >> 1;
        ui32 l_width = (width + 1) >> 1;
        for (ui32 i = 0; i < h_width; ++i) {
          dp[2 * i] = lp[i];
          dp[2 * i + 1] = hp[i] + lp[i];
        }
        if (l_width > h_width)
          dp[width - 1] = lp[l_width - 1];
      }
      else
      {
        ui32 h_width = (width + 1) >> 1;
        dp[0] = hp[0] + lp[0];
        for (ui32 i = 1; i < h_width; ++i) {
          dp[2 * i - 1] = lp[i - 1];
          dp[2 * i] = hp[i] + lp[i - 1];
        }
        if ((width & 1) == 0)
          dp[width - 1] = lp[(width >> 1) - 1];
      }
    }

    //////////////////////////////////////////////////////////////////////////
    void rev_horz_syn_arb(const param_atk* atk, const line_buf* dst,
                          const line_buf* lsrc, const line_buf* hsrc,
                          ui32 width, bool even)
    {
      if (width > 1)
      {
        if (is_fused_prev_sample_kernel(atk))
        {
          if (dst->flags & line_buf::LFT_16BIT)
            gen_rev_horz_syn_prev_T<si16>(dst->i16, lsrc->i16, hsrc->i16,
                                          width, even);
          else if (dst->flags & line_buf::LFT_32BIT)
            gen_rev_horz_syn_prev_T<si32>(dst->i32, lsrc->i32, hsrc->i32,
                                          width, even);
          else
            gen_rev_horz_syn_prev_T<si64>(dst->i64, lsrc->i64, hsrc->i64,
                                          width, even);
        }
        else if (dst->flags & line_buf::LFT_16BIT)
          gen_rev_horz_syn_arb_T<si16>(atk, dst->i16, lsrc->i16, hsrc->i16,
                                       width, even);
        else if (dst->flags & line_buf::LFT_32BIT)
          gen_rev_horz_syn_arb_T<si32>(atk, dst->i32, lsrc->i32, hsrc->i32,
                                       width, even);
        else
          gen_rev_horz_syn_arb_T<si64>(atk, dst->i64, lsrc->i64, hsrc->i64,
                                       width, even);
      }
      else
      {
        if (dst->flags & line_buf::LFT_16BIT) {
          if (even)
            dst->i16[0] = lsrc->i16[0];
          else
            dst->i16[0] = (si16)(hsrc->i16[0] >> 1);
        }
        else if (dst->flags & line_buf::LFT_32BIT) {
          if (even)
            dst->i32[0] = lsrc->i32[0];
          else
            dst->i32[0] = hsrc->i32[0] >> 1;
        }
        else {
          if (even)
            dst->i64[0] = lsrc->i64[0];
          else
            dst->i64[0] = hsrc->i64[0] >> 1;
        }
      }
    }

    //////////////////////////////////////////////////////////////////////////
    template <typename T>
    static
    void gen_rev_vert_step_one_tap_T(const lifting_step* s, const T* sp,
                                     T* dst, ui32 repeat, bool synthesis)
    {
      const T a = s->rev.Aatk;
      const T b = s->rev.Batk;
      const ui8 e = s->rev.Eatk;

      if (a == -1 && b == 0 && e == 0)
      { // previous-sample predict
        if (synthesis)
          for (ui32 i = repeat; i > 0; --i)
            *dst++ += *sp++;
        else
          for (ui32 i = repeat; i > 0; --i)
            *dst++ -= *sp++;
      }
      else if (a == 0)
      { // null step; only the constant contributes, if anything
        const T v = (T)(b >> e);
        if (v != 0)
        {
          if (synthesis)
            for (ui32 i = repeat; i > 0; --i)
              *dst++ -= v;
          else
            for (ui32 i = repeat; i > 0; --i)
              *dst++ += v;
        }
      }
      else
      { // general case
        if (synthesis)
          for (ui32 i = repeat; i > 0; --i, ++dst, ++sp)
            *dst = (T)(*dst - ((b + a * *sp) >> e));
        else
          for (ui32 i = repeat; i > 0; --i, ++dst, ++sp)
            *dst = (T)(*dst + ((b + a * *sp) >> e));
      }
    }

    //////////////////////////////////////////////////////////////////////////
    void rev_vert_step_one_tap(const lifting_step* s, const line_buf* src,
                               const line_buf* aug, ui32 repeat,
                               bool synthesis)
    {
      if (aug->flags & line_buf::LFT_16BIT)
      {
        assert(src == NULL || src->flags & line_buf::LFT_16BIT);
        gen_rev_vert_step_one_tap_T<si16>(s, src->i16, aug->i16, repeat,
                                          synthesis);
      }
      else if (aug->flags & line_buf::LFT_32BIT)
      {
        assert(src == NULL || src->flags & line_buf::LFT_32BIT);
        gen_rev_vert_step_one_tap_T<si32>(s, src->i32, aug->i32, repeat,
                                          synthesis);
      }
      else
      {
        assert((aug->flags & line_buf::LFT_64BIT) &&
               (src == NULL || src->flags & line_buf::LFT_64BIT));
        gen_rev_vert_step_one_tap_T<si64>(s, src->i64, aug->i64, repeat,
                                          synthesis);
      }
    }

    //////////////////////////////////////////////////////////////////////////
    static
    void gen_rev_horz_syn32(const param_atk* atk, const line_buf* dst,
                            const line_buf* lsrc, const line_buf* hsrc, 
                            ui32 width, bool even)
    {
      if (width > 1)
      {
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
          si32* dp = aug;
          if (a == 1)
          { // 5/3 update and any case with a == 1
            for (ui32 i = aug_width; i > 0; --i, sp++, dp++)
              *dp -= (b + (sp[-1] + sp[0])) >> e;
          }
          else if (a == -1 && b == 1 && e == 1)
          {  // 5/3 predict
            for (ui32 i = aug_width; i > 0; --i, sp++, dp++)
              *dp += (sp[-1] + sp[0]) >> e;
          }
          else if (a == -1)
          { // any case with a == -1, which is not 5/3 predict
            for (ui32 i = aug_width; i > 0; --i, sp++, dp++)
              *dp -= (b - (sp[-1] + sp[0])) >> e;
          }
          else {
            // general case
            for (ui32 i = aug_width; i > 0; --i, sp++, dp++)
              *dp -= (b + a * (sp[-1] + sp[0])) >> e;
          }

          // swap buffers
          si32* t = aug; aug = oth; oth = t;
          ev = !ev;
          ui32 w = aug_width; aug_width = oth_width; oth_width = w;
        }

        // combine both lsrc and hsrc into dst
        si32* sph = hsrc->i32;
        si32* spl = lsrc->i32;
        si32* dp = dst->i32;
        ui32 w = width;
        if (!even)
        {
          *dp++ = *sph++; --w;
        }
        for (; w > 1; w -= 2)
        {
          *dp++ = *spl++; *dp++ = *sph++;
        }
        if (w)
        {
          *dp++ = *spl++; --w;
        }
      }
      else {
        if (even)
          dst->i32[0] = lsrc->i32[0];
        else
          dst->i32[0] = hsrc->i32[0] >> 1;
      }
    }

    //////////////////////////////////////////////////////////////////////////
    static
    void gen_rev_horz_syn64(const param_atk* atk, const line_buf* dst, 
                            const line_buf* lsrc, const line_buf* hsrc, 
                            ui32 width, bool even)
    {
      if (width > 1)
      {
        bool ev = even;
        si64* oth = hsrc->i64, * aug = lsrc->i64;
        ui32 aug_width = (width + (even ? 1 : 0)) >> 1;  // low pass
        ui32 oth_width = (width + (even ? 0 : 1)) >> 1;  // high pass
        ui32 num_steps = atk->get_num_steps();
        for (ui32 j = 0; j < num_steps; ++j)
        {
          const lifting_step* s = atk->get_step(j);
          const si64 a = s->rev.Aatk;
          const si64 b = s->rev.Batk;
          const ui8 e = s->rev.Eatk;

          // extension
          oth[-1] = oth[0];
          oth[oth_width] = oth[oth_width - 1];
          // lifting step
          const si64* sp = oth + (ev ? 0 : 1);
          si64* dp = aug;
          if (a == 1)
          { // 5/3 update and any case with a == 1
            for (ui32 i = aug_width; i > 0; --i, sp++, dp++)
              *dp -= (b + (sp[-1] + sp[0])) >> e;
          }
          else if (a == -1 && b == 1 && e == 1)
          {  // 5/3 predict
            for (ui32 i = aug_width; i > 0; --i, sp++, dp++)
              *dp += (sp[-1] + sp[0]) >> e;
          }
          else if (a == -1)
          { // any case with a == -1, which is not 5/3 predict
            for (ui32 i = aug_width; i > 0; --i, sp++, dp++)
              *dp -= (b - (sp[-1] + sp[0])) >> e;
          }
          else {
            // general case
            for (ui32 i = aug_width; i > 0; --i, sp++, dp++)
              *dp -= (b + a * (sp[-1] + sp[0])) >> e;
          }

          // swap buffers
          si64* t = aug; aug = oth; oth = t;
          ev = !ev;
          ui32 w = aug_width; aug_width = oth_width; oth_width = w;
        }

        // combine both lsrc and hsrc into dst
        si64* sph = hsrc->i64;
        si64* spl = lsrc->i64;
        si64* dp = dst->i64;
        ui32 w = width;
        if (!even)
        {
          *dp++ = *sph++; --w;
        }
        for (; w > 1; w -= 2)
        {
          *dp++ = *spl++; *dp++ = *sph++;
        }
        if (w)
        {
          *dp++ = *spl++; --w;
        }
      }
      else {
        if (even)
          dst->i64[0] = lsrc->i64[0];
        else
          dst->i64[0] = hsrc->i64[0] >> 1;
      }
    }

    /////////////////////////////////////////////////////////////////////////
    void gen_rev_horz_syn(const param_atk* atk, const line_buf* dst, 
                          const line_buf* lsrc, const line_buf* hsrc, 
                          ui32 width, bool even)
    {
      if (dst->flags & line_buf::LFT_32BIT) 
      {
        assert((lsrc == NULL || lsrc->flags & line_buf::LFT_32BIT) && 
               (hsrc == NULL || hsrc->flags & line_buf::LFT_32BIT));
        gen_rev_horz_syn32(atk, dst, lsrc, hsrc, width, even);
      }
      else 
      {
        assert((dst == NULL || dst->flags & line_buf::LFT_64BIT) &&
               (lsrc == NULL || lsrc->flags & line_buf::LFT_64BIT) && 
               (hsrc == NULL || hsrc->flags & line_buf::LFT_64BIT));
        gen_rev_horz_syn64(atk, dst, lsrc, hsrc, width, even);
      }
    }    

    //////////////////////////////////////////////////////////////////////////
    void gen_irv_vert_step(const lifting_step* s, const line_buf* sig, 
                           const line_buf* other, const line_buf* aug, 
                           ui32 repeat, bool synthesis)
    {
      float a = s->irv.Aatk;

      if (synthesis)
        a = -a;

      float* dst = aug->f32;
      const float* src1 = sig->f32, * src2 = other->f32;
      for (ui32 i = repeat; i > 0; --i)
        *dst++ += a * (*src1++ + *src2++);
    }

    //////////////////////////////////////////////////////////////////////////
    void gen_irv_vert_times_K(float K, const line_buf* aug, ui32 repeat)
    {
      float* dst = aug->f32;
      for (ui32 i = repeat; i > 0; --i)
        *dst++ *= K;
    }

    /////////////////////////////////////////////////////////////////////////
    void gen_irv_horz_ana(const param_atk* atk, const line_buf* ldst, 
                          const line_buf* hdst, const line_buf* src, 
                          ui32 width, bool even)
    {
      if (width > 1)
      {
        // split src into ldst and hdst
        float* dph = hdst->f32;
        float* dpl = ldst->f32;
        float* sp = src->f32;
        ui32 w = width;
        if (!even)
        {
          *dph++ = *sp++; --w;
        }
        for (; w > 1; w -= 2)
        {
          *dpl++ = *sp++; *dph++ = *sp++;
        }
        if (w)
        {
          *dpl++ = *sp++; --w;
        }

        float* hp = hdst->f32, * lp = ldst->f32;
        ui32 l_width = (width + (even ? 1 : 0)) >> 1;  // low pass
        ui32 h_width = (width + (even ? 0 : 1)) >> 1;  // high pass
        ui32 num_steps = atk->get_num_steps();
        for (ui32 j = num_steps; j > 0; --j)
        {
          const lifting_step* s = atk->get_step(j - 1);
          const float a = s->irv.Aatk;

          // extension
          lp[-1] = lp[0];
          lp[l_width] = lp[l_width - 1];
          // lifting step
          const float* sp = lp + (even ? 1 : 0);
          float* dp = hp;
          for (ui32 i = h_width; i > 0; --i, sp++, dp++)
            *dp += a * (sp[-1] + sp[0]);

          // swap buffers
          float* t = lp; lp = hp; hp = t;
          even = !even;
          ui32 w = l_width; l_width = h_width; h_width = w;
        }

        {
          float K = atk->get_K();
          float K_inv = 1.0f / K;
          float* dp;

          dp = lp;
          for (ui32 i = l_width; i > 0; --i)
            *dp++ *= K_inv;

          dp = hp;
          for (ui32 i = h_width; i > 0; --i)
            *dp++ *= K;
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
    void gen_irv_horz_syn(const param_atk* atk, const line_buf* dst, 
                          const line_buf* lsrc, const line_buf* hsrc, 
                          ui32 width, bool even)
    {
      if (width > 1)
      {
        bool ev = even;
        float* oth = hsrc->f32, * aug = lsrc->f32;
        ui32 aug_width = (width + (even ? 1 : 0)) >> 1;  // low pass
        ui32 oth_width = (width + (even ? 0 : 1)) >> 1;  // high pass

        {
          float K = atk->get_K();
          float K_inv = 1.0f / K;
          float* dp;

          dp = aug;
          for (ui32 i = aug_width; i > 0; --i)
            *dp++ *= K;

          dp = oth;
          for (ui32 i = oth_width; i > 0; --i)
            *dp++ *= K_inv;
        }

        ui32 num_steps = atk->get_num_steps();
        for (ui32 j = 0; j < num_steps; ++j)
        {
          const lifting_step* s = atk->get_step(j);
          const float a = s->irv.Aatk;

          // extension
          oth[-1] = oth[0];
          oth[oth_width] = oth[oth_width - 1];
          // lifting step
          const float* sp = oth + (ev ? 0 : 1);
          float* dp = aug;
          for (ui32 i = aug_width; i > 0; --i, sp++, dp++)
            *dp -= a * (sp[-1] + sp[0]);

          // swap buffers
          float* t = aug; aug = oth; oth = t;
          ev = !ev;
          ui32 w = aug_width; aug_width = oth_width; oth_width = w;
        }

        // combine both lsrc and hsrc into dst
        float* sph = hsrc->f32;
        float* spl = lsrc->f32;
        float* dp = dst->f32;
        ui32 w = width;
        if (!even)
        { *dp++ = *sph++; --w; }
        for (; w > 1; w -= 2)
        { *dp++ = *spl++; *dp++ = *sph++; }
        if (w)
        { *dp++ = *spl++; --w; }
      }
      else {
        if (even)
          dst->f32[0] = lsrc->f32[0];
        else
          dst->f32[0] = hsrc->f32[0] * 0.5f;
      }
    }

#endif // !OJPH_ENABLE_WASM_SIMD

  }
}
