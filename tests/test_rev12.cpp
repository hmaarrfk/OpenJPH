//***************************************************************************/
// This software is released under the 2-Clause BSD license, included
// below.
//
// Copyright (c) Mark Harfouche
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
// File: test_rev12.cpp
// Author: Mark Harfouche
// Date: 8 August 2026
//***************************************************************************/

// Tests for the rev12 (previous-sample predict-only, 1/2) wavelet,
// signaled with a Part 2 ATK marker segment carrying an arbitrary (ARB)
// filter.  Like the rev13 kernel, the low-pass subband of each
// decomposition holds the even-indexed samples of the previous resolution
// untouched, so decoding r skipped resolutions returns exactly
// image[::2^r, ::2^r]; unlike rev13, the high-pass subband holds exact
// previous-sample differences, X(2n+1) - X(2n).

#include <vector>

#include "ojph_mem.h"
#include "ojph_file.h"
#include "ojph_codestream.h"
#include "ojph_params.h"
#include "gtest/gtest.h"

namespace {

// mask-like content (a disc on a background) plus scattered impulses, so
// both smooth and detail subbands are exercised
static ojph::si32 sample_value(ojph::ui32 x, ojph::ui32 y,
                               ojph::ui32 width, ojph::ui32 height)
{
  ojph::si32 dx = (ojph::si32)x - (ojph::si32)(width / 2);
  ojph::si32 dy = (ojph::si32)y - (ojph::si32)(height / 2);
  ojph::si32 r = (ojph::si32)(width / 3);
  ojph::si32 v = (dx * dx + dy * dy < r * r) ? 137 : 0;
  if ((x * 31u + y * 17u) % 97u == 0)
    v = (ojph::si32)((x + y) % 200u);
  return v;
}

static void encode(ojph::mem_outfile& file,
                   ojph::ui32 width, ojph::ui32 height,
                   ojph::ui32 num_decomps, ojph::ui32 kernel,
                   const std::vector<ojph::si32>* samples = NULL)
{
  ojph::codestream codestream;

  ojph::param_siz siz = codestream.access_siz();
  siz.set_image_extent(ojph::point(width, height));
  siz.set_num_components(1);
  siz.set_component(0, ojph::point(1, 1), 8, false);
  siz.set_image_offset(ojph::point(0, 0));
  siz.set_tile_offset(ojph::point(0, 0));

  ojph::param_cod cod = codestream.access_cod();
  cod.set_num_decomposition(num_decomps);
  cod.set_reversible(true);
  cod.set_wavelet_kern(kernel);
  codestream.set_planar(false);

  file.open();
  codestream.write_headers(&file);

  ojph::ui32 next_comp;
  ojph::line_buf* cur_line = codestream.exchange(NULL, next_comp);
  for (ojph::ui32 y = 0; y < height; ++y)
  {
    for (ojph::ui32 x = 0; x < width; ++x)
      cur_line->i32[x] = samples != NULL
        ? (*samples)[(size_t)y * width + x]
        : sample_value(x, y, width, height);
    cur_line = codestream.exchange(cur_line, next_comp);
  }
  codestream.flush();
  codestream.close();
}

static std::vector<ojph::si32> decode(ojph::mem_outfile& file,
                                      ojph::ui32 skipped_res,
                                      ojph::ui32& recon_width,
                                      ojph::ui32& recon_height)
{
  ojph::mem_infile infile;
  infile.open(file.get_data(), file.get_used_size());

  ojph::codestream codestream;
  codestream.read_headers(&infile);
  codestream.restrict_input_resolution(skipped_res, skipped_res);
  codestream.create();

  ojph::param_siz siz = codestream.access_siz();
  recon_width = siz.get_recon_width(0);
  recon_height = siz.get_recon_height(0);

  std::vector<ojph::si32> result((size_t)recon_width * recon_height);
  for (ojph::ui32 y = 0; y < recon_height; ++y)
  {
    ojph::ui32 comp_num;
    ojph::line_buf* line = codestream.pull(comp_num);
    for (ojph::ui32 x = 0; x < recon_width; ++x)
      result[(size_t)y * recon_width + x] = line->i32[x];
  }
  codestream.close();
  return result;
}

} // namespace

///////////////////////////////////////////////////////////////////////////////
// Full-resolution decoding must be lossless, and decoding r skipped
// resolutions must return exactly image[::2^r, ::2^r] -- for even and odd
// dimensions, including single-row and single-column images.
TEST(TestRev12, LevelsAreExactSubsampling) {
  const struct { ojph::ui32 width, height, num_decomps; } cases[] = {
    { 256, 256, 3 },
    { 255, 193, 3 },
    {  17,  13, 2 },
    {  64, 256, 4 },
    {   1,  64, 2 },
    {  64,   1, 2 },
  };

  for (auto& c : cases)
  {
    ojph::mem_outfile file;
    encode(file, c.width, c.height, c.num_decomps,
           ojph::param_cod::OJPH_WAVELET_REV12);

    for (ojph::ui32 skip = 0; skip <= c.num_decomps; ++skip)
    {
      ojph::ui32 rw, rh;
      std::vector<ojph::si32> recon = decode(file, skip, rw, rh);

      ojph::ui32 step = 1u << skip;
      ASSERT_EQ(rw, (c.width + step - 1) / step)
        << c.width << "x" << c.height << " skip " << skip;
      ASSERT_EQ(rh, (c.height + step - 1) / step)
        << c.width << "x" << c.height << " skip " << skip;

      for (ojph::ui32 y = 0; y < rh; ++y)
        for (ojph::ui32 x = 0; x < rw; ++x)
          ASSERT_EQ(recon[(size_t)y * rw + x],
                    sample_value(x * step, y * step, c.width, c.height))
            << c.width << "x" << c.height << " skip " << skip
            << " at (" << x << ", " << y << ")";
    }
  }
}

///////////////////////////////////////////////////////////////////////////////
// The codestream must carry an ATK marker segment describing an arbitrary
// (ARB) reversible filter, and a reader must classify it, from the ATK
// lifting steps, as reversible and predict-only.
TEST(TestRev12, SignalingAndDetection) {
  ojph::mem_outfile file;
  encode(file, 64, 64, 2, ojph::param_cod::OJPH_WAVELET_REV12);

  ojph::mem_infile infile;
  infile.open(file.get_data(), file.get_used_size());
  ojph::codestream codestream;
  codestream.read_headers(&infile);
  ojph::param_cod cod = codestream.access_cod();
  EXPECT_EQ(cod.get_wavelet_kern(), ojph::param_cod::OJPH_WAVELET_REV12);
  EXPECT_TRUE(cod.is_reversible());
  EXPECT_TRUE(cod.is_predict_only());
  codestream.close();
}

///////////////////////////////////////////////////////////////////////////////
// A handcrafted image distinguishing the previous-sample prediction from
// the symmetric (rev13) prediction through the coded detail subbands.
// The image is a single vertical step edge: zero for x < 5, then 100 for
// x >= 5, on an 8x8 grid.  The edge falls between an even sample (x = 4)
// and an odd sample (x = 5), so the previous-sample prediction has one
// nonzero horizontal difference per row, while every level still decodes
// to the exact subsample for both kernels.  This mainly documents intent;
// exactness is what is asserted.
TEST(TestRev12, StepEdgeIsExactAtEveryLevel) {
  const ojph::ui32 width = 8, height = 8, num_decomps = 2;
  std::vector<ojph::si32> image((size_t)width * height, 0);
  for (ojph::ui32 y = 0; y < height; ++y)
    for (ojph::ui32 x = 5; x < width; ++x)
      image[(size_t)y * width + x] = 100;

  const ojph::ui32 kernels[] = {
    ojph::param_cod::OJPH_WAVELET_REV12,
    ojph::param_cod::OJPH_WAVELET_REV13,
  };
  for (ojph::ui32 kernel : kernels)
  {
    ojph::mem_outfile file;
    encode(file, width, height, num_decomps, kernel, &image);

    for (ojph::ui32 skip = 0; skip <= num_decomps; ++skip)
    {
      ojph::ui32 rw, rh;
      std::vector<ojph::si32> recon = decode(file, skip, rw, rh);
      ojph::ui32 step = 1u << skip;
      for (ojph::ui32 y = 0; y < rh; ++y)
        for (ojph::ui32 x = 0; x < rw; ++x)
          ASSERT_EQ(recon[(size_t)y * rw + x],
                    image[(size_t)(y * step) * width + (x * step)])
            << "kernel " << kernel << " skip " << skip
            << " at (" << x << ", " << y << ")";
    }
  }
}

///////////////////////////////////////////////////////////////////////////////
// The rev13 and rev53 kernels share the ATK reading and writing code that
// this feature extended; make sure they remain lossless.
TEST(TestRev12, OtherKernelsRemainLossless) {
  const ojph::ui32 kernels[] = {
    ojph::param_cod::OJPH_WAVELET_REV13,
    ojph::param_cod::OJPH_WAVELET_REV53,
  };
  for (ojph::ui32 kernel : kernels)
  {
    ojph::mem_outfile file;
    encode(file, 255, 193, 3, kernel);

    ojph::ui32 rw, rh;
    std::vector<ojph::si32> recon = decode(file, 0, rw, rh);
    ASSERT_EQ(rw, 255u);
    ASSERT_EQ(rh, 193u);
    for (ojph::ui32 y = 0; y < rh; ++y)
      for (ojph::ui32 x = 0; x < rw; ++x)
        ASSERT_EQ(recon[(size_t)y * rw + x], sample_value(x, y, rw, rh))
          << "kernel " << kernel << " at (" << x << ", " << y << ")";
  }
}
