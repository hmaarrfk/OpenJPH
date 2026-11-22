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
// File: test_sample_range.cpp
// Author: Mark Harfouche
// Date: 22 November 2026
//***************************************************************************/

// Reversible coding must reproduce every sample of the declared range,
// including its extremes, whatever the number of decompositions; with no
// decomposition the most negative sample once lost a bit-plane.

#include <cstdint>
#include <vector>

#include "ojph_mem.h"
#include "ojph_file.h"
#include "ojph_codestream.h"
#include "ojph_params.h"
#include "gtest/gtest.h"

namespace {

struct range { int64_t low, high; };

static range sample_range(ojph::ui32 bit_depth, bool is_signed)
{
  if (is_signed)
    return { -(int64_t(1) << (bit_depth - 1)),
             (int64_t(1) << (bit_depth - 1)) - 1 };
  return { 0, (int64_t(1) << bit_depth) - 1 };
}

// Both extremes on every row, and values spread over the range between
static int64_t sample_value(range r, size_t i)
{
  if (i % 5 == 0)
    return r.low;
  if (i % 5 == 1)
    return r.high;
  return r.low + (int64_t)((i * 2654435761u) % (uint64_t)(r.high - r.low + 1));
}

} // namespace

///////////////////////////////////////////////////////////////////////////////
TEST(TestSampleRange, ReversibleKeepsTheExtremes) {
  const ojph::ui32 width = 67, height = 41;
  for (ojph::ui32 kernel : {ojph::param_cod::OJPH_WAVELET_REV53,
                            ojph::param_cod::OJPH_WAVELET_REV13,
                            ojph::param_cod::OJPH_WAVELET_REV12})
  for (ojph::ui32 bit_depth : {1u, 2u, 8u, 13u, 16u, 31u, 32u})
  for (bool is_signed : {true, false})
  for (ojph::ui32 num_decomps : {0u, 1u, 3u})
  {
    range r = sample_range(bit_depth, is_signed);
    std::vector<int64_t> image((size_t)width * height);
    for (size_t i = 0; i < image.size(); ++i)
      image[i] = sample_value(r, i);

    ojph::mem_outfile file;
    file.open();
    {
      ojph::codestream codestream;
      ojph::param_siz siz = codestream.access_siz();
      siz.set_image_extent(ojph::point(width, height));
      siz.set_num_components(1);
      siz.set_component(0, ojph::point(1, 1), bit_depth, is_signed);
      ojph::param_cod cod = codestream.access_cod();
      cod.set_num_decomposition(num_decomps);
      cod.set_reversible(true);
      cod.set_wavelet_kern(kernel);
      codestream.set_planar(false);
      codestream.write_headers(&file);
      ojph::ui32 comp;
      ojph::line_buf* line = codestream.exchange(NULL, comp);
      for (ojph::ui32 y = 0; y < height; ++y)
      {
        for (ojph::ui32 x = 0; x < width; ++x)
        {
          int64_t v = image[(size_t)y * width + x];
          if (line->flags & ojph::line_buf::LFT_64BIT)
            line->i64[x] = v;
          else
            line->i32[x] = (ojph::si32)(uint32_t)v;
        }
        line = codestream.exchange(line, comp);
      }
      codestream.flush();
      codestream.close();
    }

    ojph::mem_infile infile;
    infile.open(file.get_data(), file.get_used_size());
    ojph::codestream codestream;
    codestream.read_headers(&infile);
    codestream.create();
    for (ojph::ui32 y = 0; y < height; ++y)
    {
      ojph::ui32 comp;
      ojph::line_buf* line = codestream.pull(comp);
      for (ojph::ui32 x = 0; x < width; ++x)
      {
        int64_t v = (line->flags & ojph::line_buf::LFT_64BIT)
          ? line->i64[x]
          : is_signed ? (int64_t)line->i32[x]
                      : (int64_t)(uint32_t)line->i32[x];
        ASSERT_EQ(v, image[(size_t)y * width + x])
          << "kernel " << kernel << ", " << bit_depth << " bits, signed "
          << is_signed << ", " << num_decomps << " decompositions, at ("
          << x << ", " << y << ")";
      }
    }
    codestream.close();
  }
}
