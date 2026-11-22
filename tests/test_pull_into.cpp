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
// File: test_pull_into.cpp
// Author: Mark Harfouche
// Date: 22 November 2026
//***************************************************************************/

// Tests for codestream::pull_into, which writes each reconstructed row
// into a caller's buffer of 1, 2 or 4-byte samples, saturating 1 and 2-byte
// samples to the destination type, and for reversible
// predict-only codestreams of signed samples, which are held in 16-bit
// lines when they fit.

#include <cstring>
#include <vector>

#include "ojph_mem.h"
#include "ojph_file.h"
#include "ojph_codestream.h"
#include "ojph_params.h"
#include "gtest/gtest.h"

namespace {

struct config {
  ojph::ui32 bit_depth;
  bool is_signed;
  bool reversible;
  ojph::ui32 kernel;
  ojph::ui32 tile;
};

// The extremes of the range, a smooth ramp and scattered impulses
static ojph::si32 sample_value(const config& c, ojph::ui32 x, ojph::ui32 y)
{
  ojph::si64 range = (ojph::si64)1 << c.bit_depth;
  ojph::si64 low = c.is_signed ? -(range / 2) : 0;
  ojph::si64 v;
  if ((x * 31u + y * 17u) % 23u == 0)
    v = (x + y) % 2 ? low : low + range - 1;
  else
    v = low + (ojph::si64)((x * 7u + y * 3u) % (ojph::ui64)range);
  return (ojph::si32)v;
}

static void encode(ojph::mem_outfile& file, const config& c,
                   ojph::ui32 width, ojph::ui32 height)
{
  ojph::codestream codestream;
  ojph::param_siz siz = codestream.access_siz();
  siz.set_image_extent(ojph::point(width, height));
  siz.set_num_components(1);
  siz.set_component(0, ojph::point(1, 1), c.bit_depth, c.is_signed);
  if (c.tile)
    siz.set_tile_size(ojph::size(c.tile, c.tile));

  ojph::param_cod cod = codestream.access_cod();
  cod.set_num_decomposition(3);
  cod.set_reversible(c.reversible);
  if (c.reversible)
    cod.set_wavelet_kern(c.kernel);
  if (!c.reversible)
    codestream.access_qcd().set_irrev_quant(0.0001f);
  codestream.set_planar(false);

  file.open();
  codestream.write_headers(&file);
  ojph::ui32 next_comp;
  ojph::line_buf* line = codestream.exchange(NULL, next_comp);
  for (ojph::ui32 y = 0; y < height; ++y)
  {
    for (ojph::ui32 x = 0; x < width; ++x)
      line->i32[x] = sample_value(c, x, y);
    line = codestream.exchange(line, next_comp);
  }
  codestream.flush();
  codestream.close();
}

static std::vector<ojph::si64> decode_with_pull(ojph::mem_outfile& file,
                                                ojph::ui32 width,
                                                ojph::ui32 height)
{
  ojph::mem_infile infile;
  infile.open(file.get_data(), file.get_used_size());
  ojph::codestream codestream;
  codestream.read_headers(&infile);
  codestream.create();
  std::vector<ojph::si64> result((size_t)width * height);
  for (ojph::ui32 y = 0; y < height; ++y)
  {
    ojph::ui32 comp_num;
    ojph::line_buf* line = codestream.pull(comp_num);
    for (ojph::ui32 x = 0; x < width; ++x)
      result[(size_t)y * width + x] = line->i32[x];
  }
  codestream.close();
  return result;
}

static std::vector<ojph::ui8> decode_with_pull_into(ojph::mem_outfile& file,
                                                    ojph::ui32 width,
                                                    ojph::ui32 height,
                                                    ojph::ui32 sample_bytes,
                                                    bool dst_signed)
{
  ojph::mem_infile infile;
  infile.open(file.get_data(), file.get_used_size());
  ojph::codestream codestream;
  codestream.read_headers(&infile);
  codestream.create();
  std::vector<ojph::ui8> result((size_t)width * height * sample_bytes);
  for (ojph::ui32 y = 0; y < height; ++y)
  {
    ojph::ui32 comp_num;
    EXPECT_TRUE(codestream.pull_into(
      result.data() + (size_t)y * width * sample_bytes, sample_bytes,
      dst_signed, comp_num));
    EXPECT_EQ(comp_num, 0u);
  }
  codestream.close();
  return result;
}

static ojph::si64 narrowed(ojph::si64 v, ojph::ui32 sample_bytes,
                           bool is_signed)
{
  if (sample_bytes == 4)
    return is_signed ? (ojph::si64)(ojph::si32)v : (ojph::si64)(ojph::ui32)v;
  ojph::ui32 bits = sample_bytes * 8;
  ojph::si64 low = is_signed ? -((ojph::si64)1 << (bits - 1)) : 0;
  ojph::si64 high = is_signed ? ((ojph::si64)1 << (bits - 1)) - 1
                              : ((ojph::si64)1 << bits) - 1;
  return v < low ? low : (v > high ? high : v);
}

static ojph::si64 read_sample(const std::vector<ojph::ui8>& buf, size_t i,
                              ojph::ui32 sample_bytes, bool is_signed)
{
  ojph::si64 v = 0;
  if (sample_bytes == 1) {
    ojph::ui8 s; std::memcpy(&s, buf.data() + i, 1);
    v = is_signed ? (ojph::si64)(ojph::si8)s : (ojph::si64)s;
  }
  else if (sample_bytes == 2) {
    ojph::ui16 s; std::memcpy(&s, buf.data() + i * 2, 2);
    v = is_signed ? (ojph::si64)(ojph::si16)s : (ojph::si64)s;
  }
  else {
    ojph::ui32 s; std::memcpy(&s, buf.data() + i * 4, 4);
    v = is_signed ? (ojph::si64)(ojph::si32)s : (ojph::si64)s;
  }
  return v;
}

} // namespace

///////////////////////////////////////////////////////////////////////////////
// pull_into must write the samples pull returns, saturated to signed and
// unsigned 1 and 2-byte samples and as they are at 4 bytes, for 16-bit and
// 32-bit internal lines, several tiles, and the irreversible path, which
// pull_into serves through pull.
TEST(TestPullInto, MatchesPull) {
  const config configs[] = {
    {  8, false, true, ojph::param_cod::OJPH_WAVELET_REV12, 0 },
    {  2, true,  true, ojph::param_cod::OJPH_WAVELET_REV12, 0 },
    {  7, true,  true, ojph::param_cod::OJPH_WAVELET_REV13, 0 },
    {  8, true,  true, ojph::param_cod::OJPH_WAVELET_REV53, 0 },
    { 12, false, true, ojph::param_cod::OJPH_WAVELET_REV53, 0 },
    { 16, true,  true, ojph::param_cod::OJPH_WAVELET_REV12, 0 },
    { 20, false, true, ojph::param_cod::OJPH_WAVELET_REV53, 0 },
    {  8, false, true, ojph::param_cod::OJPH_WAVELET_REV12, 33 },
    {  8, false, false, 0, 0 },
  };
  const ojph::ui32 width = 97, height = 70;
  for (const config& c : configs)
  {
    ojph::mem_outfile file;
    encode(file, c, width, height);
    std::vector<ojph::si64> expected = decode_with_pull(file, width, height);
    for (ojph::ui32 sample_bytes : {1u, 2u, 4u})
      for (bool dst_signed : {false, true})
      {
        if (sample_bytes == 4 && (dst_signed != c.is_signed))
          continue;
        std::vector<ojph::ui8> got = decode_with_pull_into(
          file, width, height, sample_bytes, dst_signed);
        for (size_t i = 0; i < expected.size(); ++i)
          ASSERT_EQ(read_sample(got, i, sample_bytes, dst_signed),
                    narrowed(expected[i], sample_bytes, dst_signed))
            << c.bit_depth << " bits, signed " << c.is_signed
            << ", tile " << c.tile << ", " << sample_bytes
            << "-byte samples, destination signed " << dst_signed
            << ", at " << i;
      }
  }
}

///////////////////////////////////////////////////////////////////////////////
// Signed samples of a reversible predict-only codestream must decode
// losslessly, from the most negative to the most positive value their bit
// depth holds, through pull and pull_into.
TEST(TestPullInto, SignedPredictOnlyIsLossless) {
  const ojph::ui32 width = 131, height = 67;
  for (ojph::ui32 kernel : {ojph::param_cod::OJPH_WAVELET_REV12,
                            ojph::param_cod::OJPH_WAVELET_REV13})
    for (ojph::ui32 bit_depth : {1u, 2u, 5u, 9u, 12u})
    {
      config c = { bit_depth, true, true, kernel, 0 };
      ojph::mem_outfile file;
      encode(file, c, width, height);
      std::vector<ojph::si64> pulled = decode_with_pull(file, width, height);
      std::vector<ojph::ui8> into =
        decode_with_pull_into(file, width, height, 2, true);
      for (ojph::ui32 y = 0; y < height; ++y)
        for (ojph::ui32 x = 0; x < width; ++x)
        {
          size_t i = (size_t)y * width + x;
          ASSERT_EQ(pulled[i], sample_value(c, x, y))
            << "kernel " << kernel << ", " << bit_depth << " bits";
          ASSERT_EQ(read_sample(into, i, 2, true), sample_value(c, x, y))
            << "kernel " << kernel << ", " << bit_depth << " bits";
        }
    }
}
