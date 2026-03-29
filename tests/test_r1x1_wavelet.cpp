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

#include <vector>

#include "ojph_codestream.h"
#include "ojph_file.h"
#include "ojph_mem.h"
#include "ojph_params.h"
#include "gtest/gtest.h"

namespace {

ojph::si32 circle_sample(ojph::ui32 x, ojph::ui32 y, ojph::ui32 width,
                         ojph::ui32 height, ojph::si32 radius)
{
  const ojph::si32 cx = (ojph::si32)(width / 2);
  const ojph::si32 cy = (ojph::si32)(height / 2);
  const ojph::si32 dx = (ojph::si32)x - cx;
  const ojph::si32 dy = (ojph::si32)y - cy;
  if (dx * dx + dy * dy <= radius * radius)
    return 128;
  return 0;
}

std::vector<ojph::ui8> encode_circle_to_memory(const std::vector<ojph::si32>& full_plane,
                                               ojph::ui32 width,
                                               ojph::ui32 height,
                                               ojph::ui32 num_decompositions,
                                               bool use_r1x1_wavelet)
{
  using namespace ojph;

  mem_outfile encoded;
  encoded.open(65536u, false);

  {
    codestream writer;
    param_siz siz = writer.access_siz();
    siz.set_image_extent(point(width, height));
    siz.set_num_components(1);
    siz.set_component(0, point(1, 1), 8, false);
    siz.set_image_offset(point(0, 0));
    siz.set_tile_size(size(width, height));
    siz.set_tile_offset(point(0, 0));

    param_cod cod = writer.access_cod();
    cod.set_num_decomposition(num_decompositions);
    cod.set_block_dims(64, 64);
    cod.set_progression_order("RLCP");
    cod.set_color_transform(false);
    cod.set_reversible(true);
    if (use_r1x1_wavelet)
      cod.set_r1x1(true);

    writer.set_planar(false);
    writer.set_tilepart_divisions(false, false);
    writer.request_tlm_marker(false);
    writer.write_headers(&encoded, NULL, 0);

    ui32 next_comp = 0;
    line_buf* row_line = writer.exchange(NULL, next_comp);
    const ui32 enc_height =
      siz.get_image_extent().y - siz.get_image_offset().y;
    const ui32 enc_width =
      siz.get_image_extent().x - siz.get_image_offset().x;

    for (ui32 row = 0; row < enc_height; ++row) {
      si32* dst = row_line->i32;
      const si32* src_row = full_plane.data() + (size_t)row * width;
      for (ui32 x = 0; x < enc_width; ++x)
        dst[x] = src_row[x];
      row_line = writer.exchange(row_line, next_comp);
    }
    writer.flush();
    writer.close();
  }

  std::vector<ui8> bytes(encoded.get_data(),
                         encoded.get_data() + encoded.get_used_size());
  encoded.close();
  return bytes;
}

} // namespace

TEST(TestExecutables, R1x1WaveletCircleMemoryDecodeMatchesGridSubsample) {
  using namespace ojph;

  const ui32 width = 512;
  const ui32 height = 512;
  const si32 radius = 150;
  const ui32 num_decompositions = 5;

  std::vector<si32> full_plane((size_t)width * (size_t)height);
  for (ui32 y = 0; y < height; ++y)
    for (ui32 x = 0; x < width; ++x)
      full_plane[(size_t)y * width + x] =
        circle_sample(x, y, width, height, radius);

  std::vector<ui8> codestream_bytes =
    encode_circle_to_memory(full_plane, width, height, num_decompositions,
                            true);

  for (ui32 level = 0; level <= num_decompositions; ++level) {
    mem_infile reader_file;
    reader_file.open(codestream_bytes.data(), codestream_bytes.size());

    codestream reader;
    reader.set_planar(false);
    reader.read_headers(&reader_file);
    reader.restrict_input_resolution(level, level);
    reader.create();

    param_siz rsiz = reader.access_siz();
    const ui32 recon_w = rsiz.get_recon_width(0);
    const ui32 recon_h = rsiz.get_recon_height(0);
    ASSERT_EQ(recon_w, width >> level);
    ASSERT_EQ(recon_h, height >> level);

    for (ui32 row = 0; row < recon_h; ++row) {
      ui32 comp_num = 0;
      line_buf* line = reader.pull(comp_num);
      ASSERT_EQ(comp_num, 0u);
      const si32* samples = line->i32;
      const ui32 src_y = row << level;
      for (ui32 col = 0; col < recon_w; ++col) {
        const ui32 src_x = col << level;
        const si32 expected =
          full_plane[(size_t)src_y * width + src_x];
        EXPECT_EQ(samples[col], expected)
          << "level=" << level << " row=" << row << " col=" << col;
      }
    }
    reader.close();
    reader_file.close();
  }
}
