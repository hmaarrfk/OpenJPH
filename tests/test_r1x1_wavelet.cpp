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

#include "ojph_arch.h"
#include "ojph_mem.h"
#include "../src/core/transform/ojph_transform.h"
#include "gtest/gtest.h"

TEST(R1X1Wavelet, HorzRoundTripEvenAligned) {
  using namespace ojph;

  si32 src_buf[4] = {1, 2, 3, 4};
  si32 l_buf[4] = {0, 0, 0, 0};
  si32 h_buf[4] = {0, 0, 0, 0};
  si32 dst_buf[4] = {0, 0, 0, 0};

  line_buf src;
  line_buf ldst;
  line_buf hdst;
  line_buf dst;

  src.wrap(src_buf, 4, 0);
  ldst.wrap(l_buf, 4, 0);
  hdst.wrap(h_buf, 4, 0);
  dst.wrap(dst_buf, 4, 0);

  local::r1x1_rev_horz_ana(nullptr, &ldst, &hdst, &src, 4, true);
  local::r1x1_rev_horz_syn(nullptr, &dst, &ldst, &hdst, 4, true);

  for (int i = 0; i < 4; ++i)
    EXPECT_EQ(dst_buf[i], src_buf[i]);
}

TEST(R1X1Wavelet, IrreversibleOneXoneQstepDegradesGracefully) {
  using namespace ojph;

  const int width = 128;
  float src_buf[width];
  float low_buf[width];
  float high_buf[width];
  float recon_buf[width];

  for (int i = 0; i < width; ++i)
    src_buf[i] = static_cast<float>((i * 3) & 0xFF);

  line_buf src;
  line_buf ldst;
  line_buf hdst;
  line_buf dst;

  src.wrap(src_buf, width, sizeof(float));
  ldst.wrap(low_buf, width, sizeof(float));
  hdst.wrap(high_buf, width, sizeof(float));
  dst.wrap(recon_buf, width, sizeof(float));

  local::param_atk atk_state;
  local::param_atk* atk = atk_state.get_atk(3);
  ASSERT_NE(atk, nullptr);
  EXPECT_FALSE(atk->is_reversible());
  EXPECT_EQ(atk->get_num_steps(), 0u);

  auto compute_mse = [&](float qstep) {
    local::param_qcd qcd_state;
    qcd_state.set_irrev_quant(1);
    ojph_unused(qcd_state);

    local::irv_horz_ana(atk, &ldst, &hdst, &src, width, true);
    for (int i = 0; i < width; ++i)
      low_buf[i] = std::round(low_buf[i] / qstep) * qstep;
    local::irv_horz_syn(atk, &dst, &ldst, &hdst, width, true);

    double se = 0.0;
    for (int i = 0; i < width; ++i) {
      double d = static_cast<double>(src_buf[i]) -
                 static_cast<double>(recon_buf[i]);
      se += d * d;
    }
    return static_cast<double>(se / width);
  };

  double mse_small = compute_mse(1.0f);
  double mse_large = compute_mse(4.0f);

  EXPECT_LE(mse_small, mse_large);
}
