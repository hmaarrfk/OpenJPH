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
