// Micro-benchmark: isolated HT cleanup-pass block encoder timing.
// Calls the internal (local) codeblock encoder entry points directly on
// synthetic 64x64 codeblocks resembling mask content.
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>

#include "ojph_mem.h"
#include "../src/core/coding/ojph_block_encoder.h"

using namespace ojph;
using namespace ojph::local;
using clk = std::chrono::steady_clock;

typedef void (*enc_fn)(ui32*, ui32, ui32, ui32, ui32, ui32, ui32*,
                       mem_elastic_allocator*, coded_lists*&);

static const ui32 W = 64, H = 64, STRIDE = 64;
static const ui32 K_MAX = 9; // 8-bit reversible, one extra bit

// value in sign-magnitude, as rev_tx_to_cb32 produces
static ui32 sm(int v)
{
  ui32 mag = (ui32)(v < 0 ? -v : v);
  ui32 sign = v < 0 ? 0x80000000u : 0;
  return (mag << (31 - K_MAX)) | sign;
}

// pattern 0: vertical step edge at x=32 (mask boundary)
// pattern 1: sparse: one diagonal line of +/-255
// pattern 2: dense pseudo-random 8-bit (worst case)
// pattern 3: horizontal band edge (rows 0-31 zero)
static void fill_block(ui32* buf, int pattern)
{
  memset(buf, 0, STRIDE * H * sizeof(ui32));
  unsigned s = 12345;
  for (ui32 y = 0; y < H; ++y)
    for (ui32 x = 0; x < W; ++x) {
      int v = 0;
      switch (pattern) {
        case 0: v = (x == 32) ? 255 : (x == 33 ? -128 : 0); break;
        case 1: v = (x == (y & 63)) ? ((y & 1) ? -255 : 255) : 0; break;
        case 2:
          s = s * 1103515245u + 12345u;
          v = (int)((s >> 16) & 0xFF) - 128;
          break;
        case 3: v = (y == 32) ? 255 : (y == 33 ? -128 : 0); break;
      }
      buf[y * STRIDE + x] = v ? sm(v) : 0;
    }
}

static const char* names[4] =
  { "vert edge ", "diag line ", "dense rand", "horz edge " };

int main(int argc, char** argv)
{
  int iters = argc > 1 ? atoi(argv[1]) : 20000;

  initialize_block_encoder_tables_avx2();
  initialize_block_encoder_tables_hwy();

  std::vector<ui32> buf(STRIDE * H);
  ui32 lengths[2];

  struct { const char* name; enc_fn fn; } encoders[] = {
    { "avx2", ojph_encode_codeblock_avx2 },
    { "hwy ", ojph_encode_codeblock_hwy },
  };

  // correctness: identical bytes for every pattern, incl. odd sizes
  static const ui32 sizes[][2] =
    { {64, 64}, {47, 33}, {17, 13}, {33, 64}, {64, 1} };
  for (int pat = 0; pat < 4; ++pat) {
    fill_block(buf.data(), pat);
    for (auto& wh : sizes) {
      mem_elastic_allocator elastic(1 << 20);
      coded_lists *c_a = NULL, *c_h = NULL;
      ui32 len_a[2], len_h[2];
      ojph_encode_codeblock_avx2(buf.data(), K_MAX - 1, 1, wh[0], wh[1],
                                 STRIDE, len_a, &elastic, c_a);
      ojph_encode_codeblock_hwy(buf.data(), K_MAX - 1, 1, wh[0], wh[1],
                                STRIDE, len_h, &elastic, c_h);
      if (len_a[0] != len_h[0] ||
          memcmp(c_a->buf, c_h->buf, len_a[0]) != 0) {
        printf("MISMATCH on pattern %d, %ux%u\n", pat, wh[0], wh[1]);
        return 1;
      }
    }
  }
  printf("outputs identical on all patterns and sizes\n");

  for (auto& e : encoders) {
    printf("%s:\n", e.name);
    for (int pat = 0; pat < 4; ++pat) {
      fill_block(buf.data(), pat);
      double best = 1e9;
      size_t bytes = 0;
      for (int rep = 0; rep < 15; ++rep) {
        mem_elastic_allocator elastic(1 << 24);
        coded_lists* coded = NULL;
        auto t0 = clk::now();
        for (int i = 0; i < iters; ++i)
          e.fn(buf.data(), K_MAX - 1, 1, W, H, STRIDE, lengths,
               &elastic, coded);
        double dt = std::chrono::duration<double>(clk::now() - t0).count();
        best = best < dt / iters ? best : dt / iters;
        bytes = lengths[0];
      }
      printf("  %s: %8.1f ns/block  (%zu bytes)\n",
             names[pat], best * 1e9, bytes);
    }
  }
  return 0;
}
