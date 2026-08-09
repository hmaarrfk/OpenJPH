// Microbenchmark: gen vs avx2 vs hwy tx_from_cb kernels.
#include <chrono>
#include <cstdio>
#include <cstring>
#include <vector>
#include "ojph_defs.h"

namespace ojph { namespace local {
  void gen_rev_tx_from_cb16(const ui32*, si16*, ui32, ui32);
  void rev_tx_from_cb16(const ui32*, si16*, ui32, ui32);
  void gen_rev_tx_from_cb32(const ui32*, void*, ui32, float, ui32);
  void avx2_rev_tx_from_cb32(const ui32*, void*, ui32, float, ui32);
  void rev_tx_from_cb32(const ui32*, void*, ui32, float, ui32);
}}

using namespace ojph;
using namespace ojph::local;

template <typename F, typename D>
double bench(F f, const ui32* src, D* dst, ui32 w, int iters)
{
  double best = 1e30;
  for (int t = 0; t < 7; ++t) {
    auto t0 = std::chrono::steady_clock::now();
    for (int i = 0; i < iters; ++i)
      f(src, dst, w);
    auto t1 = std::chrono::steady_clock::now();
    double ns = std::chrono::duration<double, std::nano>(t1-t0).count()/iters;
    if (ns < best) best = ns;
  }
  return best;
}

int main()
{
  const ui32 K = 8;
  std::vector<ui32> src(4096);
  for (size_t i = 0; i < src.size(); ++i)
    src[i] = (ui32)((i * 2654435761u) & 0x807FFFFFu); // sign + 23 mag bits
  std::vector<si16> d16a(4096), d16b(4096);
  std::vector<si32> d32a(4096), d32b(4096);

  for (ui32 w : {64u, 33u, 1024u}) {
    int iters = 200000;
    double g16 = bench([](const ui32*s, si16*d, ui32 w)
      { gen_rev_tx_from_cb16(s,d,K,w); }, src.data(), d16a.data(), w, iters);
    double h16 = bench([](const ui32*s, si16*d, ui32 w)
      { rev_tx_from_cb16(s,d,K,w); }, src.data(), d16b.data(), w, iters);
    if (memcmp(d16a.data(), d16b.data(), w*2)) { printf("cb16 MISMATCH\n"); return 1; }
    double g32 = bench([](const ui32*s, si32*d, ui32 w)
      { gen_rev_tx_from_cb32(s,d,K,0.f,w); }, src.data(), d32a.data(), w, iters);
    double a32 = bench([](const ui32*s, si32*d, ui32 w)
      { avx2_rev_tx_from_cb32(s,d,K,0.f,w); }, src.data(), d32b.data(), w, iters);
    if (memcmp(d32a.data(), d32b.data(), w*4)) { printf("avx2 MISMATCH\n"); return 1; }
    double h32 = bench([](const ui32*s, si32*d, ui32 w)
      { rev_tx_from_cb32(s,d,K,0.f,w); }, src.data(), d32b.data(), w, iters);
    if (memcmp(d32a.data(), d32b.data(), w*4)) { printf("hwy32 MISMATCH\n"); return 1; }
    printf("w=%4u  cb16 gen %6.1f  hwy %6.1f ns | cb32 gen %6.1f"
           "  avx2 %6.1f  hwy %6.1f ns\n", w, g16, h16, g32, a32, h32);
  }
  return 0;
}
