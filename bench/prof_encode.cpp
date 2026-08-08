// Encode mask_0 (4096x4096 uint8 raw) repeatedly for profiling.
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <vector>

#include "ojph_file.h"
#include "ojph_mem.h"
#include "ojph_params.h"
#include "ojph_codestream.h"

int main(int argc, char** argv)
{
  const ojph::ui32 N = 4096;
  int reps = argc > 1 ? atoi(argv[1]) : 50;
  int kernel = argc > 2 ? atoi(argv[2]) : 3;
  int zeros = argc > 3 ? atoi(argv[3]) : 0;

  std::vector<unsigned char> img((size_t)N * N);
  FILE* f = fopen("mask_0.raw", "rb");
  if (!f || fread(img.data(), 1, img.size(), f) != img.size())
  { printf("cannot read mask_0.raw\n"); return 1; }
  fclose(f);
  if (zeros)
    for (auto& v : img) v = 0;

  { // cost of the uint8 -> si32 line copies alone
    std::vector<ojph::si32> dummy(N);
    auto t0 = std::chrono::steady_clock::now();
    for (ojph::ui32 y = 0; y < N; ++y) {
      const unsigned char* sp = img.data() + (size_t)y * N;
      for (ojph::ui32 x = 0; x < N; ++x)
        dummy[x] = sp[x];
      asm volatile("" :: "r"(dummy.data()) : "memory");
    }
    double dt = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - t0).count();
    printf("uint8->si32 copy alone: %.2f ms\n", dt * 1e3);
  }

  double best = 1e9, total = 0.0;
  size_t bytes = 0;
  for (int r = 0; r < reps; ++r)
  {
    auto t0 = std::chrono::steady_clock::now();

    ojph::codestream cs;
    ojph::param_siz siz = cs.access_siz();
    siz.set_image_extent(ojph::point(N, N));
    siz.set_num_components(1);
    siz.set_component(0, ojph::point(1, 1), 8, false);
    siz.set_image_offset(ojph::point(0, 0));
    siz.set_tile_offset(ojph::point(0, 0));
    ojph::param_cod cod = cs.access_cod();
    cod.set_num_decomposition(8);
    cod.set_reversible(true);
    cod.set_wavelet_kern((ojph::ui32)kernel);
    cs.set_planar(false);

    ojph::mem_outfile out;
    out.open(1 << 20);
    cs.write_headers(&out);
    ojph::ui32 nc;
    ojph::line_buf* line = cs.exchange(NULL, nc);
    for (ojph::ui32 y = 0; y < N; ++y) {
      const unsigned char* sp = img.data() + (size_t)y * N;
      for (ojph::ui32 x = 0; x < N; ++x)
        line->i32[x] = sp[x];
      line = cs.exchange(line, nc);
    }
    cs.flush();
    bytes = out.get_used_size();
    cs.close();

    double dt = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - t0).count();
    best = best < dt ? best : dt;
    total += dt;
  }
  printf("kernel %d: %zu bytes, best %.2f ms, avg %.2f ms over %d reps\n",
         kernel, bytes, best * 1e3, total / reps * 1e3, reps);
  return 0;
}
