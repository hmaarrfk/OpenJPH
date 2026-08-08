#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <vector>
#include "ojph_file.h"
#include "ojph_mem.h"
#include "ojph_params.h"
#include "ojph_codestream.h"
int main(int argc, char** argv) {
  const ojph::ui32 N = 4096;
  int kernel = argc > 1 ? atoi(argv[1]) : 3;
  std::vector<unsigned char> img((size_t)N * N);
  FILE* f = fopen("mask_0.raw", "rb");
  if (fread(img.data(), 1, img.size(), f) != img.size()) return 1;
  fclose(f);
  ojph::mem_outfile out;
  {
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
    out.open(1 << 20);
    cs.write_headers(&out);
    ojph::ui32 nc;
    ojph::line_buf* line = cs.exchange(NULL, nc);
    for (ojph::ui32 y = 0; y < N; ++y) {
      const unsigned char* sp = img.data() + (size_t)y * N;
      for (ojph::ui32 x = 0; x < N; ++x) line->i32[x] = sp[x];
      line = cs.exchange(line, nc);
    }
    cs.flush();
    cs.close();
  }
  std::vector<unsigned char> rec((size_t)N * N);
  double best = 1e9;
  for (int r = 0; r < 20; ++r) {
    auto t0 = std::chrono::steady_clock::now();
    ojph::mem_infile in;
    in.open(out.get_data(), out.get_used_size());
    ojph::codestream cs;
    cs.read_headers(&in);
    cs.create();
    for (ojph::ui32 y = 0; y < N; ++y) {
      ojph::ui32 comp;
      ojph::line_buf* line = cs.pull(comp);
      unsigned char* dp = rec.data() + (size_t)y * N;
      const ojph::si32* sp = line->i32;
      for (ojph::ui32 x = 0; x < N; ++x) dp[x] = (unsigned char)sp[x];
    }
    cs.close();
    double dt = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - t0).count();
    best = best < dt ? best : dt;
  }
  bool ok = true;
  for (size_t i = 0; i < img.size(); ++i) ok &= img[i] == rec[i];
  printf("kernel %d decode incl. u8 output: %.2f ms, lossless %d, "
         "%zu bytes\n", kernel, best * 1e3, (int)ok, out.get_used_size());
  return 0;
}
