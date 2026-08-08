#include <chrono>
#include <cstdio>
#include <vector>
#include "ojph_file.h"
#include "ojph_mem.h"
#include "ojph_params.h"
#include "ojph_codestream.h"
using clk = std::chrono::steady_clock;
static double ms(clk::time_point a, clk::time_point b)
{ return std::chrono::duration<double>(b - a).count() * 1e3; }
int main()
{
  const ojph::ui32 N = 4096;
  std::vector<unsigned char> img((size_t)N * N);
  FILE* f = fopen("mask_0.raw", "rb");
  if (fread(img.data(), 1, img.size(), f) != img.size()) return 1;
  fclose(f);
  double t_setup = 1e9, t_push = 1e9, t_flush = 1e9;
  for (int r = 0; r < 30; ++r)
  {
    auto t0 = clk::now();
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
    cod.set_wavelet_kern(3);
    cs.set_planar(false);
    ojph::mem_outfile out;
    out.open(1 << 20);
    cs.write_headers(&out);      // includes pre_alloc + finalize_alloc
    auto t1 = clk::now();
    ojph::ui32 nc;
    ojph::line_buf* line = cs.exchange(NULL, nc);
    for (ojph::ui32 y = 0; y < N; ++y) {
      const unsigned char* sp = img.data() + (size_t)y * N;
      for (ojph::ui32 x = 0; x < N; ++x)
        line->i32[x] = sp[x];
      line = cs.exchange(line, nc);   // transform + block collection+coding
    }
    auto t2 = clk::now();
    cs.flush();                  // packet assembly + writing
    cs.close();
    auto t3 = clk::now();
    t_setup = std::min(t_setup, ms(t0, t1));
    t_push  = std::min(t_push,  ms(t1, t2));
    t_flush = std::min(t_flush, ms(t2, t3));
  }
  printf("setup+alloc+headers: %6.2f ms\n", t_setup);
  printf("push (copy+dwt+code): %5.2f ms\n", t_push);
  printf("flush (packets+write): %4.2f ms\n", t_flush);
  return 0;
}
