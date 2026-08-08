// Encode mask_0 (4096x4096 uint8 raw) once and save the codestream.
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
  int kernel = argc > 1 ? atoi(argv[1]) : 3;
  const char* outname = argc > 2 ? argv[2] : "out.j2c";

  std::vector<unsigned char> img((size_t)N * N);
  FILE* f = fopen("mask_0.raw", "rb");
  if (!f || fread(img.data(), 1, img.size(), f) != img.size())
  { printf("cannot read mask_0.raw\n"); return 1; }
  fclose(f);

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
  size_t bytes = out.get_used_size();
  cs.close();

  FILE* g = fopen(outname, "wb");
  if (!g || fwrite(out.get_data(), 1, bytes, g) != bytes)
  { printf("cannot write %s\n", outname); return 1; }
  fclose(g);
  printf("kernel %d: wrote %zu bytes to %s\n", kernel, bytes, outname);
  return 0;
}
