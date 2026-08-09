// Encode random images over many sizes, offsets, and kernels, write the
// codestreams to a directory, and verify lossless round-trips; running
// this against two builds and comparing the directories verifies that
// the builds produce byte-identical codestreams on all the edge cases
// (odd widths and heights, odd offsets, tiny images).
//
// When compiled with -DOJPH_BENCH_HAVE_HWY (and linked against libhwy),
// the OJPH_FORCE_TARGET environment variable forces a Highway dispatch
// target by disabling everything better (sse4, avx2, avx3, avx3_dl,
// avx3_zen4, avx3_spr, emu128, scalar), so one build can be checked
// target by target.
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include "ojph_file.h"
#include "ojph_mem.h"
#include "ojph_params.h"
#include "ojph_codestream.h"

// compile with -DOJPH_BENCH_HAVE_HWY (and link -lhwy) to allow forcing
// the hwy dispatch target through the OJPH_FORCE_TARGET environment
// variable (sse4, avx2, avx3, avx3_dl, avx3_zen4, avx3_spr, emu128,
// scalar)
#ifdef OJPH_BENCH_HAVE_HWY
#include <hwy/targets.h>
static void force_target_from_env()
{
  const char* t = getenv("OJPH_FORCE_TARGET");
  if (!t) return;
  int64_t best = 0;
  if (!strcmp(t, "sse4")) best = HWY_SSE4;
  else if (!strcmp(t, "avx2")) best = HWY_AVX2;
  else if (!strcmp(t, "avx3")) best = HWY_AVX3;
  else if (!strcmp(t, "avx3_dl")) best = HWY_AVX3_DL;
  else if (!strcmp(t, "avx3_zen4")) best = HWY_AVX3_ZEN4;
  else if (!strcmp(t, "avx3_spr")) best = HWY_AVX3_SPR;
  else if (!strcmp(t, "emu128")) best = HWY_EMU128;
  else if (!strcmp(t, "scalar")) best = HWY_SCALAR;
  else { printf("unknown OJPH_FORCE_TARGET %s\n", t); exit(1); }
  // disable everything better than the requested target
  hwy::DisableTargets(best - 1);
  const int64_t sup = hwy::SupportedTargets();
  // narrow hwy's chosen dispatch target to the masked set (see
  // target_matrix.cpp)
  hwy::GetChosenTarget().Update(sup);
  if ((sup & best) == 0)
  { printf("%s: not supported on this CPU\n", t); exit(2); }
}
#else
static void force_target_from_env() {}
#endif

static ojph::ui32 rng_state = 42;
static ojph::ui32 rng()
{
  rng_state = rng_state * 1664525u + 1013904223u;
  return rng_state >> 16;
}

int main(int argc, char** argv)
{
  if (argc < 2) { printf("usage: parity_sweep <outdir>\n"); return 1; }
  std::string outdir = argv[1];
  force_target_from_env();

  const ojph::ui32 sizes[] = { 1, 2, 3, 4, 5, 7, 8, 15, 16, 17,
                               31, 32, 33, 63, 64, 127, 129 };
  const ojph::ui32 offs[] = { 0, 1, 2, 3 };
  int failures = 0, count = 0;

  for (int kernel = 1; kernel <= 3; ++kernel)
    for (ojph::ui32 off : offs)
      for (ojph::ui32 w : sizes)
        for (ojph::ui32 h : sizes)
        {
          rng_state = 42 + w * 131 + h * 17 + off;
          std::vector<ojph::ui8> img((size_t)w * h);
          for (auto& v : img) v = (ojph::ui8)(rng() & 255);

          ojph::codestream cs;
          ojph::param_siz siz = cs.access_siz();
          siz.set_image_extent(ojph::point(off + w, off + h));
          siz.set_num_components(1);
          siz.set_component(0, ojph::point(1, 1), 8, false);
          siz.set_image_offset(ojph::point(off, off));
          siz.set_tile_offset(ojph::point(0, 0));
          ojph::param_cod cod = cs.access_cod();
          cod.set_num_decomposition(5);
          cod.set_reversible(true);
          cod.set_wavelet_kern((ojph::ui32)kernel);
          cs.set_planar(false);

          ojph::mem_outfile out;
          out.open(1 << 16);
          cs.write_headers(&out);
          ojph::ui32 nc;
          ojph::line_buf* line = cs.exchange(NULL, nc);
          for (ojph::ui32 y = 0; y < h; ++y) {
            for (ojph::ui32 x = 0; x < w; ++x)
              line->i32[x] = img[(size_t)y * w + x];
            line = cs.exchange(line, nc);
          }
          cs.flush();
          size_t bytes = out.get_used_size();
          cs.close();

          // lossless round-trip
          bool ok = true;
          {
            ojph::mem_infile in;
            in.open(out.get_data(), bytes);
            ojph::codestream dcs;
            dcs.read_headers(&in);
            dcs.create();
            for (ojph::ui32 y = 0; y < h; ++y) {
              ojph::ui32 comp;
              ojph::line_buf* l = dcs.pull(comp);
              for (ojph::ui32 x = 0; x < w; ++x)
                ok &= l->i32[x] == (ojph::si32)img[(size_t)y * w + x];
            }
            dcs.close();
          }
          if (!ok) {
            printf("LOSSY: k%d off%u %ux%u\n", kernel, off, w, h);
            ++failures;
          }

          char name[128];
          snprintf(name, sizeof(name), "%s/k%d_o%u_%ux%u.j2c",
                   outdir.c_str(), kernel, off, w, h);
          FILE* f = fopen(name, "wb");
          if (!f || fwrite(out.get_data(), 1, bytes, f) != bytes)
          { printf("cannot write %s\n", name); return 1; }
          fclose(f);
          ++count;
        }

  printf("%d codestreams written, %d lossless failures\n", count, failures);
  return failures != 0;
}
