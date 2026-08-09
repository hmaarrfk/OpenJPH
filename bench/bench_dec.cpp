// Decode benchmark for hwy-dec work.
// Encodes mask_0.raw (4096x4096 u8, 8 levels, reversible) once per kernel,
// then times full-res and reduced-res decode, best-of-N.
// args: kernel(1|2|3) reps [mode] [bits]
//   mode: mask (default) | zero (all-zero image, isolates non-block cost)
//   bits: 8 (default) | 16 (mask values scaled by 257)
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>

#include "ojph_file.h"
#include "ojph_mem.h"
#include "ojph_params.h"
#include "ojph_codestream.h"

static const ojph::ui32 N = 4096;
static std::vector<ojph::ui16> IMG; // u8 data also stored here

static void encode(ojph::mem_outfile& out, int kernel, int bits)
{
  ojph::codestream cs;
  ojph::param_siz siz = cs.access_siz();
  siz.set_image_extent(ojph::point(N, N));
  siz.set_num_components(1);
  siz.set_component(0, ojph::point(1, 1), (ojph::ui32)bits, false);
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
    const ojph::ui16* sp = IMG.data() + (size_t)y * N;
    if (line->flags & ojph::line_buf::LFT_16BIT) {
      ojph::si16* dp = line->i16;
      for (ojph::ui32 x = 0; x < N; ++x) dp[x] = (ojph::si16)sp[x];
    } else {
      ojph::si32* dp = line->i32;
      for (ojph::ui32 x = 0; x < N; ++x) dp[x] = sp[x];
    }
    line = cs.exchange(line, nc);
  }
  cs.flush();
  cs.close();
}

// decode at reduced resolution r into dst (size (N>>r)^2)
static void decode(const ojph::mem_outfile& out, ojph::ui32 r,
                   ojph::ui16* dst)
{
  ojph::mem_infile in;
  in.open(out.get_data(), const_cast<ojph::mem_outfile&>(out).get_used_size());
  ojph::codestream cs;
  cs.read_headers(&in);
  if (r > 0)
    cs.restrict_input_resolution(r, r);
  cs.create();
  ojph::ui32 n = N >> r;
  for (ojph::ui32 y = 0; y < n; ++y) {
    ojph::ui32 comp;
    ojph::line_buf* line = cs.pull(comp);
    ojph::ui16* dp = dst + (size_t)y * n;
    if (line->flags & ojph::line_buf::LFT_16BIT) {
      const ojph::si16* sp = line->i16;
      for (ojph::ui32 x = 0; x < n; ++x) dp[x] = (ojph::ui16)sp[x];
    } else {
      const ojph::si32* sp = line->i32;
      for (ojph::ui32 x = 0; x < n; ++x) dp[x] = (ojph::ui16)sp[x];
    }
  }
  cs.close();
}

int main(int argc, char** argv)
{
  int kernel = argc > 1 ? atoi(argv[1]) : 3;
  int reps = argc > 2 ? atoi(argv[2]) : 20;
  const char* mode = argc > 3 ? argv[3] : "mask";
  int bits = argc > 4 ? atoi(argv[4]) : 8;

  IMG.resize((size_t)N * N, 0);
  if (strcmp(mode, "zero") != 0) {
    std::vector<ojph::ui8> raw((size_t)N * N);
    FILE* f = fopen("mask_0.raw", "rb");
    if (!f) { fprintf(stderr, "mask_0.raw not found\n"); return 1; }
    if (fread(raw.data(), 1, raw.size(), f) != raw.size()) {
      fprintf(stderr, "short read\n"); return 1;
    }
    fclose(f);
    for (size_t i = 0; i < raw.size(); ++i)
      IMG[i] = (ojph::ui16)(bits == 16 ? raw[i] * 257 : raw[i]);
  }

  ojph::mem_outfile out;
  encode(out, kernel, bits);
  size_t clen = out.get_used_size();
  printf("kernel=%d mode=%s bits=%d codestream=%zu bytes\n",
         kernel, mode, bits, clen);

  std::vector<ojph::ui16> dst((size_t)N * N);
  for (ojph::ui32 r = 0; r <= 3; ++r) {
    ojph::ui32 n = N >> r;
    double best = 1e30;
    for (int i = 0; i < reps; ++i) {
      auto t0 = std::chrono::steady_clock::now();
      decode(out, r, dst.data());
      auto t1 = std::chrono::steady_clock::now();
      double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
      if (ms < best) best = ms;
    }
    // verify: r=0 must equal source; r>0 exact subsampling for kernels 2,3
    bool ok = true; const char* vs = "";
    if (r == 0) {
      for (size_t i = 0; i < (size_t)n * n; ++i)
        ok &= dst[i] == IMG[i];
      vs = ok ? "LOSSLESS" : "MISMATCH";
    } else if (kernel >= 2) {
      for (ojph::ui32 y = 0; y < n && ok; ++y)
        for (ojph::ui32 x = 0; x < n; ++x)
          ok &= dst[(size_t)y * n + x] ==
                IMG[((size_t)y << r) * N + ((size_t)x << r)];
      vs = ok ? "EXACT-SUBSAMPLE" : "MISMATCH";
    }
    printf("  r=%u %ux%u  best=%.3f ms  %s\n", r, n, n, best, vs);
    if (!ok) return 2;
  }
  return 0;
}
