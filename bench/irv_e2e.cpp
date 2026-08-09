// End-to-end irreversible (9/7) encode/decode benchmark at 4096x4096.
// Contents:
//   mask    - mask_0.raw, u8, 1 component (not photographic, but the
//             project's reference content)
//   synth8  - smooth gradients + sinusoids + mild noise, u8 RGB,
//             3 components with the ICT colour transform
//   synth16 - the same pattern, u16, 1 component
// Reports min and median encode and decode times over the reps; decode
// also reports PSNR versus the source as a sanity check.
//
// Build like bench/irv_micro.cpp (public API only):
//   g++ -O3 -std=c++14 -o bench/irv_e2e bench/irv_e2e.cpp \
//     -I src/core/openjph build-t/src/core/libojph.a
// args: content(mask|synth8|synth16) [reps]
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include "ojph_file.h"
#include "ojph_mem.h"
#include "ojph_params.h"
#include "ojph_codestream.h"

static const ojph::ui32 N = 4096;
static std::vector<ojph::ui16> IMG; // nc planes of N*N, u8 stored too
static ojph::ui32 NC = 1;
static ojph::ui32 BITS = 8;

static void make_synth(ojph::ui32 nc, ojph::ui32 bits)
{
  NC = nc; BITS = bits;
  IMG.resize((size_t)nc * N * N);
  const double maxv = (double)((1u << bits) - 1);
  ojph::ui32 rng = 0x12345678u;
  for (ojph::ui32 y = 0; y < N; ++y)
    for (ojph::ui32 x = 0; x < N; ++x)
    {
      double fx = (double)x / N, fy = (double)y / N;
      // smooth photographic-like base: gradients + low-freq sinusoids
      double base = 0.35 + 0.25 * fx + 0.15 * fy
        + 0.12 * sin(6.3 * fx + 2.0 * sin(3.1 * fy))
        + 0.08 * sin(17.0 * fy + 3.0 * fx);
      for (ojph::ui32 c = 0; c < nc; ++c)
      {
        // channel tint + mild sensor-like noise
        double v = base + 0.06 * (double)c * (fx - fy);
        rng = rng * 1664525u + 1013904223u;
        v += ((double)(rng >> 8) / 16777216.0 - 0.5) * 0.02;
        v = v < 0.0 ? 0.0 : (v > 1.0 ? 1.0 : v);
        IMG[((size_t)c * N + y) * N + x] = (ojph::ui16)(v * maxv + 0.5);
      }
    }
}

static bool load_mask()
{
  NC = 1; BITS = 8;
  std::vector<ojph::ui8> raw((size_t)N * N);
  FILE* f = fopen("mask_0.raw", "rb");
  if (!f || fread(raw.data(), 1, raw.size(), f) != raw.size())
  { if (f) fclose(f); return false; }
  fclose(f);
  IMG.resize((size_t)N * N);
  for (size_t i = 0; i < raw.size(); ++i)
    IMG[i] = raw[i];
  return true;
}

static void encode(ojph::mem_outfile& out)
{
  ojph::codestream cs;
  ojph::param_siz siz = cs.access_siz();
  siz.set_image_extent(ojph::point(N, N));
  siz.set_num_components(NC);
  for (ojph::ui32 c = 0; c < NC; ++c)
    siz.set_component(c, ojph::point(1, 1), BITS, false);
  siz.set_image_offset(ojph::point(0, 0));
  siz.set_tile_offset(ojph::point(0, 0));
  ojph::param_cod cod = cs.access_cod();
  cod.set_num_decomposition(8);
  cod.set_reversible(false);           // 9/7 irreversible
  cod.set_color_transform(NC == 3);    // ICT
  cs.access_qcd().set_irrev_quant(1.0f / 256.0f);
  cs.set_planar(false);

  out.open(1 << 20);
  cs.write_headers(&out);
  ojph::ui32 nc;
  ojph::line_buf* line = cs.exchange(NULL, nc);
  for (ojph::ui32 y = 0; y < N; ++y)
    for (ojph::ui32 c = 0; c < NC; ++c) {
      const ojph::ui16* sp = IMG.data() + ((size_t)c * N + y) * N;
      ojph::si32* dp = line->i32;
      for (ojph::ui32 x = 0; x < N; ++x)
        dp[x] = sp[x];
      line = cs.exchange(line, nc);
    }
  cs.flush();
  cs.close();
}

static void decode(const ojph::mem_outfile& out, ojph::ui16* dst)
{
  ojph::mem_infile in;
  in.open(out.get_data(),
          const_cast<ojph::mem_outfile&>(out).get_used_size());
  ojph::codestream cs;
  cs.read_headers(&in);
  cs.create();
  const ojph::si32 maxv = (ojph::si32)((1u << BITS) - 1);
  for (ojph::ui32 y = 0; y < N; ++y)
    for (ojph::ui32 c = 0; c < NC; ++c) {
      ojph::ui32 comp;
      ojph::line_buf* line = cs.pull(comp);
      ojph::ui16* dp = dst + ((size_t)comp * N + y) * N;
      const ojph::si32* sp = line->i32;
      for (ojph::ui32 x = 0; x < N; ++x) {
        ojph::si32 v = sp[x];
        v = v < 0 ? 0 : (v > maxv ? maxv : v);
        dp[x] = (ojph::ui16)v;
      }
    }
  cs.close();
}

static void stats(const char* what, std::vector<double>& t)
{
  std::sort(t.begin(), t.end());
  printf("  %-8s min %7.2f ms   median %7.2f ms\n", what,
         t.front(), t[t.size() / 2]);
  fflush(stdout);
}

int main(int argc, char** argv)
{
  const char* content = argc > 1 ? argv[1] : "mask";
  int reps = argc > 2 ? atoi(argv[2]) : 15;

  if (!strcmp(content, "mask")) {
    if (!load_mask()) { fprintf(stderr, "mask_0.raw not found\n"); return 1; }
  }
  else if (!strcmp(content, "synth8"))  make_synth(3, 8);
  else if (!strcmp(content, "synth16")) make_synth(1, 16);
  else { fprintf(stderr, "unknown content %s\n", content); return 1; }

  ojph::mem_outfile out;
  std::vector<double> te, td;
  for (int r = 0; r < reps; ++r) {
    ojph::mem_outfile o;
    auto t0 = std::chrono::steady_clock::now();
    encode(o);
    te.push_back(std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - t0).count());
    if (r == 0) { // keep the first for decoding
      out.open(1);
      out.write(o.get_data(), (int)o.get_used_size());
    }
  }
  printf("%s: %u comp x %u bit, codestream %zu bytes\n", content, NC, BITS,
         (size_t)out.get_used_size());
  stats("encode", te);

  std::vector<ojph::ui16> dst((size_t)NC * N * N);
  for (int r = 0; r < reps; ++r) {
    auto t0 = std::chrono::steady_clock::now();
    decode(out, dst.data());
    td.push_back(std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - t0).count());
  }
  stats("decode", td);

  double se = 0.0;
  for (size_t i = 0; i < dst.size(); ++i) {
    double d = (double)dst[i] - (double)IMG[i];
    se += d * d;
  }
  double maxv = (double)((1u << BITS) - 1);
  double mse = se / (double)dst.size();
  printf("  psnr   %.2f dB\n", mse > 0 ?
         10.0 * log10(maxv * maxv / mse) : 999.0);
  return 0;
}
