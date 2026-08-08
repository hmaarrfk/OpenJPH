// Encode/decode harness for kernel x bit-depth profiling.
// args: kernel(2|3) bits(8|16|32) mode(enc|dec|both) reps
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

// precomputed source image in its native type (like a numpy array)
static std::vector<ojph::ui8>  IMG8;
static std::vector<ojph::ui16> IMG16;
static std::vector<ojph::ui32> IMG32;

static ojph::si64 pixel(ojph::ui32 x, ojph::ui32 y, int bits)
{
  // mask-like: disc + squares, scaled to the bit depth
  ojph::si64 v = 0;
  ojph::si64 dx = (ojph::si64)x - 2048, dy = (ojph::si64)y - 1400;
  if (dx * dx + dy * dy < 900ll * 900ll) v = 137;
  if (x > 2500 && x < 3300 && y > 2200 && y < 3000) v = 90;
  if (x > 600 && x < 1400 && y > 2600 && y < 3400) v = 200;
  if (bits == 16) v *= 257;
  else if (bits == 32) v *= 16843009ll;
  return v;
}

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
    if (line->flags & ojph::line_buf::LFT_32BIT) {
      ojph::si32* dp = line->i32;
      if (bits == 8) {
        const ojph::ui8* sp = IMG8.data() + (size_t)y * N;
        for (ojph::ui32 x = 0; x < N; ++x) dp[x] = sp[x];
      } else if (bits == 16) {
        const ojph::ui16* sp = IMG16.data() + (size_t)y * N;
        for (ojph::ui32 x = 0; x < N; ++x) dp[x] = sp[x];
      } else {
        const ojph::ui32* sp = IMG32.data() + (size_t)y * N;
        for (ojph::ui32 x = 0; x < N; ++x) dp[x] = (ojph::si32)sp[x];
      }
    } else {
      ojph::si64* dp = line->i64;
      const ojph::ui32* sp = IMG32.data() + (size_t)y * N;
      for (ojph::ui32 x = 0; x < N; ++x) dp[x] = (ojph::si64)sp[x];
    }
    line = cs.exchange(line, nc);
  }
  cs.flush();
  cs.close();
}

static bool decode(const ojph::mem_outfile& out, int bits, bool check)
{
  ojph::mem_infile in;
  in.open(out.get_data(), const_cast<ojph::mem_outfile&>(out).get_used_size());
  ojph::codestream cs;
  cs.read_headers(&in);
  cs.create();
  bool ok = true;
  for (ojph::ui32 y = 0; y < N; ++y) {
    ojph::ui32 comp;
    ojph::line_buf* line = cs.pull(comp);
    if (check) {
      if (line->flags & ojph::line_buf::LFT_32BIT) {
        for (ojph::ui32 x = 0; x < N; ++x)
          ok &= line->i32[x] == (ojph::si32)pixel(x, y, bits);
      } else {
        for (ojph::ui32 x = 0; x < N; ++x)
          ok &= line->i64[x] == pixel(x, y, bits);
      }
    }
  }
  cs.close();
  return ok;
}

int main(int argc, char** argv)
{
  int kernel = argc > 1 ? atoi(argv[1]) : 3;
  int bits = argc > 2 ? atoi(argv[2]) : 8;
  const char* mode = argc > 3 ? argv[3] : "both";
  int reps = argc > 4 ? atoi(argv[4]) : 20;

  if (bits == 8) {
    IMG8.resize((size_t)N * N);
    for (ojph::ui32 y = 0; y < N; ++y)
      for (ojph::ui32 x = 0; x < N; ++x)
        IMG8[(size_t)y * N + x] = (ojph::ui8)pixel(x, y, bits);
  } else if (bits == 16) {
    IMG16.resize((size_t)N * N);
    for (ojph::ui32 y = 0; y < N; ++y)
      for (ojph::ui32 x = 0; x < N; ++x)
        IMG16[(size_t)y * N + x] = (ojph::ui16)pixel(x, y, bits);
  } else {
    IMG32.resize((size_t)N * N);
    for (ojph::ui32 y = 0; y < N; ++y)
      for (ojph::ui32 x = 0; x < N; ++x)
        IMG32[(size_t)y * N + x] = (ojph::ui32)pixel(x, y, bits);
  }
  ojph::mem_outfile out;
  encode(out, kernel, bits);
  size_t nbytes = out.get_used_size();
  bool ok = decode(out, bits, true);

  double t_enc = 1e9, t_dec = 1e9;
  if (strcmp(mode, "dec") != 0)
    for (int r = 0; r < reps; ++r) {
      ojph::mem_outfile o2;
      auto t0 = std::chrono::steady_clock::now();
      encode(o2, kernel, bits);
      double dt = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - t0).count();
      t_enc = t_enc < dt ? t_enc : dt;
    }
  if (strcmp(mode, "enc") != 0)
    for (int r = 0; r < reps; ++r) {
      auto t0 = std::chrono::steady_clock::now();
      decode(out, bits, false);
      double dt = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - t0).count();
      t_dec = t_dec < dt ? t_dec : dt;
    }

  printf("kernel %d bits %d: %zu bytes, lossless %d, enc %.2f ms, "
         "dec %.2f ms\n", kernel, bits, nbytes, (int)ok,
         t_enc < 1e8 ? t_enc * 1e3 : -1.0,
         t_dec < 1e8 ? t_dec * 1e3 : -1.0);
  return ok ? 0 : 1;
}
