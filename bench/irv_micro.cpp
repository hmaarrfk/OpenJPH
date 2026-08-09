// Microbenchmark of every kernel on the irreversible (9/7 float) path:
// DWT horizontal analysis/synthesis and vertical lifting steps, the K
// scaling, the ICT colour transform, the float<->integer line converts,
// and the quantization transfer kernels (tx_to_cb32/tx_from_cb32).
// Times whatever is behind the dispatched function pointers (SIMD when
// available) plus the generic implementations for reference, so the same
// harness compares builds.
//
// Build (from the repo root, after building build-t):
//   g++ -O3 -std=c++14 -o bench/irv_micro bench/irv_micro.cpp \
//     -I src/core/openjph -I src/core/codestream -I src/core/transform \
//     -I src/core/coding build-t/src/core/libojph.a  # or link the .so
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>

#include "ojph_mem.h"
#include "ojph_params.h"
// internal headers
#include "ojph_params_local.h"
#include "ojph_transform.h"
#include "ojph_colour.h"
#include "ojph_codeblock_fun.h"

using namespace ojph;
using namespace ojph::local;

namespace ojph { namespace local {
  // generic implementations (non-static; normally reached via dispatch)
  void gen_irv_horz_ana(const param_atk*, const line_buf*, const line_buf*,
                        const line_buf*, ui32, bool);
  void gen_irv_horz_syn(const param_atk*, const line_buf*, const line_buf*,
                        const line_buf*, ui32, bool);
  void gen_irv_vert_step(const lifting_step*, const line_buf*,
                         const line_buf*, const line_buf*, ui32, bool);
  void gen_irv_vert_times_K(float, const line_buf*, ui32);
  void gen_ict_forward(const float*, const float*, const float*,
                       float*, float*, float*, ui32);
  void gen_ict_backward(const float*, const float*, const float*,
                        float*, float*, float*, ui32);
  void gen_irv_convert_to_float(const line_buf*, ui32, line_buf*,
                                ui32, bool, ui32);
  void gen_irv_convert_to_integer(const line_buf*, line_buf*, ui32,
                                  ui32, bool, ui32);
  void gen_irv_tx_to_cb32(const void*, ui32*, ui32, float, ui32, ui32*);
  void gen_irv_tx_from_cb32(const ui32*, void*, ui32, float, ui32);
}}

static const ui32 W = 4096;
static int ITERS = 4000;   // per rep
static int REPS  = 20;     // best-of

// carve lines from one arena so relative line placement is deterministic
static char* arena = NULL;
static size_t arena_used = 0;
template <typename T> static T* alloc_line(ui32 width)
{
  const size_t arena_size = 1 << 24;
  if (arena == NULL)
    arena = (char*)aligned_alloc(4096, arena_size);
  size_t bytes = ((size_t)(width + 2) * sizeof(T) + 191) & ~(size_t)63;
  char* p = arena + arena_used;
  arena_used += bytes + 320; // stagger lines to avoid 4K aliasing
  if (arena_used > arena_size)
    abort();
  memset(p, 0, bytes);
  return (T*)(p + 64); // leave one 64B block before the line
}

static void fillf(float* p, ui32 n)
{
  for (ui32 i = 0; i < n; ++i)
    p[i] = 0.25f * sinf(0.013f * (float)i) +
           0.0625f * (float)((i * 37) % 251) / 251.0f - 0.15f;
}

struct lines
{
  line_buf l, h, s;
  void init()
  {
    float* lb = alloc_line<float>(W); fillf(lb, W / 2);
    float* hb = alloc_line<float>(W); fillf(hb, W / 2);
    float* sb = alloc_line<float>(W); fillf(sb, W);
    l.wrap(lb, W / 2 + 1, 1);
    h.wrap(hb, W / 2 + 1, 1);
    s.wrap(sb, W + 1, 1);
  }
};

template <typename F> static double best_of(F&& fn)
{
  double best = 1e99;
  for (int r = 0; r < REPS; ++r)
  {
    auto t0 = std::chrono::steady_clock::now();
    for (int i = 0; i < ITERS; ++i)
      fn();
    double dt = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - t0).count() / ITERS;
    best = best < dt ? best : dt;
  }
  return best;
}

static void report(const char* name, double t, ui32 n = W)
{
  printf("  %-36s %8.1f ns  %6.2f GS/s\n", name, t * 1e9, n / t * 1e-9);
  fflush(stdout);
}

int main(int argc, char** argv)
{
  if (argc > 1) ITERS = atoi(argv[1]);
  init_wavelet_transform_functions();
  init_colour_transform_functions();

  param_atk atk97_root;
  param_atk& atk97 = *atk97_root.get_atk(0); // init_irv97

  printf("irv97 kernels, %u samples, best of %d x %d iters\n",
         W, REPS, ITERS);

  { // horizontal DWT
    lines x; x.init();
    report("irv97 horz ana (dispatched)", best_of([&]{
      irv_horz_ana(&atk97, &x.l, &x.h, &x.s, W, true); }));
    report("irv97 horz ana (gen)", best_of([&]{
      gen_irv_horz_ana(&atk97, &x.l, &x.h, &x.s, W, true); }));
    report("irv97 horz syn (dispatched)", best_of([&]{
      irv_horz_syn(&atk97, &x.s, &x.l, &x.h, W, true); }));
    report("irv97 horz syn (gen)", best_of([&]{
      gen_irv_horz_syn(&atk97, &x.s, &x.l, &x.h, W, true); }));
  }

  { // vertical DWT
    lines a, b, c;
    a.init(); b.init(); c.init();
    report("irv97 vert step (dispatched)", best_of([&]{
      irv_vert_step(atk97.get_step(0), &a.s, &b.s, &c.s, W, false); }));
    report("irv97 vert step (gen)", best_of([&]{
      gen_irv_vert_step(atk97.get_step(0), &a.s, &b.s, &c.s, W, false); }));
    report("irv97 vert times K (dispatched)", best_of([&]{
      irv_vert_times_K(atk97.get_K(), &c.s, W); }));
    report("irv97 vert times K (gen)", best_of([&]{
      gen_irv_vert_times_K(atk97.get_K(), &c.s, W); }));
  }

  { // ICT
    float* r = alloc_line<float>(W); fillf(r, W);
    float* g = alloc_line<float>(W); fillf(g, W);
    float* b = alloc_line<float>(W); fillf(b, W);
    float* y = alloc_line<float>(W);
    float* cb = alloc_line<float>(W);
    float* cr = alloc_line<float>(W);
    report("ict forward (dispatched)", best_of([&]{
      ict_forward(r, g, b, y, cb, cr, W); }));
    report("ict forward (gen)", best_of([&]{
      gen_ict_forward(r, g, b, y, cb, cr, W); }));
    report("ict backward (dispatched)", best_of([&]{
      ict_backward(y, cb, cr, r, g, b, W); }));
    report("ict backward (gen)", best_of([&]{
      gen_ict_backward(y, cb, cr, r, g, b, W); }));
  }

  { // integer <-> float line converts (encode / decode side)
    line_buf iline, fline;
    si32* ip = alloc_line<si32>(W);
    for (ui32 i = 0; i < W; ++i)
      ip[i] = (si32)((i * 37) % 251);
    float* fp = alloc_line<float>(W); fillf(fp, W);
    iline.wrap(ip, W + 1, 1);
    fline.wrap(fp, W + 1, 1);
    report("irv convert int->float u8 (disp.)", best_of([&]{
      irv_convert_to_float(&iline, 0, &fline, 8, false, W); }));
    report("irv convert int->float u8 (gen)", best_of([&]{
      gen_irv_convert_to_float(&iline, 0, &fline, 8, false, W); }));
    report("irv convert int->float u16 (disp.)", best_of([&]{
      irv_convert_to_float(&iline, 0, &fline, 16, false, W); }));
    fillf(fp, W);
    report("irv convert float->int u8 (disp.)", best_of([&]{
      irv_convert_to_integer(&fline, &iline, 0, 8, false, W); }));
    report("irv convert float->int u8 (gen)", best_of([&]{
      gen_irv_convert_to_integer(&fline, &iline, 0, 8, false, W); }));
    report("irv convert float->int u16 (disp.)", best_of([&]{
      irv_convert_to_integer(&fline, &iline, 0, 16, false, W); }));
  }

  { // quantization transfer kernels, at codeblock width and line width
    codeblock_fun cf;
    cf.init(false); // irreversible
    float* fp = alloc_line<float>(W); fillf(fp, W);
    ui32* cbp = alloc_line<ui32>(W);
    float* op = alloc_line<float>(W);
    ui32 max_val[8];
    const float delta = 1.0f / 8192.0f, delta_inv = 8192.0f;
    for (ui32 w : { 64u, W })
    {
      char nm[64];
      memset(max_val, 0, sizeof(max_val));
      snprintf(nm, sizeof(nm), "irv tx_to_cb32 w=%u (dispatched)", w);
      report(nm, best_of([&]{
        cf.tx_to_cb32(fp, cbp, 30, delta_inv, w, max_val); }), w);
      snprintf(nm, sizeof(nm), "irv tx_to_cb32 w=%u (gen)", w);
      report(nm, best_of([&]{
        gen_irv_tx_to_cb32(fp, cbp, 30, delta_inv, w, max_val); }), w);
      snprintf(nm, sizeof(nm), "irv tx_from_cb32 w=%u (dispatched)", w);
      report(nm, best_of([&]{
        cf.tx_from_cb32(cbp, op, 30, delta, w); }), w);
      snprintf(nm, sizeof(nm), "irv tx_from_cb32 w=%u (gen)", w);
      report(nm, best_of([&]{
        gen_irv_tx_from_cb32(cbp, op, 30, delta, w); }), w);
    }
  }
  return 0;
}
