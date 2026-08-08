// Micro-benchmark of the reversible DWT paths on 4096-sample lines.
// Benchmarks whatever is behind the dispatched function pointers (SIMD
// when available) plus the generic arb paths, for rev13 (WS) and rev12
// (ARB prev-sample) kernels, in the line data types the codec uses.
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>

#include "ojph_mem.h"
#include "ojph_params.h"
// internal headers
#include "ojph_params_local.h"
#include "ojph_transform.h"

using namespace ojph;
using namespace ojph::local;

static const ui32 W = 4096;
static const int  ITERS = 4000;   // per rep
static const int  REPS  = 20;     // best-of

// carve lines from one arena, like the codec allocator does, so that the
// relative placement of the lines is deterministic (independent heap
// allocations can alias in the cache and make timings bimodal)
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

template <typename T> static void fill(T* p, ui32 n)
{
  for (ui32 i = 0; i < n; ++i)
    p[i] = (T)((i * 37) % 251);
}

struct lines
{
  line_buf l, h, s;
  template <typename T> void init()
  {
    T* lb = alloc_line<T>(W); fill(lb, W / 2);
    T* hb = alloc_line<T>(W); fill(hb, W / 2);
    T* sb = alloc_line<T>(W); fill(sb, W);
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

static void report(const char* name, double t)
{
  printf("  %-34s %8.1f ns/line  %6.2f GS/s\n", name, t * 1e9,
         W / t * 1e-9);
}

int main()
{
  init_wavelet_transform_functions();

  param_atk atk13, atk12, atk53_root;
  atk13.init_rev13();
  atk12.init_rev12();
  param_atk& atk53 = *atk53_root.get_atk(1); // init_rev53

  printf("horizontal, %u samples/line, best of %d x %d iters\n",
         W, REPS, ITERS);
  {
    lines x; x.init<si32>();
    report("rev13 WS horz ana (dispatched)", best_of([&]{
      rev_horz_ana(&atk13, &x.l, &x.h, &x.s, W, true); }));
    report("rev13 WS horz syn (dispatched)", best_of([&]{
      rev_horz_syn(&atk13, &x.s, &x.l, &x.h, W, true); }));
    report("rev53 WS horz ana (dispatched)", best_of([&]{
      rev_horz_ana(&atk53, &x.l, &x.h, &x.s, W, true); }));
    report("rev53 WS horz syn (dispatched)", best_of([&]{
      rev_horz_syn(&atk53, &x.s, &x.l, &x.h, W, true); }));
    report("rev12 arb horz ana si32", best_of([&]{
      rev_horz_ana_arb(&atk12, &x.l, &x.h, &x.s, W, true); }));
    report("rev12 arb horz syn si32", best_of([&]{
      rev_horz_syn_arb(&atk12, &x.s, &x.l, &x.h, W, true); }));
  }
  {
    lines x; x.init<si16>();
    x.l.flags = line_buf::LFT_16BIT | line_buf::LFT_INTEGER;
    x.h.flags = line_buf::LFT_16BIT | line_buf::LFT_INTEGER;
    x.s.flags = line_buf::LFT_16BIT | line_buf::LFT_INTEGER;
    report("rev12 arb horz ana si16", best_of([&]{
      rev_horz_ana_arb(&atk12, &x.l, &x.h, &x.s, W, true); }));
    report("rev12 arb horz syn si16", best_of([&]{
      rev_horz_syn_arb(&atk12, &x.s, &x.l, &x.h, W, true); }));
  }

  printf("vertical, %u samples/step, best of %d x %d iters\n",
         W, REPS, ITERS);
  {
    lines a, b, c;
    a.init<si32>(); b.init<si32>(); c.init<si32>();
    report("rev13 WS vert predict (disp.)", best_of([&]{
      rev_vert_step(atk13.get_step(1), &a.s, &b.s, &c.s, W, false); }));
    report("rev13 WS vert null step (disp.)", best_of([&]{
      rev_vert_step(atk13.get_step(0), &a.s, &b.s, &c.s, W, false); }));
    report("rev12 one-tap vert si32", best_of([&]{
      rev_vert_step_one_tap(atk12.get_step(1), &a.s, &c.s, W, false); }));
  }
  {
    lines a, c;
    a.init<si16>(); c.init<si16>();
    a.s.flags = line_buf::LFT_16BIT | line_buf::LFT_INTEGER;
    c.s.flags = line_buf::LFT_16BIT | line_buf::LFT_INTEGER;
    report("rev12 one-tap vert si16", best_of([&]{
      rev_vert_step_one_tap(atk12.get_step(1), &a.s, &c.s, W, false); }));
  }
  return 0;
}
