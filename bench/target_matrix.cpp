// Per-Highway-target benchmark of the dispatched kernels and of
// end-to-end encode/decode.  The first argument forces a target by
// disabling everything better, so running the binary once per target
// gives a target-vs-target matrix on one machine:
//
//   target_matrix [native|sse4|avx2|avx3|avx3_dl|avx3_zen4|avx3_spr]
//                 [path/to/mask_0.raw]
//
// The force must happen before the first dispatched call, which is why
// it is a process-level flag rather than an in-process loop: the codec
// freezes some choices (which kernels are installed, the AVX2
// hand-written survivors) when the function pointers are initialized.
// Exit code 2 means the requested target is not supported by this CPU.
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include <hwy/targets.h>

#include "ojph_defs.h"
#include "ojph_arch.h"
#include "ojph_file.h"
#include "ojph_mem.h"
#include "ojph_params.h"
#include "ojph_codestream.h"
// internal headers
#include "ojph_params_local.h"
#include "ojph_transform.h"
#include "ojph_colour.h"

namespace ojph { namespace local {
  // generic and hwy codestream kernels (ojph_codestream_gen/hwy.cpp)
  ui32 gen_find_max_val32(ui32*);
  ui32 find_max_val32(ui32*);
  void gen_rev_tx_to_cb32(const void*, ui32*, ui32, float, ui32, ui32*);
  void rev_tx_to_cb32(const void*, ui32*, ui32, float, ui32, ui32*);
  void irv_tx_to_cb32(const void*, ui32*, ui32, float, ui32, ui32*);
  void gen_rev_tx_from_cb16(const ui32*, si16*, ui32, ui32);
  void rev_tx_from_cb16(const ui32*, si16*, ui32, ui32);
  void gen_rev_tx_from_cb32(const ui32*, void*, ui32, float, ui32);
  void avx2_rev_tx_from_cb32(const ui32*, void*, ui32, float, ui32);
  void rev_tx_from_cb32(const ui32*, void*, ui32, float, ui32);
  void irv_tx_from_cb32(const ui32*, void*, ui32, float, ui32);
  void rev_convert16(const si16*, si32*, si32, ui32);
  // the hand-written AVX2 vertical step survivor
  void avx2_rev_vert_step(const lifting_step*, const line_buf*,
                          const line_buf*, const line_buf*, ui32, bool);
  // per-target instantiations of the hwy vertical step, to compare
  // against the survivor (the dispatched pointer prefers the survivor)
  namespace N_AVX2 {
    void rev_vert_step32(const lifting_step*, const si32*, const si32*,
                         si32*, ui32, bool);
  }
  namespace N_AVX3 {
    void rev_vert_step32(const lifting_step*, const si32*, const si32*,
                         si32*, ui32, bool);
  }
}}

using namespace ojph;
using namespace ojph::local;

static const ui32 W = 4096;
static const int  ITERS = 4000;
static const int  REPS  = 15;

// carve lines from one arena so relative placement is deterministic
static char* arena = NULL;
static size_t arena_used = 0;
template <typename T> static T* alloc_line(ui32 width)
{
  const size_t arena_size = 1 << 24;
  if (arena == NULL)
    arena = (char*)aligned_alloc(4096, arena_size);
  size_t bytes = ((size_t)(width + 2) * sizeof(T) + 191) & ~(size_t)63;
  char* p = arena + arena_used;
  arena_used += bytes + 320;
  if (arena_used > arena_size)
    abort();
  memset(p, 0, bytes);
  return (T*)(p + 64);
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

static void report_n(const char* name, double t, ui32 n)
{
  printf("  %-36s %8.1f ns/line  %6.2f GS/s\n", name, t * 1e9,
         n / t * 1e-9);
}

static void report(const char* name, double t)
{
  report_n(name, t, W);
}

//----------------------------------------------------------------------
// end-to-end encode/decode of a 4096x4096 8-bit mask
static std::vector<ui8> load_mask(const char* path)
{
  std::vector<ui8> img((size_t)W * W);
  FILE* f = fopen(path, "rb");
  if (!f || fread(img.data(), 1, img.size(), f) != img.size())
  {
    printf("cannot read %s\n", path);
    exit(1);
  }
  fclose(f);
  return img;
}

static void encode(const std::vector<ui8>& img, int kernel,
                   mem_outfile& out)
{
  ojph::codestream cs;
  ojph::param_siz siz = cs.access_siz();
  siz.set_image_extent(point(W, W));
  siz.set_num_components(1);
  siz.set_component(0, point(1, 1), 8, false);
  siz.set_image_offset(point(0, 0));
  siz.set_tile_offset(point(0, 0));
  ojph::param_cod cod = cs.access_cod();
  cod.set_num_decomposition(8);
  cod.set_reversible(true);
  cod.set_wavelet_kern((ui32)kernel);
  cs.set_planar(false);

  out.open(1 << 20);
  cs.write_headers(&out);
  ui32 nc;
  line_buf* line = cs.exchange(NULL, nc);
  for (ui32 y = 0; y < W; ++y) {
    const ui8* sp = img.data() + (size_t)y * W;
    if (line->flags & line_buf::LFT_32BIT)
      for (ui32 x = 0; x < W; ++x) line->i32[x] = sp[x];
    else
      for (ui32 x = 0; x < W; ++x) line->i16[x] = sp[x];
    line = cs.exchange(line, nc);
  }
  cs.flush();
  cs.close();
}

static void decode(mem_outfile& out)
{
  mem_infile in;
  in.open(out.get_data(), out.get_used_size());
  ojph::codestream cs;
  cs.read_headers(&in);
  cs.create();
  for (ui32 y = 0; y < W; ++y) {
    ui32 comp;
    cs.pull(comp);
  }
  cs.close();
}

//----------------------------------------------------------------------
static int64_t parse_target(const std::string& s)
{
  if (s == "native")    return 0;
  if (s == "sse4")      return HWY_SSE4;
  if (s == "avx2")      return HWY_AVX2;
  if (s == "avx3")      return HWY_AVX3;
  if (s == "avx3_dl")   return HWY_AVX3_DL;
  if (s == "avx3_zen4") return HWY_AVX3_ZEN4;
  if (s == "avx3_spr")  return HWY_AVX3_SPR;
  printf("unknown target %s\n", s.c_str());
  exit(1);
}

int main(int argc, char** argv)
{
  const std::string tname = argc > 1 ? argv[1] : "native";
  const char* mask_path = argc > 2 ? argv[2] : "mask_0.raw";

  const int64_t bit = parse_target(tname);
  if (bit != 0)
  {
    // keep only the requested target and worse ones; must precede any
    // dispatched call so the freeze-time choices see the same view
    hwy::DisableTargets(bit - 1);
    if ((hwy::SupportedTargets() & bit) == 0)
    {
      printf("%s: not supported on this CPU\n", tname.c_str());
      return 2;
    }
  }
  {
    int64_t sup = hwy::SupportedTargets();
    // SupportedTargets() re-initializes hwy's chosen dispatch target
    // with the full detected set and counts on its caller to narrow it
    // to the returned (masked) set; do so, or the force above would be
    // ignored by dispatch
    hwy::GetChosenTarget().Update(sup);
    printf("forced %s; best dispatch target now %s\n", tname.c_str(),
           hwy::TargetName(sup & (-sup)));
  }

  init_wavelet_transform_functions();
  init_colour_transform_functions();

  param_atk atk13, atk12, root;
  atk13.init_rev13();
  atk12.init_rev12();
  param_atk& atk53 = *root.get_atk(1);  // the classic rev 5/3
  param_atk& atk97 = *root.get_atk(0);  // the irreversible 9/7

  printf("horizontal, %u samples/line, best of %d x %d iters\n",
         W, REPS, ITERS);
  {
    lines x; x.init<si32>();
    report("rev13 WS horz ana", best_of([&]{
      rev_horz_ana(&atk13, &x.l, &x.h, &x.s, W, true); }));
    report("rev13 WS horz syn", best_of([&]{
      rev_horz_syn(&atk13, &x.s, &x.l, &x.h, W, true); }));
    report("rev53 WS horz ana", best_of([&]{
      rev_horz_ana(&atk53, &x.l, &x.h, &x.s, W, true); }));
    report("rev53 WS horz syn", best_of([&]{
      rev_horz_syn(&atk53, &x.s, &x.l, &x.h, W, true); }));
    report("rev12 prev horz ana si32", best_of([&]{
      rev_horz_ana_arb(&atk12, &x.l, &x.h, &x.s, W, true); }));
    report("rev12 prev horz syn si32", best_of([&]{
      rev_horz_syn_arb(&atk12, &x.s, &x.l, &x.h, W, true); }));
  }
  {
    lines x; x.init<si16>();
    x.l.flags = line_buf::LFT_16BIT | line_buf::LFT_INTEGER;
    x.h.flags = line_buf::LFT_16BIT | line_buf::LFT_INTEGER;
    x.s.flags = line_buf::LFT_16BIT | line_buf::LFT_INTEGER;
    report("rev12 prev horz ana si16", best_of([&]{
      rev_horz_ana_arb(&atk12, &x.l, &x.h, &x.s, W, true); }));
    report("rev12 prev horz syn si16", best_of([&]{
      rev_horz_syn_arb(&atk12, &x.s, &x.l, &x.h, W, true); }));
  }
  {
    lines x; x.init<float>();
    report("irv97 horz ana", best_of([&]{
      irv_horz_ana(&atk97, &x.l, &x.h, &x.s, W, true); }));
    report("irv97 horz syn", best_of([&]{
      irv_horz_syn(&atk97, &x.s, &x.l, &x.h, W, true); }));
  }

  printf("vertical, %u samples/step\n", W);
  {
    lines a, b, c;
    a.init<si32>(); b.init<si32>(); c.init<si32>();
    report("rev13 vert predict (dispatched)", best_of([&]{
      rev_vert_step(atk13.get_step(1), &a.s, &b.s, &c.s, W, false); }));
#if defined(OJPH_ARCH_X86_64) || defined(OJPH_ARCH_I386)
    if (get_cpu_ext_level() >= X86_CPU_EXT_LEVEL_AVX2)
      report("rev13 vert predict (avx2 surv)", best_of([&]{
        avx2_rev_vert_step(atk13.get_step(1), &a.s, &b.s, &c.s,
                           W, false); }));
    if (hwy::SupportedTargets() & HWY_AVX2)
      report("rev13 vert predict (hwy avx2)", best_of([&]{
        N_AVX2::rev_vert_step32(atk13.get_step(1), a.s.i32, b.s.i32,
                                c.s.i32, W, false); }));
    if (hwy::SupportedTargets() & HWY_AVX3)
      report("rev13 vert predict (hwy avx3)", best_of([&]{
        N_AVX3::rev_vert_step32(atk13.get_step(1), a.s.i32, b.s.i32,
                                c.s.i32, W, false); }));
#endif
  }

  printf("colour, %u samples/line\n", W);
  {
    lines a, b, c;
    a.init<si32>(); b.init<si32>(); c.init<si32>();
    report("rct forward", best_of([&]{
      rct_forward(&a.s, &b.s, &c.s, &a.l, &b.l, &c.l, W / 2); }));
    report("rct backward", best_of([&]{
      rct_backward(&a.l, &b.l, &c.l, &a.s, &b.s, &c.s, W / 2); }));
    report("rev_convert 32->32", best_of([&]{
      rev_convert(&a.s, 0, &b.s, 0, 128, W); }));
  }

  printf("codeblock tx, 1024 samples\n");
  {
    const ui32 n = 1024, K = 8;
    std::vector<ui32> cb(n + 64);
    for (ui32 i = 0; i < n + 64; ++i)
      cb[i] = (i * 2654435761u) & 0x807FFFFFu; // sign + 23 mag bits
    std::vector<si32> l32(n + 64);
    std::vector<si16> l16(n + 64);
    std::vector<float> lf(n + 64);
    for (ui32 i = 0; i < n + 64; ++i) {
      l32[i] = (si32)(i * 2654435761u) >> 12;
      lf[i] = (float)l32[i];
    }
    ui32 max_val[8];
    report_n("tx_to_cb32 rev (hwy)", best_of([&]{
      memset(max_val, 0, sizeof(max_val));
      rev_tx_to_cb32(l32.data(), cb.data(), K, 0.f, n, max_val); }), n);
    report_n("tx_to_cb32 irv (hwy)", best_of([&]{
      memset(max_val, 0, sizeof(max_val));
      irv_tx_to_cb32(lf.data(), cb.data(), K, 0.5f, n, max_val); }), n);
    report_n("tx_from_cb32 rev (hwy)", best_of([&]{
      rev_tx_from_cb32(cb.data(), l32.data(), K, 0.f, n); }), n);
#if defined(OJPH_ARCH_X86_64) || defined(OJPH_ARCH_I386)
    if (get_cpu_ext_level() >= X86_CPU_EXT_LEVEL_AVX2)
      report_n("tx_from_cb32 rev (avx2 surv)", best_of([&]{
        avx2_rev_tx_from_cb32(cb.data(), l32.data(), K, 0.f, n); }), n);
#endif
    report_n("tx_from_cb32 irv (hwy)", best_of([&]{
      irv_tx_from_cb32(cb.data(), lf.data(), K, 0.5f, n); }), n);
    report_n("tx_from_cb16 (hwy)", best_of([&]{
      rev_tx_from_cb16(cb.data(), l16.data(), K, n); }), n);
    report_n("rev_convert16", best_of([&]{
      rev_convert16(l16.data(), l32.data(), 128, n); }), n);
    double t = best_of([&]{ find_max_val32(max_val); });
    printf("  %-36s %8.2f ns/call\n", "find_max_val32", t * 1e9);
  }

  printf("end-to-end, 4096x4096 u8 mask, best of 5\n");
  {
    std::vector<ui8> img = load_mask(mask_path);
    for (int k = 2; k <= 3; ++k)
    {
      mem_outfile out;
      double te = 1e99, td = 1e99;
      for (int r = 0; r < 5; ++r)
      {
        auto t0 = std::chrono::steady_clock::now();
        encode(img, k, out);
        double dt = std::chrono::duration<double>(
          std::chrono::steady_clock::now() - t0).count();
        te = te < dt ? te : dt;
      }
      for (int r = 0; r < 5; ++r)
      {
        auto t0 = std::chrono::steady_clock::now();
        decode(out);
        double dt = std::chrono::duration<double>(
          std::chrono::steady_clock::now() - t0).count();
        td = td < dt ? td : dt;
      }
      printf("  kernel %d  encode %7.2f ms  decode %7.2f ms  "
             "(%zu bytes)\n", k, te * 1e3, td * 1e3,
             (size_t)out.get_used_size());
    }
  }
  return 0;
}
