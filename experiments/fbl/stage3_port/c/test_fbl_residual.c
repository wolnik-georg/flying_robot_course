/* Host test of fbl_residual.c against the Python reference (test_vectors.txt) plus the peer-velocity rules. Build and run: make test */
#include "fbl_residual.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* scripted peers for the differencing test (overrides the weak symbol) */
static float g_x[FBL_MAX_NEIGHBOURS], g_y[FBL_MAX_NEIGHBOURS], g_z[FBL_MAX_NEIGHBOURS]; static uint32_t g_ts[FBL_MAX_NEIGHBOURS]; static int g_n;
uint8_t peer_get_all(float *xs, float *ys, float *zs, uint32_t *ts, uint8_t max)
{ for (int k = 0; k < g_n && k < max; k++) { xs[k] = g_x[k]; ys[k] = g_y[k]; zs[k] = g_z[k]; ts[k] = g_ts[k]; } return (uint8_t)g_n; }

static int fails = 0;
#define CHECK(c, ...) do { if (!((c))) { fails++; printf("FAIL: " __VA_ARGS__); printf("\n"); } } while (0)

int main(void)
{
  FILE *f = fopen("test_vectors.txt", "r");
  if (!f) { printf("test_vectors.txt missing: run gen_test_vectors.py\n"); return 2; }
  float cd; int ncase;
  if (fscanf(f, "%f %d", &cd, &ncase) != 2) return 2;
  double max_abs_acc = 0, max_abs_jerk = 0, max_ratio_acc = 0, max_ratio_jerk = 0; int wrong_active = 0, n_nonzero = 0;
  for (int c = 0; c < ncase; c++) {
    int n; float p[3], v[3], pn[3][3] = {{0}}, vn[3][3] = {{0}}, ea[3], ej[3]; int ena;
    if (fscanf(f, "%d", &n) != 1) return 2;
    for (int i = 0; i < 3; i++) if (fscanf(f, "%f", &p[i]) != 1) return 2;
    for (int i = 0; i < 3; i++) if (fscanf(f, "%f", &v[i]) != 1) return 2;
    for (int k = 0; k < n; k++) { for (int i = 0; i < 3; i++) if (fscanf(f, "%f", &pn[k][i]) != 1) return 2; for (int i = 0; i < 3; i++) if (fscanf(f, "%f", &vn[k][i]) != 1) return 2; }
    for (int i = 0; i < 3; i++) if (fscanf(f, "%f", &ea[i]) != 1) return 2;
    for (int i = 0; i < 3; i++) if (fscanf(f, "%f", &ej[i]) != 1) return 2;
    if (fscanf(f, "%d", &ena) != 1) return 2;
    fbl_res_t out; fbl_residual_eval_abs(p, v, (const float (*)[3])pn, (const float (*)[3])vn, n, cd, &out);
    for (int i = 0; i < 3; i++) {
      const double da = fabs((double)out.acc[i] - ea[i]), dj = fabs((double)out.jerk[i] - ej[i]);
      if (da > max_abs_acc) { max_abs_acc = da; }
      if (dj > max_abs_jerk) { max_abs_jerk = dj; }
      { const double ra = da / (1e-5 + 1e-5 * fabs(ea[i])), rj = dj / (1e-4 + 1e-5 * fabs(ej[i])); if (ra > max_ratio_acc) { max_ratio_acc = ra; } if (rj > max_ratio_jerk) { max_ratio_jerk = rj; } }
    }
    if (fabs(ea[2]) > 0) n_nonzero++;
    CHECK(out.acc[0] == 0.0f && out.acc[1] == 0.0f && out.jerk[0] == 0.0f && out.jerk[1] == 0.0f, "case %d: horizontal output must be exactly zero", c);
    (void)wrong_active;
  }
  fclose(f);
  printf("vectors: %d cases (%d with a non-zero residual)\n  acceleration: max abs error %.3e m/s^2, worst error / tolerance %.3f (tolerance 1e-5 + 1e-5 |ref|)\n  jerk:         max abs error %.3e m/s^3, worst error / tolerance %.3f (tolerance 1e-4 + 1e-5 |ref|)\n", ncase, n_nonzero, max_abs_acc, max_ratio_acc, max_abs_jerk, max_ratio_jerk);
  CHECK(max_ratio_acc <= 1.0, "acceleration outside the tolerance");
  CHECK(max_ratio_jerk <= 1.0, "jerk outside the tolerance");

  /* peer-velocity differencing rules (same as NS2) */
  fbl_state_t s; fbl_res_t out; const float pos[3] = {0, 0, 0.5f}, vel[3] = {0, 0, 0};
  fbl_reset(&s); g_n = 1; g_x[0] = 0.0f; g_y[0] = 0.0f; g_z[0] = 1.0f; g_ts[0] = 1000;
  fbl_residual_eval(&s, pos, vel, 0.43f, &out);                       /* first sample: zero velocity */
  CHECK(out.n_active == 1 && s.vel[0][2] == 0.0f && out.jerk[2] == 0.0f, "first sample must give zero peer velocity (jerk %g)", out.jerk[2]);
  fbl_residual_eval(&s, pos, vel, 0.43f, &out);                       /* same timestamp: keep estimate */
  CHECK(s.vel[0][2] == 0.0f, "unchanged timestamp keeps the estimate");
  g_ts[0] = 1010; g_z[0] = 1.0f + 0.005f;                              /* 10 ms, +5 mm -> 0.5 m/s */
  fbl_residual_eval(&s, pos, vel, 0.43f, &out);
  CHECK(fabsf(s.vel[0][2] - 0.5f) < 1e-3f, "peer velocity from differencing: %g (expected 0.5)", s.vel[0][2]);
  g_ts[0] = 1700; g_z[0] = 2.0f;                                       /* 690 ms gap: resync, zero velocity */
  fbl_residual_eval(&s, pos, vel, 0.43f, &out);
  CHECK(s.vel[0][2] == 0.0f, "gap >= 500 ms resyncs with zero velocity (%g)", s.vel[0][2]);
  g_n = 3; g_x[1] = 0.05f; g_y[1] = 0; g_z[1] = 0.9f; g_ts[1] = 10; g_x[2] = 0; g_y[2] = 0.9f; g_z[2] = 1.0f; g_ts[2] = 10; g_z[0] = 1.1f; g_ts[0] = 1710;
  fbl_reset(&s); fbl_residual_eval(&s, pos, vel, 0.43f, &out);        /* 3 neighbours: the far one (0.9 m lateral) must not contribute */
  CHECK(out.n_active == 2, "three neighbours, two inside the gate: n_active %d", out.n_active);
  g_n = 0; fbl_residual_eval(&s, pos, vel, 0.43f, &out);
  CHECK(out.n_active == 0 && out.acc[2] == 0.0f, "no neighbour -> zero output");
  /* neighbour below: no residual */
  const float below_p[1][3] = {{0, 0, 0.0f}}, below_v[1][3] = {{0, 0, 0}};
  fbl_residual_eval_abs(pos, vel, below_p, below_v, 1, 0.43f, &out);
  CHECK(out.n_active == 0 && out.acc[2] == 0.0f, "neighbour below must give zero");
  printf(fails ? "RESULT: %d FAILURES\n" : "RESULT: ALL PASSED\n", fails);
  return fails ? 1 : 0;
}
