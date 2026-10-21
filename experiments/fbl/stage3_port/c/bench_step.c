#define _POSIX_C_SOURCE 199309L
/* Host micro-benchmark: one FBL step (residual eval + position law + command mapping) against a dense network of the size of NS2 (about 11.5k multiply-adds, same layer shapes).
   The NS2 evaluation takes 526 us on the drone (measured); the ratio of the host times scales that to the FBL step.  make bench */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include "fbl_law.h"
#include "fbl_residual.h"
static double now(void) { struct timespec t; clock_gettime(CLOCK_MONOTONIC, &t); return t.tv_sec + 1e-9 * t.tv_nsec; }
static float net_eval(const float *w, const int *dims, int nl, float *a, float *b, int in)
{   /* dense layers dims[0]->dims[1]->..., relu, weights consumed sequentially */
  int o = 0; (void)in;
  for (int l = 0; l < nl; l++) {
    for (int j = 0; j < dims[l + 1]; j++) { float s = w[o++]; for (int i = 0; i < dims[l]; i++) s += w[o++] * a[i]; b[j] = s > 0 ? s : 0; }
    for (int j = 0; j < dims[l + 1]; j++) a[j] = b[j];
  }
  return a[0];
}
int main(void)
{
  const int N = 200000; double t0, t_fbl, t_net; volatile float sink = 0;
  fbl_gains_t g; fbl_gains_default_bundle(&g); fbl_law_state_t st; fbl_law_reset(&st); fbl_state_t ps; fbl_reset(&ps);
  fbl_ref_t ref = {{0, 0, 0.5f}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
  float pos[3] = {0.05f, 0.02f, 0.5f}, vel[3] = {0, 0, 0}, R[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1}, w[3] = {0.01f, 0, 0}, acc[3], jerk[3], thrust, wd[3], tq[3], res_ang[3] = {0, 0, 0};
  float peer[1][3] = {{0.06f, 0.0f, 1.0f}}, pvel[1][3] = {{0, 0, 0}}; fbl_res_t res;
  t0 = now();
  for (int i = 0; i < N; i++) {
    pos[0] = 0.05f + 1e-4f * (i % 7); peer[0][0] = 0.06f + 1e-4f * (i % 5);
    fbl_residual_eval_abs(pos, vel, (const float (*)[3])peer, (const float (*)[3])pvel, 1, 0.43f, &res);
    fbl_law_position_dt(&st, &g, 0.002f, pos, vel, &ref, acc, jerk);
    fbl_law_command(R, w, acc, jerk, 0.0f, 0.0f, res.acc, res.jerk, res_ang, &g, &thrust, wd, tq);
    sink += thrust + wd[0] + tq[1];
  }
  t_fbl = (now() - t0) / N * 1e6;
  /* NS2-sized network: phi_s 6-25-40-40-20, phi_g 4-25-40-40-20, rho 20-40-40-40-1 (about 11.5k parameters) */
  static float W[20000]; for (int i = 0; i < 20000; i++) W[i] = 0.01f * ((i * 7919) % 200 - 100);
  const int d1[] = {6, 25, 40, 40, 20}, d2[] = {20, 40, 40, 40, 1}; float a[64], b[64];
  t0 = now();
  for (int i = 0; i < N; i++) {
    for (int k = 0; k < 6; k++) a[k] = 0.01f * k + 1e-4f * (i % 9); sink += net_eval(W, d1, 4, a, b, 6);
    for (int k = 0; k < 4; k++) a[k] = 0.02f * k; sink += net_eval(W, d1, 4, a, b, 4);
    for (int k = 0; k < 20; k++) a[k] = 0.01f * k; sink += net_eval(W + 8000, d2, 4, a, b, 20);
  }
  t_net = (now() - t0) / N * 1e6;
  printf("host: FBL step %.3f us, NS2-sized network %.3f us -> ratio %.4f\n", t_fbl, t_net, t_fbl / t_net);
  printf("estimated FBL step on the drone: %.0f us (NS2 evaluation there: 526 us; controller slot 2000 us)\n", t_fbl / t_net * 526.0);
  return (int)sink == 12345;
}
