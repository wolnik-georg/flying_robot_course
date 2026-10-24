/* Gate 4: network term of the residual (fbl_nn_pair) against the bundle's DownwashResidual._acceleration_nn_analytical_jvp (gen_nn_vectors.py):
   sample checkpoint (9 neurons) and a 4-neuron net (paper hardware size). Pass: within acc 1e-5 + 1e-5|ref|, jerk 1e-4 + 1e-5|ref| of the bundle (as test_fbl_residual), OR at least as accurate as the bundle's own float32
   result against the float64 evaluation of the same code (C error vs float64 <= 2 x bundle float32 error vs float64 + 1e-7). */
#include <math.h>
#include <stdio.h>
#include "fbl_residual.h"
static int run(const char *path)
{
  FILE *f = fopen(path, "r"); if (!f) { printf("%s missing\n", path); return 2; }
  fbl_nn_t nn = {0}; int nh, n;
  if (fscanf(f, "%d", &nh) != 1 || nh > FBL_NN_HMAX) return 2; nn.nh = (uint8_t)nh;
  for (int k = 0; k < nh; k++) for (int i = 0; i < 2; i++) if (fscanf(f, "%f", &nn.w1[k][i]) != 1) return 2;
  for (int k = 0; k < nh; k++) if (fscanf(f, "%f", &nn.b1[k]) != 1) return 2;
  for (int i = 0; i < 3; i++) for (int k = 0; k < nh; k++) if (fscanf(f, "%f", &nn.w2[i][k]) != 1) return 2;
  for (int i = 0; i < 3; i++) if (fscanf(f, "%f", &nn.b2[i]) != 1) return 2;
  if (fscanf(f, "%d", &n) != 1) return 2;
  double wa = 0, wj = 0, ma = 0, mj = 0, c64a = 0, c64j = 0, b64a = 0, b64j = 0, fail = 0;
  for (int s = 0; s < n; s++) {
    float p[3], v[3], pt[3], vt[3], ea[3], ej[3]; double ta[3], tj[3];
    for (int i = 0; i < 3; i++) if (fscanf(f, "%f", &p[i]) != 1) return 2;
    for (int i = 0; i < 3; i++) if (fscanf(f, "%f", &v[i]) != 1) return 2;
    for (int i = 0; i < 3; i++) if (fscanf(f, "%f", &pt[i]) != 1) return 2;
    for (int i = 0; i < 3; i++) if (fscanf(f, "%f", &vt[i]) != 1) return 2;
    for (int i = 0; i < 3; i++) if (fscanf(f, "%f", &ea[i]) != 1) return 2;
    for (int i = 0; i < 3; i++) if (fscanf(f, "%f", &ej[i]) != 1) return 2;
    for (int i = 0; i < 3; i++) if (fscanf(f, "%lf", &ta[i]) != 1) return 2;
    for (int i = 0; i < 3; i++) if (fscanf(f, "%lf", &tj[i]) != 1) return 2;
    float a[3] = {0}, j[3] = {0}; fbl_nn_pair(&nn, p, v, pt, vt, a, j);
    for (int i = 0; i < 3; i++) {
      const double da = fabs(a[i] - ea[i]), dj = fabs(j[i] - ej[i]);
      if (da > ma) ma = da;
      if (dj > mj) mj = dj;
      const double ca = fabs(a[i] - ta[i]), cj = fabs(j[i] - tj[i]), ba = fabs(ea[i] - ta[i]), bj = fabs(ej[i] - tj[i]);
      if (ca > c64a) c64a = ca;
      if (cj > c64j) c64j = cj;
      if (ba > b64a) b64a = ba;
      if (bj > b64j) b64j = bj;
      const double ra = (da / (1e-5 + 1e-5 * fabs(ea[i])) <= 1.0) ? 0.0 : ca / (2.0 * ba + 1e-7);
      const double rj = (dj / (1e-4 + 1e-5 * fabs(ej[i])) <= 1.0) ? 0.0 : cj / (2.0 * bj + 1e-7);
      if (da / (1e-5 + 1e-5 * fabs(ea[i])) > wa) wa = da / (1e-5 + 1e-5 * fabs(ea[i]));
      if (dj / (1e-4 + 1e-5 * fabs(ej[i])) > wj) wj = dj / (1e-4 + 1e-5 * fabs(ej[i]));
      if (ra > fail) fail = ra;
      if (rj > fail) fail = rj;
    }
  }
  fclose(f);
  printf("%s: %d neurons, %d states\n  vs bundle float32: acc max %.3e (worst / tol %.3f), jerk max %.3e (worst / tol %.3f)\n"
         "  vs float64 truth: C acc %.3e jerk %.3e | bundle float32 acc %.3e jerk %.3e | outside tolerance AND less accurate than the bundle: worst ratio %.3f\n",
         path, nh, n, ma, wa, mj, wj, c64a, c64j, b64a, b64j, fail);
  return (fail <= 1.0) ? 0 : 1;
}
int main(void)
{
  int r = run("nn_vectors_sample.txt") | run("nn_vectors_h4.txt");
  printf(r == 0 ? "RESULT: ALL PASSED\n" : "RESULT: FAILED\n");
  return r;
}
