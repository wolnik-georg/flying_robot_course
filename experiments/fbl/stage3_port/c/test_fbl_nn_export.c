/* End-to-end check of the weights export: the sample checkpoint exported by export_fbl_nn_weights.py and linked as the weak symbol fbl_nn_weights,
   evaluated through fbl_residual_set_nn / fbl_nn_pair, against the bundle (nn_vectors_sample.txt, columns as in test_fbl_nn.c). */
#include <math.h>
#include <stdio.h>
#include "fbl_residual.h"
int main(void)
{
  if (&fbl_nn_weights == 0) { printf("fbl_nn_weights not linked\n"); return 2; }
  FILE *f = fopen("nn_vectors_sample.txt", "r"); if (!f) return 2;
  int nh, n; float dummy; if (fscanf(f, "%d", &nh) != 1) return 2;
  if (nh != fbl_nn_weights.nh) { printf("neurons %d vs %d\n", nh, fbl_nn_weights.nh); return 1; }
  for (int k = 0; k < nh * 2 + nh + 3 * nh + 3; k++) if (fscanf(f, "%f", &dummy) != 1) return 2;
  if (fscanf(f, "%d", &n) != 1) return 2;
  double wa = 0, wj = 0;
  for (int s = 0; s < n; s++) {
    float x[18]; double t[6];
    for (int i = 0; i < 18; i++) if (fscanf(f, "%f", &x[i]) != 1) return 2;
    for (int i = 0; i < 6; i++) if (fscanf(f, "%lf", &t[i]) != 1) return 2;
    float a[3] = {0}, j[3] = {0}; fbl_nn_pair(&fbl_nn_weights, &x[0], &x[3], &x[6], &x[9], a, j);
    for (int i = 0; i < 3; i++) {
      const double ra = fabs(a[i] - x[12 + i]) / (1e-5 + 1e-5 * fabs(x[12 + i])), rj = fabs(j[i] - x[15 + i]) / (1e-4 + 1e-5 * fabs(x[15 + i]));
      if (ra > wa) wa = ra;
      if (rj > wj) wj = rj;
    }
  }
  printf("exported sample weights (%d neurons) vs bundle, %d states: worst / tolerance acc %.3f, jerk %.3f\n", nh, n, wa, wj);
  printf((wa <= 1.0 && wj <= 1.0) ? "RESULT: ALL PASSED\n" : "RESULT: FAILED\n");
  return (wa <= 1.0 && wj <= 1.0) ? 0 : 1;
}
