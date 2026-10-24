/* fbl_law_position_dt2 (ctrlFbl.abg_on_new = 1) against the authors' FBLControl._position_commands with the estimator driven at the measurement rate
   (gen_law_vectors_abg.py). Driven exactly as the firmware does: dt_int = tick (2 ms), dt_est = time since the last new velocity (0 when unchanged). */
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "fbl_law.h"
int main(void)
{
  FILE *f = fopen("law_vectors_abg.txt", "r");
  if (!f) { printf("law_vectors_abg.txt missing\n"); return 2; }
  int nseq, nstep; if (fscanf(f, "%d %d", &nseq, &nstep) != 2) return 2;
  fbl_gains_t g; fbl_gains_default_bundle(&g);
  double wa = 0, wj = 0, max_a = 0, max_j = 0;
  for (int s = 0; s < nseq; s++) {
    fbl_law_state_t st; fbl_law_reset(&st); float last_v[3] = {0}, age = 0.0f, prev_t = 0.0f;
    for (int i = 0; i < nstep; i++) {
      float t, pos[3], vel[3], e_acc[3], e_jerk[3]; fbl_ref_t ref;
      if (fscanf(f, "%f %f %f %f %f %f %f", &t, &pos[0], &pos[1], &pos[2], &vel[0], &vel[1], &vel[2]) != 7) return 2;
      for (int k = 0; k < 3; k++) if (fscanf(f, "%f", &ref.x[k]) != 1) return 2;
      for (int k = 0; k < 3; k++) if (fscanf(f, "%f", &ref.x_dot[k]) != 1) return 2;
      for (int k = 0; k < 3; k++) if (fscanf(f, "%f", &ref.x_ddot[k]) != 1) return 2;
      for (int k = 0; k < 3; k++) if (fscanf(f, "%f", &ref.x_dddot[k]) != 1) return 2;
      for (int k = 0; k < 3; k++) if (fscanf(f, "%f", &e_acc[k]) != 1) return 2;
      for (int k = 0; k < 3; k++) if (fscanf(f, "%f", &e_jerk[k]) != 1) return 2;
      const float dt = (i == 0) ? 0.0f : t - prev_t; prev_t = t;
      age += dt; float dt_est = 0.0f;
      if (i == 0 || memcmp(vel, last_v, sizeof(last_v)) != 0) { dt_est = age; age = 0.0f; memcpy(last_v, vel, sizeof(last_v)); }
      float acc[3], jerk[3]; fbl_law_position_dt2(&st, &g, dt_est, dt, pos, vel, &ref, acc, jerk);
      for (int k = 0; k < 3; k++) {
        const double ea = fabs(acc[k] - e_acc[k]), ej = fabs(jerk[k] - e_jerk[k]);
        if (ea > max_a) max_a = ea;
        if (ej > max_j) max_j = ej;
        const double ta = ea / (1e-3 + 2e-5 * fabs(e_acc[k])), tj = ej / (1e-2 + 2e-5 * fabs(e_jerk[k]));
        if (ta > wa) wa = ta;
        if (tj > wj) wj = tj;
      }
    }
  }
  fclose(f);
  printf("position law, estimator at the measurement rate: %d sequences x %d ticks (new velocity every 1-4 ticks)\n  acceleration command: max abs error %.2e m/s^2, worst / tolerance %.3f\n  jerk command: max abs error %.2e m/s^3, worst / tolerance %.3f\n", nseq, nstep, max_a, wa, max_j, wj);
  if (wa <= 1.0 && wj <= 1.0) { printf("ALL PASSED\n"); return 0; }
  printf("FAILED\n"); return 1;
}
