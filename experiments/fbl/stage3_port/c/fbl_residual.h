/*
 * Learned-FBL downwash residual (ROM only), C implementation for the Omar C controller tree. STAGE 3 PREPARATION: lives under
 * experiments/fbl/stage3_port/c/, NOT in the firmware tree (nothing here is flashed or flown). Specification: docs/fbl/PORT_SPEC.md.
 * Reference implementation (Python, float32, same steps): experiments/fbl/stage3_port/fbl_rom_runtime.py.
 *
 * Output = world-frame acceleration and jerk that an upper vehicle causes on the ego vehicle (negative z = pushed down): force purely along +z,
 * acc_z = w * C_D * a1(z0, r0), jerk_z = w * C_D * (da1/dz0 * dz0dot + da1/dr0 * dr0dot), with the per-unit-C_D table a1 generated from the authors'
 * analytic ROM on the Crazyflie 2.1 Brushless constants (fbl_rom_table.c). The upper vehicle is treated as upright (flow axis = world z).
 * Gate: neighbour above (z0 = z_top - z_ego), Z_LO < z0 < Z_FADE1, r0 < R_MAX_USE; linear fade to zero between Z_FADE0 and Z_FADE1.
 * No dynamic memory, no globals, only sqrtf / floorf.
 */
#ifndef FBL_RESIDUAL_H
#define FBL_RESIDUAL_H

#include <stdint.h>

#define FBL_MAX_NEIGHBOURS 3
#define FBL_Z_LO 0.05f
#define FBL_Z_FADE0 1.2f
#define FBL_Z_FADE1 1.4f
#define FBL_R_MAX_USE 0.6f

typedef struct {
  float acc[3];   /* world frame, m/s^2 */
  float jerk[3];  /* world frame, m/s^3 */
  uint8_t n_active; /* neighbours that contributed */
} fbl_res_t;

typedef struct {
  float x, y, z;
  uint32_t ts;
} fbl_peer_prev_t;

typedef struct {
  fbl_peer_prev_t prev[FBL_MAX_NEIGHBOURS]; /* previous peer sample (position, timestamp ms) */
  float vel[FBL_MAX_NEIGHBOURS][3];         /* last peer velocity estimate */
} fbl_state_t;

void fbl_reset(fbl_state_t *s);

/* One neighbour, explicit states (world frame). Adds nothing to out when the gate is closed. Returns 1 when the neighbour contributed. */
int fbl_residual_pair(const float p[3], const float v[3], const float p_top[3], const float v_top[3], float c_d, float acc[3], float jerk[3]);

/* n <= FBL_MAX_NEIGHBOURS neighbours given as absolute states; sums the pairwise residuals (as the bundle does). */
void fbl_residual_eval_abs(const float p[3], const float v[3], const float peer_p[][3], const float peer_v[][3], int n, float c_d, fbl_res_t *out);

/* Peers from peer_get_all() (weak symbol, same API as for NS2): per-peer velocity by differencing with the NS2 rules (first sample -> zero velocity,
 * unchanged timestamp -> last estimate, gap 0 < dt < 500 ms -> difference, otherwise resync with zero velocity). */
void fbl_residual_eval(fbl_state_t *s, const float pos[3], const float vel[3], float c_d, fbl_res_t *out);

#endif
