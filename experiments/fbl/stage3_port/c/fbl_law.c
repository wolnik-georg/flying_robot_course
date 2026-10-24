#include "fbl_law.h"
#include <math.h>
#include <string.h>

void fbl_gains_default_bundle(fbl_gains_t *g)
{
  const float pk[3] = {56.0f, 56.0f, 45.0f}, vk[3] = {38.0f, 38.0f, 21.6f}, ik[3] = {48.0f, 48.0f, 3.0f}, rk[3] = {238.0f, 238.0f, 140.0f};
  const float J[3] = {1.43e-5f, 1.43e-5f, 2.89e-5f};
  for (int i = 0; i < 3; i++) { g->pos_kp[i] = pk[i]; g->vel_kp[i] = vk[i]; g->int_kp[i] = ik[i]; g->rate_k[i] = rk[i]; g->inertia[i] = J[i]; }
  g->mass = 0.03f; g->gravity = 9.81f; g->abg_beta = 0.4f; g->abg_gamma = 0.1f; g->w_max = 0.0f;
}

void fbl_law_reset(fbl_law_state_t *s) { memset(s, 0, sizeof(*s)); }

static void position_core(fbl_law_state_t *s, const fbl_gains_t *g, float dt_est, float dt_int, const float pos[3], const float vel[3], const fbl_ref_t *ref,
                          float acc_cmd[3], float jerk_cmd[3])
{
  /* alpha-beta-gamma acceleration estimate from the velocity (the bundle's estimator); a non-advancing interval skips the update */
  if (dt_est > 0.0f) {
    for (int i = 0; i < 3; i++) {
      const float vel_pred = s->vel_est[i] + dt_est * s->acc_est[i];
      const float residual = vel[i] - vel_pred;
      s->vel_est[i] = vel_pred + g->abg_beta * residual;
      s->acc_est[i] = s->acc_est[i] + (g->abg_gamma / dt_est) * residual;
    }
  }
  for (int i = 0; i < 3; i++) {
    const float e = pos[i] - ref->x[i];
    const float ev = vel[i] - ref->x_dot[i];
    s->int_err[i] += e * dt_int;
    acc_cmd[i] = ref->x_ddot[i] - g->int_kp[i] * s->int_err[i] - g->pos_kp[i] * e - g->vel_kp[i] * ev;
    jerk_cmd[i] = ref->x_dddot[i] - g->int_kp[i] * e - g->pos_kp[i] * ev - g->vel_kp[i] * (s->acc_est[i] - ref->x_ddot[i]);
  }
}

void fbl_law_position(fbl_law_state_t *s, const fbl_gains_t *g, float t, const float pos[3], const float vel[3], const fbl_ref_t *ref,
                      float acc_cmd[3], float jerk_cmd[3])
{
  float dt_est = 0.0f, dt_int = 0.0f;
  if (!s->init) {
    /* first call initialises the estimator (estimate 0); the integral clock starts here (the bundle integrates t - 0, which assumes t starts at 0) */
    for (int i = 0; i < 3; i++) { s->vel_est[i] = vel[i]; s->acc_est[i] = 0.0f; }
    s->prev_t = t; s->int_t = t; s->init = 1;
  } else {
    dt_est = t - s->prev_t;
    if (dt_est > 0.0f) { s->prev_t = t; } else { dt_est = 0.0f; }
    dt_int = t - s->int_t;
    if (dt_int < 0.0f) dt_int = 0.0f;
    s->int_t = t;
  }
  position_core(s, g, dt_est, dt_int, pos, vel, ref, acc_cmd, jerk_cmd);
}

void fbl_law_position_dt(fbl_law_state_t *s, const fbl_gains_t *g, float dt, const float pos[3], const float vel[3], const fbl_ref_t *ref,
                         float acc_cmd[3], float jerk_cmd[3])
{
  float d = dt;
  if (!s->init) {
    for (int i = 0; i < 3; i++) { s->vel_est[i] = vel[i]; s->acc_est[i] = 0.0f; }
    s->init = 1;
    d = 0.0f;
  }
  position_core(s, g, d, d, pos, vel, ref, acc_cmd, jerk_cmd);
}

void fbl_law_position_dt2(fbl_law_state_t *s, const fbl_gains_t *g, float dt_est, float dt_int, const float pos[3], const float vel[3], const fbl_ref_t *ref,
                          float acc_cmd[3], float jerk_cmd[3])
{
  /* estimator interval and integral interval given separately: the estimator can run at the measurement rate (dt_est = time since the last new
     velocity, 0 = no new sample: estimate held) while the integral runs at the controller rate. The first call only initialises (as _dt). */
  if (!s->init) {
    for (int i = 0; i < 3; i++) { s->vel_est[i] = vel[i]; s->acc_est[i] = 0.0f; }
    s->init = 1;
    dt_est = 0.0f; dt_int = 0.0f;
  }
  position_core(s, g, dt_est, dt_int, pos, vel, ref, acc_cmd, jerk_cmd);
}

void fbl_law_command(const float R[9], const float w[3], const float acc_cmd[3], const float jerk_cmd[3], float yaw, float yaw_rate,
                     const float res_acc[3], const float res_jerk[3], const float res_ang_acc[3], const fbl_gains_t *g,
                     float *thrust, float w_des[3], float torque[3])
{
  /* body axes in the world: columns of R */
  const float xb[3] = {R[0], R[3], R[6]}, yb[3] = {R[1], R[4], R[7]}, zb[3] = {R[2], R[5], R[8]};
  /* Eq. 15: thrust along the measured body z axis */
  float f[3];
  for (int i = 0; i < 3; i++) f[i] = acc_cmd[i] - res_acc[i];
  f[2] += g->gravity;
  const float th = g->mass * (f[0] * zb[0] + f[1] * zb[1] + f[2] * zb[2]);
  *thrust = th;
  /* corrected jerk in the body frame: R^T m (j_cmd - j_res); omega_xy = e3 x (that) / max(thrust, 0.01) */
  float jw[3];
  for (int i = 0; i < 3; i++) jw[i] = g->mass * (jerk_cmd[i] - res_jerk[i]);
  float jb[3];
  for (int i = 0; i < 3; i++) jb[i] = R[0 * 3 + i] * jw[0] + R[1 * 3 + i] * jw[1] + R[2 * 3 + i] * jw[2];
  const float inv = 1.0f / (th > 0.01f ? th : 0.01f);
  const float wx = -jb[1] * inv, wy = jb[0] * inv;               /* e3 x jb = (-jb_y, jb_x, 0) */
  /* Eq. 10: yaw component from d/dt (y_body . heading) = 0 */
  const float hx = cosf(yaw), hy = sinf(yaw);
  const float hdx = -yaw_rate * sinf(yaw), hdy = yaw_rate * cosf(yaw);
  const float denom = xb[0] * hx + xb[1] * hy;
  float wz = 0.0f;
  if (fabsf(denom) >= 1e-6f) {
    wz = (wx * (zb[0] * hx + zb[1] * hy) + (yb[0] * hdx + yb[1] * hdy)) / denom;
  }
  w_des[0] = wx; w_des[1] = wy; w_des[2] = wz;
  if (g->w_max > 0.0f) {                                           /* optional limit of the commanded rates (firmware); torque follows the limited rates */
    for (int i = 0; i < 3; i++) { if (w_des[i] > g->w_max) w_des[i] = g->w_max; else if (w_des[i] < -g->w_max) w_des[i] = -g->w_max; }
  }
  /* Eq. 19: torque = J (K (w_des - w) - residual angular acceleration), diagonal J and K */
  for (int i = 0; i < 3; i++) torque[i] = g->inertia[i] * (g->rate_k[i] * (w_des[i] - w[i]) - res_ang_acc[i]);
}
