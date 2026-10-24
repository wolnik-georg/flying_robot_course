#include "fbl_residual.h"
#include "fbl_rom_table.h"
#include <math.h>
#include <string.h>

/* The host SIL build renames peer_get_all (-Dpeer_get_all=peer_get_all_fw_unused, for traj_iface.c). This module must reference the
   real name provided by the host layer (oot_host.c, injected peers) or by the firmware, exactly like ns2_residual.c. */
#undef peer_get_all
extern uint8_t peer_get_all(float *xs, float *ys, float *zs, uint32_t *ts, uint8_t max) __attribute__((weak));

void fbl_reset(fbl_state_t *s) { memset(s, 0, sizeof(*s)); }

static const fbl_nn_t *s_nn = 0;
void fbl_residual_set_nn(const fbl_nn_t *nn) { s_nn = (nn != 0 && nn->nh > 0 && nn->nh <= FBL_NN_HMAX) ? nn : 0; }

void fbl_nn_pair(const fbl_nn_t *nn, const float p[3], const float v[3], const float p_top[3], const float v_top[3], float acc[3], float jerk[3])
{
  /* geometry with the upper vehicle upright (flow axis +world z, zero derivative), d = ego - top */
  const float d[3] = {p[0] - p_top[0], p[1] - p_top[1], p[2] - p_top[2]};
  const float dd[3] = {v[0] - v_top[0], v[1] - v_top[1], v[2] - v_top[2]};
  const float z0 = fabsf(d[2]);
  const float dz0 = (d[2] > 0.0f ? 1.0f : (d[2] < 0.0f ? -1.0f : 0.0f)) * dd[2];
  const float r0 = sqrtf(d[0] * d[0] + d[1] * d[1]);
  const float dr0 = (d[0] * dd[0] + d[1] * dd[1]) / (r0 > 1e-12f ? r0 : 1e-12f);
  /* network value and directional derivative */
  float f[3] = {nn->b2[0], nn->b2[1], nn->b2[2]}, df[3] = {0.0f, 0.0f, 0.0f};
  for (int k = 0; k < nn->nh; k++) {
    const float hk = tanhf(nn->w1[k][0] * r0 + nn->w1[k][1] * z0 + nn->b1[k]);
    const float gk = (1.0f - hk * hk) * (nn->w1[k][0] * dr0 + nn->w1[k][1] * dz0);
    for (int i = 0; i < 3; i++) { f[i] += nn->w2[i][k] * hk; df[i] += nn->w2[i][k] * gk; }
  }
  /* flow frame [x_n y_n z_n] and its time derivative (Gram-Schmidt as in the bundle, r_hat = top - ego) */
  const float rh[3] = {-d[0], -d[1], -d[2]}, rhd[3] = {-dd[0], -dd[1], -dd[2]};
  const float nrm = sqrtf(rh[0] * rh[0] + rh[1] * rh[1] + rh[2] * rh[2]) + 1e-8f;
  float x0[3], dx0[3];
  for (int i = 0; i < 3; i++) x0[i] = rh[i] / nrm;
  const float x0p = x0[0] * rhd[0] + x0[1] * rhd[1] + x0[2] * rhd[2];
  for (int i = 0; i < 3; i++) dx0[i] = (rhd[i] - x0[i] * x0p) / nrm;
  const float y0[3] = {-x0[1], x0[0], 0.0f}, dy0[3] = {-dx0[1], dx0[0], 0.0f};          /* z x x0, z = (0, 0, 1) */
  float yn0 = sqrtf(y0[0] * y0[0] + y0[1] * y0[1]); if (yn0 < 1e-8f) yn0 = 1e-8f;
  const float yn[3] = {y0[0] / yn0, y0[1] / yn0, 0.0f};
  const float ynp = yn[0] * dy0[0] + yn[1] * dy0[1];
  const float dyn[3] = {(dy0[0] - yn[0] * ynp) / yn0, (dy0[1] - yn[1] * ynp) / yn0, 0.0f};
  const float x1[3] = {yn[1], -yn[0], 0.0f}, dx1[3] = {dyn[1], -dyn[0], 0.0f};          /* y_n x z */
  float xn0 = sqrtf(x1[0] * x1[0] + x1[1] * x1[1]); if (xn0 < 1e-8f) xn0 = 1e-8f;
  const float xn[3] = {x1[0] / xn0, x1[1] / xn0, 0.0f};
  const float xnp = xn[0] * dx1[0] + xn[1] * dx1[1];
  const float dxn[3] = {(dx1[0] - xn[0] * xnp) / xn0, (dx1[1] - xn[1] * xnp) / xn0, 0.0f};
  /* acc = R f, jerk = dR f + R df (dz_n = 0) */
  for (int i = 0; i < 3; i++) {
    const float zi = (i == 2) ? 1.0f : 0.0f;
    acc[i] += xn[i] * f[0] + yn[i] * f[1] + zi * f[2];
    jerk[i] += dxn[i] * f[0] + dyn[i] * f[1] + xn[i] * df[0] + yn[i] * df[1] + zi * df[2];
  }
}

static float fbl_fade(float z0)
{
  float f = (FBL_Z_FADE1 - z0) / (FBL_Z_FADE1 - FBL_Z_FADE0);
  return f < 0.0f ? 0.0f : (f > 1.0f ? 1.0f : f);
}

/* r grid node j: r = RMAX * (j / (NR - 1))^2 (computed in double, stored as float like the generator does) */
static float r_node(int j)
{
  const double u = (double)j / (double)(FBL_ROM_NR - 1);
  return (float)((double)FBL_ROM_RMAX * u * u);
}

/* bilinear lookup of a1, da1/dz0, da1/dr0 at (z0, r0); z0 in [ZMIN, ZMAX], r0 in [0, RMAX] */
static void lookup(float z0, float r0, float out[3])
{
  const float zspan = FBL_ROM_ZMAX - FBL_ROM_ZMIN;
  const float fz = (z0 - FBL_ROM_ZMIN) / zspan * (float)(FBL_ROM_NZ - 1);
  int i = (int)floorf(fz);
  if (i < 0) i = 0;
  if (i > FBL_ROM_NZ - 2) i = FBL_ROM_NZ - 2;
  const float sz = fz - (float)i;
  const float rc = r0 < FBL_ROM_RMAX ? r0 : FBL_ROM_RMAX;
  const float u = sqrtf(rc / FBL_ROM_RMAX) * (float)(FBL_ROM_NR - 1);
  int j = (int)floorf(u);
  if (j < 0) j = 0;
  if (j > FBL_ROM_NR - 2) j = FBL_ROM_NR - 2;
  const float rj = r_node(j), rj1 = r_node(j + 1);
  const float sr = (r0 - rj) / (rj1 - rj);
  const float *a = &fbl_rom_table[((i) * FBL_ROM_NR + j) * 3];
  const float *b = &fbl_rom_table[((i + 1) * FBL_ROM_NR + j) * 3];
  const float *c = &fbl_rom_table[((i) * FBL_ROM_NR + j + 1) * 3];
  const float *d = &fbl_rom_table[((i + 1) * FBL_ROM_NR + j + 1) * 3];
  for (int k = 0; k < 3; k++) {
    out[k] = (1.0f - sz) * (1.0f - sr) * a[k] + sz * (1.0f - sr) * b[k] + (1.0f - sz) * sr * c[k] + sz * sr * d[k];
  }
}

int fbl_residual_pair(const float p[3], const float v[3], const float p_top[3], const float v_top[3], float c_d, float acc[3], float jerk[3])
{
  const float dx = p[0] - p_top[0], dy = p[1] - p_top[1], dz = p[2] - p_top[2];
  const float z0 = -dz;                                   /* upper vehicle above the ego vehicle -> positive */
  if (!(FBL_Z_LO < z0 && z0 < FBL_Z_FADE1)) return 0;
  const float r0 = sqrtf(dx * dx + dy * dy);
  if (r0 >= FBL_R_MAX_USE) return 0;
  float t[3];
  lookup(z0, r0, t);
  const float ddx = v[0] - v_top[0], ddy = v[1] - v_top[1], ddz = v[2] - v_top[2];
  const float dz0 = -ddz;                                 /* d(z0)/dt */
  const float rr = r0 > 1e-6f ? r0 : 1e-6f;
  const float dr0 = (dx * ddx + dy * ddy) / rr;           /* d(r0)/dt */
  const float w = fbl_fade(z0);
  acc[2] += w * c_d * t[0];
  jerk[2] += w * c_d * (t[1] * dz0 + t[2] * dr0);
  if (s_nn) {                                             /* network term inside the same gate and fade as the ROM (ours; the bundle has no gate) */
    float a[3] = {0.0f, 0.0f, 0.0f}, j[3] = {0.0f, 0.0f, 0.0f};
    fbl_nn_pair(s_nn, p, v, p_top, v_top, a, j);
    for (int i = 0; i < 3; i++) { acc[i] += w * a[i]; jerk[i] += w * j[i]; }
  }
  return 1;
}

void fbl_residual_eval_abs(const float p[3], const float v[3], const float peer_p[][3], const float peer_v[][3], int n, float c_d, fbl_res_t *out)
{
  memset(out, 0, sizeof(*out));
  if (n > FBL_MAX_NEIGHBOURS) n = FBL_MAX_NEIGHBOURS;
  for (int k = 0; k < n; k++) {
    out->n_active += (uint8_t)fbl_residual_pair(p, v, peer_p[k], peer_v[k], c_d, out->acc, out->jerk);
  }
}

void fbl_residual_eval(fbl_state_t *s, const float pos[3], const float vel[3], float c_d, fbl_res_t *out)
{
  float xs[FBL_MAX_NEIGHBOURS] = {0}, ys[FBL_MAX_NEIGHBOURS] = {0}, zs[FBL_MAX_NEIGHBOURS] = {0};
  uint32_t ts[FBL_MAX_NEIGHBOURS] = {0};
  float pp[FBL_MAX_NEIGHBOURS][3] = {{0}}, pv[FBL_MAX_NEIGHBOURS][3] = {{0}};
  int n = 0;
  if (peer_get_all != 0) n = (int)peer_get_all(xs, ys, zs, ts, FBL_MAX_NEIGHBOURS);
  if (n > FBL_MAX_NEIGHBOURS) n = FBL_MAX_NEIGHBOURS;
  for (int k = 0; k < n; k++) {
    const uint32_t pt = s->prev[k].ts, ts_new = ts[k], dt_ms = ts_new - pt;
    if (pt == 0) {
      s->prev[k].x = xs[k]; s->prev[k].y = ys[k]; s->prev[k].z = zs[k]; s->prev[k].ts = ts_new;
      s->vel[k][0] = s->vel[k][1] = s->vel[k][2] = 0.0f;
    } else if (ts_new == pt) {
      /* unchanged timestamp: keep the last estimate */
    } else if (dt_ms > 0 && dt_ms < 500) {
      const float inv = 1000.0f / (float)dt_ms;
      s->vel[k][0] = (xs[k] - s->prev[k].x) * inv; s->vel[k][1] = (ys[k] - s->prev[k].y) * inv; s->vel[k][2] = (zs[k] - s->prev[k].z) * inv;
      s->prev[k].x = xs[k]; s->prev[k].y = ys[k]; s->prev[k].z = zs[k]; s->prev[k].ts = ts_new;
    } else {
      s->prev[k].x = xs[k]; s->prev[k].y = ys[k]; s->prev[k].z = zs[k]; s->prev[k].ts = ts_new;
      s->vel[k][0] = s->vel[k][1] = s->vel[k][2] = 0.0f;
    }
    pp[k][0] = xs[k]; pp[k][1] = ys[k]; pp[k][2] = zs[k];
    pv[k][0] = s->vel[k][0]; pv[k][1] = s->vel[k][1]; pv[k][2] = s->vel[k][2];
  }
  fbl_residual_eval_abs(pos, vel, (const float (*)[3])pp, (const float (*)[3])pv, n, c_d, out);
}
