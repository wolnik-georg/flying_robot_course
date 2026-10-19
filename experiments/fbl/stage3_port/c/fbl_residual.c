#include "fbl_residual.h"
#include "fbl_rom_table.h"
#include <math.h>
#include <string.h>

/* The host SIL build renames peer_get_all (-Dpeer_get_all=peer_get_all_fw_unused, for traj_iface.c). This module must reference the
   real name provided by the host layer (oot_host.c, injected peers) or by the firmware, exactly like ns2_residual.c. */
#undef peer_get_all
extern uint8_t peer_get_all(float *xs, float *ys, float *zs, uint32_t *ts, uint8_t max) __attribute__((weak));

void fbl_reset(fbl_state_t *s) { memset(s, 0, sizeof(*s)); }

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
