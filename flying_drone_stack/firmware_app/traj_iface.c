/**
 * traj_iface.c — Trajectory upload interface for onboard spline evaluation.
 *
 * Declares a coefficient buffer and CRTP parameters that allow the laptop to:
 *   1. Upload spline coefficients via the ci/cv/cw protocol (one float at a time).
 *   2. Set trajectory metadata (origin, hover height, number of segments).
 *   3. Switch between Mode B (laptop setpoint passthrough) and Mode D (onboard eval).
 *   4. Trigger the onboard trajectory by writing traj.start = 1.
 *
 * Buffer layout (per segment, 19 floats):
 *   [0]      = duration  (s)
 *   [1..9]   = cx[0..8] — degree-8 polynomial coefficients for x, normalised to t∈[0,1]
 *   [10..18] = cy[0..8] — degree-8 polynomial coefficients for y, normalised to t∈[0,1]
 *
 * Max 12 segments → 228 floats total (960 bytes).  Easily fits in SRAM.
 *
 * Coefficient upload protocol (laptop side):
 *   for idx in 0..n_coefs:
 *     set traj.ci = idx          (u8, 0..227)
 *     set traj.cv = coefs[idx]   (f32)
 *     set traj.cw = 1            (commit trigger)
 *     poll traj.cw until == 0   (firmware clears after writing)
 *
 * The commit flag is cleared by the Rust controller task (500 Hz) on the next
 * tick after the write — safely within the ~5 ms radio round-trip.
 *
 * Mode switch:
 *   traj.mode = 0  → Mode B passthrough (incoming CRTP setpoint used as-is)
 *   traj.mode = 1  → Mode D onboard eval (setpoint ignored; spline drives controller)
 *
 * Start trigger:
 *   traj.start = 1  → firmware latches current tick as T0; stays 1 until mode=0 resets it.
 *
 * These symbols are declared extern in firmware_app/src/lib.rs and accessed there.
 */

#include "param.h"
#include <stdint.h>
#include <stdbool.h>

/* ── Coefficient buffer ──────────────────────────────────────────────────── */
#define TRAJ_MAX_SEGS        12u
#define TRAJ_FLOATS_PER_SEG  19u

float   g_traj_coefs[TRAJ_MAX_SEGS * TRAJ_FLOATS_PER_SEG]; /* zero-initialised by C runtime */

/* ── Trajectory metadata ─────────────────────────────────────────────────── */
uint8_t g_traj_n_segs    = 0;     /* number of valid segments in g_traj_coefs  */
uint8_t g_traj_mode      = 0;     /* 0 = passthrough (Mode B), 1 = onboard eval (Mode D) */
uint8_t g_traj_start     = 0;     /* laptop writes 1 to start; firmware latches T0        */
float   g_traj_origin_x  = 0.0f;  /* EKF x offset sampled before takeoff [m]             */
float   g_traj_origin_y  = 0.0f;  /* EKF y offset sampled before takeoff [m]             */
float   g_traj_hover_z   = 1.0f;  /* constant flight altitude [m]                        */
float   g_traj_dz        = 0.0f;  /* Z gain per lap [m]; 0=flat, +0.40=ascending helix   */

/* ── Upload protocol ─────────────────────────────────────────────────────── */
uint8_t g_traj_coef_ci   = 0;     /* coefficient index (0..227)                          */
float   g_traj_coef_cv   = 0.0f;  /* coefficient value to write at g_traj_coef_ci        */
uint8_t g_traj_coef_cw   = 0;     /* commit flag: laptop sets 1; Rust clears to 0        */

/* ── CRTP parameter group ────────────────────────────────────────────────── */
PARAM_GROUP_START(traj)
  PARAM_ADD(PARAM_UINT8, mode,  &g_traj_mode)
  PARAM_ADD(PARAM_UINT8, start, &g_traj_start)
  PARAM_ADD(PARAM_UINT8, nseg,  &g_traj_n_segs)
  PARAM_ADD(PARAM_FLOAT, ox,    &g_traj_origin_x)
  PARAM_ADD(PARAM_FLOAT, oy,    &g_traj_origin_y)
  PARAM_ADD(PARAM_FLOAT, hz,    &g_traj_hover_z)
  PARAM_ADD(PARAM_FLOAT, dz,    &g_traj_dz)
  PARAM_ADD(PARAM_UINT8, ci,    &g_traj_coef_ci)
  PARAM_ADD(PARAM_FLOAT, cv,    &g_traj_coef_cv)
  PARAM_ADD(PARAM_UINT8, cw,    &g_traj_coef_cw)
PARAM_GROUP_STOP(traj)
