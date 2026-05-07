/**
 * traj_iface.c — Trajectory upload interface for onboard spline evaluation.
 *
 * Declares coefficient buffers and CRTP parameters that allow the laptop to:
 *   1. Upload position coefficients via ci/cv/cw (one float at a time).
 *   2. Upload Z-axis coefficients via zci/zcv/zcw (3D trajectories only).
 *   3. Upload attitude coefficients via aci/acv/acw (planning Mode 2 only).
 *   4. Set trajectory metadata (origin, hover height, number of segments).
 *   5. Switch between Mode B (passthrough) and Mode D (onboard eval).
 *   6. Trigger the onboard trajectory by writing traj.start = 1.
 *
 * Position buffer layout (per segment, 19 floats):
 *   [0]      = duration  (s)
 *   [1..9]   = cx[0..8] — degree-8 polynomial for x, normalised to t∈[0,1]
 *   [10..18] = cy[0..8] — degree-8 polynomial for y, normalised to t∈[0,1]
 * Max 12 segments → 228 floats (ci index 0..227).
 *
 * Z-axis buffer layout (per segment, 9 floats):
 *   [0..8]   = cz[0..8] — degree-8 polynomial for z offset [m], normalised
 * Max 12 segments → 108 floats (zci index 0..107).
 * z_mode = 0: z = traj.hz + traj.dz * lap_frac (constant + linear ramp, default).
 * z_mode = 1: z = traj.hz + poly_eval(cz) — full 3D trajectory (loop, etc.).
 *
 * Attitude buffer layout (per segment, 18 floats, planning Mode 2 only):
 *   [0..8]   = croll[0..8]  — degree-8 polynomial for roll  [rad], normalised
 *   [9..17]  = cpitch[0..8] — degree-8 polynomial for pitch [rad], normalised
 * Max 12 segments → 216 floats (aci index 0..215).
 * att_mode = 0: firmware derives attitude from flatness (default — Mode 0/1 behaviour).
 * att_mode = 1: firmware evaluates uploaded roll/pitch polynomials at 500 Hz.
 *
 * Upload protocol (same for pos, z, and att — different param names):
 *   set traj.ci/zci/aci = idx   (u8)
 *   set traj.cv/zcv/acv = value (f32)
 *   set traj.cw/zcw/acw = 1     (commit trigger; firmware clears to 0 within 2 ms)
 *
 * Mode switch:
 *   traj.mode = 0  → Mode B passthrough (laptop sends full-state setpoints)
 *   traj.mode = 1  → Mode D onboard eval (all trajectories: flat, helix, loop)
 *                    z_mode selects Z source: 0=ramp (flat/helix), 1=polynomial (loop)
 *
 * These symbols are declared extern in firmware_app/src/lib.rs and accessed there.
 */

#include "param.h"
#include <stdint.h>
#include <stdbool.h>

/* ── Position coefficient buffer ─────────────────────────────────────────── */
#define TRAJ_MAX_SEGS        12u
#define TRAJ_FLOATS_PER_SEG  19u   /* duration + cx[9] + cy[9] */

float   g_traj_coefs[TRAJ_MAX_SEGS * TRAJ_FLOATS_PER_SEG]; /* zero-initialised by C runtime */

/* ── Z-axis coefficient buffer (3D trajectories: loop, etc.) ─────────────── */
/* Layout per segment: [cz[9]] — 9 floats, no duration field.                  */
/* z_mode=0: z = hover_z + dz*lap_frac (linear ramp, circle/figure8/helix).   */
/* z_mode=1: z = hover_z + poly_eval(cz) — full 3D trajectory.                */
#define TRAJ_Z_FLOATS_PER_SEG    9u   /* cz[9] */

float   g_traj_z_coefs[TRAJ_MAX_SEGS * TRAJ_Z_FLOATS_PER_SEG]; /* zero-initialised */
uint8_t g_traj_z_mode    = 0;    /* 0 = ramp (default), 1 = polynomial          */

/* ── Z upload protocol ───────────────────────────────────────────────────── */
uint8_t g_traj_z_ci      = 0;     /* z coefficient index (0..107)               */
float   g_traj_z_cv      = 0.0f;  /* z coefficient value to write               */
uint8_t g_traj_z_cw      = 0;     /* commit flag: laptop sets 1; Rust clears    */

/* ── Attitude coefficient buffer (planning Mode 2 only) ─────────────────── */
/* Layout per segment: [croll[9], cpitch[9]] — 18 floats, no duration field.  */
/* att_mode=0: firmware uses flatness (unchanged Mode 0/1 behaviour).          */
/* att_mode=1: firmware evaluates these polynomials at 500 Hz.                 */
#define TRAJ_ATT_FLOATS_PER_SEG  18u  /* croll[9] + cpitch[9] */

float   g_traj_att_coefs[TRAJ_MAX_SEGS * TRAJ_ATT_FLOATS_PER_SEG]; /* zero-initialised */
uint8_t g_traj_att_mode  = 0;    /* 0 = derive from flatness (default), 1 = polynomial */

/* att_ctrl_mode controls HOW the polynomial is blended with feedback (when att_mode=1):
 *   0 = hard override: polynomial sets rd AND omega_d (both replaced by polynomial).
 *       Required for loop/flip where flatness is undefined at zero-thrust events.
 *   1 = hybrid: polynomial provides omega_d feedforward ONLY; rd is still derived from
 *       the total feedback force via flatness — position tracking errors are always
 *       reflected in the rotation command. Recommended for normal flat trajectories.
 */
uint8_t g_traj_att_ctrl_mode = 1; /* 1 = hybrid (default — stable for all flat trajectories) */

/* ── Attitude upload protocol ────────────────────────────────────────────── */
uint8_t g_traj_att_ci    = 0;     /* attitude coefficient index (0..215)                 */
float   g_traj_att_cv    = 0.0f;  /* attitude coefficient value to write at g_traj_att_ci */
uint8_t g_traj_att_cw    = 0;     /* commit flag: laptop sets 1; Rust clears to 0        */

/* ── Trajectory metadata ─────────────────────────────────────────────────── */
uint8_t g_traj_n_segs    = 0;     /* number of valid segments in g_traj_coefs  */
uint8_t g_traj_mode      = 0;     /* 0 = passthrough (Mode B), 1 = onboard eval (Mode D) */
uint8_t g_traj_start     = 0;     /* laptop writes 1 to start; firmware latches T0        */
float   g_traj_origin_x  = 0.0f;  /* EKF x offset sampled before takeoff [m]             */
float   g_traj_origin_y  = 0.0f;  /* EKF y offset sampled before takeoff [m]             */
float   g_traj_hover_z   = 1.0f;  /* constant flight altitude [m]                        */
float   g_traj_dz        = 0.0f;  /* Z gain per lap [m]; 0=flat, +0.40=ascending helix   */

/* ── Position upload protocol ────────────────────────────────────────────── */
uint8_t g_traj_coef_ci   = 0;     /* coefficient index (0..227)                          */
float   g_traj_coef_cv   = 0.0f;  /* coefficient value to write at g_traj_coef_ci        */
uint8_t g_traj_coef_cw   = 0;     /* commit flag: laptop sets 1; Rust clears to 0        */

/* ── CRTP parameter group ────────────────────────────────────────────────── */
PARAM_GROUP_START(traj)
  PARAM_ADD(PARAM_UINT8, mode,     &g_traj_mode)
  PARAM_ADD(PARAM_UINT8, start,    &g_traj_start)
  PARAM_ADD(PARAM_UINT8, nseg,     &g_traj_n_segs)
  PARAM_ADD(PARAM_FLOAT, ox,       &g_traj_origin_x)
  PARAM_ADD(PARAM_FLOAT, oy,       &g_traj_origin_y)
  PARAM_ADD(PARAM_FLOAT, hz,       &g_traj_hover_z)
  PARAM_ADD(PARAM_FLOAT, dz,       &g_traj_dz)
  PARAM_ADD(PARAM_UINT8, ci,       &g_traj_coef_ci)
  PARAM_ADD(PARAM_FLOAT, cv,       &g_traj_coef_cv)
  PARAM_ADD(PARAM_UINT8, cw,       &g_traj_coef_cw)
  PARAM_ADD(PARAM_UINT8, z_mode,   &g_traj_z_mode)
  PARAM_ADD(PARAM_UINT8, zci,      &g_traj_z_ci)
  PARAM_ADD(PARAM_FLOAT, zcv,      &g_traj_z_cv)
  PARAM_ADD(PARAM_UINT8, zcw,      &g_traj_z_cw)
  PARAM_ADD(PARAM_UINT8, att_mode,      &g_traj_att_mode)
  PARAM_ADD(PARAM_UINT8, att_ctrl_mode, &g_traj_att_ctrl_mode)
  PARAM_ADD(PARAM_UINT8, aci,           &g_traj_att_ci)
  PARAM_ADD(PARAM_FLOAT, acv,           &g_traj_att_cv)
  PARAM_ADD(PARAM_UINT8, acw,           &g_traj_att_cw)
PARAM_GROUP_STOP(traj)
