/*
 * Learned-FBL control law (paper Eqs. 15, 10, 19, 17-18), C implementation for the Omar C controller tree. STAGE 3 PREPARATION: lives under
 * experiments/fbl/stage3_port/c/, NOT in the firmware tree. Port of the authors' `compute_fbl_command` and of `FBLControl._position_commands`
 * (position PID with integral, alpha-beta-gamma acceleration estimate, acceleration and jerk commands). Specification: docs/fbl/PORT_SPEC.md.
 *
 * Frames: R maps body vectors to world (row-major, R[3*i + j]); linear quantities in the world frame; body rates and torques in the body frame.
 * The thrust is projected on the MEASURED body z axis (Sec. 3.3, footnote 2). The 0.01 N floor protects only the rate inversion.
 * Residual inputs are the outputs of fbl_residual.c (acceleration and jerk of the downwash, force only: the angular acceleration residual is passed as zero).
 */
#ifndef FBL_LAW_H
#define FBL_LAW_H

#include <stdint.h>

typedef struct {
  float pos_kp[3];   /* position gains (acceleration per m)        bundle: 56, 56, 45 */
  float vel_kp[3];   /* velocity gains                              38, 38, 21.6 */
  float int_kp[3];   /* integral gains                              48, 48, 3 */
  float rate_k[3];   /* body-rate gains (acceleration gains, torque gain = J K)  238, 238, 140 */
  float inertia[3];  /* diagonal inertia kg m^2 */
  float mass;        /* kg */
  float gravity;     /* m/s^2 */
  float abg_beta;    /* alpha-beta-gamma: velocity correction gain   0.4 */
  float abg_gamma;   /* acceleration correction gain                 0.1 */
} fbl_gains_t;

typedef struct {
  float int_err[3];  /* integral of the position error */
  float vel_est[3];  /* estimator velocity */
  float acc_est[3];  /* estimator acceleration */
  float prev_t;      /* estimator: time of the previous accepted update [s] */
  float int_t;       /* integral: time of the previous call [s] */
  uint8_t init;      /* 0 until the first update */
} fbl_law_state_t;

/* flat output / reference: position, velocity, acceleration, jerk (world frame) */
typedef struct { float x[3], x_dot[3], x_ddot[3], x_dddot[3]; } fbl_ref_t;

void fbl_gains_default_bundle(fbl_gains_t *g);          /* the bundle's gains with mass and inertia of the 30 g simulation vehicle (change them for ours) */
void fbl_law_reset(fbl_law_state_t *s);

/* Eqs. 17-18: acceleration and jerk command from the state, the reference and the acceleration estimate (updates the state). t in seconds. */
void fbl_law_position(fbl_law_state_t *s, const fbl_gains_t *g, float t, const float pos[3], const float vel[3], const fbl_ref_t *ref,
                      float acc_cmd[3], float jerk_cmd[3]);

/* Eqs. 15, 10, 19: thrust [N], desired body rates [rad/s], body torque [N m]. Stateless. */
void fbl_law_command(const float R[9], const float w[3], const float acc_cmd[3], const float jerk_cmd[3], float yaw, float yaw_rate,
                     const float res_acc[3], const float res_jerk[3], const float res_ang_acc[3], const fbl_gains_t *g,
                     float *thrust, float w_des[3], float torque[3]);

#endif
