/* Host-side glue so the out-of-tree controller can run in the simulator.
 *
 * traj_iface.c is compiled unchanged (with the param.h/log.h stubs beside this file)
 * so the INDI gains, trajectory globals and their tuned defaults come from exactly the
 * same source as the firmware. Only two things must differ on the host:
 *
 *   1. RPM feedback. On the drone rpm_get_all() reads the RPM deck or DShot telemetry
 *      through the log system. Here the simulator knows the true motor speeds, so it
 *      pushes them in with oot_set_rpm() and rpm_get_all() returns those. This matters:
 *      attitude INDI derives tau_current from RPM^2, and without it the controller
 *      silently falls back to tau_prev -- i.e. degraded INDI, not the real thing.
 *      traj_iface.c's own rpm_get_all is renamed away by -Drpm_get_all=... at compile
 *      time, so there is no duplicate symbol.
 *
 *   2. Log sinks. The indi_*_write() functions are the firmware's log bridge. Here they
 *      just latch the latest values so the simulator can read the residual, the INDI
 *      torque and the filtered angular acceleration -- the same signals the uSD deck
 *      records in flight.
 */
/* setup.py renames traj_iface.c's versions of these out of the way with -D so the
 * host ones below win. That -D applies to every source in the build, including this
 * file, so undo it here -- otherwise these definitions would be renamed too and the
 * firmware's originals would collide with them. */
#undef rpm_get_all
#undef indi_log_write
#undef indi_tau_write
#undef indi_notch_log_write
#undef indi_a_res_write

#include <stdint.h>
#include <stdbool.h>
#include "log.h"
#include "platform_defaults.h"

/* ── symbols the firmware normally provides, stubbed for the host ──────────────
 * traj_iface.c includes the real firmware log.h (it comes first on the include
 * path), so it links against the real log API. log.c is not part of the bindings
 * build, so provide the two functions it actually calls. Returning "not found"
 * means traj_iface.c's own rpm reader would report zeros -- which is moot, because
 * that function is renamed away and oot_host.c's version is used instead.
 *
 * rust_eh_personality is the unwinding hook a no_std Rust staticlib still
 * references even when built with panic=abort. Nothing here can unwind, so an
 * empty definition is enough to satisfy the linker. */
logVarId_t logGetVarId(const char *group, const char *name)
{
    (void)group; (void)name;
    return 0xffffu;               /* always "not present" */
}

uint32_t logGetUint(logVarId_t varid) { (void)varid; return 0u; }
float    logGetFloat(logVarId_t varid) { (void)varid; return 0.0f; }

void rust_eh_personality(void) {}

static uint16_t g_host_rpm[4] = {0, 0, 0, 0};

void oot_set_rpm(uint16_t m1, uint16_t m2, uint16_t m3, uint16_t m4)
{
    g_host_rpm[0] = m1; g_host_rpm[1] = m2; g_host_rpm[2] = m3; g_host_rpm[3] = m4;
}

void rpm_get_all(uint16_t *m1, uint16_t *m2, uint16_t *m3, uint16_t *m4)
{
    *m1 = g_host_rpm[0]; *m2 = g_host_rpm[1];
    *m3 = g_host_rpm[2]; *m4 = g_host_rpm[3];
}

/* ── log sinks: latch the values the controller publishes ─────────────────── */
static float l_alp_raw[3], l_alp[3], l_tau[3], l_alp_notch[3], l_a_res[3];

void indi_log_write(float arx, float ary, float arz, float ax, float ay, float az)
{
    l_alp_raw[0]=arx; l_alp_raw[1]=ary; l_alp_raw[2]=arz;
    l_alp[0]=ax; l_alp[1]=ay; l_alp[2]=az;
}
void indi_tau_write(float tx, float ty, float tz)      { l_tau[0]=tx; l_tau[1]=ty; l_tau[2]=tz; }
void indi_notch_log_write(float x, float y, float z)   { l_alp_notch[0]=x; l_alp_notch[1]=y; l_alp_notch[2]=z; }
void indi_a_res_write(float x, float y, float z)       { l_a_res[0]=x; l_a_res[1]=y; l_a_res[2]=z; }

/* Readers for Python. a_res is f_res/m -- the residual the thesis measures. */
float oot_get_a_res(int i)     { return (i>=0 && i<3) ? l_a_res[i]     : 0.0f; }
float oot_get_tau(int i)       { return (i>=0 && i<3) ? l_tau[i]       : 0.0f; }
float oot_get_alp(int i)       { return (i>=0 && i<3) ? l_alp[i]       : 0.0f; }
float oot_get_alp_raw(int i)   { return (i>=0 && i<3) ? l_alp_raw[i]   : 0.0f; }
float oot_get_alp_notch(int i) { return (i>=0 && i<3) ? l_alp_notch[i] : 0.0f; }

/* Airframe constants the firmware was COMPILED with. The simulator needs these to
   invert powerDistribution exactly: it turns force into PWM as
   pwm = force / THRUST_MAX * UINT16_MAX, so the plant must turn PWM back into force
   the same way, or the drone silently delivers a different thrust than was commanded
   and every controller shows the same steady-state droop. Exposing them here rather
   than duplicating the numbers in Python keeps one source of truth: change the
   platform define and the simulated airframe follows. */
float oot_thrust_max(void)     { return THRUST_MAX; }
float oot_arm_length(void)     { return ARM_LENGTH; }
float oot_thrust2torque(void)  { return THRUST2TORQUE; }

/* ---- Per-vehicle controller state, host simulator only --------------------
   The simulator drives every simulated drone through this one compiled controller,
   so without this they would share one set of INDI filters, integrators and last_tick.
   oot_select_drone(i) parks the current vehicle's state and loads vehicle i's, so each
   one behaves as if it had the controller to itself -- which on real hardware it does. */
#include <string.h>
#include <stdlib.h>

#define OOT_MAX_DRONES 16

extern unsigned char *oot_state_ptr(void);
extern size_t oot_state_size(void);

static unsigned char *g_slot[OOT_MAX_DRONES];
static unsigned char *g_template = NULL;   /* pristine post-Init state */
static int g_current = -1;

void oot_select_drone(int idx)
{
  if (idx < 0 || idx >= OOT_MAX_DRONES || idx == g_current) {
    return;
  }
  size_t n = oot_state_size();
  unsigned char *live = oot_state_ptr();

  if (g_template == NULL) {
    /* First call, before any vehicle has run: whatever controllerOutOfTreeInit left
       behind is the correct starting state. Zeroing instead would be subtly wrong --
       it would give every vehicle after the first a state Init never produced. */
    g_template = malloc(n);
    memcpy(g_template, live, n);
  }
  if (g_current >= 0) {
    memcpy(g_slot[g_current], live, n);          /* park the outgoing vehicle */
  }
  if (g_slot[idx] == NULL) {
    g_slot[idx] = malloc(n);
    memcpy(g_slot[idx], g_template, n);          /* fresh vehicle, fresh controller */
  }
  memcpy(live, g_slot[idx], n);                  /* load the incoming vehicle */
  g_current = idx;
}
