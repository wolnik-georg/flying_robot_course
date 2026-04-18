/*
 * wrapper.h — minimal header for bindgen
 *
 * Pulls in the Crazyflie stabilizer types (control_t, setpoint_t, sensorData_t,
 * state_t, stabilizerStep_t, …) and the out-of-tree controller interface.
 *
 * Include paths are provided by build.rs via clang_arg("-I...").
 */
#include "stabilizer_types.h"
#include "controller.h"
