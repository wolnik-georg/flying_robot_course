/* Minimal stubs for test_oot4_dispatch_wrapper.c -- indi defaults to 0 (geometric only) with no
 * runtime PARAM override available in this isolated harness, so the RPM/param read path is
 * never actually exercised; these just need to exist for the link to succeed. */
#include <stdint.h>
#include "log.h"
#include "param.h"

logVarId_t logGetVarId(const char *g, const char *n) { (void)g; (void)n; return 0xffffu; }
uint32_t logGetUint(logVarId_t v) { (void)v; return 0u; }
float logGetFloat(logVarId_t v) { (void)v; return 0.0f; }
paramVarId_t paramGetVarId(const char *g, const char *n) {
    (void)g; (void)n; paramVarId_t id = {1, 0}; return id;
}
uint32_t paramGetUint(paramVarId_t v) { (void)v; return 0u; }  /* deck NOT present -> indi path inert */
uint64_t usecTimestamp(void) { static uint64_t t = 0; t += 2000; return t; }
float powerDistributionGetMaxThrust(void) { return 0.2f; }
