/**
 * Host test: rpm_filter_step() vs verbatim copy of pre-refactor rpm_get_all() per-motor logic.
 * Run via test_rpm_filter.sh
 */
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

#include "../rpm_filter.h"

/* Verbatim pre-2026-10-08 refactor logic (traj_iface.c rpm_get_all inner loop). */
static uint16_t rpm_filter_step_legacy_ref(uint16_t raw_rpm, uint8_t rpm_source,
                                           uint16_t *rpm_prev)
{
    const uint16_t dshot_rpm_abs_max = 28000u;
    const uint16_t dshot_slew_max_rpm = 10000u;
    uint16_t v = raw_rpm;

    if (v == 0xffffu) {
        v = 0u;
    }

    if (rpm_source != 0 && v > 0u) {
        bool reject = false;
        if (v > dshot_rpm_abs_max) {
            reject = true;
        } else if (*rpm_prev > 500u) {
            uint16_t lo = v < *rpm_prev ? v : *rpm_prev;
            uint16_t hi = v < *rpm_prev ? *rpm_prev : v;
            if ((uint16_t)(hi - lo) > dshot_slew_max_rpm) {
                reject = true;
            }
        }
        if (reject && *rpm_prev > 0u) {
            v = *rpm_prev;
        } else if (!reject) {
            *rpm_prev = v;
        }
    } else if (rpm_source == 0 && v > 0u) {
        *rpm_prev = v;
    }
    return v;
}

static int compare_one(uint16_t raw, uint8_t src, uint16_t prev_in,
                       uint64_t *mismatch_out)
{
    uint16_t prev_a = prev_in;
    uint16_t prev_b = prev_in;
    uint16_t out_a = rpm_filter_step_legacy_ref(raw, src, &prev_a);
    uint16_t out_b = rpm_filter_step(raw, src, &prev_b);
    if (out_a != out_b || prev_a != prev_b) {
        if (mismatch_out) {
            (*mismatch_out)++;
        }
        return 1;
    }
    return 0;
}

static int run_random_bitidentical(uint64_t n, uint64_t *mismatches)
{
    uint16_t prev_ref[2] = {0, 0};
    uint16_t prev_new[2] = {0, 0};
    for (uint64_t i = 0; i < n; i++) {
        uint16_t raw = (uint16_t)(rand() & 0xffff);
        uint8_t src = (uint8_t)(i & 1u);
        uint16_t out_ref = rpm_filter_step_legacy_ref(raw, src, &prev_ref[src]);
        uint16_t out_new = rpm_filter_step(raw, src, &prev_new[src]);
        if (out_ref != out_new || prev_ref[src] != prev_new[src]) {
            (*mismatches)++;
            if (*mismatches <= 5) {
                fprintf(stderr,
                        "mismatch i=%llu raw=%u src=%u out_ref=%u out_new=%u prev_ref=%u prev_new=%u\n",
                        (unsigned long long)i, raw, src, out_ref, out_new, prev_ref[src],
                        prev_new[src]);
            }
            return 1;
        }
    }
    return 0;
}

static int run_edge_cases(uint64_t *mismatches)
{
    static const uint16_t edges[] = {
        0, 1, 500, 501, 12000, 27999, 28000, 28001, 65534, 65535,
    };
    for (uint8_t src = 0; src <= 1; src++) {
        for (size_t pi = 0; pi < sizeof(edges) / sizeof(edges[0]); pi++) {
            uint16_t prev = edges[pi];
            for (size_t ri = 0; ri < sizeof(edges) / sizeof(edges[0]); ri++) {
                if (compare_one(edges[ri], src, prev, mismatches)) {
                    fprintf(stderr, "edge fail src=%u prev=%u raw=%u\n", src, prev, edges[ri]);
                    return 1;
                }
            }
        }
    }
    return 0;
}

static int run_jump_sequences(uint64_t *mismatches)
{
    const int jumps[] = {9999, 10000, 10001};
    for (uint8_t src = 0; src <= 1; src++) {
        for (size_t j = 0; j < sizeof(jumps) / sizeof(jumps[0]); j++) {
            uint16_t base = 15000;
            int delta = jumps[j];
            uint16_t seq[] = {base, (uint16_t)(base + delta), base, 0xffffu, base};
            uint16_t prev_ref = 600u;
            uint16_t prev_new = 600u;
            for (size_t k = 0; k < sizeof(seq) / sizeof(seq[0]); k++) {
                uint16_t out_ref = rpm_filter_step_legacy_ref(seq[k], src, &prev_ref);
                uint16_t out_new = rpm_filter_step(seq[k], src, &prev_new);
                if (out_ref != out_new || prev_ref != prev_new) {
                    (*mismatches)++;
                    fprintf(stderr, "jump seq fail src=%u jump=%d k=%zu raw=%u\n", src, delta,
                            k, seq[k]);
                    return 1;
                }
            }
        }
    }
    return 0;
}

/* Step 2: non-sentinel must still match legacy; sentinel hold on DShot only. */
static int run_sentinel_new_behavior(void)
{
    uint16_t prev = 19000u;
    uint16_t p_ref = prev;
    uint16_t p_new = prev;
    uint16_t out_ref = rpm_filter_step_legacy_ref(0xffffu, 1, &p_ref);
    uint16_t out_new = rpm_filter_step(0xffffu, 1, &p_new);
    if (out_ref != 0u) {
        fprintf(stderr, "legacy sentinel expected 0, got %u\n", out_ref);
        return 1;
    }
    if (out_new != prev || p_new != prev) {
        fprintf(stderr, "new sentinel hold fail out=%u prev=%u\n", out_new, p_new);
        return 1;
    }

    /* Deck path: sentinel still → 0 */
    prev = 19000u;
    p_new = prev;
    out_new = rpm_filter_step(0xffffu, 0, &p_new);
    if (out_new != 0u || p_new != prev) {
        fprintf(stderr, "deck sentinel fail out=%u prev=%u\n", out_new, p_new);
        return 1;
    }

    /* Sentinel-heavy sequences: outputs stay in [0, 28000] (hover band + gaps) */
    const uint16_t hover_seq[] = {0, 12000, 19000, 21000, 22000, 0xffff, 0xffff, 19000, 0xffff};
    prev = 0;
    for (size_t k = 0; k < sizeof(hover_seq) / sizeof(hover_seq[0]); k++) {
        uint16_t out = rpm_filter_step(hover_seq[k], 1, &prev);
        if (out > 28000u) {
            fprintf(stderr, "hover seq output > 28000: %u at k=%zu raw=%u\n", out, k,
                    hover_seq[k]);
            return 1;
        }
    }
    return 0;
}

static int run_non_sentinel_still_legacy(uint64_t n)
{
    for (uint64_t i = 0; i < n; i++) {
        uint16_t raw = (uint16_t)(rand() & 0xffff);
        if (raw == 0xffffu) {
            continue;
        }
        uint8_t src = (uint8_t)(i & 1u);
        uint16_t prev_r = (uint16_t)(1000 + (i % 20000));
        uint16_t prev_a = prev_r;
        uint16_t prev_b = prev_r;
        uint16_t oa = rpm_filter_step_legacy_ref(raw, src, &prev_a);
        uint16_t ob = rpm_filter_step(raw, src, &prev_b);
        if (oa != ob || prev_a != prev_b) {
            fprintf(stderr, "non-sentinel drift raw=%u src=%u\n", raw, src);
            return 1;
        }
    }
    return 0;
}

int main(int argc, char **argv)
{
    int mode = 1; /* default step 1 = bit-identical vs legacy */
    if (argc >= 2) {
        mode = (argv[1][0] == '2') ? 2 : 1;
    }

    srand(0xC0FFEE);
    uint64_t mismatches = 0;

    if (mode == 1) {
        if (run_edge_cases(&mismatches)) {
            return 1;
        }
        if (run_jump_sequences(&mismatches)) {
            return 1;
        }
        if (run_random_bitidentical(1000000u, &mismatches)) {
            fprintf(stderr, "FAIL: %llu mismatches in 1e6 random\n",
                    (unsigned long long)mismatches);
            return 1;
        }
        printf("PASS step1: bit-identical vs legacy on edges, jumps, 1e6 random (mismatches=%llu)\n",
               (unsigned long long)mismatches);
        return 0;
    }

    if (mode != 2) {
        return 1;
    }

    if (run_non_sentinel_still_legacy(500000u)) {
        return 1;
    }
    if (run_sentinel_new_behavior()) {
        return 1;
    }
    printf("PASS step2: non-sentinel matches legacy; sentinel hold-last-good (DShot); outputs in [0,28000]\n");
    return 0;
}
