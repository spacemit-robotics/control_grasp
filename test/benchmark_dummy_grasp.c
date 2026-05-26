/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 */

#define _POSIX_C_SOURCE 200809L

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include "grasp.h"

static uint64_t monotonic_ns(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000000ULL + (uint64_t)ts.tv_nsec;
}

int main(int argc, char **argv) {
    int iters = 20000;
    double max_avg_us = 200.0;
    if (argc > 1) {
        iters = atoi(argv[1]);
    }
    if (argc > 2) {
        max_avg_us = atof(argv[2]);
    }

    struct grasp_dev *dev = grasp_alloc("dummy", NULL);
    if (!dev) {
        fprintf(stderr, "failed to alloc dummy grasp\n");
        return 1;
    }

    uint64_t start = monotonic_ns();
    for (int i = 0; i < iters; ++i) {
        if (grasp_execute(dev, (i & 1) ? GRASP_CMD_GRAB : GRASP_CMD_RELEASE, 0.5f) != GRASP_OK) {
            fprintf(stderr, "grasp_execute failed at iter=%d\n", i);
            grasp_free(dev);
            return 2;
        }
        grasp_tick(dev, 0.001f);
    }
    uint64_t end = monotonic_ns();

    double total_us = (double)(end - start) / 1000.0;
    double avg_us = total_us / (double)iters;
    printf("iters=%d total_us=%.3f avg_us=%.3f threshold_us=%.3f\n", iters, total_us, avg_us, max_avg_us);
    if (avg_us > max_avg_us) {
        fprintf(stderr, "PERF_FAIL avg_us=%.3f threshold_us=%.3f\n", avg_us, max_avg_us);
        grasp_free(dev);
        return 3;
    }

    printf("PERF_OK avg_us=%.3f threshold_us=%.3f\n", avg_us, max_avg_us);
    grasp_free(dev);
    return 0;
}
