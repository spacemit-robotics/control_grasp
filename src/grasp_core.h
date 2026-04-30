/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file grasp_core.h
 * @brief Private header for grasp/end-effector control component (internal use only)
 */

#ifndef GRASP_CORE_H
#define GRASP_CORE_H

#include "../include/grasp.h"
#include <pthread.h>
#include <stddef.h>

/* ==========================================================================
 * 1. Virtual Operations Table (driver interface)
 * ========================================================================== */

struct grasp_ops {
    int (*execute)(struct grasp_dev *dev, grasp_cmd_type_t type, float effort);
    int (*set_position)(struct grasp_dev *dev, float position);
    grasp_state_t (*get_state)(struct grasp_dev *dev);
    int (*get_feedback)(struct grasp_dev *dev, float *out_pos, float *out_load);
    void (*tick)(struct grasp_dev *dev, float dt_s);
    int (*calibrate)(struct grasp_dev *dev); /* 可选：校准功能 */
    void (*free)(struct grasp_dev *dev);
};

/* ==========================================================================
 * 2. Device Structure (internal)
 * ========================================================================== */

struct grasp_dev {
    /* Identity */
    const char *name;

    /* Operations */
    const struct grasp_ops *ops;
    void *priv_data;

    /* State (protected by lock) */
    grasp_state_t state;
    float cur_position; /* [0.0 ~ 1.0] */
    float cur_load;     /* raw load/current */
    grasp_config_t config;
    pthread_mutex_t state_lock;

    /* Timeout tracking */
    float elapsed_ms; /* accumulated time since last command */

    /* Runtime */
    bool running;
};

/* ==========================================================================
 * 3. Driver Registry
 * ========================================================================== */

typedef struct grasp_dev *(*grasp_factory_t)(const char *name, void *args);

struct grasp_driver_info {
    const char *name;
    grasp_factory_t factory;
    struct grasp_driver_info *next;
};

void grasp_driver_register(struct grasp_driver_info *info);

/**
 * @brief Register a grasp driver (called at load time via constructor)
 * @param _name    Driver name string
 * @param _factory Factory function
 */
#define REGISTER_GRASP_DRIVER(_name, _factory)                                 \
    static struct grasp_driver_info __drv_info_##_factory = {                  \
        .name = _name, .factory = _factory, .next = NULL};                     \
    __attribute__((constructor)) static void __auto_reg_##_factory(void) {     \
        grasp_driver_register(&__drv_info_##_factory);                         \
    }

/* ==========================================================================
 * 4. Internal Helpers
 * ========================================================================== */

/**
 * @brief Allocate a grasp device with private data
 */
struct grasp_dev *grasp_dev_alloc(const char *name, size_t priv_size);

/**
 * @brief Default free implementation
 */
void grasp_dev_free_default(struct grasp_dev *dev);

#endif  // GRASP_CORE_H
