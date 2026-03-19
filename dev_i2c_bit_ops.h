/*
 * Copyright (c) 2026 Maruolang
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author        Notes
 * 2026-03-12     Maruolang     first version
 */

#ifndef __DEV_I2C_BIT_OPS_H__
#define __DEV_I2C_BIT_OPS_H__

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SOFT_I2C_LOG_OUT(...) ((void)0)
// #define SOFT_I2C_LOG_OUT(...) ((void)printf(__VA_ARGS__))
#define SOFT_I2C_LOG_E_ENABLE 0
#define SOFT_I2C_LOG_W_ENABLE 0
#define SOFT_I2C_LOG_D_ENABLE 0
#define SOFT_I2C_BITOPS_DEBUG_ENABLE 0

/* return */
#define SOFT_I2C_EOK 0       /**< There is no error */
#define SOFT_I2C_ERROR -1    /**< A generic/unknown error happens */
#define SOFT_I2C_ETIMEOUT -2 /**< Timed out */
#define SOFT_I2C_EIO -3      /**< IO error */

/* msg flag */
#define SOFT_I2C_WR (0x0000)           /*!< write flag */
#define SOFT_I2C_RD (1u << 0)          /*!< read flag  */
#define SOFT_I2C_ADDR_10BIT (1u << 2)  /*!< ten bit chip address */
#define SOFT_I2C_NO_START (1u << 4)    /*!< not generate START */
#define SOFT_I2C_IGNORE_NACK (1u << 5) /*!< ignore NACK */
#define SOFT_I2C_NO_READ_ACK (1u << 6) /*! do not ACK */
#define SOFT_I2C_NO_STOP (1u << 7)     /*!< not generate STOP*/

/**
 * @brief I2C Message
 */
typedef struct {
  uint16_t addr;
  uint16_t flags;
  uint16_t len;
  uint8_t *buf;
} soft_i2c_msg_t;

typedef struct {
  void *user_data;

  struct {
    void (*set_sda)(void *user_data, int32_t state);
    void (*set_scl)(void *user_data, int32_t state);
    int32_t (*get_sda)(void *user_data);
    int32_t (*get_scl)(void *user_data);

    int32_t (*pin_init)(void *user_data); /* 可选 */
    void (*pin_deinit)(void *user_data);  /* 可选 */
  } io;

  struct {
    void (*udelay)(void *user_data, uint32_t us);
    uint32_t (*get_tick)(void *user_data);
  } sys;

  struct {
    uint32_t delay_us; /* scl and sda line delay */
    uint32_t timeout_tick;
    uint32_t retries;
  } cfg;

  bool init_flag;
} soft_i2c_t;

int32_t soft_i2c_init(soft_i2c_t *i2c);
void soft_i2c_deinit(soft_i2c_t *i2c);

/* 成功返回 num，失败返回错误 */
int32_t soft_i2c_xfer(soft_i2c_t *i2c, soft_i2c_msg_t msgs[], uint32_t num);

#ifdef __cplusplus
}
#endif

#endif
