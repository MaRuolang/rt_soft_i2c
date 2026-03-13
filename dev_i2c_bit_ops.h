/*
 * Copyright (c)
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author        Notes
 * 2026-03-12     Maruolang     first version
 */

#ifndef __DEV_I2C_BIT_OPS_H__
#define __DEV_I2C_BIT_OPS_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef int rt_bool_t; /**< boolean type */
typedef int rt_err_t;

#define RT_TRUE 1  /**< boolean true  */
#define RT_FALSE 0 /**< boolean fails */

#define RT_EOK 0      /**< There is no error */
#define RT_ERROR 1    /**< A generic/unknown error happens */
#define RT_ETIMEOUT 2 /**< Timed out */
#define RT_EIO 8      /**< IO error */

#define RT_I2C_WR 0x0000             /*!< i2c write flag */
#define RT_I2C_RD (1u << 0)          /*!< i2c read flag  */
#define RT_I2C_ADDR_10BIT (1u << 2)  /*!< this is a ten bit chip address */
#define RT_I2C_NO_START (1u << 4)    /*!< do not generate START condition */
#define RT_I2C_IGNORE_NACK (1u << 5) /*!< ignore NACK from slave */
#define RT_I2C_NO_READ_ACK (1u << 6) /* when I2C reading, we do not ACK */
#define RT_I2C_NO_STOP (1u << 7)     /*!< do not generate STOP condition */

/**
 * @brief I2C Message
 */
struct rt_i2c_msg {
  uint16_t addr;
  uint16_t flags;
  uint16_t len;
  uint8_t *buf;
};

struct rt_i2c_bit_ops {
  void *data; /* private data for lowlevel routines */
  void (*set_sda)(void *data, int32_t state);
  void (*set_scl)(void *data, int32_t state);
  int32_t (*get_sda)(void *data);
  int32_t (*get_scl)(void *data);

  void (*udelay)(uint32_t us);
  uint32_t (*tick_get)(void);

  uint32_t delay_us; /* scl and sda line delay */
  uint32_t timeout;  /* in tick */
  uint32_t retries;

  void (*pin_init)(void);
  rt_bool_t i2c_pin_init_flag;
};

int32_t i2c_bit_xfer(struct rt_i2c_bit_ops *ops, struct rt_i2c_msg msgs[],
                     uint32_t num);

#ifdef __cplusplus
}
#endif

#endif
