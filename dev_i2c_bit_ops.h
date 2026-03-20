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

#ifndef SOFT_I2C_LOG_OUT
/**
 * @brief 软件 I2C 日志输出宏。
 *
 * @note 默认关闭输出，用户可在包含头文件前重定义该宏。
 */
#define SOFT_I2C_LOG_OUT(...) ((void)0)
#endif

#ifndef SOFT_I2C_LOG_E_ENABLE
#define SOFT_I2C_LOG_E_ENABLE 0 /**< 是否开启错误日志。 */
#endif

#ifndef SOFT_I2C_LOG_W_ENABLE
#define SOFT_I2C_LOG_W_ENABLE 0 /**< 是否开启告警日志。 */
#endif

#ifndef SOFT_I2C_LOG_D_ENABLE
#define SOFT_I2C_LOG_D_ENABLE 0 /**< 是否开启调试日志。 */
#endif

#ifndef SOFT_I2C_BITOPS_DEBUG_ENABLE
#define SOFT_I2C_BITOPS_DEBUG_ENABLE 0 /**< 是否开启位操作级总线检查。 */
#endif

#ifndef SOFT_I2C_DEFAULT_DELAY_US
#define SOFT_I2C_DEFAULT_DELAY_US 5U /**< 默认半周期延时基准，单位为微秒。 */
#endif

#ifndef SOFT_I2C_DEFAULT_TIMEOUT_TICK
#define SOFT_I2C_DEFAULT_TIMEOUT_TICK 100U /**< 默认时钟拉伸等待超时 tick。 */
#endif

#ifndef SOFT_I2C_BUS_RECOVERY_PULSE_COUNT
#define SOFT_I2C_BUS_RECOVERY_PULSE_COUNT                                      \
  9U /**< 总线恢复时最多发送的 SCL 脉冲数。 */
#endif

#ifndef SOFT_I2C_INIT_BUS_RECOVERY_ENABLE
#define SOFT_I2C_INIT_BUS_RECOVERY_ENABLE                                      \
  0 /**< 是否在初始化阶段启用总线恢复流程。 */
#endif

/**
 * @brief 软件 I2C 返回码定义。
 */
#define SOFT_I2C_EOK 0       /**< 执行成功。 */
#define SOFT_I2C_ERROR -1    /**< 通用错误。 */
#define SOFT_I2C_ETIMEOUT -2 /**< 执行超时。 */
#define SOFT_I2C_EIO -3      /**< 总线 IO 错误。 */

/**
 * @brief 软件 I2C 消息标志位定义。
 */
#define SOFT_I2C_WR (0x0000U)          /**< 写消息。 */
#define SOFT_I2C_RD (1u << 0)          /**< 读消息。 */
#define SOFT_I2C_ADDR_10BIT (1u << 2)  /**< 使用 10 位从机地址。 */
#define SOFT_I2C_NO_START (1u << 4)    /**< 本消息前不发送 START。 */
#define SOFT_I2C_IGNORE_NACK (1u << 5) /**< 忽略从机返回的 NACK。 */
#define SOFT_I2C_NO_READ_ACK (1u << 6) /**< 读消息时不主动发送 ACK/NACK。 */
#define SOFT_I2C_NO_STOP (1u << 7)     /**< 本消息结束后不发送 STOP。 */

/**
 * @brief 设置 GPIO 线状态回调类型。
 *
 * @param[in] user_data 用户私有上下文。
 * @param[in] state GPIO 目标状态，0 表示拉低，非 0 表示释放为高。
 */
typedef void (*soft_i2c_set_line_fn)(void *user_data, int32_t state);

/**
 * @brief 读取 GPIO 线状态回调类型。
 *
 * @param[in] user_data 用户私有上下文。
 *
 * @return GPIO 当前状态，0 表示低电平，非 0 表示高电平。
 */
typedef int32_t (*soft_i2c_get_line_fn)(void *user_data);

/**
 * @brief I/O 引脚初始化回调类型。
 *
 * @param[in] user_data 用户私有上下文。
 *
 * @return 成功返回 `SOFT_I2C_EOK`，失败返回负错误码。
 */
typedef int32_t (*soft_i2c_pin_init_fn)(void *user_data);

/**
 * @brief I/O 引脚反初始化回调类型。
 *
 * @param[in] user_data 用户私有上下文。
 */
typedef void (*soft_i2c_pin_deinit_fn)(void *user_data);

/**
 * @brief 微秒级延时回调类型。
 *
 * @param[in] user_data 用户私有上下文。
 * @param[in] us 延时长度，单位为微秒。
 */
typedef void (*soft_i2c_udelay_fn)(void *user_data, uint32_t us);

/**
 * @brief 获取系统 tick 回调类型。
 *
 * @param[in] user_data 用户私有上下文。
 *
 * @return 当前系统 tick。
 */
typedef uint32_t (*soft_i2c_get_tick_fn)(void *user_data);

/**
 * @brief 软件 I2C 消息描述符。
 */
typedef struct {
  uint16_t addr;  /**< 从机地址，支持 7 位或 10 位。 */
  uint16_t flags; /**< 传输标志位，参考 `SOFT_I2C_*` 宏。 */
  uint16_t len;   /**< 本次传输长度，单位为字节。 */
  uint8_t *buf;   /**< 数据缓冲区指针。 */
} soft_i2c_msg_t;

/**
 * @brief 软件 I2C GPIO 操作集合。
 */
typedef struct {
  soft_i2c_set_line_fn set_sda; /**< 设置 SDA 电平，必选。 */
  soft_i2c_set_line_fn set_scl; /**< 设置 SCL 电平，必选。 */
  soft_i2c_get_line_fn get_sda; /**< 读取 SDA 电平，必选。 */
  soft_i2c_get_line_fn get_scl; /**< 读取 SCL 电平，可选。 */

  soft_i2c_pin_init_fn pin_init;     /**< 引脚初始化回调，可选。 */
  soft_i2c_pin_deinit_fn pin_deinit; /**< 引脚反初始化回调，可选。 */
} soft_i2c_io_ops_t;

/**
 * @brief 软件 I2C 系统依赖操作集合。
 */
typedef struct {
  soft_i2c_udelay_fn udelay; /**< 微秒级延时回调，必选。 */
  soft_i2c_get_tick_fn
      get_tick; /**< 获取系统 tick 回调，启用 SCL 读取时必选。 */
} soft_i2c_sys_ops_t;

/**
 * @brief 软件 I2C 运行配置。
 */
typedef struct {
  uint32_t delay_us;     /**< SCL/SDA 基础延时，单位为微秒。 */
  uint32_t timeout_tick; /**< SCL 拉高等待超时，单位为 tick。 */
  uint32_t retries;      /**< 地址发送失败时的重试次数。 */
} soft_i2c_cfg_t;

/**
 * @brief 软件 I2C 控制器对象。
 *
 * @note 调用者需要在 `soft_i2c_init()` 前完成必要回调与配置填充。
 */
typedef struct {
  void *user_data;        /**< 底层平台私有上下文。 */
  soft_i2c_io_ops_t io;   /**< GPIO 操作集合。 */
  soft_i2c_sys_ops_t sys; /**< 系统依赖操作集合。 */
  soft_i2c_cfg_t cfg;     /**< 运行时配置。 */
  bool init_flag;         /**< 初始化状态，由库内部维护。 */
} soft_i2c_t;

/**
 * @brief 初始化软件 I2C 对象。
 *
 * @param[in,out] i2c 软件 I2C 对象指针。
 *
 * @return 成功返回 `SOFT_I2C_EOK`，失败返回负错误码。
 */
int32_t soft_i2c_init(soft_i2c_t *i2c);

/**
 * @brief 反初始化软件 I2C 对象。
 *
 * @param[in,out] i2c 软件 I2C 对象指针。
 */
void soft_i2c_deinit(soft_i2c_t *i2c);

/**
 * @brief 执行软件 I2C 消息传输。
 *
 * @param[in,out] i2c 软件 I2C 对象指针。
 * @param[in,out] msgs 消息数组指针。
 * @param[in] num 消息数量。
 *
 * @return 成功时返回已完成消息数，失败时返回负错误码。
 */
int32_t soft_i2c_xfer(soft_i2c_t *i2c, soft_i2c_msg_t msgs[], uint32_t num);

#ifdef __cplusplus
}
#endif

#endif
