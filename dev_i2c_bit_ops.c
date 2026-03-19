/*
 * Copyright (c) 2026 Maruolang
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author        Notes
 * 2026-03-12     Maruolang     first version
 */

#include "dev_i2c_bit_ops.h"

#include <stddef.h>

#if SOFT_I2C_LOG_E_ENABLE
#define SOFT_I2C_LOG_E(...) (SOFT_I2C_LOG_OUT("[E] " __VA_ARGS__))
#else
#define SOFT_I2C_LOG_E(...) ((void)0)
#endif

#if SOFT_I2C_LOG_W_ENABLE
#define SOFT_I2C_LOG_W(...) (SOFT_I2C_LOG_OUT("[W] " __VA_ARGS__))
#else
#define SOFT_I2C_LOG_W(...) ((void)0)
#endif

#if SOFT_I2C_LOG_D_ENABLE
#define SOFT_I2C_LOG_D(...) (SOFT_I2C_LOG_OUT("[D] " __VA_ARGS__))
#else
#define SOFT_I2C_LOG_D(...) ((void)0)
#endif

#define SOFT_I2C_SET_SDA(i2c, val) ((i2c)->io.set_sda((i2c)->user_data, (val)))
#define SOFT_I2C_SET_SCL(i2c, val) ((i2c)->io.set_scl((i2c)->user_data, (val)))
#define SOFT_I2C_GET_SDA(i2c) ((i2c)->io.get_sda((i2c)->user_data))
#define SOFT_I2C_GET_SCL(i2c) ((i2c)->io.get_scl((i2c)->user_data))
#define SOFT_I2C_GET_TICK(i2c) ((i2c)->sys.get_tick((i2c)->user_data))

#define SOFT_I2C_SDA_LOW(i2c) SOFT_I2C_SET_SDA((i2c), 0)
#define SOFT_I2C_SDA_HIGH(i2c) SOFT_I2C_SET_SDA((i2c), 1)
#define SOFT_I2C_SCL_LOW(i2c) SOFT_I2C_SET_SCL((i2c), 0)

/**
 * @brief 按半个时钟周期执行延时。
 *
 * @param[in] i2c 软件 I2C 对象。
 */
static inline void soft_i2c_delay_half(soft_i2c_t *i2c) {
  i2c->sys.udelay(i2c->user_data, (i2c->cfg.delay_us + 1U) >> 1);
}

/**
 * @brief 按一个完整时钟周期执行延时。
 *
 * @param[in] i2c 软件 I2C 对象。
 */
static inline void soft_i2c_delay_full(soft_i2c_t *i2c) {
  i2c->sys.udelay(i2c->user_data, i2c->cfg.delay_us);
}

/**
 * @brief 校验对象配置并补齐默认参数。
 *
 * @param[in,out] i2c 软件 I2C 对象。
 *
 * @note `get_scl` 为可选回调，仅在启用 SCL 实际电平检测时要求同时提供
 *       `get_tick`。
 *
 * @return 成功返回 `SOFT_I2C_EOK`，失败返回负错误码。
 */
static int32_t soft_i2c_prepare_context(soft_i2c_t *i2c) {
  if (i2c == NULL) {
    return SOFT_I2C_ERROR;
  }

  if ((i2c->io.set_sda == NULL) || (i2c->io.set_scl == NULL) ||
      (i2c->io.get_sda == NULL) || (i2c->sys.udelay == NULL)) {
    return SOFT_I2C_ERROR;
  }

  if ((i2c->io.get_scl != NULL) && (i2c->sys.get_tick == NULL)) {
    return SOFT_I2C_ERROR;
  }

  if (i2c->cfg.delay_us == 0U) {
    i2c->cfg.delay_us = SOFT_I2C_DEFAULT_DELAY_US;
  }

  if ((i2c->io.get_scl != NULL) && (i2c->cfg.timeout_tick == 0U)) {
    i2c->cfg.timeout_tick = SOFT_I2C_DEFAULT_TIMEOUT_TICK;
  }

  return SOFT_I2C_EOK;
}

/**
 * @brief 释放 SCL 线并等待其真正变为高电平。
 *
 * @param[in] i2c 软件 I2C 对象。
 *
 * @return 成功返回 `SOFT_I2C_EOK`，超时返回 `SOFT_I2C_ETIMEOUT`。
 */
static int32_t soft_i2c_release_scl(soft_i2c_t *i2c) {
  uint32_t start_tick = 0U;

  SOFT_I2C_SET_SCL(i2c, 1);

  if (i2c->io.get_scl == NULL) {
    goto done;
  }

  start_tick = SOFT_I2C_GET_TICK(i2c);
  while (!SOFT_I2C_GET_SCL(i2c)) {
    if ((SOFT_I2C_GET_TICK(i2c) - start_tick) > i2c->cfg.timeout_tick) {
      return SOFT_I2C_ETIMEOUT;
    }
    soft_i2c_delay_half(i2c);
  }

#if SOFT_I2C_BITOPS_DEBUG_ENABLE
  if (SOFT_I2C_GET_TICK(i2c) != start_tick) {
    SOFT_I2C_LOG_D("wait %lu tick for SCL line to go high",
                   (unsigned long)(SOFT_I2C_GET_TICK(i2c) - start_tick));
  }
#endif

done:
  soft_i2c_delay_half(i2c);

  return SOFT_I2C_EOK;
}

/**
 * @brief 发送起始信号。
 *
 * @param[in] i2c 软件 I2C 对象。
 */
static void soft_i2c_start(soft_i2c_t *i2c) {
#if SOFT_I2C_BITOPS_DEBUG_ENABLE
  if ((i2c->io.get_scl != NULL) && !SOFT_I2C_GET_SCL(i2c)) {
    SOFT_I2C_LOG_E("I2C bus error, SCL line low");
  }
  if (!SOFT_I2C_GET_SDA(i2c)) {
    SOFT_I2C_LOG_E("I2C bus error, SDA line low");
  }
#endif

  SOFT_I2C_SDA_LOW(i2c);
  soft_i2c_delay_half(i2c);
  SOFT_I2C_SCL_LOW(i2c);
}

/**
 * @brief 发送重复起始信号。
 *
 * @param[in] i2c 软件 I2C 对象。
 *
 * @return 成功返回 `SOFT_I2C_EOK`，失败返回负错误码。
 */
static int32_t soft_i2c_restart(soft_i2c_t *i2c) {
  int32_t ret = SOFT_I2C_EOK;

  SOFT_I2C_SDA_HIGH(i2c);
  ret = soft_i2c_release_scl(i2c);
  if (ret != SOFT_I2C_EOK) {
    return ret;
  }

  soft_i2c_delay_half(i2c);
  SOFT_I2C_SDA_LOW(i2c);
  soft_i2c_delay_half(i2c);
  SOFT_I2C_SCL_LOW(i2c);

  return SOFT_I2C_EOK;
}

/**
 * @brief 发送停止信号。
 *
 * @param[in] i2c 软件 I2C 对象。
 *
 * @return 成功返回 `SOFT_I2C_EOK`，失败返回负错误码。
 */
static int32_t soft_i2c_stop(soft_i2c_t *i2c) {
  int32_t ret = SOFT_I2C_EOK;

  SOFT_I2C_SDA_LOW(i2c);
  soft_i2c_delay_half(i2c);

  ret = soft_i2c_release_scl(i2c);
  if (ret != SOFT_I2C_EOK) {
    return ret;
  }

  soft_i2c_delay_half(i2c);
  SOFT_I2C_SDA_HIGH(i2c);
  soft_i2c_delay_full(i2c);

  return SOFT_I2C_EOK;
}

/**
 * @brief 等待从机 ACK/NACK。
 *
 * @param[in] i2c 软件 I2C 对象。
 *
 * @return `1` 表示 ACK，`0` 表示 NACK，负值表示错误。
 */
static int32_t soft_i2c_wait_ack(soft_i2c_t *i2c) {
  int32_t ack = 0;
  int32_t ret = SOFT_I2C_EOK;

  SOFT_I2C_SDA_HIGH(i2c);
  soft_i2c_delay_half(i2c);

  ret = soft_i2c_release_scl(i2c);
  if (ret != SOFT_I2C_EOK) {
    SOFT_I2C_LOG_W("wait ack timeout");
    return ret;
  }

  ack = !SOFT_I2C_GET_SDA(i2c);
  SOFT_I2C_LOG_D("%s", ack ? "ACK" : "NACK");

  SOFT_I2C_SCL_LOW(i2c);

  return ack;
}

/**
 * @brief 向总线写入单个字节。
 *
 * @param[in] i2c 软件 I2C 对象。
 * @param[in] data 待写入数据。
 *
 * @return `1` 表示收到 ACK，`0` 表示收到 NACK，负值表示错误。
 */
static int32_t soft_i2c_write_byte(soft_i2c_t *i2c, uint8_t data) {
  int32_t bit_index = 0;
  uint8_t bit_value = 0U;
  int32_t ret = SOFT_I2C_EOK;

  for (bit_index = 7; bit_index >= 0; bit_index--) {
    SOFT_I2C_SCL_LOW(i2c);
    bit_value = (uint8_t)((data >> bit_index) & 0x01U);
    SOFT_I2C_SET_SDA(i2c, bit_value);
    soft_i2c_delay_half(i2c);

    ret = soft_i2c_release_scl(i2c);
    if (ret != SOFT_I2C_EOK) {
      SOFT_I2C_LOG_D("i2c_writeb: 0x%02x, wait scl pin high timeout at bit %ld",
                     (unsigned int)data, (long)bit_index);
      return ret;
    }
  }

  SOFT_I2C_SCL_LOW(i2c);
  soft_i2c_delay_half(i2c);

  return soft_i2c_wait_ack(i2c);
}

/**
 * @brief 从总线读取单个字节。
 *
 * @param[in] i2c 软件 I2C 对象。
 *
 * @return 成功返回读取到的字节值，失败返回负错误码。
 */
static int32_t soft_i2c_read_byte(soft_i2c_t *i2c) {
  uint8_t bit_index = 0U;
  uint8_t data = 0U;
  int32_t ret = SOFT_I2C_EOK;

  SOFT_I2C_SDA_HIGH(i2c);
  soft_i2c_delay_half(i2c);

  for (bit_index = 0; bit_index < 8U; bit_index++) {
    data <<= 1;

    ret = soft_i2c_release_scl(i2c);
    if (ret != SOFT_I2C_EOK) {
      SOFT_I2C_LOG_D("i2c_readb: wait scl pin high timeout at bit %u",
                     (unsigned int)(7U - bit_index));
      return ret;
    }

    if (SOFT_I2C_GET_SDA(i2c)) {
      data |= 0x01U;
    }

    SOFT_I2C_SCL_LOW(i2c);
    soft_i2c_delay_full(i2c);
  }

  return (int32_t)data;
}

/**
 * @brief 连续发送数据字节。
 *
 * @param[in] i2c 软件 I2C 对象。
 * @param[in] msg I2C 消息。
 *
 * @return 成功返回已发送字节数，失败返回负错误码。
 */
static int32_t soft_i2c_send_bytes(soft_i2c_t *i2c, const soft_i2c_msg_t *msg) {
  int32_t ret = SOFT_I2C_EOK;
  uint32_t bytes = 0U;
  const uint8_t *ptr = msg->buf;
  int32_t count = (int32_t)msg->len;
  const bool ignore_nack = ((msg->flags & SOFT_I2C_IGNORE_NACK) != 0U);

  while (count > 0) {
    ret = soft_i2c_write_byte(i2c, *ptr);

    if ((ret > 0) || (ignore_nack && (ret == 0))) {
      count--;
      ptr++;
      bytes++;
    } else if (ret == 0) {
      SOFT_I2C_LOG_D("send bytes: NACK.");
      return (int32_t)bytes;
    } else {
      SOFT_I2C_LOG_E("send bytes: error %ld", (long)ret);
      return ret;
    }
  }

  return (int32_t)bytes;
}

/**
 * @brief 在读流程中发送 ACK 或 NACK。
 *
 * @param[in] i2c 软件 I2C 对象。
 * @param[in] ack 非 0 表示发送 ACK，0 表示发送 NACK。
 *
 * @return 成功返回 `SOFT_I2C_EOK`，失败返回负错误码。
 */
static int32_t soft_i2c_send_ack_or_nack(soft_i2c_t *i2c, int32_t ack) {
  int32_t ret = SOFT_I2C_EOK;

  SOFT_I2C_SET_SDA(i2c, ack ? 0 : 1);
  soft_i2c_delay_half(i2c);

  ret = soft_i2c_release_scl(i2c);
  if (ret != SOFT_I2C_EOK) {
    SOFT_I2C_LOG_E("ACK or NACK timeout.");
    return ret;
  }

  SOFT_I2C_SCL_LOW(i2c);

  return SOFT_I2C_EOK;
}

/**
 * @brief 连续接收数据字节。
 *
 * @param[in] i2c 软件 I2C 对象。
 * @param[out] msg I2C 消息。
 *
 * @return 成功返回已接收字节数，失败返回负错误码。
 */
static int32_t soft_i2c_recv_bytes(soft_i2c_t *i2c, soft_i2c_msg_t *msg) {
  int32_t val = SOFT_I2C_EOK;
  int32_t bytes = 0;
  uint8_t *ptr = msg->buf;
  int32_t count = (int32_t)msg->len;
  const uint32_t flags = msg->flags;

  while (count > 0) {
    val = soft_i2c_read_byte(i2c);
    if (val < 0) {
      return val;
    }

    *ptr = (uint8_t)val;
    bytes++;
    ptr++;
    count--;

    SOFT_I2C_LOG_D("receive bytes: 0x%02x, %s", (unsigned int)((uint8_t)val),
                   (flags & SOFT_I2C_NO_READ_ACK)
                       ? "(No ACK/NACK)"
                       : (count > 0 ? "ACK" : "NACK"));

    if ((flags & SOFT_I2C_NO_READ_ACK) == 0U) {
      val = soft_i2c_send_ack_or_nack(i2c, (count > 0));
      if (val < 0) {
        return val;
      }
    }
  }

  return bytes;
}

/**
 * @brief 向从机发送地址字节，并按需执行重试。
 *
 * @param[in] i2c 软件 I2C 对象。
 * @param[in] addr 地址字节。
 * @param[in] retries 重试次数。
 *
 * @return `1` 表示收到 ACK，`0` 表示收到 NACK，负值表示错误。
 */
static int32_t soft_i2c_send_address_byte(soft_i2c_t *i2c, uint8_t addr,
                                          int32_t retries) {
  int32_t retry_index = 0;
  int32_t ret = 0;

  for (retry_index = 0; retry_index <= retries; retry_index++) {
    ret = soft_i2c_write_byte(i2c, addr);
    if ((ret == 1) || (ret < 0) || (retry_index == retries)) {
      break;
    }

    SOFT_I2C_LOG_D("send stop condition");
    ret = soft_i2c_stop(i2c);
    if (ret < 0) {
      return ret;
    }

    soft_i2c_delay_full(i2c);
    SOFT_I2C_LOG_D("send start condition");
    soft_i2c_start(i2c);
  }

  return ret;
}

/**
 * @brief 根据消息内容发送从机地址阶段。
 *
 * @param[in] i2c 软件 I2C 对象。
 * @param[in] msg I2C 消息。
 *
 * @return 成功返回 `SOFT_I2C_EOK`，失败返回负错误码。
 */
static int32_t soft_i2c_send_slave_address(soft_i2c_t *i2c,
                                           const soft_i2c_msg_t *msg) {
  const uint16_t flags = msg->flags;
  const bool ignore_nack = ((flags & SOFT_I2C_IGNORE_NACK) != 0U);
  uint8_t addr1 = 0U;
  uint8_t addr2 = 0U;
  int32_t retries = ignore_nack ? 0 : (int32_t)i2c->cfg.retries;
  int32_t ret = SOFT_I2C_EOK;

  if ((flags & SOFT_I2C_ADDR_10BIT) != 0U) {
    addr1 = (uint8_t)(0xF0U | ((msg->addr >> 7) & 0x06U));
    addr2 = (uint8_t)(msg->addr & 0xFFU);

    SOFT_I2C_LOG_D("addr1: %u, addr2: %u", (unsigned int)addr1,
                   (unsigned int)addr2);

    ret = soft_i2c_send_address_byte(i2c, addr1, retries);
    if (ret < 0) {
      return ret;
    }
    if ((ret != 1) && !ignore_nack) {
      SOFT_I2C_LOG_W("NACK: sending first addr");
      return SOFT_I2C_EIO;
    }

    ret = soft_i2c_write_byte(i2c, addr2);
    if (ret < 0) {
      return ret;
    }
    if ((ret != 1) && !ignore_nack) {
      SOFT_I2C_LOG_W("NACK: sending second addr");
      return SOFT_I2C_EIO;
    }

    if ((flags & SOFT_I2C_RD) != 0U) {
      SOFT_I2C_LOG_D("send repeated start condition");
      ret = soft_i2c_restart(i2c);
      if (ret < 0) {
        return ret;
      }

      addr1 |= 0x01U;
      ret = soft_i2c_send_address_byte(i2c, addr1, retries);
      if (ret < 0) {
        return ret;
      }
      if ((ret != 1) && !ignore_nack) {
        SOFT_I2C_LOG_E("NACK: sending repeated addr");
        return SOFT_I2C_EIO;
      }
    }
  } else {
    addr1 = (uint8_t)(msg->addr << 1);
    if ((flags & SOFT_I2C_RD) != 0U) {
      addr1 |= 0x01U;
    }

    ret = soft_i2c_send_address_byte(i2c, addr1, retries);
    if (ret < 0) {
      return ret;
    }
    if ((ret != 1) && !ignore_nack) {
      return SOFT_I2C_EIO;
    }
  }

  return SOFT_I2C_EOK;
}

int32_t soft_i2c_xfer(soft_i2c_t *i2c, soft_i2c_msg_t msgs[], uint32_t num) {
  soft_i2c_msg_t *msg = NULL;
  int32_t ret = SOFT_I2C_EOK;
  uint32_t msg_index = 0U;
  bool transfer_active = false;

  if (i2c == NULL) {
    return SOFT_I2C_ERROR;
  }

  if (num == 0U) {
    return 0;
  }

  if ((msgs == NULL) || !i2c->init_flag) {
    return SOFT_I2C_ERROR;
  }

  for (msg_index = 0U; msg_index < num; msg_index++) {
    msg = &msgs[msg_index];

    if ((msg->len > 0U) && (msg->buf == NULL)) {
      ret = SOFT_I2C_ERROR;
      goto out;
    }

    transfer_active = true;

    if ((msg->flags & SOFT_I2C_NO_START) == 0U) {
      if (msg_index > 0U) {
        ret = soft_i2c_restart(i2c);
        if (ret != SOFT_I2C_EOK) {
          goto out;
        }
      } else {
        SOFT_I2C_LOG_D("send start condition");
        soft_i2c_start(i2c);
      }

      ret = soft_i2c_send_slave_address(i2c, msg);
      if (ret != SOFT_I2C_EOK) {
        SOFT_I2C_LOG_D("send address failed for device 0x%02x at msg %lu",
                       (unsigned int)msgs[msg_index].addr,
                       (unsigned long)msg_index);
        goto out;
      }
    }

    if ((msg->flags & SOFT_I2C_RD) != 0U) {
      ret = soft_i2c_recv_bytes(i2c, msg);
      if (ret >= 1) {
        SOFT_I2C_LOG_D("read %ld byte%s", (long)ret, ret == 1 ? "" : "s");
      }
      if (ret < 0) {
        goto out;
      }
      if ((uint32_t)ret < msg->len) {
        ret = SOFT_I2C_EIO;
        goto out;
      }
    } else {
      ret = soft_i2c_send_bytes(i2c, msg);
      if (ret >= 1) {
        SOFT_I2C_LOG_D("write %ld byte%s", (long)ret, ret == 1 ? "" : "s");
      }
      if (ret < 0) {
        goto out;
      }
      if ((uint32_t)ret < msg->len) {
        ret = SOFT_I2C_ERROR;
        goto out;
      }
    }
  }

  ret = (int32_t)msg_index;

out:
  if (transfer_active && (msg != NULL) &&
      ((msg->flags & SOFT_I2C_NO_STOP) == 0U)) {
    int32_t stop_ret = SOFT_I2C_EOK;

    SOFT_I2C_LOG_D("send stop condition");
    stop_ret = soft_i2c_stop(i2c);
    if ((ret >= 0) && (stop_ret < 0)) {
      ret = stop_ret;
    }
  }

  return ret;
}

int32_t soft_i2c_init(soft_i2c_t *i2c) {
  int32_t ret = SOFT_I2C_EOK;

  if (i2c == NULL) {
    return SOFT_I2C_ERROR;
  }

  if (i2c->init_flag) {
    return SOFT_I2C_EOK;
  }

  ret = soft_i2c_prepare_context(i2c);
  if (ret != SOFT_I2C_EOK) {
    return ret;
  }

  if (i2c->io.pin_init != NULL) {
    ret = i2c->io.pin_init(i2c->user_data);
    if (ret != SOFT_I2C_EOK) {
      return ret;
    }
  }

  SOFT_I2C_SDA_HIGH(i2c);
  ret = soft_i2c_release_scl(i2c);
  if (ret != SOFT_I2C_EOK) {
    if (i2c->io.pin_deinit != NULL) {
      i2c->io.pin_deinit(i2c->user_data);
    }
    return ret;
  }

  soft_i2c_delay_full(i2c);
  i2c->init_flag = true;

  return SOFT_I2C_EOK;
}

void soft_i2c_deinit(soft_i2c_t *i2c) {
  if ((i2c == NULL) || !i2c->init_flag) {
    return;
  }

  SOFT_I2C_SDA_HIGH(i2c);
  (void)soft_i2c_release_scl(i2c);
  soft_i2c_delay_full(i2c);

  if (i2c->io.pin_deinit != NULL) {
    i2c->io.pin_deinit(i2c->user_data);
  }

  i2c->init_flag = false;
}
