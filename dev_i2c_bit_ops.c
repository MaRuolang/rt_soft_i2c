/*
 * Copyright (c)
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author        Notes
 * 2026-03-12     Maruolang     first version
 */

#include "dev_i2c_bit_ops.h"

#include <stddef.h>

#define LOG_E_ENABLE 1
#define LOG_W_ENABLE 1
#define LOG_D_ENABLE 1

#if (LOG_E_ENABLE || LOG_W_ENABLE || LOG_D_ENABLE)
#include <stdio.h>
#define RT_I2C_BITOPS_DEBUG
#endif

#if LOG_E_ENABLE
#define LOG_E(...) ((void)printf("[E] " __VA_ARGS__))
#else
#define LOG_E(...) ((void)0)
#endif

#if LOG_W_ENABLE
#define LOG_W(...) ((void)printf("[W] " __VA_ARGS__))
#else
#define LOG_W(...) ((void)0)
#endif

#if LOG_D_ENABLE
#define LOG_D(...) ((void)printf("[D] " __VA_ARGS__))
#else
#define LOG_D(...) ((void)0)
#endif

#define SET_SDA(ops, val) ops->set_sda(ops->data, val)
#define SET_SCL(ops, val) ops->set_scl(ops->data, val)
#define GET_SDA(ops) ops->get_sda(ops->data)
#define GET_SCL(ops) ops->get_scl(ops->data)

#define GET_TICK(ops) ((ops)->tick_get())

static inline void i2c_delay(struct rt_i2c_bit_ops *ops) {
  ops->udelay((ops->delay_us + 1) >> 1);
}

static inline void i2c_delay2(struct rt_i2c_bit_ops *ops) {
  ops->udelay(ops->delay_us);
}

#define SDA_L(ops) SET_SDA(ops, 0)
#define SDA_H(ops) SET_SDA(ops, 1)
#define SCL_L(ops) SET_SCL(ops, 0)

/**
 * release scl line, and wait scl line to high.
 */
static rt_err_t SCL_H(struct rt_i2c_bit_ops *ops) {
  uint32_t start;

  SET_SCL(ops, 1);

  if (!ops->get_scl)
    goto done;

  start = GET_TICK(ops);
  while (!GET_SCL(ops)) {
    if ((GET_TICK(ops) - start) > ops->timeout)
      return -RT_ETIMEOUT;
    i2c_delay(ops);
  }
#ifdef RT_I2C_BITOPS_DEBUG
  if (GET_TICK(ops) != start) {
    LOG_D("wait %u tick for SCL line to go high", GET_TICK(ops) - start);
  }
#endif

done:
  i2c_delay(ops);

  return RT_EOK;
}

static void i2c_start(struct rt_i2c_bit_ops *ops) {
#ifdef RT_I2C_BITOPS_DEBUG
  if (ops->get_scl && !GET_SCL(ops)) {
    LOG_E("I2C bus error, SCL line low");
  }
  if (ops->get_sda && !GET_SDA(ops)) {
    LOG_E("I2C bus error, SDA line low");
  }
#endif
  SDA_L(ops);
  i2c_delay(ops);
  SCL_L(ops);
}

static void i2c_restart(struct rt_i2c_bit_ops *ops) {
  SDA_H(ops);
  SCL_H(ops);
  i2c_delay(ops);
  SDA_L(ops);
  i2c_delay(ops);
  SCL_L(ops);
}

static void i2c_stop(struct rt_i2c_bit_ops *ops) {
  SDA_L(ops);
  i2c_delay(ops);
  SCL_H(ops);
  i2c_delay(ops);
  SDA_H(ops);
  i2c_delay2(ops);
}

static inline rt_bool_t i2c_waitack(struct rt_i2c_bit_ops *ops) {
  rt_bool_t ack;

  SDA_H(ops);
  i2c_delay(ops);

  if (SCL_H(ops) < 0) {
    LOG_W("wait ack timeout");

    return -RT_ETIMEOUT;
  }

  ack = !GET_SDA(ops); /* ACK : SDA pin is pulled low */
  LOG_D("%s", ack ? "ACK" : "NACK");

  SCL_L(ops);

  return ack;
}

static int32_t i2c_writeb(struct rt_i2c_bit_ops *ops, uint8_t data) {
  int32_t i;
  uint8_t bit;

  for (i = 7; i >= 0; i--) {
    SCL_L(ops);
    bit = (data >> i) & 1;
    SET_SDA(ops, bit);
    i2c_delay(ops);
    if (SCL_H(ops) < 0) {
      LOG_D("i2c_writeb: 0x%02x, "
            "wait scl pin high timeout at bit %d",
            data, i);

      return -RT_ETIMEOUT;
    }
  }
  SCL_L(ops);
  i2c_delay(ops);

  return i2c_waitack(ops);
}

static int32_t i2c_readb(struct rt_i2c_bit_ops *ops) {
  uint8_t i;
  uint8_t data = 0;

  SDA_H(ops);
  i2c_delay(ops);
  for (i = 0; i < 8; i++) {
    data <<= 1;

    if (SCL_H(ops) < 0) {
      LOG_D("i2c_readb: wait scl pin high "
            "timeout at bit %d",
            7 - i);

      return -RT_ETIMEOUT;
    }

    if (GET_SDA(ops))
      data |= 1;
    SCL_L(ops);
    i2c_delay2(ops);
  }

  return data;
}

static int32_t i2c_send_bytes(struct rt_i2c_bit_ops *ops,
                              struct rt_i2c_msg *msg) {
  int32_t ret;
  uint32_t bytes = 0;
  const uint8_t *ptr = msg->buf;
  int32_t count = msg->len;
  uint16_t ignore_nack = msg->flags & RT_I2C_IGNORE_NACK;

  while (count > 0) {
    ret = i2c_writeb(ops, *ptr);

    if ((ret > 0) || (ignore_nack && (ret == 0))) {
      count--;
      ptr++;
      bytes++;
    } else if (ret == 0) {
      LOG_D("send bytes: NACK.");

      return 0;
    } else {
      LOG_E("send bytes: error %d", ret);

      return ret;
    }
  }

  return bytes;
}

static rt_err_t i2c_send_ack_or_nack(struct rt_i2c_bit_ops *ops, int ack) {
  if (ack)
    SET_SDA(ops, 0);
  i2c_delay(ops);
  if (SCL_H(ops) < 0) {
    LOG_E("ACK or NACK timeout.");

    return -RT_ETIMEOUT;
  }
  SCL_L(ops);

  return RT_EOK;
}

static int32_t i2c_recv_bytes(struct rt_i2c_bit_ops *ops,
                              struct rt_i2c_msg *msg) {
  int32_t val;
  int32_t bytes = 0; /* actual bytes */
  uint8_t *ptr = msg->buf;
  int32_t count = msg->len;
  const uint32_t flags = msg->flags;

  while (count > 0) {
    val = i2c_readb(ops);
    if (val >= 0) {
      *ptr = val;
      bytes++;
    } else {
      break;
    }

    ptr++;
    count--;

    LOG_D("receive bytes: 0x%02x, %s", val,
          (flags & RT_I2C_NO_READ_ACK) ? "(No ACK/NACK)"
                                       : (count ? "ACK" : "NACK"));

    if (!(flags & RT_I2C_NO_READ_ACK)) {
      val = i2c_send_ack_or_nack(ops, count);
      if (val < 0)
        return val;
    }
  }

  return bytes;
}

static int32_t i2c_send_address(struct rt_i2c_bit_ops *ops, uint8_t addr,
                                int32_t retries) {
  int32_t i;
  rt_err_t ret = 0;

  for (i = 0; i <= retries; i++) {
    ret = i2c_writeb(ops, addr);
    if (ret == 1 || i == retries)
      break;
    LOG_D("send stop condition");
    i2c_stop(ops);
    i2c_delay2(ops);
    LOG_D("send start condition");
    i2c_start(ops);
  }

  return ret;
}

static rt_err_t i2c_bit_send_address(struct rt_i2c_bit_ops *ops,
                                     struct rt_i2c_msg *msg) {
  uint16_t flags = msg->flags;
  uint16_t ignore_nack = msg->flags & RT_I2C_IGNORE_NACK;

  uint8_t addr1, addr2;
  int32_t retries;
  rt_err_t ret;

  retries = ignore_nack ? 0 : ops->retries;

  if (flags & RT_I2C_ADDR_10BIT) {
    addr1 = 0xf0 | ((msg->addr >> 7) & 0x06);
    addr2 = msg->addr & 0xff;

    LOG_D("addr1: %d, addr2: %d", addr1, addr2);

    ret = i2c_send_address(ops, addr1, retries);
    if ((ret != 1) && !ignore_nack) {
      LOG_W("NACK: sending first addr");

      return -RT_EIO;
    }

    ret = i2c_writeb(ops, addr2);
    if ((ret != 1) && !ignore_nack) {
      LOG_W("NACK: sending second addr");

      return -RT_EIO;
    }
    if (flags & RT_I2C_RD) {
      LOG_D("send repeated start condition");
      i2c_restart(ops);
      addr1 |= 0x01;
      ret = i2c_send_address(ops, addr1, retries);
      if ((ret != 1) && !ignore_nack) {
        LOG_E("NACK: sending repeated addr");

        return -RT_EIO;
      }
    }
  } else {
    /* 7-bit addr */
    addr1 = msg->addr << 1;
    if (flags & RT_I2C_RD)
      addr1 |= 1;
    ret = i2c_send_address(ops, addr1, retries);
    if ((ret != 1) && !ignore_nack)
      return -RT_EIO;
  }

  return RT_EOK;
}

int32_t i2c_bit_xfer(struct rt_i2c_bit_ops *ops, struct rt_i2c_msg msgs[],
                     uint32_t num) {
  struct rt_i2c_msg *msg;
  int32_t ret;
  uint32_t i;
  uint16_t ignore_nack;

  if ((ops->i2c_pin_init_flag == RT_FALSE) && (ops->pin_init != NULL)) {
    ops->pin_init();
    ops->i2c_pin_init_flag = RT_TRUE;
  }

  if (num == 0)
    return 0;

  for (i = 0; i < num; i++) {
    msg = &msgs[i];
    ignore_nack = msg->flags & RT_I2C_IGNORE_NACK;
    if (!(msg->flags & RT_I2C_NO_START)) {
      if (i) {
        i2c_restart(ops);
      } else {
        LOG_D("send start condition");
        i2c_start(ops);
      }
      ret = i2c_bit_send_address(ops, msg);
      if ((ret != RT_EOK) && !ignore_nack) {
        LOG_D("receive NACK from device addr 0x%02x msg %u", msgs[i].addr, i);
        goto out;
      }
    }
    if (msg->flags & RT_I2C_RD) {
      ret = i2c_recv_bytes(ops, msg);
      if (ret >= 1) {
        LOG_D("read %d byte%s", ret, ret == 1 ? "" : "s");
      }
      if (ret < msg->len) {
        if (ret >= 0)
          ret = -RT_EIO;
        goto out;
      }
    } else {
      ret = i2c_send_bytes(ops, msg);
      if (ret >= 1) {
        LOG_D("write %d byte%s", ret, ret == 1 ? "" : "s");
      }
      if (ret < msg->len) {
        if (ret >= 0)
          ret = -RT_ERROR;
        goto out;
      }
    }
  }
  ret = i;

out:
  if (!(msg->flags & RT_I2C_NO_STOP)) {
    LOG_D("send stop condition");
    i2c_stop(ops);
  }

  return ret;
}
