/****************************************************************************
 * arch/xtensa/src/esp32s3/esp32s3_usbserial.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <string.h>
#include <assert.h>
#include <debug.h>

#ifdef CONFIG_SERIAL_TERMIOS
#  include <termios.h>
#endif

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/kmalloc.h>
#include <nuttx/init.h>
#include <nuttx/serial/serial.h>
#include <nuttx/serial/tioctl.h>
#include <arch/irq.h>

#include "xtensa.h"
#include "hardware/esp32s3_soc.h"
#include "hardware/esp32s3_system.h"
#include "hardware/esp32s3_usb_serial_jtag.h"

#include "esp32s3_config.h"
#include "esp32s3_irq.h"

/****************************************************************************
 * Pre-processor Macros
 ****************************************************************************/

/* The hardware TX FIFO accepts up to 64 bytes per WR_DONE.  A full 64-byte
 * USB bulk packet may be held by the host until the next packet; stay at 63.
 */

#define ESP32S3_USBCDC_BUFFERSIZE   256
#define ESP32S3_USB_TX_CHUNK        63
#define ESP32S3_USB_TX_WAIT_RETRIES 40000

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct esp32s3_priv_s
{
  const uint8_t  periph;        /* peripheral ID */
  const uint8_t  irq;           /* IRQ number assigned to the peripheral */
  int            cpu;           /* CPU id */
  int            cpuint;        /* CPU interrupt assigned */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int esp32s3_interrupt(int irq, void *context, void *arg);

/* Serial driver methods */

static int  esp32s3_setup(struct uart_dev_s *dev);
static void esp32s3_shutdown(struct uart_dev_s *dev);
static int  esp32s3_attach(struct uart_dev_s *dev);
static void esp32s3_detach(struct uart_dev_s *dev);
static void esp32s3_txint(struct uart_dev_s *dev, bool enable);
static void esp32s3_rxint(struct uart_dev_s *dev, bool enable);
static bool esp32s3_rxavailable(struct uart_dev_s *dev);
static bool esp32s3_txready(struct uart_dev_s *dev);
static bool esp32s3_txempty(struct uart_dev_s *dev);
static void esp32s3_send(struct uart_dev_s *dev, int ch);
static int  esp32s3_receive(struct uart_dev_s *dev, unsigned int *status);
static int  esp32s3_ioctl(struct file *filep, int cmd, unsigned long arg);

static bool esp32s3_usb_hw_tx_free(void);
static void esp32s3_usb_hw_tx_flush_locked(void);
static void esp32s3_usb_hw_tx_queue(uint8_t ch);
static void esp32s3_hw_send_char(int ch);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static char g_rxbuffer[ESP32S3_USBCDC_BUFFERSIZE];
static char g_txbuffer[ESP32S3_USBCDC_BUFFERSIZE];

/* Batch bytes into one USB IN packet instead of WR_DONE per character (~1ms
 * each on full-speed USB, which looks like typewriter output in minicom).
 */

static uint8_t g_hw_txbuf[ESP32S3_USB_TX_CHUNK];
static size_t  g_hw_txlen;

static struct esp32s3_priv_s g_usbserial_priv =
{
  .periph = ESP32S3_PERIPH_USB_DEVICE,
  .irq    = ESP32S3_IRQ_USB_DEVICE,
  .cpu    = 0,
  .cpuint = -ENOMEM,
};

static struct uart_ops_s g_uart_ops =
{
  .setup       = esp32s3_setup,
  .shutdown    = esp32s3_shutdown,
  .attach      = esp32s3_attach,
  .detach      = esp32s3_detach,
  .txint       = esp32s3_txint,
  .rxint       = esp32s3_rxint,
  .rxavailable = esp32s3_rxavailable,
  .txready     = esp32s3_txready,
  .txempty     = esp32s3_txempty,
  .send        = esp32s3_send,
  .receive     = esp32s3_receive,
  .ioctl       = esp32s3_ioctl,
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

uart_dev_t g_uart_usbserial =
{
  .isconsole = true,
  .recv      =
    {
      .size    = ESP32S3_USBCDC_BUFFERSIZE,
      .buffer  = g_rxbuffer,
    },
  .xmit      =
    {
      .size    = ESP32S3_USBCDC_BUFFERSIZE,
      .buffer  = g_txbuffer,
    },
  .ops       = &g_uart_ops,
  .priv      = &g_usbserial_priv,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32s3_interrupt
 ****************************************************************************/

static int esp32s3_interrupt(int irq, void *context, void *arg)
{
  struct uart_dev_s *dev = (struct uart_dev_s *)arg;
  uint32_t regval;

  regval = getreg32(USB_SERIAL_JTAG_INT_ST_REG);

  if (regval & USB_SERIAL_JTAG_SERIAL_IN_EMPTY_INT_ST)
    {
      putreg32(USB_SERIAL_JTAG_SERIAL_IN_EMPTY_INT_CLR,
               USB_SERIAL_JTAG_INT_CLR_REG);
      uart_xmitchars(dev);
    }

  if (regval & USB_SERIAL_JTAG_SERIAL_OUT_RECV_PKT_INT_ST)
    {
      putreg32(USB_SERIAL_JTAG_SERIAL_OUT_RECV_PKT_INT_CLR,
               USB_SERIAL_JTAG_INT_CLR_REG);
      uart_recvchars(dev);
    }

  return OK;
}

static int esp32s3_setup(struct uart_dev_s *dev)
{
  (void)dev;

  modifyreg32(SYSTEM_PERIP_CLK_EN0_REG, 0, SYSTEM_USB_DEVICE_CLK_EN);
  modifyreg32(SYSTEM_PERIP_RST_EN0_REG, SYSTEM_USB_DEVICE_RST, 0);
  modifyreg32(USB_SERIAL_JTAG_CONF0_REG, 0, USB_SERIAL_JTAG_USB_PAD_ENABLE);
  modifyreg32(USB_SERIAL_JTAG_MEM_CONF_REG, USB_SERIAL_JTAG_USB_MEM_PD,
              USB_SERIAL_JTAG_USB_MEM_CLK_EN);
  putreg32(UINT32_MAX, USB_SERIAL_JTAG_INT_CLR_REG);
  return OK;
}

static void esp32s3_shutdown(struct uart_dev_s *dev)
{
  UNUSED(dev);
}

static void esp32s3_txint(struct uart_dev_s *dev, bool enable)
{
  irqstate_t flags = enter_critical_section();

  if (enable)
    {
      modifyreg32(USB_SERIAL_JTAG_INT_ENA_REG, 0,
                  USB_SERIAL_JTAG_SERIAL_IN_EMPTY_INT_ENA);

      if (esp32s3_txready(dev))
        {
          uart_xmitchars(dev);
        }
    }
  else
    {
      modifyreg32(USB_SERIAL_JTAG_INT_ENA_REG,
                  USB_SERIAL_JTAG_SERIAL_IN_EMPTY_INT_ENA, 0);
    }

  leave_critical_section(flags);
}

static void esp32s3_rxint(struct uart_dev_s *dev, bool enable)
{
  UNUSED(dev);

  if (enable)
    {
      modifyreg32(USB_SERIAL_JTAG_INT_ENA_REG, 0,
                  USB_SERIAL_JTAG_SERIAL_OUT_RECV_PKT_INT_ENA);
    }
  else
    {
      modifyreg32(USB_SERIAL_JTAG_INT_ENA_REG,
                  USB_SERIAL_JTAG_SERIAL_OUT_RECV_PKT_INT_ENA, 0);
    }
}

static int esp32s3_attach(struct uart_dev_s *dev)
{
  struct esp32s3_priv_s *priv = dev->priv;
  int ret;

  DEBUGASSERT(priv->cpuint == -ENOMEM);

  priv->cpu = up_cpu_index();
  priv->cpuint = esp32s3_setup_irq(priv->cpu, priv->periph,
                                   ESP32S3_INT_PRIO_DEF,
                                   ESP32S3_CPUINT_LEVEL);
  if (priv->cpuint < 0)
    {
      return priv->cpuint;
    }

  ret = irq_attach(priv->irq, esp32s3_interrupt, dev);
  if (ret == OK)
    {
      up_enable_irq(priv->irq);
    }

  return ret;
}

static void esp32s3_detach(struct uart_dev_s *dev)
{
  struct esp32s3_priv_s *priv = dev->priv;

  DEBUGASSERT(priv->cpuint != -ENOMEM);

  up_disable_irq(priv->irq);
  irq_detach(priv->irq);
  esp32s3_teardown_irq(priv->cpu, priv->periph, priv->cpuint);

  priv->cpuint = -ENOMEM;
}

static bool esp32s3_rxavailable(struct uart_dev_s *dev)
{
  uint32_t regval;

  UNUSED(dev);

  regval = getreg32(USB_SERIAL_JTAG_EP1_CONF_REG);

  return regval & USB_SERIAL_JTAG_SERIAL_OUT_EP_DATA_AVAIL;
}

static bool esp32s3_txempty(struct uart_dev_s *dev)
{
  uint32_t retries = ESP32S3_USB_TX_WAIT_RETRIES;
  irqstate_t flags;

  UNUSED(dev);

  flags = enter_critical_section();
  esp32s3_usb_hw_tx_flush_locked();

  while (!esp32s3_usb_hw_tx_free())
    {
      if (retries-- == 0)
        {
          leave_critical_section(flags);
          return false;
        }
    }

  leave_critical_section(flags);
  return true;
}

static bool esp32s3_txready(struct uart_dev_s *dev)
{
  UNUSED(dev);

  if (g_hw_txlen < ESP32S3_USB_TX_CHUNK)
    {
      return true;
    }

  return esp32s3_usb_hw_tx_free();
}

static bool esp32s3_usb_hw_tx_free(void)
{
  return (getreg32(USB_SERIAL_JTAG_EP1_CONF_REG) &
          USB_SERIAL_JTAG_SERIAL_IN_EP_DATA_FREE) != 0;
}

static void esp32s3_usb_hw_tx_flush_locked(void)
{
  size_t i;
  uint32_t retries = ESP32S3_USB_TX_WAIT_RETRIES;

  if (g_hw_txlen == 0)
    {
      return;
    }

  while (!esp32s3_usb_hw_tx_free())
    {
      if (retries-- == 0)
        {
          g_hw_txlen = 0;
          return;
        }
    }

  for (i = 0; i < g_hw_txlen; i++)
    {
      putreg32(g_hw_txbuf[i], USB_SERIAL_JTAG_EP1_REG);
    }

  putreg32(USB_SERIAL_JTAG_WR_DONE, USB_SERIAL_JTAG_EP1_CONF_REG);
  g_hw_txlen = 0;
}

static void esp32s3_usb_hw_tx_queue(uint8_t ch)
{
  if (g_hw_txlen >= ESP32S3_USB_TX_CHUNK)
    {
      esp32s3_usb_hw_tx_flush_locked();
    }

  g_hw_txbuf[g_hw_txlen++] = ch;

  if (g_hw_txlen >= ESP32S3_USB_TX_CHUNK)
    {
      esp32s3_usb_hw_tx_flush_locked();
    }
}

static void esp32s3_hw_send_char(int ch)
{
  irqstate_t flags = enter_critical_section();

  esp32s3_usb_hw_tx_queue((uint8_t)ch);

  if (ch == '\n' || ch == '\r')
    {
      esp32s3_usb_hw_tx_flush_locked();
    }

  leave_critical_section(flags);
}

static void esp32s3_send(struct uart_dev_s *dev, int ch)
{
  int nexttail = dev->xmit.tail + 1;
  irqstate_t flags;

  if (nexttail >= dev->xmit.size)
    {
      nexttail = 0;
    }

  flags = enter_critical_section();
  esp32s3_usb_hw_tx_queue((uint8_t)ch);

  if (g_hw_txlen > 0 && nexttail == dev->xmit.head)
    {
      esp32s3_usb_hw_tx_flush_locked();
    }

  leave_critical_section(flags);
}

static int esp32s3_receive(struct uart_dev_s *dev, unsigned int *status)
{
  UNUSED(dev);

  *status = 0;
  return getreg32(USB_SERIAL_JTAG_EP1_REG) & USB_SERIAL_JTAG_RDWR_BYTE;
}

static int esp32s3_ioctl(struct file *filep, int cmd, unsigned long arg)
{
#if defined(CONFIG_SERIAL_TERMIOS)
  struct inode      *inode = filep->f_inode;
  struct uart_dev_s *dev   = inode->i_private;
#else
  UNUSED(filep);
#endif
  int                ret   = OK;

  switch (cmd)
    {
#ifdef CONFIG_SERIAL_TERMIOS
    case TCGETS:
      {
        struct termios *termiosp = (struct termios *)arg;

        if (!termiosp)
          {
            ret = -EINVAL;
          }
        else
          {
            termiosp->c_cflag = CS8;
            cfsetispeed(termiosp, B115200);
            cfsetospeed(termiosp, B115200);
          }
      }
      break;

    case TCSETS:
      ret = -ENOTTY;
      break;
#endif /* CONFIG_SERIAL_TERMIOS */

    default:
      ret = -ENOTTY;
      break;
    }

  UNUSED(dev);
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void esp32s3_usbserial_write(char ch)
{
  esp32s3_hw_send_char(ch);
}
