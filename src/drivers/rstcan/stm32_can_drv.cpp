#include <nuttx/config.h>

#include <stdio.h>
#include <string.h>
#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <semaphore.h>
#include <errno.h>
#include <debug.h>

#include <arch/board/board.h>
#include <nuttx/arch.h>
#include <nuttx/can/can.h>

#include "up_internal.h"
#include "up_arch.h"

//#include "os_internal.h"

#include "chip.h"
#include "stm32.h"
#include "stm32_can.h"
#include "stm32_can_drv.h"
#include "rstcan_manager.h"

#define _INAK_TIMEOUT 65535
/* Mailboxes ****************************************************************/

#define _ALL_MAILBOXES  (CAN_TSR_TME0 | CAN_TSR_TME1 | CAN_TSR_TME2)

/* Bit timing ***************************************************************/


#ifndef CONFIG_CAN_TSEG1
#define CONFIG_CAN_TSEG1 6
#endif

#ifndef CONFIG_CAN_TSEG2
#define CONFIG_CAN_TSEG2 7
#endif

#define _BIT_QUANTA (CONFIG_CAN_TSEG1 + CONFIG_CAN_TSEG2 + 1)

#define _MSGLEN(nbytes)        (sizeof(struct rstcan_frame) - RSTCAN_FRAME_MAX_DATA + (nbytes))

static struct can_describe g_can1desc =
{
  .port             = 1,
  .rx0_irq          = STM32_IRQ_CAN1RX0,
  .tx_irq           = STM32_IRQ_CAN1TX,
  .base_addr        = STM32_CAN1_BASE,
  .filter           = 0,
  .baudrate         = CONFIG_CAN1_BAUD,
  .tx_sem           = {0},
  .tx_waiter        = 0,
  .tx_buf_size      = CAN_DRV_PORT_BUF_SIZE,
  .tx_buf_head      = 0,
  .tx_buf_tail      = 0,
};

#if (RSTCAN_PORT_NUM == 2)
static struct can_describe g_can2desc =
{
  .port             = 2,
  .rx0_irq          = STM32_IRQ_CAN2RX0,
  .tx_irq           = STM32_IRQ_CAN2TX,
  .base_addr        = STM32_CAN2_BASE,
  .filter           = 14,//can2的filter编号从14开始
  .baudrate         = CONFIG_CAN2_BAUD,
  .tx_sem           = {0},
  .tx_waiter        = 0,
  .tx_buf_size      = CAN_DRV_PORT_BUF_SIZE,
  .tx_buf_head      = 0,
  .tx_buf_tail      = 0,
};
#endif

#if (RSTCAN_PORT_NUM == 2)
struct can_describe *p_can_desc[RSTCAN_PORT_NUM]={&g_can1desc, &g_can2desc};
#else
struct can_describe *p_can_desc[RSTCAN_PORT_NUM]={&g_can1desc};
#endif

/****************************************************************************
 * Name: _bittiming
 *
 * Description:
 *   Set the CAN bit timing register (BTR) based on the configured BAUD.
 *
 * "The bit timing logic monitors the serial bus-line and performs sampling
 *  and adjustment of the sample point by synchronizing on the start-bit edge
 *  and resynchronizing on the following edges.
 *
 * "Its operation may be explained simply by splitting nominal bit time into
 *  three segments as follows:
 *
 * 1. "Synchronization segment (SYNC_SEG): a bit change is expected to occur
 *     within this time segment. It has a fixed length of one time quantum
 *     (1 x tCAN).
 * 2. "Bit segment 1 (BS1): defines the location of the sample point. It
 *     includes the PROP_SEG and PHASE_SEG1 of the CAN standard. Its duration
 *     is programmable between 1 and 16 time quanta but may be automatically
 *     lengthened to compensate for positive phase drifts due to differences
 *     in the frequency of the various nodes of the network.
 * 3. "Bit segment 2 (BS2): defines the location of the transmit point. It
 *     represents the PHASE_SEG2 of the CAN standard. Its duration is
 *     programmable between 1 and 8 time quanta but may also be automatically
 *     shortened to compensate for negative phase drifts."
 *
 * Pictorially:
 *
 *  |<----------------- NOMINAL BIT TIME ----------------->|
 *  |<- SYNC_SEG ->|<------ BS1 ------>|<------ BS2 ------>|
 *  |<---- Tq ---->|<----- Tbs1 ------>|<----- Tbs2 ------>|
 *
 * Where
 *   Tbs1 is the duration of the BS1 segment
 *   Tbs2 is the duration of the BS2 segment
 *   Tq is the "Time Quantum"
 *
 * Relationships:
 *
 *   baud = 1 / bit_time
 *   bit_time = Tq + Tbs1 + Tbs2
 *   Tbs1 = Tq * ts1
 *   Tbs2 = Tq * ts2
 *   Tq = brp * Tpclk1
 *
 * Where:
 *   Tpclk1 is the period of the APB1 clock (PCLK1).
 *
 * Input Parameter:
 *   priv - A reference to the CAN block status
 *
 * Returned Value:
 *   Zero on success; a negated errno on failure
 *
 ****************************************************************************/
static int _bittiming(struct can_describe *priv)
{
  uint32_t tmp;
  uint32_t brp;
  uint32_t ts1;
  uint32_t ts2;

//  printf("CAN%d PCLK1: %d baud: %d\n",
  //          priv->port, STM32_PCLK1_FREQUENCY, priv->baudrate);

  /* Try to get _BIT_QUANTA quanta in one bit_time.
   *
   *   bit_time = Tq*(ts1 + ts2 + 1)
   *   nquanta  = bit_time / Tq
   *   nquanta  = (ts1 + ts2 + 1)
   *
   *   bit_time = brp * Tpclk1 * (ts1 + ts2 + 1)
   *   nquanta  = bit_time / brp / Tpclk1
   *            = PCLK1 / baud / brp
   *   brp      = PCLK1 / baud / nquanta;
   *
   * Example:
   *   PCLK1 = 42,000,000 baud = 1,000,000 nquanta = 14 : brp = 3
   *   PCLK1 = 42,000,000 baud =   700,000 nquanta = 14 : brp = 4
   */

  tmp = STM32_PCLK1_FREQUENCY / priv->baudrate;
  if (tmp < _BIT_QUANTA)
    {
      /* At the smallest brp value (1), there are already too few bit times
       * (PCLCK1 / baud) to meet our goal.  brp must be one and we need
       * make some reasonable guesses about ts1 and ts2.
       */

      brp = 1;

      /* In this case, we have to guess a good value for ts1 and ts2 */

      ts1 = (tmp - 1) >> 1;
      ts2 = tmp - ts1 - 1;
      if (ts1 == ts2 && ts1 > 1 && ts2 < CAN_BTR_TSEG2_MAX)
        {
          ts1--;
          ts2++;          
        }
    }

  /* Otherwise, nquanta is _BIT_QUANTA, ts1 is CONFIG_CAN_TSEG1, ts2 is
   * CONFIG_CAN_TSEG2 and we calculate brp to achieve _BIT_QUANTA quanta
   * in the bit time
   */

  else
    {
      ts1 = CONFIG_CAN_TSEG1;
      ts2 = CONFIG_CAN_TSEG2;
      brp = (tmp + (_BIT_QUANTA/2)) / _BIT_QUANTA;
      DEBUGASSERT(brp >=1 && brp <= CAN_BTR_BRP_MAX);
    }

//  printf("TS1: %d TS2: %d BRP: %d\n", ts1, ts2, brp);

 /* Configure bit timing.  This also does the the following, less obvious
   * things.  Unless loopback mode is enabled, it:
   *
   * - Disables silent mode.
   * - Disables loopback mode.
   *
   * NOTE that for the time being, SJW is set to 1 just because I don't
   * know any better.
   */

  tmp = ((brp - 1) << CAN_BTR_BRP_SHIFT) | ((ts1 - 1) << CAN_BTR_TS1_SHIFT) |
        ((ts2 - 1) << CAN_BTR_TS2_SHIFT) | ((1 - 1) << CAN_BTR_SJW_SHIFT);
#ifdef CONFIG_CAN_LOOPBACK
//tmp |= (CAN_BTR_LBKM | CAN_BTR_SILM);
  tmp |= CAN_BTR_LBKM;
#endif

  _writereg(priv, STM32_CAN_BTR_OFFSET, tmp);
  return OK;
}
/****************************************************************************
 * Name: _cellinit
 *
 * Description:
 *   CAN cell initialization
 *
 * Input Parameter:
 *   priv - A pointer to the private data structure for this CAN block
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/
static int _cellinit(struct can_describe *priv)
{
    volatile uint32_t timeout;
    uint32_t regval;
    int ret;

//    printf("CAN%d\n", priv->port);

    /* Exit from sleep mode */

    regval  = _readreg(priv, STM32_CAN_MCR_OFFSET);
    regval &= ~CAN_MCR_SLEEP;
    _writereg(priv, STM32_CAN_MCR_OFFSET, regval);

    /* Configure CAN behavior.  Priority driven request order, not message ID. */

    regval |= CAN_MCR_TXFP;
    _writereg(priv, STM32_CAN_MCR_OFFSET, regval);

    /* Enter initialization mode */

    regval |= CAN_MCR_INRQ;
    _writereg(priv, STM32_CAN_MCR_OFFSET, regval);
    //printf("base_addr=%x write=%x inrq=%x\n", priv->base_addr, regval, MEM_REG(priv->base_addr+STM32_CAN_MCR_OFFSET)/*_readreg(priv, STM32_CAN_MCR_OFFSET)*/);

    /* Wait until initialization mode is acknowledged */

    for (timeout = _INAK_TIMEOUT; timeout > 0; timeout--)
    {
      regval = _readreg(priv, STM32_CAN_MSR_OFFSET);
      if ((regval & CAN_MSR_INAK) != 0)
        {
          /* We are in initialization mode */

          break;
        }
    }

    /* Check for a timeout */

    if (timeout < 1)
    {
      printf("ERROR: Timed out waiting to enter initialization mode\n");
      return -ETIMEDOUT;
    }

    /* Disable the following modes:
    *
    *  - Time triggered communication mode
    *  - Automatic bus-off management
    *  - Automatic wake-up mode
    *  - No automatic retransmission
    *  - Receive FIFO locked mode
    *  - Transmit FIFO priority
    */

    regval   = _readreg(priv, STM32_CAN_MCR_OFFSET);
    regval &= ~(CAN_MCR_TXFP | CAN_MCR_RFLM | CAN_MCR_NART | CAN_MCR_AWUM | CAN_MCR_ABOM | CAN_MCR_TTCM);
    _writereg(priv, STM32_CAN_MCR_OFFSET, regval);

    /* Configure bit timing. */

    ret = _bittiming(priv);
    if (ret < 0)
    {
      printf("ERROR: Failed to set bit timing: %d\n", ret);
      return ret;
    }

    /* Exit initialization mode */

    regval  = _readreg(priv, STM32_CAN_MCR_OFFSET);
    regval &= ~CAN_MCR_INRQ;
    _writereg(priv, STM32_CAN_MCR_OFFSET, regval);

    /* Wait until the initialization mode exit is acknowledged */

    for (timeout = _INAK_TIMEOUT; timeout > 0; timeout--)
    {
      regval = _readreg(priv, STM32_CAN_MSR_OFFSET);
      if ((regval & CAN_MSR_INAK) == 0)
        {
          /* We are out of initialization mode */

          break;
        }
    }

    /* Check for a timeout */

    if (timeout < 1)
    {
      printf("ERROR: Timed out waiting to exit initialization mode: %08x\n", regval);
      return -ETIMEDOUT;
    }
    return OK;


}

/****************************************************************************
 * Name: _filterinit
 *
 * Description:
 *   CAN filter initialization.  CAN filters are not currently used by this
 *   driver.  The CAN filters can be configured in a different way:
 *
 *   1. As a match of specific IDs in a list (IdList mode), or as
 *   2. And ID and a mask (IdMask mode).
 *
 *   Filters can also be configured as:
 *
 *   3. 16- or 32-bit.  The advantage of 16-bit filters is that you get
 *      more filters;  The advantage of 32-bit filters is that you get 
 *      finer control of the filtering.
 *
 *   One filter is set up for each CAN.  The filter resources are shared
 *   between the two CAN modules:  CAN1 uses only filter 0 (but reserves
 *   0 through CAN_NFILTERS/2-1); CAN2 uses only filter CAN_NFILTERS/2
 *   (but reserves CAN_NFILTERS/2 through CAN_NFILTERS-1).
 *
 *   32-bit IdMask mode is configured.  However, both the ID and the MASK
 *   are set to zero thus supressing all filtering because anything masked
 *   with zero matches zero.
 *
 * Input Parameter:
 *   priv - A pointer to the private data structure for this CAN block
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int _filterinit(struct can_describe *priv)
{
  uint32_t regval;
  uint32_t bitmask;
  //由于滤波器是两个can公用，所以只有一个地址
  struct can_describe *priv_can1 = p_can_desc[0];

  if(priv_can1 == NULL)
      return -1;
//  printf("CAN%d filter: %d\n", priv->port, priv->filter);

  /* Get the bitmask associated with the filter used by this CAN block */

  bitmask = ((uint32_t)1) << priv->filter;

  /* Enter filter initialization mode */

  regval  = _readreg(priv_can1, STM32_CAN_FMR_OFFSET);
  regval |= CAN_FMR_FINIT;
  _writereg(priv_can1, STM32_CAN_FMR_OFFSET, regval);

  /* Disable the filter */

  regval  = _readreg(priv_can1, STM32_CAN_FA1R_OFFSET);
  regval &= ~bitmask;
  _writereg(priv_can1, STM32_CAN_FA1R_OFFSET, regval);

  /* Select the 32-bit scale for the filter */

  regval  = _readreg(priv_can1, STM32_CAN_FS1R_OFFSET);
  regval |= bitmask;
  _writereg(priv_can1, STM32_CAN_FS1R_OFFSET, regval);
 
  /* There are 14 or 28 filter banks (depending) on the device.  Each filter bank is
   * composed of two 32-bit registers, CAN_FiR:
   */

  _writereg(priv_can1,  STM32_CAN_FIR_OFFSET(priv->filter, 1), 0);
  _writereg(priv_can1,  STM32_CAN_FIR_OFFSET(priv->filter, 2), 0);

 /* Set Id/Mask mode for the filter */

  regval  = _readreg(priv_can1, STM32_CAN_FM1R_OFFSET);
  regval &= ~bitmask;
  _writereg(priv_can1, STM32_CAN_FM1R_OFFSET, regval);

 /* Assign FIFO 0 for the filter */

  regval  = _readreg(priv_can1, STM32_CAN_FFA1R_OFFSET);
  regval &= ~bitmask;
  _writereg(priv_can1, STM32_CAN_FFA1R_OFFSET, regval);
 
  /* Enable the filter */

  regval  = _readreg(priv_can1, STM32_CAN_FA1R_OFFSET);
  regval |= bitmask;
  _writereg(priv_can1, STM32_CAN_FA1R_OFFSET, regval);

  /* Exit filter initialization mode */

  regval  = _readreg(priv_can1, STM32_CAN_FMR_OFFSET);
  regval &= ~CAN_FMR_FINIT;
  _writereg(priv_can1, STM32_CAN_FMR_OFFSET, regval);
  return OK;
}

static int32_t _recv_frame(struct can_describe *desc, struct rstcan_frame *frame)
{
    RSTCan_Manager::get_instance(desc->port-1)->can_rx(frame);

    return 0;
}

/****************************************************************************
 * Name: can_drv_rx0_isr
 *
 * Description:
 *   CAN RX FIFO 0 interrupt handler
 *
 * Input Parameters:
 *   irq - The IRQ number of the interrupt.
 *   context - The register state save array at the time of the interrupt.
 *
 * Returned Value:
 *   Zero on success; a negated errno on failure
 *
 ****************************************************************************/

static int can_drv_rx0_isr(int irq, void *context, void *arg)
{
    FAR struct can_describe *priv = NULL;
    struct rstcan_frame frame;
    uint32_t regval;
    int npending;

    if (g_can1desc.rx0_irq == irq)
    {
        priv = &g_can1desc;
    }
#if (RSTCAN_PORT_NUM == 2)
    else if (g_can2desc.rx0_irq == irq)
    {
        priv = &g_can2desc;
    }
#endif

    /* Verify that a message is pending in FIFO 0 */

    regval   = _readreg(priv, STM32_CAN_RF0R_OFFSET);
    npending = (regval & CAN_RFR_FMP_MASK) >> CAN_RFR_FMP_SHIFT;
    if (npending < 1)
    {
      //printf("WARNING: No messages pending\n");
      return OK;
    }

    /* Get the CAN identifier. */

    regval = _readreg(priv, STM32_CAN_RI0R_OFFSET);

    if ((regval & CAN_RIR_IDE) != 0)
    {
        frame.can_id  = (regval & CAN_RIR_EXID_MASK) >> CAN_RIR_EXID_SHIFT;
        frame.ide     = true;
    }
    else
    {
        frame.can_id    = (regval & CAN_RIR_STID_MASK) >> CAN_RIR_STID_SHIFT;
        frame.ide       = false;
    }

    /* Extract the RTR bit */

    frame.rtr   = (regval & CAN_RIR_RTR) != 0 ? true : false;

    /* Get the DLC */

    regval       = _readreg(priv, STM32_CAN_RDT0R_OFFSET);
    frame.dlc   = (regval & CAN_RDTR_DLC_MASK) >> CAN_RDTR_DLC_SHIFT;

    /* Save the message data */

    regval  = _readreg(priv, STM32_CAN_RDL0R_OFFSET);
    frame.data[0] = (regval & CAN_RDLR_DATA0_MASK) >> CAN_RDLR_DATA0_SHIFT;
    frame.data[1] = (regval & CAN_RDLR_DATA1_MASK) >> CAN_RDLR_DATA1_SHIFT;
    frame.data[2] = (regval & CAN_RDLR_DATA2_MASK) >> CAN_RDLR_DATA2_SHIFT;
    frame.data[3] = (regval & CAN_RDLR_DATA3_MASK) >> CAN_RDLR_DATA3_SHIFT;

    regval  = _readreg(priv, STM32_CAN_RDH0R_OFFSET);
    frame.data[4] = (regval & CAN_RDHR_DATA4_MASK) >> CAN_RDHR_DATA4_SHIFT;
    frame.data[5] = (regval & CAN_RDHR_DATA5_MASK) >> CAN_RDHR_DATA5_SHIFT;
    frame.data[6] = (regval & CAN_RDHR_DATA6_MASK) >> CAN_RDHR_DATA6_SHIFT;
    frame.data[7] = (regval & CAN_RDHR_DATA7_MASK) >> CAN_RDHR_DATA7_SHIFT;

    /* Provide the data to the app*/

    _recv_frame(priv, &frame);

    /* Release the FIFO0 */
    regval  = _readreg(priv, STM32_CAN_RF0R_OFFSET);
    regval |= CAN_RFR_RFOM;
    _writereg(priv, STM32_CAN_RF0R_OFFSET, regval);
    return OK;
}

static bool _txfifo_empty(FAR struct can_describe *priv)
{
  uint32_t regval;

  /* Return true if all mailboxes are available */

  regval = _readreg(priv, STM32_CAN_TSR_OFFSET);
  //printf("CAN%d TSR: %08x\n", priv->port, regval);

  if ((regval & _ALL_MAILBOXES ) == _ALL_MAILBOXES )
    {
      return true;
    }
  return false;
}

static bool _txfifo_ready(FAR struct can_describe *priv)
{
  uint32_t regval;

  /* Return true if any mailbox is available */

  regval = _readreg(priv, STM32_CAN_TSR_OFFSET);
//  printf("CAN%d TSR: %08x\n", priv->port, regval);

  //if ((regval & _ALL_MAILBOXES ) != 0)
  //can控制器bug, 必须等3个邮箱都空再发,不然数据量大的时候会丢数据
  if ((regval & _ALL_MAILBOXES ) == _ALL_MAILBOXES )
    {
      return true;
    }
  return false;
}

/****************************************************************************
 * Name: _send_fifo
 *
 * Description:
 *    Send one can message.
 *
 *    One CAN-message consists of a maximum of 10 bytes.  A message is
 *    composed of at least the first 2 bytes (when there are no data bytes).
 *
 *    Byte 0:      Bits 0-7: Bits 3-10 of the 11-bit CAN identifier
 *    Byte 1:      Bits 5-7: Bits 0-2 of the 11-bit CAN identifier
 *                 Bit 4:    Remote Tranmission Request (RTR)
 *                 Bits 0-3: Data Length Code (DLC)
 *    Bytes 2-10: CAN data
 *
 * Input Parameters:
 *   dev - An instance of the "upper half" can driver state structure.
 *
 * Returned Value:
 *   Zero on success; a negated errno on failure
 *
 ****************************************************************************/

static int _send_fifo(FAR struct can_describe *priv, FAR struct rstcan_frame *msg)
{
    FAR uint8_t *ptr;
    uint32_t regval;
    uint32_t tmp;
    int dlc;
    int txmb;

    //printf("send CAN%d ID: %d DLC: %d\n", priv->port, msg->can_id, msg->dlc);

    /* Select one empty transmit mailbox */

    regval = _readreg(priv, STM32_CAN_TSR_OFFSET);
    if ((regval & CAN_TSR_TME0) != 0)
    {
      txmb = 0;
    }
    else if ((regval & CAN_TSR_TME1) != 0)
    {
      txmb = 1;
    }
    else if ((regval & CAN_TSR_TME2) != 0)
    {
      txmb = 2;
    }
    else
    {
      printf("ERROR: No available mailbox\n");
      return -EBUSY;
    }

    /* Clear TXRQ, RTR, IDE, EXID, and STID fields */

    regval  = _readreg(priv, STM32_CAN_TIR_OFFSET(txmb));
    regval &= ~(CAN_TIR_TXRQ | CAN_TIR_RTR | CAN_TIR_IDE | CAN_TIR_EXID_MASK | CAN_TIR_STID_MASK);
    _writereg(priv, STM32_CAN_TIR_OFFSET(txmb), regval);

    /* Set up the ID, standard 11-bit or extended 29-bit. */
    regval &= ~CAN_TIR_EXID_MASK;
    if (msg->ide)
    {
      DEBUGASSERT(msg->can_id < (1 << 29));
      regval |= (msg->can_id << CAN_TIR_EXID_SHIFT) | CAN_TIR_IDE;
    }
    else
    {
      DEBUGASSERT(msg->can_id < (1 << 11));
      regval |= msg->can_id << CAN_TIR_STID_SHIFT;
    }

    //robsense modify 判断是否要发远程帧
    if(msg->rtr)
      regval |= CAN_TIR_RTR;

    _writereg(priv, STM32_CAN_TIR_OFFSET(txmb), regval);

    /* Set up the DLC */

    dlc     = msg->dlc;
    regval  = _readreg(priv, STM32_CAN_TDTR_OFFSET(txmb));
    regval &= ~(CAN_TDTR_DLC_MASK | CAN_TDTR_TGT);
    regval |= (uint32_t)dlc << CAN_TDTR_DLC_SHIFT;
    _writereg(priv, STM32_CAN_TDTR_OFFSET(txmb), regval);

    /* Set up the data fields */

    ptr    = msg->data;
    regval = 0;

    if (dlc > 0)
    {
      tmp    = (uint32_t)*ptr++;
      regval = tmp << CAN_TDLR_DATA0_SHIFT;

      if (dlc > 1)
       {
         tmp    = (uint32_t)*ptr++;
         regval |= tmp << CAN_TDLR_DATA1_SHIFT;

         if (dlc > 2)
           {
             tmp    = (uint32_t)*ptr++;
             regval |= tmp << CAN_TDLR_DATA2_SHIFT;

             if (dlc > 3)
               {
                 tmp    = (uint32_t)*ptr++;
                 regval |= tmp << CAN_TDLR_DATA3_SHIFT;
               }
           }
       }
    }
    _writereg(priv, STM32_CAN_TDLR_OFFSET(txmb), regval);

    regval = 0;
    if (dlc > 4)
    {
      tmp    = (uint32_t)*ptr++;
      regval = tmp << CAN_TDHR_DATA4_SHIFT;

      if (dlc > 5)
       {
         tmp    = (uint32_t)*ptr++;
         regval |= tmp << CAN_TDHR_DATA5_SHIFT;

         if (dlc > 6)
           {
             tmp    = (uint32_t)*ptr++;
             regval |= tmp << CAN_TDHR_DATA6_SHIFT;

             if (dlc > 7)
               {
                 tmp    = (uint32_t)*ptr++;
                 regval |= tmp << CAN_TDHR_DATA7_SHIFT;
               }
           }
       }
    }
    _writereg(priv, STM32_CAN_TDHR_OFFSET(txmb), regval);

    /* Enable the transmit mailbox empty interrupt (may already be enabled) */

    regval  = _readreg(priv, STM32_CAN_IER_OFFSET);
    regval |= CAN_IER_TMEIE;
    _writereg(priv, STM32_CAN_IER_OFFSET, regval);

    /* Request transmission */

    regval  = _readreg(priv, STM32_CAN_TIR_OFFSET(txmb));
    regval |= CAN_TIR_TXRQ;  /* Transmit Mailbox Request */
    _writereg(priv, STM32_CAN_TIR_OFFSET(txmb), regval);

    //  can_dumpmbregs(priv, "After send");
    return OK;
}

static int32_t _send_hardware(struct can_describe *desc)
{
    int32_t ret = 0;
    uint16_t _tail;

    while(BUF_AVAILABLE(desc->tx_buf) != 0 && _txfifo_ready(desc))
    {
       /* Send the next message at the FIFO queue index */

        ret = _send_fifo(desc, &desc->tx_buf[desc->tx_buf_head]);
        if (ret != OK)
        {
            printf("dev_send failed: %d\n", ret);
            break;
        }

        BUF_ADVANCEHEAD(desc->tx_buf, 1);
    }


    return 0;
}

//获取发送缓冲剩余的字节数
int32_t can_drv_get_send_space(void *fd)
{
    struct can_describe *desc = (struct can_describe *)fd;
    uint16_t _head;

    return BUF_SPACE(desc->tx_buf) * RSTCAN_FRAME_MAX_DATA;
    
}

static void _send_timeout(void *arg)
{
    sem_t *p_sem = (sem_t *)arg;
    sem_post(p_sem);
}

int32_t can_drv_send_frame(void *fd, struct rstcan_frame *frame)
{
    uint16_t msglen;
    uint16_t _head, space;
    irqstate_t            flags = 0;
    struct can_describe *desc = (struct can_describe *)fd;
    struct hrt_call timeout_call;

    //关闭中断,确保原子操作,防止其他线程打断
    flags = up_irq_save();

    space = BUF_SPACE(desc->tx_buf);
    if (space == 0) {
      //  printf("warning can tx buf full\n");
        up_irq_restore(flags);
        return -1;
    }

    msglen = _MSGLEN(frame->dlc);
    memcpy(&desc->tx_buf[desc->tx_buf_tail], frame, msglen);
    BUF_ADVANCETAIL(desc->tx_buf, 1);

    if(_txfifo_empty(desc))
    {
        _send_hardware(desc);    
        hrt_call_after(&timeout_call,
			       1000,
			       _send_timeout,
			       (void *)&desc->tx_sem);

        /* Wait for a message to be sent */
        desc->tx_waiter++;
        sem_wait(&desc->tx_sem);
        desc->tx_waiter--;
        hrt_cancel(&timeout_call);
    }

    up_irq_restore(flags);
    return 0;
}

/****************************************************************************
 * Name: can_drv_tx_isr
 *
 * Description:
 *   CAN TX mailbox complete interrupt handler
 *
 * Input Parameters:
 *   irq - The IRQ number of the interrupt.
 *   context - The register state save array at the time of the interrupt.
 *
 * Returned Value:
 *   Zero on success; a negated errno on failure
 *
 ****************************************************************************/

static int can_drv_tx_isr(int irq, void *context, void *arg)
{
    FAR struct can_describe *priv = NULL;
    uint32_t regval;

    if (g_can1desc.tx_irq == irq)
    {
        priv = &g_can1desc;
    }
#if (RSTCAN_PORT_NUM == 2)
    else if (g_can2desc.tx_irq == irq)
    {
        priv = &g_can2desc;
    }
#endif


    /* Get the transmit status */

    regval = _readreg(priv, STM32_CAN_TSR_OFFSET);

    /* Check for RQCP0: Request completed mailbox 0 */

    if ((regval & CAN_TSR_RQCP0) != 0)
    {
      /* Writing '1' to RCP0 clears RCP0 and all the status bits (TXOK0,
       * ALST0 and TERR0) for Mailbox 0.
       */

      _writereg(priv, STM32_CAN_TSR_OFFSET, CAN_TSR_RQCP0);

      /* Check for errors */

      if ((regval & CAN_TSR_TXOK0) != 0)
        {
          /* Tell the upper half that the tansfer is finished. */

            (void)_send_hardware(priv);
            if(priv->tx_waiter > 0)
                sem_post(&priv->tx_sem);
        }
    }

    /* Check for RQCP1: Request completed mailbox 1 */

    if ((regval & CAN_TSR_RQCP1) != 0)
    {
      /* Writing '1' to RCP1 clears RCP1 and all the status bits (TXOK1,
       * ALST1 and TERR1) for Mailbox 1.
       */

      _writereg(priv, STM32_CAN_TSR_OFFSET, CAN_TSR_RQCP1);

      /* Check for errors */

      if ((regval & CAN_TSR_TXOK1) != 0)
        {
          /* Tell the upper half that the tansfer is finished. */

          (void)_send_hardware(priv);
            if(priv->tx_waiter > 0)
                sem_post(&priv->tx_sem);
        }
    }

    /* Check for RQCP2: Request completed mailbox 2 */

    if ((regval & CAN_TSR_RQCP2) != 0)
    {
      /* Writing '1' to RCP2 clears RCP2 and all the status bits (TXOK2,
       * ALST2 and TERR2) for Mailbox 2.
       */

      _writereg(priv, STM32_CAN_TSR_OFFSET, CAN_TSR_RQCP2);

      /* Check for errors */

      if ((regval & CAN_TSR_TXOK2) != 0)
        {
          /* Tell the upper half that the tansfer is finished. */

          (void)_send_hardware(priv);
            if(priv->tx_waiter > 0)
                sem_post(&priv->tx_sem);
        }
    }

    return OK;
}


void *can_drv_init(uint8_t port)
{
    uint32_t regval;
    int32_t ret = 0;
    struct can_describe *priv = p_can_desc[port];

    printf("CAN%d RX0 irq: %d TX irq: %d\n", priv->port, priv->rx0_irq, priv->tx_irq);

    //初始化can的发送信号量
    sem_init(&priv->tx_sem, 0, 0);

    if(port == 0)
    {
        /* Configure CAN1 pins.  The ambiguous settings in the stm32*_pinmap.h
        * file must have been disambiguated in the board.h file.
        */

        stm32_configgpio(GPIO_CAN1_RX);
        stm32_configgpio(GPIO_CAN1_TX);

    }
#if (RSTCAN_PORT_NUM == 2)
    else if(port == 1)
    {
        /* Configure CAN2 pins.  The ambiguous settings in the stm32*_pinmap.h
        * file must have been disambiguated in the board.h file.
        */

        stm32_configgpio(GPIO_CAN2_RX);
        stm32_configgpio(GPIO_CAN2_TX);

    }
#endif
    else
    {
        printf("no such can port:%d\n", port);
        return NULL;
    }

    /* CAN cell initialization */
    ret = _cellinit(priv);
    if (ret < 0)
    {
        printf("CAN%d cell initialization failed: %d\n", priv->port, ret);
        return NULL;
    }

    /* CAN filter initialization */
    ret = _filterinit(priv);
    if (ret < 0)
    {
      printf("CAN%d filter initialization failed: %d\n", priv->port, ret);
      return NULL;
    }

    /* Attach the CAN RX FIFO 0 interrupt and TX interrupts.  The others are not used */

    ret = irq_attach(priv->rx0_irq, can_drv_rx0_isr, nullptr);
    if (ret < 0)
    {
        printf("Failed to attach CAN%d RX0 IRQ (%d)", priv->port, priv->rx0_irq);
        return NULL;
    }

    ret = irq_attach(priv->tx_irq, can_drv_tx_isr, nullptr);
    if (ret < 0)
    {
        printf("Failed to attach CAN%d TX IRQ (%d)", priv->port, priv->tx_irq);
        return NULL;
    }

    /* Enable the interrupts at the NVIC.  Interrupts arestill disabled in
    * the CAN module.  Since we coming out of reset here, there should be
    * no pending interrupts.
    */

    up_enable_irq(priv->rx0_irq);
    up_enable_irq(priv->tx_irq);

    /* Enable the transmit mailbox empty interrupt (may already be enabled) */

    regval  = _readreg(priv, STM32_CAN_IER_OFFSET);
    regval |= CAN_IER_TMEIE;
    _writereg(priv, STM32_CAN_IER_OFFSET, regval);

    /* Enable/disable the FIFO 0 message pending interrupt */
    /*有数据在fifo中pending就产生中断*/
    regval = _readreg(priv, STM32_CAN_IER_OFFSET);
    regval |= CAN_IER_FMPIE0;
    _writereg(priv, STM32_CAN_IER_OFFSET, regval);

    return (void *)priv;
}



