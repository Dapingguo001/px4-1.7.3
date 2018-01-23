#ifndef __STM32_CAN_DRV__
#define __STM32_CAN_DRV__

#include "rstcan_master.h"

#if defined(CONFIG_ARCH_BOARD_PHENIX_GH)
#define RSTCAN_PORT_NUM     (1)
#elif defined(CONFIG_ARCH_BOARD_PX4FMU_ROB)
#define RSTCAN_PORT_NUM     (2)
#else
//#error Missing definition: RSTCAN_PORT_NUM must be defined in stm32_can_drv.h
#define RSTCAN_PORT_NUM     (1)
#endif
#define CAN_DRV_PORT_BUF_SIZE     (64)

# define readreg32(a)           (*(volatile uint32_t *)(a))
# define writereg32(a,v)        (*(volatile uint32_t *)(a) = (v))

struct can_describe{
    uint8_t     port;           /* CAN port number (1 or 2) */
    uint8_t     rx0_irq;        /* CAN RX FIFO 0 IRQ number */
    uint8_t     tx_irq;         /* CAN TX IRQ number */
    uint32_t    base_addr;      /* Base address of the CAN registers */
    uint8_t     filter;         /* Filter number */
    uint32_t    baudrate;       /* Configured baud */

    sem_t       tx_sem;                  /* Counting semaphore */
    uint8_t     tx_waiter;                  /* Counting semaphore */
    uint16_t tx_buf_size;
    volatile uint16_t       tx_buf_head;                 /* Index to the head [IN] in the circular buffer */
    volatile uint16_t       tx_buf_tail;                 /* Index to the tail [OUT] in the circular buffer */
    struct rstcan_frame tx_buf[CAN_DRV_PORT_BUF_SIZE];                 /* Circular buffer of CAN messages */
};

/**********************************ring buffer********************************************/
#define BUF_AVAILABLE(buf) ((buf##_head > (_tail=buf##_tail))? (buf##_size - buf##_head) + _tail: _tail - buf##_head)
#define BUF_SPACE(buf) (((_head=buf##_head) > buf##_tail)?(_head - buf##_tail) - 1:((buf##_size - buf##_tail) + _head) - 1)
#define BUF_EMPTY(buf) (buf##_head == buf##_tail)
#define BUF_ADVANCETAIL(buf, n) buf##_tail = (buf##_tail + n) % buf##_size
#define BUF_ADVANCEHEAD(buf, n) buf##_head = (buf##_head + n) % buf##_size


#ifdef __cplusplus
extern "C"{
#endif

inline uint32_t _readreg(struct can_describe *desc, int offset)
{
    return readreg32(desc->base_addr+offset);
}

inline void _writereg(struct can_describe *desc, int offset, uint32_t value)
{
    writereg32(desc->base_addr+offset, value);
}

void *can_drv_init(uint8_t port);
int32_t can_drv_get_send_space(void *fd);
int32_t can_drv_send_frame(void *fd, struct rstcan_frame *frame);

#ifdef __cplusplus
}
#endif

#endif
