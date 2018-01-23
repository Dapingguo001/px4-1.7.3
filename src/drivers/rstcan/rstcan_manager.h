/*********************************************************************************
 *Copyright(C),2015-2017, Robsense Tech. All rights reserved.
 *FileName:    rstcan_manager.h
 *Author:      HeBin
 *Version:     0.1
 *Date:        2017-11-10 10:39:09
 *Last Modify: 2017-11-10 10:39:09
 *Description: 
**********************************************************************************/

#pragma once
#include <pthread.h>
#include <nuttx/wqueue.h>
#include <nuttx/clock.h>
#include <drivers/drv_hrt.h>
#include "rstcan_master.h"
#include "stm32_can_drv.h"
#include "rstcan_node.h"


class RSTCan_Manager : public device::CDev
{
public:
    RSTCan_Manager(const char *name, const char *path, int32_t canbus);
    virtual ~RSTCan_Manager();
	virtual int		init();
	virtual int	    ioctl(struct file *filp, int cmd, unsigned long arg);

    void can_rx(struct rstcan_frame *frame);

	static RSTCan_Manager *get_instance(uint8_t port);
	class RSTCan_Node *get_node_instance(uint16_t node_type, uint16_t node_pos);
    int32_t _broadcast_all(uint8_t msg_sub_type, uint8_t *buf, uint32_t len);

private:
    int32_t can_bus;
    void *can_fd;
    void *master_node;

    class RSTCan_Node *rstcan_node[RSTCAN_NODE_NR][RSTCAN_MAX_INDEX];

    uint32_t    invalid_node_count;
    uint32_t    unknow_node_count;

    work_s _work;
    static void _task_trampoline(void *arg);

    int32_t _alloc_slave_id();
};


