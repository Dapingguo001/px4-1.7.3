/*********************************************************************************
 *Copyright(C),2015-2017, Robsense Tech. All rights reserved.
 *FileName:    rstcan_node_4g_ctrl.cpp
 *Author:      HeBin
 *Version:     0.1
 *Date:        2017-11-21 19:08:30
 *Last Modify: 2018-01-26 15:39:39
 *Description: 
**********************************************************************************/

#include "rstcan_master.h"
#include "rstcan_manager.h"
#include <drivers/drv_rstcan.h>

#define WAIT_ACK_TIMOUT_US  3000000   

RSTCan_Node_4G_CTRL::RSTCan_Node_4G_CTRL(const char *name, const char *path, void *_master_node, void *_slave_node):
    RSTCan_Node(name, path, _master_node, _slave_node)
{

}

RSTCan_Node_4G_CTRL::~RSTCan_Node_4G_CTRL()
{

}

int RSTCan_Node_4G_CTRL::init()
{
    int ret = -1;

    ret = RSTCan_Node::init();
    if(ret != 0)
    {
        goto out;
    }

    ret = rstcan_register_msg(slave_node, RSTCAN_MSG_TYPE_NOR, 40, false); //注册普通消息,用于收发控制信息

out:
    return ret;
}


int32_t RSTCan_Node_4G_CTRL::_wait_ack(uint8_t *buf, uint32_t len)
{
    struct rstcan_msg msg;
    hrt_abstime start_time = 0;
    hrt_abstime now_time = 0;

    start_time = hrt_absolute_time();
    now_time  = start_time;
    while(now_time - start_time <= WAIT_ACK_TIMOUT_US)
    {
        now_time = hrt_absolute_time();

        RSTCAN_NOR_MSG_CONSTRUCTOR(msg, RADIO_4G_ACK_SUB_MSG, (uint8_t *)(buf), len);
        if(rstcan_recv_msg(slave_node, &msg) == RSTCAN_RECV_SUCCESS && msg.sub_type == RADIO_4G_ACK_SUB_MSG)
        {
            return msg.len;
        }
    }

    return 0;
}

int RSTCan_Node_4G_CTRL::ioctl(device::file_t *filp, int cmd, unsigned long arg)
{
	int ret = -ENOTTY;
#if 0
    struct rstcan_msg msg;
    uint8_t ack_len = 0;
    
    switch(cmd)
    {
        case RSTCAN_IOC_SET_4G_SERVER:
        {
            char *server = (char *)arg;
            uint8_t reply[5] = {0};

            RSTCAN_NOR_MSG_CONSTRUCTOR(msg, RADIO_4G_SET_SERVER_SUB_MSG, (uint8_t *)server, strlen(server)+1);
            ret = rstcan_send_msg(master_node, slave_node, &msg);
            if(ret != 0)
            {
                return -EIO;
            }
            
            ack_len = _wait_ack(reply, sizeof(reply));
            if(ack_len != 0 && strcmp((const char *)reply, RADIO_4G_CTRL_ACK_OK) == 0)
            {
                ret = 0;
            }
            else
            {
                printf("set 4g server failed: %s\n", reply);
                ret = EIO;
            }

            break;
        }
        case RSTCAN_IOC_GET_4G_BOOT:
        {
            struct radio_4g_status status;

            RSTCAN_NOR_MSG_CONSTRUCTOR(msg, RADIO_4G_GET_STATUS_SUB_MSG, NULL, 0);
            ret = rstcan_send_msg(master_node, slave_node, &msg);
            if(ret != 0)
            {
                return -EIO;
            }

            ack_len = _wait_ack((uint8_t *)&status, sizeof(status));
            if(ack_len != sizeof(status))
            {
                printf("get boot status failed: %d\n", ack_len);
                ret = -EIO;
            }
            else
            {
                *(int32_t *)arg = status.bootup;
                ret = 0;
            }

            break;
        }
        default:
            /* see if the parent class can make any use of it */
            ret = CDev::ioctl(filp, cmd, arg);
            break;
    }

#endif
    return ret;
}

void RSTCan_Node_4G_CTRL::test(char *arg)
{
#if 0 
    int32_t ret = 0;
    int32_t bootup = 0;

    ret = ioctl(NULL, RSTCAN_IOC_GET_4G_BOOT, (unsigned long)&bootup);

    if(ret == 0 && bootup == 1)
    {
        ret = ioctl(NULL, RSTCAN_IOC_SET_4G_SERVER, (unsigned long)"TCP,114.114.114.114,65535");
        if(ret == 0)
        {
            printf("set server sussess\n");
        }
    }
    else
    {
        printf("4G module not bootup yet, wait\n");
    }
#endif
}

