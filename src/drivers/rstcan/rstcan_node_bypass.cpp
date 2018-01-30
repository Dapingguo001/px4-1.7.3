/*********************************************************************************
 *Copyright(C),2015-2017, Robsense Tech. All rights reserved.
 *FileName:    rstcan_node_bypass.cpp
 *Author:      HeBin
 *Version:     0.1
 *Date:        2017-11-10 13:53:27
 *Last Modify: 2018-01-30 17:17:05
 *Description: 
**********************************************************************************/

#include "rstcan_master.h"
#include "rstcan_manager.h"

RSTCan_Node_Bypass::RSTCan_Node_Bypass(const char *name, const char *path, void *_master_node, void *_slave_node, uint8_t idx):
    RSTCan_Node(name, path, _master_node, _slave_node, idx)
{

}

RSTCan_Node_Bypass::~RSTCan_Node_Bypass()
{

}

int RSTCan_Node_Bypass::init()
{
    int ret = -1;

    ret = RSTCan_Node::init();
    if(ret != 0)
    {
        goto out;
    }

    //初始化互斥锁，信号量当作互斥锁用时，信号量初始值为1
//    sem_init(&send_sem, 0, 1);
//    sem_init(&recv_sem, 0, 1);

    ret = rstcan_register_msg(slave_node, RSTCAN_MSG_TYPE_SVC, 512, true); //注册服务消息,用于收取透传数据

out:
    return ret;
}

ssize_t RSTCan_Node_Bypass::read(struct file *filp, char *buffer, size_t buflen)
{
    struct rstcan_msg msg;

    RSTCAN_SVC_MSG_CONSTRUCTOR(msg, BYPASS_NODES_DATA_SUB_MSG, (uint8_t *)(buffer), buflen);
    if(rstcan_recv_msg(slave_node, &msg) != RSTCAN_RECV_SUCCESS)
    {
        return 0;
    }
    
    return msg.len;
}

ssize_t	RSTCan_Node_Bypass::write(struct file *filp, const char *buffer, size_t len)
{
    uint32_t ret = 0;
    struct rstcan_msg msg;
    size_t bytes = 0;
    size_t once_bytes = 0;
    
    while(bytes < len)
    {
        if(len - bytes <= RSTCAN_PAYLOAD_BYTES)
        {
            once_bytes = len - bytes;
        }
        else
        {
            once_bytes = RSTCAN_PAYLOAD_BYTES;
        }

        RSTCAN_SVC_MSG_CONSTRUCTOR(msg, BYPASS_NODES_DATA_SUB_MSG, (uint8_t *)buffer+bytes, once_bytes);
        ret = rstcan_send_msg(master_node, slave_node, &msg);
        if(ret != 0)
        {
            break;
        }

        //printf("send %d bytes total:%d\n", once_bytes, len);
        bytes += once_bytes;
    }
    //printf("send %d bytes use %u\n", len,t2-t1);

    return bytes;
}


int	RSTCan_Node_Bypass::ioctl(struct file *filp, int cmd, unsigned long arg)
{
    int32_t ret = 0;
    switch (cmd)
    {

        case FIONREAD:
        {
            int32_t count;

            irqstate_t state = up_irq_save();

            count = rstcan_get_data_len(slave_node, RSTCAN_MSG_TYPE_SVC);

            up_irq_restore(state);

            *(int32_t *)arg = count;

            break;
        }

        case FIONWRITE:
        {
            irqstate_t state = up_irq_save();

            *(int32_t *)arg = rstcan_get_send_space(master_node);

            up_irq_restore(state);

            break;
        }

        //for px4
        case FIONSPACE:
        {
            irqstate_t state = up_irq_save();

            *(int32_t *)arg = rstcan_get_send_space(master_node);

            up_irq_restore(state);

            break;
        }

        default:
            /* see if the parent class can make any use of it */
            ret = CDev::ioctl(filp, cmd, arg);
            break;
    }

    return ret;
   
}

