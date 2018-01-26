/*********************************************************************************
 *Copyright(C),2015-2018, Robsense Tech. All rights reserved.
 *FileName:    rstcan_node_radar.cpp
 *Author:      HeBin
 *Version:     0.1
 *Date:        2018-01-15 11:23:52
 *Last Modify: 2018-01-26 15:39:39
 *Description: 
**********************************************************************************/

#include "rstcan_master.h"
#include "rstcan_manager.h"
#include <drivers/drv_rstcan.h>
#include <uORB/topics/rstcan_radar.h>

RSTCan_Node_Radar::RSTCan_Node_Radar(const char *name, const char *path, void *_master_node, void *_slave_node):
    RSTCan_Node(name, path, _master_node, _slave_node),
    _advert_pub(nullptr)
{

}

RSTCan_Node_Radar::~RSTCan_Node_Radar()
{

}

int RSTCan_Node_Radar::init()
{
    int ret = -1;

    ret = RSTCan_Node::init();
    if(ret != 0)
    {
        goto out;
    }

    ret = rstcan_register_msg(slave_node, RSTCAN_MSG_TYPE_SVC, 64, false); //注册普通消息,用于收发控制信息

    RSTCan_Node::node_task_start(0, this);

out:
    return ret;
}

int32_t RSTCan_Node_Radar::_task()
{
    int32_t ret = -1;
    rstcan_msg_t msg;
    radar_data_t data = {0};//can总线传来的数据
    struct rstcan_radar_s orb_msg = {0};//通过orb给应用的数据

    _advert_pub = orb_advertise(ORB_ID(rstcan_radar), &orb_msg);
    ::printf("advertise success\n");
    
    while(1)
    {
        RSTCAN_SVC_MSG_CONSTRUCTOR(msg, RADAR_GET_DATA_SUB_MSG, (uint8_t *)&data, sizeof(data));
        ret = rstcan_recv_msg(slave_node, &msg);
        if(ret == RSTCAN_RECV_SUCCESS)
        {
            //::printf("distance=%d\n", data.distance);
            memcpy(&data, msg.data, msg.len);
            orb_msg.data_mask = data.data_mask;
            orb_msg.distance = data.distance;    // mm
            orb_msg.speed = data.speed;          // cm/s
            orb_publish(ORB_ID(rstcan_radar), _advert_pub, &orb_msg);
            continue;
        }
        //如果还在组包，不要休眠,尽快收完一包
        else if(ret == RSTCAN_RECV_GOING)
        {
            continue;
        }

        usleep(10000);
    }
   
    return 0;
}

int	RSTCan_Node_Radar::ioctl(struct file *filp, int cmd, unsigned long arg)
{
    return 0;
}

void RSTCan_Node_Radar::test(char *arg)
{
    int32_t count = 0;
    bool update = true;
    struct rstcan_radar_s orb_msg;
    int _orb_sub_fd;

    _orb_sub_fd = orb_subscribe(ORB_ID(rstcan_radar));

    while(1)
    {
        orb_check(_orb_sub_fd, &update);
        if(update)
        {
            count++;
            orb_copy(ORB_ID(rstcan_radar), _orb_sub_fd, &orb_msg);    
        
            ::printf("distance = %d\n", orb_msg.distance);
        }

        if(count >= 10)
        {
            break;
        }
    
        usleep(500000);
    }
}
