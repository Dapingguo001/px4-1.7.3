/*********************************************************************************
 *Copyright(C),2015-2018, Robsense Tech. All rights reserved.
 *FileName:    rstcan_node_battery.cpp
 *Author:      HeBin
 *Version:     0.1
 *Date:        2018-01-30 17:37:49
 *Last Modify: 2018-01-30 17:37:49
 *Description: 
**********************************************************************************/

#include <stdlib.h>
#include "rstcan_master.h"
#include "rstcan_manager.h"
#include <drivers/drv_rstcan.h>
#include <uORB/topics/battery_status.h>

RSTCan_Node_Battery::RSTCan_Node_Battery(const char *name, const char *path, void *_master_node, void *_slave_node, uint8_t idx):
    RSTCan_Node(name, path, _master_node, _slave_node, idx),
    _advert_pub(nullptr)
{

}

RSTCan_Node_Battery::~RSTCan_Node_Battery()
{

}

int RSTCan_Node_Battery::init()
{
    int ret = -1;

    ret = RSTCan_Node::init();
    if(ret != 0)
    {
        goto out;
    }

    ret = rstcan_register_msg(slave_node, RSTCAN_MSG_TYPE_SVC, 8, false); //注册普通消息,用于收发控制信息

    RSTCan_Node::node_task_start(1024, this);

out:
    return ret;
}

int32_t RSTCan_Node_Battery::_task()
{
    int32_t ret = -1;
    rstcan_msg_t msg;
    batt_info_t data = {0};//can总线传来的数据
    struct battery_status_s orb_msg = {0};//通过orb给应用的数据
    int instance = _node_idx;

    while(1)
    {
        RSTCAN_SVC_MSG_CONSTRUCTOR(msg, BATTERY_GET_INFO_SUB_MSG, (uint8_t *)&data, sizeof(data));
        ret = rstcan_recv_msg(slave_node, &msg);
        if(ret == RSTCAN_RECV_SUCCESS)
        {
            ::printf("vol=%d num=%d\n", data.total_vol, data.batt_num);
            memcpy(&data, msg.data, msg.len);

            orb_msg.voltage_v = (float)data.total_vol * 0.001f;
            orb_msg.voltage_filtered_v = orb_msg.voltage_v;
            orb_msg.current_a = (float)data.curr * 0.001f;
            orb_msg.current_filtered_a = orb_msg.current_a;
            orb_msg.discharged_mah = abs(data.charge_stat);//电池剩余容量
            orb_msg.remaining = orb_msg.discharged_mah/(float)data.capacity;//容量百分比?
            orb_msg.scale = 1;
            orb_msg.cell_count = data.batt_num;
            orb_msg.connected = true;
            orb_msg.system_source = 0;//XXX ???
            orb_msg.priority = 0;

            orb_publish_auto(ORB_ID(battery_status), &_advert_pub, &orb_msg, &instance, ORB_PRIO_DEFAULT);
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

int	RSTCan_Node_Battery::ioctl(struct file *filp, int cmd, unsigned long arg)
{
    return 0;
}

void RSTCan_Node_Battery::test(char *arg)
{
    int32_t count = 0;
    bool update = true;
    struct battery_status_s orb_msg;
    int _orb_sub_fd;

    _orb_sub_fd = orb_subscribe_multi(ORB_ID(battery_status), _node_idx);

    while(1)
    {
        orb_check(_orb_sub_fd, &update);
        if(update)
        {
            count++;
            orb_copy(ORB_ID(battery_status), _orb_sub_fd, &orb_msg);    
        
            ::printf("vol = %f batt_num=%d\n", (double)orb_msg.voltage_v, orb_msg.cell_count);
        }

        if(count >= 10)
        {
            break;
        }
    
        usleep(500000);
    }
}

