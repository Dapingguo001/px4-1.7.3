/*********************************************************************************
 *Copyright(C),2015-2018, Robsense Tech. All rights reserved.
 *FileName:    rstcan_node_landing_gear.cpp
 *Author:      HeBin
 *Version:     0.1
 *Date:        2018-01-15 11:23:52
 *Last Modify: 2018-01-31 13:44:44
 *Description: 
**********************************************************************************/

#include "rstcan_master.h"
#include "rstcan_manager.h"
#include <drivers/drv_rstcan.h>
#include <uORB/topics/rstcan_landing_gear.h>

RSTCan_Node_LandingGear::RSTCan_Node_LandingGear(const char *name, const char *path, void *_master_node, void *_slave_node, uint8_t idx):
    RSTCan_Node(name, path, _master_node, _slave_node, idx),
    _orb_sub_fd(0)
{

}

RSTCan_Node_LandingGear::~RSTCan_Node_LandingGear()
{

}

int RSTCan_Node_LandingGear::init()
{
    int ret = -1;

    ret = RSTCan_Node::init();
    if(ret != 0)
    {
        goto out;
    }

    ret = rstcan_register_msg(slave_node, RSTCAN_MSG_TYPE_NOR, 8, false); //注册普通消息,用于收发控制信息

    RSTCan_Node::node_task_start(1024, this);

out:
    return ret;
}

int32_t RSTCan_Node_LandingGear::_task()
{
    int32_t ret = -1;
    bool update = true;
    struct rstcan_landing_gear_s orb_msg;
    uint8_t sub_type = 0;
    rstcan_msg_t msg;
    int instance = _node_idx;

    _orb_sub_fd = orb_subscribe_multi(ORB_ID(rstcan_landing_gear), instance);

    while(1)
    {
        orb_check(_orb_sub_fd, &update);

        if(update)
        {
            orb_copy(ORB_ID(rstcan_landing_gear), _orb_sub_fd, &orb_msg);    

            ::printf("landing_gear%d cmd = %d\n", instance, orb_msg.cmd);

            switch(orb_msg.cmd)
            {
                case rstcan_landing_gear_s::RST_LANDING_GEAR_UP:
                    sub_type = LANDING_GAER_UP_SUB_MSG;

                    break;
                case rstcan_landing_gear_s::RST_LANDING_GEAR_DOWN:
                    sub_type = LANDING_GAER_DOWN_SUB_MSG;

                    break;
                case rstcan_landing_gear_s::RST_LANDING_GEAR_STOP: 
                    sub_type = LANDING_GAER_STOP_SUB_MSG;

                    break;
                default:
                    break;
            }

            RSTCAN_NOR_MSG_CONSTRUCTOR(msg, sub_type, NULL, 0);
            ret = rstcan_send_msg(master_node, slave_node, &msg);
            if(ret != 0)
            {
                ::printf("rst landing gear send %d err!!\n", sub_type);
            }

            update = false;
        }

        usleep(100000);
     
    }
   
    return 0;
}

int	RSTCan_Node_LandingGear::ioctl(struct file *filp, int cmd, unsigned long arg)
{
    return 0;
}

void RSTCan_Node_LandingGear::test(char *arg)
{
    orb_advert_t test_pub[RSTCAN_MAX_INDEX] = {nullptr};
    struct rstcan_landing_gear_s orb_msg = {0};
    int instance = _node_idx;

    ::printf("landing gear%d test, you can indicate these test items: up/down/stop\n", instance);

    if(arg == nullptr)
        return;

    //第一个发布返回的instance一定是0，如果要测试第二个节点，必须发布第二次，第三个节点以此类推
    for(int i = 0; i <= _node_idx; i++)
    {
        test_pub[i] = orb_advertise_multi(ORB_ID(rstcan_landing_gear), &orb_msg, &instance, ORB_PRIO_DEFAULT);
        orb_publish(ORB_ID(rstcan_landing_gear), test_pub[i], &orb_msg);
    }
    ::printf("advertise success\n");

    //等待空包发完
    usleep(100000);

    if(strcmp(arg, "up") == 0)
    {
        orb_msg.cmd = rstcan_landing_gear_s::RST_LANDING_GEAR_UP;
    }
    else if(strcmp(arg, "down") == 0)
    {
        orb_msg.cmd = rstcan_landing_gear_s::RST_LANDING_GEAR_DOWN;
    }
    else if(strcmp(arg, "stop") == 0)
    {
        orb_msg.cmd = rstcan_landing_gear_s::RST_LANDING_GEAR_STOP;
    }

    orb_publish(ORB_ID(rstcan_landing_gear), test_pub[_node_idx], &orb_msg);

    for(int i = 0; i <= _node_idx; i++)
        orb_unadvertise(test_pub[i]);
}
