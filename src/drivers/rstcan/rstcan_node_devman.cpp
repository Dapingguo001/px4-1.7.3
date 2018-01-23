/*********************************************************************************
 *Copyright(C),2015-2017, Robsense Tech. All rights reserved.
 *FileName:    rstcan_node_devman.cpp
 *Author:      HeBin
 *Version:     0.1
 *Date:        2017-12-13 10:43:56
 *Last Modify: 2017-12-13 10:43:56
 *Description: 
**********************************************************************************/

#include "rstcan_master.h"
#include "rstcan_manager.h"
#include <drivers/drv_rstcan.h>

RSTCan_Node_DevMan::RSTCan_Node_DevMan(const char *name, const char *path, void *_master_node, void *_slave_node):
    RSTCan_Node(name, path, _master_node, _slave_node)
{

}

RSTCan_Node_DevMan::~RSTCan_Node_DevMan()
{

}

int RSTCan_Node_DevMan::init()
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

int RSTCan_Node_DevMan::ioctl(device::file_t *filp, int cmd, unsigned long arg)
{
	int ret = -ENOTTY;

    return ret;
}

void RSTCan_Node_DevMan::test()
{

}
