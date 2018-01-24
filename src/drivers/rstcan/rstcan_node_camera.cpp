/*********************************************************************************
 *Copyright(C),2015-2018, Robsense Tech. All rights reserved.
 *FileName:    rstcan_node_camera.cpp
 *Author:      HeBin
 *Version:     0.1
 *Date:        2018-01-15 11:23:52
 *Last Modify: 2018-01-15 11:23:55
 *Description: 
**********************************************************************************/

#include "rstcan_master.h"
#include "rstcan_manager.h"
#include <drivers/drv_rstcan.h>
#include <uORB/topics/rstcan_camera.h>

RSTCan_Node_Camera::RSTCan_Node_Camera(const char *name, const char *path, void *_master_node, void *_slave_node):
    RSTCan_Node(name, path, _master_node, _slave_node),
    _orb_sub_fd(0)
{

}

RSTCan_Node_Camera::~RSTCan_Node_Camera()
{

}

int RSTCan_Node_Camera::init()
{
    int ret = -1;

    ret = RSTCan_Node::init();
    if(ret != 0)
    {
        goto out;
    }

    ret = rstcan_register_msg(slave_node, RSTCAN_MSG_TYPE_NOR, 40, false); //注册普通消息,用于收发控制信息

    RSTCan_Node::node_task_start(10000, this);

out:
    return ret;
}

int32_t RSTCan_Node_Camera::_task()
{
    int32_t ret = -1;
    bool update = true;
    struct rstcan_camera_s orb_msg;
    camera_can_payload_t payload;
    uint8_t sub_type = 0;
    rstcan_msg_t msg;
    ptz_ctrl_t *ptz;

    _orb_sub_fd = orb_subscribe(ORB_ID(rstcan_camera));

    while(1)
    {
        orb_check(_orb_sub_fd, &update);

        if(update)
        {
            orb_copy(ORB_ID(rstcan_camera), _orb_sub_fd, &orb_msg);    

            switch(orb_msg.cmd)
            {
                case rstcan_camera_s::RST_CAMERA_PTZ_CTRL:

                    ptz = (ptz_ctrl_t *)orb_msg.data;

                    sub_type = CAMERA_PTZ_CTRL_SUB_MSG;
                    payload.data_len = sizeof(ptz_ctrl_t);

                    ::printf("recv:%f %f %f data len:%d - %d\n", 
                            (double)ptz->roll, (double)ptz->pitch, (double)ptz->yaw, payload.data_len, TRANS_LEN(payload.data_len));

                    if(payload.data_len <= sizeof(payload.data))
                        memcpy(&payload.data, orb_msg.data, payload.data_len);

                    break;
                case rstcan_camera_s::RST_CAMERA_SNAPSHOT:
                    sub_type = CAMERA_SNAPSHOT_SUB_MSG;
                    payload.data_len = 0;

                    break;
                case rstcan_camera_s::RST_CAMERA_RECORD_START: 
                    sub_type = CAMERA_RECORD_START_SUB_MSG;
                    payload.data_len = 0;

                    break;
                case rstcan_camera_s::RST_CAMERA_RECORD_STOP: 
                    sub_type = CAMERA_RECORD_STOP_SUB_MSG;
                    payload.data_len = 0;

                    break;
                case rstcan_camera_s::RST_CAMERA_ZOOM_IN:
                    sub_type = CAMERA_ZOOM_IN_SUB_MSG;
                    payload.data_len = 0;

                    break;
                case rstcan_camera_s::RST_CAMERA_ZOOM_OUT:
                    sub_type = CAMERA_ZOOM_OUT_SUB_MSG;
                    payload.data_len = 0;

                    break;
                case rstcan_camera_s::RST_CAMERA_ZOOM_STOP:
                    sub_type = CAMERA_ZOOM_STOP_SUB_MSG;
                    payload.data_len = 0;

                    break;
                default:
                    break;
            }

            RSTCAN_NOR_MSG_CONSTRUCTOR(msg, sub_type, (uint8_t *)&payload, TRANS_LEN(payload.data_len));
            ret = rstcan_send_msg(master_node, slave_node, &msg);
            if(ret != 0)
            {
                ::printf("rst camera send %d err!!\n", sub_type);
            }

            update = false;
        }

        usleep(100000);
     
    }
   
    return 0;
}

int	RSTCan_Node_Camera::ioctl(struct file *filp, int cmd, unsigned long arg)
{
    return 0;
}

void RSTCan_Node_Camera::test()
{
    orb_advert_t test_pub;
    struct rstcan_camera_s orb_msg = {0};

    ::printf("camera test\n");

    test_pub = orb_advertise(ORB_ID(rstcan_camera), &orb_msg);
    ::printf("advertise success\n");

#if 0
    orb_msg.cmd = rstcan_camera_s::RST_CAMERA_SNAPSHOT;

    memset(orb_msg.data, 0, sizeof(orb_msg.data));
    orb_publish(ORB_ID(rstcan_camera), test_pub, &orb_msg);

    static bool record = true;
    if(record)
    {
        orb_msg.cmd = rstcan_camera_s::RST_CAMERA_RECORD_START;
        record = false;
    }
    else
    {
        orb_msg.cmd = rstcan_camera_s::RST_CAMERA_RECORD_STOP;
        record = true;
    }
    memset(orb_msg.data, 0, sizeof(orb_msg.data));
    orb_publish(ORB_ID(rstcan_camera), test_pub, &orb_msg);

#endif
#if 0
    usleep(500000);

    orb_msg.cmd = rstcan_camera_s::RST_CAMERA_ZOOM_STOP;
    memset(orb_msg.data, 0, sizeof(orb_msg.data));
    orb_publish(ORB_ID(rstcan_camera), test_pub, &orb_msg);
#endif
    static bool stop = false;
    ptz_ctrl_t ptz;

    if(stop == false)
    {
        ptz.roll = 0.0f;
        ptz.pitch = 0.0f;
        ptz.yaw = 1.0f;

        stop = true;
    }
    else
    {
        ptz.roll = 0.0f;
        ptz.pitch = 0.0f;
        ptz.yaw = 0.0;
        stop = false;
    }

    orb_msg.cmd = rstcan_camera_s::RST_CAMERA_PTZ_CTRL;
    memcpy(orb_msg.data, &ptz, sizeof(ptz));
    orb_publish(ORB_ID(rstcan_camera), test_pub, &orb_msg);

}
