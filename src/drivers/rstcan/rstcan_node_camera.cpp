/*********************************************************************************
 *Copyright(C),2015-2018, Robsense Tech. All rights reserved.
 *FileName:    rstcan_node_camera.cpp
 *Author:      HeBin
 *Version:     0.1
 *Date:        2018-01-15 11:23:52
 *Last Modify: 2018-01-26 15:39:39
 *Description: 
**********************************************************************************/

#include "rstcan_master.h"
#include "rstcan_manager.h"
#include <drivers/drv_rstcan.h>
#include <uORB/topics/rstcan_camera.h>

RSTCan_Node_Camera::RSTCan_Node_Camera(const char *name, const char *path, void *_master_node, void *_slave_node, uint8_t idx):
    RSTCan_Node(name, path, _master_node, _slave_node, idx),
    _orb_sub_fd(0)
{

}

RSTCan_Node_Camera::~RSTCan_Node_Camera()
{

}

int RSTCan_Node_Camera::init()
{
    int ret = -1;
 //   struct mallinfo data;

 //   data = mallinfo();
  //  ::printf("free mem:%d\n", data.fordblks);

    ret = RSTCan_Node::init();
    if(ret != 0)
    {
        goto out;
    }

    ret = rstcan_register_msg(slave_node, RSTCAN_MSG_TYPE_NOR, 16, false); //注册普通消息,用于收发控制信息

    RSTCan_Node::node_task_start(1500, this);

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
    int instance = _node_idx;

    _orb_sub_fd = orb_subscribe_multi(ORB_ID(rstcan_camera), instance);

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

void RSTCan_Node_Camera::test(char *arg)
{
    orb_advert_t test_pub[RSTCAN_MAX_INDEX] = {nullptr};
    struct rstcan_camera_s orb_msg = {0};
    int instance = _node_idx;

    ::printf("camera%d test, you can indicate these test items: ptz/record/snap/zoomin/zoomout/zoomstop/middle\n", instance);

    if(arg == nullptr)
        return;

    //第一个发布返回的instance一定是0，如果要测试第二个节点，必须发布第二次，第三个节点以此类推
    for(int i = 0; i <= _node_idx; i++)
    {
        test_pub[i] = orb_advertise_multi(ORB_ID(rstcan_camera), &orb_msg, &instance, ORB_PRIO_DEFAULT);
        orb_publish(ORB_ID(rstcan_camera), test_pub[i], &orb_msg);
    }
    ::printf("advertise success\n");

    if(strcmp(arg, "ptz") == 0)
    {
        static bool stop = false;
        static bool orentation = false;
        ptz_ctrl_t ptz;

        ptz.mode = RST_PTZ_CTRL_HEAD_LOCK_MODE;

        if(stop == false)
        {
            ptz.roll = 0.0f;
            ptz.pitch = 0.0f;
            if(orentation)
            {
                ptz.yaw = 1.0f;
                orentation = false;
            }
            else
            {
                ptz.yaw = -1.0f;
                orentation = true;
            }

            stop = true;
        }
        else
        {
            ptz.roll = 0.0f;
            ptz.pitch = 0.0f;
            ptz.yaw = 0.0;
            stop = false;
        }

        ::printf("roll=%f pitch=%f yaw=%f\n", (double)ptz.roll, (double)ptz.pitch, (double)ptz.yaw);

        orb_msg.cmd = rstcan_camera_s::RST_CAMERA_PTZ_CTRL;
        memcpy(orb_msg.data, &ptz, sizeof(ptz));
       
    }
    else if(strcmp(arg, "record") == 0)
    {
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
    }
    else if(strcmp(arg, "snap") == 0)
    {
        orb_msg.cmd = rstcan_camera_s::RST_CAMERA_SNAPSHOT;
        memset(orb_msg.data, 0, sizeof(orb_msg.data));
    }
    else if(strcmp(arg, "zoomin") == 0)
    {
        orb_msg.cmd = rstcan_camera_s::RST_CAMERA_ZOOM_IN;
        memset(orb_msg.data, 0, sizeof(orb_msg.data));
    }
    else if(strcmp(arg, "zoomout") == 0)
    {
        orb_msg.cmd = rstcan_camera_s::RST_CAMERA_ZOOM_OUT;
        memset(orb_msg.data, 0, sizeof(orb_msg.data));
    }
    else if(strcmp(arg, "zoomstop") == 0)
    {
        orb_msg.cmd = rstcan_camera_s::RST_CAMERA_ZOOM_STOP;
        memset(orb_msg.data, 0, sizeof(orb_msg.data));
    }
    else if(strcmp(arg, "middle") == 0)
    {
        ptz_ctrl_t ptz = {0};
    
        ptz.mode = RST_PTZ_CTRL_MIDDLE_MODE;

        ::printf("ptz middle\n");
        orb_msg.cmd = rstcan_camera_s::RST_CAMERA_PTZ_CTRL;
        memcpy(orb_msg.data, &ptz, sizeof(ptz));
    }

    orb_publish(ORB_ID(rstcan_camera), test_pub[_node_idx], &orb_msg);

    for(int i = 0; i <= _node_idx; i++)
        orb_unadvertise(test_pub[i]);
}
