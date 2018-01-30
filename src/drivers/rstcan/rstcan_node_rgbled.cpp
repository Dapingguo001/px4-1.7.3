/*********************************************************************************
 *Copyright(C),2015-2017, Robsense Tech. All rights reserved.
 *FileName:    rstcan_node_rgbled.cpp
 *Author:      HeBin
 *Version:     0.1
 *Date:        2017-11-21 20:24:20
 *Last Modify: 2017-11-21 20:24:20
 *Description: 
**********************************************************************************/

#include "rstcan_master.h"
#include "rstcan_manager.h"

RSTCan_Node_RgbLed::RSTCan_Node_RgbLed(const char *name, const char *path, void *_master_node, void *_slave_node, uint8_t idx):
    RSTCan_Node(name, path, _master_node, _slave_node, idx)
{

}

RSTCan_Node_RgbLed::~RSTCan_Node_RgbLed()
{

}

int RSTCan_Node_RgbLed::init()
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


int RSTCan_Node_RgbLed::ioctl(device::file_t *filp, int cmd, unsigned long arg)
{
	int ret = 0;
#if 0
    struct rstcan_msg msg;
    bool  send_can = true;
    uint8_t color = 0;
    uint8_t mode = 0;

	switch (cmd) {
	case RGBLED_SET_RGB:
    {
        RSTCAN_NOR_MSG_CONSTRUCTOR(msg, RGBLED_SET_RGB_SUB_MSG, (uint8_t *)arg, sizeof(rgbled_rgbset_t));

        break;
    }
	case RGBLED_SET_COLOR:
    {
//        uint8_t color = 0;//编译器bug,根据反汇编,定义在这里下面的赋值会被忽略,导致color是个随机值

        color = arg;

//        printf("ioctl set color: %d\n", color);

        RSTCAN_NOR_MSG_CONSTRUCTOR(msg, RGBLED_SET_COLOR_SUB_MSG, (uint8_t *)&color, sizeof(color));

        break;
    }
	case RGBLED_SET_MODE:
    {

        mode = (rgbled_mode_t)arg;

        RSTCAN_NOR_MSG_CONSTRUCTOR(msg, RGBLED_SET_MODE_SUB_MSG, (uint8_t *)&mode, sizeof(mode));

        break;
    }
	case RGBLED_SET_PATTERN://not used
        send_can = false;
        break;
	default:
		/* see if the parent class can make any use of it */
		ret = CDev::ioctl(filp, cmd, arg);
        send_can = false;
		break;
	}

    if(send_can == true)
    {
        ret = rstcan_send_msg(master_node, slave_node, &msg);
        if(ret != 0)
        {
            return -EIO;
        }
    }
#endif
	return ret;
}

void RSTCan_Node_RgbLed::test(char *arg)
{
#if 0
    int32_t ret = 0;
    rgbled_rgbset_t rgb = {100, 0, 0};
    rgbled_color_t color = RGBLED_COLOR_WHITE;

    printf("rgb led set red\n");
    ret = ioctl(NULL, RGBLED_SET_RGB, (unsigned long)&rgb);
    if(ret != 0)
    {
        printf("set rgb failed");
    }

    usleep(500000);
    rgb.red = 0;
    rgb.green = 100;
    printf("rgb led set green\n");
    ret = ioctl(NULL, RGBLED_SET_RGB, (unsigned long)&rgb);
    if(ret != 0)
    {
        printf("set rgb failed");
    }

    usleep(500000);
    rgb.green = 0;
    rgb.blue = 100;
    printf("rgb led set blue\n");
    ret = ioctl(NULL, RGBLED_SET_RGB, (unsigned long)&rgb);
    if(ret != 0)
    {
        printf("set rgb failed");
    }

    usleep(500000);
    printf("rgb led set white\n");
    ret = ioctl(NULL, RGBLED_SET_COLOR, (unsigned long)color);
    if(ret != 0)
    {
        printf("set rgb failed");
    }

#endif
}

