/*********************************************************************************
 *Copyright(C),2015-2017, Robsense Tech. All rights reserved.
 *FileName:    rstcan_manager.c
 *Author:      HeBin
 *Version:     0.1
 *Date:        2017-11-10 10:28:31
 *Last Modify: 2018-01-30 17:17:05
 *Description: 
**********************************************************************************/

#include <sys/types.h>
#include <stdbool.h>
#include <stdlib.h>
#include <semaphore.h>
#include <string.h>
#include <fcntl.h>
#include <poll.h>
#include <errno.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>

#include <nuttx/arch.h>


#include <board_config.h>
#include <nuttx/can/can.h>
#include <systemlib/otp.h>

#include "rstcan_manager.h"
#include "can_proto.h"
#include <drivers/drv_rstcan.h>

#define CANID_ALLOCATION_TIMOUT_US  200000   //id分配每一步的等待时间

static uint8_t uid[RSTCAN_UID_SIZE] = {0};

extern "C" { __EXPORT int rstcan_main(int argc, char *argv[]); }

//每个can口对应一个manager
RSTCan_Manager *manager_instance[STM32_NCAN] = {nullptr};

static struct rstcan_ops ops ={
    .can_send_frame = can_drv_send_frame,
    .can_set_filter = NULL,
    .can_get_send_space = can_drv_get_send_space,
    .usleep = usleep,
};


RSTCan_Manager::RSTCan_Manager(const char *name, const char *path, int32_t canbus):
    CDev(name, path),
    can_bus(canbus),
    can_fd(nullptr),
    invalid_node_count(0),
    unknow_node_count(0)
{
    memset(rstcan_node, 0, sizeof(rstcan_node));
}

int	RSTCan_Manager::ioctl(struct file *filp, int cmd, unsigned long arg)
{
    int32_t ret = -1;

    switch(cmd)
    {
        case RSTCAN_IOC_SLAVE_REBOOT:
            ret = _broadcast_all(BST_SLAVE_REBOOT_SUB_MSG, NULL, 0);
            break;
        
        case RSTCAN_IOC_GET_UID:
            if(arg != 0)
            {
                memcpy((void *)arg, (const void *)uid, sizeof(uid));
                ret = 0;
            }

            break;
        default:
            /* see if the parent class can make any use of it */
            ret = CDev::ioctl(filp, cmd, arg);
            break;
    }

    return ret;
}

RSTCan_Manager::~RSTCan_Manager()
{
    uint8_t node_type, node_idx;
    rstcan_stop_alloc_id_service();

    rstcan_free_node(master_node);

    for(node_type = RSTCAN_NODE_NONE; node_type < RSTCAN_NODE_MASTER; node_type++)
    {
        for(node_idx = 0; node_idx < RSTCAN_MAX_INDEX; node_idx++)
        {
            if(rstcan_node[node_type][node_idx] != nullptr)
            {
                delete rstcan_node[node_type][node_idx]; 
            }
        }
    }
}

RSTCan_Manager *RSTCan_Manager::get_instance(uint8_t port)
{
    return manager_instance[port];    
}

RSTCan_Node *RSTCan_Manager::get_node_instance(uint16_t node_type, uint16_t node_idx)
{
    if(node_type >= RSTCAN_NODE_NR || node_idx >= RSTCAN_MAX_INDEX)
    {
        printf("invalid node node_type=%d node_idx=%d!!\n", node_type, node_idx);
        return nullptr;
    }

    return rstcan_node[node_type][node_idx];
}

void RSTCan_Manager::can_rx(struct rstcan_frame *frame)
{
    uint8_t src_type, src_idx;
    int32_t ret = 0;

    ret = rstcan_put_frame(master_node, frame);

    if(ret > 0 && rstcan_get_msg_source(frame, &src_type, &src_idx) == 0)
    {
        if(rstcan_node[src_type][src_idx] != NULL)
        {
            rstcan_node[src_type][src_idx]->poll_notify(POLLIN);
        }
    }
}

int32_t RSTCan_Manager::_alloc_slave_id()
{
    int ret = -1;
    void *slave_node = NULL;

    ret = rstcan_alloc_id_service(master_node);
   
    ret = rstcan_get_new_node(&slave_node);
    if(ret == 0)
    {
        uint8_t node_type = rstcan_get_node_type(slave_node);
        uint8_t node_idx = rstcan_get_node_idx(slave_node);
        class RSTCan_Node *tmp = nullptr;
        char path[64];
        // XXX 如果有多个相同节点,在dev路径后面加上node_idx作为区分
        memset(path, 0, sizeof(path));
        switch(node_type)
        {
            case RSTCAN_NODE_4G_CONTROL:
                snprintf(path, sizeof(path), "/dev/rstcan_4g_ctrl%d", node_idx);
                tmp = new RSTCan_Node_4G_CTRL("can_4g_ctrl", path, master_node, slave_node, node_idx);
                break;
            case RSTCAN_NODE_USB:
                snprintf(path, sizeof(path), "/dev/rstcan_usb%d", node_idx);
                tmp = new RSTCan_Node_Bypass("can_usb", path, master_node, slave_node, node_idx);
                break;
            case RSTCAN_NODE_4G_TRANS:
                snprintf(path, sizeof(path), "/dev/rstcan_4g_trans%d", node_idx);
                tmp = new RSTCan_Node_Bypass("can_4g_trans", path, master_node, slave_node, node_idx);
                break;
            case RSTCAN_NODE_RGBLED:
                snprintf(path, sizeof(path), "/dev/rstcan_rgbled%d", node_idx);
                tmp = new RSTCan_Node_RgbLed("can_rgbled", path, master_node, slave_node, node_idx);
                break;
            case RSTCAN_NODE_GPS:
                snprintf(path, sizeof(path), "/dev/rstcan_gps%d", node_idx);
                tmp = new RSTCan_Node_Bypass("can_gps", path, master_node, slave_node, node_idx);
                break;
            case RSTCAN_NODE_CAMERA:
                snprintf(path, sizeof(path), "/dev/rstcan_cam%d", node_idx);
                tmp = new RSTCan_Node_Camera("can_camera", path, master_node, slave_node, node_idx);
                break;
            case RSTCAN_NODE_RADAR:
                snprintf(path, sizeof(path), "/dev/rstcan_radar%d", node_idx);
                tmp = new RSTCan_Node_Radar("can_radar", path, master_node, slave_node, node_idx);
                break;
            case RSTCAN_NODE_LANDING_GEAR:
                snprintf(path, sizeof(path), "/dev/rstcan_lg%d", node_idx);
                tmp = new RSTCan_Node_LandingGear("can_landing_gear", path, master_node, slave_node, node_idx);
                break;
            default:
                printf("no such node type:%d\n", node_type);
        }

        if(tmp != NULL)
        {
            if(tmp->init() != OK)
            {
                printf("node %d-%d init failed!!\n", node_type, node_idx);
            }

            rstcan_node[node_type][node_idx] = tmp;
        }
    }

	work_queue(LPWORK,
		   &_work,
		   (worker_t)&RSTCan_Manager::_task_trampoline,
		   this,
		   USEC2TICK(200000));


    return 0;
}

int32_t RSTCan_Manager::_broadcast_all(uint8_t msg_sub_type, uint8_t *buf, uint32_t len)
{
    struct rstcan_msg msg;

    memset(&msg, 0, sizeof(msg));
    //广播发给所有节点
    RSTCAN_BST_MSG_CONSTRUCTOR(msg, msg_sub_type, buf, len);
    rstcan_send_msg(master_node, NULL, &msg);

    return 0;
}

void RSTCan_Manager::_task_trampoline(void *arg)
{
	RSTCan_Manager *dev = (RSTCan_Manager *)arg;
    
    //分配从节点CAN ID
    dev->_alloc_slave_id();


}

int RSTCan_Manager::init()
{
    int ret = -1;

	ret = CDev::init();
    if(ret == -1)
    {
        return -1;
    }

    can_fd = can_drv_init(can_bus);//start can driver
    if(can_fd == nullptr)
    {
        printf("can driver %d init failed this=%p\n", can_bus, this) ;
        return -1;
    }

    master_node = rstcan_node_init(RSTCAN_NODE_MASTER, (char *)"master", &ops, can_fd, true, RSTCAN_NODE_NR);

    _broadcast_all(BL_BST_NODE_BOOT_SUB_MSG, NULL, 0);

    usleep(500000);

	work_queue(LPWORK,
		   &_work,
		   (worker_t)&RSTCan_Manager::_task_trampoline,
		   this,
		   USEC2TICK(10000));

    return 0;
}


namespace rstcan
{

void start();
void test(char *node_name, char *arg);
void info(char *node_name);
void usage();

void start()
{
    int i = 0;
    char *name, *path;

    printf("/*************************************************/\n");
    printf(" * librstcan git version: 0x%x\n", rstcan_get_version());
    printf("/*************************************************/\n");

    for(i=0; i<RSTCAN_PORT_NUM; i++)
    {
        asprintf(&name, "rstcan_manager%d", i);
        asprintf(&path, "%s%d", RSTCAN_MANAGER_DEV_BASE_PATH, i);
        manager_instance[i] = new RSTCan_Manager(name, path, i);
        if(manager_instance[i] == nullptr)
        {
            errx(0, "start rstcan manager failed!!\n");
            free(name);
            free(path);
            goto fail;
        }

        if (OK != manager_instance[i]->init())
        {
            errx(0, "init rstcan manager failed!!\n");
            free(name);
            free(path);
            goto fail;
        }

        free(name);
        free(path);
    }

    exit(0);

fail:

	if (manager_instance[i] != nullptr) {
		delete manager_instance[i];
		manager_instance[i] = nullptr;
	}

	errx(1, "driver start failed");

}

/**
 * Print a little info about the driver.
 */
void info(char *node_name)
{

    for(int i = 0; i < STM32_NCAN; i++)
    {
        if(manager_instance[i] == nullptr)
        {
            printf("manager%d not running\n", i);
            continue;
        }

        printf("manager%d instance = %p\n", i, manager_instance[i]);
        for(int j = 0; j < RSTCAN_NODE_NR; j++)
        {
            for(int k = 0; k < RSTCAN_MAX_INDEX; k++)
            {
                RSTCan_Node *p_node = manager_instance[i]->get_node_instance(j,k);
                if(p_node == nullptr)
                    continue;

                if(node_name == NULL)
                {
                    /*print some error status*/
                    printf(" |-> %s%d:\n", p_node->get_node_name(), p_node->get_node_idx()); 
                }
                else if(strcmp(node_name, p_node->get_node_name()) == 0)
                {
                    /*print node data*/
                    printf(">%s:\n", p_node->get_node_name());
                    p_node->print_data();
                }
            }
        }
    } 


	exit(0);
}

void test(char *node_name, char *arg)
{
    char path[64] = {0};

    if(node_name == NULL)
    {
        printf("please indicate node name\n");
        exit(0);
    }

    for(int i = 0; i < STM32_NCAN; i++)
    {
        if(manager_instance[i] == nullptr)
        {
            continue;
        }
        for(int j = 0; j < RSTCAN_NODE_NR; j++)
        {
            for(int k = 0; k < RSTCAN_MAX_INDEX; k++)
            {
                RSTCan_Node *p_node = manager_instance[i]->get_node_instance(j,k);
                if(p_node == nullptr)
                    continue;
            
                snprintf(path, sizeof(path), "%s%d", p_node->get_node_name(), p_node->get_node_idx());
                if(strcmp(node_name, path) == 0)
                {
                    p_node->test(arg);
                }
            }
        }
    }
	exit(0);
}

void usage()
{
    printf("rstcan start/info [node name]/test [node name] [arg]\n");
    exit(0);
}

}//namespace

int rstcan_main(int argc, char *argv[])
{
	int ch;
    char *node_name = NULL;
    char *arg = NULL;

	/* jump over start/off/etc and look at options first */
	while ((ch = getopt(argc, argv, "XR:")) != EOF) {
		switch (ch) {
		default:
			rstcan::usage();
			exit(0);
		}
	}

	const char *verb = argv[optind];

    //读OTP区域
    val_read((void *)uid, (volatile const void *)ADDR_OTP_START, sizeof(uid));
    ::printf("uid=");
    for(int i = 0; i < (int)sizeof(uid); i++)
    {
        ::printf("0x%x ", uid[i]);
    }
    ::printf("\n");

	/*
	 * Start/load the driver.
	 */
	if (!strcmp(verb, "start"))
    {
		rstcan::start();
    }

	/*
	 * Test the driver/device.
	 */
	if (!strcmp(verb, "test"))
    {
    
        if(argc > 2)
            node_name = argv[optind+1];
        
        if(argc > 3)
            arg = argv[optind+2];

		rstcan::test(node_name, arg);
    }
    /*
	 * Print driver information.
	 */
	if (!strcmp(verb, "info"))
    {
        if(argc > 2)
            node_name = argv[optind+1];

		rstcan::info(node_name);
    }

	if (!strcmp(verb, "usage"))
    {
        if(argc > 2)
            node_name = argv[optind+1];

		rstcan::usage();
    }
	errx(1, "unrecognized command, try 'start', 'test', 'info', 'usage'");
}
