/*********************************************************************************
 *Copyright(C),2015-2017, Robsense Tech. All rights reserved.
 *FileName:    rstcan_node.h
 *Author:      HeBin
 *Version:     0.1
 *Date:        2017-11-10 13:46:53
 *Last Modify: 2017-11-10 13:46:53
 *Description: 
**********************************************************************************/

#pragma once
#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include <semaphore.h>
#include <string.h>
#include <fcntl.h>
#include <poll.h>
#include <errno.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include <getopt.h>

#include <systemlib/perf_counter.h>
#include <systemlib/err.h>

#include <pthread.h>

#include <px4_config.h>
#include <stddef.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_accel.h>
#include <drivers/drv_gps.h>
#include <drivers/drv_gyro.h>
#include <drivers/drv_mag.h>
#include <drivers/drv_baro.h>
#include <drivers/drv_led.h>
#include <drivers/device/ringbuffer.h>
#include <drivers/device/device.h>
#include <drivers/device/device.h>

#include <board_config.h>
#include <mathlib/math/filter/LowPassFilter2p.hpp>
#include <lib/conversion/rotation.h>
#include "rstcan_master.h"
#include "can_proto.h"


class RSTCan_Node : public device::CDev
{
protected:
    void *master_node;
    uint8_t _node_idx;

    virtual int32_t _task();
private:

    hrt_call _call;
    int _task_fd;
    static void _task_trampoline(int argc, char *argv[]);

public:
    void *slave_node;

    RSTCan_Node(const char *name, const char *path, void *_master_node, void *_slave_node, uint8_t idx);
    virtual ~RSTCan_Node();
	virtual int		init();
	virtual int	    ioctl(struct file *filp, int cmd, unsigned long arg);
	virtual ssize_t	write(struct file *filp, const char *buffer, size_t len);
    virtual ssize_t read(struct file *filp, char *buffer, size_t buflen);
	virtual void	poll_notify(pollevent_t events);

    virtual void print_data();
    virtual void test(char *arg);

    void node_task_start(uint32_t stack_size, void *arg);
    void node_task_stop();
    char *get_node_name(){return (char *)_name;}
    uint8_t get_node_idx(){return _node_idx;}
};

class RSTCan_Node_Bypass : public RSTCan_Node
{
public:
    RSTCan_Node_Bypass(const char *name, const char *path, void *_master_node, void *_slave_node, uint8_t idx);
    virtual ~RSTCan_Node_Bypass();
	virtual int		init();
	virtual int	    ioctl(struct file *filp, int cmd, unsigned long arg);
	virtual ssize_t	write(struct file *filp, const char *buffer, size_t len);
    virtual ssize_t read(struct file *filp, char *buffer, size_t buflen);
//	virtual int	poll(file_t *filp, struct pollfd *fds, bool setup);

private: 
    //发送，接收互斥锁
//    sem_t send_sem;
//    sem_t recv_sem;
//bypass类的节点每种类型只接一个,如4G模块和USB节点, 否则需要单独开子类,创建设备节点时加上_class_instance作为后缀
//    int _class_instance; 
};


#define RADIO_4G_CTRL_ACK_OK     "OK"
#define RADIO_4G_CTRL_ACK_ERR    "ERR"

struct radio_4g_status{
    uint8_t bootup;//是否已启动, 1代表已启动完毕,可以发送配置信息
};

class RSTCan_Node_4G_CTRL : public RSTCan_Node
{
public:
    RSTCan_Node_4G_CTRL(const char *name, const char *path, void *_master_node, void *_slave_node, uint8_t idx);
    virtual ~RSTCan_Node_4G_CTRL();
	virtual int		init();

	virtual int	ioctl(struct file *filp, int cmd, unsigned long arg);
//    virtual void print_data();
    virtual void test(char *arg);

    int32_t _wait_ack(uint8_t *buf, uint32_t len);
private:
};

class RSTCan_Node_RgbLed: public RSTCan_Node
{
public:
    RSTCan_Node_RgbLed(const char *name, const char *path, void *_master_node, void *_slave_node, uint8_t idx);
    virtual ~RSTCan_Node_RgbLed();
    virtual int		init();

	virtual int	ioctl(struct file *filp, int cmd, unsigned long arg);
    virtual void test(char *arg);
private:
};

class RSTCan_Node_Camera: public RSTCan_Node
{
private:
    int _orb_sub_fd;
protected:
    virtual int32_t _task();
public:
    RSTCan_Node_Camera(const char *name, const char *path, void *_master_node, void *_slave_node, uint8_t idx);
    virtual ~RSTCan_Node_Camera();
    virtual int	init();

	virtual int	ioctl(struct file *filp, int cmd, unsigned long arg);
    virtual void test(char *arg);
private:
};

class RSTCan_Node_Radar: public RSTCan_Node
{
private:
    orb_advert_t _advert_pub;
protected:
    virtual int32_t _task();
public:
    RSTCan_Node_Radar(const char *name, const char *path, void *_master_node, void *_slave_node, uint8_t idx);
    virtual ~RSTCan_Node_Radar();
    virtual int	init();

	virtual int	ioctl(struct file *filp, int cmd, unsigned long arg);
    virtual void test(char *arg);
};

class RSTCan_Node_Battery: public RSTCan_Node
{
private:
    orb_advert_t _advert_pub;
protected:
    virtual int32_t _task();
public:
    RSTCan_Node_Battery(const char *name, const char *path, void *_master_node, void *_slave_node, uint8_t idx);
    virtual ~RSTCan_Node_Battery();
    virtual int	init();

	virtual int	ioctl(struct file *filp, int cmd, unsigned long arg);
    virtual void test(char *arg);
};

class RSTCan_Node_LandingGear: public RSTCan_Node
{
private:
    int _orb_sub_fd;
protected:
    virtual int32_t _task();
public:
    RSTCan_Node_LandingGear(const char *name, const char *path, void *_master_node, void *_slave_node, uint8_t idx);
    virtual ~RSTCan_Node_LandingGear();
    virtual int	init();

	virtual int	ioctl(struct file *filp, int cmd, unsigned long arg);
    virtual void test(char *arg);
private:
};


