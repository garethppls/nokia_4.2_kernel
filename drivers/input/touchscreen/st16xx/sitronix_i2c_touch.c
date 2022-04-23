/*
 * drivers/input/touchscreen/sitronix_i2c_touch.c
 *
 * Touchscreen driver for Sitronix (I2C bus)
 *
 * Copyright (C) 2011 Sitronix Technology Co., Ltd.
 *	Rudy Huang <rudy_huang@sitronix.com.tw>
 */
/*
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 2 of the License, or (at your option)
 * any later version.
 */
#include <linux/version.h>
#include <linux/module.h>
#include <linux/delay.h>

#if defined(CONFIG_FB)
#include <linux/notifier.h>
#include <linux/fb.h>

#elif defined(CONFIG_HAS_EARLYSUSPEND)
#include <linux/earlysuspend.h>
#endif


#include "sitronix_i2c_touch.h"
#ifdef SITRONIX_FW_UPGRADE_FEATURE
#include <linux/cdev.h>
#include <asm/uaccess.h>
#ifdef SITRONIX_PERMISSION_THREAD
#include <linux/fs.h>
#include <linux/fcntl.h>
#include <linux/syscalls.h>
#endif // SITRONIX_PERMISSION_THREAD
#endif // SITRONIX_FW_UPGRADE_FEATURE
#include <linux/i2c.h>
#include <linux/input.h>
#ifdef SITRONIX_SUPPORT_MT_SLOT
#include <linux/input/mt.h>
#endif // SITRONIX_SUPPORT_MT_SLOT
#include <linux/interrupt.h>
#include <linux/slab.h> // to be compatible with linux kernel 3.2.15
#include <linux/gpio.h>
//#include <mach/gpio.h>
#ifdef SITRONIX_MONITOR_THREAD
#include <linux/kthread.h>
#endif // SITRONIX_MONITOR_THREAD
//#include <linux/sp_capture.h>


#include <linux/kthread.h>
#include <linux/path.h>
#include <linux/namei.h>
#include <linux/of_gpio.h>

#ifdef SITRONIX_MULTI_SLAVE_ADDR
#if defined(CONFIG_MACH_OMAP4_PANDA)
#include <plat/gpio.h>
#endif //defined(CONFIG_MACH_OMAP4_PANDA)
#endif // SITRONIX_MULTI_SLAVE_ADDR

#include<linux/regulator/consumer.h>

#define DRIVER_AUTHOR           "Sitronix, Inc."
#define DRIVER_NAME             "sitronix"
#define DRIVER_DESC             "Sitronix I2C touch"
#define DRIVER_DATE             "20181115"
#define DRIVER_MAJOR            2
#define DRIVER_MINOR         	11
#define DRIVER_PATCHLEVEL       1
#define NAME_SIZE  20

MODULE_AUTHOR("Petitk Kao<petitk_kao@sitronix.com.tw>");
MODULE_DESCRIPTION("Sitronix I2C multitouch panels");
MODULE_LICENSE("GPL");
struct ctp_config_info{
	int input_type;
	char	*name;
	int  int_number;
};


struct ctp_config_info config_info = {
	.input_type = 1,
	.name = NULL,
	.int_number = 0,
};

#define CTP_IRQ_NUMBER          (config_info.int_number)
#define CTP_IRQ_MODE			(IRQF_TRIGGER_FALLING)
#define CTP_NAME		("sitronix")
//#define SCREEN_MAX_X	(screen_max_x)
#define SCREEN_MAX_Y	(screen_max_y)
#define PRESS_MAX		(255)

#define ST_ST1802_RAWTYPE_RAW		0x06
#define ST_ST1802_RAWTYPE_DIST		0x09
#define ST_ST1802_DIST_LENGTH		(4+2*13)

#if 0
static int screen_max_x = 0;
static int screen_max_y = 0;
static int revert_x_flag = 0;
static int revert_y_flag = 0;
static int exchange_x_y_flag = 0;
#endif
#ifdef SITRONIX_MULTI_SLAVE_ADDR
static __u32 twi_id = 0;
#endif

//begin,for lockdown node, fangzhihua.wt, 20191016
struct proc_dir_entry *sitronix_touch_proc_dir;
struct proc_dir_entry *sitronix_proc_SMWP_file = NULL;
#define SITRONIX_PROC_TOUCH 		"touchscreen"
struct proc_dir_entry *sitronix_touch_proc_dir;
#define SITRONIX_PROC_TOUCH_LOCKDOWN 	"lockdown_info"
struct proc_dir_entry *sitronix_proc_lockdown_file = NULL;
//end,for lockdown node, fangzhihua.wt, 20191016
extern char tp_info[40];   //add hardware TP info, wt.fangzhihua,20190924
char sitronix_sensor_key_status = 0;
struct sitronix_sensor_key_t sitronix_sensor_key_array[] = {
	{KEY_MENU}, // bit 2
	{KEY_HOMEPAGE}, // bit 1
	{KEY_BACK}, // bit 0
};
#ifdef SITRONIX_AA_KEY
char sitronix_aa_key_status = 0;

#ifdef SITRONIX_KEY_BOUNDARY_MANUAL_SPECIFY
#define SITRONIX_TOUCH_RESOLUTION_X 480 /* max of X value in display area */
#define SITRONIX_TOUCH_RESOLUTION_Y 854 /* max of Y value in display area */
#define SITRONIX_TOUCH_GAP_Y	10  /* Gap between bottom of display and top of touch key */
#define SITRONIX_TOUCH_MAX_Y 915  /* resolution of y axis of touch ic */
struct sitronix_AA_key sitronix_aa_key_array[] = {
	{15, 105, SITRONIX_TOUCH_RESOLUTION_Y + SITRONIX_TOUCH_GAP_Y, SITRONIX_TOUCH_MAX_Y, KEY_MENU}, /* MENU */
	{135, 225, SITRONIX_TOUCH_RESOLUTION_Y + SITRONIX_TOUCH_GAP_Y, SITRONIX_TOUCH_MAX_Y, KEY_HOME},
	{255, 345, SITRONIX_TOUCH_RESOLUTION_Y + SITRONIX_TOUCH_GAP_Y, SITRONIX_TOUCH_MAX_Y, KEY_BACK}, /* KEY_EXIT */
	{375, 465, SITRONIX_TOUCH_RESOLUTION_Y + SITRONIX_TOUCH_GAP_Y, SITRONIX_TOUCH_MAX_Y, KEY_SEARCH},
};
#else
#define SCALE_KEY_HIGH_Y 15
struct sitronix_AA_key sitronix_aa_key_array[] = {
	{0, 0, 0, 0, KEY_MENU}, /* MENU */
	{0, 0, 0, 0, KEY_HOME},
	{0, 0, 0, 0, KEY_BACK}, /* KEY_EXIT */
	{0, 0, 0, 0, KEY_SEARCH},
};
#endif // SITRONIX_KEY_BOUNDARY_MANUAL_SPECIFY
#endif // SITRONIX_AA_KEY
struct rst_pin_ctrl{
	struct i2c_client *client;
	struct pinctrl		*pinctrl;
	struct pinctrl_state	*rst_pin_ts_active;
	struct pinctrl_state	*rst_pin_ts_suspend;
	struct pinctrl_state	*rst_pin_ts_release;

};

#ifdef SITRONIX_MONITOR_THREAD
	int monitor_thread_flag = 1;
	int monitor_thread_flag_for_sysfs = 1;
#endif
#if (CTP_AP_SP_SYNC_WAY != CTP_AP_SP_SYNC_NONE)
static struct sitronix_ts_data *gSitronixPtr = NULL;
#endif

#if(CTP_AP_SP_SYNC_WAY & CTP_AP_SP_SYNC_GPIO)
//static struct gpio_config sitronix_sync_io;
static struct timer_list ctp_sync_timer;
static int ctp_sync_io_last_status = 1;
static int ctp_sync_pulse_count = 0;
static spinlock_t ctp_sync_lock;
static struct workqueue_struct *sitronix_io_sync_workqueue = NULL;
#endif

static int i2cErrorCount = 0;

#ifdef SITRONIX_MONITOR_THREAD
static struct task_struct * SitronixMonitorThread = NULL;
static int gMonitorThreadSleepInterval = 500; // 0.3 sec
static atomic_t iMonitorThreadPostpone = ATOMIC_INIT(0);

static uint8_t PreCheckData[4] ;
static int StatusCheckCount = 0;
static int sitronix_ts_delay_monitor_thread_start = DELAY_MONITOR_THREAD_START_PROBE; 
static int StatusDistErrCount = 0;
static int StatusCrashFlag = 0;
#endif // SITRONIX_MONITOR_THREAD

#ifdef CONFIG_HAS_EARLYSUSPEND
static void sitronix_ts_early_suspend(struct early_suspend *h);
static void sitronix_ts_late_resume(struct early_suspend *h);
#endif // CONFIG_HAS_EARLYSUSPEND
static int fb_notifier_callback(struct notifier_block *self,
				 unsigned long event, void *data);

struct sitronix_ts_data sitronix_ts_gpts = {0};
//static struct rst_pin_ctrl *rst_pin;
static atomic_t sitronix_ts_irq_on = ATOMIC_INIT(0);
static atomic_t sitronix_ts_in_int = ATOMIC_INIT(0);
#ifdef SITRONIX_SYSFS
static bool sitronix_ts_sysfs_created = false;
static bool sitronix_ts_sysfs_using = false;
#endif // SITRONIX_SYSFS

#ifdef ST_UPGRADE_FIRMWARE
extern int st_upgrade_fw(void);
#endif //ST_UPGRADE_FIRMWARE

#ifdef ST_TEST_RAW
extern int st_testraw_invoke(void);
#endif //ST_TEST_RAW

int st_i2c_read_bytes(u8 addr,u8 *rxbuf,int len);
int st_i2c_write_bytes(st_u8 *txbuf, int len);
static void sitronix_ts_reset_ic_v2(void);

//begin,for lockdown node, fangzhihua.wt, 20191016
static ssize_t sitronix_tplockdown_read(struct file *file, char *buf, size_t len, loff_t *pos)
{
	size_t count = 0;
	char *temp_buf;

	if(*pos) {
		stinf("sitronix_tplockdown_read return\n");
		return 0;
	}
	*pos += len;

	temp_buf = kzalloc(len, GFP_KERNEL);
	count = snprintf(temp_buf, PAGE_SIZE, "%s", "color2_");            // 2:black
	//count = snprintf(temp_buf, PAGE_SIZE, "%s\n", color2_);            // 2:black
        //ret=  sprintf(ptr, "%02X%02X%02X%02X%02X%02X%02X%02X\n",
	//		  lockdown_info[0], lockdown_info[1], lockdown_info[2], lockdown_info[3],
	//		  lockdown_info[4], lockdown_info[5], lockdown_info[6], lockdown_info[7])

	if (copy_to_user(buf, temp_buf, len))
		stinf("%s,here:%d\n", __func__, __LINE__);
	kfree(temp_buf);
	return count;
}

static struct file_operations sitronix_proc_lockdown_ops = {
	.owner = THIS_MODULE,
	.read = sitronix_tplockdown_read,
//	.write = sitronix_tplockdown_write,
};

static void sitronix_tplockdown_node(void)
{
        sitronix_touch_proc_dir = proc_mkdir(SITRONIX_PROC_TOUCH, NULL);         //touchscreen

	if (sitronix_touch_proc_dir == NULL) {
		stinf(" %s: sitronix_touch_proc_dir file create failed!\n", __func__);
	}

	sitronix_proc_lockdown_file = proc_create(SITRONIX_PROC_TOUCH_LOCKDOWN, (S_IWUSR | S_IRUGO | S_IWUGO),       //lockdown_info
									   sitronix_touch_proc_dir, &sitronix_proc_lockdown_ops);
	if (sitronix_proc_lockdown_file == NULL) {
		stinf(" %s: proc lockdown file create failed!\n", __func__);
	}
}
//end,for lockdown node, fangzhihua.wt, 20191016

#ifdef SITRONIX_FW_UPGRADE_FEATURE
#ifdef SITRONIX_PERMISSION_THREAD
SYSCALL_DEFINE3(fchmodat, int, dfd, const char __user *, filename, mode_t, mode);
static struct task_struct * SitronixPermissionThread = NULL;
static int sitronix_ts_delay_permission_thread_start = 1000;


static int sitronix_ts_permission_thread(void *data)
{
	int ret = 0;
	int retry = 0;
	mm_segment_t fs = get_fs();
	set_fs(KERNEL_DS);

	stmsg("%s start\n", __FUNCTION__);
	do{
		stmsg("delay %d ms\n", sitronix_ts_delay_permission_thread_start);
		msleep(sitronix_ts_delay_permission_thread_start);
		ret = sys_fchmodat(AT_FDCWD, "/dev/"SITRONIX_I2C_TOUCH_DEV_NAME , 0600);
		if(ret < 0)
			sterr("fail to execute sys_fchmodat, ret = %d\n", ret);
		if(retry++ > 10)
			break;
	}while(ret == -ENOENT);
	set_fs(fs);
	stmsg("%s exit\n", __FUNCTION__);
	return 0;
}
#endif // SITRONIX_PERMISSION_THREAD

int      sitronix_release(struct inode *, struct file *);
int      sitronix_open(struct inode *, struct file *);
ssize_t  sitronix_write(struct file *file, const char *buf, size_t count, loff_t *ppos);
ssize_t  sitronix_read(struct file *file, char *buf, size_t count, loff_t *ppos);
long	 sitronix_ioctl(struct file *filp, unsigned int cmd, unsigned long arg);
static struct cdev sitronix_cdev;
static struct class *sitronix_class;
static int sitronix_major = 0;

int  sitronix_open(struct inode *inode, struct file *filp)
{
	return 0;
}
EXPORT_SYMBOL(sitronix_open);

int  sitronix_release(struct inode *inode, struct file *filp)
{
	return 0;
}
EXPORT_SYMBOL(sitronix_release);

ssize_t  sitronix_write(struct file *file, const char *buf, size_t count, loff_t *ppos)
{
	int ret;
	char *tmp;

	if(!(sitronix_ts_gpts.client))
		return -EIO;

	if (count > 8192)
		count = 8192;

	tmp = (char *)kmalloc(count,GFP_KERNEL);
	if (tmp==NULL)
		return -ENOMEM;
	if (copy_from_user(tmp,buf,count)) {
		kfree(tmp);
		return -EFAULT;
	}
	stinf("writing %zu bytes.\n", count);

	ret = i2c_master_send(sitronix_ts_gpts.client, tmp, count);
	kfree(tmp);
	return ret;
}
EXPORT_SYMBOL(sitronix_write);

ssize_t  sitronix_read(struct file *file, char *buf, size_t count, loff_t *ppos)
{
	char *tmp;
	int ret;

	if(!(sitronix_ts_gpts.client))
		return -EIO;

	if (count > 8192)
		count = 8192;

	tmp = (char *)kmalloc(count,GFP_KERNEL);
	if (tmp==NULL)
		return -ENOMEM;

	stinf("reading %zu bytes.\n", count);

	ret = i2c_master_recv(sitronix_ts_gpts.client, tmp, count);
	if (ret >= 0)
		ret = copy_to_user(buf,tmp,count)?-EFAULT:ret;
	kfree(tmp);
	return ret;
}
EXPORT_SYMBOL(sitronix_read);

static int sitronix_ts_resume(struct i2c_client *client);
static int sitronix_ts_suspend(struct i2c_client *client);
void sitronix_ts_reprobe(void);
long	 sitronix_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int err = 0;
	int retval = 0;
	uint8_t temp[4];

	if (!(sitronix_ts_gpts.client))
		return -EIO;

	if (_IOC_TYPE(cmd) != SMT_IOC_MAGIC) return -ENOTTY;
	if (_IOC_NR(cmd) > SMT_IOC_MAXNR) return -ENOTTY;
	if (_IOC_DIR(cmd) & _IOC_READ)
		err = !access_ok(VERIFY_WRITE,(void __user *)arg,\
				 _IOC_SIZE(cmd));
	else if (_IOC_DIR(cmd) & _IOC_WRITE)
		err =  !access_ok(VERIFY_READ,(void __user *)arg,\
				  _IOC_SIZE(cmd));
	if (err) return -EFAULT;

	switch(cmd) {
		case IOCTL_SMT_GET_DRIVER_REVISION:
			stinf("IOCTL_SMT_GET_DRIVER_REVISION\n");
			temp[0] = SITRONIX_TOUCH_DRIVER_VERSION;
			if(copy_to_user((uint8_t __user *)arg, &temp[0], 1)){
				stinf("fail to get driver version\n");
				retval = -EFAULT;
			}
			break;
		case IOCTL_SMT_GET_FW_REVISION:
			stinf("IOCTL_SMT_GET_FW_REVISION\n");
			if(copy_to_user((uint8_t __user *)arg, &(sitronix_ts_gpts.fw_revision[0]), 4))
					retval = -EFAULT;
			break;
		case IOCTL_SMT_ENABLE_IRQ:
			stinf("IOCTL_SMT_ENABLE_IRQ\n");
			if(!atomic_read(&sitronix_ts_in_int)){
				if(!atomic_read(&sitronix_ts_irq_on)){
					atomic_set(&sitronix_ts_irq_on, 1);
					enable_irq(sitronix_ts_gpts.client->irq);
#ifdef SITRONIX_MONITOR_THREAD
					if(sitronix_ts_gpts.enable_monitor_thread == 1){
						if(!SitronixMonitorThread){
							atomic_set(&iMonitorThreadPostpone,1);
							SitronixMonitorThread = kthread_run(sitronix_ts_gpts.sitronix_mt_fp,"Sitronix","Monitorthread");
							if(IS_ERR(SitronixMonitorThread))
								SitronixMonitorThread = NULL;
						}
					}
#endif // SITRONIX_MONITOR_THREAD
				}
			}
			break;
		case IOCTL_SMT_DISABLE_IRQ:
			stinf("IOCTL_SMT_DISABLE_IRQ\n");
#ifndef SITRONIX_INT_POLLING_MODE
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,37)
//gavin			flush_work_sync(&sitronix_ts_gpts.work);
#endif // LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,37)
#else
			cancel_delayed_work_sync(&sitronix_ts_gpts.work);
#endif
			if(atomic_read(&sitronix_ts_irq_on)){
				atomic_set(&sitronix_ts_irq_on, 0);
				disable_irq_nosync(sitronix_ts_gpts.client->irq);
#ifdef SITRONIX_MONITOR_THREAD
				if(sitronix_ts_gpts.enable_monitor_thread == 1){
					if(SitronixMonitorThread){
						kthread_stop(SitronixMonitorThread);
						SitronixMonitorThread = NULL;
					}
				}
#endif // SITRONIX_MONITOR_THREAD
			}
			break;
		case IOCTL_SMT_RESUME:
			stinf("IOCTL_SMT_RESUME\n");
			sitronix_ts_resume(sitronix_ts_gpts.client);
			break;
		case IOCTL_SMT_SUSPEND:
			stinf("IOCTL_SMT_SUSPEND\n");
			sitronix_ts_suspend(sitronix_ts_gpts.client);
			break;
		case IOCTL_SMT_HW_RESET:
			stinf("IOCTL_SMT_HW_RESET\n");
			sitronix_ts_reset_ic_v2();
			break;
		case IOCTL_SMT_REPROBE:
			stinf("IOCTL_SMT_REPROBE\n");
			sitronix_ts_reprobe();
			break;
#ifdef ST_TEST_RAW			
		case IOCTL_SMT_RAW_TEST:
			stinf("IOCTL_SMT_RAW_TEST\n");
			retval = - st_testraw_invoke();			
			break;
#endif			
		default:
			retval = -ENOTTY;
	}

	return retval;
}
EXPORT_SYMBOL(sitronix_ioctl);
#endif // SITRONIX_FW_UPGRADE_FEATURE

#ifdef SITRONIX_MONITOR_THREAD
static void sitronix_ts_reset_ic_v2(void)
{
	stmsg("%s\n", __FUNCTION__);
//	gpio_direction_output(sitronix_ts_gpts.irq_gpio, 1);
//	msleep(10);	
	gpio_direction_output(sitronix_ts_gpts.reset_gpio, 1);
	msleep(10);
	gpio_direction_output(sitronix_ts_gpts.reset_gpio, 0);
	msleep(10);
	gpio_direction_output(sitronix_ts_gpts.reset_gpio, 1);
	msleep(150);
//	gpio_direction_input(sitronix_ts_gpts.irq_gpio);
}
#endif

static int sitronix_i2c_read_bytes(struct i2c_client *client, u8 addr, u8 *rxbuf, int len)
{
	int ret = 0;
	u8 txbuf = addr;
	//stmsg("%s,line=%d\n", __FUNCTION__,__LINE__);
#if defined(SITRONIX_I2C_COMBINED_MESSAGE)
	struct i2c_msg msg[2] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = 1,
			.buf = &txbuf,
		},
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = len,
			.buf = rxbuf,
		},
	};
#endif // defined(SITRONIX_I2C_COMBINED_MESSAGE)

	if(rxbuf == NULL)
		return -1;
#if defined(SITRONIX_I2C_COMBINED_MESSAGE)
	ret = i2c_transfer(client->adapter, &msg[0], 2);
#elif defined(SITRONIX_I2C_SINGLE_MESSAGE)
	ret = i2c_master_send(client, &txbuf, 1);
	if (ret < 0){
		stmsg("write 0x%x error (%d)\n", addr, ret);
		return ret;
	}
	ret = i2c_master_recv(client, rxbuf, len);
#endif // defined(SITRONIX_I2C_COMBINED_MESSAGE)
	if (ret < 0){
		stmsg("read 0x%x error (%d)\n", addr, ret);
		return ret;
	}
	return 0;
}
#if defined(SITRONIX_IDENTIFY_ID) || defined(SITRONIX_MONITOR_THREAD) || defined(SITRONIX_SMART_WAKE_UP)
static int sitronix_i2c_write_bytes(struct i2c_client *client, u8 *txbuf, int len)
{
	int ret = 0;
	//stmsg("%s,line=%d\n", __FUNCTION__,__LINE__);
#if defined(SITRONIX_I2C_COMBINED_MESSAGE)
	struct i2c_msg msg[1] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = len,
			.buf = txbuf,
		},
	};
#endif // defined(SITRONIX_I2C_COMBINED_MESSAGE)

	if(txbuf == NULL)
		return -1;
#if defined(SITRONIX_I2C_COMBINED_MESSAGE)
	ret = i2c_transfer(client->adapter, &msg[0], 1);
#elif defined(SITRONIX_I2C_SINGLE_MESSAGE)
	ret = i2c_master_send(client, txbuf, len);
#endif // defined(SITRONIX_I2C_COMBINED_MESSAGE)
	if (ret < 0){
		stmsg("write 0x%x error (%d)\n", *txbuf, ret);
		return ret;
	}
	return 0;
}
#endif

static int sitronix_get_fw_revision(struct sitronix_ts_data *ts)
{
	int ret = 0;
	uint8_t buffer[4];
	stmsg("%s,line=%d\n", __FUNCTION__,__LINE__);

	ret = sitronix_i2c_read_bytes(ts->client, FIRMWARE_REVISION_3, buffer, 4);
	if (ret < 0){
		sterr("read fw revision error (%d)\n", ret);
		return ret;
	}else{
		memcpy(ts->fw_revision, buffer, 4);
		stmsg("fw revision (hex) = %x %x %x %x\n", buffer[0], buffer[1], buffer[2], buffer[3]);
	}
	return 0;
}

static int sitronix_get_fw_version(struct sitronix_ts_data *ts)
{
	int ret = 0;
	uint8_t buffer[2];
	stmsg("%s,line=%d\n", __FUNCTION__,__LINE__);

	ret = sitronix_i2c_read_bytes(ts->client, FIRMWARE_VERSION, buffer, 1);
	if (ret < 0){
		sterr("read fw revision error (%d)\n", ret);
		return ret;
	}else{
		memcpy(ts->fw_version, buffer, 1);
		stmsg("fw revision (hex) = %x\n", buffer[0]);
	}
	return 0;
}
static int sitronix_get_max_touches(struct sitronix_ts_data *ts)
{
	int ret = 0;
	uint8_t buffer[1];
	stmsg("%s,line=%d\n", __FUNCTION__,__LINE__);

	ret = sitronix_i2c_read_bytes(ts->client, MAX_NUM_TOUCHES, buffer, 1);
	if (ret < 0){
		sterr("read max touches error (%d)\n", ret);
		return ret;
	}else{
		ts->max_touches = buffer[0];
		if (ts->max_touches > SITRONIX_MAX_SUPPORTED_POINT)
			ts->max_touches = SITRONIX_MAX_SUPPORTED_POINT;
		stmsg("max touches = %d \n",ts->max_touches);
	}
	return 0;
}

static int sitronix_get_protocol_type(struct sitronix_ts_data *ts)
{
	int ret = 0;
	uint8_t buffer[1];
	stmsg("%s,line=%d\n", __FUNCTION__,__LINE__);

	if(ts->chip_id <= 3){
		ret = sitronix_i2c_read_bytes(ts->client, I2C_PROTOCOL, buffer, 1);
		if (ret < 0){
			sterr("read i2c protocol error (%d)\n", ret);
			return ret;
		}else{
			ts->touch_protocol_type = buffer[0] & I2C_PROTOCOL_BMSK;
			stmsg("i2c protocol = %d \n", ts->touch_protocol_type);
			ts->sensing_mode = (buffer[0] & (ONE_D_SENSING_CONTROL_BMSK << ONE_D_SENSING_CONTROL_SHFT)) >> ONE_D_SENSING_CONTROL_SHFT;
			stmsg("sensing mode = %d \n", ts->sensing_mode);
		}
	}else{
		ts->touch_protocol_type = SITRONIX_A_TYPE;
		stmsg("i2c protocol = %d \n", ts->touch_protocol_type);
		ret = sitronix_i2c_read_bytes(ts->client, 0xf0, buffer, 1);
		if (ret < 0){
			sterr("read sensing mode error (%d)\n", ret);
			return ret;
		}else{
			ts->sensing_mode = (buffer[0] & ONE_D_SENSING_CONTROL_BMSK);
			stmsg("sensing mode = %d \n", ts->sensing_mode);
		}
	}
	return 0;
}

static int sitronix_get_resolution(struct sitronix_ts_data *ts)
{
	int ret = 0;
	uint8_t buffer[4];
		stmsg("%s,line=%d\n", __FUNCTION__,__LINE__);
	ret = sitronix_i2c_read_bytes(ts->client, XY_RESOLUTION_HIGH, buffer, 3);
	if (ret < 0){
		sterr("read resolution error (%d)\n", ret);
		return ret;
	}else{
		ts->resolution_x = ((buffer[0] & (X_RES_H_BMSK << X_RES_H_SHFT)) << 4) | buffer[1];
		ts->resolution_y = ((buffer[0] & Y_RES_H_BMSK) << 8) | buffer[2];
		stmsg("resolution = %d x %d\n", ts->resolution_x, ts->resolution_y);
	}
	return 0;
	
}

static int sitronix_ts_get_CHIP_ID(struct sitronix_ts_data *ts)
{
	int ret = 0;
	uint8_t buffer[3];
	stmsg("%s,line=%d\n", __FUNCTION__,__LINE__);

	stmsg("%s\n", __FUNCTION__);

	ret = sitronix_i2c_read_bytes(ts->client, CHIP_ID, buffer, 3);
	if (ret < 0){
		sterr("read Chip ID error (%d)\n", ret);
		return ret;
	}else{
		if(buffer[0] == 0){
			if(buffer[1] + buffer[2] > 32)
				ts->chip_id = 2;
			else
				ts->chip_id = 0;
		}else
			ts->chip_id = buffer[0];
		ts->Num_X = buffer[1];
		ts->Num_Y = buffer[2];
		stinf("Chip ID = %d\n", ts->chip_id);
		stinf("Num_X = %d\n", ts->Num_X);
		stinf("Num_Y = %d\n", ts->Num_Y);
	}

	return 0;
}

static int sitronix_ts_get_touch_info(struct sitronix_ts_data *ts)
{
	int ret = 0;
		stmsg("%s,line=%d\n", __FUNCTION__,__LINE__);
	ret = sitronix_get_resolution(ts);
	if(ret < 0)
		return ret;
	ret = sitronix_ts_get_CHIP_ID(ts);
	if(ret < 0)
		return ret;
	ret = sitronix_get_fw_revision(ts);
	if(ret < 0)
		return ret;
	ret = sitronix_get_fw_version(ts);
	if(ret < 0)
		return ret;
	ret = sitronix_get_protocol_type(ts);
	if(ret < 0)
		return ret;
	ret = sitronix_get_max_touches(ts);
	if(ret < 0)
		return ret;

	if((ts->fw_revision[0] == 0) && (ts->fw_revision[1] == 0)){
		if(ts->touch_protocol_type == SITRONIX_RESERVED_TYPE_0){
			ts->touch_protocol_type = SITRONIX_B_TYPE;
			stinf("i2c protocol (revised) = %d \n", ts->touch_protocol_type);
		}
	}
	if(ts->touch_protocol_type == SITRONIX_A_TYPE)
		ts->pixel_length = PIXEL_DATA_LENGTH_A;
	else if(ts->touch_protocol_type == SITRONIX_B_TYPE){
		ts->pixel_length = PIXEL_DATA_LENGTH_B;
		ts->max_touches = 2;
		stinf("max touches (revised) = %d \n", ts->max_touches);
	}

#ifdef SITRONIX_MONITOR_THREAD
	ts->RawCRC_enabled = 0;
	if(ts->chip_id > 3){
		ts->enable_monitor_thread = 1;
		ts->RawCRC_enabled = 1;
	}else if(ts->chip_id == 3){
		ts->enable_monitor_thread = 1;		
		//if(((ts->fw_revision[2] << 8) | ts->fw_revision[3]) >= (9 << 8 | 3))
		if(((ts->fw_revision[2] << 8) | ts->fw_revision[3]) >= (6 << 8 | 3))
			ts->RawCRC_enabled = 1;
	}else
		ts->enable_monitor_thread = 0;
#endif // SITRONIX_MONITOR_THREAD

	return 0;
}

static int sitronix_ts_get_device_status(struct i2c_client *client, uint8_t *err_code, uint8_t *dev_status)
{
	int ret = 0;
	uint8_t buffer[8];
	stmsg("%s,line=%d\n",__FUNCTION__,__LINE__);
	stmsg("%s\n", __FUNCTION__);
	ret = sitronix_i2c_read_bytes(client, STATUS_REG, buffer, 8);
	if (ret < 0){
		sterr("sitronix read status reg error (%d)\n", ret);
		return ret;
	}else{
		stinf("sitronix status reg = %d \n", buffer[0]);
	}

	*err_code = (buffer[0] & 0xf0) >> 4;
	*dev_status = buffer[0] & 0xf;

	return 0;
}

#ifdef SITRONIX_IDENTIFY_ID
static int sitronix_ts_Enhance_Function_control(struct sitronix_ts_data *ts, uint8_t *value)
{
	int ret = 0;
	uint8_t buffer[1];

	stmsg("%s\n", __FUNCTION__);
	ret = sitronix_i2c_read_bytes(ts->client, 0xF0, buffer, 1);
	if (ret < 0){
		sterr("read Enhance Functions status error (%d)\n", ret);
		return ret;
	}else{
		stinf("Enhance Functions status = %d \n", buffer[0]);
	}

	*value = buffer[0] & 0x4;

	return 0;
}

static int sitronix_ts_FW_Bank_Select(struct sitronix_ts_data *ts, uint8_t value)
{
	int ret = 0;
	uint8_t buffer[2];

	stmsg("%s\n", __FUNCTION__);
	ret = sitronix_i2c_read_bytes(ts->client, 0xF1, buffer, 1);
	if (ret < 0){
		sterr("read FW Bank Select status error (%d)\n", ret);
		return ret;
	}else{
		stinf("FW Bank Select status = %d \n", buffer[0]);
	}

	buffer[1] = ((buffer[0] & 0xfc) | value);
	buffer[0] = 0xF1;
	ret = sitronix_i2c_write_bytes(ts->client, buffer, 2);
	if (ret < 0){
		sterr("send FW Bank Select command error (%d)\n", ret);
		return ret;
	}

	return 0;
}

static int sitronix_get_id_info(struct sitronix_ts_data *ts, uint8_t *id_info)
{
	int ret = 0;
	uint8_t buffer[4];

	ret = sitronix_i2c_read_bytes(ts->client, 0x0C, buffer, 4);
	if (ret < 0){
		sterr("read id info error (%d)\n", ret);
		return ret;
	}else{
		memcpy(id_info, buffer, 4);
	}
	return 0;
}

static int sitronix_ts_identify(struct sitronix_ts_data *ts)
{
	int ret = 0;
	uint8_t id[4];
	uint8_t Enhance_Function = 0;

	ret = sitronix_ts_FW_Bank_Select(ts, 1);
	if(ret < 0)
		return ret;
	ret = sitronix_ts_Enhance_Function_control(ts, &Enhance_Function);
	if(ret < 0)
		return ret;
	if(Enhance_Function == 0x4){
		ret = sitronix_get_id_info(ts, &id[0]);
		if(ret < 0)
			return ret;
		stinf("id (hex) = %x %x %x %x\n", id[0], id[1], id[2], id[3]);
		if((id[0] == 1)&&(id[1] == 2)&&(id[2] == 0xb)&&(id[3] == 1)){
			return 0;
		}else{
			sterr("Error: It is not Sitronix IC\n");
			return -1;
		}
	}else{
		sterr("Error: Can not get ID of Sitronix IC\n");
		return -1;
	}
}
#endif // SITRONIX_IDENTIFY_ID

#ifdef SITRONIX_MONITOR_THREAD
static int sitronix_set_raw_data_type(struct sitronix_ts_data *ts)
{
	int ret = 0;
	uint8_t buffer[2] = {0};

	ret = sitronix_i2c_read_bytes(ts->client, DEVICE_CONTROL_REG, buffer, 1);
	if (ret < 0){
		sterr("read DEVICE_CONTROL_REG error (%d)\n", ret);
		return ret;
	}else{
		stmsg("read DEVICE_CONTROL_REG status = %d \n", buffer[0]);
	}
	if(ts->sensing_mode == SENSING_BOTH_NOT){
		buffer[1] = ((buffer[0] & 0xf3) | (0x01 << 2));
	}else{
		buffer[1] = (buffer[0] & 0xf3);
	}
	buffer[0] = DEVICE_CONTROL_REG;
	ret = sitronix_i2c_write_bytes(ts->client, buffer, 2);
	if (ret < 0){
		sterr("write DEVICE_CONTROL_REG error (%d)\n", ret);
		return ret;
	}
	return 0;
}

static int sitronix_ts_monitor_thread(void *data)
{
	int ret = 0;
	uint8_t buffer[4] = { 0, 0, 0, 0 };
	int result = 0;
	int once = 1;
	uint8_t raw_data_ofs = 0;

	stmsg("%s:\n", __FUNCTION__);

	stmsg("delay %d ms\n", sitronix_ts_delay_monitor_thread_start);
	msleep(sitronix_ts_delay_monitor_thread_start);
	while(!kthread_should_stop()){
		stmsg("%s:\n", "Sitronix_ts_monitoring 2222");
		if(atomic_read(&iMonitorThreadPostpone)){
		 		atomic_set(&iMonitorThreadPostpone,0);
		}else{
			if(once == 1){
				ret = sitronix_set_raw_data_type(&sitronix_ts_gpts);
				if (ret < 0)
					goto exit_i2c_invalid;

				if((sitronix_ts_gpts.sensing_mode == SENSING_BOTH) || (sitronix_ts_gpts.sensing_mode == SENSING_X_ONLY)){
					raw_data_ofs = 0x40;
				}else if(sitronix_ts_gpts.sensing_mode == SENSING_Y_ONLY){
					raw_data_ofs = 0x40 + sitronix_ts_gpts.Num_X * 2; 
				}else{
					raw_data_ofs = 0x40;
				}

				once = 0;
			}
			if(raw_data_ofs != 0x40){
				ret = sitronix_i2c_read_bytes(sitronix_ts_gpts.client, 0x40, buffer, 1);
				if (ret < 0){
					sterr("read raw data error (%d)\n", ret);
					result = 0;
					goto exit_i2c_invalid;
				}
			}
			ret = sitronix_i2c_read_bytes(sitronix_ts_gpts.client, raw_data_ofs, buffer, 4);
			if (ret < 0){
				sterr("read raw data error (%d)\n", ret);
				result = 0;
				goto exit_i2c_invalid;
			}else{
				stinf("%dD data h%x-%x = 0x%02x, 0x%02x, 0x%02x, 0x%02x\n", (sitronix_ts_gpts.sensing_mode == SENSING_BOTH_NOT ? 2:1), raw_data_ofs, raw_data_ofs + 3, buffer[0], buffer[1], buffer[2], buffer[3]);
				//stmsg("%dD data h%x-%x = 0x%02x, 0x%02x, 0x%02x, 0x%02x\n", (sitronix_ts_gpts.sensing_mode == SENSING_BOTH_NOT ? 2:1), raw_data_ofs, raw_data_ofs + 3, buffer[0], buffer[1], buffer[2], buffer[3]);
				result = 1;
				if ((PreCheckData[0] == buffer[0]) && (PreCheckData[1] == buffer[1]) && 
				(PreCheckData[2] == buffer[2]) && (PreCheckData[3] == buffer[3]))
					StatusCheckCount ++;
				else
					StatusCheckCount =0;
				PreCheckData[0] = buffer[0];
				PreCheckData[1] = buffer[1];
				PreCheckData[2] = buffer[2];
				PreCheckData[3] = buffer[3];
				if (3 <= StatusCheckCount){
					sterr("IC Status doesn't update! \n");
					result = -1;
					StatusCheckCount = 0;
				}
			}
			if (-1 == result){
				sterr("Chip abnormal, reset it!\n");
				sitronix_ts_reset_ic_v2();
		   		i2cErrorCount = 0;
		   		StatusCheckCount = 0;
				if(sitronix_ts_gpts.RawCRC_enabled == 0){
					ret = sitronix_set_raw_data_type(&sitronix_ts_gpts);
					if (ret < 0)
						goto exit_i2c_invalid;
				}
			}
exit_i2c_invalid:
			if(0 == result){
				i2cErrorCount ++;
				if ((2 <= i2cErrorCount)){
					sterr("I2C abnormal, reset it!\n");
					sitronix_ts_reset_ic_v2();
					if(sitronix_ts_gpts.RawCRC_enabled == 0)
						sitronix_set_raw_data_type(&sitronix_ts_gpts);
		    			i2cErrorCount = 0;
		    			StatusCheckCount = 0;
		    		}
		    	}else
		    		i2cErrorCount = 0;
		}
		msleep(gMonitorThreadSleepInterval);
	}
	sterr("%s exit\n", __FUNCTION__);
	return 0;
}

void get_dist_value(void)
{
	unsigned char rbuf[0x30];
	int i;
	signed short rawI;
	bool err_flag;
	st_i2c_read_bytes(0x40, rbuf,ST_ST1802_DIST_LENGTH);
	stmsg("%s rbuf[0] = 0x%x in \n",__func__,rbuf[0]);
	
	if(rbuf[0] == ST_ST1802_RAWTYPE_RAW)
	{
		err_flag = false;
		for(i=0;i<13;i++)
		{
			rawI = (signed short)((rbuf[4+2*i])*0x100 + rbuf[5+2*i]);
			if((rawI>700)  ||  (rawI <(-700)))
			{
				err_flag = true;
				break;
			}
		}
		if(err_flag == true)
		{
			StatusDistErrCount++;
			stmsg("%s StatusDistErrCount = %d \n",__func__,StatusDistErrCount);
		}
		else
			StatusDistErrCount=0;
	}else if(rbuf[0] == ST_ST1802_RAWTYPE_DIST)
	{
		if(rbuf[8] & 0x10)
			StatusCrashFlag = 1;
		else
			StatusCrashFlag = 0;
		
		stmsg("%s rbuf %x %x \n",__func__,rbuf[8],(rbuf[8] & 0x10));
	}
	
}

static int sitronix_ts_monitor_thread_v2(void *data)
{
	int ret = 0;
	uint8_t buffer[8] = {0};
	int result = 0;
//    int i = 0;
	stmsg("%s:\n", __FUNCTION__);

	stmsg("delay %d ms\n", sitronix_ts_delay_monitor_thread_start);	
	msleep(sitronix_ts_delay_monitor_thread_start);
	
	while(!kthread_should_stop()){
		stmsg("%s in\n", __func__);
		if(atomic_read(&iMonitorThreadPostpone)){
		 		atomic_set(&iMonitorThreadPostpone,0);
				stmsg("%s atomic close \n",__func__);
		}else if((0 == monitor_thread_flag) || (0 == monitor_thread_flag_for_sysfs)){
			stmsg("%s close mt \n",__func__);
		}else{
			ret = sitronix_i2c_read_bytes(sitronix_ts_gpts.client, 0x01, buffer, 8);
			if (ret < 0){
				stmsg("read Raw CRC error (%d)\n", ret);
				result = 0;
				goto exit_i2c_invalid;
			}else{
//HF-ST add
				if((buffer[0]&0xF) == 0x6)
				{
					sterr("%s bootcode \n",__func__);
//					for(i = 0;i<30;i++)
//					{
//					msleep(1000);
//					}
					result = 0;
					goto exit_i2c_invalid;
					
				}else if(((buffer[0]&0xFF) != 0xFF)&&((buffer[2]&0xFF) == 0xFE)){
					monitor_thread_flag_for_sysfs = 0;
					result = 1;
					sterr("%s ic status occur one err \n",__func__);
					goto exit_i2c_invalid;
				}
//HF-ST end				
				stmsg("Raw CRC = 0x%02x\n", (buffer[6]+buffer[7]));
				//stmsg("Raw CRC = 0x%02x\n", buffer[0]);
				result = 1;
				if ((PreCheckData[0] == buffer[6])&&(PreCheckData[1] == buffer[7]))
					StatusCheckCount ++;
				else
					StatusCheckCount =0;
				PreCheckData[0] = buffer[6];
				PreCheckData[1] = buffer[7];
				if (3 <= StatusCheckCount){
					sterr("IC Status doesn't update! \n");
					result = -1;
					StatusCheckCount = 0;
				}
				
				get_dist_value();
				if (3 <= StatusDistErrCount){
					sterr("sitronix dist IC Status doesn't update! \n");
					result = -1;
					StatusDistErrCount = 0;
				}
			}
			if ((-1 == result) || (1 == StatusCrashFlag)){
				sterr("Chip abnormal, reset it!\n");
		   		sitronix_ts_reset_ic_v2();
				i2cErrorCount = 0;
		   		StatusCheckCount = 0;
				StatusCrashFlag = 0;
			}
exit_i2c_invalid:
			if(0 == result){
				i2cErrorCount ++;
				if ((2 <= i2cErrorCount)){
					sterr("I2C abnormal, reset it!\n");
					sitronix_ts_reset_ic_v2();
		    			i2cErrorCount = 0;
		    			StatusCheckCount = 0;
		    		}
		    	}else
		    		i2cErrorCount = 0;
		}
		msleep(gMonitorThreadSleepInterval);
	}
	stmsg("%s exit\n", __FUNCTION__); //temp
	return 0;
}
#endif // SITRONIX_MONITOR_THREAD

static inline void sitronix_ts_pen_down(struct input_dev *input_dev, int id, u16 x, u16 y)
{
#ifdef SITRONIX_SUPPORT_MT_SLOT
	input_mt_slot(input_dev, id); 
	input_mt_report_slot_state(input_dev, MT_TOOL_FINGER, 1); 
	input_report_key(input_dev, BTN_TOUCH, 1);
	input_report_abs(input_dev,ABS_MT_PRESSURE,255);
	input_report_abs(input_dev, ABS_MT_POSITION_X, x); 
	input_report_abs(input_dev, ABS_MT_POSITION_Y, y); 
#else
	input_report_abs(input_dev,  ABS_MT_TRACKING_ID, id+1);
	input_report_abs(input_dev,  ABS_MT_POSITION_X, x);
	input_report_abs(input_dev,  ABS_MT_POSITION_Y, y);
	input_report_abs(input_dev,  ABS_X, x);
	input_report_abs(input_dev,  ABS_Y, y);
	input_report_abs(input_dev,  ABS_MT_TOUCH_MAJOR, 255);
	input_report_abs(input_dev,  ABS_MT_WIDTH_MAJOR, 255);
	input_report_abs(input_dev, ABS_MT_PRESSURE, 255);

	input_mt_sync(input_dev);
#endif
//	stmsg("sitronix: [%d](%d, %d)+\n", id, x, y);
}

static inline void sitronix_ts_pen_up(struct input_dev *input_dev, int id)
{
#ifdef SITRONIX_SUPPORT_MT_SLOT
	input_mt_slot(input_dev, id);
	input_mt_report_slot_state(input_dev, MT_TOOL_FINGER, 0); 
	input_report_key(input_dev, BTN_TOUCH, 0);
	input_report_abs(input_dev,ABS_MT_PRESSURE,0);
#else
	input_report_abs(input_dev,  ABS_MT_TRACKING_ID, id);
	input_report_abs(input_dev,  ABS_MT_TOUCH_MAJOR, 0);
	input_report_abs(input_dev,  ABS_MT_WIDTH_MAJOR, 0);
	input_report_abs(input_dev, ABS_MT_PRESSURE, 0);
#endif
	stmsg("[%d]-\n", id);
}

static inline void sitronix_ts_handle_sensor_key(struct input_dev *input_dev, struct sitronix_sensor_key_t *key_array, char *pre_key_status, char cur_key_status, int key_count)
{
	
	int i = 0;
	for(i = 0; i < key_count; i++){
		if(cur_key_status & (1 << i)){
			stmsg("lpz sensor key[%d] down\n", i);
			stmsg("kkk down now key %d \n",cur_key_status);
			input_report_key(input_dev, key_array[i].code, 1);

			input_sync(input_dev);
		}else{
			if(*pre_key_status & (1 << i)){
				//stmsg("kkk up now key %d \n",cur_key_status);
				stmsg("lpz sensor key[%d] up\n", i);
				input_report_key(input_dev, key_array[i].code, 0);				
				input_sync(input_dev);
			}
		}
	}
	*pre_key_status = cur_key_status;
}

#ifdef SITRONIX_AA_KEY
static inline void sitronix_ts_handle_aa_key(struct input_dev *input_dev, struct sitronix_AA_key *key_array, char *pre_key_status, char cur_key_status, int key_count)
{
	
	int i = 0;
	for(i = 0; i < key_count; i++){
		if(cur_key_status & (1 << i)){
			stmsg("aa key[%d] down\n", i);
			input_report_key(input_dev, key_array[i].code, 1);
			input_sync(input_dev);
		}else{
			if(*pre_key_status & (1 << i)){
				stmsg("aa key[%d] up\n", i);
				input_report_key(input_dev, key_array[i].code, 0);
				input_sync(input_dev);
			}
		}
	}
	*pre_key_status = cur_key_status;
}
#endif // SITRONIX_AA_KEY

#ifdef SITRONIX_GESTURE

static void sitronix_gesture_func(struct input_dev *input_dev,int id)
{
	if(id == G_PALM)
	{
		stmsg("Gesture for Palm to suspend \n");
		input_report_key(input_dev,KEY_POWER,1);// KEY_LEFT, 1);
		input_sync(input_dev);
		input_report_key(input_dev, KEY_POWER, 0);
		input_sync(input_dev);				
	}
}



#endif

#ifdef SITRONIX_SMART_WAKE_UP
static int swk_flag = 0;
static void sitronix_swk_set_swk_enable(struct sitronix_ts_data *ts)
{
	
	int ret = 0;
	unsigned char buffer[2] = {0};
	ret = sitronix_i2c_read_bytes(ts->client, MISC_CONTROL, buffer, 1);
	if(ret == 0)
	{
		buffer[1] = buffer[0] | 0x80;
		buffer[0] = MISC_CONTROL;
		sitronix_i2c_write_bytes(ts->client, buffer, 2);
				
		msleep(500);
	}
	
	
}
static void sitronix_swk_func(struct input_dev *input_dev, int id)
{
	if(id == DOUBLE_CLICK || id == SINGLE_CLICK)
	{
		if(swk_flag == 1)
		{
			//do wake up here
			stmsg("Smark Wake Up by Double click! \n");
			input_report_key(input_dev, KEY_POWER, 1);
			input_sync(input_dev);
			input_report_key(input_dev, KEY_POWER, 0);
			input_sync(input_dev);
			swk_flag = 0;
		}
	}
	else if(id == TOP_TO_DOWN_SLIDE)
	{
		stmsg("Smark Wake Up by TOP_TO_DOWN_SLIDE \n");
		//do wake up here		
	}
	else if(id == DOWN_TO_UP_SLIDE)
	{
		stmsg("Smark Wake Up by DOWN_TO_UP_SLIDE \n");
		//do wake up here
			}
	else if(id == LEFT_TO_RIGHT_SLIDE)
	{
		stmsg("Smark Wake Up by LEFT_TO_RIGHT_SLIDE \n");
		//do wake up here
	}
	else if(id == RIGHT_TO_LEFT_SLIDE)
	{
		stmsg("Smark Wake Up by RIGHT_TO_LEFT_SLIDE \n");
		//do wake up here
	} 
}
#endif	//SITRONIX_SMART_WAKE_UP

static void sitronix_ts_work_func(struct work_struct *work)
{
	int i;
#ifdef SITRONIX_AA_KEY
	int j;
	char aa_key_status = 0;
#endif // SITRONIX_AA_KEY
	int ret;


#ifndef SITRONIX_INT_POLLING_MODE
	struct sitronix_ts_data *ts = container_of(work, struct sitronix_ts_data, work);
#else
	struct sitronix_ts_data *ts = container_of(to_delayed_work(work), struct sitronix_ts_data, work);
#endif // SITRONIX_INT_POLLING_MODE
	u16 x, y;
	uint8_t buffer[1+ SITRONIX_MAX_SUPPORTED_POINT * PIXEL_DATA_LENGTH_A] = {0};
	uint8_t PixelCount = 0;

	stmsg("%s\n",  __FUNCTION__);
	//stmsg("%s,line=%d\n,use_irq=%d,irq_num=%d,",__FUNCTION__,__LINE__,ts->use_irq,ts->client->irq);
	atomic_set(&sitronix_ts_in_int, 1);

#ifdef SITRONIX_GESTURE
	if(!ts->suspend_state)
	{
		ret = sitronix_i2c_read_bytes(ts->client, FINGERS, buffer, 1);
		stmsg("SITRONIX_GESTURE ret:%d ,value:0x%X\n",ret,buffer[0]);
		buffer[0] &= 0xF;		
		if((ret == 0 && buffer[0] == G_PALM))
		{
			sitronix_gesture_func(ts->keyevent_input,buffer[0]);
                   	goto exit_invalid_data;
		}
	}
#endif		

#ifdef 	SITRONIX_SMART_WAKE_UP	
	if(ts->suspend_state){
//2.9.15 petitk add 
		ret = sitronix_i2c_read_bytes(ts->client, SMART_WAKE_UP_REG, buffer, 1);
		if(ret ==0 && buffer[0] !=SWK_NO)
		{			
			sitronix_swk_func(ts->keyevent_input, buffer[0]);
			goto exit_invalid_data;
		}				
	}
#endif	//SITRONIX_SMART_WAKE_UP	

	ret = sitronix_i2c_read_bytes(ts->client, KEYS_REG, buffer, 1 + ts->max_touches * ts->pixel_length);
	if (ret < 0) {
		sterr("read finger error (%d)\n", ret);
   		i2cErrorCount++;
		goto exit_invalid_data;
	}

	for(i = 0; i < ts->max_touches; i++){
		if(buffer[1 + i * ts->pixel_length + XY_COORD_H] & 0x80){			
			
			x = (u16)(buffer[1 + i * ts->pixel_length + XY_COORD_H] & 0x70) << 4 | buffer[1 + i * ts->pixel_length + X_COORD_L];
			y = (u16)(buffer[1 + i * ts->pixel_length + XY_COORD_H] & 0x0F) << 8 | buffer[1 + i * ts->pixel_length + Y_COORD_L];
			//stmsg("%s:line=%d,x=%d,y=%d\n",__FUNCTION__,__LINE__,x,y);
#ifndef SITRONIX_AA_KEY
			PixelCount++;
			//if (SpToAp_CheckState() == SP_STATE_IN_GETPIN)
		//		SpToAp_InsertTpPixel(TP_PIXEL_TYPE_DOWN, i, x, y);
			//else
				sitronix_ts_pen_down(ts->input_dev, i, x, y);
#else
#ifdef SITRONIX_KEY_BOUNDARY_MANUAL_SPECIFY
			if(y < SITRONIX_TOUCH_RESOLUTION_Y){
#else
			if(y < (ts->resolution_y - ts->resolution_y / SCALE_KEY_HIGH_Y)){
#endif // SITRONIX_KEY_BOUNDARY_MANUAL_SPECIFY
				PixelCount++;
				sitronix_ts_pen_down(ts->input_dev, i, x, y);
				//stmsg("AREA_DISPLAY\n");
			}else{
				for(j = 0; j < (sizeof(sitronix_aa_key_array)/sizeof(struct sitronix_AA_key)); j++){
					if((x >= sitronix_aa_key_array[j].x_low) &&
					(x <= sitronix_aa_key_array[j].x_high) &&
					(y >= sitronix_aa_key_array[j].y_low) &&
					(y <= sitronix_aa_key_array[j].y_high)){
						aa_key_status |= (1 << j);
						//stmsg("AREA_KEY [%d]\n", j);
						break;
					}
				}
			}
#endif // SITRONIX_AA_KEY
		}else{
#ifdef SITRONIX_SUPPORT_MT_SLOT
			sitronix_ts_pen_up(ts->input_dev, i);
#endif
		}
	}
#ifndef SITRONIX_SUPPORT_MT_SLOT
	if(PixelCount == 0)
		sitronix_ts_pen_up(ts->input_dev, 0);
#endif	
	
	input_report_key(ts->input_dev, BTN_TOUCH, PixelCount > 0);
	input_sync(ts->input_dev);	

	sitronix_ts_handle_sensor_key(ts->keyevent_input, sitronix_sensor_key_array, &sitronix_sensor_key_status, buffer[0], (sizeof(sitronix_sensor_key_array)/sizeof(struct sitronix_sensor_key_t)));
#ifdef SITRONIX_AA_KEY
	sitronix_ts_handle_aa_key(ts->keyevent_input, sitronix_aa_key_array, &sitronix_aa_key_status, aa_key_status, (sizeof(sitronix_aa_key_array)/sizeof(struct sitronix_AA_key)));
#endif // SITRONIX_AA_KEY

exit_invalid_data:
#ifdef SITRONIX_INT_POLLING_MODE
	if(PixelCount > 0){
#ifdef SITRONIX_MONITOR_THREAD
		if(ts->enable_monitor_thread == 1){
			atomic_set(&iMonitorThreadPostpone,1);
		}
#endif // SITRONIX_MONITOR_THREAD
		schedule_delayed_work(&ts->work, msecs_to_jiffies(INT_POLLING_MODE_INTERVAL));
	}else{
#ifdef CONFIG_HARDIRQS_SW_RESEND
		stmsg("Please not set HARDIRQS_SW_RESEND to prevent kernel from sending SW IRQ\n");
#endif // CONFIG_HARDIRQS_SW_RESEND
		if (ts->use_irq){
			atomic_set(&sitronix_ts_irq_on, 1);
//			enable_irq(ts->client->irq);
		}
	}
#endif // SITRONIX_INT_POLLING_MODE
#if defined(SITRONIX_LEVEL_TRIGGERED)
	if (ts->use_irq){
		atomic_set(&sitronix_ts_irq_on, 1);
//		enable_irq(ts->client->irq);
	}
#endif // defined(SITRONIX_LEVEL_TRIGGERED)
	if ((2 <= i2cErrorCount)){
//		stmsg("I2C abnormal in work_func(), reset it!\n");
//		sitronix_ts_reset_ic();
   		i2cErrorCount = 0;
#ifdef SITRONIX_MONITOR_THREAD
		if(ts->enable_monitor_thread == 1){
			StatusCheckCount = 0;
			if(ts->RawCRC_enabled == 0)
				sitronix_set_raw_data_type(&sitronix_ts_gpts);
		}
#endif // SITRONIX_MONITOR_THREAD
	}
	atomic_set(&sitronix_ts_in_int, 0);
	
}

static irqreturn_t sitronix_ts_irq_handler(int irq, void *dev_id)
{
	struct sitronix_ts_data *ts = dev_id;
	//stmsg("%s,line=%d\n",__FUNCTION__,__LINE__);
	stmsg("%s\n", __FUNCTION__);
	atomic_set(&sitronix_ts_in_int, 1);
#if defined(SITRONIX_LEVEL_TRIGGERED) || defined(SITRONIX_INT_POLLING_MODE)
	atomic_set(&sitronix_ts_irq_on, 0);
//	disable_irq_nosync(ts->client->irq);
#endif // defined(SITRONIX_LEVEL_TRIGGERED) || defined(SITRONIX_INT_POLLING_MODE)
#ifdef SITRONIX_MONITOR_THREAD
	if(ts->enable_monitor_thread == 1){
		atomic_set(&iMonitorThreadPostpone,1);
	}
#endif // SITRONIX_MONITOR_THREAD
#ifndef SITRONIX_INT_POLLING_MODE
	schedule_work(&ts->work);
#else
	schedule_delayed_work(&ts->work, msecs_to_jiffies(0));
#endif // SITRONIX_INT_POLLING_MODE
	return IRQ_HANDLED;
}

#ifdef SITRONIX_SYSFS
static ssize_t sitronix_ts_reprobe_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	stmsg("sitronix_ts_reprobe_store!!!!!\n");
	sitronix_ts_sysfs_using = true;
	sitronix_ts_reprobe();
	sitronix_ts_sysfs_using = false;
	return count;
}

static DEVICE_ATTR(reprobe, 0600, NULL, sitronix_ts_reprobe_store);

static struct attribute *sitronix_ts_attrs_v0[] = {
	&dev_attr_reprobe.attr,
	NULL,
};

static struct attribute_group sitronix_ts_attr_group_v0 = {
	.name = "sitronix_ts_attrs",
	.attrs = sitronix_ts_attrs_v0,
};

static int sitronix_ts_create_sysfs_entry(struct i2c_client *client)
{
	int err;

	err = sysfs_create_group(&(client->dev.kobj), &sitronix_ts_attr_group_v0);
	if (err) {
		dev_warn(&client->dev, "%s(%u): sysfs_create_group() failed!\n", __FUNCTION__, __LINE__);
	}
	return err;
}

static void sitronix_ts_destroy_sysfs_entry(struct i2c_client *client)
{
	sysfs_remove_group(&(client->dev.kobj), &sitronix_ts_attr_group_v0);

	return;
}
#endif // SITRONIX_SYSFS

/*******************************************************
Function:
	Disable IRQ Function.

Input:
	data:	i2c client private struct.
	
Output:
	None.
*******************************************************/
void sitronix_irq_disable(struct sitronix_ts_data *data)
{
        unsigned long irqflags;

        stmsg("%s ---start!---\n", __func__);
        spin_lock_irqsave(&data->irq_lock, irqflags);
        if (!data->irq_is_disable) {
               data->irq_is_disable = 1; 
        }
        spin_unlock_irqrestore(&data->irq_lock, irqflags);
}

/*******************************************************
Function:
	Disable IRQ Function.

Input:
	data:	i2c client private struct.
	
Output:
	None.
*******************************************************/
void sitronix_irq_enable(struct sitronix_ts_data *data)
{
        unsigned long irqflags = 0;

        stmsg("%s ---start!---\n", __func__);
    
        spin_lock_irqsave(&data->irq_lock, irqflags);
        if (data->irq_is_disable) {
                data->irq_is_disable = 0; 
        }
        spin_unlock_irqrestore(&data->irq_lock, irqflags);
}


#if (CTP_AP_SP_SYNC_WAY != CTP_AP_SP_SYNC_NONE)
int sitronix_touch_reinit(void)
{
	if(gSitronixPtr == NULL)
		return -1;
	 stmsg("%s\n",__func__);
	 sitronix_irq_enable(gSitronixPtr);
	 return 0;
}

int sitronix_touch_uninit(void)
{
	if(gSitronixPtr == NULL)
		return -1;
	 stmsg("%s\n",__func__);
	 sitronix_irq_disable(gSitronixPtr);	
	 while(cancel_work_sync(&gSitronixPtr->work));
        //flush_workqueue(sitronix_wq);
	return 0;
}
#endif

#if  (CTP_AP_SP_SYNC_WAY & CTP_AP_SP_SYNC_GPIO)
#define CTP_SYNC_IO_OUT	1
#define CTP_SYNC_IO_IN		0
#if 0

static int ap_sp_sync_gpio(struct gpio_config gpio, int direction, int value)
{
	//int status = 0;
	int ret = 0;

	if(0 != gpio_request(gpio.gpio, "ctp_sync_io")) {
		sterr("ap_sp_sync_gpio gpio_request is failed\n");
		return -1;
	}

	if(CTP_SYNC_IO_OUT == direction)
	{
		if (0 != gpio_direction_output(gpio.gpio, value)) {
			sterr(KERN_ERR "ap_sp_sync_gpio gpio set err!");
			return -1;
		}
	}
	else if(CTP_SYNC_IO_IN == direction)
	{
		if (0 != gpio_direction_input(gpio.gpio)) {
			sterr(KERN_ERR "ap_sp_sync_gpio gpio set err!");
			return -1;
		}
		ret =  __gpio_get_value(gpio.gpio);
	}

	gpio_free(gpio.gpio);

	return ret;
}
#endif

static void sitronix_io_sync_work_func(struct work_struct *work)
{
	int pin_val = 0xff;
	unsigned long irqflags;
	stmsg("%s:line=%d,",__FUNCTION__,__LINE__);
	// pin_val = ap_sp_sync_gpio(sitronix_sync_io,CTP_SYNC_IO_IN,0);
	 //stmsg("P:%d\n",pin_val);
	 if(0 == pin_val)
	 {
	 	spin_lock_irqsave(&ctp_sync_lock,irqflags);
	 	ctp_sync_pulse_count++;
	 	if(1 == ctp_sync_io_last_status)
	 	{
	 		ctp_sync_io_last_status = 0;
	 	}
		spin_unlock_irqrestore(&ctp_sync_lock,irqflags);
		
	 }
	else if(1 == pin_val)
	{
		if(!ctp_sync_io_last_status)
		{
			stmsg("C:%d\n",ctp_sync_pulse_count);
			if((ctp_sync_pulse_count > 1) && (ctp_sync_pulse_count < 4)) //tp switch to sp siginal
			{
				stmsg("U\n");
				sitronix_touch_uninit();//\B4?\AF\CA\FD\BB\E1\D7\E8\C8\FB\D0\E8?\D4?\AA?\B8\F6\B9\A4\D7\F7\B6\D3\C1\D0?
			}
			else if((ctp_sync_pulse_count > 3) && (ctp_sync_pulse_count < 6)) //tp switch to ap siginal
			{
				stmsg("R\n");
				sitronix_touch_reinit();
			}
		}
		spin_lock_irqsave(&ctp_sync_lock,irqflags);
		ctp_sync_io_last_status = 1;
		ctp_sync_pulse_count = 0;
		spin_unlock_irqrestore(&ctp_sync_lock,irqflags);
		
	}

}

void ctp_sync_timer_func(unsigned long data)
{
	stmsg("%s:line=%d,",__FUNCTION__,__LINE__);

	queue_work(sitronix_io_sync_workqueue, &gSitronixPtr->sitronix_io_sync_work);
	
	mod_timer(&ctp_sync_timer,jiffies+HZ/50);  
}
#endif

#define SITRONIX_COORDS_ARR_SIZE	4
#define MAX_BUTTONS		4
#define PROP_NAME_SIZE		24



/************************************************************************************************************************************/
char stinfbuf[16];

static ssize_t st_inform_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t num_read_chars = 0;

	stmsg("%s",__func__);
	mutex_lock(&sitronix_ts_gpts.dev_mutex);

	num_read_chars = snprintf(buf, 128, "%s%s\n", sitronix_ts_gpts.fw_revision,stinfbuf);
	
	mutex_unlock(&sitronix_ts_gpts.dev_mutex);

	return num_read_chars;
}



static ssize_t st_inform_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	stmsg("%s",__func__);
	mutex_lock(&sitronix_ts_gpts.dev_mutex);

    memset(stinfbuf, 0, 16);
    memcpy(stinfbuf, buf, count);
    stmsg("%s inbuffer:%s buf:%s \n",__func__,stinfbuf,buf);

	mutex_unlock(&sitronix_ts_gpts.dev_mutex);	

	return count;
}


static ssize_t st_fw_version_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t num_read_chars = 0;

	stmsg("%s",__func__);
	mutex_lock(&sitronix_ts_gpts.dev_mutex);

	num_read_chars = sprintf(buf, "%s\n",tp_info);//add hardware TP info, wt.fangzhihua,20190924
	mutex_unlock(&sitronix_ts_gpts.dev_mutex);

	return num_read_chars;
}

static ssize_t st_fw_version_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	stmsg("%s",__func__);

	return count;
}

static ssize_t st_rawtest_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t sret;
	stmsg("%s",__func__);

	mutex_lock(&sitronix_ts_gpts.dev_mutex);

	/*add function*/
#ifdef ST_TEST_RAW
//    sret = snprintf(buf, 200,
//		"Sitronix Raw Test Result = %d ( 0 == success, >0 failed sensor number, < 0 err) \n",sitronix_ts_gpts.rawTestResult);
    if(0 == sitronix_ts_gpts.rawTestResult)
	{
		sret = snprintf(buf,20,"Pass \n");
	}else{
		sret = snprintf(buf,20,"Fail \n");
	}
#else
	sret = snprintf(buf, 100,
		"Don't support ST_TEST_RAW mode\n");
#endif

	mutex_unlock(&sitronix_ts_gpts.dev_mutex);

	return sret;
}


static ssize_t st_rawtest_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
#ifdef ST_TEST_RAW
	int status=0;
#endif
	stmsg("%s",__func__);

	mutex_lock(&sitronix_ts_gpts.dev_mutex);
	monitor_thread_flag_for_sysfs = 0;
#ifdef ST_TEST_RAW
    status =  st_testraw_invoke();
	sitronix_ts_gpts.rawTestResult =status;
#endif
	monitor_thread_flag_for_sysfs = 1;
	mutex_unlock(&sitronix_ts_gpts.dev_mutex);	

	return count;
}

static int st_char2hex(char data)
{
    int num=0;

    if((data>='0')&&(data<='9'))
    {
      num=data-'0';
    }

    if((data>='a')&&(data<='f'))
    {
      num=data-'a'+10;
    }

    if((data>='A')&&(data<='F'))
    {
      num=data-'A'+10;
    }

    return num;
}
char st_reg_data=0;

static ssize_t st_rwreg_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    ssize_t num_read_chars;
    stmsg("%s",__func__);

	mutex_lock(&sitronix_ts_gpts.dev_mutex);

    num_read_chars = snprintf(buf, 128, "0x%x\n", st_reg_data);
	
	mutex_unlock(&sitronix_ts_gpts.dev_mutex);	
        
    //stinf("%s num_read_chars:%ld buf:%s",__func__,num_read_chars,buf);

	return num_read_chars;
}

static ssize_t st_rwreg_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    ssize_t num_chars = 0;
    int reg_addr;
    int reg_data;
    char buffer[2]={0};

    stmsg("%s",__func__);
	mutex_lock(&sitronix_ts_gpts.dev_mutex);

    //stmsg("%s count:%ld buf:%s \n",__func__,count,buf);
    num_chars=count-1;
    if(num_chars==2)
    {
        reg_addr=st_char2hex(buf[0])*0x10+st_char2hex(buf[1]);
        stmsg("%s reg_addr:0x%x \n",__func__,reg_addr);
            
        /*read register and save data to reg_data*/
        st_i2c_read_bytes(reg_addr, &st_reg_data , 1);
    }
    if(num_chars==4)
    {
         reg_addr=st_char2hex(buf[0])*0x10+st_char2hex(buf[1]);
         reg_data=st_char2hex(buf[2])*0x10+st_char2hex(buf[3]);
         stinf("%s reg_addr:0x%x reg_data:0x%x \n",__func__,reg_addr,reg_data);
              
         /*write register*/
         buffer[0]=reg_addr;
         buffer[1]=reg_data;
         st_i2c_write_bytes(buffer, 2);
    }
	mutex_unlock(&sitronix_ts_gpts.dev_mutex);	
	return count;
}

static ssize_t st_mt_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t sret = 0;
	stmsg("%s",__func__);

	mutex_lock(&sitronix_ts_gpts.dev_mutex);

	/*add function*/

	mutex_unlock(&sitronix_ts_gpts.dev_mutex);

	return sret;
}


static ssize_t st_mt_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
//	int status=0;
	stmsg("%s",__func__);

	mutex_lock(&sitronix_ts_gpts.dev_mutex);
 
   /*add function*/
#ifdef SITRONIX_MONITOR_THREAD
    if(buf[0] == '1')
		monitor_thread_flag_for_sysfs = 1;
	if(buf[0] == '0')
		monitor_thread_flag_for_sysfs = 0;
#endif
	
	mutex_unlock(&sitronix_ts_gpts.dev_mutex);	

	return count;
}

/****************************************/
/* sysfs */
/**获取tp信息**/
static DEVICE_ATTR(stinform, S_IRUGO|S_IWUSR, st_inform_show, st_inform_store);

/**手套模式：1 ，正常模式：0**/


static DEVICE_ATTR(stfwver, S_IRUGO|S_IWUSR, st_fw_version_show, st_fw_version_store);

/**皮套模式：1 ，正常模式：0**/
static DEVICE_ATTR(strawtest, S_IRUGO|S_IWUSR, st_rawtest_show, st_rawtest_store);

/*read and write register
*read example: echo 88 > sitronixtprwreg ---read register 0x88
*write example:echo 8807 > sitronixtprwreg ---write 0x07 into register 0x88
*note:the number of input must be 2 or 4.if it not enough,please fill in the 0.
*/
static DEVICE_ATTR(strwreg, S_IRUGO|S_IWUSR, st_rwreg_show, st_rwreg_store);

/*monitor*/
static DEVICE_ATTR(stmt, S_IRUGO|S_IWUSR, st_mt_show, st_mt_store);

static struct attribute *st_attributes[] = {
	&dev_attr_stinform.attr,

	&dev_attr_stfwver.attr,
	&dev_attr_strawtest.attr,
	&dev_attr_strwreg.attr,
	&dev_attr_stmt.attr,
	NULL

};

/*************add sys func***************/
static struct attribute_group st_attribute_group ={
	.attrs = st_attributes
};
int st_create_sysfs(struct i2c_client * client)
{
	int ret = 0;
	ret = sysfs_create_group(&client->dev.kobj,&st_attribute_group);
	if(0 != ret){
		dev_err(&client->dev,"%s() - ERROR :sysfs_create_group() failed.\n",__func__);
		sysfs_remove_group(&client->dev.kobj,&st_attribute_group);
	}else{
		stmsg("sysfs_create_group() ok .\n");
	}
	return ret;
}

int st_remove_sysfs(struct i2c_client *client)
{
	sysfs_remove_group(&client->dev.kobj,&st_attribute_group);
	return 0;
}

/**************************************************************************************************************************************/

#if 0
static int sitronix_ts_get_dt_coords(struct device *dev, char *name,
				   struct sitronix_i2c_touch_platform_data *pdata)
{
	struct property *prop;
	struct device_node *np = dev->of_node;
	int rc;
	u32 coords[SITRONIX_COORDS_ARR_SIZE];
	stmsg("%s,%d\n",__FUNCTION__,__LINE__);

	prop = of_find_property(np, name, NULL);
	if (!prop)
		return -EINVAL;
	if (!prop->value)
		return -ENODATA;

	rc = of_property_read_u32_array(np, name, coords,
					SITRONIX_COORDS_ARR_SIZE);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read %s\n", name);
		return rc;
	}

	if (!strcmp(name, "sitronix,panel-coords")) {
		pdata->panel_minx = coords[0];
		pdata->panel_miny = coords[1];
		pdata->panel_maxx = coords[2];
		pdata->panel_maxy = coords[3];
		stinf("%s:panel-coords,minx=%d,miny=%d,maxx=%d,maxy=%d\n",__FUNCTION__,\
				pdata->panel_minx,pdata->panel_miny,pdata->panel_maxx,pdata->panel_maxy);
	} else if (!strcmp(name, "sitronix,display-coords")) {
		pdata->x_min = coords[0];
		pdata->y_min = coords[1];
		pdata->x_max = coords[2];
		pdata->y_max = coords[3];
		stinf("%s:display-coords,minx=%d,miny=%d,maxx=%d,maxy=%d\n",__FUNCTION__,\
				pdata->panel_minx,pdata->panel_miny,pdata->panel_maxx,pdata->panel_maxy);
	} else {
		dev_err(dev, "unsupported property %s\n", name);
		return -EINVAL;
	}

	return 0;
}
#endif

static int sitronix_parse_dt(struct device *dev,
			   struct sitronix_i2c_touch_platform_data *pdata)
{
	int rc;
	struct device_node *np = dev->of_node;
	struct property *prop;

	stmsg("%s,%d\n",__FUNCTION__,__LINE__);

	pdata->name = "sitronix";
	
	/* reset, irq gpio info */
	pdata->reset_gpio = of_get_named_gpio_flags(np, "sitronix,reset-gpios",0,&pdata->reset_gpio_flags);	
	stinf("%s,pdata->reset_gpio=%d\n",__FUNCTION__,pdata->reset_gpio);
	
	if (pdata->reset_gpio < 0)
		return pdata->reset_gpio;

	pdata->irq_gpio = of_get_named_gpio_flags(np, "sitronix,interrupt-gpios",0, &pdata->irq_gpio_flags);	
	stinf("%s,pdata->irq_gpio=%d\n",__FUNCTION__,pdata->irq_gpio);
	
	if (pdata->irq_gpio < 0)
		return pdata->irq_gpio;	

	prop = of_find_property(np, "sitronix,button-map", NULL);
	if (prop) {
		pdata->num_button = prop->length / sizeof(u32);
		if (pdata->num_button > MAX_BUTTONS)
			return -EINVAL;

		rc = of_property_read_u32_array(np,
						"sitronix,button-map", pdata->button_map, pdata->num_button);
		if (rc) {
			dev_err(dev, "Unable to read key codes\n");
			return rc;
		}
	}

	return 0;
}


static int sitronix_power_on(struct sitronix_ts_data *ts)
{
	int ret = 0;
	if(ts ->vdd_ana){
		ret = regulator_enable(ts->vdd_ana);
		if(ret){
			dev_err(&ts->client->dev,"Regulator vdd enable failed ret =%d\n",ret);
		}
	}
	
	if(ts ->vcc_i2c){
		ret = regulator_enable(ts->vcc_i2c);
		if(ret){
			dev_err(&ts->client->dev,"Regulator vcc_i2c enable failed ret =%d\n",ret);
		}
	}
	return 0;
}

static int sitronix_power_off(struct sitronix_ts_data *ts)
{
	int ret = 0;
	if(ts->vcc_i2c){
		ret = regulator_disable(ts->vcc_i2c);
		if(ret){
			dev_err(&ts->client->dev,"Regulator vcc_i2c disable failed ret =%d\n",ret);
		}
	}
	if(ts->vdd_ana){
		ret = regulator_disable(ts->vdd_ana);
		if(ret){
			dev_err(&ts->client->dev,"Regulator vdd_ana disable failed ret =%d\n",ret);
		}
	}	
	return ret;
}

static int sitronix_power_init(struct sitronix_ts_data *ts)
{
	int ret;
	ts->vdd_ana = regulator_get(&ts->client->dev,"vdd_ana");
	if(IS_ERR(ts->vdd_ana)){
		ts->vdd_ana = NULL;
		ret = PTR_ERR(ts->vdd_ana);
//		dev_info(&ts->client->dev,"Regulator get failed vdd ret = %d\n",ret);
	}
	
	ts->vcc_i2c = regulator_get(&ts->client->dev,"vcc_i2c");
	if(IS_ERR(ts->vcc_i2c)){
		ts->vcc_i2c = NULL;
		ret = PTR_ERR(ts->vcc_i2c);
//		dev_info(&ts->client->dev,"Regulator get failed vcc_i2c ret = %d\n",ret);
	}
	return 0;
}


static int sitronix_power_deinit(struct sitronix_ts_data *ts)
{
	if(ts->vdd_ana)
		regulator_put(ts->vdd_ana);
	if(ts->vcc_i2c)
		regulator_put(ts->vcc_i2c);
	
	return 0;
}

static int sitronix_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int i;
	int ret = 0;
	uint16_t max_x = 0, max_y = 0;
	uint8_t err_code = 0;
	uint8_t dev_status = 0;
	uint8_t rc;
	struct sitronix_i2c_touch_platform_data *pdata;
	//struct sitronix_ts_data *sitronix_ts_gpts;

	stmsg("%s,%d\n",__FUNCTION__,__LINE__);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) 
	{
		ret = -ENODEV;
		sterr("sitronix Failed check I2C functionality\n");
		goto err_check_functionality_failed;
	}

	pdata = devm_kzalloc(&client->dev,
						 sizeof(struct sitronix_i2c_touch_platform_data),
						 GFP_KERNEL);
	if (!pdata) 
	{
		dev_err(&client->dev,"GTP Failed to allocate memory for pdata\n");
		return -ENOMEM;
	}
	ret = sitronix_parse_dt(&client->dev, pdata);
	if (ret)
		return ret;


	sitronix_ts_gpts.irq_gpio = pdata->irq_gpio;
	sitronix_ts_gpts.reset_gpio = pdata->reset_gpio;

	stinf("%s,line=%d,pdata->reset_gpio =%d\n", __FUNCTION__,__LINE__,pdata->reset_gpio);
	
	sitronix_ts_gpts.client = client;
	i2c_set_clientdata(client, &sitronix_ts_gpts);
	
	ret = sitronix_power_init(&sitronix_ts_gpts);
	if(ret){
		dev_err(&client->dev,"sitronix failed get regulator\n");
		ret = -EINVAL;
		return -ENOMEM;
	}
	
	ret = sitronix_power_on(&sitronix_ts_gpts);
	if(ret){
		dev_err(&client->dev,"regulator failed power on device\n");
		ret = -EINVAL;
		goto exit_deinit_power;
	}
	
	ret = gpio_request(sitronix_ts_gpts.irq_gpio, "STP_INT_IRQ");
	if (ret < 0) 
	{
		sterr("Failed to request GPIO:%d, ERRNO:%d",
				  (s32) sitronix_ts_gpts.irq_gpio, ret);
		gpio_free(sitronix_ts_gpts.irq_gpio);	
		return -ENODEV;
	} 
	
	stinf("%s:sitronix_ts_gpts.irq_gpio =%d\n",__FUNCTION__,sitronix_ts_gpts.irq_gpio);
	ret = gpio_direction_input(sitronix_ts_gpts.irq_gpio);
	if (ret < 0) 
	{
		gpio_free(sitronix_ts_gpts.irq_gpio);	
		return -ENODEV;
	}
	rc = gpio_request(pdata->reset_gpio, "reset_gpio");
	if (rc < 0)
	{
		sterr("sitonix failed to request gpio\n");
		gpio_free(pdata->reset_gpio);	
		return -ENODEV;
	}
	gpio_direction_output(sitronix_ts_gpts.reset_gpio, 1);
	msleep(10);	
	gpio_direction_output(sitronix_ts_gpts.reset_gpio, 0);
   	msleep(10);
	gpio_direction_output(sitronix_ts_gpts.reset_gpio, 1);
	msleep(150);

	sitronix_ts_gpts.client = client;

	sitronix_tplockdown_node();  //fangzhhua.wt add tp lockdown 20191016

#ifdef ST_UPGRADE_FIRMWARE
#ifdef ST_FIREWARE_FILE
	kthread_run(st_upgrade_fw, "Sitronix", "sitronix_update");
#else
	st_upgrade_fw();
#endif //ST_FIREWARE_FILE
	 
#endif //ST_UPGRADE_FIRMWARE
		
	if(((ret = sitronix_ts_get_device_status(client, &err_code, &dev_status)) < 0) || (dev_status == 0x6) || ((err_code == 0x8)&&(dev_status == 0x0))){
		if((dev_status == 0x6) || ((err_code == 0x8)&&(dev_status == 0x0))){
			sitronix_ts_gpts.client = client;
		}
		ret = -EPERM;
		goto err_device_info_error;
	}
	sitronix_ts_gpts.suspend_state = 0;
	if((ret = sitronix_ts_get_touch_info(&sitronix_ts_gpts)) < 0)
		goto err_device_info_error;


	sitronix_ts_gpts.client->irq = gpio_to_irq(sitronix_ts_gpts.irq_gpio);
	
	stinf("gpio num:%d, irq num:%d\n",CTP_IRQ_NUMBER,sitronix_ts_gpts.client->irq);

	mutex_init(&sitronix_ts_gpts.dev_mutex);
	//add hardware TP info, wt.fangzhihua,20190924
	memset(tp_info,0,sizeof(tp_info));
	sprintf(tp_info,"st14348_HXD_V%02X",sitronix_ts_gpts.fw_version[0]);
	sterr("tp_info : %s ",tp_info);
	//end,add hardware TP info, wt.fangzhihua,20190924

#ifdef SITRONIX_IDENTIFY_ID
	if((ret = sitronix_ts_identify(&sitronix_ts_gpts)) < 0)
		goto err_device_info_error;
#endif // SITRONIX_IDENTIFY_ID

#ifndef SITRONIX_INT_POLLING_MODE
	INIT_WORK(&(sitronix_ts_gpts.work), sitronix_ts_work_func);
#else
	INIT_DELAYED_WORK(&(sitronix_ts_gpts.work), sitronix_ts_work_func);
#endif // SITRONIX_INT_POLLING_MODE

#if(CTP_AP_SP_SYNC_WAY & CTP_AP_SP_SYNC_GPIO)
	INIT_WORK(&sitronix_ts_gpts.sitronix_io_sync_work, sitronix_io_sync_work_func);
#endif
	
#if(CTP_AP_SP_SYNC_WAY & CTP_AP_SP_SYNC_GPIO)
	sitronix_io_sync_workqueue= create_singlethread_workqueue("sitronix_io_sync_workqueue");
	if (!sitronix_io_sync_workqueue) {
			sterr("Creat sitronix_io_sync_workqueue workqueue failed.\n");
			goto err_device_info_error;
	}
#endif
	
#ifdef SITRONIX_MONITOR_THREAD
	if(sitronix_ts_gpts.enable_monitor_thread == 1){
		//== Add thread to monitor chip
		atomic_set(&iMonitorThreadPostpone,1);
		sitronix_ts_gpts.sitronix_mt_fp = sitronix_ts_gpts.RawCRC_enabled? sitronix_ts_monitor_thread_v2 : sitronix_ts_monitor_thread;
		SitronixMonitorThread = kthread_run(sitronix_ts_gpts.sitronix_mt_fp,"Sitronix","Monitorthread");
		if(IS_ERR(SitronixMonitorThread))
			SitronixMonitorThread = NULL;
	}
#endif // SITRONIX_MONITOR_THREAD

	sitronix_ts_gpts.input_dev = input_allocate_device();
	if (sitronix_ts_gpts.input_dev == NULL){
		sterr("Can not allocate memory for input device.");
		ret = -ENOMEM;
		goto err_input_dev_alloc_failed;
	}

	sitronix_ts_gpts.input_dev->name = SITRONIX_I2C_TOUCH_MT_INPUT_DEV_NAME;
	sitronix_ts_gpts.input_dev->dev.parent = &client->dev;
	sitronix_ts_gpts.input_dev->id.bustype = BUS_I2C;

	set_bit(EV_KEY, sitronix_ts_gpts.input_dev->evbit);
	set_bit(BTN_TOUCH, sitronix_ts_gpts.input_dev->keybit);
	set_bit(EV_ABS, sitronix_ts_gpts.input_dev->evbit);
	set_bit(INPUT_PROP_DIRECT, sitronix_ts_gpts.input_dev->propbit);

	sitronix_ts_gpts.keyevent_input = input_allocate_device();
	if (sitronix_ts_gpts.keyevent_input == NULL){
		sterr("Can not allocate memory for key input device.");
		ret = -ENOMEM;
		goto err_input_dev_alloc_failed;
	}
	sitronix_ts_gpts.keyevent_input->name  = SITRONIX_I2C_TOUCH_KEY_INPUT_DEV_NAME;
	sitronix_ts_gpts.keyevent_input->dev.parent = &client->dev;
	set_bit(EV_KEY, sitronix_ts_gpts.keyevent_input->evbit);
	for(i = 0; i < pdata->num_button; i++){
		set_bit(sitronix_sensor_key_array[i].code, sitronix_ts_gpts.keyevent_input->keybit);
		input_set_capability(sitronix_ts_gpts.keyevent_input,	EV_KEY, pdata->button_map[i]);
	}

#ifdef SITRONIX_SMART_WAKE_UP
	set_bit(KEY_POWER, sitronix_ts_gpts.keyevent_input->keybit);
#endif	//SITRONIX_SMART_WAKE_UP

#ifndef SITRONIX_AA_KEY
	max_x = sitronix_ts_gpts.resolution_x;
	max_y = sitronix_ts_gpts.resolution_y;
#else
#ifdef SITRONIX_KEY_BOUNDARY_MANUAL_SPECIFY
	for(i = 0; i < (sizeof(sitronix_aa_key_array)/sizeof(struct sitronix_AA_key)); i++){
		set_bit(sitronix_aa_key_array[i].code, sitronix_ts_gpts.keyevent_input->keybit);
	}
	max_x = SITRONIX_TOUCH_RESOLUTION_X;
	max_y = SITRONIX_TOUCH_RESOLUTION_Y;
#else
	for(i = 0; i < (sizeof(sitronix_aa_key_array)/sizeof(struct sitronix_AA_key)); i++){
		sitronix_aa_key_array[i].x_low = ((sitronix_ts_gpts.resolution_x / (sizeof(sitronix_aa_key_array)/sizeof(struct sitronix_AA_key)) ) * i ) + 15;
		sitronix_aa_key_array[i].x_high = ((sitronix_ts_gpts.resolution_x / (sizeof(sitronix_aa_key_array)/sizeof(struct sitronix_AA_key)) ) * (i + 1)) - 15;
		sitronix_aa_key_array[i].y_low = sitronix_ts_gpts.resolution_y - sitronix_ts_gpts.resolution_y / SCALE_KEY_HIGH_Y;
		sitronix_aa_key_array[i].y_high = sitronix_ts_gpts.resolution_y;
		stinf("key[%d] %d, %d, %d, %d\n", i, sitronix_aa_key_array[i].x_low, sitronix_aa_key_array[i].x_high, sitronix_aa_key_array[i].y_low, sitronix_aa_key_array[i].y_high);
		set_bit(sitronix_aa_key_array[i].code, sitronix_ts_gpts.keyevent_input->keybit);
	}
	max_x = sitronix_ts_gpts.resolution_x;
	max_y = sitronix_ts_gpts.resolution_y - sitronix_ts_gpts.resolution_y / SCALE_KEY_HIGH_Y;
#endif // SITRONIX_KEY_BOUNDARY_MANUAL_SPECIFY
#endif // SITRONIX_AA_KEY


	ret = input_register_device(sitronix_ts_gpts.keyevent_input);
	if(ret < 0){
		sterr("Can not register key input device.\n");
		goto err_input_register_device_failed;
	}	


#ifdef SITRONIX_SUPPORT_MT_SLOT
	input_mt_init_slots(sitronix_ts_gpts.input_dev, sitronix_ts_gpts.max_touches,INPUT_MT_DIRECT);
#endif // SITRONIX_SUPPORT_MT_SLOT
//	__set_bit(ABS_X, sitronix_ts_gpts.input_dev->absbit);
//	__set_bit(ABS_Y, sitronix_ts_gpts.input_dev->absbit);
	__set_bit(ABS_MT_TOUCH_MAJOR, sitronix_ts_gpts.input_dev->absbit);
	__set_bit(ABS_MT_WIDTH_MAJOR, sitronix_ts_gpts.input_dev->absbit);
	__set_bit(ABS_MT_POSITION_X, sitronix_ts_gpts.input_dev->absbit);
	__set_bit(ABS_MT_POSITION_Y, sitronix_ts_gpts.input_dev->absbit);
	__set_bit(ABS_MT_TOOL_TYPE, sitronix_ts_gpts.input_dev->absbit);
	__set_bit(ABS_MT_BLOB_ID, sitronix_ts_gpts.input_dev->absbit);
	__set_bit(ABS_MT_TRACKING_ID, sitronix_ts_gpts.input_dev->absbit);

	input_set_abs_params(sitronix_ts_gpts.input_dev, ABS_MT_TOUCH_MAJOR, 0,  255, 0, 0);
	input_set_abs_params(sitronix_ts_gpts.input_dev, ABS_MT_WIDTH_MAJOR, 0,  255, 0, 0);
	input_set_abs_params(sitronix_ts_gpts.input_dev, ABS_MT_TRACKING_ID, 0, sitronix_ts_gpts.max_touches, 0, 0);
	input_set_abs_params(sitronix_ts_gpts.input_dev, ABS_PRESSURE, 0, 255, 0, 0);

stmsg("%s,line=%d\n", __FUNCTION__,__LINE__);
#ifndef SITRONIX_SWAP_XY
	input_set_abs_params(sitronix_ts_gpts.input_dev, ABS_MT_POSITION_X, 0, max_x, 0, 0);
	input_set_abs_params(sitronix_ts_gpts.input_dev, ABS_MT_POSITION_Y, 0, max_y, 0, 0);
#else
	input_set_abs_params(sitronix_ts_gpts.input_dev, ABS_MT_POSITION_X, 0, max_y, 0, 0);
	input_set_abs_params(sitronix_ts_gpts.input_dev, ABS_MT_POSITION_Y, 0, max_x, 0, 0);
#endif // SITRONIX_SWAP_XY

	ret = input_register_device(sitronix_ts_gpts.input_dev);
	if(ret < 0){
		sterr("Can not register input device.\n");
		goto err_input_register_device_failed;
	}

if (sitronix_ts_gpts.client->irq){
		dev_info(&client->dev, "irq = %d\n", sitronix_ts_gpts.client->irq);
#ifdef SITRONIX_LEVEL_TRIGGERED

//	ret = request_irq(sitronix_ts_gpts.client->irq, sitronix_ts_irq_handler, IRQF_TRIGGER_FALLING |IRQF_DISABLED , client->name, &sitronix_ts_gpts);
	ret = request_threaded_irq(sitronix_ts_gpts.client->irq,NULL, sitronix_ts_irq_handler, IRQF_TRIGGER_FALLING | IRQF_ONESHOT, client->name, &sitronix_ts_gpts);
#else
	ret = request_irq(sitronix_ts_gpts.client->irq, sitronix_ts_irq_handler, IRQF_TRIGGER_FALLING ,SITRONIX_I2C_TOUCH_DRV_NAME, &sitronix_ts_gpts);
#endif // SITRONIX_LEVEL_TRIGGERED
		if (ret == 0){
			atomic_set(&sitronix_ts_irq_on, 1);
			sitronix_ts_gpts.use_irq = 1;
		}else{
			dev_err(&client->dev, "request_irq failed\n");
			goto err_request_irq_failed;
		}
	}

	if(st_create_sysfs(client) < 0)
	{
		goto err_create_sysfs_failed;
	}

#ifdef SITRONIX_SYSFS
	if(!sitronix_ts_sysfs_created){
		ret = sitronix_ts_create_sysfs_entry(client);
		if(ret < 0)
			goto err_create_sysfs_failed;
		sitronix_ts_sysfs_created = true;
	}
#endif // SITRONIX_SYSFS
stmsg("%s,line=%d\n", __FUNCTION__,__LINE__);

#if defined(CONFIG_FB)
	sitronix_ts_gpts.fb_notif.notifier_call = fb_notifier_callback;

	ret = fb_register_client(&sitronix_ts_gpts.fb_notif);

	if (ret)
		dev_err(&client->dev, "Unable to register fb_notifier: %d\n", ret);
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	sitronix_ts_gpts.early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	sitronix_ts_gpts.early_suspend.suspend = sitronix_ts_early_suspend;
	sitronix_ts_gpts.early_suspend.resume = sitronix_ts_late_resume;
	register_early_suspend(&sitronix_ts_gpts.early_suspend);	
#endif // CONFIG_HAS_EARLYSUSPEND

	//add touch pannel prepare switch to sp interface
	#if (CTP_AP_SP_SYNC_WAY != CTP_AP_SP_SYNC_NONE)
	//sitronix_ts_gpts.sitronix_driver_ops.touch_device_name = CTP_NAME;
	//sitronix_ts_gpts.sitronix_driver_ops.touch_pannel_reinit = sitronix_touch_reinit;
	//sitronix_ts_gpts.sitronix_driver_ops.touch_pannel_uninit = sitronix_touch_uninit;
	//gSitronixPtr =  &sitronix_ts_gpts;
	#endif

	#if(CTP_AP_SP_SYNC_WAY & CTP_AP_SP_SYNC_APP)
	// touch_panel_driver_add(&gSitronixPtr->sitronix_driver_ops);
	#endif
	stmsg("%s,line=%d\n", __FUNCTION__,__LINE__);
	
	#if(CTP_AP_SP_SYNC_WAY & CTP_AP_SP_SYNC_GPIO)
	//spin_lock_init(&ctp_sync_lock);
	//init_timer(&ctp_sync_timer);
	//ctp_sync_timer.expires = jiffies + HZ/50; 
	//ctp_sync_timer.function = ctp_sync_timer_func;   
	//add_timer(&ctp_sync_timer);   //crash
	#endif
	
	return 0;
err_create_sysfs_failed:
st_remove_sysfs(client);

err_request_irq_failed:
#ifdef SITRONIX_SYSFS
	input_unregister_device(sitronix_ts_gpts.input_dev);
	input_unregister_device(sitronix_ts_gpts.keyevent_input);
#endif // SITRONIX_SYSFS
err_input_register_device_failed:
err_input_dev_alloc_failed:
	#if(CTP_AP_SP_SYNC_WAY & CTP_AP_SP_SYNC_GPIO)
	destroy_workqueue(sitronix_io_sync_workqueue);
	#endif
	
	if(sitronix_ts_gpts.input_dev)
		input_free_device(sitronix_ts_gpts.input_dev);
	if(sitronix_ts_gpts.keyevent_input)
		input_free_device(sitronix_ts_gpts.keyevent_input);
#ifdef SITRONIX_MONITOR_THREAD
	if(sitronix_ts_gpts.enable_monitor_thread == 1){
		if(SitronixMonitorThread){
		      kthread_stop(SitronixMonitorThread);
		      SitronixMonitorThread = NULL;
		}
	}
#endif // SITRONIX_MONITOR_THREAD
err_device_info_error:
sitronix_power_off(&sitronix_ts_gpts);
exit_deinit_power:
sitronix_power_deinit(&sitronix_ts_gpts);
err_check_functionality_failed:

	return ret;
}

static int sitronix_ts_remove(struct i2c_client *client)
{
	struct sitronix_ts_data *ts = i2c_get_clientdata(client);
	
#if defined(CONFIG_FB)
	if (fb_unregister_client(&ts->fb_notif))
		dev_err(&client->dev, "Error occurred while unregistering fb_notifier.\n");	
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	unregister_early_suspend(&ts->early_suspend);
#endif

#ifdef SITRONIX_SYSFS
	if(!sitronix_ts_sysfs_using){
		sitronix_ts_destroy_sysfs_entry(client);
		sitronix_ts_sysfs_created = false;
	}
#endif // SITRONIX_SYSFS
#ifdef SITRONIX_MONITOR_THREAD
	if(ts->enable_monitor_thread == 1){
		if(SitronixMonitorThread){
		      kthread_stop(SitronixMonitorThread);
		      SitronixMonitorThread = NULL;
		}
	}
#endif // SITRONIX_MONITOR_THREAD
    sitronix_power_off(&sitronix_ts_gpts);
	sitronix_power_deinit(&sitronix_ts_gpts);
	i2c_set_clientdata(client, NULL);
	if (ts->use_irq)
		free_irq(client->irq, ts);
	if(ts->input_dev)
		input_unregister_device(ts->input_dev);
	if(ts->keyevent_input)
		input_unregister_device(ts->keyevent_input);
	return 0;
}

static int sitronix_ts_suspend(struct i2c_client *client)
{
	struct sitronix_ts_data *ts = i2c_get_clientdata(client);

	stmsg("%s\n", __FUNCTION__);
//2.9.15 petitk add
#ifdef SITRONIX_SMART_WAKE_UP		
	sitronix_swk_set_swk_enable(ts);
	swk_flag = 1;	
#endif //SITRONIX_SMART_WAKE_UP

#ifdef SITRONIX_MONITOR_THREAD
//	if(ts->enable_monitor_thread == 1){
//		if(SitronixMonitorThread){
//			kthread_stop(SitronixMonitorThread);
//			SitronixMonitorThread = NULL;
//		}
		sitronix_ts_delay_monitor_thread_start = DELAY_MONITOR_THREAD_START_RESUME;
//	}
    monitor_thread_flag = 0;
#endif // SITRONIX_MONITOR_THREAD
	if(ts->use_irq){
		atomic_set(&sitronix_ts_irq_on, 0);
		disable_irq_nosync(ts->client->irq);
	}
	ts->suspend_state = 1;


	gpio_direction_output(sitronix_ts_gpts.reset_gpio, 0);
    msleep(10);
//	sitronix_power_off(&sitronix_ts_gpts);
	stmsg("%s return\n", __FUNCTION__);

	return 0;
}

static int sitronix_ts_resume(struct i2c_client *client)
{
	struct sitronix_ts_data *ts = i2c_get_clientdata(client);

	stmsg("%s\n", __FUNCTION__);
//    sitronix_power_on(&sitronix_ts_gpts);
//	gpio_direction_output(sitronix_ts_gpts.reset_gpio, 1);
//	msleep(150);
    sitronix_ts_reset_ic_v2();

	ts->suspend_state = 0;
	if(ts->use_irq){
		atomic_set(&sitronix_ts_irq_on, 1);
		enable_irq(ts->client->irq);
	}
#ifdef SITRONIX_MONITOR_THREAD
//	if(ts->enable_monitor_thread == 1){
//		atomic_set(&iMonitorThreadPostpone,1);
//		SitronixMonitorThread = kthread_run(sitronix_ts_gpts.sitronix_mt_fp,"Sitronix","Monitorthread");
//		if(IS_ERR(SitronixMonitorThread))
//			SitronixMonitorThread = NULL;
//	}
    monitor_thread_flag = 1;
#endif // SITRONIX_MONITOR_THREAD
	stmsg("%s return\n", __FUNCTION__);
	
	return 0;
}


#if defined(CONFIG_FB)

/*******************************************************************************
*  Name: fb_notifier_callback
*  Brief:
*  Input:
*  Output: 
*  Return: 
*******************************************************************************/
static int fb_notifier_callback(struct notifier_block *self,
				 unsigned long event, void *data)
{
	struct fb_event *evdata = data;
	int *blank;
	struct sitronix_ts_data *sitronix_data = container_of(self, struct sitronix_ts_data, fb_notif);
				      //container_of(self, struct sitronix_ts_data, fb_notif);

	if (evdata && evdata->data && event == FB_EVENT_BLANK &&
			sitronix_data && sitronix_data->client) {
		blank = evdata->data;
		if (*blank == FB_BLANK_UNBLANK)
			sitronix_ts_resume(sitronix_data->client);
		else if (*blank == FB_BLANK_POWERDOWN)
			sitronix_ts_suspend(sitronix_data->client);
	}

	return 0;
}
#elif defined(CONFIG_HAS_EARLYSUSPEND)
static void sitronix_ts_early_suspend(struct early_suspend *h)
{
	struct sitronix_ts_data *ts;
	stmsg("%s\n", __FUNCTION__);
	ts = container_of(h, struct sitronix_ts_data, early_suspend);
	sitronix_ts_suspend(ts->client);
}

static void sitronix_ts_late_resume(struct early_suspend *h)
{
	struct sitronix_ts_data *ts;
	stmsg("%s\n", __FUNCTION__);
	ts = container_of(h, struct sitronix_ts_data, early_suspend);
	sitronix_ts_resume(ts->client);
}
#endif

static const struct i2c_device_id sitronix_ts_id[] = {
	{ SITRONIX_I2C_TOUCH_DRV_NAME, 0 },
	{ }
};
#ifdef SITRONIX_MULTI_SLAVE_ADDR
static int sitronix_ts_detect(struct i2c_client *client, struct i2c_board_info *info)
{
	uint8_t buffer[8];
	sitronix_ts_reset_ic_v2();
	stmsg("%s: bus = %d\n", __FUNCTION__, client->adapter->nr);
	if((client->adapter->nr == twi_id) && (!sitronix_i2c_read_bytes(client, STATUS_REG, buffer, 8))){
		stmsg("detect successed\n");
		strlcpy(info->type, SITRONIX_I2C_TOUCH_DRV_NAME, strlen(SITRONIX_I2C_TOUCH_DRV_NAME)+1);
		return 0;
	}else{
		stmsg("detect failed\n");
		return -ENODEV;
	}
}

const unsigned short sitronix_i2c_addr[] = {0x20, 0x38, 0x55, 0x70, 0x60, I2C_CLIENT_END};
#endif // SITRONIX_MULTI_SLAVE_ADDR



static struct of_device_id sitronix_match_table[] = {
	{ .compatible = "sitronix,st1633i", },
	{ .compatible = "sitronix,st143xx", },
	{ .compatible = "sitronix,st7125", },
	{ },
};

//@landicorp: lpz add
#if (!defined(CONFIG_FB) && !defined(CONFIG_HAS_EARLYSUSPEND))	
static int sitronix_pm_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	sitronix_ts_suspend(client);
        return 0;

}
static int sitronix_pm_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	sitronix_ts_resume(client);
        return 0;
}

static const struct dev_pm_ops sitronix_ts_dev_pm_ops = {
	.suspend = sitronix_pm_suspend,
	.resume = sitronix_pm_resume,
};
#endif
static struct i2c_driver sitronix_ts_driver = {
#ifdef SITRONIX_MULTI_SLAVE_ADDR
	.class 		= I2C_CLASS_HWMON,
#endif // SITRONIX_MULTI_SLAVE_ADDR
	.probe		= sitronix_ts_probe,
	.remove		= sitronix_ts_remove,
	.id_table	= sitronix_ts_id,
	.driver = {
		.name	= SITRONIX_I2C_TOUCH_DRV_NAME,
		.owner = THIS_MODULE,
		.of_match_table = sitronix_match_table,
#if CONFIG_PM
		.suspend = NULL,
		.resume = NULL,//sitronix_resume,
#endif
	},
#ifdef SITRONIX_MULTI_SLAVE_ADDR
	//.address_list = sitronix_i2c_addr,
	.detect = sitronix_ts_detect,
#endif // SITRONIX_MULTI_SLAVE_ADDR
};

#ifdef SITRONIX_FW_UPGRADE_FEATURE
static struct file_operations nc_fops = {
	.owner =        THIS_MODULE,
	.write		= sitronix_write,
	.read		= sitronix_read,
	.open		= sitronix_open,
	.unlocked_ioctl = sitronix_ioctl,
	.release	= sitronix_release,
};
#endif // SITRONIX_FW_UPGRADE_FEATURE
void sitronix_ts_reprobe(void)
{
	int retval = 0;	
	stmsg("sitronix call reprobe!\n");
	i2c_del_driver(&sitronix_ts_driver);
	retval = i2c_add_driver(&sitronix_ts_driver);
	if(retval < 0)
		stmsg("fail to reprobe driver!\n");
}

#ifdef CONFIG_ARCH_SUNXI
/**
 * ctp_print_info - sysconfig print function
 * return value:
 *
 */
void sitronix_print_info(struct ctp_config_info info)
{
	stinf("info.ctp_used:%d\n",info.ctp_used);
	stinf("info.twi_id:%d\n",info.twi_id);
	stinf("info.screen_max_x:%d\n",info.screen_max_x);
	stinf("info.screen_max_y:%d\n",info.screen_max_y);
	stinf("info.revert_x_flag:%d\n",info.revert_x_flag);
	stinf("info.revert_y_flag:%d\n",info.revert_y_flag);
	stinf("info.exchange_x_y_flag:%d\n",info.exchange_x_y_flag);
	stinf("info.irq_gpio_number:%d\n",info.irq_gpio.gpio);
	stinf("info.wakeup_gpio_number:%d\n",info.wakeup_gpio.gpio);
}


static int ctp_get_system_config(void)
{   
        sitronix_print_info(config_info);
        twi_id = config_info.twi_id;
        screen_max_x = config_info.screen_max_x;
        screen_max_y = config_info.screen_max_y;
        revert_x_flag = config_info.revert_x_flag;
        revert_y_flag = config_info.revert_y_flag;
        exchange_x_y_flag = config_info.exchange_x_y_flag; 
        if((screen_max_x == 0) || (screen_max_y == 0)){
                sterr("%s:read config error!\n",__func__);
                return 0;
        }
        return 1;
}
#endif

static int __init sitronix_ts_init(void)
{

#ifdef CONFIG_ARCH_SUNXI
	s32 ret = -1;
#endif
	s32 ret_iic = -2;
	#if(CTP_AP_SP_SYNC_WAY & CTP_AP_SP_SYNC_GPIO)
	//script_item_u   val;
	//script_item_value_type_e  type;
	//int pin_val = 0;
	#endif
#ifdef SITRONIX_FW_UPGRADE_FEATURE
	int result;
	int err = 0;
	dev_t devno = MKDEV(sitronix_major, 0);
#endif // SITRONIX_FW_UPGRADE_FEATURE

	#if(CTP_AP_SP_SYNC_WAY & CTP_AP_SP_SYNC_GPIO)
	//type = script_get_item("ctp_para", "ctp_sync_io", &val);
	//if(SCIRPT_ITEM_VALUE_TYPE_PIO != type) {
	//	sterr("%s: ctp_sync_io script_get_item err. \n",__func__ );
	//}
	//else
	{
		stmsg("ctp_sync_io get ok\n");
		//sitronix_sync_io = val.gpio;
	}

	//ap_sp_sync_gpio(sitronix_sync_io, CTP_SYNC_IO_IN, pin_val);
	#endif
#ifdef CONFIG_ARCH_SUNXI
	stmsg("****************************************************************\n");
	if (0)//(input_fetch_sysconfig_para(&(config_info.input_type))) 
	{
		sterr("%s: ctp_fetch_sysconfig_para err.\n", __func__);
		return 0;
	} 
	else 
	{
		//ret = input_init_platform_resource(&(config_info.input_type));
		if (0 != ret) {
			sterr("%s:ctp_ops.init_platform_resource err. \n", __func__);    
		}
	}
	
	if(config_info.ctp_used == 0)
	{
		sterr("*** ctp_used set to 0 !\n");
		sterr("*** if use ctp,please put the sys_config.fex ctp_used set to 1. \n");
		return 0;
	}

	if(!ctp_get_system_config())
	{
		sterr("%s:read config fail!\n",__func__);
		return ret;
	}
	stmsg("%s\n",__func__);
	
	//input_set_power_enable(&(config_info.input_type), 1);
	
	//sunxi_gpio_to_name(CTP_IRQ_NUMBER,irq_pin_name);
	
	//sitronix_i2c_driver.detect = ctp_detect;
#endif

	stmsg("Sitronix touch driver %d.%d.%d\n", DRIVER_MAJOR, DRIVER_MINOR, DRIVER_PATCHLEVEL);
	stmsg("Release date: %s\n", DRIVER_DATE);
#ifdef SITRONIX_FW_UPGRADE_FEATURE
	result  = alloc_chrdev_region(&devno, 0, 1, SITRONIX_I2C_TOUCH_DEV_NAME);
	if(result < 0){
		sterr("fail to allocate chrdev (%d) \n", result);
		return 0;
	}
	sitronix_major = MAJOR(devno);
	cdev_init(&sitronix_cdev, &nc_fops);
	sitronix_cdev.owner = THIS_MODULE;
	sitronix_cdev.ops = &nc_fops;
        err =  cdev_add(&sitronix_cdev, devno, 1);
	if(err){
		sterr("fail to add cdev (%d) \n", err);
		return 0;
	}
	stmsg("%s,%d\n",__FUNCTION__,__LINE__);

	sitronix_class = class_create(THIS_MODULE, SITRONIX_I2C_TOUCH_DEV_NAME);
	if (IS_ERR(sitronix_class)) {
		result = PTR_ERR(sitronix_class);
		unregister_chrdev(sitronix_major, SITRONIX_I2C_TOUCH_DEV_NAME);
		sterr("fail to create class (%d) \n", result);
		return result;
	}
	stmsg("%s,%d\n",__FUNCTION__,__LINE__);
	device_create(sitronix_class, NULL, MKDEV(sitronix_major, 0), NULL, SITRONIX_I2C_TOUCH_DEV_NAME);
#ifdef SITRONIX_PERMISSION_THREAD
	SitronixPermissionThread = kthread_run(sitronix_ts_permission_thread,"Sitronix","Permissionthread");
	if(IS_ERR(SitronixPermissionThread))
		SitronixPermissionThread = NULL;
#endif // SITRONIX_PERMISSION_THREAD
#endif // SITRONIX_FW_UPGRADE_FEATURE

	ret_iic= i2c_add_driver(&sitronix_ts_driver);
	stinf("IIC ADD DRIVER %s,ret_iic=%d\n",__FUNCTION__,ret_iic);

	return  ret_iic;
	//return i2c_add_driver(&sitronix_ts_driver);
}

static void __exit sitronix_ts_exit(void)
{
#ifdef SITRONIX_FW_UPGRADE_FEATURE
	dev_t dev_id = MKDEV(sitronix_major, 0);
#endif // SITRONIX_FW_UPGRADE_FEATURE
	i2c_del_driver(&sitronix_ts_driver);
#ifdef SITRONIX_FW_UPGRADE_FEATURE
	cdev_del(&sitronix_cdev);

	device_destroy(sitronix_class, dev_id); //delete device node under /dev
	class_destroy(sitronix_class); //delete class created by us
	unregister_chrdev_region(dev_id, 1);
#ifdef SITRONIX_PERMISSION_THREAD
	if(SitronixPermissionThread)
		SitronixPermissionThread = NULL;
#endif // SITRONIX_PERMISSION_THREAD
#endif // SITRONIX_FW_UPGRADE_FEATURE
}



#if defined(ST_TEST_RAW) || defined(ST_UPGRADE_FIRMWARE)

int st_i2c_read_bytes(u8 addr,u8 *rxbuf,int len)
{
	
	int ret = 0;
	st_u8 txbuf = addr;
	
	ret = i2c_master_send(sitronix_ts_gpts.client, &txbuf, 1);
	if (ret < 0){
		sterr("write 0x%x error (%d)\n", addr, ret);
		return ret;
	}
	usleep_range(50,100);
	ret = i2c_master_recv(sitronix_ts_gpts.client, rxbuf, len);

	if (ret < 0){
		sterr("read 0x%x error (%d)\n", addr, ret);
		return ret;
	}
	usleep_range(50,100);
	return len;
}

int st_i2c_write_bytes(st_u8 *txbuf, int len)
{
	
	int ret = 0;
	if(txbuf == NULL)
		return -1;

	ret = i2c_master_send(sitronix_ts_gpts.client, txbuf, len);
	if (ret < 0){
		sterr("write 0x%x error (%d)\n", *txbuf, ret);
		return ret;
	}
	usleep_range(50,100);
	return len;
}

int st_irq_off(void)
{
	if (sitronix_ts_gpts.use_irq){
		atomic_set(&sitronix_ts_irq_on, 0);
		disable_irq_nosync(sitronix_ts_gpts.client->irq);
	}
	return 0;
}
int st_irq_on(void)
{
	if (sitronix_ts_gpts.use_irq)
	{
		atomic_set(&sitronix_ts_irq_on, 1);
		enable_irq(sitronix_ts_gpts.client->irq);
	}
	return 0;
}
#endif

module_init(sitronix_ts_init);
module_exit(sitronix_ts_exit);

MODULE_DESCRIPTION("Sitronix Multi-Touch Driver");
MODULE_LICENSE("GPL");
