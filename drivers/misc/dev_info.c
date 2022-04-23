#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <asm/uaccess.h>

#include <linux/proc_fs.h>

#ifndef __HARDWARE_H__
#define __HARDWARE_H__

#define HARDWARE_MAX_ITEM_LONGTH		64
char tp_info[40]={"NO THIS DEVICE"};
char lcd_info[40]={"NO THIS DEVICE"};
char camera_f_info[40]={"NO THIS DEVICE"};
char camera_bsec_info[40]={"NO THIS DEVICE"};
char camera_bmain_info[40]={"NO THIS DEVICE"};
char g_sensor_info[40]={"NO THIS DEVICE"};
char e_sensor_info[20]={"NO THIS DEVICE"};
char l_sensor_info[40]={"NO THIS DEVICE"};
char p_sensor_info[20]={"NO THIS DEVICE"};
char mcp_info[40] = {"NO THIS DEVICE"};
char gyro_sensor_info[20]={"NO THIS DEVICE"};
#ifdef CONFIG_PROJECT_T89572
char charge_ic_info[20]={"SMB1360"};
char battery_info1[40]={"FH3000"};
char coulomb_counter[20]={"PM215"};
#else
char charge_ic_info[20]={"NO THIS DEVICE"};
char battery_info1[40]={"JIADE-ATL"};
char coulomb_counter[20]={"NO THIS DEVICE"};
#endif
char emmc_info[20]={"NO THIS DEVICE"}; // /dev/dev_info add emmc_info
char fp_info[40]={"NO THIS DEVICE"};
char sar_sensor_info[20]={"NO THIS DEVICE"};
enum{
	HARDWARE_LCD = 0,
	HARDWARE_TP,
	HARDWARE_FLASH,
	HARDWARE_FRONT_CAM,
	HARDWARE_BACK_CAM,
	HARDWARE_BT,
	HARDWARE_WIFI,
	HARDWARE_ACCELEROMETER,
	HARDWARE_ALSPS,
	HARDWARE_GYROSCOPE,
	HARDWARE_MAGNETOMETER,	
	HARDWARE_GPS,
	HARDWARE_FM,
	HARDWARE_SDSTATUS,
	HARDWARE_FRONT_CAM_ID,
	HARDWARE_BACK_CAM_ID,
	HARDWARE_SEC_INFO,
	SOFTWARE_SEC_INFO,
	HARDWARE_SKU_INFO,
	HARDWARE_BORD_ID_INFO,
	HARDWARE_CHARGER_IC_INFO,
	HARDWARE_NFC_INFO,
	HARDWARE_FINGER_INFO,
	HARDWARE_BATTERY_INFO,
        HARDWARE_SAR_INFO,
	HARDWARE_COULOMB_COUNTER_INFO,	
	HARDWARE_MAX_ITEM
};


#define HARDWARE_ID						'H'
#define HARDWARE_LCD_GET					_IOWR(HARDWARE_ID, 0x01, char[HARDWARE_MAX_ITEM_LONGTH])
#define HARDWARE_TP_GET						_IOWR(HARDWARE_ID, 0x02, char[HARDWARE_MAX_ITEM_LONGTH])
#define HARDWARE_FLASH_GET					_IOWR(HARDWARE_ID, 0x03, char[HARDWARE_MAX_ITEM_LONGTH])
#define HARDWARE_FRONT_CAM_GET					_IOWR(HARDWARE_ID, 0x04, char[HARDWARE_MAX_ITEM_LONGTH])
#define HARDWARE_BACK_CAM_GET					_IOWR(HARDWARE_ID, 0x05, char[HARDWARE_MAX_ITEM_LONGTH])
#define HARDWARE_ACCELEROMETER_GET				_IOWR(HARDWARE_ID, 0x06, char[HARDWARE_MAX_ITEM_LONGTH])
#define HARDWARE_ALSPS_GET			    		_IOWR(HARDWARE_ID, 0x07, char[HARDWARE_MAX_ITEM_LONGTH])
#define HARDWARE_GYROSCOPE_GET					_IOWR(HARDWARE_ID, 0x08, char[HARDWARE_MAX_ITEM_LONGTH])
#define HARDWARE_MAGNETOMETER_GET				_IOWR(HARDWARE_ID, 0x09, char[HARDWARE_MAX_ITEM_LONGTH])
#define HARDWARE_BT_GET						_IOWR(HARDWARE_ID, 0x10, char[HARDWARE_MAX_ITEM_LONGTH])
#define HARDWARE_WIFI_GET					_IOWR(HARDWARE_ID, 0x11, char[HARDWARE_MAX_ITEM_LONGTH])
#define HARDWARE_GPS_GET			    		_IOWR(HARDWARE_ID, 0x12, char[HARDWARE_MAX_ITEM_LONGTH])
#define HARDWARE_FM_GET			        		_IOWR(HARDWARE_ID, 0x13, char[HARDWARE_MAX_ITEM_LONGTH])
#define HARDWARE_SDCARD_STATUS					_IOWR(HARDWARE_ID, 0x14, char[HARDWARE_MAX_ITEM_LONGTH])
#define HARDWARE_BATTERY_ID_GET					_IOWR(HARDWARE_ID, 0x15, char[HARDWARE_MAX_ITEM_LONGTH])
#define HARDWARE_FRONT_CAM_ID_GET				_IOWR(HARDWARE_ID, 0x16, char[HARDWARE_MAX_ITEM_LONGTH])
#define HARDWARE_BACK_CAM_ID_GET				_IOWR(HARDWARE_ID, 0x17, char[HARDWARE_MAX_ITEM_LONGTH])

#define HARDWARE_NFC_GET					_IOWR(HARDWARE_ID, 0x24, char[HARDWARE_MAX_ITEM_LONGTH])
#define HARDWARE_FINGER_GET					_IOWR(HARDWARE_ID, 0x25, char[HARDWARE_MAX_ITEM_LONGTH])
#define HARDWARE_BATTERY_INFO_GET  				_IOWR(HARDWARE_ID, 0x26, char[HARDWARE_MAX_ITEM_LONGTH])
#define HARDWARE_SAR_GET					_IOWR(HARDWARE_ID, 0x33, char[HARDWARE_MAX_ITEM_LONGTH])

#define HARDWARE_SKU_INFO_GET  _IOWR(HARDWARE_ID, 0x21, char[HARDWARE_MAX_ITEM_LONGTH])    
#define HARDWARE_BOARD_ID_GET  _IOWR(HARDWARE_ID, 0x18, char[HARDWARE_MAX_ITEM_LONGTH])
#define HARDWARE_CHARGER_IC_INFO_GET  _IOWR(HARDWARE_ID, 0x86, char[HARDWARE_MAX_ITEM_LONGTH])	
#define HARDWARE_COULOMB_COUNTER_INFO_GET  _IOWR(HARDWARE_ID, 0x89, char[HARDWARE_MAX_ITEM_LONGTH])	
//#define HARDWARE_SECURE_INFO_GET  _IOWR(HARDWARE_ID, 0x19, char[HARDWARE_MAX_ITEM_LONGTH])
#define SOFTWARE_SECURE_INFO_GET  _IOWR(HARDWARE_ID, 0x20, char[HARDWARE_MAX_ITEM_LONGTH])



int hardwareinfo_set_prop(int cmd, char *name);

#endif //__HARDWARE_H__


//end hardware_info.h add wtf


extern char mtkfb_lcm_name[];//wt.wangpei add for LCD factory
#if 0
#ifdef CONFIG_MTK_EMMC_SUPPORT
extern char emmc_device_name[];//wt.wangpei modify for flash info
static int __init set_emmc_name(char *str)
{
    int i = 0;
	printk("%s : %s\n",__func__,str);
	for(i = 0;  (*(str + i) != ' ' && *(str + i) != '\0'); i++);
	strncpy((char*)emmc_device_name, (char*)str, ((i<29) ? i:29));	
	if(i>=29)
	{
		emmc_device_name[29]='\0';
	}
	return 1;
}
__setup("emmc_name=", set_emmc_name);
#endif
#endif
#ifdef WT_LCM_DEBUG   
static struct proc_dir_entry *lcmdebug_proc_entry;

void (*lcmdebug_write_function)(unsigned char* buffer, int len);
#endif	


static char hardwareinfo_name[HARDWARE_MAX_ITEM][HARDWARE_MAX_ITEM_LONGTH];

int hardwareinfo_set_prop(int cmd, char *name)
{
	// printk("%s%d,wtf add\n",__FUNCTION__,__LINE__);
	if(cmd < 0 || cmd >= HARDWARE_MAX_ITEM)
		return -1;

	strcpy(hardwareinfo_name[cmd], name);

	return 0;
}
EXPORT_SYMBOL_GPL(hardwareinfo_set_prop);
#ifdef WT_LCM_DEBUG
EXPORT_SYMBOL_GPL(lcmdebug_write_function);
#endif

static int hardwareinfo_ioctl_open(struct inode *inode, struct file *file)
{
	//printk("%s%d,wtf add\n",__FUNCTION__,__LINE__);
       return nonseekable_open(inode, file);
}
static int hardwareinfo_ioctl_release(struct inode *inode, struct file *file)
{       
        //printk("%s%d,wtf add\n",__FUNCTION__,__LINE__);
        file->private_data = NULL;
        return 0;
}
static long hardwareinfo_ioctl(struct file *file, unsigned int cmd,unsigned long arg)
{
    
	
	int ret = 0,hardwareinfo_num;
	void __user *data = (void __user *)arg;
	//printk("%s%d,wtf add\n",__FUNCTION__,__LINE__);
	switch (cmd) {
	case HARDWARE_LCD_GET:
	//printk("%s%d,wtf add\n",__FUNCTION__,__LINE__);
		hardwareinfo_num = HARDWARE_LCD;
		//hardwareinfo_set_prop(HARDWARE_LCD, "lcd");
		hardwareinfo_set_prop(HARDWARE_LCD, lcd_info);//wt.yangzheng add for LCD factory
		break;
	case HARDWARE_TP_GET:
		//printk("%s%d,wtf add\n",__FUNCTION__,__LINE__);
		hardwareinfo_num = HARDWARE_TP;
		hardwareinfo_set_prop(HARDWARE_TP, tp_info);
		break;
	case HARDWARE_FLASH_GET:
		hardwareinfo_num = HARDWARE_FLASH;
		hardwareinfo_set_prop(HARDWARE_FLASH,emmc_info);

		break;
	case HARDWARE_FRONT_CAM_GET:
		hardwareinfo_num = HARDWARE_FRONT_CAM;
	 	//hardwareinfo_set_prop(HARDWARE_FRONT_CAM,camera_f_info);
		break;
	case HARDWARE_BACK_CAM_GET:
		hardwareinfo_num = HARDWARE_BACK_CAM;
		//hardwareinfo_set_prop(HARDWARE_BACK_CAM,camera_bmain_info);
		break;
	case HARDWARE_BT_GET:
	hardwareinfo_set_prop(HARDWARE_BT, "WCN3615");
		hardwareinfo_num = HARDWARE_BT;
		break;
	case HARDWARE_WIFI_GET:
		hardwareinfo_set_prop(HARDWARE_WIFI, "WCN3615");
		hardwareinfo_num = HARDWARE_WIFI;
		break;	
	case HARDWARE_ACCELEROMETER_GET:
		hardwareinfo_num = HARDWARE_ACCELEROMETER;
		hardwareinfo_set_prop(HARDWARE_ACCELEROMETER,g_sensor_info);
		break;
	case HARDWARE_ALSPS_GET:
		hardwareinfo_num = HARDWARE_ALSPS;
		hardwareinfo_set_prop(HARDWARE_ALSPS,p_sensor_info);
		break;
	case HARDWARE_GYROSCOPE_GET:
		hardwareinfo_num = HARDWARE_GYROSCOPE;
		hardwareinfo_set_prop(HARDWARE_GYROSCOPE,gyro_sensor_info);
		break;
	case HARDWARE_MAGNETOMETER_GET:
		hardwareinfo_num = HARDWARE_MAGNETOMETER;
		hardwareinfo_set_prop(HARDWARE_MAGNETOMETER,e_sensor_info);
		break;		
	case HARDWARE_GPS_GET:
		hardwareinfo_set_prop(HARDWARE_GPS, "Qualcomm");
	    hardwareinfo_num = HARDWARE_GPS;
		break;
	case HARDWARE_FM_GET:
		hardwareinfo_set_prop(HARDWARE_FM, "WCN3615");
	    hardwareinfo_num = HARDWARE_FM;		
		break;
	case HARDWARE_SDCARD_STATUS:
	    hardwareinfo_num = HARDWARE_SDSTATUS;
		hardwareinfo_set_prop(HARDWARE_FLASH,mcp_info);		
		break;
	case HARDWARE_FRONT_CAM_ID_GET:
	   hardwareinfo_num = HARDWARE_FRONT_CAM_ID;
		break;
	case HARDWARE_BACK_CAM_ID_GET:
	    hardwareinfo_num = HARDWARE_BACK_CAM_ID;
		break;
	//case HARDWARE_SECURE_INFO_GET:
		//hardwareinfo_num = HARDWARE_SEC_INFO;
		//hardwareinfo_set_prop(HARDWARE_SEC_INFO, ret_hw_secure_info); 
		//break;
	//case SOFTWARE_SECURE_INFO_GET:
		//hardwareinfo_num = SOFTWARE_SEC_INFO;
		//hardwareinfo_set_prop(SOFTWARE_SEC_INFO, ret_sw_secure_info); 
		//break;
    case HARDWARE_SKU_INFO_GET:    
		hardwareinfo_num = HARDWARE_SKU_INFO;
		break;
   case HARDWARE_BOARD_ID_GET:
		hardwareinfo_num = HARDWARE_BORD_ID_INFO;
		break;
   case HARDWARE_CHARGER_IC_INFO_GET:
		hardwareinfo_set_prop(HARDWARE_CHARGER_IC_INFO,charge_ic_info);		
		hardwareinfo_num = HARDWARE_CHARGER_IC_INFO;
		break;
   case HARDWARE_NFC_GET:
		hardwareinfo_num = HARDWARE_NFC_INFO;
		hardwareinfo_set_prop(HARDWARE_NFC_INFO,"ST21NFC");
		break;
   case HARDWARE_SAR_GET:
		hardwareinfo_num = HARDWARE_SAR_INFO;
		hardwareinfo_set_prop(HARDWARE_SAR_INFO, sar_sensor_info);
		break;
   case HARDWARE_FINGER_GET:
		hardwareinfo_num = HARDWARE_FINGER_INFO;
		hardwareinfo_set_prop(HARDWARE_FINGER_INFO,fp_info);
		break;
   case HARDWARE_BATTERY_ID_GET:	
   case HARDWARE_BATTERY_INFO_GET:
		hardwareinfo_num = HARDWARE_BATTERY_INFO;
		hardwareinfo_set_prop(HARDWARE_BATTERY_INFO,battery_info1);
		break;
   case HARDWARE_COULOMB_COUNTER_INFO_GET:
		hardwareinfo_num = HARDWARE_COULOMB_COUNTER_INFO;
		hardwareinfo_set_prop(HARDWARE_COULOMB_COUNTER_INFO,coulomb_counter);
		break;
	default:
		ret = -EINVAL;
		goto err_out;
	}
	if (copy_to_user(data, hardwareinfo_name[hardwareinfo_num], strlen(hardwareinfo_name[hardwareinfo_num])+1)){
	   printk("%s, copy to usr error\n", __func__);
		ret =  -EINVAL;
	}
err_out:
	return ret;

}

#ifdef WT_LCM_DEBUG  
unsigned char lcm_debug_cmd[128];
unsigned char data_buffer[128];
int str2int8(unsigned char *buffer)
{
   int i = 0;
   
   for(i = 0; i < (strlen(buffer)-1)/2; i++)
   {
      if(buffer[2*i] >='0' && buffer[2*i] <= '9')
      {
          data_buffer[i] = (buffer[2*i] - '0');
      }
      else if(buffer[2*i] >='a' && buffer[2*i] <= 'f')
      {
          data_buffer[i] = (buffer[2*i] - 'a' + 0xa);
      }
      else if(buffer[2*i] >='A' && buffer[2*i] <= 'F')
      {
          data_buffer[i] = (buffer[2*i] - 'A' + 0xA);
      }
      
      data_buffer[i] = data_buffer[i] << 4;

      if(buffer[2*i+1] >='0' && buffer[2*i+1] <= '9')
      {
          data_buffer[i] |= (buffer[2*i+1] - '0');
      }
      else if(buffer[2*i+1] >='a' && buffer[2*i+1] <= 'f')
      {
          data_buffer[i] |= (buffer[2*i+1] - 'a' + 0xa);
      }
      else if(buffer[2*i+1] >='A' && buffer[2*i+1] <= 'F')
      {
          data_buffer[i] |= (buffer[2*i+1] - 'A' + 0xA);
      }
   }
   
    return i;
}
static s32 lcmdebug_write(struct file *filp, const char __user *buff, unsigned long len, void *data)
{
    u64 ret = 0;
    int data_len = 0;

memset(lcm_debug_cmd, 0, sizeof(lcm_debug_cmd));
memset(data_buffer, 0, sizeof(data_buffer));

    ret = copy_from_user(&lcm_debug_cmd, buff, len);
    if (ret)
    {
        printk("lcm_debug:copy_from_user failed.");
    }

	if(lcmdebug_write_function != NULL && !strncmp(lcm_debug_cmd, "debug:", strlen("debug:")))
	{
                data_len = str2int8(lcm_debug_cmd + strlen("debug:"));
		lcmdebug_write_function(data_buffer, data_len);
	}

return len;

}


void init_lcmdebug_node()
{
    lcmdebug_proc_entry = create_proc_entry("lcm_debug", 0666, NULL);	 
    if (lcmdebug_proc_entry == NULL)    
	{		
	       printk("lcm_debug:Couldn't create proc entry!");		 
	       //return FALSE;	 
	}	  
    else	  
	{ 	   printk("lcm_debug:Create proc entry success!");	
           lcmdebug_proc_entry->write_proc = lcmdebug_write;		
           //return TRUE;
    }
}
#endif

static struct file_operations hardwareinfo_fops = {
    .open = hardwareinfo_ioctl_open,
    .release = hardwareinfo_ioctl_release,
    .unlocked_ioctl = hardwareinfo_ioctl,
};

static struct miscdevice hardwareinfo_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "hardwareinfo",
	.fops = &hardwareinfo_fops,
};


static int __init hardwareinfo_init_module(void)
{
	int ret, i;
//	int fd = -1;

	for(i = 0; i < HARDWARE_MAX_ITEM; i++)
		strcpy(hardwareinfo_name[i], "NULL");
	
	ret = misc_register(&hardwareinfo_device);
	if(ret < 0){
		printk("%s, misc_register error\n", __func__);
		return -ENODEV;
	}
	printk("%s, misc_register sucessful\n", __func__);
#ifdef WT_LCM_DEBUG  
	init_lcmdebug_node();
#endif

	
	return 0;
}

static void __exit hardwareinfo_exit_module(void)
{
	misc_deregister(&hardwareinfo_device);
}

subsys_initcall(hardwareinfo_init_module);
module_exit(hardwareinfo_exit_module);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Ming He <heming@wingtech.com>");
