/*program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/module.h>
#include <linux/export.h>
#include "msm_camera_io_util.h"
#include "msm_flash.h"
#include <linux/cdev.h>       /* cdev_init */ //+ yinjie1.wt add flashlight node ++
#include <linux/platform_device.h>
#include <linux/of_gpio.h>




#define FLASH_NAME "qcom,gpio-flash"

#undef CDBG
#define CDBG(fmt, args...) pr_debug(fmt, ##args)

static const struct of_device_id msm_gpio_flash_dt_match[] = {
	{.compatible = "qcom,qm215-gpio-flash", .data = NULL},
	{}
};
static struct msm_flash_table qm215_gpio_flash_table;
MODULE_DEVICE_TABLE(of, msm_flash_dt_match);

static int32_t qm215_flash_low(
	struct msm_flash_ctrl_t *flash_ctrl,
	struct msm_flash_cfg_data_t *flash_data)
{
	struct msm_camera_power_ctrl_t *power_info = NULL;
	struct msm_camera_gpio_num_info *gpio_num_info = NULL;

	if (!flash_ctrl) {
		pr_err("device data NULL\n");
		return -EINVAL;
	}

	CDBG("Enter");
	power_info = &flash_ctrl->power_info;
	gpio_num_info = power_info->gpio_conf->gpio_num_info;

	if (flash_ctrl->flash_driver_type == FLASH_DRIVER_GPIO &&
		gpio_num_info->valid[SENSOR_GPIO_FL_NOW] &&
			gpio_num_info->gpio_num[SENSOR_GPIO_FL_EN]) {

		CDBG("flash op low gpio num %d(state:%d) %d(state: %d)\n",
			gpio_num_info->gpio_num[SENSOR_GPIO_FL_NOW],
			GPIO_OUT_HIGH,
			gpio_num_info->gpio_num[SENSOR_GPIO_FL_EN],
			GPIO_OUT_LOW); //2019.11.2 JLQ modify for flash
		gpio_set_value_cansleep(
			gpio_num_info->gpio_num[SENSOR_GPIO_FL_NOW],
			GPIO_OUT_HIGH);
		gpio_set_value_cansleep(
			gpio_num_info->gpio_num[SENSOR_GPIO_FL_EN],
			GPIO_OUT_LOW);  //2019.11.2 JLQ modify for flash
	}
	CDBG("Exit\n");
	return 0;
}

static int32_t qm215_flash_high(
	struct msm_flash_ctrl_t *flash_ctrl,
	struct msm_flash_cfg_data_t *flash_data)
{
	struct msm_camera_power_ctrl_t *power_info = NULL;
	struct msm_camera_gpio_num_info *gpio_num_info = NULL;

	if (!flash_ctrl) {
		pr_err("device data NULL\n");
		return -EINVAL;
	}

	CDBG("Enter\n");
	power_info = &flash_ctrl->power_info;
	gpio_num_info = power_info->gpio_conf->gpio_num_info;

	if (flash_ctrl->flash_driver_type == FLASH_DRIVER_GPIO &&
		gpio_num_info->valid[SENSOR_GPIO_FL_NOW] &&
		gpio_num_info->gpio_num[SENSOR_GPIO_FL_EN]) {

		CDBG("flash op high gpio num %d(state:%d) %d(state:%d)\n",
			gpio_num_info->gpio_num[SENSOR_GPIO_FL_NOW],
			GPIO_OUT_LOW,
			gpio_num_info->gpio_num[SENSOR_GPIO_FL_EN],
			GPIO_OUT_HIGH);
		gpio_set_value_cansleep(
			gpio_num_info->gpio_num[SENSOR_GPIO_FL_NOW],
			GPIO_OUT_LOW);
		gpio_set_value_cansleep(
			gpio_num_info->gpio_num[SENSOR_GPIO_FL_EN],
			GPIO_OUT_HIGH);
	}
	CDBG("Exit\n");

	return 0;
}

static int32_t qm215_flash_release(
	struct msm_flash_ctrl_t *flash_ctrl)
{
	int32_t rc = 0;

	if (!flash_ctrl) {
		pr_err("device data NULL\n");
		return -EINVAL;
	}

	CDBG("Enter\n");
	rc = flash_ctrl->func_tbl->camera_flash_off(flash_ctrl, NULL);
	if (rc < 0) {
		pr_err("%s:%d camera_flash_init failed rc = %d",
			__func__, __LINE__, rc);
		return rc;
	}
	flash_ctrl->flash_state = MSM_CAMERA_FLASH_RELEASE;
	CDBG("Exit\n");
	return 0;
}

static int32_t qm215_flash_off(struct msm_flash_ctrl_t *flash_ctrl,
	struct msm_flash_cfg_data_t *flash_data)
{
	struct msm_camera_power_ctrl_t *power_info = NULL;
	struct msm_camera_gpio_num_info *gpio_num_info = NULL;

	if (!flash_ctrl) {
		pr_err("device data NULL\n");
		return -EINVAL;
	}

	CDBG("Enter\n");
	power_info = &flash_ctrl->power_info;
	gpio_num_info = power_info->gpio_conf->gpio_num_info;

	if (flash_ctrl->flash_driver_type == FLASH_DRIVER_GPIO &&
		gpio_num_info->valid[SENSOR_GPIO_FL_NOW] &&
		gpio_num_info->gpio_num[SENSOR_GPIO_FL_EN]) {

		CDBG("flash off gpio num %d(state:%d) %d(state: %d)\n",
			gpio_num_info->gpio_num[SENSOR_GPIO_FL_NOW],
			GPIO_OUT_LOW,
			gpio_num_info->gpio_num[SENSOR_GPIO_FL_EN],
			GPIO_OUT_LOW);
		gpio_set_value_cansleep(
			gpio_num_info->gpio_num[SENSOR_GPIO_FL_NOW],
			GPIO_OUT_LOW);
		gpio_set_value_cansleep(
			gpio_num_info->gpio_num[SENSOR_GPIO_FL_EN],
			GPIO_OUT_LOW);
	}

	CDBG("Exit\n");
	return 0;
}

static int32_t qm215_flash_gpio_init(
	struct msm_flash_ctrl_t *flash_ctrl,
	struct msm_flash_cfg_data_t *flash_data)
{
	int32_t rc = 0;

	CDBG("Enter");
	rc = flash_ctrl->func_tbl->camera_flash_off(flash_ctrl, flash_data);

	CDBG("Exit");
	return rc;
}


static int32_t qm215_platform_flash_init(struct msm_flash_ctrl_t *flash_ctrl,
	struct msm_flash_cfg_data_t *flash_data)
{
	if (!flash_ctrl) {
		pr_err("devices data NULL\n");
		return -EINVAL;
	}

	if (flash_ctrl->flash_driver_type == FLASH_DRIVER_GPIO)
		flash_ctrl->func_tbl = &qm215_gpio_flash_table.func_tbl;

	return 0;
}

static struct class *flashlight_class;
static struct device *flashlight_device;
static int flash=0,flash1=0;


static ssize_t show_flash(struct device *dev, struct device_attribute *attr, char *buf)
{
	pr_err("get backlight duty value is %d\n", flash);
	return sprintf(buf, "%d\n", flash);
}
static ssize_t store_flash(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	char *pvalue=NULL;
	//u8 pmic_subtype ;
	//union power_supply_propval ret = {0, };
	//int rc = 0;
	flash= simple_strtol(buf, &pvalue, 0);
	pr_info("flashduty1= %d\n", flash);
	if(flash>2)
		flash=2;
	if(flash>0){
		flash1=1;
                   // gpio_direction_output(34,1); //torch mode
                    gpio_direction_output(33,1); //torch mode
		pr_debug("wt_flash_flashlight flash=%d\n", flash);
	}else if(flash == 0 && flash1 == 1){
                gpio_direction_output(33,0);
                //gpio_direction_output(34,0);
		flash1=0;
		pr_debug("wt_flash_flashlight 2 flash=%d\n", flash);
	}
	pr_debug("Exit!\n");
	return count;
}
static DEVICE_ATTR(flash, 0664, show_flash, store_flash);

static dev_t flashlight_devno;
#define FLASHLIGHT_DEVNAME            "kd_camera_flashlight"
static struct cdev flashlight_cdev;
static const struct file_operations flashlight_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = NULL,
	.open = NULL,
	.release = NULL,
#ifdef CONFIG_COMPAT
	.compat_ioctl = NULL,
#endif
};

static int32_t qm215_flash_platform_probe(struct platform_device *pdev)
{
	int32_t rc = 0;
	struct msm_flash_ctrl_t *flash_ctrl = NULL;
	struct msm_camera_power_ctrl_t *power_info = NULL;
	struct msm_camera_gpio_conf *gpio_conf = NULL;
	if (!pdev->dev.of_node) {
		pr_err("of_node NULL\n");
		return -EINVAL;
	}
	CDBG("enter probe\n");
	rc = camera_flash_platform_probe(pdev);
	if (rc >= 0) {
		flash_ctrl =
			(struct msm_flash_ctrl_t *) platform_get_drvdata(pdev);
		CDBG("device data %pK\n", flash_ctrl);
		if (!flash_ctrl) {
			pr_err("of_node NULL\n");
			return -EINVAL;
		}
		power_info = &flash_ctrl->power_info;
		gpio_conf = power_info->gpio_conf;
		rc = msm_camera_request_gpio_table(gpio_conf->cam_gpio_req_tbl,
				gpio_conf->cam_gpio_req_tbl_size, 1);
		if (rc < 0) {
			pr_err("%s: request gpio failed\n", __func__);
			return rc;
		}
		flash_ctrl->platform_flash_init = qm215_platform_flash_init;
	}


        rc =0; 
        rc = alloc_chrdev_region(&flashlight_devno, 0, 1, FLASHLIGHT_DEVNAME);
        if (rc) {
            pr_err("alloc_chrdev_region fail: %d ~", rc);
        } else {
            pr_err("major: %d, minor: %d ~", MAJOR(flashlight_devno),MINOR(flashlight_devno));
        }
        cdev_init(&flashlight_cdev, &flashlight_fops);
        flashlight_cdev.owner = THIS_MODULE;
        rc = cdev_add(&flashlight_cdev, flashlight_devno, 1);
        if (rc) {
            pr_err("cdev_add fail: %d ~", rc);
        }

    flashlight_class = class_create(THIS_MODULE, "flashlightdrv");
      if (IS_ERR(flashlight_class)) {
          pr_err("[flashlight_probe] Unable to create class, err = %d ~",
               (int)PTR_ERR(flashlight_class));
          return  -1 ;
      }
      flashlight_device =
          device_create(flashlight_class, NULL, flashlight_devno, NULL, FLASHLIGHT_DEVNAME);
      if (NULL == flashlight_device) {
          pr_err("device_create fail ~");
      }
    
      rc = device_create_file(flashlight_device,&dev_attr_flash);
     if (rc) {
         pr_err("device_create_file flash fail!\n");
     }

	return rc;
}

static struct platform_driver msm_gpio_flash_platform_driver = {
	.probe = qm215_flash_platform_probe,
	.driver = {
		.name = "qcom,camera-gpio-flash",
		.owner = THIS_MODULE,
		.of_match_table = msm_gpio_flash_dt_match,
	},
};

static int __init qm215_gpio_flash_init_module(void)
{
	int32_t rc = 0;

	CDBG("Enter\n");
	rc = platform_driver_register(&msm_gpio_flash_platform_driver);
	if (rc)
		pr_err("platform probe for flash failed");

	return rc;
}

static void __exit qm215_gpio_flash_exit_module(void)
{
	platform_driver_unregister(&msm_gpio_flash_platform_driver);
}

static struct msm_flash_table qm215_gpio_flash_table = {
	.flash_driver_type = FLASH_DRIVER_GPIO,
	.func_tbl = {
		.camera_flash_init = qm215_flash_gpio_init,
		.camera_flash_release = qm215_flash_release,
		.camera_flash_off = qm215_flash_off,
		.camera_flash_low = qm215_flash_low,
		.camera_flash_high = qm215_flash_high,
		.camera_flash_query_current = NULL,
	},
};

module_init(qm215_gpio_flash_init_module);
module_exit(qm215_gpio_flash_exit_module);
MODULE_DESCRIPTION("MSM GPIO FLASH");
MODULE_LICENSE("GPL v2");
