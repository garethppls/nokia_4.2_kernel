/*
 * LED Heartbeat Trigger
 *
 * Copyright (C) 2006 Atsushi Nemoto <anemo@mba.ocn.ne.jp>
 *
 * Based on Richard Purdie's ledtrig-timer.c and some arch's
 * CONFIG_HEARTBEAT code.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/timer.h>
#include <linux/sched.h>
#include <linux/leds.h>
#include <linux/reboot.h>
#include "../leds.h"

static int panic_heartbeats;

struct heartbeat_trig_data {
	unsigned int phase;
	unsigned int period;
	struct timer_list timer;
	unsigned int invert;
};

static void led_heartbeat_function(unsigned long data)
{
	struct led_classdev *led_cdev = (struct led_classdev *) data;
	struct heartbeat_trig_data *heartbeat_data = led_cdev->trigger_data;
	unsigned long brightness = LED_OFF;
	unsigned long delay = 0;

	if (unlikely(panic_heartbeats)) {
		led_set_brightness_nosleep(led_cdev, LED_OFF);
		return;
	}

	//Single timel 4 4 3 2 1 0 0 0 0 0
	switch (heartbeat_data->phase) {
		case 0:
			
			heartbeat_data->phase++;
			
			brightness = 36;
			break;
		case 1:
			
			heartbeat_data->phase++;
			
			brightness = 35;
			break;
		case 2:
			
			heartbeat_data->phase++;
			
			brightness = 33;
			break;
		case 3:
			heartbeat_data->phase++;
			
			brightness = 31;
			break;
		case 4:
			heartbeat_data->phase++;
			
			brightness = 29;
			break;
		case 5:
			heartbeat_data->phase++;
			
			brightness = 27;
			break;
		case 6:
			heartbeat_data->phase++;
			
			brightness = 25;
			break;
		case 7:
			heartbeat_data->phase++;
			
			brightness = 23;
			break;
		case 8:
			heartbeat_data->phase++;
			
			brightness = 21;
			break;
		case 9:
			heartbeat_data->phase++;
			
			brightness = 19;
			break;
		case 10:
			
			heartbeat_data->phase++;
			
			brightness = 17;
			break;
		case 11:
			
			heartbeat_data->phase++;
			
			brightness = 15;
			break;
		case 12:
			
			heartbeat_data->phase++;
			
			brightness = 13;
			break;
		case 13:
			heartbeat_data->phase++;
			
			brightness = 12;
			break;
		case 14:
			heartbeat_data->phase++;
			
			brightness = 10;
			break;
		case 15:
			heartbeat_data->phase++;
			
			brightness = 8;
			break;
		case 16:
			heartbeat_data->phase++;
			
			brightness = 6;
			break;
		case 17:
			heartbeat_data->phase++;
			
			brightness = 4;
			break;
		case 18:
			heartbeat_data->phase++;
			
			brightness = 2;
			break;
		case 19:
			heartbeat_data->phase++;
			
			brightness = 0;
			break;
		default:
			heartbeat_data->phase++;
			
			brightness = 0;
			printk("wingtech-led<%s-%d>\n",__func__,__LINE__);
			break;
	}
	if (heartbeat_data->phase != 21){
		//led_set_brightness_nosleep(led_cdev, brightness);
		led_set_brightness(led_cdev, brightness);
		led_cdev->usr_brightness_req = brightness;		
		delay = msecs_to_jiffies(50);
		mod_timer(&heartbeat_data->timer, jiffies + delay);
	}else if (heartbeat_data->phase == 21){
		printk("wingtech-led:singletime over\n");
	}
}


static ssize_t led_invert_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct heartbeat_trig_data *heartbeat_data = led_cdev->trigger_data;

	return sprintf(buf, "%u\n", heartbeat_data->invert);
}

static ssize_t led_invert_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct heartbeat_trig_data *heartbeat_data = led_cdev->trigger_data;
	unsigned long state;
	int ret;

	ret = kstrtoul(buf, 0, &state);
	if (ret)
		return ret;

	heartbeat_data->invert = !!state;

	return size;
}

static DEVICE_ATTR(invert, 0644, led_invert_show, led_invert_store);

static void heartbeat_trig_activate(struct led_classdev *led_cdev)
{
	struct heartbeat_trig_data *heartbeat_data;
	int rc;

	heartbeat_data = kzalloc(sizeof(*heartbeat_data), GFP_KERNEL);
	if (!heartbeat_data)
		return;

	led_cdev->trigger_data = heartbeat_data;
	rc = device_create_file(led_cdev->dev, &dev_attr_invert);
	if (rc) {
		kfree(led_cdev->trigger_data);
		return;
	}

	setup_timer(&heartbeat_data->timer,
		    led_heartbeat_function, (unsigned long) led_cdev);
	heartbeat_data->phase = 0;
	led_heartbeat_function(heartbeat_data->timer.data);
	led_cdev->activated = true;
}

static void heartbeat_trig_deactivate(struct led_classdev *led_cdev)
{
	struct heartbeat_trig_data *heartbeat_data = led_cdev->trigger_data;

	if (led_cdev->activated) {
		del_timer_sync(&heartbeat_data->timer);
		device_remove_file(led_cdev->dev, &dev_attr_invert);
		kfree(heartbeat_data);
		led_cdev->activated = false;
	}
}

static struct led_trigger heartbeat_led_trigger = {
	.name     = "singletimedark",
	.activate = heartbeat_trig_activate,
	.deactivate = heartbeat_trig_deactivate,
};

static int heartbeat_reboot_notifier(struct notifier_block *nb,
				     unsigned long code, void *unused)
{
	led_trigger_unregister(&heartbeat_led_trigger);
	return NOTIFY_DONE;
}

static int heartbeat_panic_notifier(struct notifier_block *nb,
				     unsigned long code, void *unused)
{
	panic_heartbeats = 1;
	return NOTIFY_DONE;
}

static struct notifier_block heartbeat_reboot_nb = {
	.notifier_call = heartbeat_reboot_notifier,
};

static struct notifier_block heartbeat_panic_nb = {
	.notifier_call = heartbeat_panic_notifier,
};

static int __init heartbeat_trig_init(void)
{
	int rc = led_trigger_register(&heartbeat_led_trigger);

	if (!rc) {
		atomic_notifier_chain_register(&panic_notifier_list,
					       &heartbeat_panic_nb);
		register_reboot_notifier(&heartbeat_reboot_nb);
	}
	return rc;
}

static void __exit heartbeat_trig_exit(void)
{
	unregister_reboot_notifier(&heartbeat_reboot_nb);
	atomic_notifier_chain_unregister(&panic_notifier_list,
					 &heartbeat_panic_nb);
	led_trigger_unregister(&heartbeat_led_trigger);
}

module_init(heartbeat_trig_init);
module_exit(heartbeat_trig_exit);

MODULE_AUTHOR("Atsushi Nemoto <anemo@mba.ocn.ne.jp>");
MODULE_DESCRIPTION("Heartbeat LED trigger");
MODULE_LICENSE("GPL");
