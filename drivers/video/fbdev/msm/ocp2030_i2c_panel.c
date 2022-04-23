#include <linux/init.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
//#include <cust_i2c.h>
//#include "cust_gpio_usage.h"
//#include "lcm_drv.h"
#include "mdss_panel.h"
//#include <linux/debugfs.h>

#define OCP2131_DEV_NAME "ocp2131"


//static struct i2c_board_info __initdata ocp2131_i2c_boardinfo={ I2C_BOARD_INFO(OCP2131_DEV_NAME, 0x3e)};
static const struct i2c_device_id ocp2131_i2c_id[] = {{OCP2131_DEV_NAME,0},{}};

static struct i2c_client *i2c_client = NULL;

static const struct of_device_id panel_i2c_of_match[] = {
	 {
	  .compatible = "qcom,I2C_PANEL_LDO",
	  },
 };

u32 gpio_enn;
//u32 reset_gpio_flags;
u32 gpio_enp;

static int ocp2131_power_enable(int enn , int enp);

struct 
{	
	LCM_LDO_VOLTAGE_E vol;
	int reg_value;
} ocp2131_vol_reg[] ={
	//Frequently-used
	{LCM_LDO_VOL_5V8,0x12},
	{LCM_LDO_VOL_5V5,0x0f},

	{LCM_LDO_VOL_4V0,0x00},
	{LCM_LDO_VOL_4V1,0x01},
	{LCM_LDO_VOL_4V2,0x02},
	{LCM_LDO_VOL_4V3,0x03},
	{LCM_LDO_VOL_4V4,0x04},
	{LCM_LDO_VOL_4V5,0x05},
	{LCM_LDO_VOL_4V6,0x06},
	{LCM_LDO_VOL_4V7,0x07},
	{LCM_LDO_VOL_4V8,0x08},
	{LCM_LDO_VOL_4V9,0x09},
	{LCM_LDO_VOL_5V0,0x0a},
	{LCM_LDO_VOL_5V1,0x0b},
	{LCM_LDO_VOL_5V2,0x0c},
	{LCM_LDO_VOL_5V3,0x0d},
	{LCM_LDO_VOL_5V4,0x0e},
	
	{LCM_LDO_VOL_5V6,0x10},
	{LCM_LDO_VOL_5V7,0x11},
	
	{LCM_LDO_VOL_5V9,0x13},
	{LCM_LDO_VOL_6V0,0x14},

	{LCM_LDO_VOL_MAX,0xff}
};


static int ocp2131_read_chip_info(struct i2c_client *client)
{
        u8 id_data[4] = {0};
        int ret;        
        ret = i2c_smbus_read_i2c_block_data(client,0x03,1,&id_data[0]);
        if(ret < 0)
        {
           return -1;
        }
        printk("%s,0x%x.\n",__FUNCTION__,id_data[0]);
		
        return 0;
}


int ocp2131_set_voltage(LCM_LDO_VOLTAGE_E avdd,LCM_LDO_VOLTAGE_E avee){
	u8 i = 0;
	u8 reg_value_avdd =0xff;
	u8 reg_value_avee =0xff;

	if(avdd == LCM_LDO_VOL_0V || avee == LCM_LDO_VOL_0V){
		//SET_PWR_PIN(0);
                ocp2131_power_enable(0 , 0);
		printk("%s,write config:%x,%x\n",__FUNCTION__,avdd,avee);
		return 0;
	}

	if(i2c_client == NULL)
		return -1;
	
	while(ocp2131_vol_reg[i].vol != LCM_LDO_VOL_MAX){
		if(avdd == ocp2131_vol_reg[i].vol)
			reg_value_avdd = ocp2131_vol_reg[i].reg_value;
		if(avee == ocp2131_vol_reg[i].vol)
			reg_value_avee = ocp2131_vol_reg[i].reg_value;

		if(reg_value_avdd != 0xff && reg_value_avee != 0xff)
			break;
		
		i++;
	}

	if(reg_value_avdd == 0xff || reg_value_avee == 0xff){
		printk("%s,unsupport voltage:%d,%d\n",__FUNCTION__,avdd,avee);
		return -1;
	}

	//SET_PWR_PIN(1);
	//ocp2131_power_enable(1 , 1);
	gpio_direction_output(gpio_enp, 1);
	if(0 != i2c_smbus_write_i2c_block_data(i2c_client, 0, 1, &reg_value_avdd))
		return -1;
	
	mdelay(1);

	gpio_direction_output(gpio_enn, 1);
	if(0 != i2c_smbus_write_i2c_block_data(i2c_client, 1, 1, &reg_value_avee))
		return -1;

	printk("%s,write config:%x,%x\n",__FUNCTION__,avdd,avee);
	return 0;
}
EXPORT_SYMBOL(ocp2131_set_voltage);


static int ocp2131_parse_dt(struct device *dev)
{   
    struct device_node *np = dev->of_node;

    printk("start %s:%d\n", __func__, __LINE__);

    /* enn, enp gpio info */
    gpio_enn = of_get_named_gpio(np,"ocp2131,enn-gpio", 0);
    if (!gpio_is_valid(gpio_enn))
		printk("%s:%d, enn gpio not specified\n",__func__, __LINE__);

    gpio_enp = of_get_named_gpio(np,"ocp2131,enp-gpio", 0);
    if (!gpio_is_valid(gpio_enp))
		printk("%s:%d, enp gpio not specified\n",__func__, __LINE__);

    printk("end %s:%d\n", __func__, __LINE__);
    return 0;
}

//int ocp2131_power_enable(int enn , int enp)
static int ocp2131_gpio_configure(void)
{
    int ret = 0;

   // FTS_FUNC_ENTER();
    /* request enn gpio */
    if (gpio_is_valid(gpio_enn)) {
        ret = gpio_request(gpio_enn, "ocp2131_enn_gpio");
        if (ret) {
            printk("[GPIO]enn gpio request failed");
            goto err_enn_gpio_req;
        }

        /*ret = gpio_direction_output(gpio_enn, 1);
        if (ret) {
           printk("[GPIO]set_direction for enn gpio failed");
            goto err_enn_gpio_dir;
        }*/
    }

    /* request enp gpio */
    if (gpio_is_valid(gpio_enp)) {
        ret = gpio_request(gpio_enp, "ocp2131_enp_gpio");
        if (ret) {
            printk("[GPIO]enp gpio request failed");
            goto err_enp_gpio_dir;
        }

        /*ret = gpio_direction_output(gpio_enp, 1);
        if (ret) {
            printk("[GPIO]set_direction for enp gpio failed");
            goto err_enp_gpio_dir;
        }*/
    }

   // FTS_FUNC_EXIT();
    return 0;

err_enp_gpio_dir:
    if (gpio_is_valid(gpio_enp))
        gpio_free(gpio_enp);
//err_enn_gpio_dir:
//    if (gpio_is_valid(gpio_enn))
//        gpio_free(gpio_enn);
err_enn_gpio_req:
    //FTS_FUNC_EXIT();
    return ret;
}

static int ocp2131_power_enable(int enn , int enp)
{
    //FTS_FUNC_ENTER();
    if(enn)
        gpio_direction_output(gpio_enn, 1);
    else
        gpio_direction_output(gpio_enn, 0);
    
    mdelay(1);

    if(enp)
        gpio_direction_output(gpio_enp, 1);
    else
        gpio_direction_output(gpio_enp, 0);

    //FTS_FUNC_EXIT();
    return 0;
}

static int ocp2131_i2c_remove(struct i2c_client *client)
{
	return 0;
}
/*static int ocp2131_i2c_detect(struct i2c_client *client, int kind, struct i2c_board_info *info) {
	strcpy(info->type, OCP2131_DEV_NAME);
	return 0;
}*/

static int ocp2131_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id) {
	int rc = 0;
	printk("%s probe start\n",__FUNCTION__);

	rc = ocp2131_parse_dt(&client->dev);
	if (rc) {
		printk("%s: failed to parse gpio params, rc=%d\n",
						__func__, rc);
		return -ENODEV;
	}
	
	// enable gpio must be enabled
	if(ocp2131_read_chip_info(client) != 0){
		printk("%s,i2c err!\n",__FUNCTION__);
		return -ENODEV;
	}

	i2c_client = client;

	rc = ocp2131_gpio_configure();
        if (rc) {
            printk("[GPIO]Failed to configure the gpios");
            //goto err_gpio_config;
        }

	printk("%s probe ok\n",__FUNCTION__);
	return 0;
}

//static struct of_device_id fts_match_table[] = {
//    { .compatible = "focaltech,fts", },
//    { },
//};

struct i2c_driver ocp2131_i2c_driver = {
	.driver = {
		.name = OCP2131_DEV_NAME,
		.owner = THIS_MODULE,
        	.of_match_table = panel_i2c_of_match,
	},
	.probe = ocp2131_i2c_probe,
	.remove = ocp2131_i2c_remove,
	.id_table = ocp2131_i2c_id,
	//.detect = ocp2131_i2c_detect
};


/* called when loaded into kernel */
static int __init ocp2131_driver_init(void) {
	printk("ocp2131_driver_init\n");
	
	//i2c_register_board_info(I2C_I2C_LCM_POWER_CHANNEL, &ocp2131_i2c_boardinfo, 1);	

	if(i2c_add_driver(&ocp2131_i2c_driver)!=0) {
		printk("ocp2131 unable to add i2c driver.\n");
		return -1;
	}
	return 0;
}

/* should never be called */
static void __exit ocp2131_driver_exit(void) {
	printk("ocp2131_driver_exit\n");
	//input_unregister_device(tpd->dev);
	i2c_del_driver(&ocp2131_i2c_driver);
}


module_init(ocp2131_driver_init);
module_exit(ocp2131_driver_exit);

MODULE_AUTHOR("WingTech Driver Team");
MODULE_DESCRIPTION("Panel Power Supply Driver");
MODULE_LICENSE("GPL v2");



