#include <linux/init.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/sched.h>
#include <linux/workqueue.h>

//#include <asm/hardware.h>
//#include <asm/arch/gpio.h>
//add by inter.park
#include <mach/hardware.h>
#include <linux/gpio.h>

#include <linux/irq.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include "bma020_i2c.h"
#include "bma020.h"

#define I2C_M_WR				0x00
#define I2C_DF_NOTIFY			0x01

static int __devinit bma020_probe(struct i2c_client *, const struct i2c_device_id *);
static int __devexit bma020_remove(struct i2c_client *);

static struct i2c_client *g_client;	

char i2c_acc_bma020_read(u8 reg, u8 *val, unsigned int len )
{
	int 	 err;
	struct 	 i2c_msg msg[1];
		
	unsigned char data[1];
	if( (g_client == NULL) || (!g_client->adapter) )
	{
		return -ENODEV;
	}
	
	msg->addr 	= g_client->addr;
	msg->flags 	= I2C_M_WR;
	msg->len 	= 1;
	msg->buf 	= data;
	*data       = reg;

	err = i2c_transfer(g_client->adapter, msg, 1);

	if (err >= 0) 
	{
		msg->flags = I2C_M_RD;
		msg->len   = len;
		msg->buf   = val;
		err = i2c_transfer(g_client->adapter, msg, 1);
	}

	if (err >= 0) 
	{
		return 0;
	}
	printk("%s %d i2c transfer error\n", __func__, __LINE__);/* add by inter.park */

	return err;

}
char i2c_acc_bma020_write( u8 reg, u8 *val )
{
	int err;
	struct i2c_msg msg[1];
	unsigned char data[2];

	if( (g_client == NULL) || (!g_client->adapter) ){
		return -ENODEV;
	}
	
	data[0] = reg;
	data[1] = *val;

	msg->addr = g_client->addr;
	msg->flags = I2C_M_WR;
	msg->len = 2;
	msg->buf = data;
	
	err = i2c_transfer(g_client->adapter, msg, 1);

	if (err >= 0) return 0;

	printk("%s %d i2c transfer error\n", __func__, __LINE__);/* add by inter.park */
	return err;
}

MODULE_DEVICE_TABLE(i2c, bma020_ids);

static int __devexit bma020_remove(struct i2c_client *client)
{	
	g_client = NULL;
	return 0;
}

static const struct i2c_device_id bma020_ids[] = {	
	{ "kr3dm_i2c_driver", 0 },
	{ }
};

struct i2c_driver acc_bma020_i2c_driver =
{
	.driver	= {
		.name	= "kr3dm_i2c_driver",
	},
	.class		= I2C_CLASS_HWMON,
	.probe		= bma020_probe,
	.remove		= __devexit_p(bma020_remove),
	.id_table	= bma020_ids,

};

static int __devinit bma020_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	int err = 0;

        printk("BMA020 %s called \n",__func__);

        if ( !i2c_check_functionality(client->adapter,I2C_FUNC_SMBUS_BYTE_DATA) ) {
                printk(KERN_INFO "byte op is not permited.\n");
                goto ERROR0;
        }

	client->addr = BMA020_I2C_ADDR >> 1;
        client->driver = &acc_bma020_i2c_driver;
        client->flags = I2C_DF_NOTIFY | I2C_M_IGNORE_NAK;

	g_client = client;
        return 0;

ERROR0:
        printk(KERN_ERR "[KR3DM][%s] probe failed!\n", __func__);
        return err;
}
int i2c_acc_bma020_init(void)
{
	int ret;
        printk("%s called \n",__func__);
	if ( (ret = i2c_add_driver(&acc_bma020_i2c_driver)) ) 
	{
		printk("Driver registration failed, module not inserted.\n");
		return ret;
	}

	return 0;
}

void i2c_acc_bma020_exit(void)
{
	printk("[BMA020] i2c_exit\n");
	i2c_del_driver(&acc_bma020_i2c_driver); 
}

