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

#include "bma020_i2c.h"

//static int i2c_acc_bma020_attach_adapter(struct i2c_adapter *adapter);
//static int i2c_acc_bma020_probe_client(struct i2c_adapter *, int,  int);
static int __devinit i2c_acc_bma020_probe(struct i2c_client *client, const struct i2c_device_id *id);
//static int i2c_acc_bma020_detach_client(struct i2c_client *client);
static int __devexit i2c_acc_bma020_remove(struct i2c_client *client);
static int i2c_acc_bma020_detect(struct i2c_client *client, int kind, struct i2c_board_info *info);


#define	ACC_SENSOR_ADDRESS		0x38

#define I2C_M_WR				0x00
#define I2C_DF_NOTIFY			0x01

struct i2c_device_id i2c_acc_bma020_idtable[] = {
    { "kr3dm", 0 },
    { }
};


#if 0
static struct i2c_client *g_client;
static unsigned short ignore[] = { I2C_CLIENT_END };

static unsigned short normal_addr[] = {
	ACC_SENSOR_ADDRESS,
	I2C_CLIENT_END
};
#endif

static struct i2c_client *g_client;

static unsigned short ignore[] = { I2C_CLIENT_END };
static unsigned short normal_addr[] = { I2C_CLIENT_END };
static unsigned short probe_addr[] = { 5, ACC_SENSOR_ADDRESS, I2C_CLIENT_END };

static struct i2c_client_address_data addr_data = {
	.normal_i2c		= normal_addr,
	.probe			= probe_addr,
	.ignore			= ignore,
};

struct i2c_driver acc_bma020_i2c_driver =
{
	.driver = {
		.name = "kr3dm",
	},
	.class  =       I2C_CLASS_HWMON,
	.probe  =       i2c_acc_bma020_probe,
	.remove =       __devexit_p(i2c_acc_bma020_remove),
	.detect =       i2c_acc_bma020_detect,
	.id_table =     i2c_acc_bma020_idtable,
	.address_data = &addr_data
//	.attach_adapter	= &i2c_acc_bma020_attach_adapter,
//	.detach_client	= &i2c_acc_bma020_detach_client,
};

MODULE_DEVICE_TABLE(i2c, i2c_acc_bma020_idtable);

int i2c_acc_bma020_init(void)
{
	int ret;

	printk("[BMA020] i2c_init\n");
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

//static int i2c_acc_bma020_attach_adapter(struct i2c_adapter *adapter)
//{
//	return i2c_probe(adapter, &addr_data, &i2c_acc_bma020_probe_client);
//}

static int i2c_acc_bma020_detect(struct i2c_client *client, int kind, struct i2c_board_info *info)
{
        printk("[BMA020] i2c_detect\n");
        strlcpy(info->type, "kr3dm", I2C_NAME_SIZE);
        return 0;
}

static int __devinit i2c_acc_bma020_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int err = 0;
	printk("[BMA020] i2c_probe\n");

	if ( !i2c_check_functionality(client->adapter,I2C_FUNC_SMBUS_BYTE_DATA) ) {
		printk(KERN_INFO "byte op is not permited.\n");
		return -1;
	}

	client->driver = &acc_bma020_i2c_driver;


	g_client = client;


	return 0;

//	ERROR1:
//		printk("i2c_acc_bma020_probe_client() ERROR1\n");/* add by inter.park */
//		kfree(new_client);
//	ERROR0:
//		printk("i2c_acc_bma020_probe_client() ERROR0\n");/* add by inter.park */
//    	return err;
}

static int __devexit i2c_acc_bma020_remove(struct i2c_client *client)
{
	int err;

	printk("[BMA020] i2c_detach_client\n");

	g_client = NULL;
	return 0;
}

