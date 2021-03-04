#include <linux/init.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/i2c.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/ide.h>
#include <linux/errno.h>
#include <linux/gpio.h>
#include <asm/mach/map.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_gpio.h>
#include <asm/io.h>
#include <linux/device.h>

#include <linux/platform_device.h>


#define HMC5883L_CNT	1
#define HMC5883L_NAME	"hmc5883l"

#define HMC5883L_X_MSB	0x03
#define HMC5883L_X_LSB	0x04
#define HMC5883L_Z_MSB	0x05
#define HMC5883L_Z_LSB	0x06
#define HMC5883L_Y_MSB	0x07
#define HMC5883L_Y_LSB	0x08

struct hmc5883l_dev {
	dev_t devid;				/* 设备号 	 */
	struct cdev cdev;			/* cdev 	*/
	struct class *class;		/* 类 		*/
	struct device *device;		/* 设备 	 */
	struct device_node	*nd; 	/* 设备节点 */
	int major;					/* 主设备号 */
	void *private_data;			/* 私有数据 		*/

	short mag_x_adc;		/* 陀螺仪X轴原始值 	 */
	short mag_y_adc;		/* 陀螺仪Y轴原始值		*/
	short mag_z_adc;		/* 陀螺仪Z轴原始值 		*/

};

static struct hmc5883l_dev hmc5883ldev;



static int hmc5883l_read_regs(struct hmc5883l_dev *dev, u8 reg, void *val, int len)
{
	int ret;
	struct i2c_msg msg[2];
	struct i2c_client *client = (struct i2c_client *)dev->private_data;

	/* msg[0]为发送要读取的首地址 */
	msg[0].addr = client->addr;			/* hmc5883l地址 */
	msg[0].flags = 0;					/* 标记为发送数据 */
	msg[0].buf = &reg;					/* 读取的首地址 */
	msg[0].len = 1;						/* reg长度*/

	/* msg[1]读取数据 */
	msg[1].addr = client->addr;			/* hmc5883l地址 */
	msg[1].flags = I2C_M_RD;			/* 标记为读取数据*/
	msg[1].buf = val;					/* 读取数据缓冲区 */
	msg[1].len = len;					/* 要读取的数据长度*/

	ret = i2c_transfer(client->adapter, msg, 2);
	if(ret == 2) {
		ret = 0;
	} else {
		printk("i2c rd failed=%d reg=%06x len=%d\n",ret, reg, len);
		ret = -EREMOTEIO;
	}
	return ret;
}

static s32 hmc5883l_write_regs(struct hmc5883l_dev *dev, u8 reg, u8 *buf, u8 len)
{
	u8 b[256];
	struct i2c_msg msg;
	struct i2c_client *client = (struct i2c_client *)dev->private_data;
	
	b[0] = reg;					/* 寄存器首地址 */
	memcpy(&b[1],buf,len);		/* 将要写入的数据拷贝到数组b里面 */
		
	msg.addr = client->addr;	/* hmc5883l地址 */
	msg.flags = 0;				/* 标记为写数据 */

	msg.buf = b;				/* 要写入的数据缓冲区 */
	msg.len = len + 1;			/* 要写入的数据长度 */

	return i2c_transfer(client->adapter, &msg, 1);
}


static unsigned char hmc5883l_read_reg(struct hmc5883l_dev *dev, u8 reg)
{
	unsigned char data = 0;

	hmc5883l_read_regs(dev, reg, &data, sizeof(data));
	return data;

}

static void hmc5883l_write_reg(struct hmc5883l_dev *dev, u8 reg, u8 data)
{
	u8 buf = 0;
	buf = data;
	hmc5883l_write_regs(dev, reg, &buf, 1);
}

void hmc5883l_readdata(struct hmc5883l_dev *dev)
{
	
    short buf[6];
	/*
	int i = 0;
	*/
	
	buf[0] = hmc5883l_read_reg(dev,HMC5883L_X_LSB);
	buf[1] = hmc5883l_read_reg(dev,HMC5883L_X_MSB);
	buf[2] = hmc5883l_read_reg(dev,HMC5883L_Y_LSB);
	buf[3] = hmc5883l_read_reg(dev,HMC5883L_Y_MSB);
	buf[4] = hmc5883l_read_reg(dev,HMC5883L_Z_LSB);
	buf[5] = hmc5883l_read_reg(dev,HMC5883L_Z_MSB);
	

	printk("\n HMC5883L_X_LSB = %x \n",buf[0]);
	printk("\n HMC5883L_X_MSB = %x \n",buf[1]);
	printk("\n HMC5883L_Y_LSB = %x \n",buf[2]);
	printk("\n HMC5883L_Y_MSB = %x \n",buf[3]);
	printk("\n HMC5883L_Z_LSB = %x \n",buf[4]);
	printk("\n HMC5883L_Z_MSB = %x \n",buf[5]);
	
	dev->mag_x_adc =  ((short)buf[1] << 8 | (short)buf[0] );
	dev->mag_y_adc =  ((short)buf[3] << 8 | (short)buf[2] );
	dev->mag_z_adc =  ((short)buf[5] << 8 | (short)buf[4] );
	
	/*
	for(i = 0;i < 6;i++){
		buf[i] = hmc5883l_read_reg(dev,0x03 + i);
	}

	printk("\n HMC5883L_X_LSB = %x \n",buf[1]);
	printk("\n HMC5883L_X_MSB = %x \n",buf[0]);
	printk("\n HMC5883L_Y_LSB = %x \n",buf[5]);
	printk("\n HMC5883L_Y_MSB = %x \n",buf[4]);
	printk("\n HMC5883L_Z_LSB = %x \n",buf[3]);
	printk("\n HMC5883L_Z_MSB = %x \n",buf[2]);

	dev->mag_x_adc =  ((short)buf[0] << 8 | (short)buf[1] );
	dev->mag_y_adc =  ((short)buf[4] << 8 | (short)buf[5] );
	dev->mag_z_adc =  ((short)buf[2] << 8 | (short)buf[3] );
	*/
	printk("\n dev->mag_x_adc = %x \n",dev->mag_x_adc);
	printk("\n dev->mag_y_adc = %x \n",dev->mag_y_adc);
	printk("\n dev->mag_z_adc = %x \n",dev->mag_z_adc);


}

static int hmc5883l_open(struct inode *inode, struct file *filp)
{
	unsigned char data = 0;
	filp->private_data = &hmc5883ldev;
	

	data = hmc5883l_read_reg(&hmc5883ldev,0x0a);
	if( data!= 0x48){
		printk(KERN_DEBUG "\n Identification Register 0x0a check failure [%02x] \n",data);
	}

	data = hmc5883l_read_reg(&hmc5883ldev,0x0b);
	if( data!= 0x34){
		printk(KERN_DEBUG "\n Identification Register 0x0a check failure [%02x] \n",data);
	}

	data = hmc5883l_read_reg(&hmc5883ldev,0x0c);
	if( data!= 0x33){
		printk(KERN_DEBUG "\n Identification Register 0x0a check failure [%02x] \n",data);
	}

	/* 初始化AP3216C */
	hmc5883l_write_reg(&hmc5883ldev, 0x00, 0x70);	
	mdelay(50);	
	hmc5883l_write_reg(&hmc5883ldev, 0x01, 0xa0);	
	mdelay(50);											
	hmc5883l_write_reg(&hmc5883ldev, 0x02, 0x00);	
	mdelay(50);	
	return 0;
}

static ssize_t hmc5883l_read(struct file *filp, char __user *buf, size_t cnt, loff_t *off)
{
	int data[3];
	long err = 0;

	struct hmc5883l_dev *dev = (struct hmc5883l_dev *)filp->private_data;
	
	hmc5883l_readdata(dev);

	data[0] = dev->mag_x_adc;
	data[1] = dev->mag_y_adc;
	data[2] = dev->mag_z_adc;

	printk("\n user data[0] = %x \n",data[0]);
	printk("\n user data[1] = %x \n",data[1]);
	printk("\n user data[2] = %x \n",data[2]);
	err = copy_to_user(buf, data, sizeof(data));
	return 0;
}

static int hmc5883l_release(struct inode *inode, struct file *filp)
{
	return 0;
}

static const struct file_operations hmc5883l_ops = {
	.owner = THIS_MODULE,
	.open = hmc5883l_open,
	.read = hmc5883l_read,
	.release = hmc5883l_release,
};

static int hmc5883l_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	/* 1、构建设备号 */
	if (hmc5883ldev.major) {
		hmc5883ldev.devid = MKDEV(hmc5883ldev.major, 0);
		register_chrdev_region(hmc5883ldev.devid, HMC5883L_CNT, HMC5883L_NAME);
	} else {
		alloc_chrdev_region(&hmc5883ldev.devid, 0, HMC5883L_CNT, HMC5883L_NAME);
		hmc5883ldev.major = MAJOR(hmc5883ldev.devid);
	}

	/* 2、注册设备 */
	cdev_init(&hmc5883ldev.cdev, &hmc5883l_ops);
	cdev_add(&hmc5883ldev.cdev, hmc5883ldev.devid, HMC5883L_CNT);

	/* 3、创建类 */
	hmc5883ldev.class = class_create(THIS_MODULE, HMC5883L_NAME);
	if (IS_ERR(hmc5883ldev.class)) {
		return PTR_ERR(hmc5883ldev.class);
	}

	/* 4、创建设备 */
	hmc5883ldev.device = device_create(hmc5883ldev.class, NULL, hmc5883ldev.devid, NULL, HMC5883L_NAME);
	if (IS_ERR(hmc5883ldev.device)) {
		return PTR_ERR(hmc5883ldev.device);
	}

	hmc5883ldev.private_data = client;

	return 0;
}

static int hmc5883l_remove(struct i2c_client *client)
{
	/* 删除设备 */
	cdev_del(&hmc5883ldev.cdev);
	unregister_chrdev_region(hmc5883ldev.devid, HMC5883L_CNT);

	/* 注销掉类和设备 */
	device_destroy(hmc5883ldev.class, hmc5883ldev.devid);
	class_destroy(hmc5883ldev.class);
	return 0;
}


/*定义ID 匹配表*/
static const struct i2c_device_id hmc5883l_id[] = {
    {"morse,i2c_hmc5883l", 0},
    {}
};


/*定义设备树匹配表*/
static const struct of_device_id hmc5883l_of_match_table[] = {
    {.compatible = "morse,i2c_hmc5883l"},
    {/* sentinel */}
};

/*定义i2c总线设备结构体*/
struct i2c_driver hmc5883l_driver = {
    .probe = hmc5883l_probe,
    .remove = hmc5883l_remove,
    .id_table = hmc5883l_id,
    .driver = {
            .name = "morse,i2c_hmc5883l",
            .owner = THIS_MODULE,
            .of_match_table = hmc5883l_of_match_table,
    },
};


/*
 * 驱动初始化函数
 */
static int __init hmc5883l_driver_init(void)
{
	int ret = 0;

	ret = i2c_add_driver(&hmc5883l_driver);
    return 0;
}

/*
 * 驱动注销函数
 */
static void __exit hmc5883l_driver_exit(void)
{
	i2c_del_driver(&hmc5883l_driver);
}


module_init(hmc5883l_driver_init);
module_exit(hmc5883l_driver_exit);


MODULE_AUTHOR("morse");
MODULE_DESCRIPTION("HMC5883L i2c driver");
MODULE_LICENSE("GPL");