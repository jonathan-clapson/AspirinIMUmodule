/*
* Copyright (C) 2012 Invensense, Inc.
*
* This software is licensed under the terms of the GNU General Public
* License version 2, as published by the Free Software Foundation, and
* may be copied, distributed, and modified under those terms.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*/

#include <linux/init.h>
#include <linux/module.h>
//#include <linux/mutex.h>
//#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/gpio.h>
//#include <asm/uaccess.h>
#include <linux/semaphore.h>
#include <linux/types.h>
#include <linux/delay.h>

#include "inv_mpu_iio.h"

/*#include <linux/err.h>
#include <linux/sysfs.h>
#include <linux/jiffies.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/kfifo.h>
#include <linux/spinlock.h>*/

#define SPI_BUS 0
#define SPI_BUS_CS0 0
#define SPI_BUS_CS1 1
#define SPI_BUS_SPEED 1000000 //1MHz

#define SIG_COND_RESET 		(1<<0)
#define I2C_MST_RESET		(1<<1)
#define FIFO_RESET		(1<<2)
#define I2C_IF_DIS		(1<<4)
#define I2C_MST_EN 		(1<<5)
#define FIFO_EN			(1<<6)

#define REGADDR(addr)		((u16)addr<<8)
#define REGREAD 		(1<<15)
#define REGWRITE 		(0<<15)

#define SPIWRITEREG(addr, data) (cpu_to_be16(REGWRITE|(REGADDR(addr))|(data)))
#define SPIREADREG(addr) (cpu_to_be16(REGREAD|REGADDR(addr)))

#define SPIBUFSIZE 2

/* Module Params */
short gpio_power = -1;
module_param(gpio_power, short, S_IRUSR);
MODULE_PARM_DESC(gpio_power, "The gpio pin number to use as a power toggle");

const char this_driver_name[] = "MPU6000";

/*struct mpu6050_control {
	struct spi_message msg;
	struct spi_transfer transfer;
	uint32_t busy;
	uint32_t spi_callbacks;
	uint32_t busy_counter;
	uint16_t *tx_buff;
};*/

struct mpu6000_dev {
	struct semaphore spi_sem;
	dev_t devt;
	struct spi_device *spi_device;
};
static struct mpu6000_dev mpu6000_dev;


static int mpu6000_probe(struct spi_device *spi_device)
{
	if (down_interruptible(&mpu6000_dev.spi_sem))
		return -EBUSY;

	mpu6000_dev.spi_device = spi_device;

	up(&mpu6000_dev.spi_sem);

	return 0;
}

static int mpu6000_remove(struct spi_device *spi_device)
{
	if (down_interruptible(&mpu6000_dev.spi_sem))
		return -EBUSY;

	mpu6000_dev.spi_device = NULL;

	up(&mpu6000_dev.spi_sem);

	return 0;
}

static int __init add_mpu6000_device_to_bus(void)
{
	struct spi_master *spi_master;
	struct spi_device *spi_device;
	struct device *pdev;
	char buff[64];
	int status = 0;

	spi_master = spi_busnum_to_master(SPI_BUS);
	if (!spi_master) {
		printk(KERN_ALERT "spi_busnum_to_master(%d) returned NULL\n",
			SPI_BUS);
		printk(KERN_ALERT "Missing modprobe omap2_mcspi?\n");
		return -1;
	}

	spi_device = spi_alloc_device(spi_master);
	if (!spi_device) {
		put_device(&spi_master->dev);
		printk(KERN_ALERT "spi_alloc_device() failed\n");
		return -1;
	}

	/* specify a chip select line */
	spi_device->chip_select = SPI_BUS_CS0;

	/* Check whether this SPI bus.cs is already claimed */
	snprintf(buff, sizeof(buff), "%s.%u",
			dev_name(&spi_device->master->dev),
			spi_device->chip_select);

	pdev = bus_find_device_by_name(spi_device->dev.bus, NULL, buff);
 	if (pdev) {
		/* We are not going to use this spi_device, so free it */
		spi_dev_put(spi_device);

 		printk("bus is claimed!\n");

		/*
		 * There is already a device configured for this bus.cs combination.
		 * It's okay if it's us. This happens if we previously loaded then
		 * unloaded our driver.
		 * If it is not us, we complain and fail.
		 */
		if (pdev->driver && pdev->driver->name &&
				strcmp(this_driver_name, pdev->driver->name)) {
			printk(KERN_ALERT
				"Driver [%s] already registered for %s\n",
				pdev->driver->name, buff);
			status = -1;
		}
	} else {
		printk("bus is free!\n");

		spi_device->max_speed_hz = SPI_BUS_SPEED;
		spi_device->mode = SPI_MODE_3;
		spi_device->bits_per_word = 8;
		spi_device->irq = -1;
		spi_device->controller_state = NULL;
		spi_device->controller_data = NULL;
		strlcpy(spi_device->modalias, this_driver_name, SPI_NAME_SIZE);
		status = spi_add_device(spi_device);

		if (status < 0) {
			spi_dev_put(spi_device);
			printk(KERN_ALERT "spi_add_device() failed: %d\n",
				status);
		}
	}

	put_device(&spi_master->dev);
	printk("loaded!\n");

	return status;
}

static struct spi_driver mpu6000_driver = {
	.driver = {
		.name 		= this_driver_name,
		.owner		= THIS_MODULE,
	},

	.probe = mpu6000_probe,
	.remove = mpu6000_remove,
	//.suspend = mpu6000_suspend,
	//.resume = mpu6000_resume,
};

static int __init reset_mpu6000_power(void)
{
	if (!gpio_is_valid(gpio_power)) {
		printk(KERN_ALERT "The requested GPIO is not available \n");
		return -EINVAL;
	}
	
	if (gpio_request_one(gpio_power, GPIOF_OUT_INIT_LOW, "mpu6000_power")) {
		printk(KERN_ALERT "Unable to request gpio %d", gpio_power);
		return -EINVAL;
	}
	
	//FIXME: need to set timeout
	msleep (500);
	
	gpio_set_value(gpio_power, 1);
	
	//FIXME: need to wait for initialisation time
	//msleep (0);
	
	//FIXME: is there an error code for success?
	return 0;
}

static void __init read_device(void)
{
	
	struct spi_transfer init;
	memset(&init, 0, sizeof(init));
	
	__be16 initb = SPIWRITEREG(INV_MPU60x0_REG_RAW_ACCEL ,0x80);

	init.tx_buf = &initb;
	init.rx_buf = NULL;
	init.len = sizeof(__be16);
	init.cs_change = 0;
	
	struct spi_message *spi_msg = spi_message_alloc(1, GFP_KERNEL);
	
	spi_msg->spi = mpu6000_dev.spi_device;
	spi_msg->actual_length = 1*sizeof(__be16);
	spi_msg->context = NULL;

	spi_message_add_tail(&init, spi_msg);
	
	int error = spi_sync(mpu6000_dev.spi_device, spi_msg);
	if (error < 0)
	{
		printk("failed to send message\n");
		return;
	}
	spi_message_free(spi_msg);
		
	int readint;
	int i;
	for (i=0; i<7; i++)
	{
		struct spi_transfer read;
		memset(&read, 0, sizeof(read));

		read.tx_buf = NULL;
		read.rx_buf = &readint;
		read.len = sizeof(__be16);
		read.cs_change = 0;
		
		struct spi_message *spi_msg = spi_message_alloc(1, GFP_KERNEL);
		
		spi_msg->spi = mpu6000_dev.spi_device;
		spi_msg->actual_length = 1*sizeof(__be16);
		spi_msg->context = NULL;
	
		spi_message_add_tail(&read, spi_msg);
		
		int error = spi_sync(mpu6000_dev.spi_device, spi_msg);
		if (error < 0)
		{
			printk("failed to send message\n");
			return;
		}
		spi_message_free(spi_msg);
			
	}
}

static void __init mpu6000_init_device(void)
{
	struct spi_transfer sampleRateTrans;
	struct spi_transfer sampleRateTrans2;
	struct spi_transfer configTrans;
	struct spi_transfer gyroConfigTrans;
	struct spi_transfer accelConfigTrans;
	struct spi_transfer unknown;
	
	memset(&sampleRateTrans, 0, sizeof(sampleRateTrans));
	memset(&sampleRateTrans2, 0, sizeof(sampleRateTrans2));
	memset(&configTrans, 0, sizeof(configTrans));
	memset(&gyroConfigTrans, 0, sizeof(gyroConfigTrans));
	memset(&accelConfigTrans, 0, sizeof(accelConfigTrans));
	memset(&unknown, 0, sizeof(unknown));
	
	__be16 srTransb = SPIWRITEREG(INV_MPU60x0_REG_SAMPLE_RATE_DIV, 0x04);
	__be16 srTrans2b = SPIWRITEREG(INV_MPU60x0_REG_SAMPLE_RATE_DIV, 0x09);
	__be16 configTransb = SPIWRITEREG(INV_MPU60x0_REG_CONFIG , 0x09);
	__be16 gyroConfigTransb = SPIWRITEREG(INV_MPU60x0_REG_GYRO_CONFIG , INV_MPU60x0_FSR_1000DPS);
	__be16 accelConfigTransb = SPIWRITEREG(INV_MPU60x0_REG_ACCEL_CONFIG , INV_MPU60x0_FS_08G);
	__be16 unknownb = SPIWRITEREG(0x38, 0x40);
	
	sampleRateTrans.tx_buf = &srTransb;
	sampleRateTrans.rx_buf = NULL;
	sampleRateTrans.len = sizeof(__be16);
	sampleRateTrans.cs_change = 1;
	sampleRateTrans.delay_usecs = 1000;
	
	sampleRateTrans2.tx_buf = &srTrans2b;
	sampleRateTrans2.rx_buf = NULL;
	sampleRateTrans2.len = sizeof(__be16);
	sampleRateTrans2.cs_change = 1;
	sampleRateTrans2.delay_usecs = 1000;
	
	configTrans.tx_buf = &configTransb;
	configTrans.rx_buf = NULL;
	configTrans.len = sizeof(__be16);
	configTrans.cs_change = 1;
	configTrans.delay_usecs = 1000;
	
	gyroConfigTrans.tx_buf = &gyroConfigTransb;
	gyroConfigTrans.rx_buf = NULL;
	gyroConfigTrans.len = sizeof(__be16);
	gyroConfigTrans.cs_change = 1;
	gyroConfigTrans.delay_usecs = 1000;
	
	accelConfigTrans.tx_buf = &accelConfigTransb;
	accelConfigTrans.rx_buf = NULL;
	accelConfigTrans.len = sizeof(__be16);
	accelConfigTrans.cs_change = 1;
	accelConfigTrans.delay_usecs = 1000;
	
	unknown.tx_buf = &unknownb;
	unknown.rx_buf = NULL;
	unknown.len = sizeof(__be16);
	unknown.cs_change = 1;
	unknown.delay_usecs = 1000;
	
	struct spi_message *spi_msg = spi_message_alloc(6, GFP_KERNEL);

	spi_msg->spi = mpu6000_dev.spi_device;
	spi_msg->context = NULL;
	
	spi_message_add_tail(&sampleRateTrans, spi_msg);
	spi_message_add_tail(&sampleRateTrans2, spi_msg);
	spi_message_add_tail(&configTrans, spi_msg);
	spi_message_add_tail(&gyroConfigTrans, spi_msg);
	spi_message_add_tail(&accelConfigTrans, spi_msg);
	spi_message_add_tail(&unknown, spi_msg);
	
	int error = spi_sync(mpu6000_dev.spi_device, spi_msg);
	if (error < 0)
	{
		printk("failed to send message\n");
		return;
	}
	spi_message_free(spi_msg);
}

static int __init mpu6000_init_spi(void)
{
	int error;

	/*mpu6050_ctl.tx_buff = kmalloc(SPI_BUFF_SIZE, GFP_KERNEL);
	if (!spike_ctl.tx_buff) {
		return -ENOMEM;
	}*/

	error = spi_register_driver(&mpu6000_driver);
	if (error < 0) {
		printk(KERN_ALERT "spi_register_driver() failed %d\n", error);
		return error;
	}

	error = add_mpu6000_device_to_bus();
	if (error < 0) {
		printk(KERN_ALERT "add_mpu6000_to_bus() failed\n");
		goto failed;
	}
	printk("add device to bus succeeded!\n");
	
	/* Need to reset power to allow enabling of the SPI Interface */
	/*error = reset_mpu6000_power();
	if (error < 0) {
		goto failed;
	}*/
	
	__be16 bufPwrReset = SPIWRITEREG(INV_MPU60x0_REG_PWR_MGMT_1, 0x01 << 7); //reset
	__be16 bufUserCtrl = SPIWRITEREG(INV_MPU60x0_REG_USER_CTRL, 
		(1 << 2) |	// Trigger a FIFO_RESET
		(1 << 1) |	// Trigger a I2C_MST_RESET
		(1 << 0) );	// Trigger a SIG_COND_RESET
	// MPU60X0_REG_PWR_MGMT_1
	__be16 bufPwrSet = SPIWRITEREG(INV_MPU60x0_REG_PWR_MGMT_1,
		0x01);		// -switch to gyroX clock
	
	__be16 bufDisI2C = SPIWRITEREG(INV_MPU60x0_REG_USER_CTRL,
           (1 << 5) |		// I2C_MST_EN: Enable Aux I2C Master Mode
           (1 << 4) |		// I2C_IF_DIS: Disable I2C on primary interface
           (0 << 1) );		// Trigger a I2C_MST_RESET	
	
	struct spi_transfer spi_writePwrReset;
	struct spi_transfer spi_writeUserCtrl;
	struct spi_transfer spi_writePwrSet;
	struct spi_transfer spi_writeBufDisI2C;
	
	memset(&spi_writePwrReset, 0, sizeof(struct spi_transfer));
	//memset(&spi_writeUserCtrl, 0, sizeof(struct spi_transfer));
	//memset(&spi_writePwrSet, 0, sizeof(struct spi_transfer));
	//memset(&spi_writeBufDisI2C, 0, sizeof(struct spi_transfer));
	
	spi_writePwrReset.tx_buf = &bufPwrReset;
	spi_writePwrReset.rx_buf = NULL;
	spi_writePwrReset.len = sizeof(bufPwrReset);
	spi_writePwrReset.cs_change = 1;
	spi_writePwrReset.delay_usecs = 40000;
	
	/*spi_writeUserCtrl.tx_buf = &bufUserCtrl;
	spi_writeUserCtrl.rx_buf = NULL;
	spi_writeUserCtrl.len = sizeof(bufUserCtrl);
	spi_writeUserCtrl.cs_change = 1;
	
	spi_writePwrSet.tx_buf = &bufPwrSet;
	spi_writePwrSet.rx_buf = NULL;
	spi_writePwrSet.len = sizeof(bufPwrSet);
	spi_writePwrSet.cs_change = 1;
	
	spi_writeBufDisI2C.tx_buf = &bufDisI2C;
	spi_writeBufDisI2C.rx_buf = NULL;
	spi_writeBufDisI2C.len = sizeof(bufDisI2C);
	spi_writeBufDisI2C.cs_change = 1;*/

	__be16 bufRead = SPIREADREG(INV_MPU60x0_WHO_AM_I);
	//__be16 bufRes = 0;
	int bufRes = 0;
	
	struct spi_transfer spi_request;
	memset(&spi_request, 0, sizeof(struct spi_transfer));

	spi_request.tx_buf = &bufRead;
	spi_request.rx_buf = &bufRes;
	spi_request.len = sizeof(u16);
	spi_request.cs_change = 1;

	struct spi_message *spi_msg = spi_message_alloc(2, GFP_KERNEL);

	spi_msg->spi = mpu6000_dev.spi_device;
	spi_msg->context = NULL;
	
	spi_message_add_tail(&spi_writePwrReset, spi_msg);
	spi_message_add_tail(&spi_request, spi_msg);
	/*spi_message_add_tail(&spi_writeUserCtrl, spi_msg);
	spi_message_add_tail(&spi_writePwrSet, spi_msg);
	spi_message_add_tail(&spi_writeBufDisI2C, spi_msg);*/

	error = spi_sync(mpu6000_dev.spi_device, spi_msg);
	if (error < 0)
	{
		printk("failed to send message\n");
		goto failed;
	}
	printk("device id is: %X\n", bufRes);

	spi_message_free(spi_msg);
	
	//mpu6000_init_device();
	//read_device();

	return 0;

failed:
	spi_unregister_driver(&mpu6000_driver);
	gpio_free(gpio_power);
	return error;
}


static int __init mpu6000_init(void)
{
	memset(&mpu6000_dev, 0, sizeof(mpu6000_dev));

	sema_init(&mpu6000_dev.spi_sem, 1);

	if (mpu6000_init_spi() < 0)
		return -1;

	printk("%s loaded\n", this_driver_name);

	return 0;
}
module_init(mpu6000_init);

static void __exit mpu6000_exit(void)
{
	//FIXME: need to unregister device?
	spi_unregister_device(mpu6000_dev.spi_device);
	
	spi_unregister_driver(&mpu6000_driver);
	gpio_free(gpio_power);
}
module_exit(mpu6000_exit);

MODULE_AUTHOR("Invensense Corporation");
MODULE_DESCRIPTION("Invensense device MPU6000 driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("0.1");
