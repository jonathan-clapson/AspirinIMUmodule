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

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/sysfs.h>
#include <linux/jiffies.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/kfifo.h>
#include <linux/spinlock.h>
#include <linux/semaphore.h>
#include "mpu60x0/inv_mpu_iio.h"


#define REGADDR(addr)	((u16)addr<<8)
#define REGREAD 		(0x80)
#define SPIWRITEREG(addr, data) (cpu_to_be16((REGADDR(addr))|(data)))
#define SPIREADREG(addr) (cpu_to_be16(REGREAD|REGADDR(addr)))

#define SPI_BUS 0
#define SPI_BUS_CS0 0
#define SPI_BUS_CS1 1
#define SPI_BUS_SPEED 1000000 //1MHz

const char this_driver_name[] = "MPU60x0";

/*
 * this is the gyro scale translated from dynamic range plus/minus
 * {250, 500, 1000, 2000} to rad/s
 */
static const int gyro_scale_60x0[] = {133090, 266181, 532362, 1064724};

/*
 * this is the accel scale translated from dynamic range plus/minus
 * {2, 4, 8, 16} to m/s^2
 */
static const int accel_scale[] = {598, 1196, 2392, 4785};

static const struct inv_mpu60x0_reg_map reg_set_60x0 = {
	.sample_rate_div	= INV_MPU60x0_REG_SAMPLE_RATE_DIV,
	.lpf                    = INV_MPU60x0_REG_CONFIG,
	.user_ctrl              = INV_MPU60x0_REG_USER_CTRL,
	.fifo_en                = INV_MPU60x0_REG_FIFO_EN,
	.gyro_config            = INV_MPU60x0_REG_GYRO_CONFIG,
	.accl_config            = INV_MPU60x0_REG_ACCEL_CONFIG,
	.fifo_count_h           = INV_MPU60x0_REG_FIFO_COUNT_H,
	.fifo_r_w               = INV_MPU60x0_REG_FIFO_R_W,
	.raw_gyro               = INV_MPU60x0_REG_RAW_GYRO,
	.raw_accl               = INV_MPU60x0_REG_RAW_ACCEL,
	.temperature            = INV_MPU60x0_REG_TEMPERATURE,
	.int_enable             = INV_MPU60x0_REG_INT_ENABLE,
	.pwr_mgmt_1             = INV_MPU60x0_REG_PWR_MGMT_1,
	.pwr_mgmt_2             = INV_MPU60x0_REG_PWR_MGMT_2,
};

static const struct inv_mpu60x0_chip_config chip_config_60x0 = {
	.fsr = INV_MPU60x0_FSR_2000DPS,
	.lpf = INV_MPU60x0_FILTER_20HZ,
	.fifo_rate = INV_MPU60x0_INIT_FIFO_RATE,
	.gyro_fifo_enable = false,
	.accl_fifo_enable = false,
	.accl_fs = INV_MPU60x0_FS_02G,
};

static const struct inv_mpu60x0_hw hw_info[INV_NUM_PARTS] = {
	{
		.num_reg = 117,
		.name = "MPU60x0",
		.reg = &reg_set_60x0,
		.config = &chip_config_60x0,
	},
};



int inv_mpu60x0_write_reg(struct inv_mpu60x0_state *st, int reg, u8 d)
{	
	__be16 buf = SPIWRITEREG(reg, d);
	struct spi_transfer spi_transaction = { 
		.tx_buf = &buf,
		.len = sizeof(__be16),
		.cs_change = 1 
	};
		
	return spi_sync_transfer (st->spi_dev, &spi_transaction, 1);
}
int inv_mpu60x0_read_reg(struct inv_mpu60x0_state *st, int reg, u8 *d)
{
	int error;
	
	__be16 buf = SPIREADREG(reg);
	__be16 readBuf;
	struct spi_transfer spi_transaction = { 
		.tx_buf = &buf,
		.rx_buf = &readBuf,
		.len = sizeof(__be16),
		.cs_change = 1 
	};
		
	error = spi_sync_transfer (st->spi_dev, &spi_transaction, 1);
	*d = (u8) be16_to_cpu(readBuf);
	
	return error;	
}

int inv_mpu60x0_switch_engine(struct inv_mpu60x0_state *st, bool en, u32 mask)
{
	u8 d, mgmt_1;
	int result;

	/* switch clock needs to be careful. Only when gyro is on, can
	   clock source be switched to gyro. Otherwise, it must be set to
	   internal clock */
	/* get the current value of power management 1 register and clear lower 3 (clock) bits */
	if (INV_MPU60x0_BIT_PWR_GYRO_STBY == mask) {
		result = inv_mpu60x0_read_reg(st, st->reg->pwr_mgmt_1, &mgmt_1);
		if (result < 0)
			return result;

		mgmt_1 &= ~INV_MPU60x0_BIT_CLK_MASK;
	}

	if ((INV_MPU60x0_BIT_PWR_GYRO_STBY == mask) && (!en)) {
		/* turning off gyro requires switch to internal clock first.
		   Then turn off gyro engine */
		mgmt_1 |= INV_CLK_INTERNAL;
		result = inv_mpu60x0_write_reg(st, st->reg->pwr_mgmt_1, mgmt_1);
		if (result)
			return result;
	}

	result = inv_mpu60x0_read_reg(st, st->reg->pwr_mgmt_2, &d);
	if (result < 0)
		return result;
	if (en)
		d &= ~mask;
	else
		d |= mask;
	result = inv_mpu60x0_write_reg(st, st->reg->pwr_mgmt_2, d);
	if (result)
		return result;

	if (en) {
		/* Wait for output stablize */
		msleep(INV_MPU60x0_TEMP_UP_TIME);
		if (INV_MPU60x0_BIT_PWR_GYRO_STBY == mask) {
			/* switch internal clock to PLL */
			mgmt_1 |= INV_CLK_PLL;
			result = inv_mpu60x0_write_reg(st,
					st->reg->pwr_mgmt_1, mgmt_1);
			if (result)
				return result;
		}
	}

	return 0;
}

int inv_mpu60x0_set_power_itg(struct inv_mpu60x0_state *st, bool power_on)
{
	int result;

	if (power_on)
		result = inv_mpu60x0_write_reg(st, st->reg->pwr_mgmt_1, 0);
	else
		result = inv_mpu60x0_write_reg(st, st->reg->pwr_mgmt_1,
						INV_MPU60x0_BIT_SLEEP);
	if (result)
		return result;

	if (power_on)
		msleep(INV_MPU60x0_REG_UP_TIME);

	return 0;
}

/**
 *  inv_mpu60x0_init_config() - Initialize hardware, disable FIFO.
 *
 *  Initial configuration:
 *  FSR: ± 2000DPS
 *  DLPF: 20Hz
 *  FIFO rate: 50Hz
 *  Clock source: Gyro PLL
 */
static int inv_mpu60x0_init_config(struct iio_dev *indio_dev)
{
	int result;
	u8 d;
	struct inv_mpu60x0_state *st = iio_priv(indio_dev);

	result = inv_mpu60x0_set_power_itg(st, true);
	if (result)
		return result;
	d = (INV_MPU60x0_FSR_2000DPS << INV_MPU60x0_GYRO_CONFIG_FSR_SHIFT);
	result = inv_mpu60x0_write_reg(st, st->reg->gyro_config, d);
	if (result)
		return result;

	d = INV_MPU60x0_FILTER_20HZ;
	result = inv_mpu60x0_write_reg(st, st->reg->lpf, d);
	if (result)
		return result;

	d = INV_MPU60x0_ONE_K_HZ / INV_MPU60x0_INIT_FIFO_RATE - 1;
	result = inv_mpu60x0_write_reg(st, st->reg->sample_rate_div, d);
	if (result)
		return result;

	d = (INV_MPU60x0_FS_02G << INV_MPU60x0_ACCL_CONFIG_FSR_SHIFT);
	result = inv_mpu60x0_write_reg(st, st->reg->accl_config, d);
	if (result)
		return result;

	memcpy(&st->chip_config, hw_info[st->chip_type].config,
		sizeof(struct inv_mpu60x0_chip_config));
	result = inv_mpu60x0_set_power_itg(st, false);

	return result;
}

static int inv_mpu60x0_sensor_show(struct inv_mpu60x0_state  *st, int reg,
				int axis, int *val)
{
	int ind, result;
	__be16 d;
	u8 ld, ud;

	ind = (axis - IIO_MOD_X) * 2;
	/* done to here */
	result = inv_mpu60x0_read_reg(st, reg+ind, &ld);
	if (result < 0 )
		return -EINVAL;
		
	result = inv_mpu60x0_read_reg(st, reg+ind+1, &ud);
	if (result < 0)
		return -EINVAL;
		
	d = (__be16)(ud<<8) | (__be16)(ld);
	*val = (short)be16_to_cpup(&d);

	return IIO_VAL_INT;
}

static int inv_mpu60x0_read_raw(struct iio_dev *indio_dev,
			      struct iio_chan_spec const *chan,
			      int *val,
			      int *val2,
			      long mask) {
	struct inv_mpu60x0_state  *st = iio_priv(indio_dev);

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
	{
		int ret, result;

		ret = IIO_VAL_INT;
		result = 0;
		mutex_lock(&indio_dev->mlock);
		if (!st->chip_config.enable) {
			result = inv_mpu60x0_set_power_itg(st, true);
			if (result)
				goto error_read_raw;
		}
		/* when enable is on, power is already on */
		switch (chan->type) {
		case IIO_ANGL_VEL:
			if (!st->chip_config.gyro_fifo_enable ||
					!st->chip_config.enable) {
				result = inv_mpu60x0_switch_engine(st, true,
						INV_MPU60x0_BIT_PWR_GYRO_STBY);
				if (result)
					goto error_read_raw;
			}
			ret =  inv_mpu60x0_sensor_show(st, st->reg->raw_gyro,
						chan->channel2, val);
			if (!st->chip_config.gyro_fifo_enable ||
					!st->chip_config.enable) {
				result = inv_mpu60x0_switch_engine(st, false,
						INV_MPU60x0_BIT_PWR_GYRO_STBY);
				if (result)
					goto error_read_raw;
			}
			break;
		case IIO_ACCEL:
			if (!st->chip_config.accl_fifo_enable ||
					!st->chip_config.enable) {
				result = inv_mpu60x0_switch_engine(st, true,
						INV_MPU60x0_BIT_PWR_ACCL_STBY);
				if (result)
					goto error_read_raw;
			}
			ret = inv_mpu60x0_sensor_show(st, st->reg->raw_accl,
						chan->channel2, val);
			if (!st->chip_config.accl_fifo_enable ||
					!st->chip_config.enable) {
				result = inv_mpu60x0_switch_engine(st, false,
						INV_MPU60x0_BIT_PWR_ACCL_STBY);
				if (result)
					goto error_read_raw;
			}
			break;
		case IIO_TEMP:
			/* wait for stablization */
			msleep(INV_MPU60x0_SENSOR_UP_TIME);
			inv_mpu60x0_sensor_show(st, st->reg->temperature,
							IIO_MOD_X, val);
			break;
		default:
			ret = -EINVAL;
			break;
		}
error_read_raw:
		if (!st->chip_config.enable)
			result |= inv_mpu60x0_set_power_itg(st, false);
		mutex_unlock(&indio_dev->mlock);
		if (result)
			return result;

		return ret;
	}
	case IIO_CHAN_INFO_SCALE:
		switch (chan->type) {
		case IIO_ANGL_VEL:
			*val  = 0;
			*val2 = gyro_scale_60x0[st->chip_config.fsr];

			return IIO_VAL_INT_PLUS_NANO;
		case IIO_ACCEL:
			*val = 0;
			*val2 = accel_scale[st->chip_config.accl_fs];

			return IIO_VAL_INT_PLUS_MICRO;
		case IIO_TEMP:
			*val = 0;
			*val2 = INV_MPU60x0_TEMP_SCALE;

			return IIO_VAL_INT_PLUS_MICRO;
		default:
			return -EINVAL;
		}
	case IIO_CHAN_INFO_OFFSET:
		switch (chan->type) {
		case IIO_TEMP:
			*val = INV_MPU60x0_TEMP_OFFSET;

			return IIO_VAL_INT;
		default:
			return -EINVAL;
		}
	default:
		return -EINVAL;
	}
}

static int inv_mpu60x0_write_fsr(struct inv_mpu60x0_state *st, int fsr)
{
	int result;
	u8 d;

	if (fsr < 0 || fsr > INV_MPU60x0_MAX_GYRO_FS_PARAM)
		return -EINVAL;
	if (fsr == st->chip_config.fsr)
		return 0;

	d = (fsr << INV_MPU60x0_GYRO_CONFIG_FSR_SHIFT);
	result = inv_mpu60x0_write_reg(st, st->reg->gyro_config, d);
	if (result)
		return result;
	st->chip_config.fsr = fsr;

	return 0;
}

static int inv_mpu60x0_write_accel_fs(struct inv_mpu60x0_state *st, int fs)
{
	int result;
	u8 d;

	if (fs < 0 || fs > INV_MPU60x0_MAX_ACCL_FS_PARAM)
		return -EINVAL;
	if (fs == st->chip_config.accl_fs)
		return 0;

	d = (fs << INV_MPU60x0_ACCL_CONFIG_FSR_SHIFT);
	result = inv_mpu60x0_write_reg(st, st->reg->accl_config, d);
	if (result)
		return result;
	st->chip_config.accl_fs = fs;

	return 0;
}

static int inv_mpu60x0_write_raw(struct iio_dev *indio_dev,
			       struct iio_chan_spec const *chan,
			       int val,
			       int val2,
			       long mask) {
	struct inv_mpu60x0_state  *st = iio_priv(indio_dev);
	int result;

	mutex_lock(&indio_dev->mlock);
	/* we should only update scale when the chip is disabled, i.e.,
		not running */
	if (st->chip_config.enable) {
		result = -EBUSY;
		goto error_write_raw;
	}
	result = inv_mpu60x0_set_power_itg(st, true);
	if (result)
		goto error_write_raw;

	switch (mask) {
	case IIO_CHAN_INFO_SCALE:
		switch (chan->type) {
		case IIO_ANGL_VEL:
			result = inv_mpu60x0_write_fsr(st, val);
			break;
		case IIO_ACCEL:
			result = inv_mpu60x0_write_accel_fs(st, val);
			break;
		default:
			result = -EINVAL;
			break;
		}
		break;
	default:
		result = -EINVAL;
		break;
	}

error_write_raw:
	result |= inv_mpu60x0_set_power_itg(st, false);
	mutex_unlock(&indio_dev->mlock);

	return result;
}

/**
 *  inv_mpu60x0_set_lpf() - set low pass filer based on fifo rate.
 *
 *                  Based on the Nyquist principle, the sampling rate must
 *                  exceed twice of the bandwidth of the signal, or there
 *                  would be alising. This function basically search for the
 *                  correct low pass parameters based on the fifo rate, e.g,
 *                  sampling frequency.
 */
static int inv_mpu60x0_set_lpf(struct inv_mpu60x0_state *st, int rate)
{
	const int hz[] = {188, 98, 42, 20, 10, 5};
	const int d[] = {INV_MPU60x0_FILTER_188HZ, INV_MPU60x0_FILTER_98HZ,
			INV_MPU60x0_FILTER_42HZ, INV_MPU60x0_FILTER_20HZ,
			INV_MPU60x0_FILTER_10HZ, INV_MPU60x0_FILTER_5HZ};
	int i, h, result;
	u8 data;

	h = (rate >> 1);
	i = 0;
	while ((h < hz[i]) && (i < ARRAY_SIZE(d) - 1))
		i++;
	data = d[i];
	result = inv_mpu60x0_write_reg(st, st->reg->lpf, data);
	if (result)
		return result;
	st->chip_config.lpf = data;

	return 0;
}

/**
 * inv_mpu60x0_fifo_rate_store() - Set fifo rate.
 */
static ssize_t inv_mpu60x0_fifo_rate_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	s32 fifo_rate;
	u8 d;
	int result;
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct inv_mpu60x0_state *st = iio_priv(indio_dev);

	if (kstrtoint(buf, 10, &fifo_rate))
		return -EINVAL;
	if (fifo_rate < INV_MPU60x0_MIN_FIFO_RATE ||
				fifo_rate > INV_MPU60x0_MAX_FIFO_RATE)
		return -EINVAL;
	if (fifo_rate == st->chip_config.fifo_rate)
		return count;

	mutex_lock(&indio_dev->mlock);
	if (st->chip_config.enable) {
		result = -EBUSY;
		goto fifo_rate_fail;
	}
	result = inv_mpu60x0_set_power_itg(st, true);
	if (result)
		goto fifo_rate_fail;

	d = INV_MPU60x0_ONE_K_HZ / fifo_rate - 1;
	result = inv_mpu60x0_write_reg(st, st->reg->sample_rate_div, d);
	if (result)
		goto fifo_rate_fail;
	st->chip_config.fifo_rate = fifo_rate;

	result = inv_mpu60x0_set_lpf(st, fifo_rate);
	if (result)
		goto fifo_rate_fail;

fifo_rate_fail:
	result |= inv_mpu60x0_set_power_itg(st, false);
	mutex_unlock(&indio_dev->mlock);
	if (result)
		return result;

	return count;
}

/**
 * inv_fifo_rate_show() - Get the current sampling rate.
 */
static ssize_t inv_fifo_rate_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct inv_mpu60x0_state *st = iio_priv(dev_to_iio_dev(dev));

	return sprintf(buf, "%d\n", st->chip_config.fifo_rate);
}

/**
 * inv_attr_show() - calling this function will show current
 *                    parameters.
 */
static ssize_t inv_attr_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct inv_mpu60x0_state *st = iio_priv(dev_to_iio_dev(dev));
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	s8 *m;

	switch (this_attr->address) {
	/* In MPU60x0, the two matrix are the same because gyro and accel
	   are integrated in one chip */
	case ATTR_GYRO_MATRIX:
	case ATTR_ACCL_MATRIX:
		m = st->plat_data.orientation;

		return sprintf(buf, "%d, %d, %d; %d, %d, %d; %d, %d, %d\n",
			m[0], m[1], m[2], m[3], m[4], m[5], m[6], m[7], m[8]);
	default:
		return -EINVAL;
	}
}

/**
 * inv_mpu60x0_validate_trigger() - validate_trigger callback for invensense
 *                                  MPU60x0 device.
 * @indio_dev: The IIO device
 * @trig: The new trigger
 *
 * Returns: 0 if the 'trig' matches the trigger registered by the MPU60x0
 * device, -EINVAL otherwise.
 */
static int inv_mpu60x0_validate_trigger(struct iio_dev *indio_dev,
					struct iio_trigger *trig)
{
	struct inv_mpu60x0_state *st = iio_priv(indio_dev);

	if (st->trig != trig)
		return -EINVAL;

	return 0;
}

#define INV_MPU60x0_CHAN(_type, _channel2, _index)                    \
	{                                                             \
		.type = _type,                                        \
		.modified = 1,                                        \
		.channel2 = _channel2,                                \
		.info_mask =  IIO_CHAN_INFO_SCALE_SHARED_BIT          \
				| IIO_CHAN_INFO_RAW_SEPARATE_BIT,     \
		.scan_index = _index,                                 \
		.scan_type = {                                        \
				.sign = 's',                          \
				.realbits = 16,                       \
				.storagebits = 16,                    \
				.shift = 0 ,                          \
				.endianness = IIO_BE,                 \
			     },                                       \
	}

static const struct iio_chan_spec inv_mpu_channels[] = {
	IIO_CHAN_SOFT_TIMESTAMP(INV_MPU60x0_SCAN_TIMESTAMP),
	/*
	 * Note that temperature should only be via polled reading only,
	 * not the final scan elements output.
	 */
	{
		.type = IIO_TEMP,
		.info_mask =  IIO_CHAN_INFO_RAW_SEPARATE_BIT
				| IIO_CHAN_INFO_OFFSET_SEPARATE_BIT
				| IIO_CHAN_INFO_SCALE_SEPARATE_BIT,
		.scan_index = -1,
	},
	INV_MPU60x0_CHAN(IIO_ANGL_VEL, IIO_MOD_X, INV_MPU60x0_SCAN_GYRO_X),
	INV_MPU60x0_CHAN(IIO_ANGL_VEL, IIO_MOD_Y, INV_MPU60x0_SCAN_GYRO_Y),
	INV_MPU60x0_CHAN(IIO_ANGL_VEL, IIO_MOD_Z, INV_MPU60x0_SCAN_GYRO_Z),

	INV_MPU60x0_CHAN(IIO_ACCEL, IIO_MOD_X, INV_MPU60x0_SCAN_ACCL_X),
	INV_MPU60x0_CHAN(IIO_ACCEL, IIO_MOD_Y, INV_MPU60x0_SCAN_ACCL_Y),
	INV_MPU60x0_CHAN(IIO_ACCEL, IIO_MOD_Z, INV_MPU60x0_SCAN_ACCL_Z),
};

/* constant IIO attribute */
static IIO_CONST_ATTR_SAMP_FREQ_AVAIL("10 20 50 100 200 500");
static IIO_DEV_ATTR_SAMP_FREQ(S_IRUGO | S_IWUSR, inv_fifo_rate_show,
	inv_mpu60x0_fifo_rate_store);
static IIO_DEVICE_ATTR(in_gyro_matrix, S_IRUGO, inv_attr_show, NULL,
	ATTR_GYRO_MATRIX);
static IIO_DEVICE_ATTR(in_accel_matrix, S_IRUGO, inv_attr_show, NULL,
	ATTR_ACCL_MATRIX);

static struct attribute *inv_attributes[] = {
	&iio_dev_attr_in_gyro_matrix.dev_attr.attr,
	&iio_dev_attr_in_accel_matrix.dev_attr.attr,
	&iio_dev_attr_sampling_frequency.dev_attr.attr,
	&iio_const_attr_sampling_frequency_available.dev_attr.attr,
	NULL,
};

static const struct attribute_group inv_attribute_group = {
	.attrs = inv_attributes
};

static const struct iio_info mpu_info = {
	.driver_module = THIS_MODULE,
	.read_raw = &inv_mpu60x0_read_raw,
	.write_raw = &inv_mpu60x0_write_raw,
	.attrs = &inv_attribute_group,
	.validate_trigger = inv_mpu60x0_validate_trigger,
};

/**
 *  inv_check_and_setup_chip() - check and setup chip.
 */
static int inv_check_and_setup_chip(struct inv_mpu60x0_state *st)
{
	int result;

	st->chip_type = INV_MPU60x0;
	st->hw  = &hw_info[st->chip_type];
	st->reg = hw_info[st->chip_type].reg;

	/* reset to make sure previous state are not there */
	result = inv_mpu60x0_write_reg(st, st->reg->pwr_mgmt_1,
					INV_MPU60x0_BIT_H_RESET);
	if (result)
		return result;
	msleep(INV_MPU60x0_POWER_UP_TIME);
	/* toggle power state. After reset, the sleep bit could be on
		or off depending on the OTP settings. Toggling power would
		make it in a definite state as well as making the hardware
		state align with the software state */
	result = inv_mpu60x0_set_power_itg(st, false);
	if (result)
		return result;
	result = inv_mpu60x0_set_power_itg(st, true);
	if (result)
		return result;

	result = inv_mpu60x0_switch_engine(st, false,
					INV_MPU60x0_BIT_PWR_ACCL_STBY);
	if (result)
		return result;
	result = inv_mpu60x0_switch_engine(st, false,
					INV_MPU60x0_BIT_PWR_GYRO_STBY);
	if (result)
		return result;

	return 0;
}

static int __init add_device_to_bus(void)
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

static int inv_mpu_probe(struct spi_device *spi_device)
{
	struct iio_dev *indio_dev;
	struct inv_mpu60x0_state *st;
	int result; 
	
	/*result = add_device_to_bus();
	if (result < 0)
		goto out_no_free;
	printk("device successfully added to bus\n");*/
		
	indio_dev = iio_device_alloc(sizeof(*st));
	if (indio_dev == NULL) {
		printk("Failed to allocate iio device\n");
		result = -ENOMEM;
		goto out_no_free;
	}
	
	st = iio_priv(indio_dev);
	st->spi_dev = spi_device;
	//this makes things die? why is it valid in i2c but not spi?!
	/*st->plat_data = *(struct inv_mpu60x0_platform_data
				*)dev_get_platdata(&spi_device->dev);*/
	printk("iio device allocated\n");
	// power is turned on inside check chip type
	result = inv_check_and_setup_chip(st);
	if (result){
		printk("Failed to setup mpu60x0\n");
		goto out_free;
	}

	result = inv_mpu60x0_init_config(indio_dev);
	if (result) {
		dev_err(&spi_device->dev,
			"Could not initialize device.\n");
		goto out_free;
	}
	printk("configured device\n");

	//i2c_set_clientdata(spi_device, indio_dev);
	spi_set_drvdata(spi_device, indio_dev);
	indio_dev->dev.parent = &spi_device->dev;
	printk("here1\n");
	indio_dev->name = spi_get_device_id(spi_device)->name;
	printk("here2\n");
	indio_dev->channels = inv_mpu_channels;
	indio_dev->num_channels = ARRAY_SIZE(inv_mpu_channels);

	indio_dev->info = &mpu_info;
	indio_dev->modes = INDIO_BUFFER_TRIGGERED;

	result = iio_triggered_buffer_setup(indio_dev,
					    inv_mpu60x0_irq_handler,
					    inv_mpu60x0_read_fifo,
					    NULL);
	if (result) {
		dev_err(&st->client->dev, "configure buffer fail %d\n",
				result);
		goto out_free;
	}
	printk("iio triggers setup\n");
	result = inv_mpu60x0_probe_trigger(indio_dev);
	if (result) {
		dev_err(&st->client->dev, "trigger probe fail %d\n", result);
		goto out_unreg_ring;
	}
	printk("probe triggers complete\n");

	INIT_KFIFO(st->timestamps);
	spin_lock_init(&st->time_stamp_lock);
	printk("kfifo initialised\n");
	result = iio_device_register(indio_dev);
	if (result) {
		dev_err(&st->client->dev, "IIO register fail %d\n", result);
		goto out_remove_trigger;
	}
	printk("iio device registered\n");
	
out_remove_trigger:
	inv_mpu60x0_remove_trigger(st);
out_unreg_ring:
	iio_triggered_buffer_cleanup(indio_dev);
out_free:
	iio_device_free(indio_dev);
out_no_free:

	return result;
}

static int inv_mpu_remove(struct spi_device *spi_device)
{
	struct iio_dev *indio_dev = spi_get_drvdata(spi_device);
	struct inv_mpu60x0_state *st = iio_priv(indio_dev);

	iio_device_unregister(indio_dev);
	inv_mpu60x0_remove_trigger(st);
	iio_triggered_buffer_cleanup(indio_dev);
	iio_device_free(indio_dev);
	
	printk("inv_mpu unloaded\n");

	return 0;
}

/*#ifdef CONFIG_PM_SLEEP

static int inv_mpu_resume(struct device *dev)
{
	return inv_mpu60x0_set_power_itg(
		iio_priv(i2c_get_clientdata(to_i2c_client(dev))), true);
}

static int inv_mpu_suspend(struct device *dev)
{
	return inv_mpu60x0_set_power_itg(
		iio_priv(i2c_get_clientdata(to_i2c_client(dev))), false);
}
static SIMPLE_DEV_PM_OPS(inv_mpu_pmops, inv_mpu_suspend, inv_mpu_resume);

#define INV_MPU60x0_PMOPS (&inv_mpu_pmops)
#else
#define INV_MPU60x0_PMOPS NULL
#endif */ /* CONFIG_PM_SLEEP */

static const struct spi_device_id inv_mpu_id[] = {
	{"mpu60x0", INV_MPU60x0},
	{}
};
MODULE_DEVICE_TABLE(spi, inv_mpu_id);

static struct spi_driver inv_mpu_driver = {
	.driver = {
		.name 		= this_driver_name,
		.owner		= THIS_MODULE,
//		.pm		= INV_MPU60x0_PMOPS,
	},

	.probe = inv_mpu_probe,
	.remove = inv_mpu_remove,
	//.suspend = mpu6000_suspend,
	//.resume = mpu6000_resume,
	//.id_table = inv_mpu_id,
};

static int __init mpu60x0_init(void)
{
	int error;
	//memset(&mpu6000_dev, 0, sizeof(mpu6000_dev));

	//sema_init(&mpu6000_dev.spi_sem, 1);

	error = spi_register_driver(&inv_mpu_driver);
	if (error < 0) {
		printk(KERN_ALERT "spi_register_driver() failed %d\n", error);
		return error;
	}
	
	error = add_device_to_bus();
	if (error < 0) {
		printk(KERN_ALERT "add_mpu600_to_bus() failed\n");
		goto failed;
	}

	printk("%s loaded\n", this_driver_name);

	return 0;
	
failed:
	spi_unregister_driver(&inv_mpu_driver);
	return error;
}
module_init(mpu60x0_init);

static void __exit mpu60x0_exit(void)
{
	//FIXME: need to unregister device?
	//spi_unregister_device(mpu6000_dev.spi_device);
	
	spi_unregister_driver(&inv_mpu_driver);
}
module_exit(mpu60x0_exit);

MODULE_AUTHOR("Jonathan Clapson");
MODULE_DESCRIPTION("Invensense device MPU60x0 driver");
MODULE_LICENSE("GPL");
