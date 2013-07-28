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

#include "inv_mpu_iio.h"

static void inv_scan_query(struct iio_dev *indio_dev)
{
	struct inv_mpu60x0_state  *st = iio_priv(indio_dev);

	st->chip_config.gyro_fifo_enable =
		test_bit(INV_MPU60x0_SCAN_GYRO_X,
			indio_dev->active_scan_mask) ||
			test_bit(INV_MPU60x0_SCAN_GYRO_Y,
			indio_dev->active_scan_mask) ||
			test_bit(INV_MPU60x0_SCAN_GYRO_Z,
			indio_dev->active_scan_mask);

	st->chip_config.accl_fifo_enable =
		test_bit(INV_MPU60x0_SCAN_ACCL_X,
			indio_dev->active_scan_mask) ||
			test_bit(INV_MPU60x0_SCAN_ACCL_Y,
			indio_dev->active_scan_mask) ||
			test_bit(INV_MPU60x0_SCAN_ACCL_Z,
			indio_dev->active_scan_mask);
}

/**
 *  inv_mpu60x0_set_enable() - enable chip functions.
 *  @indio_dev:	Device driver instance.
 *  @enable: enable/disable
 */
static int inv_mpu60x0_set_enable(struct iio_dev *indio_dev, bool enable)
{
	struct inv_mpu60x0_state *st = iio_priv(indio_dev);
	int result;

	if (enable) {
		result = inv_mpu60x0_set_power_itg(st, true);
		if (result)
			return result;
		inv_scan_query(indio_dev);
		if (st->chip_config.gyro_fifo_enable) {
			result = inv_mpu60x0_switch_engine(st, true,
					INV_MPU60x0_BIT_PWR_GYRO_STBY);
			if (result)
				return result;
		}
		if (st->chip_config.accl_fifo_enable) {
			result = inv_mpu60x0_switch_engine(st, true,
					INV_MPU60x0_BIT_PWR_ACCL_STBY);
			if (result)
				return result;
		}
		result = inv_reset_fifo(indio_dev);
		if (result)
			return result;
	} else {
		result = inv_mpu60x0_write_reg(st, st->reg->fifo_en, 0);
		if (result)
			return result;

		result = inv_mpu60x0_write_reg(st, st->reg->int_enable, 0);
		if (result)
			return result;

		result = inv_mpu60x0_write_reg(st, st->reg->user_ctrl, 0);
		if (result)
			return result;

		result = inv_mpu60x0_switch_engine(st, false,
					INV_MPU60x0_BIT_PWR_GYRO_STBY);
		if (result)
			return result;

		result = inv_mpu60x0_switch_engine(st, false,
					INV_MPU60x0_BIT_PWR_ACCL_STBY);
		if (result)
			return result;
		result = inv_mpu60x0_set_power_itg(st, false);
		if (result)
			return result;
	}
	st->chip_config.enable = enable;

	return 0;
}

/**
 * inv_mpu_data_rdy_trigger_set_state() - set data ready interrupt state
 * @trig: Trigger instance
 * @state: Desired trigger state
 */
static int inv_mpu_data_rdy_trigger_set_state(struct iio_trigger *trig,
						bool state)
{
	return inv_mpu60x0_set_enable(trig->private_data, state);
}

static const struct iio_trigger_ops inv_mpu_trigger_ops = {
	.owner = THIS_MODULE,
	.set_trigger_state = &inv_mpu_data_rdy_trigger_set_state,
};

int inv_mpu60x0_probe_trigger(struct iio_dev *indio_dev)
{
	int ret;
	struct inv_mpu60x0_state *st = iio_priv(indio_dev);

	st->trig = iio_trigger_alloc("%s-dev%d",
					indio_dev->name,
					indio_dev->id);
	if (st->trig == NULL) {
		ret = -ENOMEM;
		printk("trigger allocation failed!\n");
		goto error_ret;
	}
	ret = request_irq(st->client->irq, &iio_trigger_generic_data_rdy_poll,
				IRQF_TRIGGER_RISING,
				"inv_mpu",
				st->trig);
	if (ret) {
		printk("request irq failed!\n");
		goto error_free_trig;
	}
	st->trig->dev.parent = &st->client->dev;
	st->trig->private_data = indio_dev;
	st->trig->ops = &inv_mpu_trigger_ops;
	ret = iio_trigger_register(st->trig);
	if (ret)
	{
		printk("trigger registration failed!\n");
		goto error_free_irq;		
	}
	indio_dev->trig = st->trig;

	return 0;

error_free_irq:
	free_irq(st->client->irq, st->trig);
error_free_trig:
	iio_trigger_free(st->trig);
error_ret:
	return ret;
}

void inv_mpu60x0_remove_trigger(struct inv_mpu60x0_state *st)
{
	iio_trigger_unregister(st->trig);
	free_irq(st->client->irq, st->trig);
	iio_trigger_free(st->trig);
}
