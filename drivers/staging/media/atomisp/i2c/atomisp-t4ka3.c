// SPDX-License-Identifier: GPL-2.0
/*
 * Support for T4KA3 8M camera sensor.
 *
 * Copyright (C) 2015 Intel Corporation. All Rights Reserved.
 * Copyright (C) 2016 XiaoMi, Inc.
 * Copyright (C) 2024 Hans de Goede <hansg@kernel.org>
 */

#include <linux/bitops.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/gpio/machine.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/kmod.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/pm_runtime.h>
#include <linux/string.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <media/v4l2-device.h>
#include <asm/intel-mid.h>
#include <linux/firmware.h>
#include <linux/acpi.h>
#include <linux/proc_fs.h>

#include "t4ka3.h"

static inline struct t4ka3_device *ctrl_to_t4ka3(struct v4l2_ctrl *ctrl)
{
	return container_of(ctrl->handler, struct t4ka3_device, ctrls.handler);
}

/* the bayer order mapping table
 *          hflip=0			hflip=1
 * vflip=0  atomisp_bayer_order_grbg atomisp_bayer_order_rggb
 * vflip=1  atomisp_bayer_order_bggr atomisp_bayer_order_gbrg
 *
 * usage: t4ka3_bayer_order_mapping[vflip][hflip]
 */

/* T4KA3 default GRBG */
static const int t4ka3_bayer_order_mapping[2][2] = {
	{ atomisp_bayer_order_grbg, atomisp_bayer_order_rggb },
	{ atomisp_bayer_order_bggr, atomisp_bayer_order_gbrg }
};

static int t4ka3_detect(struct i2c_client *client, u16 *id);

static int t4ka3_read_reg(struct i2c_client *client, u16 len,
						u16 reg, u16 *val)
{
	struct i2c_msg msg[2];
	u16 data[T4KA3_SHORT_MAX] = {0};
	int err, i;
	int retry_cnt = 5;

	if (len > T4KA3_BYTE_MAX) {
		dev_err(&client->dev,
			"%s error, invalid data length\n", __func__);
		return -EINVAL;
	}

	memset(msg, 0 , sizeof(msg));

	msg[0].addr = client->addr;
	/*msg[0].addr = 0x37;*/
	msg[0].flags = 0;
	msg[0].len = I2C_MSG_LENGTH;
	msg[0].buf = (u8 *)data;
	/* high byte goes first */
	data[0] = cpu_to_be16(reg);

	msg[1].addr = client->addr;
	/*msg[1].addr = 0x37;*/
	msg[1].len = len;
	msg[1].flags = I2C_M_RD;
	msg[1].buf = (u8 *)data;

	while (retry_cnt-- > 0) {
		err = i2c_transfer(client->adapter, msg, 2);
		if (err != 2) {
			if (err >= 0)
				err = -EIO;

			if (retry_cnt <= 0)
				goto error;
		} else
			break;
	}

	/* high byte comes first */
	if (len == T4KA3_8BIT)
		*val = (u8)data[0];
	else {
		/* 16-bit access is default when len > 1 */
		for (i = 0; i < (len >> 1); i++)
			val[i] = be16_to_cpu(data[i]);
	}

	return 0;

error:
	dev_err(&client->dev, "read from offset 0x%x error %d", reg, err);
	return err;
}

static int t4ka3_i2c_write(struct i2c_client *client, u16 len, u8 *data)
{
	struct i2c_msg msg;
	const int num_msg = 1;
	int ret;

	msg.addr = client->addr;
	/*msg.addr = 0x37;*/

	msg.flags = 0;
	msg.len = len;
	msg.buf = data;
	ret = i2c_transfer(client->adapter, &msg, 1);

	if (ret != num_msg)
		dev_err(&client->dev, "%s error!! ret = %d\n", __func__, ret);

	return ret == num_msg ? 0 : -EIO;
}

static int t4ka3_write_reg(struct i2c_client *client, u16 data_length,
							u16 reg, u16 val)
{
	int ret;
	unsigned char data[4] = {0};
	u16 *wreg = (u16 *)data;
	const u16 len = data_length + sizeof(u16); /* 16-bit address + data */
	int retry_cnt = 5;

	if (data_length != T4KA3_8BIT && data_length != T4KA3_16BIT) {
		dev_err(&client->dev,
			"%s error, invalid data_length\n", __func__);
		return -EINVAL;
	}

	/* high byte goes out first */
	*wreg = cpu_to_be16(reg);

	if (data_length == T4KA3_8BIT) {
		data[2] = (u8)(val);
	} else {
		/* T4KA3_16BIT */
		u16 *wdata = (u16 *)&data[2];
		*wdata = cpu_to_be16(val);
	}

	while (retry_cnt-- > 0) {
		ret = t4ka3_i2c_write(client, len, data);
		if (ret) {

		} else
			break;
	}

	return ret;
}

/*
 * t4ka3_write_reg_array - Initializes a list of T4KA3 registers
 * @client: i2c driver client structure
 * @reglist: list of registers to be written
 *
 * This function initializes a list of registers. When consecutive addresses
 * are found in a row on the list, this function creates a buffer and sends
 * consecutive data in a single i2c_transfer().
 *
 * __t4ka3_flush_reg_array, __t4ka3_buf_reg_array() and
 * __t4ka3_write_reg_is_consecutive() are internal functions to
 * t4ka3_write_reg_array_fast() and should be not used anywhere else.
 *
 */

static int __t4ka3_flush_reg_array(struct i2c_client *client,
				     struct t4ka3_write_ctrl *ctrl)
{
	u16 size;

	if (ctrl->index == 0)
		return 0;

	size = sizeof(u16) + ctrl->index; /* 16-bit address + data */
	ctrl->buffer.addr = cpu_to_be16(ctrl->buffer.addr);
	ctrl->index = 0;

	return t4ka3_i2c_write(client, size, (u8 *)&ctrl->buffer);
}

static int __t4ka3_buf_reg_array(struct i2c_client *client,
				   struct t4ka3_write_ctrl *ctrl,
				   const struct t4ka3_reg *next)
{
	int size;
	u16 *data16;

	switch (next->type) {
	case T4KA3_8BIT:
		size = 1;
		ctrl->buffer.data[ctrl->index] = (u8)next->val;
		break;
	case T4KA3_16BIT:
		size = 2;
		data16 = (u16 *)&ctrl->buffer.data[ctrl->index];
		*data16 = cpu_to_be16((u16)next->val);
		break;
	default:
		return -EINVAL;
	}

	/* When first item is added, we need to store its starting address */
	if (ctrl->index == 0)
		ctrl->buffer.addr = next->sreg;

	ctrl->index += size;

	/*
	 * Buffer cannot guarantee free space for u32? Better flush it to avoid
	 * possible lack of memory for next item.
	 */
	if (ctrl->index + sizeof(u16) >= T4KA3_MAX_WRITE_BUF_SIZE)
		return __t4ka3_flush_reg_array(client, ctrl);

	return 0;
}

static int __t4ka3_write_reg_is_consecutive(struct i2c_client *client,
				   struct t4ka3_write_ctrl *ctrl,
				   const struct t4ka3_reg *next)
{
	if (ctrl->index == 0)
		return 1;
	return ctrl->buffer.addr + ctrl->index == next->sreg;
}

static int t4ka3_write_reg_array(struct i2c_client *client,
			const struct t4ka3_reg *reglist)
{
	const struct t4ka3_reg *next = reglist;
	struct t4ka3_write_ctrl ctrl;
	int err;

	ctrl.index = 0;
	for (; next->type != T4KA3_TOK_TERM; next++) {
		switch (next->type & T4KA3_TOK_MASK) {
		case T4KA3_TOK_DELAY:
			err = __t4ka3_flush_reg_array(client, &ctrl);
			if (err) {
				dev_err(&client->dev,
					"%s: write error\n", __func__);
			return err;
			}
			msleep(next->val);
			break;
		default:
			/*
			* If next address is not consecutive, data needs to be
			* flushed before proceed.
			*/
			if (!__t4ka3_write_reg_is_consecutive(client, &ctrl,
						next)) {

			err = __t4ka3_flush_reg_array(client, &ctrl);
			if (err) {
				dev_err(&client->dev,
					"%s@%d: write error\n", __func__, __LINE__);
				return err;
				}
			}
			err = __t4ka3_buf_reg_array(client, &ctrl, next);
			if (err) {
				dev_err(&client->dev,
					"%s@%d: write error\n", __func__, __LINE__);
				return err;
			}
			break;
		}
	}


	return __t4ka3_flush_reg_array(client, &ctrl);
}

static int __t4ka3_init(struct v4l2_subdev *sd, u32 val)
{

	/* restore settings */
	t4ka3_res = t4ka3_res_preview;
	N_RES = N_RES_PREVIEW;

	return 0;
}

static int t4ka3_init(struct v4l2_subdev *sd, u32 val)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret = 0;

	ret = __t4ka3_init(sd, val);
	dev_info(&client->dev, "t4ka3_init_config\n");

	ret = t4ka3_write_reg_array(client, t4ka3_init_config);

	return ret;
}

/*
 * distance - calculate the distance
 * @res: resolution
 * @w: width
 * @h: height
 *
 * Get the gap between resolution and w/h.
 * res->width/height smaller than w/h wouldn't be considered.
 * Returns the value of gap or -1 if fail.
 */
#define LARGEST_ALLOWED_RATIO_MISMATCH 600
static int distance(struct t4ka3_resolution *res, u32 w, u32 h)
{
	unsigned int w_ratio = ((res->width << 13) / w);
	unsigned int h_ratio;
	int match;

	if (h == 0)
		return -EPERM;
	h_ratio = ((res->height << 13) / h);
	if (h_ratio == 0)
		return -EPERM;
	match   = abs(((w_ratio << 13) / h_ratio) - ((int)8192));

	if ((w_ratio < (int)8192) || (h_ratio < (int)8192)  ||
		(match > LARGEST_ALLOWED_RATIO_MISMATCH))
		return -EPERM;

	return w_ratio + h_ratio;
}

/* Return the nearest higher resolution index */
static int nearest_resolution_index(int w, int h)
{
	int i;
	int idx = -1;
	int dist;
	int min_dist = INT_MAX;
	struct t4ka3_resolution *tmp_res = NULL;

	for (i = 0; i < N_RES; i++) {
		tmp_res = &t4ka3_res[i];
		dist = distance(tmp_res, w, h);
		if (dist == -1)
			continue;
		if (dist < min_dist) {
			min_dist = dist;
			idx = i;
		}
	}

	return idx;
}

static int __t4ka3_try_mbus_fmt(struct v4l2_subdev *sd,
				struct v4l2_mbus_framefmt *fmt)
{
	struct t4ka3_device *dev = to_t4ka3_sensor(sd);
	int idx;

	mutex_lock(&dev->input_lock);

	if ((fmt->width > T4KA3_RES_WIDTH_MAX) ||
		(fmt->height > T4KA3_RES_HEIGHT_MAX)) {
		fmt->width = T4KA3_RES_WIDTH_MAX;
		fmt->height = T4KA3_RES_HEIGHT_MAX;
		fmt->code = MEDIA_BUS_FMT_SGRBG10_1X10;
	} else {
		idx = nearest_resolution_index(fmt->width, fmt->height);

		/*
		 * nearest_resolution_index() doesn't return smaller
		 *  resolutions. If it fails, it means the requested
		 *  resolution is higher than wecan support. Fallback
		 *  to highest possible resolution in this case.
		 */
		if (idx == -1)
			idx = N_RES - 1;

		fmt->width = t4ka3_res[idx].width;
		fmt->height = t4ka3_res[idx].height;
		fmt->code = t4ka3_res[idx].code;
	}

	mutex_unlock(&dev->input_lock);
	return 0;
}

static int __t4ka3_set_mbus_fmt(struct v4l2_subdev *sd,
			        struct v4l2_mbus_framefmt *fmt)
{
	struct t4ka3_device *dev = to_t4ka3_sensor(sd);
	const struct t4ka3_reg *t4ka3_def_reg;
	struct camera_mipi_info *t4ka3_info = NULL;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret;
	u8 tmp;
	int vflip, hflip;

	dev_info(&client->dev, "enter t4ka3_set_mbus_fmt\n");

	t4ka3_info = v4l2_get_subdev_hostdata(sd);
	if (t4ka3_info == NULL)
		return -EINVAL;

	ret = __t4ka3_try_mbus_fmt(sd, fmt);
	if (ret) {
		v4l2_err(sd, "try fmt fail\n");
		return ret;
	}

	mutex_lock(&dev->input_lock);
	dev->fmt_idx = nearest_resolution_index(fmt->width, fmt->height);
	dev_info(&client->dev, "fmt_idx %d, width %d , height %d\n",
		dev->fmt_idx, fmt->width, fmt->height);
	/* Sanity check */
	if (unlikely(dev->fmt_idx == -1)) {
		mutex_unlock(&dev->input_lock);
		v4l2_err(sd, "get resolution fail\n");
		return -EINVAL;
	}

	t4ka3_def_reg = t4ka3_res[dev->fmt_idx].regs;

	t4ka3_info->input_format = ATOMISP_INPUT_FORMAT_RAW_10;
	/* enable group hold */
	ret = t4ka3_write_reg_array(client, t4ka3_param_hold);
	if (ret) {
		mutex_unlock(&dev->input_lock);
		return ret;
	}

	ret = t4ka3_write_reg_array(client, t4ka3_def_reg);
	if (ret) {
		mutex_unlock(&dev->input_lock);
		return ret;
	}

	/* disable group hold */
	ret = t4ka3_write_reg_array(client, t4ka3_param_update);
	if (ret) {
		mutex_unlock(&dev->input_lock);
		return ret;
	}
	dev->coarse_itg = 0;
	dev->gain = 0;

	ret = t4ka3_read_reg(client, T4KA3_8BIT,
				T4KA3_REG_IMG_ORIENTATION, (u16 *)&tmp);
	if (ret) {
		mutex_unlock(&dev->input_lock);
		return ret;
	}
	hflip = tmp & T4KA3_HFLIP_BIT;
	vflip = (tmp & T4KA3_VFLIP_BIT) >> T4KA3_VFLIP_OFFSET;
	t4ka3_info->raw_bayer_order =
				t4ka3_bayer_order_mapping[vflip][hflip];
	dev_info(&client->dev, "bayer order %d\n",
		t4ka3_info->raw_bayer_order);

	/*t4ka3_write_reg(client, T4KA3_16BIT,
			T4KA3_REG_TEST_PATTERN_MODE, 0x0304);*/

	mutex_unlock(&dev->input_lock);
	return 0;
}

/* Horizontal flip the image. */
static int t4ka3_t_hflip(struct v4l2_subdev *sd, int value)
{
	struct i2c_client *c = v4l2_get_subdevdata(sd);
	struct t4ka3_device *dev = to_t4ka3_sensor(sd);
	int ret;
	u16 val;

	/* enable group hold */
	ret = t4ka3_write_reg_array(c, t4ka3_param_hold);

	ret = t4ka3_read_reg(c, T4KA3_8BIT,
				T4KA3_REG_IMG_ORIENTATION, &val);
	if (ret)
		return ret;
	if (value)
		val |= T4KA3_HFLIP_BIT;
	else
		val &= ~T4KA3_HFLIP_BIT;
	ret = t4ka3_write_reg(c, T4KA3_8BIT,
				T4KA3_REG_IMG_ORIENTATION, val);
	if (ret)
		return ret;

	ret = t4ka3_write_reg_array(c, t4ka3_param_update);

	dev->flip = val;

	return ret;
}

/* Vertically flip the image */
static int t4ka3_t_vflip(struct v4l2_subdev *sd, int value)
{
	struct i2c_client *c = v4l2_get_subdevdata(sd);
	struct t4ka3_device *dev = to_t4ka3_sensor(sd);
	int ret;
	u16 val;

	/* enable group hold */
	ret = t4ka3_write_reg_array(c, t4ka3_param_hold);

	ret = t4ka3_read_reg(c, T4KA3_8BIT,
				T4KA3_REG_IMG_ORIENTATION, &val);
	if (ret)
		return ret;
	if (value)
		val |= T4KA3_VFLIP_BIT;
	else
		val &= ~T4KA3_VFLIP_BIT;
	ret = t4ka3_write_reg(c, T4KA3_8BIT,
				T4KA3_REG_IMG_ORIENTATION, val);
	if (ret)
		return ret;

	ret = t4ka3_write_reg_array(c, t4ka3_param_update);

	dev->flip = val;

	return ret;
}

static int t4ka3_test_pattern(struct v4l2_subdev *sd, s32 value)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	return t4ka3_write_reg(client, T4KA3_8BIT,
			T4KA3_REG_TEST_PATTERN_MODE, 0x0304);
}

static long __t4ka3_set_exposure(struct v4l2_subdev *sd,
					u16 coarse_itg,
					u16 gain, u16 digitalgain)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret;
	u16 lines_per_frame;
	struct t4ka3_device *dev = to_t4ka3_sensor(sd);

	/* Validate exposure:  cannot exceed VTS-4 where VTS is 16bit */
	coarse_itg = clamp_t(u16, coarse_itg,
		T4KA3_COARSE_INTEGRATION_TIME_MIN,
					T4KA3_MAX_EXPOSURE_SUPPORTED);
	/* Validate gain: must not exceed maximum 8bit value */
	gain = clamp_t(u16, gain, T4KA3_MIN_GLOBAL_GAIN_SUPPORTED,
					T4KA3_MAX_GLOBAL_GAIN_SUPPORTED);

	/* check coarse integration time margin */
	if (coarse_itg > T4K3A_LINES_PER_FRAME -
					T4KA3_COARSE_INTEGRATION_TIME_MARGIN)
		lines_per_frame = coarse_itg +
					T4KA3_COARSE_INTEGRATION_TIME_MARGIN;
	else
		lines_per_frame = T4K3A_LINES_PER_FRAME;

	ret = t4ka3_write_reg(client, T4KA3_16BIT,
				T4KA3_REG_FRAME_LENGTH_LINES,
				lines_per_frame);
	if (ret)
		goto out_disable;

	/* set exposure gain */
	ret = t4ka3_write_reg(client, T4KA3_16BIT,
				T4KA3_REG_COARSE_INTEGRATION_TIME,
				coarse_itg);
	if (ret)
		goto out_disable;

	/* set analogue gain */
	ret = t4ka3_write_reg(client, T4KA3_16BIT,
					T4KA3_REG_GLOBAL_GAIN, gain);
	if (ret)
		goto out_disable;
	/* set digital gain*/
	ret = t4ka3_write_reg(client, T4KA3_16BIT,
					T4KA3_REG_DIGGAIN_GREEN_R, digitalgain);
	if (ret)
		goto out_disable;
	ret = t4ka3_write_reg(client, T4KA3_16BIT,
					T4KA3_REG_DIGGAIN_RED, digitalgain);
	if (ret)
		goto out_disable;
	ret = t4ka3_write_reg(client, T4KA3_16BIT,
					T4KA3_REG_DIGGAIN_BLUE, digitalgain);
	if (ret)
		goto out_disable;
	ret = t4ka3_write_reg(client, T4KA3_16BIT,
					T4KA3_REG_DIGGAIN_GREEN_B, digitalgain);
	if (ret)
		goto out_disable;

	dev->gain       = gain;
	dev->coarse_itg = coarse_itg;
	dev->digital_gain = digitalgain;
out_disable:
/*	t4ka3_write_reg_array(client, t4ka3_param_update);
out:*/
	return ret;
}

static int t4ka3_set_exposure(struct v4l2_subdev *sd, u16 exposure,
				u16 gain,  u16 digitalgain)
{
	struct t4ka3_device *dev = to_t4ka3_sensor(sd);
	int ret;

	mutex_lock(&dev->input_lock);
	ret = __t4ka3_set_exposure(sd, exposure, gain, digitalgain);
	mutex_unlock(&dev->input_lock);

	return ret;
}
static long t4ka3_s_exposure(struct v4l2_subdev *sd,
			struct atomisp_exposure *exposure)
{
	u16 coarse_itg, analog_gain, digital_gain;

	coarse_itg = exposure->integration_time[0];
	analog_gain = exposure->gain[0];
	digital_gain = exposure->gain[1];

	return t4ka3_set_exposure(sd, coarse_itg, analog_gain, digital_gain);
}

static int t4ka3_update_exposure_range (struct v4l2_subdev *sd)
{
	int exp_max;
	struct t4ka3_device *sensor;

	if (sd == NULL)
		return -1;
	
	sensor = to_t4ka3_sensor (sd);

	exp_max = sensor->format.height + sensor->ctrls.vblank->val - T4KA3_COARSE_INTEGRATION_TIME_MARGIN;

	return __v4l2_ctrl_modify_range(sensor->ctrls.exposure, 0, exp_max,
					1, exp_max);
}

static long t4ka3_ioctl(struct v4l2_subdev *sd,
						unsigned int cmd, void *arg)
{

	switch (cmd) {
	case ATOMISP_IOC_S_EXPOSURE:
		return t4ka3_s_exposure(sd, (struct atomisp_exposure *)arg);
	default:
		return -EINVAL;
	}
	return 0;
}

static u8 vid;
static struct proc_dir_entry *iddir;
static struct proc_dir_entry *idfile;
static int vendorid_proc_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%d\n", vid);
	return 0;
}

static int vendorid_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, vendorid_proc_show, NULL);
}
static const struct proc_ops vendorid_proc_ops = {
	.proc_open = vendorid_proc_open,
	.proc_read = seq_read,
	.proc_lseek = seq_lseek,
	.proc_release = single_release,
 };

static int t4ka3_vendorid_procfs_init(struct i2c_client *client)
{
	iddir = proc_mkdir("camera", NULL);
	if (!iddir) {
		dev_err(&client->dev, "can't create /proc/camera/\n");
		return -EPERM;
	}

	idfile = proc_create("backvid", 0644, iddir,
				&vendorid_proc_ops);
	if (!idfile) {
		dev_err(&client->dev, "Can't create file /proc/camera/backvid\n");
		remove_proc_entry("camera", iddir);
		return -EPERM;
	}

	return 0;
}
static int t4ka3_vendorid_procfs_uninit(void)
{
	if (idfile != NULL)
		proc_remove(idfile);
	if (iddir != NULL)
		proc_remove(iddir);
	return 0;
}

static int t4ka3_detect(struct i2c_client *client, u16 *id)
{
	struct i2c_adapter *adapter = client->adapter;
	u16 high, low;

	/* i2c check */
	if (!i2c_check_functionality(adapter, I2C_FUNC_I2C))
		return -ENODEV;

	/* check sensor chip ID	 */
	if (t4ka3_read_reg(client, T4KA3_8BIT, T4KA3_REG_PRODUCT_ID,
			     &high)) {
		dev_err(&client->dev, "sensor_id_high = 0x%x\n", high);
		return -ENODEV;
	}

	if (t4ka3_read_reg(client, T4KA3_8BIT, T4KA3_REG_PRODUCT_ID + 1,
			     &low)) {
		dev_err(&client->dev, "sensor_id_low = 0x%x\n", low);
		return -ENODEV;
	}

	*id = (((u8) high) << 8) | (u8) low;

	if (*id != T4KA3_PRODUCT_ID) {
		dev_err(&client->dev, "main sensor t4ka3 ID error\n");
		return -ENODEV;
	}

	dev_info(&client->dev, "sensor detect find sensor_id = 0x%x\n", *id);

	return 0;
}

static int
t4ka3_s_config(struct v4l2_subdev *sd, int irq)
{
	struct t4ka3_device *dev = to_t4ka3_sensor(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	u16 sensor_id;
	int ret;

	ret = pm_runtime_get_sync(dev->sd.dev);
	if (ret) {
		dev_err(&client->dev, "t4ka3 power-up err");
		return ret;
	}

	ret = t4ka3_detect(client, &sensor_id);
	if (ret) {
		dev_err(&client->dev, "Failed to detect sensor.\n");
		goto fail_detect;
	}
	dev_info(&client->dev, "s_config finish\n");

fail_detect:
	pm_runtime_put(dev->sd.dev);;
	return ret;
}

static int t4ka3_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct t4ka3_device *dev = ctrl_to_t4ka3(ctrl);
	struct i2c_client *client = v4l2_get_subdevdata(&dev->sd);
	int ret = 0;

	switch (ctrl->id) {
	case V4L2_CID_TEST_PATTERN:
		dev_dbg(&client->dev, "%s: V4L2_CID_TEST_PATTERN: %d\n",
			__func__, ctrl->val);
		ret = t4ka3_test_pattern(&dev->sd, ctrl->val);
		break;
	case V4L2_CID_VFLIP:
		dev_dbg(&client->dev, "%s: V4L2_CID_VFLIP: %d\n",
			__func__, ctrl->val);
		ret = t4ka3_t_vflip(&dev->sd, ctrl->val);
		break;
	case V4L2_CID_HFLIP:
		dev_dbg(&client->dev, "%s: V4L2_CID_HFLIP: %d\n",
			__func__, ctrl->val);
		ret = t4ka3_t_hflip(&dev->sd, ctrl->val);
		break;
	case V4L2_CID_VBLANK:
		dev_dbg(&client->dev, "%s: V4L2_CID_VBLANK: %d\n",
			__func__, ctrl->val);
		t4ka3_update_exposure_range (&dev->sd);
		break;
	default:
		ret = -EINVAL;
	}
	return ret;
}

static int t4ka3_recovery(struct v4l2_subdev *sd)
{
	int ret;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct t4ka3_device *dev = to_t4ka3_sensor(sd);

	dev_info(&client->dev, "Start recovering T4KA3.\n");

	ret = pm_runtime_put(sd->dev);
	if (ret) {
		dev_err(&client->dev, "power-down err.\n");
		return ret;
	}

	ret = pm_runtime_get_sync(sd->dev);
	if (ret) {
		dev_err(&client->dev, "power-up err.\n");
		return ret;
	}

	/* enable group hold */
	ret = t4ka3_write_reg_array(client, t4ka3_param_hold);
	if (ret)
		return ret;

	ret = t4ka3_write_reg(client, T4KA3_8BIT,
			T4KA3_REG_IMG_ORIENTATION, dev->flip);
	if (ret)
		return ret;

	ret = t4ka3_write_reg_array(client, t4ka3_res[dev->fmt_idx].regs);
	if (ret)
		return ret;

	/* disable group hold */
	ret = t4ka3_write_reg_array(client, t4ka3_param_update);
	if (ret)
		return ret;

	return ret;
}


static int t4ka3_s_stream(struct v4l2_subdev *sd, int enable)
{
	int ret;
	u16 id;

	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct t4ka3_device *dev = to_t4ka3_sensor(sd);

	mutex_lock(&dev->input_lock);

	if (dev->streaming == enable) {
		dev_warn (&client->dev, "Stream aleady %s\n", enable ? "started" : "stopped");
		goto error_unlock;
	}

	if (enable) {

		dev_info (&client->dev, "power on while streaming set on t4ka3");

		ret = pm_runtime_get_sync(dev->sd.dev);
		if (ret) {
			dev_err(&client->dev, "power-up err.\n");
			goto error_unlock;
		}

		dev_info (&client->dev, "Detect t4ka3\n");
		ret = t4ka3_detect(client, &id);
		if (ret) {
			ret = t4ka3_recovery(sd);
			if (ret) {
				dev_err(&client->dev, "recovery err.\n");
				goto error_powerdown;
			}
		}

		ret = t4ka3_write_reg_array(client,
					    t4ka3_streaming);
		if (ret) {
			dev_err (&client->dev, "Error on setting reg\n");
			goto error_powerdown;
		}

		dev->streaming = 1;
	} else {

		ret = t4ka3_write_reg_array(client,
					    t4ka3_suspend);
		if (ret) {
			dev_err(&client->dev, "Error on writing streaming config\n");
			goto error_powerdown;
		}

		ret = pm_runtime_put(dev->sd.dev);
		if (ret)
			goto error_unlock;

		dev->streaming = 0;
	}

	mutex_unlock(&dev->input_lock);
	return ret;

error_powerdown:
	ret = pm_runtime_put(dev->sd.dev);
error_unlock:
	mutex_unlock(&dev->input_lock);
	return ret;
}

static int
t4ka3_enum_mbus_code(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *sd_state,
				struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index >= MAX_FMTS)
		return -EINVAL;
	code->code = MEDIA_BUS_FMT_SGRBG10_1X10;

	return 0;
}

static int
t4ka3_enum_frame_size(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *sd_state,
				struct v4l2_subdev_frame_size_enum *fse)
{
	int index = fse->index;

	if (index >= N_RES)
		return -EINVAL;

	fse->min_width = t4ka3_res[index].width;
	fse->min_height = t4ka3_res[index].height;
	fse->max_width = t4ka3_res[index].width;
	fse->max_height = t4ka3_res[index].height;

	return 0;
}

static struct v4l2_mbus_framefmt *
__t4ka3_get_pad_format(struct t4ka3_device *sensor, struct v4l2_subdev *sd,
			 struct v4l2_subdev_state *sd_state, unsigned int pad,
			 enum v4l2_subdev_format_whence which)
{
	switch (which) {
	case V4L2_SUBDEV_FORMAT_TRY:
		return v4l2_subdev_state_get_format(sd_state, pad);
	case V4L2_SUBDEV_FORMAT_ACTIVE:
		return &sensor->format;
	default:
		return NULL;
	}
}

static int
t4ka3_get_pad_format(struct v4l2_subdev *sd,
		       struct v4l2_subdev_state *sd_state,
		       struct v4l2_subdev_format *fmt)
{
	struct t4ka3_device *dev = to_t4ka3_sensor(sd);
	struct v4l2_mbus_framefmt *format =
			__t4ka3_get_pad_format(dev, sd, sd_state,
						fmt->pad, fmt->which);

	fmt->format = *format;

	return 0;
}

static int t4ka3_set_pad_format(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *sd_state,
				struct v4l2_subdev_format *fmt)
{
	struct t4ka3_device *dev = to_t4ka3_sensor(sd);

	if (fmt->pad)
		return -EINVAL;

	if (fmt->which == V4L2_SUBDEV_FORMAT_ACTIVE)
		dev->format = fmt->format;

	/* This calls __t4ka3_try_mbus_fmt() internally */
	return __t4ka3_set_mbus_fmt(sd, &fmt->format);
}

static int t4ka3_get_frame_interval(struct v4l2_subdev *sd,
				    struct v4l2_subdev_state *sd_state,
				    struct v4l2_subdev_frame_interval *interval)
{
	interval->interval.numerator = 1;
	interval->interval.denominator = T4K3A_FPS;
	return 0;
}

static int t4ka3_g_skip_frames(struct v4l2_subdev *sd, u32 *frames)
{
	struct t4ka3_device *dev = to_t4ka3_sensor(sd);

	mutex_lock(&dev->input_lock);
	*frames = t4ka3_res[dev->fmt_idx].skip_frames;
	mutex_unlock(&dev->input_lock);

	return 0;
}

static struct v4l2_ctrl_ops t4ka3_ctrl_ops = {
	.s_ctrl = t4ka3_s_ctrl,
};

static const struct v4l2_subdev_sensor_ops t4ka3_sensor_ops = {
	.g_skip_frames = t4ka3_g_skip_frames,
};

static const struct v4l2_subdev_video_ops t4ka3_video_ops = {
	.s_stream = t4ka3_s_stream,
};

static const struct v4l2_subdev_core_ops t4ka3_core_ops = {
	.ioctl = t4ka3_ioctl,
//	.s_power = t4ka3_s_power,
	.init = t4ka3_init,
};

static const struct v4l2_subdev_pad_ops t4ka3_pad_ops = {
	.enum_mbus_code = t4ka3_enum_mbus_code,
	.enum_frame_size = t4ka3_enum_frame_size,
	.get_fmt = t4ka3_get_pad_format,
	.set_fmt = t4ka3_set_pad_format,
	.get_frame_interval = t4ka3_get_frame_interval,
};

static const struct v4l2_subdev_ops t4ka3_ops = {
	.core = &t4ka3_core_ops,
	.video = &t4ka3_video_ops,
	.pad = &t4ka3_pad_ops,
	.sensor = &t4ka3_sensor_ops,
};

static void t4ka3_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct t4ka3_device *dev = to_t4ka3_sensor(sd);

	/*sysfs_remove_group(&client->dev.kobj, t4ka3_attribute_group);*/
	v4l2_device_unregister_subdev(sd);
	media_entity_cleanup(&dev->sd.entity);
	atomisp_unregister_subdev(sd);
	t4ka3_vendorid_procfs_uninit();
	v4l2_ctrl_handler_free(&dev->ctrls.handler);
	pm_runtime_disable(&client->dev);
	kfree(dev);
}

static int t4ka3_init_controls(struct t4ka3_device *sensor)
{
	const struct v4l2_ctrl_ops *ops = &t4ka3_ctrl_ops;
	struct t4ka3_ctrls *ctrls = &sensor->ctrls;
	struct v4l2_ctrl_handler *hdl = &ctrls->handler;
	int def, max;
	/* FIXME copied from ov2680 very likely wrong */
	static const char * const test_pattern_menu[] = {
		"Disabled",
		"Color Bars",
		"Random Data",
		"Square",
		"Black Image",
	};

	v4l2_ctrl_handler_init(hdl, 4);

	hdl->lock = &sensor->input_lock;

	ctrls->vflip = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_VFLIP, 0, 1, 1, 0);
	ctrls->hflip = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_HFLIP, 0, 1, 1, 0);

	ctrls->test_pattern = v4l2_ctrl_new_std_menu_items(hdl, ops,
					V4L2_CID_TEST_PATTERN,
					ARRAY_SIZE(test_pattern_menu) - 1,
					0, 0, test_pattern_menu);
	ctrls->link_freq = v4l2_ctrl_new_int_menu(hdl, NULL, V4L2_CID_LINK_FREQ,
						  0, 0, sensor->link_freq);

	def = T4K3A_LINES_PER_FRAME - T4KA3_RES_HEIGHT_MAX;
	max = 0xffff - T4KA3_RES_HEIGHT_MAX;
	/* FIXME: need a datasheet to verify the min VBI */
	ctrls->vblank = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_VBLANK,
					  4, max, 1, def);

	max = T4K3A_LINES_PER_FRAME - T4KA3_COARSE_INTEGRATION_TIME_MARGIN;
	ctrls->exposure = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_EXPOSURE,
					    0, max, 1, max);

	if (hdl->error)
		return hdl->error;

	ctrls->vflip->flags |= V4L2_CTRL_FLAG_MODIFY_LAYOUT;
	ctrls->hflip->flags |= V4L2_CTRL_FLAG_MODIFY_LAYOUT;
	ctrls->link_freq->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	sensor->sd.ctrl_handler = hdl;
	return 0;
}

/*
 * T4KA3 can be control through only one powerdown GPIO but ACPI lists
 * two GPIOs for T4KA3. The function for the second GPIO is uncleared.
 * Therefore, we still get two GPIOs from ACPI and set them with the
 * same value with out of tree driver.
 */
static const struct acpi_gpio_params t4ka3_powerdown_gpio = { 0, 0, true };
static const struct acpi_gpio_params t4ka3_powerdown_alt_gpio = { 1, 0, true };

static const struct acpi_gpio_mapping t4ka3_gpio_mapping[] = {
	{ "powerdown-gpios", &t4ka3_powerdown_gpio, 1 },
	{ "powerdown-alt-gpios", &t4ka3_powerdown_alt_gpio, 1 },
	{ },
};

static int t4ka3_pm_suspend(struct device *dev)
{
	struct t4ka3_device *sensor = dev_get_drvdata(dev);

	gpiod_set_value_cansleep(sensor->powerdown_gpio, 1);
	gpiod_set_value_cansleep(sensor->powerdown_alt_gpio, 1);

	return 0;
}

static int t4ka3_pm_resume(struct device *dev)
{
	int ret = 0;
	u16 sensor_id;
	struct t4ka3_device *sensor = dev_get_drvdata(dev);
	struct i2c_client *client = v4l2_get_subdevdata(&sensor->sd);

	usleep_range(5000, 6000);

	gpiod_set_value_cansleep(sensor->powerdown_gpio, 0);
	gpiod_set_value_cansleep(sensor->powerdown_alt_gpio, 0);

	/* waiting for the sensor after powering up */
	msleep(20);

	ret = t4ka3_detect(client, &sensor_id);
	
	if (ret) {
		dev_err(&client->dev, "sensor detect failed\n");
		return ret;
	}

	return 0;
}

static DEFINE_RUNTIME_DEV_PM_OPS(t4ka3_pm_ops, t4ka3_pm_suspend, t4ka3_pm_resume, NULL);

static int t4ka3_probe(struct i2c_client *client)
{
	struct t4ka3_device *dev;
	int ret = 0;

	/* allocate sensor device & init sub device */
	dev = devm_kzalloc(&client->dev, sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	mutex_init(&dev->input_lock);

	dev->link_freq[0] = T4K3A_LINK_FREQ;
	dev->fmt_idx = 0;
	iddir = NULL;
	idfile = NULL;

	v4l2_i2c_subdev_init(&(dev->sd), client, &t4ka3_ops);

	ret = devm_acpi_dev_add_driver_gpios(&client->dev, t4ka3_gpio_mapping);
	if (ret) {
		dev_err (&client->dev, "Error on adding ACPI GPIO.");
		return ret;
	}

	dev->powerdown_gpio = devm_gpiod_get(&client->dev, "powerdown", GPIOD_OUT_HIGH);
	if (IS_ERR(dev->powerdown_gpio))
		return dev_err_probe(&client->dev, PTR_ERR(dev->powerdown_gpio), "getting powerdown GPIO\n");

	dev->powerdown_alt_gpio = devm_gpiod_get_optional(&client->dev, "powerdown-alt", GPIOD_OUT_HIGH);
	if (IS_ERR(dev->powerdown_alt_gpio))
		return dev_err_probe(&client->dev, PTR_ERR(dev->powerdown_alt_gpio), "getting powerdown-alt GPIO\n");
	
	pm_runtime_set_suspended(&client->dev);
	pm_runtime_enable(&client->dev);
	pm_runtime_set_autosuspend_delay(&client->dev, 1000);
	pm_runtime_use_autosuspend(&client->dev);

	ret = t4ka3_s_config(&dev->sd, client->irq);
	if (ret) {
		dev_err(&client->dev, "configuration fail!!\n");
		atomisp_gmin_remove_subdev(&dev->sd);
		goto out_free;
	}

	ret = atomisp_register_sensor_no_gmin(&dev->sd, 4, ATOMISP_INPUT_FORMAT_RAW_10,
					      atomisp_bayer_order_bggr);
	if (ret) {
		t4ka3_remove(client);
		return ret;
	}

	dev->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	dev->pad.flags = MEDIA_PAD_FL_SOURCE;
	dev->format.code = MEDIA_BUS_FMT_SGRBG10_1X10;
	dev->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	dev->flip = 0;

	ret = t4ka3_init_controls(dev);
	if (ret) {
		t4ka3_remove(client);
		return ret;
	}

	ret = media_entity_pads_init(&dev->sd.entity, 1, &dev->pad);
	if (ret)
		t4ka3_remove(client);

	ret = t4ka3_vendorid_procfs_init(client);
	if (ret) {
		dev_err(&client->dev, "%s Failed to proc fs\n", __func__);
		t4ka3_vendorid_procfs_uninit();
	}

	v4l2_info(client, "%s: done!! with ret %d\n", __func__, ret);

	return ret;

out_free:
	v4l2_device_unregister_subdev(&dev->sd);
	return ret;
}

static const struct i2c_device_id t4ka3_id[] = {
	{T4KA3_NAME, 0},
	{ }
};
MODULE_DEVICE_TABLE(i2c, t4ka3_id);

/*Temp ID, need change to official one after get from TOSHIBA*/
static struct acpi_device_id T4KA3_acpi_match[] = {
	{ "TOSB0001" },
	{ "XMCC0003" },
	{},
};
MODULE_DEVICE_TABLE(acpi, T4KA3_acpi_match);

static struct i2c_driver t4ka3_driver = {
	.driver = {
		.name = T4KA3_NAME,
		.acpi_match_table = ACPI_PTR(T4KA3_acpi_match),
		.pm = pm_sleep_ptr(&t4ka3_pm_ops),
	},
	.probe = t4ka3_probe,
	.remove = t4ka3_remove,
	.id_table = t4ka3_id,
};

module_i2c_driver(t4ka3_driver)

MODULE_DESCRIPTION("A low-level driver for T4KA3 sensor");
MODULE_AUTHOR("HARVEY LV <harvey.lv@intel.com>");
MODULE_LICENSE("GPL");
