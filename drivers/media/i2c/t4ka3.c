// SPDX-License-Identifier: GPL-2.0
/*
 * Support for T4KA3 8M camera sensor.
 *
 * Copyright (C) 2015 Intel Corporation. All Rights Reserved.
 * Copyright (C) 2016 XiaoMi, Inc.
 * Copyright (C) 2024 Hans de Goede <hansg@kernel.org>
 */

#include <linux/acpi.h>
#include <linux/bits.h>
#include <linux/delay.h>
#include <linux/dev_printk.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/mod_devicetable.h>
#include <linux/mutex.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/types.h>

#include <media/media-entity.h>
#include <media/v4l2-async.h>
#include <media/v4l2-cci.h>
#include <media/v4l2-common.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-subdev.h>

#define T4KA3_NATIVE_WIDTH			3280
#define T4KA3_NATIVE_HEIGHT			2464
#define T4KA3_NATIVE_START_LEFT			0
#define T4KA3_NATIVE_START_TOP			0
#define T4KA3_ACTIVE_WIDTH			3280
#define T4KA3_ACTIVE_HEIGHT			2460
#define T4KA3_ACTIVE_START_LEFT			0
#define T4KA3_ACTIVE_START_TOP			2
#define T4KA3_MIN_CROP_WIDTH			2
#define T4KA3_MIN_CROP_HEIGHT			2

#define T4KA3_PIXELS_PER_LINE			3440
#define T4KA3_LINES_PER_FRAME			2492
#define T4KA3_FPS				30
#define T4KA3_PIXEL_RATE \
	(T4KA3_PIXELS_PER_LINE * T4KA3_LINES_PER_FRAME * T4KA3_FPS)

/*
 * TODO this really should be derived from the 19.2 MHz xvclk combined
 * with the PLL settings. But without a datasheet this is the closest
 * approximation possible.
 *
 * link-freq = pixel_rate * bpp / (lanes * 2)
 * (lanes * 2) because CSI lanes use double-data-rate (DDR) signalling.
 * bpp = 10 and lanes = 4
 */
#define T4KA3_LINK_FREQ				((s64)T4KA3_PIXEL_RATE * 10 / 8)


#define T4KA3_REG_PRODUCT_ID_HIGH		CCI_REG8(0x0000)
#define T4KA3_REG_PRODUCT_ID_LOW		CCI_REG8(0x0001)
#define T4KA3_PRODUCT_ID			0x1490

#define T4KA3_REG_STREAM			CCI_REG8(0x0100)
#define T4KA3_REG_IMG_ORIENTATION		CCI_REG8(0x0101)
#define T4KA3_HFLIP_BIT				BIT(0)
#define T4KA3_VFLIP_BIT				BIT(1)
#define T4KA3_REG_PARAM_HOLD			CCI_REG8(0x0104)
#define T4KA3_REG_COARSE_INTEGRATION_TIME	CCI_REG16(0x0202)
#define T4KA3_COARSE_INTEGRATION_TIME_MARGIN	6
#define T4KA3_REG_DIGGAIN_GREEN_R		CCI_REG16(0x020e)
#define T4KA3_REG_DIGGAIN_RED			CCI_REG16(0x0210)
#define T4KA3_REG_DIGGAIN_BLUE			CCI_REG16(0x0212)
#define T4KA3_REG_DIGGAIN_GREEN_B		CCI_REG16(0x0214)
#define T4KA3_REG_GLOBAL_GAIN			CCI_REG16(0x0234)
#define T4KA3_MIN_GLOBAL_GAIN_SUPPORTED		0x0080
#define T4KA3_MAX_GLOBAL_GAIN_SUPPORTED		0x07ff
#define T4KA3_REG_FRAME_LENGTH_LINES		CCI_REG16(0x0340) /* aka VTS */
/* FIXME: need a datasheet to verify the min + max vblank values */
#define T4KA3_MIN_VBLANK			4
#define T4KA3_MAX_VBLANK			0xffff
#define T4KA3_REG_PIXELS_PER_LINE		CCI_REG16(0x0342) /* aka HTS */
/* These 2 being horz/vert start is a guess (no datasheet), always 0 */
#define T4KA3_REG_HORZ_START			CCI_REG16(0x0344)
#define T4KA3_REG_VERT_START			CCI_REG16(0x0346)
/* Always 3279 (T4KA3_NATIVE_WIDTH - 1, window is used to crop */
#define T4KA3_REG_HORZ_END			CCI_REG16(0x0348)
/* Always 2463 (T4KA3_NATIVE_HEIGHT - 1, window is used to crop */
#define T4KA3_REG_VERT_END			CCI_REG16(0x034a)
/* Output size (after cropping/window) */
#define T4KA3_REG_HORZ_OUTPUT_SIZE		CCI_REG16(0x034c)
#define T4KA3_REG_VERT_OUTPUT_SIZE		CCI_REG16(0x034e)
/* Window/crop start + size *after* binning */
#define T4KA3_REG_WIN_START_X			CCI_REG16(0x0408)
#define T4KA3_REG_WIN_START_Y			CCI_REG16(0x040a)
#define T4KA3_REG_WIN_WIDTH			CCI_REG16(0x040c)
#define T4KA3_REG_WIN_HEIGHT			CCI_REG16(0x040e)
#define T4KA3_REG_TEST_PATTERN_MODE		CCI_REG8(0x0601)

struct t4ka3_resolution {
	const struct cci_reg_sequence *regs;
	int width;
	int height;
	int regs_len;
	u32 skip_frames;
};

struct t4ka3_ctrls {
	struct v4l2_ctrl_handler handler;
	struct v4l2_ctrl *hflip;
	struct v4l2_ctrl *vflip;
	struct v4l2_ctrl *vblank;
	struct v4l2_ctrl *hblank;
	struct v4l2_ctrl *exposure;
	struct v4l2_ctrl *test_pattern;
	struct v4l2_ctrl *link_freq;
	struct v4l2_ctrl *gain;
};

struct t4ka3_data {
	struct v4l2_subdev sd;
	struct media_pad pad;
	struct v4l2_mbus_framefmt format;
	struct mutex lock; /* serialize sensor's ioctl */
	struct t4ka3_ctrls ctrls;
	struct device *dev;
	struct regmap *regmap;
	struct gpio_desc *powerdown_gpio;
	struct gpio_desc *reset_gpio;
	s64 link_freq[1];
	const struct t4ka3_resolution *res;
	int streaming;
};

/**
 * struct t4ka3_reg - MI sensor  register format
 * @reg: 16-bit offset to register
 * @val: 8/16/32-bit register value
 *
 * Define a structure for sensor register initialization values
 */
struct t4ka3_reg {
	u16 sreg;
	u32 val;	/* @set value for read/mod/write, @mask */
};

#define to_t4ka3_sensor(x) container_of(x, struct t4ka3_data, sd)

/* init settings */
static const struct cci_reg_sequence t4ka3_init_config[] = {
	{CCI_REG8(0x4136), 0x13},
	{CCI_REG8(0x4137), 0x33},
	{CCI_REG8(0x3094), 0x01},
	{CCI_REG8(0x0233), 0x01},
	{CCI_REG8(0x4B06), 0x01},
	{CCI_REG8(0x4B07), 0x01},
	{CCI_REG8(0x3028), 0x01},
	{CCI_REG8(0x3032), 0x14},
	{CCI_REG8(0x305C), 0x0C},
	{CCI_REG8(0x306D), 0x0A},
	{CCI_REG8(0x3071), 0xFA},
	{CCI_REG8(0x307E), 0x0A},
	{CCI_REG8(0x307F), 0xFC},
	{CCI_REG8(0x3091), 0x04},
	{CCI_REG8(0x3092), 0x60},
	{CCI_REG8(0x3096), 0xC0},
	{CCI_REG8(0x3100), 0x07},
	{CCI_REG8(0x3101), 0x4C},
	{CCI_REG8(0x3118), 0xCC},
	{CCI_REG8(0x3139), 0x06},
	{CCI_REG8(0x313A), 0x06},
	{CCI_REG8(0x313B), 0x04},
	{CCI_REG8(0x3143), 0x02},
	{CCI_REG8(0x314F), 0x0E},
	{CCI_REG8(0x3169), 0x99},
	{CCI_REG8(0x316A), 0x99},
	{CCI_REG8(0x3171), 0x05},
	{CCI_REG8(0x31A1), 0xA7},
	{CCI_REG8(0x31A2), 0x9C},
	{CCI_REG8(0x31A3), 0x8F},
	{CCI_REG8(0x31A4), 0x75},
	{CCI_REG8(0x31A5), 0xEE},
	{CCI_REG8(0x31A6), 0xEA},
	{CCI_REG8(0x31A7), 0xE4},
	{CCI_REG8(0x31A8), 0xE4},
	{CCI_REG8(0x31DF), 0x05},
	{CCI_REG8(0x31EC), 0x1B},
	{CCI_REG8(0x31ED), 0x1B},
	{CCI_REG8(0x31EE), 0x1B},
	{CCI_REG8(0x31F0), 0x1B},
	{CCI_REG8(0x31F1), 0x1B},
	{CCI_REG8(0x31F2), 0x1B},
	{CCI_REG8(0x3204), 0x3F},
	{CCI_REG8(0x3205), 0x03},
	{CCI_REG8(0x3210), 0x01},
	{CCI_REG8(0x3216), 0x68},
	{CCI_REG8(0x3217), 0x58},
	{CCI_REG8(0x3218), 0x58},
	{CCI_REG8(0x321A), 0x68},
	{CCI_REG8(0x321B), 0x60},
	{CCI_REG8(0x3238), 0x03},
	{CCI_REG8(0x3239), 0x03},
	{CCI_REG8(0x323A), 0x05},
	{CCI_REG8(0x323B), 0x06},
	{CCI_REG8(0x3243), 0x03},
	{CCI_REG8(0x3244), 0x08},
	{CCI_REG8(0x3245), 0x01},
	{CCI_REG8(0x3307), 0x19},
	{CCI_REG8(0x3308), 0x19},
	{CCI_REG8(0x3320), 0x01},
	{CCI_REG8(0x3326), 0x15},
	{CCI_REG8(0x3327), 0x0D},
	{CCI_REG8(0x3328), 0x01},
	{CCI_REG8(0x3380), 0x01},
	{CCI_REG8(0x339E), 0x07},
	{CCI_REG8(0x3424), 0x00},
	{CCI_REG8(0x343C), 0x01},
	{CCI_REG8(0x3398), 0x04},
	{CCI_REG8(0x343A), 0x10},
	{CCI_REG8(0x339A), 0x22},
	{CCI_REG8(0x33B4), 0x00},
	{CCI_REG8(0x3393), 0x01},
	{CCI_REG8(0x33B3), 0x6E},
	{CCI_REG8(0x3433), 0x06},
	{CCI_REG8(0x3433), 0x00},
	{CCI_REG8(0x33B3), 0x00},
	{CCI_REG8(0x3393), 0x03},
	{CCI_REG8(0x33B4), 0x03},
	{CCI_REG8(0x343A), 0x00},
	{CCI_REG8(0x339A), 0x00},
	{CCI_REG8(0x3398), 0x00}
};

static struct cci_reg_sequence const t4ka3_736x496_30fps[] = {
	{CCI_REG8(0x0112), 0x0A},
	{CCI_REG8(0x0113), 0x0A},
	{CCI_REG8(0x0114), 0x03},
	{CCI_REG8(0x4136), 0x13},
	{CCI_REG8(0x4137), 0x33},
	{CCI_REG8(0x0820), 0x0A},
	{CCI_REG8(0x0821), 0x0D},
	{CCI_REG8(0x0822), 0x00},
	{CCI_REG8(0x0823), 0x00},
	{CCI_REG8(0x0301), 0x0A},
	{CCI_REG8(0x0303), 0x01},
	{CCI_REG8(0x0305), 0x04},
	{CCI_REG8(0x0306), 0x02},
	{CCI_REG8(0x0307), 0x18},
	{CCI_REG8(0x030B), 0x01},
	{CCI_REG8(0x034C), 0x02},
	{CCI_REG8(0x034D), 0xE0},
	{CCI_REG8(0x034E), 0x01},
	{CCI_REG8(0x034F), 0xEE},
	{CCI_REG8(0x0340), 0x09},
	{CCI_REG8(0x0341), 0xBC},
	{CCI_REG8(0x0342), 0x0D},
	{CCI_REG8(0x0343), 0x70},
	{CCI_REG8(0x0344), 0x00},
	{CCI_REG8(0x0345), 0x00},
	{CCI_REG8(0x0346), 0x00},
	{CCI_REG8(0x0347), 0x00},
	{CCI_REG8(0x0348), 0x0C},
	{CCI_REG8(0x0349), 0xCF},
	{CCI_REG8(0x034A), 0x09},
	{CCI_REG8(0x034B), 0x9F},
	{CCI_REG8(0x0408), 0x01},
	{CCI_REG8(0x0409), 0x74},
	{CCI_REG8(0x040A), 0x00},
	{CCI_REG8(0x040B), 0xFA},
	{CCI_REG8(0x040C), 0x02},
	{CCI_REG8(0x040D), 0xE0},
	{CCI_REG8(0x040E), 0x01},
	{CCI_REG8(0x040F), 0xF0},
	{CCI_REG8(0x0900), 0x01},
	{CCI_REG8(0x0901), 0x22},
	{CCI_REG8(0x0902), 0x00},
	{CCI_REG8(0x4220), 0x00},
	{CCI_REG8(0x4222), 0x01},
	{CCI_REG8(0x3380), 0x01},
	{CCI_REG8(0x3090), 0x88},
	{CCI_REG8(0x3394), 0x20},
	{CCI_REG8(0x3090), 0x08},
	{CCI_REG8(0x3394), 0x10}
};

static struct cci_reg_sequence const t4ka3_896x736_30fps[] = {
	{CCI_REG8(0x0112), 0x0A},
	{CCI_REG8(0x0113), 0x0A},
	{CCI_REG8(0x0114), 0x03},
	{CCI_REG8(0x4136), 0x13},
	{CCI_REG8(0x4137), 0x33},
	{CCI_REG8(0x0820), 0x0A},
	{CCI_REG8(0x0821), 0x0D},
	{CCI_REG8(0x0822), 0x00},
	{CCI_REG8(0x0823), 0x00},
	{CCI_REG8(0x0301), 0x0A},
	{CCI_REG8(0x0303), 0x01},
	{CCI_REG8(0x0305), 0x04},
	{CCI_REG8(0x0306), 0x02},
	{CCI_REG8(0x0307), 0x18},
	{CCI_REG8(0x030B), 0x01},
	{CCI_REG8(0x034C), 0x03},
	{CCI_REG8(0x034D), 0x80},
	{CCI_REG8(0x034E), 0x02},
	{CCI_REG8(0x034F), 0xDE},
	{CCI_REG8(0x0340), 0x09},
	{CCI_REG8(0x0341), 0xBC},
	{CCI_REG8(0x0342), 0x0D},
	{CCI_REG8(0x0343), 0x70},
	{CCI_REG8(0x0344), 0x00},
	{CCI_REG8(0x0345), 0x00},
	{CCI_REG8(0x0346), 0x00},
	{CCI_REG8(0x0347), 0x00},
	{CCI_REG8(0x0348), 0x0C},
	{CCI_REG8(0x0349), 0xCF},
	{CCI_REG8(0x034A), 0x09},
	{CCI_REG8(0x034B), 0x9F},
	{CCI_REG8(0x0408), 0x01},
	{CCI_REG8(0x0409), 0x74},
	{CCI_REG8(0x040A), 0x00},
	{CCI_REG8(0x040B), 0xFA},
	{CCI_REG8(0x040C), 0x03},
	{CCI_REG8(0x040D), 0x80},
	{CCI_REG8(0x040E), 0x02},
	{CCI_REG8(0x040F), 0xE0},
	{CCI_REG8(0x0900), 0x01},
	{CCI_REG8(0x0901), 0x22},
	{CCI_REG8(0x0902), 0x00},
	{CCI_REG8(0x4220), 0x00},
	{CCI_REG8(0x4222), 0x01},
	{CCI_REG8(0x3380), 0x01},
	{CCI_REG8(0x3090), 0x88},
	{CCI_REG8(0x3394), 0x20},
	{CCI_REG8(0x3090), 0x08},
	{CCI_REG8(0x3394), 0x10}
};

#if 0 /* unused */
static struct t4ka3_reg const t4ka3_1296x736_30fps[] = {
	{T4KA3_8BIT, 0x0112, 0x0A},
	{T4KA3_8BIT, 0x0113, 0x0A},
	{T4KA3_8BIT, 0x0114, 0x03},
	{T4KA3_8BIT, 0x4136, 0x13},
	{T4KA3_8BIT, 0x4137, 0x33},
	{T4KA3_8BIT, 0x0820, 0x0A},
	{T4KA3_8BIT, 0x0821, 0x0D},
	{T4KA3_8BIT, 0x0822, 0x00},
	{T4KA3_8BIT, 0x0823, 0x00},
	{T4KA3_8BIT, 0x0301, 0x0A},
	{T4KA3_8BIT, 0x0303, 0x01},
	{T4KA3_8BIT, 0x0305, 0x04},
	{T4KA3_8BIT, 0x0306, 0x02},
	{T4KA3_8BIT, 0x0307, 0x18},
	{T4KA3_8BIT, 0x030B, 0x01},
	{T4KA3_8BIT, 0x034C, 0x05},
	{T4KA3_8BIT, 0x034D, 0x10},
	{T4KA3_8BIT, 0x034E, 0x02},
	{T4KA3_8BIT, 0x034F, 0xDE},
	{T4KA3_8BIT, 0x0340, 0x09},
	{T4KA3_8BIT, 0x0341, 0xBC},
	{T4KA3_8BIT, 0x0342, 0x0D},
	{T4KA3_8BIT, 0x0343, 0x70},
	{T4KA3_8BIT, 0x0344, 0x00},
	{T4KA3_8BIT, 0x0345, 0x00},
	{T4KA3_8BIT, 0x0346, 0x00},
	{T4KA3_8BIT, 0x0347, 0x00},
	{T4KA3_8BIT, 0x0348, 0x0C},
	{T4KA3_8BIT, 0x0349, 0xCF},
	{T4KA3_8BIT, 0x034A, 0x09},
	{T4KA3_8BIT, 0x034B, 0x9F},
	{T4KA3_8BIT, 0x0408, 0x00},
	{T4KA3_8BIT, 0x0409, 0xAC},
	{T4KA3_8BIT, 0x040A, 0x00},
	{T4KA3_8BIT, 0x040B, 0xFA},
	{T4KA3_8BIT, 0x040C, 0x05},
	{T4KA3_8BIT, 0x040D, 0x10},
	{T4KA3_8BIT, 0x040E, 0x02},
	{T4KA3_8BIT, 0x040F, 0xE0},
	{T4KA3_8BIT, 0x0900, 0x01},
	{T4KA3_8BIT, 0x0901, 0x22},
	{T4KA3_8BIT, 0x0902, 0x00},
	{T4KA3_8BIT, 0x4220, 0x00},
	{T4KA3_8BIT, 0x4222, 0x01},
	{T4KA3_8BIT, 0x3380, 0x01},
	{T4KA3_8BIT, 0x3090, 0x88},
	{T4KA3_8BIT, 0x3394, 0x20},
	{T4KA3_8BIT, 0x3090, 0x08},
	{T4KA3_8BIT, 0x3394, 0x10},
	{T4KA3_TOK_TERM, 0, 0 }
};
#endif

#if 0 /* unused */
static struct t4ka3_reg const t4ka3_1632x1224_30fps[] = {
	{T4KA3_8BIT, 0x0112, 0x0A},
	{T4KA3_8BIT, 0x0113, 0x0A},
	{T4KA3_8BIT, 0x0114, 0x03},
	{T4KA3_8BIT, 0x4136, 0x13},
	{T4KA3_8BIT, 0x4137, 0x33},
	{T4KA3_8BIT, 0x0820, 0x0A},
	{T4KA3_8BIT, 0x0821, 0x0D},
	{T4KA3_8BIT, 0x0822, 0x00},
	{T4KA3_8BIT, 0x0823, 0x00},
	{T4KA3_8BIT, 0x0301, 0x0A},
	{T4KA3_8BIT, 0x0303, 0x01},
	{T4KA3_8BIT, 0x0305, 0x04},
	{T4KA3_8BIT, 0x0306, 0x02},
	{T4KA3_8BIT, 0x0307, 0x18},
	{T4KA3_8BIT, 0x030B, 0x01},
	{T4KA3_8BIT, 0x034C, 0x06},
	{T4KA3_8BIT, 0x034D, 0x60},
	{T4KA3_8BIT, 0x034E, 0x04},
	{T4KA3_8BIT, 0x034F, 0xC6},
	{T4KA3_8BIT, 0x0340, 0x09},
	{T4KA3_8BIT, 0x0341, 0xBC},
	{T4KA3_8BIT, 0x0342, 0x0D},
	{T4KA3_8BIT, 0x0343, 0x70},
	{T4KA3_8BIT, 0x0344, 0x00},
	{T4KA3_8BIT, 0x0345, 0x00},
	{T4KA3_8BIT, 0x0346, 0x00},
	{T4KA3_8BIT, 0x0347, 0x00},
	{T4KA3_8BIT, 0x0348, 0x0C},
	{T4KA3_8BIT, 0x0349, 0xCF},
	{T4KA3_8BIT, 0x034A, 0x09},
	{T4KA3_8BIT, 0x034B, 0x9F},
	{T4KA3_8BIT, 0x0408, 0x00},
	{T4KA3_8BIT, 0x0409, 0x04},
	{T4KA3_8BIT, 0x040A, 0x00},
	{T4KA3_8BIT, 0x040B, 0x06},
	{T4KA3_8BIT, 0x040C, 0x06},
	{T4KA3_8BIT, 0x040D, 0x60},
	{T4KA3_8BIT, 0x040E, 0x04},
	{T4KA3_8BIT, 0x040F, 0xC8},
	{T4KA3_8BIT, 0x0900, 0x01},
	{T4KA3_8BIT, 0x0901, 0x22},
	{T4KA3_8BIT, 0x0902, 0x00},
	{T4KA3_8BIT, 0x4220, 0x00},
	{T4KA3_8BIT, 0x4222, 0x01},
	{T4KA3_8BIT, 0x3380, 0x01},
	{T4KA3_8BIT, 0x3090, 0x88},
	{T4KA3_8BIT, 0x3394, 0x20},
	{T4KA3_8BIT, 0x3090, 0x08},
	{T4KA3_8BIT, 0x3394, 0x10},
	{T4KA3_TOK_TERM, 0, 0 }
};
#endif

static struct cci_reg_sequence const t4ka3_1936x1096_30fps[] = {
	{CCI_REG8(0x0112), 0x0A},
	{CCI_REG8(0x0113), 0x0A},
	{CCI_REG8(0x0114), 0x03},
	{CCI_REG8(0x4136), 0x13},
	{CCI_REG8(0x4137), 0x33},
	{CCI_REG8(0x0820), 0x0A},
	{CCI_REG8(0x0821), 0x0D},
	{CCI_REG8(0x0822), 0x00},
	{CCI_REG8(0x0823), 0x00},
	{CCI_REG8(0x0301), 0x0A},
	{CCI_REG8(0x0303), 0x01},
	{CCI_REG8(0x0305), 0x04},
	{CCI_REG8(0x0306), 0x02},
	{CCI_REG8(0x0307), 0x18},
	{CCI_REG8(0x030B), 0x01},
	{CCI_REG8(0x034C), 0x07},
	{CCI_REG8(0x034D), 0x90},
	{CCI_REG8(0x034E), 0x04},
	{CCI_REG8(0x034F), 0x46},
	{CCI_REG8(0x0340), 0x09},
	{CCI_REG8(0x0341), 0xBC},
	{CCI_REG8(0x0342), 0x0D},
	{CCI_REG8(0x0343), 0x70},
	{CCI_REG8(0x0344), 0x00},
	{CCI_REG8(0x0345), 0x00},
	{CCI_REG8(0x0346), 0x00},
	{CCI_REG8(0x0347), 0x00},
	{CCI_REG8(0x0348), 0x0c},
	{CCI_REG8(0x0349), 0xCF},
	{CCI_REG8(0x034A), 0x09},
	{CCI_REG8(0x034B), 0x9F},
	{CCI_REG8(0x0408), 0x02},
	{CCI_REG8(0x0409), 0xA0},
	{CCI_REG8(0x040A), 0x02},
	{CCI_REG8(0x040B), 0xAE},
	{CCI_REG8(0x040C), 0x07},
	{CCI_REG8(0x040D), 0x90},
	{CCI_REG8(0x040E), 0x04},
	{CCI_REG8(0x040F), 0x4B}, /* Should be 0x48 ? */
	{CCI_REG8(0x0900), 0x01},
	{CCI_REG8(0x0901), 0x11},
	{CCI_REG8(0x0902), 0x00},
	{CCI_REG8(0x4220), 0x00},
	{CCI_REG8(0x4222), 0x01},
	{CCI_REG8(0x3380), 0x01},
	{CCI_REG8(0x3090), 0x88},
	{CCI_REG8(0x3394), 0x20},
	{CCI_REG8(0x3090), 0x08},
	{CCI_REG8(0x3394), 0x10}
};

#if 0 /* unused */
static struct t4ka3_reg const t4ka3_2064x1552_30fps[] = {
	{T4KA3_8BIT, 0x0112, 0x0A},
	{T4KA3_8BIT, 0x0113, 0x0A},
	{T4KA3_8BIT, 0x0114, 0x03},
	{T4KA3_8BIT, 0x4136, 0x13},
	{T4KA3_8BIT, 0x4137, 0x33},
	{T4KA3_8BIT, 0x0820, 0x0A},
	{T4KA3_8BIT, 0x0821, 0x0D},
	{T4KA3_8BIT, 0x0822, 0x00},
	{T4KA3_8BIT, 0x0823, 0x00},
	{T4KA3_8BIT, 0x0301, 0x0A},
	{T4KA3_8BIT, 0x0303, 0x01},
	{T4KA3_8BIT, 0x0305, 0x04},
	{T4KA3_8BIT, 0x0306, 0x02},
	{T4KA3_8BIT, 0x0307, 0x18},
	{T4KA3_8BIT, 0x030B, 0x01},
	{T4KA3_8BIT, 0x034C, 0x08},
	{T4KA3_8BIT, 0x034D, 0x10},
	{T4KA3_8BIT, 0x034E, 0x06},
	{T4KA3_8BIT, 0x034F, 0x0E},
	{T4KA3_8BIT, 0x0340, 0x09},
	{T4KA3_8BIT, 0x0341, 0xBC},
	{T4KA3_8BIT, 0x0342, 0x0D},
	{T4KA3_8BIT, 0x0343, 0x70},
	{T4KA3_8BIT, 0x0344, 0x00},
	{T4KA3_8BIT, 0x0345, 0x00},
	{T4KA3_8BIT, 0x0346, 0x00},
	{T4KA3_8BIT, 0x0347, 0x00},
	{T4KA3_8BIT, 0x0348, 0x0C},
	{T4KA3_8BIT, 0x0349, 0xCF},
	{T4KA3_8BIT, 0x034A, 0x09},
	{T4KA3_8BIT, 0x034B, 0x9F},
	{T4KA3_8BIT, 0x0408, 0x02},
	{T4KA3_8BIT, 0x0409, 0x60},
	{T4KA3_8BIT, 0x040A, 0x01},
	{T4KA3_8BIT, 0x040B, 0xCA},
	{T4KA3_8BIT, 0x040C, 0x08},
	{T4KA3_8BIT, 0x040D, 0x10},
	{T4KA3_8BIT, 0x040E, 0x06},
	{T4KA3_8BIT, 0x040F, 0x10},
	{T4KA3_8BIT, 0x0900, 0x01},
	{T4KA3_8BIT, 0x0901, 0x11},
	{T4KA3_8BIT, 0x0902, 0x00},
	{T4KA3_8BIT, 0x4220, 0x00},
	{T4KA3_8BIT, 0x4222, 0x01},
	{T4KA3_8BIT, 0x3380, 0x01},
	{T4KA3_8BIT, 0x3090, 0x88},
	{T4KA3_8BIT, 0x3394, 0x20},
	{T4KA3_8BIT, 0x3090, 0x08},
	{T4KA3_8BIT, 0x3394, 0x10},
	{T4KA3_TOK_TERM, 0, 0 }
};
#endif

#if 0 /* unused */
static struct t4ka3_reg const t4ka3_2576x1936_30fps[] = {
	{T4KA3_8BIT, 0x0112, 0x0A},
	{T4KA3_8BIT, 0x0113, 0x0A},
	{T4KA3_8BIT, 0x0114, 0x03},
	{T4KA3_8BIT, 0x4136, 0x13},
	{T4KA3_8BIT, 0x4137, 0x33},
	{T4KA3_8BIT, 0x0820, 0x0A},
	{T4KA3_8BIT, 0x0821, 0x0D},
	{T4KA3_8BIT, 0x0822, 0x00},
	{T4KA3_8BIT, 0x0823, 0x00},
	{T4KA3_8BIT, 0x0301, 0x0A},
	{T4KA3_8BIT, 0x0303, 0x01},
	{T4KA3_8BIT, 0x0305, 0x04},
	{T4KA3_8BIT, 0x0306, 0x02},
	{T4KA3_8BIT, 0x0307, 0x18},
	{T4KA3_8BIT, 0x030B, 0x01},
	{T4KA3_8BIT, 0x034C, 0x0A},
	{T4KA3_8BIT, 0x034D, 0x10},
	{T4KA3_8BIT, 0x034E, 0x07},
	{T4KA3_8BIT, 0x034F, 0x8E},
	{T4KA3_8BIT, 0x0340, 0x09},
	{T4KA3_8BIT, 0x0341, 0xBC},
	{T4KA3_8BIT, 0x0342, 0x0D},
	{T4KA3_8BIT, 0x0343, 0x70},
	{T4KA3_8BIT, 0x0344, 0x00},
	{T4KA3_8BIT, 0x0345, 0x00},
	{T4KA3_8BIT, 0x0346, 0x00},
	{T4KA3_8BIT, 0x0347, 0x00},
	{T4KA3_8BIT, 0x0348, 0x0C},
	{T4KA3_8BIT, 0x0349, 0xCF},
	{T4KA3_8BIT, 0x034A, 0x09},
	{T4KA3_8BIT, 0x034B, 0x9F},
	{T4KA3_8BIT, 0x0408, 0x01},
	{T4KA3_8BIT, 0x0409, 0x60},
	{T4KA3_8BIT, 0x040A, 0x01},
	{T4KA3_8BIT, 0x040B, 0x0A},
	{T4KA3_8BIT, 0x040C, 0x0A},
	{T4KA3_8BIT, 0x040D, 0x10},
	{T4KA3_8BIT, 0x040E, 0x07},
	{T4KA3_8BIT, 0x040F, 0x90},
	{T4KA3_8BIT, 0x0900, 0x01},
	{T4KA3_8BIT, 0x0901, 0x11},
	{T4KA3_8BIT, 0x0902, 0x00},
	{T4KA3_8BIT, 0x4220, 0x00},
	{T4KA3_8BIT, 0x4222, 0x01},
	{T4KA3_8BIT, 0x3380, 0x01},
	{T4KA3_8BIT, 0x3090, 0x88},
	{T4KA3_8BIT, 0x3394, 0x20},
	{T4KA3_8BIT, 0x3090, 0x08},
	{T4KA3_8BIT, 0x3394, 0x10},
	{T4KA3_TOK_TERM, 0, 0 }
};
#endif

#if 0 /* unused */
static struct t4ka3_reg const t4ka3_3280x1852_30fps[] = {
	{T4KA3_8BIT, 0x0112, 0x0A},
	{T4KA3_8BIT, 0x0113, 0x0A},
	{T4KA3_8BIT, 0x0114, 0x03},
	{T4KA3_8BIT, 0x4136, 0x13},
	{T4KA3_8BIT, 0x4137, 0x33},
	{T4KA3_8BIT, 0x0820, 0x0A},
	{T4KA3_8BIT, 0x0821, 0x0D},
	{T4KA3_8BIT, 0x0822, 0x00},
	{T4KA3_8BIT, 0x0823, 0x00},
	{T4KA3_8BIT, 0x0301, 0x0A},
	{T4KA3_8BIT, 0x0303, 0x01},
	{T4KA3_8BIT, 0x0305, 0x04},
	{T4KA3_8BIT, 0x0306, 0x02},
	{T4KA3_8BIT, 0x0307, 0x18},
	{T4KA3_8BIT, 0x030B, 0x01},
	{T4KA3_8BIT, 0x034C, 0x0C},
	{T4KA3_8BIT, 0x034D, 0xD0},
	{T4KA3_8BIT, 0x034E, 0x07},
	{T4KA3_8BIT, 0x034F, 0x3A},
	{T4KA3_8BIT, 0x0340, 0x09},
	{T4KA3_8BIT, 0x0341, 0xBC},
	{T4KA3_8BIT, 0x0342, 0x0D},
	{T4KA3_8BIT, 0x0343, 0x70},
	{T4KA3_8BIT, 0x0344, 0x00},
	{T4KA3_8BIT, 0x0345, 0x00},
	{T4KA3_8BIT, 0x0346, 0x00},
	{T4KA3_8BIT, 0x0347, 0x00},
	{T4KA3_8BIT, 0x0348, 0x0C},
	{T4KA3_8BIT, 0x0349, 0xCF},
	{T4KA3_8BIT, 0x034A, 0x09},
	{T4KA3_8BIT, 0x034B, 0x9f},
	{T4KA3_8BIT, 0x0408, 0x00},
	{T4KA3_8BIT, 0x0409, 0x00},
	{T4KA3_8BIT, 0x040A, 0x01},
	{T4KA3_8BIT, 0x040B, 0x34},
	{T4KA3_8BIT, 0x040C, 0x0C},
	{T4KA3_8BIT, 0x040D, 0xD0},
	{T4KA3_8BIT, 0x040E, 0x07},
	{T4KA3_8BIT, 0x040F, 0x3C},
	{T4KA3_8BIT, 0x0900, 0x01},
	{T4KA3_8BIT, 0x0901, 0x11},
	{T4KA3_8BIT, 0x0902, 0x00},
	{T4KA3_8BIT, 0x4220, 0x00},
	{T4KA3_8BIT, 0x4222, 0x01},
	{T4KA3_8BIT, 0x3380, 0x01},
	{T4KA3_8BIT, 0x3090, 0x88},
	{T4KA3_8BIT, 0x3394, 0x20},
	{T4KA3_8BIT, 0x3090, 0x08},
	{T4KA3_8BIT, 0x3394, 0x10},
	{T4KA3_TOK_TERM, 0, 0 }
};
#endif

static struct cci_reg_sequence const t4ka3_3280x2464_30fps[] = {
	{CCI_REG8(0x0112), 0x0A},
	{CCI_REG8(0x0113), 0x0A},
	{CCI_REG8(0x0114), 0x03},
	{CCI_REG8(0x4136), 0x13},
	{CCI_REG8(0x4137), 0x33},
	{CCI_REG8(0x0820), 0x0A},
	{CCI_REG8(0x0821), 0x0D},
	{CCI_REG8(0x0822), 0x00},
	{CCI_REG8(0x0823), 0x00},
	{CCI_REG8(0x0301), 0x0A},
	{CCI_REG8(0x0303), 0x01},
	{CCI_REG8(0x0305), 0x04},
	{CCI_REG8(0x0306), 0x02},
	{CCI_REG8(0x0307), 0x18},
	{CCI_REG8(0x030B), 0x01},
	{CCI_REG8(0x034C), 0x0C}, /* horizontal output size 3280 */
	{CCI_REG8(0x034D), 0xD0},
	{CCI_REG8(0x034E), 0x09}, /* vertical output size 2662 */
	{CCI_REG8(0x034F), 0x9E},
	{CCI_REG8(0x0340), 0x09}, /* vts / lines per frame 2492 */
	{CCI_REG8(0x0341), 0xBC},
	{CCI_REG8(0x0342), 0x0D}, /* hts / pixels per line 3440 */
	{CCI_REG8(0x0343), 0x70},
	{CCI_REG8(0x0344), 0x00},
	{CCI_REG8(0x0345), 0x00},
	{CCI_REG8(0x0346), 0x00},
	{CCI_REG8(0x0347), 0x00},
	{CCI_REG8(0x0348), 0x0C}, /* horizontal end 3279 */
	{CCI_REG8(0x0349), 0xCF},
	{CCI_REG8(0x034A), 0x09}, /* vertical end 2463 */
	{CCI_REG8(0x034B), 0x9F},
	{CCI_REG8(0x0408), 0x00},
	{CCI_REG8(0x0409), 0x00},
	{CCI_REG8(0x040A), 0x00},
	{CCI_REG8(0x040B), 0x02},
	{CCI_REG8(0x040C), 0x0C},
	{CCI_REG8(0x040D), 0xD0},
	{CCI_REG8(0x040E), 0x09},
	{CCI_REG8(0x040F), 0xA0},
	{CCI_REG8(0x0900), 0x01},
	{CCI_REG8(0x0901), 0x11},
	{CCI_REG8(0x0902), 0x00},
	{CCI_REG8(0x4220), 0x00},
	{CCI_REG8(0x4222), 0x01},
	{CCI_REG8(0x3380), 0x01},
	{CCI_REG8(0x3090), 0x88},
	{CCI_REG8(0x3394), 0x20},
	{CCI_REG8(0x3090), 0x08},
	{CCI_REG8(0x3394), 0x10}
};

const struct t4ka3_resolution t4ka3_res[] = {

	{
		.regs = t4ka3_736x496_30fps,
		.regs_len = ARRAY_SIZE(t4ka3_736x496_30fps),
		.width = 736,
		.height = 496,
		.skip_frames = 2,
	},
	{
		.regs = t4ka3_896x736_30fps,
		.regs_len = ARRAY_SIZE(t4ka3_896x736_30fps),
		.width = 896,
		.height = 736,
		.skip_frames = 2,
	},
	{
		.regs = t4ka3_1936x1096_30fps,
		.regs_len = ARRAY_SIZE(t4ka3_1936x1096_30fps),
		.width = 1936,
		.height = 1096,
		.skip_frames = 2,
	},
/*	{
		.regs = t4ka3_3280x2464_30fps,
		.width = 3280,
		.height = 2464,
	}, */
};

static inline struct t4ka3_data *ctrl_to_t4ka3(struct v4l2_ctrl *ctrl)
{
	return container_of(ctrl->handler, struct t4ka3_data, ctrls.handler);
}

/* T4KA3 default GRBG */
static const int t4ka3_hv_flip_bayer_order[] = {
	MEDIA_BUS_FMT_SGRBG10_1X10,
	MEDIA_BUS_FMT_SBGGR10_1X10,
	MEDIA_BUS_FMT_SRGGB10_1X10,
	MEDIA_BUS_FMT_SGBRG10_1X10,
};

static int t4ka3_detect(struct t4ka3_data *sensor, u16 *id);

static void t4ka3_set_bayer_order(struct t4ka3_data *sensor,
				  struct v4l2_mbus_framefmt *fmt)
{
	int hv_flip = 0;

	if (sensor->ctrls.vflip && sensor->ctrls.vflip->val)
		hv_flip += 1;

	if (sensor->ctrls.hflip && sensor->ctrls.hflip->val)
		hv_flip += 2;

	fmt->code = t4ka3_hv_flip_bayer_order[hv_flip];
}

static int t4ka3_update_exposure_range(struct t4ka3_data *sensor)
{
	int exp_max = sensor->format.height + sensor->ctrls.vblank->val -
		      T4KA3_COARSE_INTEGRATION_TIME_MARGIN;

	return __v4l2_ctrl_modify_range(sensor->ctrls.exposure, 0, exp_max,
					1, exp_max);
}

static void t4ka3_fill_format(struct t4ka3_data *sensor,
			      struct v4l2_mbus_framefmt *fmt,
			      unsigned int width, unsigned int height)
{
	memset(fmt, 0, sizeof(*fmt));
	fmt->width = width;
	fmt->height = height;
	fmt->field = V4L2_FIELD_NONE;
	fmt->colorspace = V4L2_COLORSPACE_SRGB;
	t4ka3_set_bayer_order(sensor, fmt);
}

static int t4ka3_set_pad_format(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *sd_state,
				struct v4l2_subdev_format *format)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct t4ka3_data *sensor = to_t4ka3_sensor(sd);
	struct v4l2_mbus_framefmt *fmt = &format->format;
	const struct t4ka3_resolution *res;
	int def, max, ret;

	dev_info(&client->dev, "enter t4ka3_set_mbus_fmt\n");

	res = v4l2_find_nearest_size(t4ka3_res, ARRAY_SIZE(t4ka3_res),
				     width, height, fmt->width, fmt->height);
	t4ka3_fill_format(sensor, fmt, res->width, res->height);

	if (format->which == V4L2_SUBDEV_FORMAT_TRY)
		return 0;

	mutex_lock(&sensor->lock);
	sensor->res = res;
	sensor->format = *fmt;

	/* vblank range is height dependent adjust and reset to default */
	max = T4KA3_MAX_VBLANK - res->height;
	def = T4KA3_LINES_PER_FRAME - res->height;
	ret = __v4l2_ctrl_modify_range(sensor->ctrls.vblank, T4KA3_MIN_VBLANK,
				       max, 1, def);
	if (ret)
		goto unlock;

	ret = __v4l2_ctrl_s_ctrl(sensor->ctrls.vblank, def);
	if (ret)
		goto unlock;

	def = T4KA3_ACTIVE_WIDTH - res->width;
	ret = __v4l2_ctrl_modify_range(sensor->ctrls.hblank, def, def, 1, def);
	if (ret)
		goto unlock;
	ret = __v4l2_ctrl_s_ctrl(sensor->ctrls.hblank, def);
	if (ret)
		goto unlock;

	/* exposure range depends on vts which may have changed */
	ret = t4ka3_update_exposure_range(sensor);
	if (ret)
		goto unlock;

	dev_info(&client->dev, "width %d , height %d\n", res->width, res->height);

unlock:
	mutex_unlock(&sensor->lock);
	return ret;
}

/* Horizontal flip the image. */
static int t4ka3_t_hflip(struct v4l2_subdev *sd, int value)
{
	struct t4ka3_data *sensor = to_t4ka3_sensor(sd);
	int ret;
	u64 val;

	if (sensor->streaming)
		return -EBUSY;

	val = value ? T4KA3_HFLIP_BIT : 0;

	ret = cci_update_bits(sensor->regmap, T4KA3_REG_IMG_ORIENTATION,
			      T4KA3_HFLIP_BIT, val, NULL);
	if (ret)
		return ret;

	t4ka3_set_bayer_order(sensor, &sensor->format);
	return 0;
}

/* Vertically flip the image */
static int t4ka3_t_vflip(struct v4l2_subdev *sd, int value)
{
	struct t4ka3_data *sensor = to_t4ka3_sensor(sd);
	int ret;
	u64 val;

	if (sensor->streaming)
		return -EBUSY;

	val = value ? T4KA3_VFLIP_BIT : 0;

	ret = cci_update_bits(sensor->regmap, T4KA3_REG_IMG_ORIENTATION,
			      T4KA3_VFLIP_BIT, val, NULL);
	if (ret)
		return ret;

	t4ka3_set_bayer_order(sensor, &sensor->format);
	return 0;
}

static int t4ka3_test_pattern(struct t4ka3_data *sensor, s32 value)
{
	return cci_write(sensor->regmap, T4KA3_REG_TEST_PATTERN_MODE, value, NULL);
}

static int t4ka3_detect(struct t4ka3_data *sensor, u16 *id)
{
	struct i2c_client *client = v4l2_get_subdevdata(&sensor->sd);
	struct i2c_adapter *adapter = client->adapter;
	u64 high, low;

	/* i2c check */
	if (!i2c_check_functionality(adapter, I2C_FUNC_I2C))
		return -ENODEV;

	/* check sensor chip ID	 */
	if (cci_read(sensor->regmap, T4KA3_REG_PRODUCT_ID_HIGH, &high, NULL)) {
		dev_err(&client->dev, "sensor_id_high = 0x%llx\n", high);
		return -ENODEV;
	}

	if (cci_read(sensor->regmap, T4KA3_REG_PRODUCT_ID_LOW, &low, NULL)) {
		dev_err(&client->dev, "sensor_id_low = 0x%llx\n", low);
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
	struct t4ka3_data *sensor = to_t4ka3_sensor(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	u16 sensor_id;
	int ret;

	ret = pm_runtime_get_sync(sensor->sd.dev);
	if (ret) {
		dev_err(&client->dev, "t4ka3 power-up err");
		return ret;
	}

	ret = t4ka3_detect(sensor, &sensor_id);
	if (ret) {
		dev_err(&client->dev, "Failed to detect sensor.\n");
		goto fail_detect;
	}
	dev_info(&client->dev, "s_config finish\n");

fail_detect:
	pm_runtime_put(sensor->sd.dev);;
	return ret;
}

static int t4ka3_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct t4ka3_data *sensor = ctrl_to_t4ka3(ctrl);
	struct i2c_client *client = v4l2_get_subdevdata(&sensor->sd);
	int ret;

	/* Update exposure range on vblank changes */
	if (ctrl->id == V4L2_CID_VBLANK) {
		ret = t4ka3_update_exposure_range(sensor);
		if (ret)
			return ret;
	}

	/* Only apply changes to the controls if the device is powered up */
	if (!pm_runtime_get_if_in_use(sensor->sd.dev)) {
		t4ka3_set_bayer_order(sensor, &sensor->format);
		return 0;
	}

	switch (ctrl->id) {
	case V4L2_CID_TEST_PATTERN:
		dev_dbg(&client->dev, "%s: V4L2_CID_TEST_PATTERN: %d\n",
			__func__, ctrl->val);
		ret = t4ka3_test_pattern(sensor, ctrl->val);
		break;
	case V4L2_CID_VFLIP:
		dev_dbg(&client->dev, "%s: V4L2_CID_VFLIP: %d\n",
			__func__, ctrl->val);
		ret = t4ka3_t_vflip(&sensor->sd, ctrl->val);
		break;
	case V4L2_CID_HFLIP:
		dev_dbg(&client->dev, "%s: V4L2_CID_HFLIP: %d\n",
			__func__, ctrl->val);
		ret = t4ka3_t_hflip(&sensor->sd, ctrl->val);
		break;
	case V4L2_CID_VBLANK:
		dev_dbg(&client->dev, "%s: V4L2_CID_VBLANK: %d\n",
			__func__, ctrl->val);
		ret = cci_write(sensor->regmap, T4KA3_REG_FRAME_LENGTH_LINES,
				sensor->format.height + ctrl->val, NULL);
		break;
	case V4L2_CID_EXPOSURE:
		ret = cci_write(sensor->regmap, T4KA3_REG_COARSE_INTEGRATION_TIME,
				ctrl->val, NULL);
		break;
	case V4L2_CID_ANALOGUE_GAIN:
		ret = cci_write(sensor->regmap, T4KA3_REG_GLOBAL_GAIN,
				ctrl->val, NULL);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	pm_runtime_put(sensor->sd.dev);
	return ret;
}

static int t4ka3_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct t4ka3_data *sensor = to_t4ka3_sensor(sd);
	int ret;

	mutex_lock(&sensor->lock);

	if (sensor->streaming == enable) {
		dev_warn (&client->dev, "Stream aleady %s\n", enable ? "started" : "stopped");
		goto error_unlock;
	}

	if (enable) {
		dev_info (&client->dev, "power on while streaming set on t4ka3");

		ret = pm_runtime_get_sync(sensor->sd.dev);
		if (ret) {
			dev_err(&client->dev, "power-up err.\n");
			goto error_unlock;
		}

		cci_multi_reg_write(sensor->regmap, t4ka3_init_config,
				    ARRAY_SIZE(t4ka3_init_config), &ret);
		/* enable group hold */
		cci_write(sensor->regmap, T4KA3_REG_PARAM_HOLD, 1, &ret);
		cci_multi_reg_write(sensor->regmap, sensor->res->regs,
				    sensor->res->regs_len, &ret);
		if (ret)
			goto error_powerdown;

		/* Restore value of all ctrls */
		ret = __v4l2_ctrl_handler_setup(&sensor->ctrls.handler);
		if (ret)
			goto error_powerdown;

		/* disable group hold */
		cci_write(sensor->regmap, T4KA3_REG_PARAM_HOLD, 0, &ret);
		cci_write(sensor->regmap, T4KA3_REG_STREAM, 1, &ret);
		if (ret)
			goto error_powerdown;

		sensor->streaming = 1;
	} else {
		ret = cci_write(sensor->regmap, T4KA3_REG_STREAM, 0, NULL);
		if (ret)
			goto error_powerdown;

		ret = pm_runtime_put(sensor->sd.dev);
		if (ret)
			goto error_unlock;

		sensor->streaming = 0;
	}

	mutex_unlock(&sensor->lock);
	return ret;

error_powerdown:
	ret = pm_runtime_put(sensor->sd.dev);
error_unlock:
	mutex_unlock(&sensor->lock);
	return ret;
}

static int
t4ka3_enum_mbus_code(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *sd_state,
				struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index)
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

	if (index >= ARRAY_SIZE(t4ka3_res))
		return -EINVAL;

	fse->min_width = t4ka3_res[index].width;
	fse->min_height = t4ka3_res[index].height;
	fse->max_width = t4ka3_res[index].width;
	fse->max_height = t4ka3_res[index].height;

	return 0;
}

static struct v4l2_mbus_framefmt *
__t4ka3_get_pad_format(struct t4ka3_data *sensor, struct v4l2_subdev *sd,
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
	struct t4ka3_data *sensor = to_t4ka3_sensor(sd);
	struct v4l2_mbus_framefmt *format =
			__t4ka3_get_pad_format(sensor, sd, sd_state,
						fmt->pad, fmt->which);

	fmt->format = *format;

	return 0;
}

static int t4ka3_g_skip_frames(struct v4l2_subdev *sd, u32 *frames)
{
	struct t4ka3_data *sensor = to_t4ka3_sensor(sd);

	mutex_lock(&sensor->lock);
	*frames = sensor->res->skip_frames;
	mutex_unlock(&sensor->lock);

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

static const struct v4l2_subdev_pad_ops t4ka3_pad_ops = {
	.enum_mbus_code = t4ka3_enum_mbus_code,
	.enum_frame_size = t4ka3_enum_frame_size,
	.get_fmt = t4ka3_get_pad_format,
	.set_fmt = t4ka3_set_pad_format,
};

static const struct v4l2_subdev_ops t4ka3_ops = {
	.video = &t4ka3_video_ops,
	.pad = &t4ka3_pad_ops,
	.sensor = &t4ka3_sensor_ops,
};

static void t4ka3_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct t4ka3_data *sensor = to_t4ka3_sensor(sd);

	v4l2_async_unregister_subdev(&sensor->sd);
	media_entity_cleanup(&sensor->sd.entity);
	v4l2_ctrl_handler_free(&sensor->ctrls.handler);
	pm_runtime_disable(&client->dev);
}

static int t4ka3_init_controls(struct t4ka3_data *sensor)
{
	const struct v4l2_ctrl_ops *ops = &t4ka3_ctrl_ops;
	struct t4ka3_ctrls *ctrls = &sensor->ctrls;
	struct v4l2_ctrl_handler *hdl = &ctrls->handler;
	int def, max;
	static const char * const test_pattern_menu[] = {
		"Disabled",
		"Solid White",
		"Color Bars",
		"Gradient",
		"Random Data",
	};

	v4l2_ctrl_handler_init(hdl, 4);

	hdl->lock = &sensor->lock;

	ctrls->vflip = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_VFLIP, 0, 1, 1, 0);
	ctrls->hflip = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_HFLIP, 0, 1, 1, 0);

	ctrls->test_pattern = v4l2_ctrl_new_std_menu_items(hdl, ops,
					V4L2_CID_TEST_PATTERN,
					ARRAY_SIZE(test_pattern_menu) - 1,
					0, 0, test_pattern_menu);
	ctrls->link_freq = v4l2_ctrl_new_int_menu(hdl, NULL, V4L2_CID_LINK_FREQ,
						  0, 0, sensor->link_freq);

	def = T4KA3_LINES_PER_FRAME - T4KA3_ACTIVE_HEIGHT;
	max = T4KA3_MAX_VBLANK - T4KA3_ACTIVE_HEIGHT;
	ctrls->vblank = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_VBLANK,
					  T4KA3_MIN_VBLANK, max, 1, def);

	def = T4KA3_PIXELS_PER_LINE - sensor->format.width;
	ctrls->hblank = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_HBLANK,
					  def, def, 1, def);

	max = T4KA3_LINES_PER_FRAME - T4KA3_COARSE_INTEGRATION_TIME_MARGIN;
	ctrls->exposure = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_EXPOSURE,
					    0, max, 1, max);

	ctrls->gain = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_ANALOGUE_GAIN,
					T4KA3_MIN_GLOBAL_GAIN_SUPPORTED,
					T4KA3_MAX_GLOBAL_GAIN_SUPPORTED,
					1, T4KA3_MIN_GLOBAL_GAIN_SUPPORTED);

	if (hdl->error)
		return hdl->error;

	ctrls->vflip->flags |= V4L2_CTRL_FLAG_MODIFY_LAYOUT;
	ctrls->hflip->flags |= V4L2_CTRL_FLAG_MODIFY_LAYOUT;
	ctrls->link_freq->flags |= V4L2_CTRL_FLAG_READ_ONLY;
	ctrls->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	sensor->sd.ctrl_handler = hdl;
	return 0;
}

static int t4ka3_pm_suspend(struct device *dev)
{
	struct t4ka3_data *sensor = dev_get_drvdata(dev);

	gpiod_set_value_cansleep(sensor->powerdown_gpio, 1);
	gpiod_set_value_cansleep(sensor->reset_gpio, 1);

	return 0;
}

static int t4ka3_pm_resume(struct device *dev)
{
	int ret = 0;
	u16 sensor_id;
	struct t4ka3_data *sensor = dev_get_drvdata(dev);
	struct i2c_client *client = v4l2_get_subdevdata(&sensor->sd);

	usleep_range(5000, 6000);

	gpiod_set_value_cansleep(sensor->powerdown_gpio, 0);
	gpiod_set_value_cansleep(sensor->reset_gpio, 0);

	/* waiting for the sensor after powering up */
	msleep(20);

	ret = t4ka3_detect(sensor, &sensor_id);
	
	if (ret) {
		dev_err(&client->dev, "sensor detect failed\n");
		return ret;
	}

	return 0;
}

static DEFINE_RUNTIME_DEV_PM_OPS(t4ka3_pm_ops, t4ka3_pm_suspend, t4ka3_pm_resume, NULL);

static int t4ka3_probe(struct i2c_client *client)
{
	struct t4ka3_data *sensor;
	struct fwnode_handle *fwnode;
	int ret = 0;

	/*
	 * Sometimes the fwnode graph is initialized by the bridge driver.
	 * Bridge drivers doing this may also add GPIO mappings, wait for this.
	 */
	fwnode = fwnode_graph_get_next_endpoint(dev_fwnode(&client->dev), NULL);
	if (!fwnode)
		return dev_err_probe(&client->dev, -EPROBE_DEFER, "waiting for fwnode graph endpoint\n");

	fwnode_handle_put(fwnode);

	/* allocate sensor device & init sub device */
	sensor = devm_kzalloc(&client->dev, sizeof(*sensor), GFP_KERNEL);
	if (!sensor)
		return -ENOMEM;

	mutex_init(&sensor->lock);

	sensor->link_freq[0] = T4KA3_LINK_FREQ;
	sensor->res = &t4ka3_res[0];
	t4ka3_fill_format(sensor, &sensor->format, sensor->res->width, sensor->res->height);

	v4l2_i2c_subdev_init(&(sensor->sd), client, &t4ka3_ops);

	sensor->powerdown_gpio = devm_gpiod_get(&client->dev, "powerdown", GPIOD_OUT_HIGH);
	if (IS_ERR(sensor->powerdown_gpio))
		return dev_err_probe(&client->dev, PTR_ERR(sensor->powerdown_gpio), "getting powerdown GPIO\n");

	sensor->reset_gpio = devm_gpiod_get_optional(&client->dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(sensor->reset_gpio))
		return dev_err_probe(&client->dev, PTR_ERR(sensor->reset_gpio), "getting reset GPIO\n");
	
	pm_runtime_set_suspended(&client->dev);
	pm_runtime_enable(&client->dev);
	pm_runtime_set_autosuspend_delay(&client->dev, 1000);
	pm_runtime_use_autosuspend(&client->dev);

	sensor->regmap = devm_cci_regmap_init_i2c(client, 16);
	if (IS_ERR(sensor->regmap))
		return PTR_ERR(sensor->regmap);
	sensor->dev = &client->dev;

	ret = t4ka3_s_config(&sensor->sd, client->irq);
	if (ret)
		goto err_pm_runtime;

	sensor->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	sensor->pad.flags = MEDIA_PAD_FL_SOURCE;
	sensor->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;

	ret = t4ka3_init_controls(sensor);
	if (ret)
		goto err_controls;

	ret = media_entity_pads_init(&sensor->sd.entity, 1, &sensor->pad);
	if (ret)
		goto err_controls;

	ret = v4l2_async_register_subdev_sensor(&sensor->sd);
	if (ret)
		goto err_media_entity;

	return 0;

err_media_entity:
	media_entity_cleanup(&sensor->sd.entity);
err_controls:
	v4l2_ctrl_handler_free(&sensor->ctrls.handler);
err_pm_runtime:
	pm_runtime_disable(&client->dev);
	return ret;
}

static struct acpi_device_id t4ka3_acpi_match[] = {
	{ "XMCC0003" },
	{},
};
MODULE_DEVICE_TABLE(acpi, t4ka3_acpi_match);

static struct i2c_driver t4ka3_driver = {
	.driver = {
		.name = "t4ka3",
		.acpi_match_table = ACPI_PTR(t4ka3_acpi_match),
		.pm = pm_sleep_ptr(&t4ka3_pm_ops),
	},
	.probe = t4ka3_probe,
	.remove = t4ka3_remove,
};
module_i2c_driver(t4ka3_driver)

MODULE_DESCRIPTION("A low-level driver for T4KA3 sensor");
MODULE_AUTHOR("HARVEY LV <harvey.lv@intel.com>");
MODULE_LICENSE("GPL");
