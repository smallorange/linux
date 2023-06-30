/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Support for Intel Camera Imaging ISP subsystem.
 * Copyright (c) 2015, Intel Corporation.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */

#ifndef __INPUT_SYSTEM_LOCAL_H_INCLUDED__
#define __INPUT_SYSTEM_LOCAL_H_INCLUDED__

#include "csi_rx.h"
#include "pixelgen.h"
#include "isys_stream2mmio.h"
#include "isys_irq.h"

typedef enum {
	MIPI_FORMAT_2401_SHORT1 = 0x08,
	MIPI_FORMAT_2401_SHORT2,
	MIPI_FORMAT_2401_SHORT3,
	MIPI_FORMAT_2401_SHORT4,
	MIPI_FORMAT_2401_SHORT5,
	MIPI_FORMAT_2401_SHORT6,
	MIPI_FORMAT_2401_SHORT7,
	MIPI_FORMAT_2401_SHORT8,
	MIPI_FORMAT_2401_EMBEDDED = 0x12,
	MIPI_FORMAT_2401_YUV420_8 = 0x18,
	MIPI_FORMAT_2401_YUV420_10,
	MIPI_FORMAT_2401_YUV420_8_LEGACY,
	MIPI_FORMAT_2401_YUV420_8_SHIFT = 0x1C,
	MIPI_FORMAT_2401_YUV420_10_SHIFT,
	MIPI_FORMAT_2401_YUV422_8 = 0x1E,
	MIPI_FORMAT_2401_YUV422_10,
	MIPI_FORMAT_2401_RGB444 = 0x20,
	MIPI_FORMAT_2401_RGB555,
	MIPI_FORMAT_2401_RGB565,
	MIPI_FORMAT_2401_RGB666,
	MIPI_FORMAT_2401_RGB888,
	MIPI_FORMAT_2401_RAW6 = 0x28,
	MIPI_FORMAT_2401_RAW7,
	MIPI_FORMAT_2401_RAW8,
	MIPI_FORMAT_2401_RAW10,
	MIPI_FORMAT_2401_RAW12,
	MIPI_FORMAT_2401_RAW14,
	MIPI_FORMAT_2401_CUSTOM0 = 0x30,
	MIPI_FORMAT_2401_CUSTOM1,
	MIPI_FORMAT_2401_CUSTOM2,
	MIPI_FORMAT_2401_CUSTOM3,
	MIPI_FORMAT_2401_CUSTOM4,
	MIPI_FORMAT_2401_CUSTOM5,
	MIPI_FORMAT_2401_CUSTOM6,
	MIPI_FORMAT_2401_CUSTOM7,
	//MIPI_FORMAT_RAW16, /*not supported by 2401*/
	//MIPI_FORMAT_RAW18,
	N_MIPI_FORMAT_2401
} mipi_format_2401_t;

#define N_MIPI_FORMAT_CUSTOM	8

/* The number of stores for compressed format types */
#define	N_MIPI_COMPRESSOR_CONTEXT	(N_RX_CHANNEL_ID * N_MIPI_FORMAT_CUSTOM)
typedef struct input_system_state_s	input_system_state_t;
struct input_system_state_s {
	ibuf_ctrl_state_t	ibuf_ctrl_state[N_IBUF_CTRL_ID];
	csi_rx_fe_ctrl_state_t	csi_rx_fe_ctrl_state[N_CSI_RX_FRONTEND_ID];
	csi_rx_be_ctrl_state_t	csi_rx_be_ctrl_state[N_CSI_RX_BACKEND_ID];
	pixelgen_ctrl_state_t	pixelgen_ctrl_state[N_PIXELGEN_ID];
	stream2mmio_state_t	stream2mmio_state[N_STREAM2MMIO_ID];
	isys_irqc_state_t	isys_irqc_state[N_ISYS_IRQ_ID];
};
#endif /* __INPUT_SYSTEM_LOCAL_H_INCLUDED__ */
