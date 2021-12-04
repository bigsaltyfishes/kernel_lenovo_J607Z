/*
 * dbmdx-usecase-config-def  --  Definition of USE CASE configurations
 *
 * Copyright (C) 2014 DSP Group
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _DBMDX_USECASE_CONFIG_DEF_H
#define _DBMDX_USECASE_CONFIG_DEF_H

#include "dbmdx-interface.h"

#define NUM_OF_TDM_CONF_REGS		6
#define NUMBER_OF_TDM_CONFIGS_VA	4
#define NUMBER_OF_TDM_CONFIGS_VA_VE	8
#define NUM_OF_AUDIO_ROUTING_CONF	9
#define MAX_USECASE_FILENAME_LEN	256
#define RESERVED_OFFSET			32

#define DBMDX_USECASE_MGR_CMD_MASK			0x0000000f
#define DBMDX_USECASE_MGR_CMD_BITS			0
#define DBMDX_USECASE_MGR_MODE_MASK			0x000000f0
#define DBMDX_USECASE_MGR_MODE_BITS			4
#define DBMDX_USECASE_MGR_UID_MASK			0x000fff00
#define DBMDX_USECASE_MGR_UID_BITS			8
#define DBMDX_USECASE_MGR_MODEL_MODE_MASK		0x00f00000
#define DBMDX_USECASE_MGR_MODEL_MODE_BITS		20
#define DBMDX_USECASE_MGR_MODEL_SELECT_MASK		0x03000000
#define DBMDX_USECASE_MGR_MODEL_SELECT_BITS		24
#define DBMDX_USECASE_MGR_MODEL_OPTIONS_MASK		0x1C000000
#define DBMDX_USECASE_MGR_MODEL_OPTIONS_BITS		26
#define DBMDX_USECASE_MGR_MODEL_CUSTOM_PARAMS_MASK	0xE0000000
#define DBMDX_USECASE_MGR_MODEL_CUSTOM_PARAMS_BITS	29


#define DBMDX_USECASE_MGR_LM_MODEL_OPERATION_MASK	0x000000f0
#define DBMDX_USECASE_MGR_LM_MODEL_OPERATION_BITS	4
#define DBMDX_USECASE_MGR_LM_MODEL_TYPE_MASK		0x00000f00
#define DBMDX_USECASE_MGR_LM_MODEL_TYPE_BITS		8
#define DBMDX_USECASE_MGR_LM_MODEL_MODE_MASK		0x0000f000
#define DBMDX_USECASE_MGR_LM_MODEL_MODE_BITS		12
#define DBMDX_USECASE_MGR_LM_MODEL_OPTIONS_MASK		0x000f0000
#define DBMDX_USECASE_MGR_LM_MODEL_OPTIONS_BITS		16
#define DBMDX_USECASE_MGR_LM_MODEL_CUSTOM_PARAMS_MASK	0x00f00000
#define DBMDX_USECASE_MGR_LM_MODEL_CUSTOM_PARAMS_BITS	20
#define DBMDX_USECASE_MGR_LM_MODEL_PREFIX_MASK		0xff000000
#define DBMDX_USECASE_MGR_LM_MODEL_PREFIX_BITS		24

#define DBMDX_USECASE_MGR_LM_OP_STOP_DETECTION		0x0000
#define DBMDX_USECASE_MGR_LM_OP_START_DETECTION		0x0001
#define DBMDX_USECASE_MGR_LM_OP_LOAD_MODEL		0x0002
#define DBMDX_USECASE_MGR_LM_OP_UNLOAD_MODEL		0x0003
#define DBMDX_USECASE_MGR_LM_OP_UPDATE_PARAMS		0x0004





#define DBMDX_USECASE_MGR_CMD_SET_IDLE		0x0000
#define DBMDX_USECASE_MGR_CMD_LOAD		0x0001
#define DBMDX_USECASE_MGR_CMD_START		0x0002
#define DBMDX_USECASE_MGR_CMD_SET_MODE		0x0004
#define DBMDX_USECASE_MGR_CMD_LOAD_AMODEL	0x0008

#define DBMDX_USECASE_ID_HWREV_00		0x0000
#define DBMDX_USECASE_ID_HWREV_01		0x0001
#define DBMDX_USECASE_ID_HWREV_02		0x0002
#define DBMDX_USECASE_ID_HWREV_03		0x0003
#define DBMDX_USECASE_ID_HWREV_04		0x0004
#define DBMDX_USECASE_ID_HWREV_05		0x0005

#define DBMDX_USECASE_ID_PRJ_MANGO		0x0010
#define DBMDX_USECASE_ID_PRJ_MELON		0x0020
#define DBMDX_USECASE_ID_PRJ_ORANGE		0x0030
#define DBMDX_USECASE_ID_PRJ_KIWI		0x0040

#define DBMDX_USECASE_ID_UC_IDX_00		0x0000
#define DBMDX_USECASE_ID_UC_IDX_01		0x0100
#define DBMDX_USECASE_ID_UC_IDX_02		0x0200
#define DBMDX_USECASE_ID_UC_IDX_03		0x0300
#define DBMDX_USECASE_ID_UC_IDX_04		0x0400
#define DBMDX_USECASE_ID_UC_IDX_05		0x0500
#define DBMDX_USECASE_ID_UC_IDX_06		0x0600
#define DBMDX_USECASE_ID_UC_IDX_07		0x0700
#define DBMDX_USECASE_ID_UC_IDX_08		0x0800
#define DBMDX_USECASE_ID_UC_IDX_09		0x0900
#define DBMDX_USECASE_ID_UC_IDX_FF		0xFF00


#define DBMDX_UC_SEQ_CMD_IDLE			0x00000000
#define DBMDX_UC_SEQ_CMD_CHANGE_CLK_SRC		0x0AA00000
#define DBMDX_UC_SEQ_CMD_LOAD_ASRP_PARAMS	0x0AA10000
#define DBMDX_UC_SEQ_CMD_SET_ASRP_DELAY		0x0AA20000
#define DBMDX_UC_SEQ_CMD_SET_ASRP_GAIN		0x0AA30000
#define DBMDX_UC_SEQ_CMD_SET_AUDIO_ROUTING	0x0AA40000
#define DBMDX_UC_SEQ_CMD_CONFIG_MICS		0x0AA50000
#define DBMDX_UC_SEQ_CMD_CONFIG_TDM		0x0AA60000
#define DBMDX_UC_SEQ_CMD_SET_IOM		0x0AA70000
#define DBMDX_UC_SEQ_CMD_CONFIG_VTE		0x0AA80000
#define DBMDX_UC_SEQ_CMD_USLEEP			0x0AAA0000
#define DBMDX_UC_SEQ_CMD_MSLEEP			0x0AAB0000
#define DBMDX_UC_SEQ_CMD_COMPLEX_CLB_1		0x0AAC0000
#define DBMDX_UC_SEQ_CMD_COMPLEX_CLB_2		0x0AAD0000
#define DBMDX_UC_SEQ_CMD_COMPLEX_CLB_3		0x0AAE0000
#define DBMDX_UC_SEQ_CMD_HOOK_ON_INIT		0x0AB00000
#define DBMDX_UC_SEQ_CMD_HOOK_VA_VE_REG		0x0AB10000
#define DBMDX_UC_SEQ_CMD_HOOK_VA_REG		0x0AB20000
#define DBMDX_UC_SEQ_CMD_HOOK_PRE_TDM		0x0AB30000
#define DBMDX_UC_SEQ_CMD_HOOK_POST_TDM		0x0AB40000
#define DBMDX_UC_SEQ_CMD_HOOK_POST_TDM_VA	0x0AB50000
#define DBMDX_UC_SEQ_CMD_HOOK_POST_TDM_VA_VE	0x0AB60000
#define DBMDX_UC_SEQ_CMD_HOOK_ON_EXIT		0x0AB70000


#define DBMDX_UC_SEQ_CMD_VAL_TDM0_VA_TX		0x0001
#define DBMDX_UC_SEQ_CMD_VAL_TDM0_VA_RX		0x0002
#define DBMDX_UC_SEQ_CMD_VAL_TDM1_VA_TX		0x0003
#define DBMDX_UC_SEQ_CMD_VAL_TDM1_VA_RX		0x0004
#define DBMDX_UC_SEQ_CMD_VAL_TDM0_VA_VE_TX	0x0005
#define DBMDX_UC_SEQ_CMD_VAL_TDM0_VA_VE_RX	0x0006
#define DBMDX_UC_SEQ_CMD_VAL_TDM1_VA_VE_TX	0x0007
#define DBMDX_UC_SEQ_CMD_VAL_TDM1_VA_VE_RX	0x0008
#define DBMDX_UC_SEQ_CMD_VAL_TDM2_VA_VE_TX	0x0009
#define DBMDX_UC_SEQ_CMD_VAL_TDM2_VA_VE_RX	0x000A
#define DBMDX_UC_SEQ_CMD_VAL_TDM3_VA_VE_TX	0x000B
#define DBMDX_UC_SEQ_CMD_VAL_TDM3_VA_VE_RX	0x000C



#if HOST_HW_TDM_CONF_ASYNC_DOWN_DATA_DOWN
#define HW_TDM_POLARITY 0x0000000F
#elif HOST_HW_TDM_CONF_ASYNC_DOWN_DATA_UP
#define HW_TDM_POLARITY 0x0000000D
#elif HOST_HW_TDM_CONF_ASYNC_UP_DATA_DOWN
#define HW_TDM_POLARITY 0x00000007
#elif HOST_HW_TDM_CONF_ASYNC_UP_DATA_UP
#define HW_TDM_POLARITY 0x00000005
#else
#define HW_TDM_POLARITY 0x00000005
#endif

enum TDM_TYPE_TX_RX {
	TDM_TYPE_TX = 0,
	TDM_TYPE_RX,
};

enum TDM_INTERFACE {
	TDM_INTERFACE_VA = 0,
	TDM_INTERFACE_VA_VE,
};

struct io_register {
	u32			addr;
	u32			value;
};

enum TDM_CLK_FREQ_CONF {
	TDM_CLK_FREQ_0 = 0,
	TDM_CLK_FREQ_16 = 16000,
	TDM_CLK_FREQ_48 = 48000,
};

enum MIC_CONFIG_TYPE {
	DO_NOT_CONFIG_MICS = 0,
	MIC_CONFIG_BY_USECASE = 1,
	MIC_CONFIG_BY_USER_MASK = 2,
};

enum START_CMD_TYPE {
	START_CMD_TYPE_TDM = 0,
	START_CMD_TYPE_OPMODE = 1,
};

enum OUTPUT_GAIN_TYPE {
	ASRP_TX_OUT_GAIN = 0x1,
	ASRP_VCPF_OUT_GAIN = 0x2,
	ASRP_RX_OUT_GAIN = 0x4,
};

struct tdm_config {
	u32			tdm_index;
	enum TDM_TYPE_TX_RX	tdm_type;
	enum TDM_INTERFACE	tdm_interface;
	bool			enabled;
	u32			tdm_reg_config;
	u32			num_of_io_reg_configs;
	struct io_register	io_reg_configs[NUM_OF_TDM_CONF_REGS];
};

struct usecase_config {
	const char		*usecase_name;
	u32			id;
	u32			hw_rev;
	bool			send_va_asrp_parms;
	const char		*va_asrp_params_file_name;
	enum dbmdx_mem_loc	va_asrp_mem_loc;
	bool			send_va_ve_asrp_parms;
	const char		*va_ve_asrp_params_file_name;
	enum dbmdx_mem_loc	va_ve_asrp_mem_loc;
	enum MIC_CONFIG_TYPE	config_mics;
	u32			mic_config[4];
	u32			mic_freq;
	u32			audio_routing_config[NUM_OF_AUDIO_ROUTING_CONF];
	struct tdm_config	tdm_configs_va[NUMBER_OF_TDM_CONFIGS_VA];
	struct tdm_config	tdm_configs_va_ve[NUMBER_OF_TDM_CONFIGS_VA_VE];
	u32			*va_cfg_values;
	u32			num_of_va_cfg_values;
        u32			*va_post_tdm_cfg_values;
	u32			num_of_va_post_tdm_cfg_values;
	u32			*va_ve_cfg_values;
	u32			num_of_va_ve_cfg_values;
	bool			send_va_start_cmd;
	enum START_CMD_TYPE	va_start_cmd_type;
	u32			va_start_cmd;
	u32			va_start_i2s_buffering_cmd;
	bool			send_va_ve_start_cmd;
	enum START_CMD_TYPE	va_ve_start_cmd_type;
	u32			va_ve_start_cmd;
	bool			change_clock_src;
	enum TDM_CLK_FREQ_CONF	tdm_clock_freq;
	int			number_of_bits;
	enum dbmdx_clock_op	clock_op_va;
	enum dbmdx_clock_op	clock_op_va_ve;
	struct clock_config	clock_config_va;
	struct clock_config	clock_config_va_ve;
	bool			i2s_master_clock_va;
	bool			i2s_master_clock_va_ve;
	bool			usecase_requires_amodel;
	u32			usecase_amodel_mode;
	bool			usecase_sets_detection_mode;
	bool			usecase_supports_us_buffering;
	bool			usecase_supports_i2s_buffering;
	bool			va_chip_low_power_mode;
	bool			va_ve_chip_low_power_mode;
	enum OUTPUT_GAIN_TYPE	asrp_output_gain_type;
	u32			num_of_output_channels;
	int			(*complex_clb_1)(struct dbmdx_private *p,
						struct usecase_config *uc);
	int			(*complex_clb_2)(struct dbmdx_private *p,
						struct usecase_config *uc);
	int			(*complex_clb_3)(struct dbmdx_private *p,
						struct usecase_config *uc);
};

#endif
