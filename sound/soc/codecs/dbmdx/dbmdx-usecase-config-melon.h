/*
 * dbmdx-usecase-config-melon.h  --  Preset USE CASE configurations
 *
 * Copyright (C) 2014 DSP Group
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#ifdef DBMDX_MELON_USECASES_SUPPORTED

#ifndef _DBMDX_USECASE_CONFIG_MELON_H
#define _DBMDX_USECASE_CONFIG_MELON_H

#include "dbmdx-interface.h"
#include "dbmdx-usecase-config-def.h"

#define DBMDX_MELON_OPTIMIZED 1

int melon_uc_lp_model_config(struct dbmdx_private *p,
				struct usecase_config *uc);

int melon_uc_nr_model_config(struct dbmdx_private *p,
				struct usecase_config *uc);
int uc_load_models_general(struct dbmdx_private *p,
					struct usecase_config *uc);

int uc_load_models_unload_all(struct dbmdx_private *p,
					struct usecase_config *uc);

int uc_stop_all_models(struct dbmdx_private *p,
					struct usecase_config *uc);

static struct usecase_config config_uc_melon_idle = {
	.usecase_name = "uc_melon_idle",
	.id	= (DBMDX_USECASE_ID_UC_IDX_FF |
			DBMDX_USECASE_ID_PRJ_MELON |
			DBMDX_USECASE_ID_HWREV_00),
	.hw_rev = 0,
	.send_va_asrp_parms = false,
	.va_asrp_params_file_name = "",
	.send_va_ve_asrp_parms = false,
	.va_ve_asrp_params_file_name = "",
	.change_clock_src = true,
	.tdm_clock_freq = TDM_CLK_FREQ_48,
	.number_of_bits = 16,
	.clock_op_va = DBMDX_CLOCK_OP_SWITCH_TO_MASTER_CLOCK,
	.clock_config_va = {
			.wanted_pll = 0,
			.wanted_tl3_clk = 73000000,
			.wanted_ahb_clk = 0,
			.wanted_apb_clk = 0,
			.use_pll_post_div = false,
		},
	.usecase_requires_amodel = false,
	.usecase_sets_detection_mode = false,
	.usecase_supports_us_buffering = false,
	.va_chip_low_power_mode = false,
	.va_ve_chip_low_power_mode = false,
	.asrp_output_gain_type = 0,

	.num_of_va_cfg_values = 0,
	.num_of_va_ve_cfg_values = 0,
	.va_start_cmd_type = START_CMD_TYPE_OPMODE,
	.va_start_cmd = (DBMDX_IDLE),
	.va_ve_start_cmd_type = START_CMD_TYPE_OPMODE,
	.send_va_ve_start_cmd = false,
	.va_ve_start_cmd = (DBMDX_IDLE),
};




/* Melon Low Power Mode Usecase */
static struct usecase_config config_uc_melon_low_power = {
	.usecase_name = "uc_melon_low_power",
	.id	= (DBMDX_USECASE_ID_UC_IDX_00 |
			DBMDX_USECASE_ID_PRJ_MELON |
			DBMDX_USECASE_ID_HWREV_00),
	.hw_rev = 0,
	.send_va_asrp_parms = false,
	.va_asrp_params_file_name = "",
	.send_va_ve_asrp_parms = false,
	.va_ve_asrp_params_file_name = "",
	.change_clock_src = true,
	.tdm_clock_freq = TDM_CLK_FREQ_48,
	.number_of_bits = 16,
	.clock_op_va = DBMDX_CLOCK_OP_SWITCH_TO_MASTER_CLOCK,
	.clock_config_va = {
			.wanted_pll = 0,
#ifndef DBMDX_MELON_OPTIMIZED
			.wanted_tl3_clk = 98000000,
#else
			.wanted_tl3_clk = 27000000,
#endif
			.wanted_ahb_clk = 0,
			.wanted_apb_clk = 0,
			.use_pll_post_div = false,
		},
	.usecase_requires_amodel = true,
	.usecase_amodel_mode = 1,
	.usecase_sets_detection_mode = true,
	.usecase_supports_us_buffering = true,
	.va_chip_low_power_mode = false,
	.va_ve_chip_low_power_mode = false,
	.asrp_output_gain_type = 0,
	.complex_clb_1 = uc_load_models_general,
	.complex_clb_2 = melon_uc_lp_model_config,


	.va_cfg_values = (u32 []){
		(DBMDX_UC_SEQ_CMD_COMPLEX_CLB_1),
		(DBMDX_REGN_UART_SPEED | DBMDX_REGV_UART_BAUD_RATE_1_Mhz),
		(DBMDX_UC_SEQ_CMD_CHANGE_CLK_SRC),
		(DBMDX_UC_SEQ_CMD_COMPLEX_CLB_2),
		(DBMDX_REGN_BUFFERING_NORMAL_AMPLITUDE |
			DBMDX_REGV_USE_PHRASE_LEN_FROM_WWE |
			DBMDX_REGV_NORMALIZE_TO_MINUS_6dB),
		(DBMDX_REGN_AUDIO_STREAMING_SRC_SELECT |
			DBMDX_REGV_NO_STREAM_CH_4 |
			DBMDX_REGV_NO_STREAM_CH_3 |
			DBMDX_REGV_NO_STREAM_CH_2 |
			DBMDX_REGV_STREAM_CH_1_CP_0),
		(DBMDX_UC_SEQ_CMD_CONFIG_MICS),
#ifndef DBMDX_MIC_TYPE_IS_DIGITAL
		(DBMDX_REGN_MICROPHONE_ANALOG_GAIN |
			DBMDX_REGV_SAR_GAIN_EN |
			DBMDX_REGV_SAR_GAIN_31_8_DB),
#else
		(DBMDX_REGN_DIGITAL_GAIN |
			DBMDX_REGV_GAIN_AFFECTS_ALL_MICS |
			DBMDX_REGV_DIGITAL_GAIN_3_DB),
#endif
	},
	.num_of_va_cfg_values = 8,
	.config_mics = MIC_CONFIG_BY_USECASE,

	.mic_config = {
#ifdef DBMDX_MIC_TYPE_IS_DIGITAL
		(DBMDX_REGV_DM_CLOCK_SRC_DM0_GPIO8_DM1_GPIO14 |
		 DBMDX_REGV_DM_DATA_SRC_DM0_GPIO9_DM1_GPIO13 |
		 DBMDX_REGV_DDF_AUDIO_ATTENUATION_0dB |
		 DBMDX_REGV_DM_CLK_FREQ_1536_1200_SR_8KHz_16KHz_32KHz_48KHz |
		 DBMDX_REGV_CLOCK_POLARITY_FALLING_EDGE |
		 DBMDX_REGV_DDF_AND_DM_CONFIG_DDF0_DM0),
#else
		(DBMDX_REGV_DDF_AUDIO_ATTENUATION_0dB |
		 DBMDX_REGV_SAR_IIR_FILTER_128 |
		 DBMDX_REGV_SAR_ADC_IN_SEL_MIC2 |
		 DBMDX_REGV_DDF_AND_DM_CONFIG_SAR_DDF_SAR_ADC),
#endif
		0x0000,
		0x0000,
		0x0000},
#ifdef DBMDX_MIC_TYPE_IS_DIGITAL
	.mic_freq = 1536000,
#else
	.mic_freq = 384000,
#endif
	.num_of_va_ve_cfg_values = 0,

	.audio_routing_config = {
		DBMDX_UNDEFINED_REGISTER,
		DBMDX_UNDEFINED_REGISTER,
		DBMDX_UNDEFINED_REGISTER,
		DBMDX_UNDEFINED_REGISTER,
		DBMDX_UNDEFINED_REGISTER,
		DBMDX_UNDEFINED_REGISTER,
		DBMDX_UNDEFINED_REGISTER,
		DBMDX_UNDEFINED_REGISTER,
		DBMDX_UNDEFINED_REGISTER,},

	.tdm_configs_va = {
		/* DBMD4 TDM0_TX is disabled */
		{	.tdm_index	= 0x0,
			.tdm_type	= TDM_TYPE_TX,
			.tdm_interface	= TDM_INTERFACE_VA,
			.enabled	= false,
		},
		/* DBMD4 TDM0_RX is disabled */
		{	.tdm_index	= 0x0,
			.tdm_type	= TDM_TYPE_RX,
			.tdm_interface	= TDM_INTERFACE_VA,
			.enabled	= false,
		},
		/* DBMD4 TDM1_TX is disabled */
		{	.tdm_index	= 0x1,
			.tdm_type	= TDM_TYPE_TX,
			.tdm_interface	= TDM_INTERFACE_VA,
			.enabled	= false,
		},
		/* DBMD4 TDM1_RX is disabled */
		{	.tdm_index	= 0x0,
			.tdm_type	= TDM_TYPE_RX,
			.tdm_interface	= TDM_INTERFACE_VA,
			.enabled	= false,
		},

	},
	.va_start_cmd_type = START_CMD_TYPE_OPMODE,
	.send_va_start_cmd = true,
	.va_start_cmd = (DBMDX_DETECTION),

	.va_ve_start_cmd_type = START_CMD_TYPE_OPMODE,
	.send_va_ve_start_cmd = false,
	.va_ve_start_cmd = (DBMDX_IDLE),
};


/* Melon NR Dual Mic Usecase*/
static struct usecase_config config_uc_melon_nr_dual_mic = {
#if defined(DBMDX_I2S_BUFFERING_SUPPORTED)
#if defined(DBMDX_MELON_SRATE_48000)
	.usecase_name = "uc_melon_nr_dual_mic_48k",
#else
	.usecase_name = "uc_melon_nr_dual_mic_16k",
#endif
#else
	.usecase_name = "uc_melon_nr_dual_mic",
#endif
	.id	= (DBMDX_USECASE_ID_UC_IDX_01 |
			DBMDX_USECASE_ID_PRJ_MELON |
			DBMDX_USECASE_ID_HWREV_00),
	.hw_rev = 0,
	.send_va_asrp_parms = true,
	.va_asrp_params_file_name = "asrp_params_melon_nr.bin",
	.va_asrp_mem_loc = DBMDX_MEM_LOC_DCTM,
	.send_va_ve_asrp_parms = false,
	.va_ve_asrp_params_file_name = NULL,
	.change_clock_src = true,
#if defined(DBMDX_I2S_BUFFERING_SUPPORTED) && !defined(VA_I2S_MASTER)
	.clock_op_va = DBMDX_CLOCK_OP_SWITCH_TO_TDM_CLOCK,
#else
	.clock_op_va = DBMDX_CLOCK_OP_SWITCH_TO_MASTER_CLOCK,
#endif

#if defined(DBMDX_MELON_SRATE_48000)
	.tdm_clock_freq = TDM_CLK_FREQ_48,
#else
	.tdm_clock_freq = TDM_CLK_FREQ_16,
#endif

#ifdef AEC_REF_32_TO_16_BIT
	.number_of_bits = 32,
#else
	.number_of_bits = 16,
#endif
#ifdef VA_I2S_MASTER
	.i2s_master_clock_va = true,
#endif
	.clock_config_va = {
			.wanted_pll = 0,
#ifndef DBMDX_MELON_OPTIMIZED
			.wanted_tl3_clk = 124000000,
#elif defined(DBMDX_I2S_BUFFERING_SUPPORTED) && !defined(VA_I2S_MASTER)
			.wanted_tl3_clk = 55000000,
#else
			.wanted_tl3_clk = 61000000,
#endif
			.wanted_ahb_clk = 0,
			.wanted_apb_clk = 0,
			.use_pll_post_div = false,
		},

	.usecase_requires_amodel = true,
	.usecase_amodel_mode = 1,
	.usecase_sets_detection_mode = true,
	.usecase_supports_us_buffering = true,
	.va_chip_low_power_mode = false,
	.va_ve_chip_low_power_mode = false,
	.asrp_output_gain_type = (ASRP_TX_OUT_GAIN),
	.complex_clb_1 = uc_load_models_general,
	.complex_clb_2 = melon_uc_nr_model_config,

	.va_cfg_values = (u32 []){
		(DBMDX_UC_SEQ_CMD_COMPLEX_CLB_1),
		(DBMDX_UC_SEQ_CMD_CHANGE_CLK_SRC),
		(DBMDX_UC_SEQ_CMD_LOAD_ASRP_PARAMS),
		(DBMDX_REGN_ASRP_OUTPUT_ROUTING |
			DBMDX_REGV_ASRP_OUTPUT_SRC_BFPF_1 |
			DBMDX_REGV_ASRP_OUTPUT_DEST_TX_1),
		(DBMDX_UC_SEQ_CMD_SET_ASRP_GAIN),
		(DBMDX_REGN_LOAD_NEW_ACOUSTIC_MODEL |
			DBMDX_REGV_LOAD_ENGINE_TYPE_ASRP |
			DBMDX_REGV_OP_TYPE_LOAD_FILE |
			DBMDX_REGV_INIT_KILL_SET |
			DBMDX_REGV_BLK_START_NUM_0 |
			DBMDX_REGV_LOAD_MODEL_TO_DTCM |
			DBMDX_REGV_ASRP_2ND_INIT),
		(DBMDX_REGN_AUDIO_PROCESSING_CONFIG |
#ifdef DBMDX_I2S_BUFFERING_SUPPORTED
			DBMDX_REGV_EN_COPY_HIST |
#endif /* DBMDX_I2S_BUFFERING_SUPPORTED */
			DBMDX_REGV_POST_DET_MODE_SWITCH_TO_STREAMING |
			DBMDX_REGV_ALGO1_EN_FW_MODE_1_AND_MODE_2),
		(DBMDX_UC_SEQ_CMD_COMPLEX_CLB_2),
		(DBMDX_REGN_BUFFERING_NORMAL_AMPLITUDE |
			DBMDX_REGV_USE_PHRASE_LEN_FROM_WWE |
			DBMDX_REGV_NORMALIZE_TO_MINUS_6dB),
		(DBMDX_UC_SEQ_CMD_SET_AUDIO_ROUTING),
		(DBMDX_REGN_AUDIO_STREAMING_SRC_SELECT |
			DBMDX_REGV_NO_STREAM_CH_4 |
			DBMDX_REGV_NO_STREAM_CH_3 |
			DBMDX_REGV_NO_STREAM_CH_2 |
			DBMDX_REGV_STREAM_CH_1_CP_0),
		(DBMDX_UC_SEQ_CMD_CONFIG_MICS),
#ifndef DBMDX_MIC_TYPE_IS_DIGITAL
		(DBMDX_REGN_MICROPHONE_ANALOG_GAIN |
			DBMDX_REGV_SD_GAIN_EN |
			DBMDX_REGV_SD_GAIN_32_DB |
			DBMDX_REGV_SAR_GAIN_EN |
			DBMDX_REGV_SAR_GAIN_31_8_DB),
#else
		(DBMDX_REGN_DIGITAL_GAIN |
			DBMDX_REGV_GAIN_AFFECTS_ALL_MICS |
			DBMDX_REGV_DIGITAL_GAIN_3_DB),
#endif
	},

	.num_of_va_cfg_values = 13,

#if defined(DBMDX_I2S_BUFFERING_SUPPORTED) && !defined(VA_I2S_MASTER)
	.va_post_tdm_cfg_values = (u32 []){
		(DBMDX_REGN_AUDIO_ROUTING_CONFIG |
#if defined(DBMDX_MELON_SRATE_48000) && defined(AEC_REF_32_TO_16_BIT)
			DBMDX_REGV_TDM_SYNC_DELAY_6_CLKS_CYCLES |
#else
			DBMDX_REGV_TDM_SYNC_DELAY_4_CLKS_CYCLES |
#endif
			DBMDX_REGV_TDM_SYNC_RIGHT_CH |
			DBMDX_REGV_USE_TDM_MUSIC_TO_SYNC |
			DBMDX_REGV_MUSIC_IN_TDM0) },
	.num_of_va_post_tdm_cfg_values = 1,
	.usecase_supports_i2s_buffering = true,
#endif /* DBMDX_I2S_BUFFERING_SUPPORTED && !VA_I2S_MASTER */

	.config_mics = MIC_CONFIG_BY_USECASE,
	.mic_config = {
#ifdef DBMDX_MIC_TYPE_IS_DIGITAL
		(DBMDX_REGV_DM_CLOCK_SRC_DM0_GPIO8_DM1_GPIO14 |
		 DBMDX_REGV_DM_DATA_SRC_DM0_GPIO9_DM1_GPIO13 |
		 DBMDX_REGV_DDF_AUDIO_ATTENUATION_0dB |
		 DBMDX_REGV_SYNCED_START |
		 DBMDX_REGV_DM_CLK_FREQ_1536_1200_SR_8KHz_16KHz_32KHz_48KHz |
		 DBMDX_REGV_CLOCK_POLARITY_FALLING_EDGE |
		 DBMDX_REGV_DDF_AND_DM_CONFIG_DDF0_DM0),
		(DBMDX_REGV_DM_CLOCK_SRC_DM0_GPIO8_DM1_GPIO14 |
		 DBMDX_REGV_DM_DATA_SRC_DM0_GPIO9_DM1_GPIO13 |
		 DBMDX_REGV_DDF_AUDIO_ATTENUATION_0dB |
		 DBMDX_REGV_DM_CLK_FREQ_1536_1200_SR_8KHz_16KHz_32KHz_48KHz |
		 DBMDX_REGV_CLOCK_POLARITY_RISING_EDGE |
		 DBMDX_REGV_DDF_AND_DM_CONFIG_DDF1_DM0),

#else
		(DBMDX_REGV_SYNCED_START |
		 DBMDX_REGV_DDF_AUDIO_ATTENUATION_0dB |
		 DBMDX_REGV_SAR_IIR_FILTER_SD_ALIGN_D8D |
		 DBMDX_REGV_DDF_AND_DM_CONFIG_SAR_DDF_SAR_ADC),
		(DBMDX_REGV_DDF_AUDIO_ATTENUATION_0dB |
		 DBMDX_REGV_DM_CLK_FREQ_1536_1200_SR_8KHz_16KHz_32KHz_48KHz |
		 DBMDX_REGV_DDF_AND_DM_CONFIG_SD_DDF_SD_ADC),
#endif
		 0x0000,
		 0x0000 },
#ifdef DBMDX_MIC_TYPE_IS_DIGITAL
	.mic_freq = 1536000,
#else
	.mic_freq = 384000,
#endif
	.num_of_va_ve_cfg_values = 0,

	.audio_routing_config = {
		(DBMDX_REGV_IO_SET_0 |
			DBMDX_REGV_IO_3N_2_NO_CP |
			DBMDX_REGV_IO_3N_1_CP_1 |
			DBMDX_REGV_IO_3N_0_CP_0),
		(DBMDX_REGV_IO_SET_1 |
			DBMDX_REGV_IO_3N_2_NO_CP |
			DBMDX_REGV_IO_3N_1_NO_CP |
			DBMDX_REGV_IO_3N_0_NO_CP),
		(DBMDX_REGV_IO_SET_2 |
			DBMDX_REGV_IO_3N_2_NO_CP |
			DBMDX_REGV_IO_3N_1_NO_CP |
			DBMDX_REGV_IO_3N_0_NO_CP),
		(DBMDX_REGV_IO_SET_3 |
			DBMDX_REGV_IO_3N_2_NO_CP |
			DBMDX_REGV_IO_3N_1_NO_CP |
			DBMDX_REGV_IO_3N_0_NO_CP),
		(DBMDX_REGV_IO_SET_4 |
			DBMDX_REGV_IO_3N_2_NO_CP |
			DBMDX_REGV_IO_3N_1_NO_CP |
			DBMDX_REGV_IO_3N_0_NO_CP),
		(DBMDX_REGV_IO_SET_5 |
			DBMDX_REGV_IO_3N_2_NO_CP |
			DBMDX_REGV_IO_3N_1_NO_CP |
			DBMDX_REGV_IO_3N_0_CP_0),
		(DBMDX_REGV_IO_SET_6 |
			DBMDX_REGV_IO_3N_2_NO_CP |
			DBMDX_REGV_IO_3N_1_NO_CP |
			DBMDX_REGV_IO_3N_0_NO_CP),
		(DBMDX_REGV_IO_SET_7 |
			DBMDX_REGV_IO_3N_2_NO_CP |
			DBMDX_REGV_IO_3N_1_NO_CP |
			DBMDX_REGV_IO_3N_0_NO_CP),
		DBMDX_UNDEFINED_REGISTER },

	.tdm_configs_va = {
		/* DBMD4 TDM0_TX is connected to Host Codec */
		{	.tdm_index	= 0x0,
			.tdm_type	= TDM_TYPE_TX,
			.tdm_interface	= TDM_INTERFACE_VA,
#ifdef DBMDX_I2S_BUFFERING_SUPPORTED
			.enabled	= true,
#else
			.enabled	= false,
#endif
			.tdm_reg_config	=
				(DBMDX_REGV_DEMUX_MUX_DISABLE |
				DBMDX_REGV_SAMPLE_WIDTH_16_BIT |
				DBMDX_REGV_NUM_OF_CHANNELS_1_CH |
#if defined(DBMDX_MELON_SRATE_48000)
				DBMDX_REGV_INPUT_OUTPUT_SAMPLE_RATE_48_KHz |
#else
				DBMDX_REGV_INPUT_OUTPUT_SAMPLE_RATE_16_KHz |
#endif
		DBMDX_REGV_RX_TX_I2S_CH_SUM_HIGH_AND_LOW_INTO_ONE_SAMPLE |
#if defined(DBMDX_MELON_SRATE_48000) && defined(DBMDX_I2S_CATCHUP_SUPPORTED)
				DBMDX_REGV_RESAMPLE_RATIO_NO_RESAMPLING |
				DBMDX_REGV_RESAMPLE_TYPE_INTERPOLATION |
#elif defined(DBMDX_MELON_SRATE_48000)
				DBMDX_REGV_RESAMPLE_RATIO_3_1 |
				DBMDX_REGV_RESAMPLE_TYPE_INTERPOLATION |
#else
				DBMDX_REGV_RESAMPLE_RATIO_NO_RESAMPLING |
				DBMDX_REGV_RESAMPLE_TYPE_DECIMATION |
#endif
				DBMDX_REGV_HW_BLOCK_EN |
				DBMDX_REGV_RX_TX_CP3),

			.num_of_io_reg_configs = 4,
#ifdef AEC_REF_32_TO_16_BIT
			.io_reg_configs	= {
				{		.addr = DBMD4_TDM_0_TX_ADDR,
#ifdef VA_I2S_MASTER
						.value = 0x00804052
#else
						.value = 0x00804053
#endif
				},
				{		.addr = DBMD4_TDM_0_TX_ADDR + 4,
#ifdef VA_I2S_MASTER
						.value = 0x00641064
#else
						.value = 0x00000064
#endif


				},
				{		.addr = DBMD4_TDM_0_TX_ADDR + 6,
						.value = 0x101F003F
				},
				{		.addr = DBMD4_TDM_0_TX_ADDR+0xA,
#ifdef VA_I2S_MASTER
						.value = 0x00000055
#else
						.value = 0x00000005
#endif

				}
			},
#else	/* AEC_REF_32_TO_16_BIT */
			.io_reg_configs	= {
				{		.addr = DBMD4_TDM_0_TX_ADDR,
#ifdef VA_I2S_MASTER
						.value = 0x00804052
#else
						.value = 0x00804053
#endif
				},
				{		.addr = DBMD4_TDM_0_TX_ADDR + 4,
						.value = 0x00241024
				},
				{		.addr = DBMD4_TDM_0_TX_ADDR + 6,
						.value = 0x100F001F
				},
				{		.addr = DBMD4_TDM_0_TX_ADDR+0xA,
						.value = 0x0000000F
				}
			},
#endif /* AEC_REF_32_TO_16_BIT */
		},
		/* DBMD4 TDM0_RX is disabled */
		{	.tdm_index	= 0x0,
			.tdm_type	= TDM_TYPE_RX,
			.tdm_interface	= TDM_INTERFACE_VA,
			.enabled	= false,
		},
		/* DBMD4 TDM1_TX is disabled */
		{	.tdm_index	= 0x1,
			.tdm_type	= TDM_TYPE_TX,
			.tdm_interface	= TDM_INTERFACE_VA,
			.enabled	= false,
		},
		/* DBMD4 TDM1_RX is disabled */
		{	.tdm_index	= 0x0,
			.tdm_type	= TDM_TYPE_RX,
			.tdm_interface	= TDM_INTERFACE_VA,
			.enabled	= false,
		},

	},
	.va_start_cmd_type = START_CMD_TYPE_OPMODE,
	.send_va_start_cmd = true,
	.va_start_cmd = (DBMDX_DETECTION),
#ifdef DBMDX_I2S_BUFFERING_SUPPORTED
	.va_start_i2s_buffering_cmd = (DBMDX_REGV_TDM0_TX_EN |
				DBMDX_REGV_PROC_EN_SWITCH_FW_TO_BUFFERING_MODE),
#endif
	.send_va_ve_start_cmd = false,
};

/***********************************************************************/


#ifdef OKG_2_CH_AEC_ONLY_AUDIO_BUF
#define DBMDX_MELON_BARGEIN_GA_MODE
#include "dbmdx-usecase-config-melon-bargein.h"
#undef DBMDX_MELON_BARGEIN_GA_MODE
#endif

#define DBMDX_MELON_BARGEIN_1MIC
#include "dbmdx-usecase-config-melon-bargein.h"
#undef DBMDX_MELON_BARGEIN_1MIC

#define DBMDX_MELON_BARGEIN_2MIC
#include "dbmdx-usecase-config-melon-bargein.h"
#undef DBMDX_MELON_BARGEIN_2MIC

#if defined(OKG_2_CH_AEC_ONLY_AUDIO_BUF)
#if !defined(DBMDX_MELON_BARGEIN_GA_MODE)
#define DBMDX_MELON_BARGEIN_GA_MODE 1
#endif
#endif
/* Melon VC Usecase */
static struct usecase_config config_uc_melon_vc = {
#if defined(DBMDX_MELON_SRATE_48000)
	.usecase_name = "uc_melon_vc_1mic_48k",
#else
	.usecase_name = "uc_melon_vc_1mic_16k",
#endif
	.id	= (DBMDX_USECASE_ID_UC_IDX_07 |
			DBMDX_USECASE_ID_PRJ_MELON |
			DBMDX_USECASE_ID_HWREV_00),
#ifdef VC_RX_PROCESS_SUPPORTED
	.va_asrp_params_file_name = "asrp_params_melon_vc_rx_proc.bin",
#else
	.va_asrp_params_file_name = "asrp_params_melon_vc.bin",
#endif
	.hw_rev = 0,
	.send_va_asrp_parms = true,
	.va_asrp_mem_loc = DBMDX_MEM_LOC_DCTM,
	.send_va_ve_asrp_parms = false,
	.va_ve_asrp_params_file_name = NULL,
	.change_clock_src = true,
#ifdef VA_I2S_MASTER
	.clock_op_va = DBMDX_CLOCK_OP_SWITCH_TO_MASTER_CLOCK,
#else
	.clock_op_va = DBMDX_CLOCK_OP_SWITCH_TO_TDM_CLOCK,
#endif
#if defined(DBMDX_MELON_SRATE_48000)
	.tdm_clock_freq = TDM_CLK_FREQ_48,
#else
	.tdm_clock_freq = TDM_CLK_FREQ_16,
#endif
#ifdef AEC_REF_32_TO_16_BIT
	.number_of_bits = 32,
#else
	.number_of_bits = 16,
#endif
#ifdef VA_I2S_MASTER
	.i2s_master_clock_va = true,
#endif
	.clock_config_va = {
			.wanted_pll = 0,
#ifndef DBMDX_MELON_OPTIMIZED
			.wanted_tl3_clk = 124000000,
#else /* Optimized */
#if defined(DBMDX_MELON_SRATE_48000) && defined(VC_RX_PROCESS_SUPPORTED)
			.wanted_tl3_clk = 120000000,
#elif defined(DBMDX_MELON_SRATE_48000)
			.wanted_tl3_clk = 110000000,
#else
			.wanted_tl3_clk = 98000000,
#endif
#endif
			.wanted_ahb_clk = 0,
			.wanted_apb_clk = 0,
			.use_pll_post_div = false,
		},

	.usecase_requires_amodel = false,
	.usecase_amodel_mode = 1,
	.usecase_sets_detection_mode = true,
	.usecase_supports_us_buffering = true,
	.va_chip_low_power_mode = false,
	.va_ve_chip_low_power_mode = false,
	.asrp_output_gain_type = (ASRP_VCPF_OUT_GAIN | ASRP_RX_OUT_GAIN),
	.complex_clb_1 = uc_load_models_unload_all,
	.complex_clb_2 = uc_stop_all_models,

	.va_cfg_values = (u32 []){
		(DBMDX_UC_SEQ_CMD_COMPLEX_CLB_1),
		(DBMDX_UC_SEQ_CMD_CHANGE_CLK_SRC),
		(DBMDX_UC_SEQ_CMD_COMPLEX_CLB_2),
		(DBMDX_UC_SEQ_CMD_LOAD_ASRP_PARAMS),
		(DBMDX_UC_SEQ_CMD_CONFIG_TDM |
			DBMDX_UC_SEQ_CMD_VAL_TDM0_VA_RX),
		(DBMDX_REGN_LOAD_NEW_ACOUSTIC_MODEL |
			DBMDX_REGV_LOAD_ENGINE_TYPE_ASRP |
			DBMDX_REGV_OP_TYPE_LOAD_FILE |
			DBMDX_REGV_INIT_KILL_SET |
			DBMDX_REGV_BLK_START_NUM_0 |
			DBMDX_REGV_LOAD_MODEL_TO_DTCM |
			DBMDX_REGV_ASRP_2ND_INIT),
		(DBMDX_UC_SEQ_CMD_SET_ASRP_DELAY),
		(DBMDX_REGN_ASRP_OUTPUT_ROUTING |
			DBMDX_REGV_ASRP_OUTPUT_SRC_VCPF_1 |
			DBMDX_REGV_ASRP_OUTPUT_DEST_TX_1),
		(DBMDX_UC_SEQ_CMD_SET_ASRP_GAIN),
		(DBMDX_REGN_AUDIO_PROCESSING_CONFIG |
			DBMDX_REGV_ALGO1_EN_FW_MODE_2_ONLY),
		(DBMDX_VA_USLEEP | 0x1388),
		(DBMDX_UC_SEQ_CMD_CONFIG_TDM |
			DBMDX_UC_SEQ_CMD_VAL_TDM0_VA_TX),
#if !defined(VA_I2S_MASTER)
		(DBMDX_REGN_AUDIO_ROUTING_CONFIG |
#ifdef DBMDX_MELON_OPTIMIZED
#if defined(DBMDX_MELON_SRATE_48000) && defined(AEC_REF_32_TO_16_BIT)
			DBMDX_REGV_TDM_SYNC_DELAY_6_CLKS_CYCLES |
#else
			DBMDX_REGV_TDM_SYNC_DELAY_5_CLKS_CYCLES |
#endif
#else
#if defined(DBMDX_MELON_SRATE_48000) && defined(AEC_REF_32_TO_16_BIT)
			DBMDX_REGV_TDM_SYNC_DELAY_5_CLKS_CYCLES |
#else
			DBMDX_REGV_TDM_SYNC_DELAY_4_CLKS_CYCLES |
#endif
#endif
			DBMDX_REGV_TDM_SYNC_RIGHT_CH |
			DBMDX_REGV_USE_TDM_MUSIC_TO_SYNC |
			DBMDX_REGV_MUSIC_IN_TDM0),
#endif
#ifdef VC_RX_PROCESS_SUPPORTED
		(DBMDX_UC_SEQ_CMD_CONFIG_TDM |
			DBMDX_UC_SEQ_CMD_VAL_TDM1_VA_TX),
#endif
		(DBMDX_UC_SEQ_CMD_SET_AUDIO_ROUTING),
		(DBMDX_REGN_AUDIO_STREAMING_SRC_SELECT |
			DBMDX_REGV_NO_STREAM_CH_4 |
			DBMDX_REGV_NO_STREAM_CH_3 |
			DBMDX_REGV_NO_STREAM_CH_2 |
			DBMDX_REGV_NO_STREAM_CH_1),
		(DBMDX_UC_SEQ_CMD_CONFIG_MICS),
#ifndef DBMDX_MIC_TYPE_IS_DIGITAL
		(DBMDX_REGN_MICROPHONE_ANALOG_GAIN |
			DBMDX_REGV_SD_GAIN_EN |
			DBMDX_REGV_SD_GAIN_32_DB |
			DBMDX_REGV_SAR_GAIN_EN |
			DBMDX_REGV_SAR_GAIN_31_8_DB),
#else
		(DBMDX_REGN_DIGITAL_GAIN |
			DBMDX_REGV_GAIN_AFFECTS_ALL_MICS |
			DBMDX_REGV_DIGITAL_GAIN_3_DB),
#endif
#if !defined(VA_I2S_MASTER)
		(DBMDX_UC_SEQ_CMD_SET_IOM | 2),
#endif
	},
#if !defined(VA_I2S_MASTER)
#ifdef VC_RX_PROCESS_SUPPORTED
	.num_of_va_cfg_values = 19,
#else
	.num_of_va_cfg_values = 18,
#endif
#else
#ifdef VC_RX_PROCESS_SUPPORTED
	.num_of_va_cfg_values = 17,
#else
	.num_of_va_cfg_values = 16,
#endif
#endif

	.config_mics = MIC_CONFIG_BY_USECASE,

	.mic_config = {
#ifdef DBMDX_MIC_TYPE_IS_DIGITAL
		(DBMDX_REGV_DM_CLOCK_SRC_DM0_GPIO8_DM1_GPIO14 |
		 DBMDX_REGV_DM_DATA_SRC_DM0_GPIO9_DM1_GPIO13 |
		 DBMDX_REGV_DDF_AUDIO_ATTENUATION_0dB |
		 DBMDX_REGV_DM_CLK_FREQ_1536_1200_SR_8KHz_16KHz_32KHz_48KHz |
		 DBMDX_REGV_CLOCK_POLARITY_FALLING_EDGE |
		 DBMDX_REGV_DDF_AND_DM_CONFIG_DDF0_DM0),
#else
		(DBMDX_REGV_DDF_AUDIO_ATTENUATION_0dB |
		 DBMDX_REGV_SAR_IIR_FILTER_SD_ALIGN_D8D |
		 DBMDX_REGV_DDF_AND_DM_CONFIG_SAR_DDF_SAR_ADC),
#endif
		0x0000,
		0x0000,
		0x0000 },
#ifdef DBMDX_MIC_TYPE_IS_DIGITAL
	.mic_freq = 1536000,
#else
	.mic_freq = 384000,
#endif
	.num_of_va_ve_cfg_values = 0,
	.audio_routing_config = {
		(DBMDX_REGV_IO_SET_0 |
			DBMDX_REGV_IO_3N_2_NO_CP |
			DBMDX_REGV_IO_3N_1_NO_CP |
			DBMDX_REGV_IO_3N_0_CP_0),
		(DBMDX_REGV_IO_SET_1 |
			DBMDX_REGV_IO_3N_2_NO_CP |
			DBMDX_REGV_IO_3N_1_NO_CP |
			DBMDX_REGV_IO_3N_0_NO_CP),
		(DBMDX_REGV_IO_SET_2 |
			DBMDX_REGV_IO_3N_2_CP_2 |
			DBMDX_REGV_IO_3N_1_NO_CP |
			DBMDX_REGV_IO_3N_0_NO_CP),
		(DBMDX_REGV_IO_SET_3 |
			DBMDX_REGV_IO_3N_2_NO_CP |
			DBMDX_REGV_IO_3N_1_NO_CP |
			DBMDX_REGV_IO_3N_0_NO_CP),
		(DBMDX_REGV_IO_SET_4 |
			DBMDX_REGV_IO_3N_2_NO_CP |
			DBMDX_REGV_IO_3N_1_NO_CP |
			DBMDX_REGV_IO_3N_0_NO_CP),
		(DBMDX_REGV_IO_SET_5 |
			DBMDX_REGV_IO_3N_2_NO_CP |
			DBMDX_REGV_IO_3N_1_NO_CP |
			DBMDX_REGV_IO_3N_0_CP_0),
		(DBMDX_REGV_IO_SET_6 |
			DBMDX_REGV_IO_3N_2_NO_CP |
			DBMDX_REGV_IO_3N_1_NO_CP |
			DBMDX_REGV_IO_3N_0_NO_CP),
		(DBMDX_REGV_IO_SET_7 |
#ifdef VC_RX_PROCESS_SUPPORTED
			DBMDX_REGV_IO_3N_2_CP_2 |
#else
			DBMDX_REGV_IO_3N_2_NO_CP |
#endif
			DBMDX_REGV_IO_3N_1_NO_CP |
			DBMDX_REGV_IO_3N_0_NO_CP),
		(DBMDX_REGV_IO_SET_8 |
			DBMDX_REGV_IO_3N_2_NO_CP |
			DBMDX_REGV_IO_3N_1_NO_CP |
			DBMDX_REGV_IO_3N_0_NO_CP)},

	.tdm_configs_va = {
		/* DBMD4 TDM0_TX is connected to Host Codec */
		{	.tdm_index	= 0x0,
			.tdm_type	= TDM_TYPE_TX,
			.tdm_interface	= TDM_INTERFACE_VA,
			.enabled	= true,
			.tdm_reg_config	=
				(DBMDX_REGV_DEMUX_MUX_DISABLE |
				DBMDX_REGV_NUM_OF_CHANNELS_1_CH |
				DBMDX_REGV_SAMPLE_WIDTH_16_BIT |
#ifdef DBMDX_MELON_SRATE_48000
				DBMDX_REGV_INPUT_OUTPUT_SAMPLE_RATE_48_KHz |
				DBMDX_REGV_RESAMPLE_TYPE_INTERPOLATION |
				DBMDX_REGV_RESAMPLE_RATIO_3_1 |
#else /* DBMDX_MELON_SRATE_48000 */
				DBMDX_REGV_INPUT_OUTPUT_SAMPLE_RATE_16_KHz |
				DBMDX_REGV_RESAMPLE_TYPE_DECIMATION |
				DBMDX_REGV_RESAMPLE_RATIO_NO_RESAMPLING |
#endif /* DBMDX_MELON_SRATE_48000 */
		DBMDX_REGV_RX_TX_I2S_CH_SUM_HIGH_AND_LOW_INTO_ONE_SAMPLE |
				DBMDX_REGV_HW_BLOCK_EN |
				DBMDX_REGV_RX_TX_CP0),

			.num_of_io_reg_configs = 4,
#ifdef AEC_REF_32_TO_16_BIT
			.io_reg_configs	= {
				{		.addr = DBMD4_TDM_0_TX_ADDR,
#ifdef VA_I2S_MASTER
						.value = 0x00804052
#else
						.value = 0x00804053
#endif
				},
				{		.addr = DBMD4_TDM_0_TX_ADDR + 4,
#ifdef VA_I2S_MASTER
						.value = 0x00641064
#else
						.value = 0x00000064
#endif
				},
				{		.addr = DBMD4_TDM_0_TX_ADDR + 6,
						.value = 0x101F003F
				},
				{		.addr = DBMD4_TDM_0_TX_ADDR+0xA,
#ifdef VA_I2S_MASTER
						.value = 0x00000055
#else
						.value = 0x00000005
#endif
				}
			},
#else
			.io_reg_configs	= {
				{		.addr = DBMD4_TDM_0_TX_ADDR,
#ifdef VA_I2S_MASTER
						.value = 0x00804052
#else
						.value = 0x00804053
#endif
				},
				{		.addr = DBMD4_TDM_0_TX_ADDR + 4,
						.value = 0x00241024
				},
				{		.addr = DBMD4_TDM_0_TX_ADDR + 6,
						.value = 0x100F001F
				},
				{		.addr = DBMD4_TDM_0_TX_ADDR+0xA,
						.value = 0x0000000F
				}
			},
#endif /* AEC_REF_32_TO_16_BIT */
		},
		/* DBMD4 TDM0_RX is connected to Host Codec */
		{	.tdm_index	= 0x0,
			.tdm_type	= TDM_TYPE_RX,
			.tdm_interface	= TDM_INTERFACE_VA,
			.enabled	= true,
			.tdm_reg_config	=
				(DBMDX_REGV_DEMUX_MUX_ENABLE |
				DBMDX_REGV_NUM_OF_CHANNELS_2_CH |
				DBMDX_REGV_SAMPLE_WIDTH_16_BIT |
#ifdef DBMDX_MELON_SRATE_48000
				DBMDX_REGV_INPUT_OUTPUT_SAMPLE_RATE_48_KHz |
				DBMDX_REGV_RESAMPLE_RATIO_3_1 |
#else
				DBMDX_REGV_INPUT_OUTPUT_SAMPLE_RATE_16_KHz |
				DBMDX_REGV_RESAMPLE_RATIO_NO_RESAMPLING |
#endif
				DBMDX_REGV_RX_TX_I2S_CH_USE_I2S_STEREO |
				DBMDX_REGV_RESAMPLE_TYPE_DECIMATION |
				DBMDX_REGV_HW_BLOCK_EN |
				DBMDX_REGV_RX_TX_CP2),

#ifdef AEC_REF_32_TO_16_BIT
			.num_of_io_reg_configs = 4,
			.io_reg_configs	= {
				{		.addr = DBMD4_TDM_0_RX_ADDR,
						.value = 0x0080405D
				},
				{		.addr = DBMD4_TDM_0_RX_ADDR + 4,
						.value = 0x00002064
				},
				{		.addr = DBMD4_TDM_0_RX_ADDR + 6,
						.value = 0x103F003F
				},
				{		.addr = DBMD4_TDM_0_RX_ADDR+0xA,
						.value = 0x00000005
				}
			},
#else
			.num_of_io_reg_configs = 3,
			.io_reg_configs	= {
				{		.addr = DBMD4_TDM_0_RX_ADDR,
						.value = 0x00800015
				},
				{		.addr = DBMD4_TDM_0_RX_ADDR + 4,
						.value = 0x00000007
				},
				{		.addr = DBMD4_TDM_0_RX_ADDR + 6,
						.value = 0x100F001F
				},
			},
#endif /* AEC_REF_32_TO_16_BIT */
		},
		/* DBMD4 TDM1_TX is disabled */
		{	.tdm_index	= 0x1,
			.tdm_type	= TDM_TYPE_TX,
			.tdm_interface	= TDM_INTERFACE_VA,
#ifdef VC_RX_PROCESS_SUPPORTED
			.enabled	= true,
#else
			.enabled	= false,
#endif
			.tdm_reg_config	=
				(DBMDX_REGV_DEMUX_MUX_DISABLE |
				DBMDX_REGV_NUM_OF_CHANNELS_1_CH |
				DBMDX_REGV_SAMPLE_WIDTH_16_BIT |
#ifdef DBMDX_MELON_SRATE_48000
				DBMDX_REGV_INPUT_OUTPUT_SAMPLE_RATE_48_KHz |
				DBMDX_REGV_RESAMPLE_TYPE_INTERPOLATION |
				DBMDX_REGV_RESAMPLE_RATIO_3_1 |
#else /* DBMDX_MELON_SRATE_48000 */
				DBMDX_REGV_INPUT_OUTPUT_SAMPLE_RATE_16_KHz |
				DBMDX_REGV_RESAMPLE_TYPE_DECIMATION |
				DBMDX_REGV_RESAMPLE_RATIO_NO_RESAMPLING |
#endif /* DBMDX_MELON_SRATE_48000 */
		DBMDX_REGV_RX_TX_I2S_CH_SUM_HIGH_AND_LOW_INTO_ONE_SAMPLE |
				DBMDX_REGV_HW_BLOCK_EN |
				DBMDX_REGV_RX_TX_CP2),

			.num_of_io_reg_configs = 4,
#ifdef AEC_REF_32_TO_16_BIT
			.io_reg_configs	= {
				{		.addr = DBMD4_TDM_0_TX_ADDR,
#ifdef VA_I2S_MASTER
						.value = 0x00804052
#else
						.value = 0x00804053
#endif
				},
				{		.addr = DBMD4_TDM_0_TX_ADDR + 4,
#ifdef VA_I2S_MASTER
						.value = 0x00641064
#else
						.value = 0x00000064
#endif
				},
				{		.addr = DBMD4_TDM_0_TX_ADDR + 6,
						.value = 0x101F003F
				},
				{		.addr = DBMD4_TDM_0_TX_ADDR+0xA,
#ifdef VA_I2S_MASTER
						.value = 0x00000055
#else
						.value = 0x00000005
#endif
				}
			},
#else
			.io_reg_configs	= {
				{		.addr = DBMD4_TDM_0_TX_ADDR,
#ifdef VA_I2S_MASTER
						.value = 0x00804052
#else
						.value = 0x00804053
#endif
				},
				{		.addr = DBMD4_TDM_0_TX_ADDR + 4,
						.value = 0x00241024
				},
				{		.addr = DBMD4_TDM_0_TX_ADDR + 6,
						.value = 0x100F001F
				},
				{		.addr = DBMD4_TDM_0_TX_ADDR+0xA,
						.value = 0x0000000F
				}
			},
#endif /* AEC_REF_32_TO_16_BIT */
		},
		/* DBMD4 TDM1_RX is disabled */
		{	.tdm_index	= 0x0,
			.tdm_type	= TDM_TYPE_RX,
			.tdm_interface	= TDM_INTERFACE_VA,
			.enabled	= false,
		},
	},
	.va_start_cmd_type = START_CMD_TYPE_TDM,
	.send_va_start_cmd = true,
	/* Enable:
	 *		TDM0_RX (HOST CODEC==>D4)
	 *		TDM0_TX (D4=>Host Codec)
	 *		TDM1_TX (D4=>Speaker) RX Process
	 */
	.va_start_cmd = (DBMDX_REGV_TDM0_RX_EN |
				DBMDX_REGV_TDM0_TX_EN |
#ifdef VC_RX_PROCESS_SUPPORTED
				DBMDX_REGV_TDM0_TX_EN |
#endif
			DBMDX_REGV_PROC_EN_SWITCH_FW_TO_BUFFERING_MODE),

#ifdef DBMDX_I2S_BUFFERING_SUPPORTED
	.va_start_i2s_buffering_cmd = (DBMDX_REGV_TDM0_TX_EN |
					DBMDX_REGV_TDM0_TX_EN |
#ifdef VC_RX_PROCESS_SUPPORTED
					DBMDX_REGV_TDM0_TX_EN |
#endif
			DBMDX_REGV_PROC_EN_SWITCH_FW_TO_BUFFERING_MODE),
#endif

	.send_va_ve_start_cmd = false,
};


/* Melon VC Usecase */
static struct usecase_config config_uc_melon_production = {
	.usecase_name = "uc_melon_production",
	.id	= (DBMDX_USECASE_ID_UC_IDX_08 |
			DBMDX_USECASE_ID_PRJ_MELON |
			DBMDX_USECASE_ID_HWREV_00),
	.va_asrp_params_file_name = NULL,
	.hw_rev = 0,
	.send_va_asrp_parms = false,
	.send_va_ve_asrp_parms = false,
	.va_ve_asrp_params_file_name = NULL,
	.change_clock_src = true,
#ifdef VA_I2S_MASTER
	.clock_op_va = DBMDX_CLOCK_OP_SWITCH_TO_MASTER_CLOCK,
#else
	.clock_op_va = DBMDX_CLOCK_OP_SWITCH_TO_TDM_CLOCK,
#endif
#if defined(DBMDX_MELON_SRATE_48000)
	.tdm_clock_freq = TDM_CLK_FREQ_48,
#else
	.tdm_clock_freq = TDM_CLK_FREQ_16,
#endif
#ifdef AEC_REF_32_TO_16_BIT
	.number_of_bits = 32,
#else
	.number_of_bits = 16,
#endif
#ifdef VA_I2S_MASTER
	.i2s_master_clock_va = true,
#endif
	.clock_config_va = {
			.wanted_pll = 0,
			.wanted_tl3_clk = 98000000,
			.wanted_ahb_clk = 0,
			.wanted_apb_clk = 0,
			.use_pll_post_div = false,
		},

	.usecase_requires_amodel = false,
	.usecase_amodel_mode = 1,
	.usecase_sets_detection_mode = true,
	.usecase_supports_us_buffering = true,
	.va_chip_low_power_mode = false,
	.va_ve_chip_low_power_mode = false,
	.asrp_output_gain_type = (ASRP_VCPF_OUT_GAIN | ASRP_RX_OUT_GAIN),
	.complex_clb_1 = uc_load_models_unload_all,
	.complex_clb_2 = uc_stop_all_models,

	.va_cfg_values = (u32 []){
		(DBMDX_UC_SEQ_CMD_COMPLEX_CLB_1),
		(DBMDX_UC_SEQ_CMD_CHANGE_CLK_SRC),
		(DBMDX_UC_SEQ_CMD_COMPLEX_CLB_2),
		(DBMDX_UC_SEQ_CMD_CONFIG_TDM |
			DBMDX_UC_SEQ_CMD_VAL_TDM0_VA_RX),
#ifdef VA_I2S_MASTER
		(DBMDX_UC_SEQ_CMD_CONFIG_TDM |
			DBMDX_UC_SEQ_CMD_VAL_TDM0_VA_TX),
#endif
		(DBMDX_REGN_AUDIO_STREAMING_SRC_SELECT |
			DBMDX_REGV_STREAM_CH_4_CP_3 |
			DBMDX_REGV_STREAM_CH_3_CP_2 |
			DBMDX_REGV_STREAM_CH_2_CP_1 |
			DBMDX_REGV_STREAM_CH_1_CP_0),
		(DBMDX_UC_SEQ_CMD_CONFIG_MICS),
#ifndef DBMDX_MIC_TYPE_IS_DIGITAL
		(DBMDX_REGN_MICROPHONE_ANALOG_GAIN |
			DBMDX_REGV_SD_GAIN_EN |
			DBMDX_REGV_SD_GAIN_32_DB |
			DBMDX_REGV_SAR_GAIN_EN |
			DBMDX_REGV_SAR_GAIN_31_8_DB),
#else
		(DBMDX_REGN_DIGITAL_GAIN |
			DBMDX_REGV_GAIN_AFFECTS_ALL_MICS |
			DBMDX_REGV_DIGITAL_GAIN_3_DB),
#endif
	},
#ifdef VA_I2S_MASTER
	.num_of_va_cfg_values = 8,
#else
	.num_of_va_cfg_values = 7,
#endif

	.config_mics = MIC_CONFIG_BY_USECASE,

	.mic_config = {
#ifdef DBMDX_MIC_TYPE_IS_DIGITAL
		(DBMDX_REGV_DM_CLOCK_SRC_DM0_GPIO8_DM1_GPIO14 |
		 DBMDX_REGV_DM_DATA_SRC_DM0_GPIO9_DM1_GPIO13 |
		 DBMDX_REGV_DDF_AUDIO_ATTENUATION_0dB |
		 DBMDX_REGV_SYNCED_START |
		 DBMDX_REGV_DM_CLK_FREQ_1536_1200_SR_8KHz_16KHz_32KHz_48KHz |
		 DBMDX_REGV_CLOCK_POLARITY_FALLING_EDGE |
		 DBMDX_REGV_DDF_AND_DM_CONFIG_DDF0_DM0),
		(DBMDX_REGV_DM_CLOCK_SRC_DM0_GPIO8_DM1_GPIO14 |
		 DBMDX_REGV_DM_DATA_SRC_DM0_GPIO9_DM1_GPIO13 |
		 DBMDX_REGV_DDF_AUDIO_ATTENUATION_0dB |
		 DBMDX_REGV_DM_CLK_FREQ_1536_1200_SR_8KHz_16KHz_32KHz_48KHz |
		 DBMDX_REGV_CLOCK_POLARITY_RISING_EDGE |
		 DBMDX_REGV_DDF_AND_DM_CONFIG_DDF1_DM0),
#else
		(DBMDX_REGV_DDF_AUDIO_ATTENUATION_0dB |
		 DBMDX_REGV_SYNCED_START |
		 DBMDX_REGV_SAR_IIR_FILTER_SD_ALIGN_D8D |
		 DBMDX_REGV_DDF_AND_DM_CONFIG_SAR_DDF_SAR_ADC),
		(DBMDX_REGV_DDF_AUDIO_ATTENUATION_0dB |
		 DBMDX_REGV_DM_CLK_FREQ_1536_1200_SR_8KHz_16KHz_32KHz_48KHz |
		 DBMDX_REGV_DDF_AND_DM_CONFIG_SD_DDF_SD_ADC),
#endif
		 0x0000,
		 0x0000 },
#ifdef DBMDX_MIC_TYPE_IS_DIGITAL
	.mic_freq = 1536000,
#else
	.mic_freq = 384000,
#endif
	.num_of_va_ve_cfg_values = 0,

	.tdm_configs_va = {
		/* DBMD4 TDM0_TX is connected to Host Codec */
		{	.tdm_index	= 0x0,
			.tdm_type	= TDM_TYPE_TX,
			.tdm_interface	= TDM_INTERFACE_VA,
#ifdef VA_I2S_MASTER
			.enabled	= true,
#else
			.enabled	= false,
#endif
			.tdm_reg_config	=
				(DBMDX_REGV_DEMUX_MUX_ENABLE |
				DBMDX_REGV_NUM_OF_CHANNELS_2_CH |
				DBMDX_REGV_SAMPLE_WIDTH_16_BIT |
				DBMDX_REGV_TX_MASTER_CLOCK_ONLY |
#ifdef DBMDX_MELON_SRATE_48000
				DBMDX_REGV_INPUT_OUTPUT_SAMPLE_RATE_48_KHz |
				DBMDX_REGV_RESAMPLE_TYPE_INTERPOLATION |
				DBMDX_REGV_RESAMPLE_RATIO_3_1 |
#else /* DBMDX_MELON_SRATE_48000 */
				DBMDX_REGV_INPUT_OUTPUT_SAMPLE_RATE_16_KHz |
				DBMDX_REGV_RESAMPLE_TYPE_DECIMATION |
				DBMDX_REGV_RESAMPLE_RATIO_NO_RESAMPLING |
#endif /* DBMDX_MELON_SRATE_48000 */
				DBMDX_REGV_RX_TX_I2S_CH_USE_I2S_STEREO |
				DBMDX_REGV_HW_BLOCK_EN |
				DBMDX_REGV_RX_TX_CP0),

			.num_of_io_reg_configs = 4,
#ifdef AEC_REF_32_TO_16_BIT
			.io_reg_configs	= {
				{		.addr = DBMD4_TDM_0_TX_ADDR,
						.value = 0x00804052
				},
				{		.addr = DBMD4_TDM_0_TX_ADDR + 4,
						.value = 0x00641064
				},
				{		.addr = DBMD4_TDM_0_TX_ADDR + 6,
						.value = 0x101F003F
				},
				{		.addr = DBMD4_TDM_0_TX_ADDR+0xA,
						.value = 0x00000055
				}
			},
#else
			.io_reg_configs	= {
				{		.addr = DBMD4_TDM_0_TX_ADDR,
						.value = 0x00804052
				},
				{		.addr = DBMD4_TDM_0_TX_ADDR + 4,
						.value = 0x00241024
				},
				{		.addr = DBMD4_TDM_0_TX_ADDR + 6,
						.value = 0x100F001F
				},
				{		.addr = DBMD4_TDM_0_TX_ADDR+0xA,
						.value = 0x0000000F
				}
			},
#endif /* AEC_REF_32_TO_16_BIT */
		},
		/* DBMD4 TDM0_RX is connected to Host Codec */
		{	.tdm_index	= 0x0,
			.tdm_type	= TDM_TYPE_RX,
			.tdm_interface	= TDM_INTERFACE_VA,
			.enabled	= true,
			.tdm_reg_config	=
				(DBMDX_REGV_DEMUX_MUX_ENABLE |
				DBMDX_REGV_NUM_OF_CHANNELS_2_CH |
				DBMDX_REGV_SAMPLE_WIDTH_16_BIT |
#ifdef DBMDX_MELON_SRATE_48000
				DBMDX_REGV_INPUT_OUTPUT_SAMPLE_RATE_48_KHz |
				DBMDX_REGV_RESAMPLE_RATIO_3_1 |
#else
				DBMDX_REGV_INPUT_OUTPUT_SAMPLE_RATE_16_KHz |
				DBMDX_REGV_RESAMPLE_RATIO_NO_RESAMPLING |
#endif
				DBMDX_REGV_RX_TX_I2S_CH_USE_I2S_STEREO |
				DBMDX_REGV_RESAMPLE_TYPE_DECIMATION |
				DBMDX_REGV_HW_BLOCK_EN |
				DBMDX_REGV_RX_TX_CP2),

#ifdef AEC_REF_32_TO_16_BIT
			.num_of_io_reg_configs = 4,
			.io_reg_configs	= {
				{		.addr = DBMD4_TDM_0_RX_ADDR,
						.value = 0x0080405D
				},
				{		.addr = DBMD4_TDM_0_RX_ADDR + 4,
						.value = 0x00002064
				},
				{		.addr = DBMD4_TDM_0_RX_ADDR + 6,
						.value = 0x103F003F
				},
				{		.addr = DBMD4_TDM_0_RX_ADDR+0xA,
						.value = 0x00000005
				}
			},
#else
			.num_of_io_reg_configs = 3,
			.io_reg_configs	= {
				{		.addr = DBMD4_TDM_0_RX_ADDR,
						.value = 0x00800015
				},
				{		.addr = DBMD4_TDM_0_RX_ADDR + 4,
						.value = 0x00000007
				},
				{		.addr = DBMD4_TDM_0_RX_ADDR + 6,
						.value = 0x100F001F
				},
			},
#endif /* AEC_REF_32_TO_16_BIT */
		},
		/* DBMD4 TDM1_TX is disabled */
		{	.tdm_index	= 0x1,
			.tdm_type	= TDM_TYPE_TX,
			.tdm_interface	= TDM_INTERFACE_VA,
			.enabled	= false,
		},
		/* DBMD4 TDM1_RX is disabled */
		{	.tdm_index	= 0x0,
			.tdm_type	= TDM_TYPE_RX,
			.tdm_interface	= TDM_INTERFACE_VA,
			.enabled	= false,
		},
	},
	.va_start_cmd_type = START_CMD_TYPE_TDM,
	.send_va_start_cmd = true,
	/* Enable:
	 *		TDM0_RX (HOST CODEC==>D4)
	 *		TDM0_TX (CLK) MASTER
	 */
	.va_start_cmd = (DBMDX_REGV_TDM0_RX_EN |
				DBMDX_REGV_TDM0_TX_EN |
#ifdef VA_I2S_MASTER
				DBMDX_REGV_TDM0_TX_EN |
#endif
			DBMDX_REGV_PROC_EN_SWITCH_FW_TO_DETECTION_MODE),

	.send_va_ve_start_cmd = false,
};

/* Melon Bypass Usecase */
static struct usecase_config config_uc_melon_2mic_bypass = {
#if defined(DBMDX_MELON_SRATE_48000)
	.usecase_name = "uc_melon_2mic_bypass_48k",
#else
	.usecase_name = "uc_melon_2mic_bypass_16k",
#endif
	.id	= (DBMDX_USECASE_ID_UC_IDX_09 |
			DBMDX_USECASE_ID_PRJ_MELON |
			DBMDX_USECASE_ID_HWREV_00),
	.va_asrp_params_file_name = "asrp_params_melon_aecnr_dual_mic_ga.bin",

	.hw_rev = 0,
	.send_va_asrp_parms = true,
	.va_asrp_mem_loc = DBMDX_MEM_LOC_DCTM,
	.send_va_ve_asrp_parms = false,
	.va_ve_asrp_params_file_name = NULL,
	.change_clock_src = true,
#ifdef VA_I2S_MASTER
	.clock_op_va = DBMDX_CLOCK_OP_SWITCH_TO_MASTER_CLOCK,
#else
	.clock_op_va = DBMDX_CLOCK_OP_SWITCH_TO_TDM_CLOCK,
#endif
#if defined(DBMDX_MELON_SRATE_48000)
	.tdm_clock_freq = TDM_CLK_FREQ_48,
#else
	.tdm_clock_freq = TDM_CLK_FREQ_16,
#endif
#ifdef AEC_REF_32_TO_16_BIT
	.number_of_bits = 32,
#else
	.number_of_bits = 16,
#endif
#ifdef VA_I2S_MASTER
	.i2s_master_clock_va = true,
#endif
	.clock_config_va = {
			.wanted_pll = 0,
#ifndef DBMDX_MELON_OPTIMIZED
			.wanted_tl3_clk = 124000000,
#else /* Optimized */
#ifdef DBMDX_MELON_BARGEIN_GA_MODE
			.wanted_tl3_clk = 75000000,
#else /* Not GA */
#ifdef DBMDX_MELON_BARGEIN_1MIC
#if defined(DMBDX_OKG_AMODEL_SUPPORT) && defined(DMBDX_ALEXA_AMODEL_SUPPORT)
			.wanted_tl3_clk = 70000000,
#else
			.wanted_tl3_clk = 60000000,
#endif
#else /* 2 mic */
#if defined(DMBDX_OKG_AMODEL_SUPPORT) && defined(DMBDX_ALEXA_AMODEL_SUPPORT)
			.wanted_tl3_clk = 90000000,
#else
			.wanted_tl3_clk = 90000000,
#endif
#endif
#endif
#endif
			.wanted_ahb_clk = 0,
			.wanted_apb_clk = 0,
			.use_pll_post_div = false,
		},

	.usecase_requires_amodel = true,
	.usecase_amodel_mode = 1,
	.usecase_sets_detection_mode = true,
	.usecase_supports_us_buffering = true,
#ifdef DBMDX_MELON_BARGEIN_GA_MODE
	.usecase_supports_i2s_buffering = true,
#else
	.usecase_supports_i2s_buffering = false,
#endif
	.va_chip_low_power_mode = false,
	.va_ve_chip_low_power_mode = false,
	.asrp_output_gain_type = (ASRP_TX_OUT_GAIN),
	.complex_clb_1 = uc_load_models_general,
	.complex_clb_2 = melon_uc_barge_in_model_config,

	.va_cfg_values = (u32 []){
		(DBMDX_UC_SEQ_CMD_COMPLEX_CLB_1),
		(DBMDX_REGN_UART_SPEED | DBMDX_REGV_UART_BAUD_RATE_3_Mhz),
		(DBMDX_UC_SEQ_CMD_CHANGE_CLK_SRC),
		(DBMDX_UC_SEQ_CMD_LOAD_ASRP_PARAMS),
		(DBMDX_UC_SEQ_CMD_CONFIG_TDM |
			DBMDX_UC_SEQ_CMD_VAL_TDM0_VA_RX),
		(DBMDX_UC_SEQ_CMD_COMPLEX_CLB_2),
		(DBMDX_REGN_BUFFERING_NORMAL_AMPLITUDE |
			DBMDX_REGV_USE_PHRASE_LEN_FROM_WWE |
			DBMDX_REGV_NORMALIZE_TO_MINUS_6dB),
		(DBMDX_REGN_LOAD_NEW_ACOUSTIC_MODEL |
			DBMDX_REGV_LOAD_ENGINE_TYPE_ASRP |
			DBMDX_REGV_OP_TYPE_LOAD_FILE |
			DBMDX_REGV_INIT_KILL_SET |
			DBMDX_REGV_BLK_START_NUM_0 |
			DBMDX_REGV_LOAD_MODEL_TO_DTCM |
			DBMDX_REGV_ASRP_2ND_INIT),
		(DBMDX_UC_SEQ_CMD_SET_ASRP_DELAY),
#ifdef DBMDX_MELON_BARGEIN_GA_MODE
		(DBMDX_REGN_ASRP_OUTPUT_ROUTING |
			DBMDX_REGV_ASRP_OUTPUT_SRC_AEC_1 |
			DBMDX_REGV_ASRP_OUTPUT_DEST_TX_1),
		(DBMDX_REGN_ASRP_OUTPUT_ROUTING |
			DBMDX_REGV_ASRP_OUTPUT_SRC_AEC_2 |
			DBMDX_REGV_ASRP_OUTPUT_DEST_TX_2),
#else
		(DBMDX_REGN_ASRP_OUTPUT_ROUTING |
			DBMDX_REGV_ASRP_OUTPUT_SRC_BFPF_1 |
			DBMDX_REGV_ASRP_OUTPUT_DEST_TX_1),
#endif
		(DBMDX_UC_SEQ_CMD_SET_ASRP_GAIN),
		(DBMDX_REGN_AUDIO_PROCESSING_CONFIG |
			DBMDX_REGV_POST_DET_MODE_SWITCH_TO_STREAMING),
		(DBMDX_VA_MSLEEP | 0x1E),
		//(DBMDX_UC_SEQ_CMD_CONFIG_TDM |
			//DBMDX_UC_SEQ_CMD_VAL_TDM0_VA_TX),
#if defined(DBMDX_I2S_BUFFERING_SUPPORTED) || defined(VA_I2S_MASTER) ||\
	defined(DBMDX_MELON_BARGEIN_GA_MODE)
		(DBMDX_UC_SEQ_CMD_CONFIG_TDM |
			DBMDX_UC_SEQ_CMD_VAL_TDM0_VA_TX),
		(DBMDX_REGN_AUDIO_ROUTING_CONFIG |
#if !defined(VA_I2S_MASTER)
#if defined(DBMDX_MELON_SRATE_48000) && defined(AEC_REF_32_TO_16_BIT)
			DBMDX_REGV_TDM_SYNC_DELAY_7_CLKS_CYCLES |
#else
			DBMDX_REGV_TDM_SYNC_DELAY_5_CLKS_CYCLES |
#endif
			DBMDX_REGV_TDM_SYNC_RIGHT_CH |
			DBMDX_REGV_USE_TDM_MUSIC_TO_SYNC |
#endif
			DBMDX_REGV_MUSIC_IN_TDM0),

#endif
		(DBMDX_UC_SEQ_CMD_SET_AUDIO_ROUTING),
		(DBMDX_REGN_AUDIO_STREAMING_SRC_SELECT |
			DBMDX_REGV_NO_STREAM_CH_4 |
			DBMDX_REGV_NO_STREAM_CH_3 |
			DBMDX_REGV_NO_STREAM_CH_2 |
#ifdef DBMDX_MELON_BARGEIN_GA_MODE
			DBMDX_REGV_NO_STREAM_CH_1),
#else
			DBMDX_REGV_STREAM_CH_1_CP_0),
#endif
		(DBMDX_UC_SEQ_CMD_CONFIG_MICS),
#ifndef DBMDX_MIC_TYPE_IS_DIGITAL
		(DBMDX_REGN_MICROPHONE_ANALOG_GAIN | 0x8d87
			/*DBMDX_REGV_SD_GAIN_EN |
			DBMDX_REGV_SD_GAIN_22_DB |
			DBMDX_REGV_SAR_GAIN_EN |
			DBMDX_REGV_SAR_GAIN_25_6_DB*/),
#else
		(DBMDX_REGN_DIGITAL_GAIN | 0x0190
			/*DBMDX_REGV_GAIN_AFFECTS_ALL_MICS |
			DBMDX_REGV_DIGITAL_GAIN_3_DB*/),
#endif
	},
#ifdef DBMDX_MELON_BARGEIN_GA_MODE
	.num_of_va_cfg_values = 20,
#else
#ifdef    DBMDX_MELON_BARGEIN_1MIC
	.num_of_va_cfg_values = 18,
#else
	.num_of_va_cfg_values = 17,
#endif /* DBMDX_MELON_BARGEIN_1MIC */
#endif

	.config_mics = MIC_CONFIG_BY_USECASE,

	.mic_config = {
#ifdef DBMDX_MIC_TYPE_IS_DIGITAL
		(0xf261/*DBMDX_REGV_DM_CLOCK_SRC_DM0_GPIO8_DM1_GPIO14 |
		 DBMDX_REGV_DM_DATA_SRC_DM0_GPIO9_DM1_GPIO13 |
		 DBMDX_REGV_DDF_AUDIO_ATTENUATION_0dB |
#ifndef DBMDX_BARGE_IN_1_MIC
		 DBMDX_REGV_SYNCED_START |
#endif
		 DBMDX_REGV_DM_CLK_FREQ_1536_1200_SR_8KHz_16KHz_32KHz_48KHz |
		 DBMDX_REGV_CLOCK_POLARITY_FALLING_EDGE |
		 DBMDX_REGV_DDF_AND_DM_CONFIG_DDF0_DM0*/),
#ifndef DBMDX_BARGE_IN_1_MIC
		(0x5264/*DBMDX_REGV_DM_CLOCK_SRC_DM0_GPIO8_DM1_GPIO14 |
		 DBMDX_REGV_DM_DATA_SRC_DM0_GPIO9_DM1_GPIO13 |
		 DBMDX_REGV_DDF_AUDIO_ATTENUATION_0dB |
		 DBMDX_REGV_DM_CLK_FREQ_1536_1200_SR_8KHz_16KHz_32KHz_48KHz |
		 DBMDX_REGV_CLOCK_POLARITY_RISING_EDGE |
		 DBMDX_REGV_DDF_AND_DM_CONFIG_DDF1_DM0*/),
#else
		 0x0000,
#endif

#else
		(0x0a08
                  /*DBMDX_REGV_DDF_AUDIO_ATTENUATION_0dB |
#ifndef DBMDX_MELON_BARGEIN_1MIC
		 DBMDX_REGV_SYNCED_START |
#endif
		 DBMDX_REGV_SAR_IIR_FILTER_SD_ALIGN_D8D |
		 DBMDX_REGV_SAR_IIR_FILTER_256 |
		 DBMDX_REGV_SAR_ADC_IN_SEL_MIC2 |
		 DBMDX_REGV_DDF_AND_DM_CONFIG_SAR_DDF_SAR_ADC*/),
#ifndef DBMDX_MELON_BARGEIN_1MIC
		(0x267
		 /*DBMDX_REGV_DDF_AUDIO_ATTENUATION_0dB |
		 DBMDX_REGV_DM_CLK_FREQ_1536_1200_SR_8KHz_16KHz_32KHz_48KHz |
		 DBMDX_REGV_DDF_AND_DM_CONFIG_SD_DDF_SD_ADC*/),
#else
		 0x0000,
#endif
#endif
		 0x0000,
		 0x0000 },
#ifdef DBMDX_MIC_TYPE_IS_DIGITAL
	.mic_freq = 1536000,
#else
	.mic_freq = 384000,
#endif
	.num_of_va_ve_cfg_values = 0,
	.audio_routing_config = {
		(DBMDX_REGV_IO_SET_0 |
			DBMDX_REGV_IO_3N_2_NO_CP |
			DBMDX_REGV_IO_3N_1_CP_1 |
			DBMDX_REGV_IO_3N_0_CP_0),
		(DBMDX_REGV_IO_SET_1 |
			DBMDX_REGV_IO_3N_2_NO_CP |
			DBMDX_REGV_IO_3N_1_NO_CP |
			DBMDX_REGV_IO_3N_0_NO_CP),
		(DBMDX_REGV_IO_SET_2 |
			DBMDX_REGV_IO_3N_2_CP_2 |
			DBMDX_REGV_IO_3N_1_NO_CP |
			DBMDX_REGV_IO_3N_0_NO_CP),
		(DBMDX_REGV_IO_SET_3 |
			DBMDX_REGV_IO_3N_2_NO_CP |
			DBMDX_REGV_IO_3N_1_NO_CP |
			DBMDX_REGV_IO_3N_0_CP_3),
		(DBMDX_REGV_IO_SET_4 |
			DBMDX_REGV_IO_3N_2_NO_CP |
			DBMDX_REGV_IO_3N_1_NO_CP |
			DBMDX_REGV_IO_3N_0_NO_CP),
		(DBMDX_REGV_IO_SET_5 |
			DBMDX_REGV_IO_3N_2_NO_CP |
#ifdef DBMDX_MELON_BARGEIN_GA_MODE
			DBMDX_REGV_IO_3N_1_CP_1 |
#else
			DBMDX_REGV_IO_3N_1_NO_CP |
#endif
			DBMDX_REGV_IO_3N_0_CP_0),
		(DBMDX_REGV_IO_SET_6 |
			DBMDX_REGV_IO_3N_2_NO_CP |
			DBMDX_REGV_IO_3N_1_NO_CP |
			DBMDX_REGV_IO_3N_0_NO_CP),
		(DBMDX_REGV_IO_SET_7 |
			DBMDX_REGV_IO_3N_2_NO_CP |
			DBMDX_REGV_IO_3N_1_NO_CP |
			DBMDX_REGV_IO_3N_0_NO_CP),
		DBMDX_UNDEFINED_REGISTER },

	.tdm_configs_va = {
		/* DBMD4 TDM0_TX is connected to Host Codec */
		{	.tdm_index	= 0x0,
			.tdm_type	= TDM_TYPE_TX,
			.tdm_interface	= TDM_INTERFACE_VA,
#if defined(DBMDX_I2S_BUFFERING_SUPPORTED) || defined(VA_I2S_MASTER) ||\
	defined(DBMDX_MELON_BARGEIN_GA_MODE)
			.enabled	= true,
#else
			.enabled	= false,
#endif
			.tdm_reg_config	=
				(0x98d0
#if 0
				DBMDX_REGV_DEMUX_MUX_ENABLE |
				DBMDX_REGV_NUM_OF_CHANNELS_2_CH |
				DBMDX_REGV_SAMPLE_WIDTH_16_BIT |
#ifdef DBMDX_MELON_BARGEIN_GA_MODE
				DBMDX_REGV_TX_FULL_OPERATION |
#else
				DBMDX_REGV_TX_MASTER_CLOCK_ONLY |
#endif

#ifdef DBMDX_MELON_SRATE_48000
				DBMDX_REGV_INPUT_OUTPUT_SAMPLE_RATE_48_KHz |
				DBMDX_REGV_RESAMPLE_TYPE_INTERPOLATION |
				DBMDX_REGV_RESAMPLE_RATIO_3_1 |
#else /* DBMDX_MELON_SRATE_48000 */
				DBMDX_REGV_INPUT_OUTPUT_SAMPLE_RATE_16_KHz |
				DBMDX_REGV_RESAMPLE_TYPE_DECIMATION |
				DBMDX_REGV_RESAMPLE_RATIO_NO_RESAMPLING |
#endif /* DBMDX_MELON_SRATE_48000 */
				DBMDX_REGV_RX_TX_I2S_CH_USE_I2S_STEREO |
				DBMDX_REGV_HW_BLOCK_EN |
				DBMDX_REGV_RX_TX_CP0
#endif
                                    ),

			.num_of_io_reg_configs = 4,
#ifdef AEC_REF_32_TO_16_BIT
			.io_reg_configs	= {
				{		.addr = DBMD4_TDM_0_TX_ADDR,
#ifdef VA_I2S_MASTER
						.value = 0x00804052
#else
						.value = 0x00804053
#endif
				},
				{		.addr = DBMD4_TDM_0_TX_ADDR + 4,
#ifdef VA_I2S_MASTER
						.value = 0x00641064
#else
						.value = 0x00000064
#endif
				},
				{		.addr = DBMD4_TDM_0_TX_ADDR + 6,
						.value = 0x101F003F
				},
				{		.addr = DBMD4_TDM_0_TX_ADDR+0xA,
#ifdef VA_I2S_MASTER
						.value = 0x00000055
#else
						.value = 0x00000005
#endif
				}
			},
#else
			.io_reg_configs	= {
				{		.addr = DBMD4_TDM_0_TX_ADDR,
#ifdef VA_I2S_MASTER
						.value = 0x00804052
#else
						.value = 0x00804053
#endif
				},
				{		.addr = DBMD4_TDM_0_TX_ADDR + 4,
						.value = 0x00241024
				},
				{		.addr = DBMD4_TDM_0_TX_ADDR + 6,
						.value = 0x100F001F
				},
				{		.addr = DBMD4_TDM_0_TX_ADDR+0xA,
						.value = 0x0000000F
				}
			},
#endif /* AEC_REF_32_TO_16_BIT */
		},
		/* DBMD4 TDM0_RX is connected to Host Codec */
		{	.tdm_index	= 0x0,
			.tdm_type	= TDM_TYPE_RX,
			.tdm_interface	= TDM_INTERFACE_VA,
			.enabled	= true,
			/* .tdm_reg_config = 16KHz: 0x112, 48KHz: 0x912*/
			.tdm_reg_config	= (0x9852),
					/*(DBMDX_REGV_DEMUX_MUX_ENABLE |
				DBMDX_REGV_NUM_OF_CHANNELS_2_CH |
				DBMDX_REGV_SAMPLE_WIDTH_16_BIT |
#ifdef DBMDX_MELON_SRATE_48000
				DBMDX_REGV_INPUT_OUTPUT_SAMPLE_RATE_48_KHz |
				DBMDX_REGV_RESAMPLE_RATIO_3_1 |
#else
				DBMDX_REGV_INPUT_OUTPUT_SAMPLE_RATE_16_KHz |
				DBMDX_REGV_RESAMPLE_RATIO_NO_RESAMPLING |
#endif
				DBMDX_REGV_RX_TX_I2S_CH_USE_I2S_STEREO |
				DBMDX_REGV_RESAMPLE_TYPE_DECIMATION |
				DBMDX_REGV_HW_BLOCK_EN |
				DBMDX_REGV_RX_TX_CP2),*/

#ifdef AEC_REF_32_TO_16_BIT
			.num_of_io_reg_configs = 4,
			.io_reg_configs	= {
				{		.addr = DBMD4_TDM_0_RX_ADDR,
						.value = 0x0080405D
				},
				{		.addr = DBMD4_TDM_0_RX_ADDR + 4,
						.value = 0x00002064
				},
				{		.addr = DBMD4_TDM_0_RX_ADDR + 6,
						.value = 0x103F003F
				},
				{		.addr = DBMD4_TDM_0_RX_ADDR+0xA,
						.value = 0x00000005
				}
			},
#else
			.num_of_io_reg_configs = 3,
			.io_reg_configs	= {
				{		.addr = DBMD4_TDM_0_RX_ADDR,
						.value = 0x00800015
				},
				{		.addr = DBMD4_TDM_0_RX_ADDR + 4,
						.value = 0x00000007
				},
				{		.addr = DBMD4_TDM_0_RX_ADDR + 6,
						.value = 0x100F001F
				},
			},
#endif /* AEC_REF_32_TO_16_BIT */
		},
		/* DBMD4 TDM1_TX is disabled */
		{	.tdm_index	= 0x1,
			.tdm_type	= TDM_TYPE_TX,
			.tdm_interface	= TDM_INTERFACE_VA,
			.enabled	= false,
		},
		/* DBMD4 TDM1_RX is disabled */
		{	.tdm_index	= 0x1,
			.tdm_type	= TDM_TYPE_RX,
			.tdm_interface	= TDM_INTERFACE_VA,
			.enabled	= false,
		},
	},
	.va_start_cmd_type = START_CMD_TYPE_TDM,
	.send_va_start_cmd = true,
	/* Enable:
	 *		TDM0_RX (HOST CODEC==>D4)
	 */
	.va_start_cmd = (DBMDX_REGV_TDM0_RX_EN |
#if defined(VA_I2S_MASTER) || defined(DBMDX_MELON_BARGEIN_GA_MODE)
			 DBMDX_REGV_TDM0_TX_EN |
#endif
			DBMDX_REGV_PROC_EN_SWITCH_FW_TO_DETECTION_MODE),
#ifdef DBMDX_MELON_BARGEIN_GA_MODE
	.va_start_i2s_buffering_cmd = (DBMDX_REGV_TDM0_TX_EN |
					DBMDX_REGV_TDM0_RX_EN |
				DBMDX_REGV_PROC_EN_SWITCH_FW_TO_DETECTION_MODE),
#elif DBMDX_I2S_BUFFERING_SUPPORTED
	.va_start_i2s_buffering_cmd = (DBMDX_REGV_TDM0_TX_EN |
					DBMDX_REGV_TDM0_RX_EN |
				DBMDX_REGV_PROC_EN_SWITCH_FW_TO_BUFFERING_MODE),
#endif

	.send_va_ve_start_cmd = false,
};


#endif /* _DBMDX_USECASE_CONFIG_MELON_H */

#endif /* DBMDX_MELON_USECASES_SUPPORTED */
