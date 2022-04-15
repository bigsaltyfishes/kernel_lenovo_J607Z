/* Copyright (c) 2015 Cirrus Logic, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#ifndef MSM_CIRRUS_PLAYBACK_H
#define MSM_CIRRUS_PLAYBACK_H

#include <linux/slab.h>
#include <sound/soc.h>
#include <dsp/apr_audio-v2.h>
#include <dsp/q6afe-v2.h>
#include <dsp/q6audio-v2.h>
#include <uapi/sound/cirrus-playback.h>

/* Payload struct for getting or setting one integer value from/to the DSP
 * module
 */
struct crus_single_data_t {
	int32_t	value;
	int32_t reserved;
};

/* Payload struct for getting or setting two integer values from/to the DSP
 * module
 */
struct crus_dual_data_t {
	int32_t data1;
	int32_t data2;
};

/* Payload struct for getting or setting three integer values from/to the DSP
 * module
 */
struct crus_triple_data_t {
	int32_t data1;
	int32_t data2;
	int32_t data3;
};

/* Payload struct for setting the RX and TX use cases */
struct crus_rx_run_case_ctrl_t {
	int32_t value;
	int32_t status_l;
	int32_t checksum_l;
	int32_t z_l;
	int32_t status_r;
	int32_t checksum_r;
	int32_t z_r;
	int32_t atemp;
};

/* Payload struct for getting calibration result from DSP module */
struct cirrus_cal_result_t {
	int32_t status_l;
	int32_t checksum_l;
	int32_t z_l;
	int32_t status_r;
	int32_t checksum_r;
	int32_t z_r;
};

#define APR_CHUNK_SIZE		256
#define CONFIG_FILE_SIZE	128
#define PAYLOAD_FOLLOWS_CONFIG	4
#define MAX_TUNING_CONFIGS	4
#define MIN_CHAN_SWAP_SAMPLES	48
#define MAX_CHAN_SWAP_SAMPLES	9600

/* Payload struct for sending an external configuration string to the DSP
 * module
 */
struct crus_external_config_t {
	uint32_t total_size;
	uint32_t chunk_size;
	int32_t done;
	int32_t reserved;
	int32_t config;
	char data[APR_CHUNK_SIZE];
};

/* Payload struct for sending an external tuning transition string to the DSP
 * module
 */
struct crus_delta_config_t {
	uint32_t total_size;
	uint32_t chunk_size;
	int32_t done;
	int32_t index;
	int32_t reserved;
	int32_t config;
	char data[APR_CHUNK_SIZE];
};

extern int afe_set_crus_params(u16 port_id, struct param_hdr_v3 param_hdr,
				u8 *param_data);
extern int afe_get_crus_params(u16 port_id, struct mem_mapping_hdr *mem_hdr,
				struct param_hdr_v3 *param_hdr);
extern int crus_afe_set_callback(
	int32_t (*crus_afe_callback_func)(void *payload, int size));
void msm_crus_pb_add_controls(struct snd_soc_component *component);

#endif /* _MSM_CIRRUS_PLAYBACK_H */

