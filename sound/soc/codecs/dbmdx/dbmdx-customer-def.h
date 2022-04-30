/*
 * dbmdx-customer.h  --  DBMDX customer definitions
 *
 * Copyright (C) 2014 DSP Group
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _DBMDX_CUSTOMER_DEF_H
#define _DBMDX_CUSTOMER_DEF_H

#define DBMD2_VA_FIRMWARE_NAME			"dbmd2_va_fw.bin"

#define DBMD4_VA_FIRMWARE_NAME			"dbmd4_va_fw.bin"

#define DBMDX_VT_GRAM_NAME			"voice_grammar.bin"
#define DBMDX_VT_NET_NAME			"voice_net.bin"
#define DBMDX_VT_AMODEL_NAME			"voice_amodel.bin"

#define DBMDX_VC_GRAM_NAME			"vc_grammar.bin"
#define DBMDX_VC_NET_NAME			"vc_net.bin"
#define DBMDX_VC_AMODEL_NAME			"vc_amodel.bin"

/* ================ Defines related to kernel vesion ===============*/



#define USE_ALSA_API_3_10_XX	0

#define SOC_BYTES_EXT_HAS_KCONTROL_FIELD	1


/* ==================================================================*/
/*
#define DBMDX_MANGO_USECASES_HW_REV_01_SUPPORTED	1
#define DBMDX_MANGO_USECASES_HW_REV_02_SUPPORTED	1
#define DBMDX_MANGO_USECASES_HW_REV_03_SUPPORTED	1
#define DBMDX_ORANGE_USECASES_SUPPORTED	1
#define DBMDX_HS_USECASES_SUPPORTED 1
*/
#define DBMDX_MELON_USECASES_SUPPORTED 1
/* #define DBMD2_DRIVES_DCLASS_SPEAKER	1 */
/* #define DBMDX_MANGO_NOVT_AUDIOSTREAM_USECASES_SUPPORTED 1 */
/* #define DBMDX_QED_SUPPORTED 1 */
/* #define DBMDX_I2S_BUFFERING_SUPPORTED	1 */
/* #define DBMDX_I2S_STREAMING_SUPPORTED	1 */

#define HOST_HW_TDM_CONF_ASYNC_DOWN_DATA_DOWN 0
#define HOST_HW_TDM_CONF_ASYNC_DOWN_DATA_UP 0
#define HOST_HW_TDM_CONF_ASYNC_UP_DATA_DOWN 0
#define HOST_HW_TDM_CONF_ASYNC_UP_DATA_UP 1

/* #define AEC_REF_32_TO_16_BIT			1 */

#define DEFAULT_D4_CLOCK_HZ	92160000
/* #define DEFAULT_D4_CLOCK_HZ	49152000 */
/* #define DEFAULT_D4_CLOCK_HZ	73728000 */

/* ================ Custom Configuration ===============*/
#define DBMDX_VA_VE_SUPPORT 1


#define DBMDX_DEFER_IF_SND_CARD_ID_0 1
/* #define DMBDX_OKG_AMODEL_SUPPORT 1 */

#define SV_FW_DETECTION_STATS 1

/* ==================================================================*/

#endif
