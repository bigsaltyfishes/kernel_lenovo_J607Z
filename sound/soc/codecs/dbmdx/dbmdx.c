/*
 * DSPG DBMDX codec driver
 *
 * Copyright (C) 2014 DSP Group
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

#define DEBUG

#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/gpio.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <linux/clk.h>
#ifdef CONFIG_OF
#include <linux/of.h>
#ifdef CONFIG_OF_I2C
#include <linux/of_i2c.h>
#endif /* CONFIG_OF_I2C */
#include <linux/of_gpio.h>
#endif /* CONFIG_OF */
#include <linux/kfifo.h>
#include <linux/vmalloc.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/initval.h>
#include <sound/tlv.h>
#include <linux/i2c.h>
#include <linux/spi/spi.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/kthread.h>
#include <linux/version.h>

#include "dbmdx-interface.h"
#include "dbmdx-customer.h"
#include "dbmdx-i2s.h"
#include <sound/dbmdx-export.h>
#include "dbmdx-usecase-config.h"

#define DBMDX_ALWAYS_RELOAD_ASRP_PARAMS		1

/* Size must be power of 2 */
#define MAX_KFIFO_BUFFER_SIZE_MONO		(32768 * 8) /* >8 seconds */
#define MAX_KFIFO_BUFFER_SIZE_STEREO		(MAX_KFIFO_BUFFER_SIZE_MONO * 2)
#define MAX_KFIFO_BUFFER_SIZE_4CH		(MAX_KFIFO_BUFFER_SIZE_MONO * 4)

#ifdef DBMDX_4CHANNELS_SUPPORT
#define MAX_SUPPORTED_CHANNELS			4
#define MAX_KFIFO_BUFFER_SIZE			MAX_KFIFO_BUFFER_SIZE_4CH
#define VA_MIC_CONFIG_SIZE			5
#else
#define MAX_SUPPORTED_CHANNELS			2
#define MAX_KFIFO_BUFFER_SIZE			MAX_KFIFO_BUFFER_SIZE_STEREO
#define VA_MIC_CONFIG_SIZE			3
#endif

#define VA_VE_MIC_CONFIG_SIZE			4

#define MIN_RETRIES_TO_WRITE_TOBUF		5
#define MAX_RETRIES_TO_WRITE_TOBUF		200
#define MAX_AMODEL_SIZE				(256 * 1024)

#define DRIVER_VERSION				"4.047.2.20200503d"

#define DBMDX_AUDIO_MODE_PCM			0
#define DBMDX_AUDIO_MODE_MU_LAW			1

#define DBMDX_SND_PCM_RATE_16000		0x0000
#define DBMDX_SND_PCM_RATE_32000		0x0100
#define DBMDX_SND_PCM_RATE_44100		0x0100
#define DBMDX_SND_PCM_RATE_48000		0x0200
#define DBMDX_SND_PCM_RATE_8000			0x0300
#define DBMDX_SND_PCM_RATE_MASK			0xFCFF
#define DBMDX_HW_VAD_MASK			0x0060

#define DIGITAL_GAIN_TLV_MIN			0
#if defined(DBMDX_FW_BELOW_300) || defined(DBMDX_FW_BELOW_280)
#define DIGITAL_GAIN_TLV_MAX			240
#define DIGITAL_GAIN_TLV_SHIFT			120
#else
#define DIGITAL_GAIN_TLV_MAX			1920
#define DIGITAL_GAIN_TLV_SHIFT			860
#endif
#define MIN_EVENT_PROCESSING_TIME_MS		500
#define FW_BACKLOG_PADDING_IN_SAMPLES		120

#ifndef RETRY_COUNT
#define RETRY_COUNT				5
#endif

#if defined(CONFIG_SND_SOC_DBMDX_SND_CAPTURE) && \
	defined(DBMDX_USE_ASLA_CONTROLS_WITH_DBMDX_CARD_ONLY)
#define SOC_CONTROLS_FOR_DBMDX_CODEC_ONLY 1
#endif

#if defined(SND_SOC_BYTES_TLV)
#define EXTERNAL_SOC_AMODEL_LOADING_ENABLED 1
#endif


enum dbmdx_fw_debug_mode {
	FW_DEBUG_OUTPUT_UART = 0,
	FW_DEBUG_OUTPUT_CLK,
	FW_DEBUG_OUTPUT_CLK_UART,
	FW_DEBUG_OUTPUT_NONE
};

#define VA_MIXER_REG(cmd)			\
	(((cmd) >> 16) & 0x7fff)

static const char *dbmdx_power_mode_names[DBMDX_PM_STATES] = {
	"BOOTING",
	"ACTIVE",
	"FALLING_ASLEEP",
	"SLEEPING",
};

static const char *dbmdx_state_names[DBMDX_NR_OF_STATES] = {
	"IDLE",
	"DETECTION",
	"BUFFERING",
	"UART_RECORDING",
	"SLEEP_PLL_ON",
	"SLEEP_PLL_OFF",
	"HIBERNATE",
	"PCM_STREAMING",
	"DETECTION_AND_STREAMING",
};

static const char *dbmdx_of_clk_names[DBMDX_NR_OF_CLKS] = {
	"dbmdx_constant_clk",
	"dbmdx_master_clk",
	"dbmdx_master_va_ve_clk",
};

#ifdef CONFIG_OF
static const char *dbmdx_of_clk_rate_names[DBMDX_NR_OF_CLKS] = {
	"constant-clk-rate",
	"master-clk-rate",
	"master_va_ve-clk-rate",
};
#endif

static const char *dbmdx_fw_names[DBMDX_FW_MAX] = {
	[DBMDX_FW_PRE_BOOT]  = "PRE_BOOT",
	[DBMDX_FW_VA]        = "VA",
	[DBMDX_FW_POWER_OFF_VA] = "POWER_OFF",
};


const char *dbmdx_model_state_names[MODEL_STATE_MAX] = {
	"UNLOADED",
	"LOADED",
	"STARTED",
};

const char *dbmdx_vte_det_type_names[DETECTION_TYPE_MAX] = {
	"PASSPHRASE",
	"VOICE ENERGY",
	"VOICE COMMAND",
	"DUAL",
};

const char *dbmdx_vte_names[DBMDX_VT_ENGINE_TYPE_MAX] = {
	"UNDEFINED",
	"SENSORY",
	"ALEXA",
	"GOOGLE",
	"BIXBY",
	"GENERAL",
};


/* Global Variables */
struct dbmdx_private *dbmdx_data;
#ifdef SND_SOC_COMPONENT
struct snd_soc_component *remote_component;
#else
struct snd_soc_codec *remote_codec;
#endif
void (*g_event_callback)(int) = NULL;
void (*g_set_i2c_freq_callback)(struct i2c_adapter*, enum i2c_freq_t) = NULL;
extern int es7210_resume(void);
extern int es7210_suspend(void);
/* Forward declarations */
static int dbmdx_perform_recovery(struct dbmdx_private *p);
static int dbmdx_read_fw_vad_settings(struct dbmdx_private *p);
static int dbmdx_va_amodel_load_file(struct dbmdx_private *p,
			int num_of_amodel_files,
			const char **amodel_fnames,
			const char *amodel_prefix,
			u32 gram_addr,
			u32 amodel_options,
			char	*amodel_buf,
			ssize_t *amodel_size,
			int *num_of_amodel_chunks,
			ssize_t *amodel_chunks_size);
static int dbmdx_va_amodel_load_dummy_model(struct dbmdx_private *p,
			u32 gram_addr,
			char	*amodel_buf,
			ssize_t *amodel_size,
			int *num_of_amodel_chunks,
			ssize_t *amodel_chunks_size);

static int dbmdx_shutdown(struct dbmdx_private *p);
static int dbmdx_va_amodel_send(struct dbmdx_private *p,  const void *data,
			   size_t size, int num_of_chunks, size_t *chunk_sizes,
			   u16 model_vt_num,
			   enum dbmdx_mem_loc model_mem_loc,
			   const void *checksum, size_t chksum_len,
			   enum dbmdx_load_amodel_mode model_type);
static int dbmdx_get_vte_from_model_type(struct dbmdx_private *p,
						enum dbmdx_vt_engine_type type,
						struct vt_engine **vt_eng);
static int dbmdx_model_loaded_correctly(struct dbmdx_private *p, u16 vt_index);
static int dbmdx_va_enable_vte_detection(struct dbmdx_private *p,
				struct vt_engine *vte,
				enum dbmdx_model_detection_type det_type,
				u16 model_custom_params,
				bool *uc_reload_required);

static int dbmdx_va_disable_vte_detection(struct dbmdx_private *p,
				struct vt_engine *vte,
				enum dbmdx_model_detection_type det_type);
static int dbmdx_va_load_amodel(struct dbmdx_private *p,
				struct vt_engine *vte,
				enum dbmdx_model_detection_type det_type,
				u32 model_options, int param);
static int dbmdx_va_unload_amodel(struct dbmdx_private *p,
				struct vt_engine *vte,
				enum dbmdx_model_detection_type det_type);


#ifdef CONFIG_OF
static int dbmdx_get_fw_interfaces(struct dbmdx_private *p,
				   const char *tag,
				   int *iarray);
#endif
static int dbmdx_va_chip_probe(struct dbmdx_private *p);
static void dbmdx_va_chip_remove(struct dbmdx_private *p);
static int dbmdx_va_ve_chip_probe(struct dbmdx_private *p);
static void dbmdx_va_ve_chip_remove(struct dbmdx_private *p);
static int dbmdx_va_load_asrp_params(struct dbmdx_private *p,
			enum dbmdx_mem_loc model_mem_loc,
			const char *dbmdx_asrp_name);
static int dbmdx_schedule_work(struct dbmdx_private *p,
					struct work_struct *work)
{
#ifdef USE_DEDICATED_WORKQUEUE
	return queue_work(p->dbmdx_workq, work);
#else
	return schedule_work(work);
#endif
}

static const char *dbmdx_fw_type_to_str(int fw_type)
{
	if (fw_type >= DBMDX_FW_MAX)
		return "ERROR";
	return dbmdx_fw_names[fw_type];
}

static int dbmdx_set_active_interface(struct dbmdx_private *p,
				       int interface_idx)
{
	if (p == NULL) {
		dev_err(p->dev,
			"%s: DBMDX platform was not initialized (p==NULL)\n",
			__func__);
		return -ENODEV;
	}

	if (interface_idx < 0) {
		dev_err(p->dev,	"%s: Interface is not supported\n", __func__);
		return -EINVAL;
	}

	if (interface_idx >= p->nr_of_interfaces) {
		dev_err(p->dev,
			"%s: Invalid interface index: %d (index range[0:%d]\n",
			__func__, interface_idx, p->nr_of_interfaces - 1);
		return -EINVAL;
	}

	p->chip = p->interfaces[interface_idx];
	p->active_interface = p->interface_types[interface_idx];

#ifdef DBMDX_VERBOSE_SWITCHING
	dev_info(p->dev, "%s: switched to interface#: %d\n",
		__func__, interface_idx);
#endif
	return 0;
}

int dbmdx_switch_to_va_chip_interface(struct dbmdx_private *p,
		enum dbmdx_interface_type interface_type)

{
	int ret = 0;

	if (!(p->va_chip_enabled)) {
		dev_err(p->dev,
			"%s: Attempt to switch to disabled VA chip\n",
			__func__);
		return -EIO;

	}

	/* set VA as active firmware */
	p->active_fw = p->active_fw_va_chip;

	ret = dbmdx_set_active_interface(p,
		p->pdata->va_interfaces[interface_type]);

	if (ret) {
		dev_err(p->dev, "%s: failed to set interface\n", __func__);
		return ret;
	}

	p->active_interface_type_va = interface_type;
	p->cur_reset_gpio = p->pdata->gpio_reset;
	p->cur_wakeup_gpio = p->pdata->gpio_wakeup;
	p->cur_wakeup_disabled = p->pdata->wakeup_disabled;
	p->cur_wakeup_set_value = p->pdata->wakeup_set_value;
	p->cur_send_wakeup_seq = p->pdata->send_wakeup_seq;
	p->cur_use_gpio_for_wakeup = p->pdata->use_gpio_for_wakeup;
	p->cur_firmware_id = p->pdata->firmware_id;
	p->cur_boot_options = p->pdata->boot_options;
	p->asleep = p->asleep_va;
	p->active_chip = DBMDX_CHIP_VA;

	return ret;
}

int dbmdx_switch_to_va_ve_chip_interface(struct dbmdx_private *p,
	enum dbmdx_interface_type interface_type)
{
	int ret = 0;

	if (!(p->va_ve_chip_enabled)) {
		dev_err(p->dev,
			"%s: Attempt to switch to disabled VA_VE chip\n",
			__func__);
		return -EIO;

	}

	/* set VA_VE as active firmware */
	p->active_fw = p->active_fw_va_ve_chip;

	ret = dbmdx_set_active_interface(p,
		p->pdata->va_ve_interfaces[interface_type]);

	if (ret) {
		dev_err(p->dev, "%s: failed to set interface\n", __func__);
		return ret;
	}

	p->active_interface_type_va_ve = interface_type;
	p->cur_reset_gpio = p->pdata->gpio_reset_va_ve;
	p->cur_wakeup_gpio = p->pdata->gpio_wakeup_va_ve;
	p->cur_wakeup_disabled = p->pdata->wakeup_va_ve_disabled;
	p->cur_wakeup_set_value = p->pdata->wakeup_va_ve_set_value;
	p->cur_send_wakeup_seq = p->pdata->send_wakeup_va_ve_seq;
	p->cur_use_gpio_for_wakeup = p->pdata->use_gpio_for_wakeup_va_ve;
	p->cur_firmware_id = p->pdata->firmware_id_va_ve;
	p->cur_boot_options = p->pdata->boot_options_va_ve;

	p->asleep = p->asleep_va_ve;

	p->active_chip = DBMDX_CHIP_VA_VE;

	return ret;
}


static int dbmdx_switch_to_user_selected_chip_interface(struct dbmdx_private *p)
{

	enum dbmdx_firmware_active	cur_active_fw;
	int ret;

	cur_active_fw = p->active_fw;

	if (!p->pdata->feature_va_ve)
		return 0;

	if ((p->va_chip_enabled &&
		(p->active_interface_type_va != DBMDX_CMD_INTERFACE)) ||
		(p->va_ve_chip_enabled &&
			(p->active_interface_type_va_ve !=
				DBMDX_CMD_INTERFACE))) {
			dev_err(p->dev,
				"%s Chips are not in CMD interface\n",
				__func__);
			return -EINVAL;
	}

	p->va_ve_flags.drv_seletected_chip = p->active_chip;

	if (p->va_ve_flags.drv_seletected_chip != p->usr_selected_chip) {

		if (p->usr_selected_chip == DBMDX_CHIP_VA) {
			ret = dbmdx_switch_to_va_chip_interface(p,
						DBMDX_CMD_INTERFACE);
			if (ret) {
				dev_err(p->dev,
					"%s Err. switching to (VA) iface\n",
						__func__);
				return -EIO;
			}
		} else if (p->usr_selected_chip == DBMDX_CHIP_VA_VE) {
			ret = dbmdx_switch_to_va_ve_chip_interface(p,
						DBMDX_CMD_INTERFACE);
			if (ret) {
				dev_err(p->dev,
					"%s Err. switching to (VA_VE) iface\n",
						__func__);
				return -EIO;
			}
		}
	}

	return 0;
}

static int dbmdx_switch_to_drv_selected_chip_interface(struct dbmdx_private *p)
{

	enum dbmdx_firmware_active	cur_active_fw;
	int ret;

	cur_active_fw = p->active_fw;

	if (!p->pdata->feature_va_ve)
		return 0;

	if ((p->va_chip_enabled &&
		(p->active_interface_type_va != DBMDX_CMD_INTERFACE)) ||
		(p->va_ve_chip_enabled &&
			(p->active_interface_type_va_ve !=
				DBMDX_CMD_INTERFACE))) {
			dev_err(p->dev,
				"%s Chips are not in CMD interface\n",
				__func__);
			return -EINVAL;
	}

	if (p->va_ve_flags.drv_seletected_chip != p->active_chip) {

		if (p->va_ve_flags.drv_seletected_chip == DBMDX_CHIP_VA) {
			ret = dbmdx_switch_to_va_chip_interface(p,
						DBMDX_CMD_INTERFACE);
			if (ret) {
				dev_err(p->dev,
					"%s Err. switching to (VA) iface\n",
						__func__);
				return -EIO;
			}
		} else if (p->va_ve_flags.drv_seletected_chip ==
							DBMDX_CHIP_VA_VE) {
			ret = dbmdx_switch_to_va_ve_chip_interface(p,
						DBMDX_CMD_INTERFACE);
			if (ret) {
				dev_err(p->dev,
					"%s Err. switching to (VA_VE) iface\n",
						__func__);
				return -EIO;
			}
		}
	}

	return 0;
}

static void dbmdx_set_va_active(struct dbmdx_private *p)
{
	/* set VA as active firmware */
	p->active_fw_va_chip = DBMDX_FW_VA;
	/* reset all flags */
	memset(&p->va_flags, 0, sizeof(p->va_flags));
	memset(&p->va_ve_flags, 0, sizeof(p->va_ve_flags));
}

static void dbmdx_set_va_ve_active(struct dbmdx_private *p)
{
	/* set VA as active firmware */
	p->active_fw_va_ve_chip = DBMDX_FW_VA_VE;
	/* reset all flags */
	memset(&p->va_ve_flags, 0, sizeof(p->va_ve_flags));
}

static void dbmdx_set_boot_active(struct dbmdx_private *p)
{
	/* set nothing as active firmware */
	p->active_fw_va_chip =  DBMDX_FW_PRE_BOOT;
	p->va_flags.mode = DBMDX_IDLE;
	p->device_ready = false;
	p->asleep = false;
	p->asleep_va = false;
}

static void dbmdx_set_boot_active_va_ve_chip(struct dbmdx_private *p)
{
	/* set nothing as active firmware */
	p->active_fw_va_ve_chip =  DBMDX_FW_PRE_BOOT;
	p->device_ready_va_ve = false;
	p->asleep_va_ve = false;
	p->va_ve_flags.mode = DBMDX_IDLE;
}

static void dbmdx_reset_set(struct dbmdx_private *p)
{
	if (p->pdata->gpio_d2strap1 >= 0)
		gpio_direction_output(p->pdata->gpio_d2strap1,
				p->pdata->gpio_d2strap1_rst_val);

	dev_dbg(p->dev, "%s: %d==>gpio%d\n", __func__, 0, p->cur_reset_gpio);

	gpio_set_value(p->cur_reset_gpio, 0);
}

static void dbmdx_reset_release(struct dbmdx_private *p)
{
	dev_dbg(p->dev, "%s: %d==>gpio%d\n", __func__, 1, p->cur_reset_gpio);

	gpio_set_value(p->cur_reset_gpio, 1);

	if (p->pdata->gpio_d2strap1 >= 0)
		gpio_direction_input(p->pdata->gpio_d2strap1);
}

static void dbmdx_reset_sequence(struct dbmdx_private *p)
{
	dbmdx_reset_set(p);
	usleep_range(DBMDX_USLEEP_RESET_TOGGLE,
		DBMDX_USLEEP_RESET_TOGGLE + 100);
	dbmdx_reset_release(p);
}

static int dbmdx_can_wakeup(struct dbmdx_private *p)
{
	if (p->cur_wakeup_disabled)
		return 0;

	/* If use_gpio_for_wakeup equals zero than transmit operation
	 * itself will wakeup the chip
	 */
	if (!p->cur_use_gpio_for_wakeup)
		return 1;

	return p->cur_wakeup_gpio < 0 ? 0 : 1;
}

static void dbmdx_wakeup_set(struct dbmdx_private *p)
{
	/* If use_gpio_for_wakeup equals zero than transmit operation
	 * itself will wakeup the chip
	 */
	if (p->cur_wakeup_disabled || p->cur_wakeup_gpio < 0 ||
		!p->cur_use_gpio_for_wakeup)
		return;

	dev_dbg(p->dev, "%s: %d==>gpio%d\n", __func__,
		p->cur_wakeup_set_value, p->cur_wakeup_gpio);

	gpio_set_value(p->cur_wakeup_gpio, p->cur_wakeup_set_value);
}

static void dbmdx_wakeup_release(struct dbmdx_private *p)
{
	/* If use_gpio_for_wakeup equals zero than transmit operation
	 * itself will wakeup the chip
	 */
	if (p->cur_wakeup_disabled || p->cur_wakeup_gpio < 0 ||
		!p->cur_use_gpio_for_wakeup)
		return;

	dev_dbg(p->dev, "%s: %d==>gpio%d\n", __func__,
		!(p->cur_wakeup_set_value), p->cur_wakeup_gpio);

	gpio_set_value(p->cur_wakeup_gpio, !(p->cur_wakeup_set_value));
}

static void dbmdx_wakeup_toggle(struct dbmdx_private *p)
{
	/* If use_gpio_for_wakeup equals zero than transmit operation
	 * itself will wakeup the chip
	 */
	if (p->cur_wakeup_disabled || p->cur_wakeup_gpio < 0 ||
		!p->cur_use_gpio_for_wakeup)
		return;

	gpio_set_value(p->cur_wakeup_gpio, p->cur_wakeup_set_value);
	usleep_range(1000, 1100);
	gpio_set_value(p->cur_wakeup_gpio, !(p->cur_wakeup_set_value));
}

static long dbmdx_clk_set_rate(struct dbmdx_private *p,
			       enum dbmdx_clocks dbmdx_clk)
{
	struct clk *clk = p->clocks[dbmdx_clk];
	int rate = p->pdata->clock_rates[dbmdx_clk];

	if (clk && (rate != -1))
		return clk_set_rate(clk, rate);

	return customer_dbmdx_clk_set_rate(p, dbmdx_clk);
}

static unsigned long dbmdx_clk_get_rate(struct dbmdx_private *p,
					enum dbmdx_clocks dbmdx_clk)
{
	struct clk *clk = p->clocks[dbmdx_clk];
	int rate = p->pdata->clock_rates[dbmdx_clk];

	if (clk)
		return clk_get_rate(clk);

	if (rate)
		return rate;

	return customer_dbmdx_clk_get_rate(p, dbmdx_clk);
}

static int dbmdx_clk_enable(struct dbmdx_private *p,
			    enum dbmdx_clocks dbmdx_clk)
{
	int ret = 0;
	struct clk *clk = p->clocks[dbmdx_clk];

	if (clk)
		ret = clk_prepare_enable(clk);
	else
		ret = customer_dbmdx_clk_enable(p, dbmdx_clk);

	if (ret < 0)
		dev_err(p->dev, "%s: %s clock enable failed\n",
			__func__,
			dbmdx_of_clk_names[dbmdx_clk]);
	else
		ret = 0;

	return ret;
}

static int dbmdx_clk_disable(struct dbmdx_private *p,
			     enum dbmdx_clocks dbmdx_clk)
{
	struct clk *clk = p->clocks[dbmdx_clk];

	if (clk)
		clk_disable_unprepare(clk);
	else
		customer_dbmdx_clk_disable(p, dbmdx_clk);

	return 0;
}

static void dbmdx_usr_op_lock(struct dbmdx_private *p)
{
	mutex_lock(&p->p_usr_op_lock);
}

static void dbmdx_usr_op_unlock(struct dbmdx_private *p)
{
	mutex_unlock(&p->p_usr_op_lock);
}


static void dbmdx_lock(struct dbmdx_private *p)
{
	mutex_lock(&p->p_lock);
}

static void dbmdx_unlock(struct dbmdx_private *p)
{
	mutex_unlock(&p->p_lock);
}

static int dbmdx_verify_checksum(struct dbmdx_private *p,
	const u8 *expect, const u8 *got, size_t size)
{
	int ret;

	ret = memcmp(expect, got, size);
	if (ret) {
		switch (size) {
		case 4:
			dev_info(p->dev,
				"%s: Got:      0x%02x 0x%02x 0x%02x 0x%02x\n",
				__func__,
				got[0], got[1], got[2], got[3]);
			dev_info(p->dev,
				"%s: Expected: 0x%02x 0x%02x 0x%02x 0x%02x\n",
				__func__,
				expect[0], expect[1], expect[2], expect[3]);
			break;
		default:
			break;
		}
	}
	return ret;
}

static ssize_t dbmdx_send_data(struct dbmdx_private *p, const void *buf,
			       size_t len)
{
	return p->chip->write(p, buf, len);
}


static int dbmdx_send_cmd_32(struct dbmdx_private *p,
	u32 command, u32 address, u32 *response)
{
	int ret;

	switch (p->active_fw) {
	case DBMDX_FW_VA:
	case DBMDX_FW_VA_VE:
		ret = p->chip->send_cmd_va_32(p, command, address, response);
		break;
	default:
		dev_err(p->dev, "%s: Don't know how to handle fw type %d\n",
			__func__, p->active_fw);
		ret = -EIO;
		break;
	}
	return ret;
}

static int dbmdx_send_cmd(struct dbmdx_private *p, u32 command, u16 *response)
{
	int ret;

	switch (p->active_fw) {
	case DBMDX_FW_VA:
	case DBMDX_FW_VA_VE:
		ret = p->chip->send_cmd_va(p, command, response);
		break;
	default:
		dev_err(p->dev, "%s: Don't know how to handle fw type %d\n",
			__func__, p->active_fw);
		ret = -EIO;
		break;
	}
	return ret;
}

static int dbmdx_va_alive(struct dbmdx_private *p)
{
	u16 result = 0;
	int ret = 0;
	unsigned long stimeout = jiffies +
				msecs_to_jiffies(DBMDX_MSLEEP_IS_ALIVE);

	do {

		/* check if VA firmware is still alive */
		ret = dbmdx_send_cmd(p, DBMDX_REGN_CHIP_ID_NUMBER, &result);
		if (ret < 0)
			continue;
		if (result == p->cur_firmware_id)
			ret = 0;
		else
			ret = -1;
	} while (time_before(jiffies, stimeout) && ret != 0);

	if (ret != 0)
		dev_err(p->dev, "%s: VA firmware dead\n", __func__);

	return ret;
}

static int dbmdx_va_alive_with_lock(struct dbmdx_private *p)
{
	int ret = 0;

	p->lock(p);

	dbmdx_set_power_mode(p, DBMDX_PM_ACTIVE);

	ret = dbmdx_va_alive(p);

	p->unlock(p);

	return ret;
}

/* Check that all enabled chips are alive */
static int dbmdx_va_ve_alive_with_lock(struct dbmdx_private *p)
{
	int ret = 0;
	int ret2 = 0;
	enum dbmdx_chip	cur_active_chip = p->active_chip;

	p->lock(p);

	dbmdx_set_power_mode(p, DBMDX_PM_ACTIVE);

	if (p->va_chip_enabled) {

		ret = dbmdx_switch_to_va_chip_interface(p, DBMDX_CMD_INTERFACE);
		if (ret) {
			dev_err(p->dev,
				"%s Error switching to (VA) CMD interface\n",
				__func__);
			ret = -EIO;
			goto out;
		}
		ret = dbmdx_va_alive(p);
		if (ret) {
			dev_err(p->dev,	"%s (VA) FW is dead\n",	__func__);
			ret = -EIO;
			goto out;
		}
	}

	if (p->va_ve_chip_enabled) {

		ret = dbmdx_switch_to_va_ve_chip_interface(p,
							DBMDX_CMD_INTERFACE);
		if (ret) {
			dev_err(p->dev,
				"%s Error switching to (VA_VE) CMD interface\n",
				__func__);
			ret = -EIO;
			goto out;
		}
		ret = dbmdx_va_alive(p);
		if (ret) {
			dev_err(p->dev,	"%s (VA_VE) FW is dead\n", __func__);
			ret = -EIO;
			goto out;
		}
	}

out:

	if (cur_active_chip == DBMDX_CHIP_VA) {
		ret2 = dbmdx_switch_to_va_chip_interface(p,
						DBMDX_CMD_INTERFACE);
		if (ret2) {
			dev_err(p->dev,
				"%s Error switching to (VA) CMD interface\n",
				__func__);
			ret = -EIO;
		}
	} else {
		ret2 = dbmdx_switch_to_va_ve_chip_interface(p,
						DBMDX_CMD_INTERFACE);
		if (ret2) {
			dev_err(p->dev,
				"%s Error switching to (VA_VE) CMD interface\n",
				__func__);
			ret = -EIO;
		}
	}

	p->unlock(p);

	return ret;
}

/* Place audio samples to kfifo according to operation flag
 *	AUDIO_CHANNEL_OP_COPY - copy samples directly to kfifo
 *	AUDIO_CHANNEL_OP_DUPLICATE_X_TO_Y - dupl. X to ch to Y (e.g.dual mono)
 *	AUDIO_CHANNEL_OP_TRUNCATE_Y_TO_X - take samples from primary channel set
 */
static int dbmdx_add_audio_samples_to_kfifo(struct dbmdx_private *p,
	struct kfifo *fifo,
	const u8 *buf,
	unsigned int buf_length,
	enum dbmdx_audio_channel_operation audio_channel_op)
{

	if (audio_channel_op == AUDIO_CHANNEL_OP_COPY)
		kfifo_in(fifo, buf, buf_length);
	else if (audio_channel_op == AUDIO_CHANNEL_OP_DUPLICATE_1_TO_2) {
		unsigned int i;
		u8 cur_sample_buf[4];

		for (i = 0; i < buf_length - 1; i += 2) {
			cur_sample_buf[0] = buf[i];
			cur_sample_buf[1] = buf[i+1];
			cur_sample_buf[2] = buf[i];
			cur_sample_buf[3] = buf[i+1];
			kfifo_in(fifo, cur_sample_buf, 4);
		}
#ifdef DBMDX_4CHANNELS_SUPPORT
	} else if (audio_channel_op == AUDIO_CHANNEL_OP_DUPLICATE_1_TO_4) {
		unsigned int i;
		u8 cur_sample_buf[8];

		for (i = 0; i < buf_length - 1; i += 2) {
			cur_sample_buf[0] = buf[i];
			cur_sample_buf[1] = buf[i+1];
			cur_sample_buf[2] = buf[i];
			cur_sample_buf[3] = buf[i+1];
			cur_sample_buf[4] = buf[i];
			cur_sample_buf[5] = buf[i+1];
			cur_sample_buf[6] = buf[i];
			cur_sample_buf[7] = buf[i+1];
			kfifo_in(fifo, cur_sample_buf, 8);
		}
	} else if (audio_channel_op == AUDIO_CHANNEL_OP_DUPLICATE_2_TO_4) {
		unsigned int i;
		u8 cur_sample_buf[8];

		for (i = 0; i < buf_length - 3; i += 4) {
			cur_sample_buf[0] = buf[i];
			cur_sample_buf[1] = buf[i+1];
			cur_sample_buf[2] = buf[i+2];
			cur_sample_buf[3] = buf[i+3];
			cur_sample_buf[4] = buf[i];
			cur_sample_buf[5] = buf[i+1];
			cur_sample_buf[6] = buf[i+2];
			cur_sample_buf[7] = buf[i+3];
			kfifo_in(fifo, cur_sample_buf, 8);
		}
#endif
	} else if (audio_channel_op == AUDIO_CHANNEL_OP_TRUNCATE_2_TO_1) {
		unsigned int i;
		u8 cur_sample_buf[2];

		for (i = 0; i < buf_length - 1; i += 4) {
			cur_sample_buf[0] = buf[i];
			cur_sample_buf[1] = buf[i+1];
			kfifo_in(fifo, cur_sample_buf, 2);
		}
#ifdef DBMDX_4CHANNELS_SUPPORT
	} else if (audio_channel_op == AUDIO_CHANNEL_OP_TRUNCATE_4_TO_1) {
		unsigned int i;
		u8 cur_sample_buf[2];

		for (i = 0; i < buf_length - 1; i += 8) {
			cur_sample_buf[0] = buf[i];
			cur_sample_buf[1] = buf[i+1];
			kfifo_in(fifo, cur_sample_buf, 2);
		}
	} else if (audio_channel_op == AUDIO_CHANNEL_OP_TRUNCATE_4_TO_2) {
		unsigned int i;
		u8 cur_sample_buf[4];

		for (i = 0; i < buf_length - 3; i += 8) {
			cur_sample_buf[0] = buf[i];
			cur_sample_buf[1] = buf[i+1];
			cur_sample_buf[2] = buf[i+2];
			cur_sample_buf[3] = buf[i+3];
			kfifo_in(fifo, cur_sample_buf, 4);
		}
#endif
	} else {
		dev_err(p->dev, "%s: Undefined audio channel operation\n",
			__func__);
		return -EIO;
	}

	return 0;
}
#if defined(CONFIG_SND_SOC_DBMDX_SND_CAPTURE)

static int dbmdx_suspend_pcm_streaming_work(struct dbmdx_private *p)
{
	int ret;

	p->va_flags.pcm_worker_active = 0;

	flush_work(&p->pcm_streaming_work);

	if (p->va_flags.pcm_streaming_active) {

		p->va_flags.pcm_streaming_pushing_zeroes = true;

		ret = dbmdx_set_pcm_timer_mode(p->active_substream, true);
		if (ret < 0) {
			dev_err(p->dev,
				"%s: Error switching to pcm timer mode\n",
			__func__);
			return -EIO;
		}
		dev_dbg(p->dev,
			"%s: Switched to pcm timer mode (pushing zeroes)\n",
			__func__);
	}

	return 0;
}

#else
static int dbmdx_suspend_pcm_streaming_work(struct dbmdx_private *p)
{
	return 0;
}
#endif


static int dbmdx_va_set_speed(struct dbmdx_private *p,
			      enum dbmdx_va_speeds speed)
{
	int ret;

	dev_info(p->dev, "%s: set speed to %u\n",
		__func__, speed);

	ret = dbmdx_send_cmd(
		p,
		(DBMDX_REGN_DSP_CLOCK_CONFIG |
		p->pdata->va_speed_cfg[speed].cfg),
		NULL);
	if (ret != 0)
		ret = -EIO;
	return ret;
}

static int dbmdx_buf_to_int(const char *buf)
{
	unsigned long val;
	int ret;

	ret = kstrtoul(buf, 0, &val);
	if (ret)
		return -EINVAL;

	return (int)val;
}


static int check_buffer_cap_for_history(struct dbmdx_private *p, u32 history)
{
	int ret;
	u32 min_buffer_size_in_bytes;
	u32 min_buffer_size_in_chunks;
	u32 audio_buffer_size_in_bytes;
	u16 val;

	if (history <= 2)
		return 0;

	ret = dbmdx_send_cmd(p, DBMDX_REGN_AUDIO_BUFFER_SIZE, &val);
	if (ret < 0) {
		dev_err(p->dev,
			"%s: failed to read DBMDX_VA_AUDIO_BUFFER_SIZE\n",
			__func__);
		return ret;
	}

	min_buffer_size_in_bytes = (p->pdata->va_buffering_pcm_rate / 1000) *
				((u32)history + MIN_EVENT_PROCESSING_TIME_MS) *
				p->pdata->va_audio_channels *
				p->bytes_per_sample;

	min_buffer_size_in_chunks = min_buffer_size_in_bytes /
					(8 * p->bytes_per_sample);

	audio_buffer_size_in_bytes = (u32)val * 8 * p->bytes_per_sample;

	if (audio_buffer_size_in_bytes < min_buffer_size_in_bytes) {

		dev_err(p->dev,	"%s: FW Audio buffer size is not enough\t"
				"for requested backlog size\t"
				"FW buffer size: %u bytes (%u smp. chunks)\t"
				"Min req. buffer size: %u bytes (%u smp. chunks)\n",
				__func__,
				audio_buffer_size_in_bytes,
				(u32)val,
				min_buffer_size_in_bytes,
				min_buffer_size_in_chunks);

		return -EIO;
	}

	dev_dbg(p->dev, "%s: FW Audio buffer size was verified\t"
			"FW buffer size: %u bytes (%u smp. chunks)\t"
			"Min req. buffer size: %u bytes (%u smp. chunks)\n",
			__func__,
			audio_buffer_size_in_bytes,
			(u32)val,
			min_buffer_size_in_bytes,
			min_buffer_size_in_chunks);
	return 0;

}


static int dbmdx_set_backlog_len(struct dbmdx_private *p, u32 history)
{
	int ret;
	int model_selected = 0;
	u16 model_selected_mask = 0;
	int vt_eng_index = 0;
	enum dbmdx_vt_engine_type type;
	bool all_engines_selected = false;
	int i;
	struct vt_engine *sel_vt_eng = NULL;

	model_selected = ((history & 0xF000) >> 12);

	history &= ~(1 << 12);

	if ((model_selected == DBMDX_VT_ENGINE_TYPE_UNDEFINED) ||
		(model_selected >= DBMDX_VT_ENGINE_TYPE_MAX))
		all_engines_selected = true;

	if (!all_engines_selected) {

		type = (enum dbmdx_vt_engine_type)model_selected;

		ret = dbmdx_get_vte_from_model_type(p, type, &sel_vt_eng);

		if (ret < 0) {
			dev_err(p->dev,	"%s: Model %d is not supported\n",
				__func__, model_selected);
			return -EINVAL;
		}
		if (!(sel_vt_eng->enabled)) {
			dev_err(p->dev,	"%s: Model %d is disabled\n",
				__func__, model_selected);
			return -EINVAL;

		}
		vt_eng_index = sel_vt_eng->vte_index;
	} else {
		vt_eng_index = 0; /* ALL Engines */
	}

	dev_info(p->dev, "%s: Model #%d, history 0x%x\n",
					__func__, model_selected, (u32)history);

	/* If history is specified in ms, we should verify that
	 * FW audio buffer size in large enough to contain the history
	 */

	ret = check_buffer_cap_for_history(p, history);

	if (ret < 0) {
		dev_err(p->dev,	"%s: FW Buffer size is not enough\n", __func__);
		return -EINVAL;
	}

	model_selected_mask = (u16)((vt_eng_index << 12) & 0xF000);

	ret = dbmdx_send_cmd(p,
			  DBMDX_REGN_VT_AUDIO_HISTORY_CFG |
			  model_selected_mask | history,
			  NULL);
	if (ret < 0) {
		dev_err(p->dev,
			"%s: failed to set backlog size of model #%d to 0x%x\n",
			__func__, model_selected, (u32)history);
		return ret;
	}

	if (all_engines_selected) {
		for (i = 0; i < DBMDX_MAX_SUPPORTED_VT_ENGINES; i++)
			*(p->vt_engines[i].history) = history;

		dev_info(p->dev, "%s: History was set to 0x%x for All Models\n",
					__func__, (u32)history);

	} else {
		*(p->vt_engines[vt_eng_index - 1].history) = history;

		dev_info(p->dev,
			"%s: History was set to 0x%x for Model %s.\n",
			__func__, (u32)history,
			dbmdx_vte_names[sel_vt_eng->eng_type]);
	}

	return 0;
}

static int dbmdx_sleeping(struct dbmdx_private *p)
{
	return p->asleep;
}

static int dbmdx_force_wake(struct dbmdx_private *p)
{
	int ret = 0;
	u16 resp = 0xffff;

	/* assert wake pin */
	p->wakeup_set(p);


	p->chip->transport_enable(p, true);

	/* give some time to wakeup */
	if (p->active_fw == DBMDX_FW_VA || p->active_fw == DBMDX_FW_VA_VE) {
		/* test if VA firmware is up */
		ret = dbmdx_va_alive(p);
		if (ret < 0) {
			dev_err(p->dev,	"%s: VA fw did not wakeup\n",
				__func__);
			ret = -EIO;
			goto out;
		}

		/* get operation mode register */
		ret = dbmdx_send_cmd(p, DBMDX_REGN_OPERATION_MODE, &resp);
		if (ret < 0) {
			dev_err(p->dev,	"%s: failed to get operation mode\n",
				__func__);
			goto out;
		}
		if (p->active_chip == DBMDX_CHIP_VA)
			p->va_flags.mode = resp;
		else
			p->va_ve_flags.va_ve_chip_mode = resp;
	}

	p->power_mode = DBMDX_PM_ACTIVE;

	/* make it not sleeping */
	if (p->active_chip == DBMDX_CHIP_VA)
		p->asleep_va = false;
	else
		p->asleep_va_ve = false;
	p->asleep = false;


	dev_dbg(p->dev, "%s: woke up\n", __func__);
out:
	return ret;
}

static int dbmdx_wake_va_ve(struct dbmdx_private *p)
{
	int ret = 0;
	int ret2 = 0;
	enum dbmdx_chip	cur_active_chip = p->active_chip;

	if (p->va_ve_chip_enabled && p->asleep_va_ve) {

		ret = dbmdx_switch_to_va_ve_chip_interface(p,
							DBMDX_CMD_INTERFACE);
		if (ret) {
			dev_err(p->dev,
				"%s Error switching to (VA_VE) CMD interface\n",
				__func__);
			ret = -EIO;
			goto out;
		}

		ret = dbmdx_force_wake(p);
		if (ret) {
			dev_err(p->dev,	"%s Cannot wake (VA_VE) chip\n",
				__func__);
			ret = -EIO;
			goto out;
		}
	}

	if (p->va_chip_enabled &&
		(p->asleep_va || p->va_flags.mode == DBMDX_DETECTION)) {

		ret = dbmdx_switch_to_va_chip_interface(p, DBMDX_CMD_INTERFACE);
		if (ret) {
			dev_err(p->dev,
				"%s Error switching to (VA) CMD interface\n",
				__func__);
			ret = -EIO;
			goto out;
		}
		ret = dbmdx_force_wake(p);
		if (ret) {
			dev_err(p->dev,	"%s Cannot wake (VA) chip\n", __func__);
			ret = -EIO;
			goto out;
		}

	}
out:
	if (cur_active_chip == DBMDX_CHIP_VA) {
		ret2 = dbmdx_switch_to_va_chip_interface(p,
						DBMDX_CMD_INTERFACE);
		if (ret2) {
			dev_err(p->dev,
				"%s Error switching to (VA) CMD interface\n",
				__func__);
			ret = -EIO;
		}
	} else {
		ret2 = dbmdx_switch_to_va_ve_chip_interface(p,
						DBMDX_CMD_INTERFACE);
		if (ret2) {
			dev_err(p->dev,
				"%s Error switching to (VA_VE) CMD interface\n",
				__func__);
			ret = -EIO;
		}
	}
	return ret;
}


static int dbmdx_wake(struct dbmdx_private *p)
{
	return dbmdx_wake_va_ve(p);
}

int dbmdx_set_power_mode(
		struct dbmdx_private *p, enum dbmdx_power_modes mode)
{
	int ret = 0;
	enum dbmdx_power_modes new_mode = p->power_mode;

	dev_dbg(p->dev, "%s: would move %s -> %s (%2.2d -> %2.2d)\n",
			__func__,
			dbmdx_power_mode_names[p->power_mode],
			dbmdx_power_mode_names[mode],
			p->power_mode,
			mode);

	switch (p->power_mode) {
	case DBMDX_PM_BOOTING:
		switch (mode) {
		case DBMDX_PM_FALLING_ASLEEP:
			/* queue delayed work to set the chip to sleep*/
			queue_delayed_work(p->dbmdx_workq,
					&p->delayed_pm_work,
					msecs_to_jiffies(100));
			new_mode = mode;
			break;
		case DBMDX_PM_BOOTING:
		case DBMDX_PM_ACTIVE:
			new_mode = mode;
			break;

		default:
			goto illegal_transition;
		}
		break;

	case DBMDX_PM_ACTIVE:
		switch (mode) {
		case DBMDX_PM_ACTIVE:
			if ((p->va_chip_enabled &&
				(p->va_flags.mode == DBMDX_BUFFERING ||
					p->va_flags.mode == DBMDX_DETECTION)) ||
			(p->va_ve_chip_enabled &&
				(p->va_ve_flags.mode == DBMDX_BUFFERING ||
				p->va_ve_flags.mode == DBMDX_DETECTION)))
				ret = dbmdx_wake(p);
			break;

		case DBMDX_PM_FALLING_ASLEEP:
			if (p->sleep_disabled) {
				dev_dbg(p->dev,
					"%s: Sleep mode is blocked\n",
					__func__);
			} else if ((p->va_chip_enabled &&
				!(p->va_flags.sleep_not_allowed)) ||
				(p->va_ve_chip_enabled &&
				!(p->va_ve_flags.sleep_not_allowed))) {
				/* queue delayed work to set the chip to sleep*/
				queue_delayed_work(p->dbmdx_workq,
					&p->delayed_pm_work,
					msecs_to_jiffies(200));
				new_mode = mode;
			} else
				dev_dbg(p->dev,
				"%s: Usecase is streaming, sleep is blocked\n",
						__func__);
			break;
		case DBMDX_PM_BOOTING:
			new_mode = mode;
			break;

		default:
			goto illegal_transition;
		}
		break;

	case DBMDX_PM_FALLING_ASLEEP:
		switch (mode) {
		case DBMDX_PM_BOOTING:
		case DBMDX_PM_ACTIVE:
			/*
			 * flush queue if going to active
			 */
			p->va_flags.cancel_pm_work = true;
			p->unlock(p);
			cancel_delayed_work_sync(&p->delayed_pm_work);
			p->va_flags.cancel_pm_work = false;
			p->lock(p);
			new_mode = mode;
			/* wakeup chip */
			ret = dbmdx_wake(p);
			break;

		case DBMDX_PM_FALLING_ASLEEP:
			break;

		default:
			goto illegal_transition;
		}
		break;

	case DBMDX_PM_SLEEPING:
		/*
		 * wakeup the chip if going to active/booting
		 */
		switch (mode) {
		case DBMDX_PM_FALLING_ASLEEP:
			dev_dbg(p->dev,
				"%s: already sleeping; leave it this way...",
				__func__);
			new_mode = DBMDX_PM_SLEEPING;
			break;
		case DBMDX_PM_ACTIVE:
		case DBMDX_PM_BOOTING:
			ret = dbmdx_wake(p);
			if (ret) {
				dev_err(p->dev,
					"%s: failed to wake the chip up!\n",
					__func__);
				return ret;
			}
		case DBMDX_PM_SLEEPING:
			new_mode = mode;
			break;
		default:
			goto illegal_transition;
		}
		break;

	default:
		dev_err(p->dev, "%s: unknown power mode: %d",
				__func__, p->power_mode);
		return -EINVAL;
	}

	dev_dbg(p->dev, "%s: has moved  %s -> %s (%2.2d -> %2.2d)\n",
			__func__,
			dbmdx_power_mode_names[p->power_mode],
			dbmdx_power_mode_names[new_mode],
			p->power_mode,
			new_mode);

	p->power_mode = new_mode;

	return 0;

illegal_transition:
	dev_err(p->dev, "%s: can't move %s -> %s\n", __func__,
		dbmdx_power_mode_names[p->power_mode],
		dbmdx_power_mode_names[mode]);
	return -EINVAL;
}

static int dbmdx_set_fw_debug_mode(struct dbmdx_private *p,
	enum dbmdx_fw_debug_mode mode)
{
	int ret = 0;

	if (!p)
		return -EAGAIN;

	if (!p->device_ready) {
		dev_err(p->dev, "%s: device not ready\n", __func__);
		return -EAGAIN;
	}

	if (p->active_fw != DBMDX_FW_VA) {
		dev_err(p->dev, "%s: VA FW is no active\n", __func__);
		return -EAGAIN;
	}

	p->lock(p);

	dbmdx_set_power_mode(p, DBMDX_PM_ACTIVE);

	switch (mode) {
	case FW_DEBUG_OUTPUT_UART:
		if (p->active_interface == DBMDX_INTERFACE_UART) {
			dev_err(p->dev, "%s: Not supported in UART mode\n",
				__func__);
			ret = -EIO;
			goto out_pm_mode;
		}

		ret = dbmdx_send_cmd(p,
				(DBMDX_REGN_FW_DEBUG_REGISTER |
				DBMDX_REGV_TOGGLE_UART_DEBUG_PRINTS), NULL);
		if (ret < 0) {
			dev_err(p->dev, "%s: failed to send cmd\n", __func__);
			ret = -EIO;
			goto out_pm_mode;
		}

		break;
	case FW_DEBUG_OUTPUT_CLK:
		ret = dbmdx_send_cmd(p,
				DBMDX_REGN_IO_PORT_ADDR_LO | 0x8, NULL);
		if (ret < 0) {
			dev_err(p->dev, "%s: failed to send cmd\n", __func__);
			ret = -EIO;
			goto out_pm_mode;
		}

		ret = dbmdx_send_cmd(p,
				DBMDX_REGN_FW_DEBUG_REGISTER | 0xB, NULL);
		if (ret < 0) {
			dev_err(p->dev, "%s: failed to send cmd\n", __func__);
			ret = -EIO;
			goto out_pm_mode;
		}

		break;
	case FW_DEBUG_OUTPUT_CLK_UART:
		if (p->active_interface == DBMDX_INTERFACE_UART) {
			dev_err(p->dev, "%s: Not supported in UART mode\n",
				__func__);
			ret = -EIO;
			goto out_pm_mode;
		}

		ret = dbmdx_send_cmd(p,
				DBMDX_REGN_IO_PORT_ADDR_LO | 0x8, NULL);
		if (ret < 0) {
			dev_err(p->dev, "%s: failed to send cmd\n", __func__);
			ret = -EIO;
			goto out_pm_mode;
		}

		ret = dbmdx_send_cmd(p,
				DBMDX_REGN_FW_DEBUG_REGISTER | 0xB, NULL);
		if (ret < 0) {
			dev_err(p->dev, "%s: failed to send cmd\n", __func__);
			ret = -EIO;
			goto out_pm_mode;
		}

		ret = dbmdx_send_cmd(p,
				(DBMDX_REGN_FW_DEBUG_REGISTER |
				DBMDX_REGV_TOGGLE_UART_DEBUG_PRINTS), NULL);
		if (ret < 0) {
			dev_err(p->dev, "%s: failed to send cmd\n", __func__);
			ret = -EIO;
			goto out_pm_mode;
		}

		break;
	default:
		dev_err(p->dev, "%s: Unsupported FW Debug mode 0x%x\n",
			__func__, mode);
		ret = -EINVAL;
		goto out_pm_mode;
	}


out_pm_mode:
	dbmdx_set_power_mode(p, DBMDX_PM_FALLING_ASLEEP);
	p->unlock(p);
	return ret;
}

static void dbmdx_delayed_pm_work_hibernate_va_ve(struct work_struct *work)
{
	int ret;
	struct dbmdx_private *p =
		container_of(work, struct dbmdx_private,
			     delayed_pm_work.work);

	dev_dbg(p->dev, "%s\n", __func__);

	p->lock(p);
	if (p->va_flags.cancel_pm_work) {
		dev_dbg(p->dev,
			"%s: the work has been just canceled\n",
			__func__);
		goto out;
	}
	if (p->sleep_disabled) {
		dev_dbg(p->dev,	"%s: Sleep mode is blocked\n",	__func__);
		goto out;
	}

	if (p->va_chip_enabled) {

		ret = dbmdx_switch_to_va_chip_interface(p,
							DBMDX_CMD_INTERFACE);
		if (ret) {
			dev_err(p->dev,
				"%s Error switching to (VA) CMD interface\n",
				__func__);
			goto out;
		}
		if (dbmdx_can_wakeup(p) && !(p->va_flags.sleep_not_allowed)) {

			p->wakeup_release(p);
			msleep(DBMDX_MSLEEP_HIBARNATE);
			/* set operation mode register */
			ret = dbmdx_send_cmd(p,
					(DBMDX_REGN_OPERATION_MODE |
					DBMDX_HIBERNATE),
				    NULL);

			if (ret) {
				dev_err(p->dev,
				"%s: fail to set to HIBERNATE (VA) - %d\n",
					__func__, ret);
				dbmdx_wake(p);
				goto out;
			}

			msleep(DBMDX_MSLEEP_HIBARNATE);

			p->asleep_va = true;
			p->power_mode = DBMDX_PM_SLEEPING;
			p->va_flags.mode = DBMDX_HIBERNATE;

			p->chip->transport_enable(p, false);
		}
	}

	if (p->va_ve_chip_enabled) {

		ret = dbmdx_switch_to_va_ve_chip_interface(p,
							DBMDX_CMD_INTERFACE);
		if (ret) {
			dev_err(p->dev,
				"%s Error switching to (VA_VE) CMD interface\n",
				__func__);
			goto out;
		}
		if (dbmdx_can_wakeup(p) &&
			!(p->va_ve_flags.sleep_not_allowed)) {

			p->wakeup_release(p);
			msleep(DBMDX_MSLEEP_HIBARNATE);
			/* set operation mode register */
			ret = dbmdx_send_cmd(p,
				    (DBMDX_REGN_OPERATION_MODE |
					 DBMDX_HIBERNATE),
				     NULL);

			if (ret) {
				dev_err(p->dev,
				"%s: fail to set to HIBERNATE (VA_VE) - %d\n",
					__func__, ret);
				dbmdx_wake(p);
				goto out;
			}

			msleep(DBMDX_MSLEEP_HIBARNATE);

			p->asleep_va_ve = true;
			p->power_mode = DBMDX_PM_SLEEPING;
			p->va_ve_flags.mode = DBMDX_HIBERNATE;

			p->chip->transport_enable(p, false);
		}
	}

	if (dbmdx_can_wakeup(p) && p->va_flags.mode == DBMDX_DETECTION) {
		if (p->va_ve_chip_enabled &&
				p->va_ve_flags.mode == DBMDX_HIBERNATE) {

			ret = dbmdx_switch_to_va_chip_interface(p,
							DBMDX_CMD_INTERFACE);
			if (ret) {
				dev_err(p->dev,
				"%s Error switching to (VA) CMD interface\n",
					__func__);
				goto out;
			}

			/* we are in LPM mode, release wakelock! */
			p->chip->transport_enable(p, false);
		}
	}

	p->power_mode = DBMDX_PM_SLEEPING;
out:
	dev_dbg(p->dev, "%s: current power mode: %s\n",
		__func__, dbmdx_power_mode_names[p->power_mode]);
	p->unlock(p);
}

/* Microphone modes */
enum dbmdx_microphone_mode {
	DBMDX_MIC_MODE_DIGITAL_LEFT = 0,
	DBMDX_MIC_MODE_DIGITAL_RIGHT,
	DBMDX_MIC_MODE_DIGITAL_STEREO_TRIG_ON_LEFT,
	DBMDX_MIC_MODE_DIGITAL_STEREO_TRIG_ON_RIGHT,
	DBMDX_MIC_MODE_ANALOG,
#ifdef DBMDX_4CHANNELS_SUPPORT
	DBMDX_MIC_MODE_DIGITAL_4CH,
#endif
	DBMDX_MIC_MODE_DISABLE,
};

enum dbmdx_microphone_type {
	DBMDX_MIC_TYPE_DIGITAL_LEFT = 0,
	DBMDX_MIC_TYPE_DIGITAL_RIGHT,
	DBMDX_MIC_TYPE_ANALOG,
#ifdef DBMDX_4CHANNELS_SUPPORT
	DBMDX_MIC_TYPE_MIC3,
	DBMDX_MIC_TYPE_MIC4,
#endif
};

enum dbmdx_microphone_gain {
	DBMDX_DIGITAL_MIC_DIGITAL_GAIN = 0,
	DBMDX_ANALOG_MIC_ANALOG_GAIN,
	DBMDX_ANALOG_MIC_DIGITAL_GAIN,
};

static int dbmdx_read_fw_vad_settings(struct dbmdx_private *p)
{
	u16 cur_config = 0xffff;
	int ret = 0;

	/* read configuration */
	ret = dbmdx_send_cmd(p,
			     DBMDX_REGN_GENERAL_CONFIG_2,
			     &cur_config);
	if (ret < 0) {
		dev_err(p->dev,
			"%s: failed to read DBMDX_VA_GENERAL_CONFIG_2\n",
			__func__);
		return ret;
	}

	cur_config &= DBMDX_HW_VAD_MASK;
	p->fw_vad_type = ((cur_config >> 5) & 0x3);

	dev_dbg(p->dev, "%s: FW Vad is set to 0x%08x\n",
		__func__, p->fw_vad_type);

	return 0;
}


static int init_sensory_vte(struct dbmdx_private *p, int vt_index)
{
	struct vt_engine *vte;
	u16 vt_select;
	int ret = 0;
	u16 cur_config;

	if (vt_index > 3 || vt_index < 1) {
		dev_err(p->dev, "%s: VT Index %d is out of range\n",
			__func__, vt_index);
		return -EINVAL;
	}

	if (vt_index == 1)
		vt_select = DBMDX_REGV_SELECT_VT_EN_1;
	else if (vt_index == 2)
		vt_select = DBMDX_REGV_SELECT_VT_EN_2;
	else
		vt_select = DBMDX_REGV_SELECT_VT_EN_3;

	vte = &(p->vt_engines[vt_index - 1]);

	vte->mode_support_mask = (DBMDX_VT_MODE_SUPPORT_PHRASE |
					DBMDX_VT_MODE_SUPPORT_VOICE_ENERGY |
					DBMDX_VT_MODE_SUPPORT_VOICE_COMMAND |
					DBMDX_VT_MODE_SUPPORT_VOICE_DUAL);
	vte->vte_index = vt_index;

	vte->amodel_options = p->pdata->amodel_options_sensory;

	vte->eng_type = DBMDX_VT_ENGINE_TYPE_SENSORY;

	if ((vte->amodel_options & DBMDX_AMODEL_INCLUDES_HEADERS) ||
		(vte->amodel_options & DBMDX_AMODEL_SINGLE_FILE_NO_HEADER)) {
		vte->num_of_amodel_files[DETECTION_TYPE_PHRASE]	= 1;
		vte->amodel_fnames[DETECTION_TYPE_PHRASE][0] =
					DBMDX_SENSORY_VT_AMODEL_NAME;
	} else {
		vte->num_of_amodel_files[DETECTION_TYPE_PHRASE] = 2;
		vte->amodel_fnames[DETECTION_TYPE_PHRASE][0] =
						DBMDX_SENSORY_VT_GRAM_NAME;
		vte->amodel_fnames[DETECTION_TYPE_PHRASE][1] =
						DBMDX_SENSORY_VT_NET_NAME;
	}

	vte->num_of_amodel_files[DETECTION_TYPE_VOICE_ENERGY] = 0;

	if ((vte->amodel_options & DBMDX_AMODEL_INCLUDES_HEADERS) ||
		(vte->amodel_options & DBMDX_AMODEL_SINGLE_FILE_NO_HEADER)) {
		vte->num_of_amodel_files[DETECTION_TYPE_VOICE_COMMAND]	= 1;
		vte->amodel_fnames[DETECTION_TYPE_VOICE_COMMAND][0] =
					DBMDX_SENSORY_VC_AMODEL_NAME;
	} else {
		vte->num_of_amodel_files[DETECTION_TYPE_VOICE_COMMAND] = 2;
		vte->amodel_fnames[DETECTION_TYPE_VOICE_COMMAND][0] =
						DBMDX_SENSORY_VC_GRAM_NAME;
		vte->amodel_fnames[DETECTION_TYPE_VOICE_COMMAND][1] =
						DBMDX_SENSORY_VC_NET_NAME;
	}

	if ((vte->amodel_options & DBMDX_AMODEL_INCLUDES_HEADERS) ||
		(vte->amodel_options & DBMDX_AMODEL_SINGLE_FILE_NO_HEADER)) {
		vte->num_of_amodel_files[DETECTION_TYPE_DUAL]	= 1;
		vte->amodel_fnames[DETECTION_TYPE_DUAL][0] =
					DBMDX_SENSORY_VT_AMODEL_NAME;
		vte->num_of_amodel_files_sec = 1;
		vte->amodel_fnames_sec[0] = DBMDX_SENSORY_VC_AMODEL_NAME;

	} else {
		vte->num_of_amodel_files[DETECTION_TYPE_DUAL] = 2;
		vte->amodel_fnames[DETECTION_TYPE_DUAL][0] =
						DBMDX_SENSORY_VT_GRAM_NAME;
		vte->amodel_fnames[DETECTION_TYPE_DUAL][1] =
						DBMDX_SENSORY_VT_NET_NAME;
		vte->num_of_amodel_files_sec = 2;
		vte->amodel_fnames_sec[0] = DBMDX_SENSORY_VC_GRAM_NAME;
		vte->amodel_fnames_sec[1] = DBMDX_SENSORY_VC_NET_NAME;

	}

	/* Frame Size */
	ret = dbmdx_send_cmd(p, DBMDX_REGN_SELECT_VT_INDEX | vt_select, NULL);

	if (ret < 0) {
		dev_err(p->dev, "%s: failed to set VT Eng. index\n", __func__);
		return -EIO;
	}

	if (dbmdx_send_cmd(p, DBMDX_REGN_VT_FRAMING_SIZE, &cur_config) < 0) {
		dev_err(p->dev,
			"%s: failed to read Global Frame Size\n", __func__);
		return -EIO;
	}

	vte->frame_size = (cur_config & DBMDX_REGV_FRAME_SIZE_MASK);

	dev_info(p->dev, "%s: Global Frame size is %u samples\n",
			__func__, vte->frame_size);

	/* History */
	vte->history = &(p->pdata->va_backlog_length_sensory);
	vte->cp = 0;
	vte->custom_params = 0;
	vte->verify_model_load = dbmdx_model_loaded_correctly;
	vte->loaded_to_fw = false;

	vte->enabled = true;

	return 0;

}

static int init_alexa_vte(struct dbmdx_private *p, int vt_index)
{
	struct vt_engine *vte;
	u16 vt_select;
	int ret = 0;
	u16 cur_config;

	if (vt_index > 3 || vt_index < 1) {
		dev_err(p->dev, "%s: VT Index %d is out of range\n",
			__func__, vt_index);
		return -EINVAL;
	}

	if (vt_index == 1)
		vt_select = DBMDX_REGV_SELECT_VT_EN_1;
	else if (vt_index == 2)
		vt_select = DBMDX_REGV_SELECT_VT_EN_2;
	else
		vt_select = DBMDX_REGV_SELECT_VT_EN_3;

	vte = &(p->vt_engines[vt_index - 1]);

	vte->mode_support_mask = (DBMDX_VT_MODE_SUPPORT_PHRASE);
	vte->vte_index = vt_index;

	vte->amodel_options = DBMDX_AMODEL_INCLUDES_HEADERS;

	vte->eng_type = DBMDX_VT_ENGINE_TYPE_ALEXA;

	vte->num_of_amodel_files[DETECTION_TYPE_PHRASE]	= 1;
	vte->amodel_fnames[DETECTION_TYPE_PHRASE][0] =
					DBMDX_ALEXA_VT_AMODEL_NAME;

	/* Frame Size */
	ret = dbmdx_send_cmd(p, DBMDX_REGN_SELECT_VT_INDEX | vt_select, NULL);

	if (ret < 0) {
		dev_err(p->dev, "%s: failed to set VT Eng. index\n", __func__);
		return -EIO;
	}

	if (dbmdx_send_cmd(p, DBMDX_REGN_VT_FRAMING_SIZE, &cur_config) < 0) {
		dev_err(p->dev,
			"%s: failed to read Global Frame Size\n", __func__);
		return -EIO;
	}

	vte->frame_size = (cur_config & DBMDX_REGV_FRAME_SIZE_MASK);

	dev_info(p->dev, "%s: Global Frame size is %u samples\n",
			__func__, vte->frame_size);

	/* History */
	vte->history = &(p->pdata->va_backlog_length_alexa);
	vte->cp = 0;
	vte->custom_params = 0;
	vte->verify_model_load = dbmdx_model_loaded_correctly;
	vte->user_state = MODEL_STATE_UNLOADED;
	vte->internal_state = MODEL_STATE_UNLOADED;
	vte->loaded_to_fw = false;

	vte->enabled = true;

	return 0;

}

static int init_google_vte(struct dbmdx_private *p, int vt_index)
{
	struct vt_engine *vte;
	u16 vt_select;
	int ret = 0;
	u16 cur_config;

	if (vt_index > 3 || vt_index < 1) {
		dev_err(p->dev, "%s: VT Index %d is out of range\n",
			__func__, vt_index);
		return -EINVAL;
	}

	if (vt_index == 1)
		vt_select = DBMDX_REGV_SELECT_VT_EN_1;
	else if (vt_index == 2)
		vt_select = DBMDX_REGV_SELECT_VT_EN_2;
	else
		vt_select = DBMDX_REGV_SELECT_VT_EN_3;

	vte = &(p->vt_engines[vt_index - 1]);

	vte->mode_support_mask = (DBMDX_VT_MODE_SUPPORT_PHRASE);
	vte->vte_index = vt_index;

	vte->amodel_options = DBMDX_AMODEL_INCLUDES_HEADERS;

	vte->eng_type = DBMDX_VT_ENGINE_TYPE_GOOGLE;

	vte->num_of_amodel_files[DETECTION_TYPE_PHRASE]	= 1;
	vte->amodel_fnames[DETECTION_TYPE_PHRASE][0] =
					DBMDX_GOOGLE_VT_AMODEL_NAME;

	/* Frame Size */
	ret = dbmdx_send_cmd(p, DBMDX_REGN_SELECT_VT_INDEX | vt_select, NULL);

	if (ret < 0) {
		dev_err(p->dev, "%s: failed to set VT Eng. index\n", __func__);
		return -EIO;
	}

	if (dbmdx_send_cmd(p, DBMDX_REGN_VT_FRAMING_SIZE, &cur_config) < 0) {
		dev_err(p->dev,
			"%s: failed to read Global Frame Size\n", __func__);
		return -EIO;
	}

	vte->frame_size = (cur_config & DBMDX_REGV_FRAME_SIZE_MASK);

	dev_info(p->dev, "%s: Global Frame size is %u samples\n",
			__func__, vte->frame_size);

	/* History */
	vte->history = &(p->pdata->va_backlog_length_google);
	vte->cp = 0;
	vte->custom_params = 0;
	vte->verify_model_load = dbmdx_model_loaded_correctly;
	vte->user_state = MODEL_STATE_UNLOADED;
	vte->internal_state = MODEL_STATE_UNLOADED;
	vte->loaded_to_fw = false;

	vte->enabled = true;

	return 0;

}

static int init_bixby_vte(struct dbmdx_private *p, int vt_index)
{
	struct vt_engine *vte;
	u16 vt_select;
	int ret = 0;
	u16 cur_config;

	if (vt_index > 3 || vt_index < 1) {
		dev_err(p->dev, "%s: VT Index %d is out of range\n",
			__func__, vt_index);
		return -EINVAL;
	}

	if (vt_index == 1)
		vt_select = DBMDX_REGV_SELECT_VT_EN_1;
	else if (vt_index == 2)
		vt_select = DBMDX_REGV_SELECT_VT_EN_2;
	else
		vt_select = DBMDX_REGV_SELECT_VT_EN_3;

	vte = &(p->vt_engines[vt_index - 1]);

	vte->mode_support_mask = (DBMDX_VT_MODE_SUPPORT_PHRASE |
					DBMDX_VT_MODE_SUPPORT_VOICE_ENERGY);
	vte->vte_index = vt_index;

	vte->amodel_options = (DBMDX_AMODEL_INCLUDES_HEADERS |
				DBMDX_VE_SEND_DUMMY_AMODEL_4B);

	vte->eng_type = DBMDX_VT_ENGINE_TYPE_BIXBY;


	vte->num_of_amodel_files[DETECTION_TYPE_PHRASE]	= 1;
	vte->amodel_fnames[DETECTION_TYPE_PHRASE][0] =
					DBMDX_BIXBY_VT_AMODEL_NAME;

	vte->num_of_amodel_files[DETECTION_TYPE_VOICE_ENERGY] = 0;

	/* Frame Size */
	ret = dbmdx_send_cmd(p, DBMDX_REGN_SELECT_VT_INDEX | vt_select, NULL);

	if (ret < 0) {
		dev_err(p->dev, "%s: failed to set VT Eng. index\n", __func__);
		return -EIO;
	}

	if (dbmdx_send_cmd(p, DBMDX_REGN_VT_FRAMING_SIZE, &cur_config) < 0) {
		dev_err(p->dev,
			"%s: failed to read Global Frame Size\n", __func__);
		return -EIO;
	}

	vte->frame_size = (cur_config & DBMDX_REGV_FRAME_SIZE_MASK);

	dev_info(p->dev, "%s: Global Frame size is %u samples\n",
			__func__, vte->frame_size);

	/* History */
	vte->history = &(p->pdata->va_backlog_length_bixby);
	vte->cp = 0;
	vte->custom_params = 0;
	vte->verify_model_load = dbmdx_model_loaded_correctly;
	vte->user_state = MODEL_STATE_UNLOADED;
	vte->internal_state = MODEL_STATE_UNLOADED;
	vte->loaded_to_fw = false;

	vte->enabled = true;

	return 0;

}

static int init_custom_vte(struct dbmdx_private *p, int vt_index)
{
	struct vt_engine *vte;
	u16 vt_select;
	int ret = 0;
	u16 cur_config;

	if (vt_index > 3 || vt_index < 1) {
		dev_err(p->dev, "%s: VT Index %d is out of range\n",
			__func__, vt_index);
		return -EINVAL;
	}

	if (vt_index == 1)
		vt_select = DBMDX_REGV_SELECT_VT_EN_1;
	else if (vt_index == 2)
		vt_select = DBMDX_REGV_SELECT_VT_EN_2;
	else
		vt_select = DBMDX_REGV_SELECT_VT_EN_3;

	vte = &(p->vt_engines[vt_index - 1]);

	vte->mode_support_mask = (DBMDX_VT_MODE_SUPPORT_PHRASE |
					DBMDX_VT_MODE_SUPPORT_VOICE_ENERGY |
					DBMDX_VT_MODE_SUPPORT_VOICE_COMMAND |
					DBMDX_VT_MODE_SUPPORT_VOICE_DUAL);
	vte->vte_index = vt_index;

	vte->amodel_options = p->pdata->amodel_options;

	vte->eng_type = DBMDX_VT_ENGINE_TYPE_GENERAL;

	if ((vte->amodel_options & DBMDX_AMODEL_INCLUDES_HEADERS) ||
		(vte->amodel_options & DBMDX_AMODEL_SINGLE_FILE_NO_HEADER)) {
		vte->num_of_amodel_files[DETECTION_TYPE_PHRASE]	= 1;
		vte->amodel_fnames[DETECTION_TYPE_PHRASE][0] =
					DBMDX_VT_AMODEL_NAME;
	} else {
		vte->num_of_amodel_files[DETECTION_TYPE_PHRASE] = 2;
		vte->amodel_fnames[DETECTION_TYPE_PHRASE][0] =
						DBMDX_VT_GRAM_NAME;
		vte->amodel_fnames[DETECTION_TYPE_PHRASE][1] =
						DBMDX_VT_NET_NAME;
	}

	vte->num_of_amodel_files[DETECTION_TYPE_VOICE_ENERGY] = 0;

	if ((vte->amodel_options & DBMDX_AMODEL_INCLUDES_HEADERS) ||
		(vte->amodel_options & DBMDX_AMODEL_SINGLE_FILE_NO_HEADER)) {
		vte->num_of_amodel_files[DETECTION_TYPE_VOICE_COMMAND]	= 1;
		vte->amodel_fnames[DETECTION_TYPE_VOICE_COMMAND][0] =
					DBMDX_VC_AMODEL_NAME;
	} else {
		vte->num_of_amodel_files[DETECTION_TYPE_VOICE_COMMAND] = 2;
		vte->amodel_fnames[DETECTION_TYPE_VOICE_COMMAND][0] =
						DBMDX_VC_GRAM_NAME;
		vte->amodel_fnames[DETECTION_TYPE_VOICE_COMMAND][1] =
						DBMDX_VC_NET_NAME;
	}

	if ((vte->amodel_options & DBMDX_AMODEL_INCLUDES_HEADERS) ||
		(vte->amodel_options & DBMDX_AMODEL_SINGLE_FILE_NO_HEADER)) {
		vte->num_of_amodel_files[DETECTION_TYPE_DUAL]	= 1;
		vte->amodel_fnames[DETECTION_TYPE_DUAL][0] =
					DBMDX_VT_AMODEL_NAME;
		vte->num_of_amodel_files_sec = 1;
		vte->amodel_fnames_sec[0] = DBMDX_VC_AMODEL_NAME;

	} else {
		vte->num_of_amodel_files[DETECTION_TYPE_DUAL] = 2;
		vte->amodel_fnames[DETECTION_TYPE_DUAL][0] =
						DBMDX_VT_GRAM_NAME;
		vte->amodel_fnames[DETECTION_TYPE_DUAL][1] =
						DBMDX_VT_NET_NAME;
		vte->num_of_amodel_files_sec = 2;
		vte->amodel_fnames_sec[0] = DBMDX_VC_GRAM_NAME;
		vte->amodel_fnames_sec[1] = DBMDX_VC_NET_NAME;

	}

	/* Frame Size */
	ret = dbmdx_send_cmd(p, DBMDX_REGN_SELECT_VT_INDEX | vt_select, NULL);

	if (ret < 0) {
		dev_err(p->dev, "%s: failed to set VT Eng. index\n", __func__);
		return -EIO;
	}

	if (dbmdx_send_cmd(p, DBMDX_REGN_VT_FRAMING_SIZE, &cur_config) < 0) {
		dev_err(p->dev,
			"%s: failed to read Global Frame Size\n", __func__);
		return -EIO;
	}

	vte->frame_size = (cur_config & DBMDX_REGV_FRAME_SIZE_MASK);

	dev_info(p->dev, "%s: Global Frame size is %u samples\n",
			__func__, vte->frame_size);

	/* History */
	vte->history = &(p->pdata->va_backlog_length);
	vte->cp = 0;
	vte->custom_params = 0;
	vte->verify_model_load = dbmdx_model_loaded_correctly;
	vte->user_state = MODEL_STATE_UNLOADED;
	vte->internal_state = MODEL_STATE_UNLOADED;
	vte->loaded_to_fw = false;
	vte->enabled = true;

	return 0;
}


static int init_empty_vte(struct dbmdx_private *p, int vt_index)
{
	struct vt_engine *vte;
	u16 vt_select;

	if (vt_index > 3 || vt_index < 1) {
		dev_err(p->dev, "%s: VT Index %d is out of range\n",
			__func__, vt_index);
		return -EINVAL;
	}

	if (vt_index == 1)
		vt_select = DBMDX_REGV_SELECT_VT_EN_1;
	else if (vt_index == 2)
		vt_select = DBMDX_REGV_SELECT_VT_EN_2;
	else
		vt_select = DBMDX_REGV_SELECT_VT_EN_3;

	vte = &(p->vt_engines[vt_index - 1]);

	vte->mode_support_mask = 0;

	vte->vte_index = vt_index;

	vte->eng_type = DBMDX_VT_ENGINE_TYPE_UNDEFINED;

	vte->enabled = false;
	vte->user_state = MODEL_STATE_UNLOADED;
	vte->internal_state = MODEL_STATE_UNLOADED;
	vte->loaded_to_fw = false;

	return 0;
}

static int free_vte(struct dbmdx_private *p, struct vt_engine *vte)
{
	struct amodel_info *cur_model;

	if (!vte)
		return 0;

	cur_model = &(vte->primary_model);

	if (cur_model->amodel_buf) {
		vfree(cur_model->amodel_buf);
		cur_model->amodel_buf = NULL;
		cur_model->amodel_loaded = false;
	}

	cur_model = &(vte->secondary_model);

	if (cur_model->amodel_buf) {
		vfree(cur_model->amodel_buf);
		cur_model->amodel_buf = NULL;
		cur_model->amodel_loaded = false;
	}

	cur_model = &(vte->ve_model);

	if (cur_model->amodel_buf) {
		vfree(cur_model->amodel_buf);
		cur_model->amodel_buf = NULL;
		cur_model->amodel_loaded = false;
	}

	cur_model = &(vte->vc_model);

	if (cur_model->amodel_buf) {
		vfree(cur_model->amodel_buf);
		cur_model->amodel_buf = NULL;
		cur_model->amodel_loaded = false;
	}

	vte->mode_support_mask = 0;

	vte->eng_type = DBMDX_VT_ENGINE_TYPE_UNDEFINED;

	vte->enabled = false;

	return 0;
}


static int dbmdx_verify_model_support(struct dbmdx_private *p)
{

	u16 cur_config = 0;
	int cur_vt_index = 0;
	u16 cur_vt_model_support;
	int number_of_supported_vt_eng = 0;
	int ret = 0;

	if (dbmdx_send_cmd(p, DBMDX_REGN_VT_MAPPING, &cur_config) < 0) {
		dev_err(p->dev,
			"%s: failed to read DBMDX_VA_FEATURE_SUPPORT\n",
			__func__);
		return -EIO;
	}

	p->vt_engines[0].enabled = false;
	p->vt_engines[1].enabled = false;
	p->vt_engines[2].enabled = false;

	for (cur_vt_index = 1; cur_vt_index < 4; cur_vt_index++) {

		cur_vt_model_support = (u16)((cur_config >>
					((cur_vt_index - 1) * 4)) & 0x000F);

		switch (cur_vt_model_support) {
		case DBMDX_REGV_VT_ENG_SENSORY:
			ret = init_sensory_vte(p, cur_vt_index);
			break;
		case DBMDX_REGV_VT_ENG_ALEXA:
			ret = init_alexa_vte(p, cur_vt_index);
			break;
		case DBMDX_REGV_VT_ENG_GOOGLE:
			ret = init_google_vte(p, cur_vt_index);
			break;
		case DBMDX_REGV_VT_ENG_SAMSUNG:
			ret = init_bixby_vte(p, cur_vt_index);
			break;
		case DBMDX_REGV_VT_ENG_NONE:
			ret = init_empty_vte(p, cur_vt_index);
			break;
		default:
			ret = init_custom_vte(p, cur_vt_index);
			break;
		}

		if (ret < 0) {
			dev_err(p->dev, "%s: Failed to init VTE #%d\n",
							__func__, cur_vt_index);
			return -EIO;

		}
		if (cur_vt_model_support != DBMDX_REGV_VT_ENG_NONE)
			number_of_supported_vt_eng++;
	}

	dev_info(p->dev, "%s: %d VTE Engines are supported\n",
					__func__, number_of_supported_vt_eng);

	ret = dbmdx_send_cmd(p, DBMDX_REGN_VT_FRAMING_SIZE |
					DBMDX_REGV_SELECT_VT_EN_GLOBAL, NULL);

	if (dbmdx_send_cmd(p, DBMDX_REGN_VT_FRAMING_SIZE, &cur_config) < 0) {
		dev_err(p->dev,
			"%s: failed to read Global Frame Size\n", __func__);
		return -EIO;
	}

	dev_info(p->dev, "%s: Global Frame size is %u samples\n",
			__func__, (cur_config & DBMDX_REGV_FRAME_SIZE_MASK));

	/* FW doesn't support VTE */
	if (!number_of_supported_vt_eng) {
		dev_dbg(p->dev,
			"%s: Cannot verify VTE support, no supported engines\n",
			__func__);
		return -EIO;
	}

	return 0;
}

static int dbmdx_disable_gain_normalization(struct dbmdx_private *p)
{
	u16 cur_config = 0xffff;
	u16 upd_config = 0xffff;
	int ret = 0;

	if (p->va_flags.gain_normalization_disabled) {
		dev_dbg(p->dev,
		"%s: The Gain normalization is already disabled, do nothing\n",
			__func__);
		return 0;
	}

	if (dbmdx_switch_to_va_chip_interface(p, DBMDX_CMD_INTERFACE)) {
		dev_err(p->dev,
			"%s Error switching to (VA) CMD interface\n",
			__func__);
		return -EIO;
	}

	/* read configuration */
	ret = dbmdx_send_cmd(p,
			     DBMDX_REGN_BUFFERING_NORMAL_AMPLITUDE,
			     &cur_config);
	if (ret < 0) {
		dev_err(p->dev,
		"%s: failed to read DBMDX_REGN_BUFFERING_NORMAL_AMPLITUDE\n",
			__func__);
		return ret;
	}

	ret = dbmdx_send_cmd(p,
			     DBMDX_REGN_BUFFERING_NORMAL_AMPLITUDE,
			     NULL);
	if (ret < 0) {
		dev_err(p->dev,
		"%s: failed to set DBMDX_REGN_BUFFERING_NORMAL_AMPLITUDE\n",
			__func__);
		return ret;
	}

	/* read configuration */
	ret = dbmdx_send_cmd(p,
			     DBMDX_REGN_BUFFERING_NORMAL_AMPLITUDE,
			     &upd_config);
	if (ret < 0) {
		dev_err(p->dev,
		"%s: failed to read DBMDX_REGN_BUFFERING_NORMAL_AMPLITUDE\n",
			__func__);
		return ret;
	}

	if (upd_config != 0) {
		dev_err(p->dev,
		"%s: failed to update DBMDX_REGN_BUFFERING_NORMAL_AMPLITUDE\n",
			__func__);
		return -EIO;
	}

	p->va_flags.gain_normalization_val = cur_config;

	p->va_flags.gain_normalization_disabled = true;


	dev_dbg(p->dev, "%s: Gain Normalization is disabled\n", __func__);

	return 0;
}

static int dbmdx_restore_gain_normalization(struct dbmdx_private *p)
{
	int ret = 0;
	u16 upd_config = 0xffff;

	if (!(p->va_flags.gain_normalization_disabled)) {
		dev_dbg(p->dev,
		"%s: The Gain normalization is not disabled, do nothing\n",
			__func__);
		return 0;
	}

	if (dbmdx_switch_to_va_chip_interface(p, DBMDX_CMD_INTERFACE)) {
		dev_err(p->dev,
			"%s Error switching to (VA) CMD interface\n",
			__func__);
		return -EIO;
	}

	ret = dbmdx_send_cmd(p,
			     (DBMDX_REGN_BUFFERING_NORMAL_AMPLITUDE |
			     p->va_flags.gain_normalization_val),
			     NULL);
	if (ret < 0) {
		dev_err(p->dev,
		"%s: failed to set DBMDX_REGN_BUFFERING_NORMAL_AMPLITUDE\n",
			__func__);
		return ret;
	}
	/* read configuration */
	ret = dbmdx_send_cmd(p,
			     DBMDX_REGN_BUFFERING_NORMAL_AMPLITUDE,
			     &upd_config);
	if (ret < 0) {
		dev_err(p->dev,
		"%s: failed to read DBMDX_REGN_BUFFERING_NORMAL_AMPLITUDE\n",
			__func__);
		return ret;
	}


	if (upd_config != p->va_flags.gain_normalization_val) {
		dev_err(p->dev,
		"%s: failed to update DBMDX_REGN_BUFFERING_NORMAL_AMPLITUDE\n",
			__func__);
		return -EIO;
	}

	p->va_flags.gain_normalization_disabled = false;

	dev_dbg(p->dev, "%s: Gain Normalization is restored\n", __func__);

	return 0;
}


static int dbmdx_get_fw_errors(struct dbmdx_private *p, enum dbmdx_chip chip,
				u16 *num_of_errors, u32 *err_code)
{
	int ret = 0;


	if (chip == DBMDX_CHIP_VA && !p->va_chip_enabled) {
		dev_err(p->dev, "%s: VA CHIP is disabled", __func__);
		return -EINVAL;
	}

	if (chip == DBMDX_CHIP_VA_VE && !p->va_ve_chip_enabled) {
		dev_err(p->dev, "%s: VA_VE CHIP is disabled", __func__);
		return -EINVAL;
	}

	if (chip == DBMDX_CHIP_VA_VE) {

		ret = dbmdx_switch_to_va_ve_chip_interface(p,
						DBMDX_CMD_INTERFACE);
		if (ret) {
			dev_err(p->dev,
				"%s Error switching to VA_VE interface\n",
				__func__);
			return -EIO;
		}
	} else {
		ret = dbmdx_switch_to_va_chip_interface(p,
						DBMDX_CMD_INTERFACE);
		if (ret) {
			dev_err(p->dev,
				"%s Error switching to VA interface\n",
				__func__);
			return -EIO;
		}

	}

	ret = dbmdx_send_cmd(p,	(DBMDX_REGN_FW_STATUS_INDEX |
					DBMDX_REGV_READ_FW_ERROR_COUNTER),
					NULL);
	if (ret < 0) {
		dev_err(p->dev,	"%s: failed to write reg\n", __func__);
		return -EIO;
	}

	ret = dbmdx_send_cmd(p, DBMDX_REGN_IO_PORT_ADDR_LO, num_of_errors);
	if (ret < 0) {
		dev_err(p->dev, "%s: Error reading number of errors\n",
				__func__);
		return -EIO;
	}

	if (*num_of_errors > 0)	{
		u16 err_code_low, err_code_hi;

		ret = dbmdx_send_cmd(p, DBMDX_REGN_IO_PORT_ADDR_HI,
							&err_code_low);

		if (ret < 0) {
			dev_err(p->dev, "%s: Error reading error code\n",
					__func__);
			return -EIO;
		}
		ret = dbmdx_send_cmd(p, DBMDX_REGN_IO_PORT_VALUE_HI,
							&err_code_hi);

		if (ret < 0) {
			dev_err(p->dev, "%s: Error reading error code\n",
					__func__);
			return -EIO;
		}

		*err_code = (u32)((((u32)(err_code_hi) << 16) & 0xFFFF0000) +
				(u32)(((u32)(err_code_low) & 0xFFFF)));
	}

	return 0;
}

static int dbmdx_set_pcm_streaming_mode(struct dbmdx_private *p, u16 mode)
{
	u16 cur_config = 0xffff;
	u16 cur_mode = 0;
	int ret = 0;

	/* read configuration */
	ret = dbmdx_send_cmd(p,
			     DBMDX_REGN_GENERAL_CONFIG_2,
			     &cur_config);
	if (ret < 0) {
		dev_err(p->dev,
			"%s: failed to read DBMDX_VA_GENERAL_CONFIG_2\n",
			__func__);
		return ret;
	}

	if (mode > 1)
		mode = 1;

	cur_mode = ((cur_config >> 12) & 0x1);

	if (cur_mode == mode) {
		dev_dbg(p->dev,
		"%s: Current PCM Strm. mode is already %d, no need to set\n",
		__func__, cur_mode);
		return 0;

	}

	cur_config &= ~(1 << 12);
	cur_config |= (mode << 12);

	ret = dbmdx_send_cmd(p,
			     DBMDX_REGN_GENERAL_CONFIG_2 | cur_config,
			     NULL);

	if (ret < 0) {
		dev_err(p->dev,
			"%s: failed to set DBMDX_VA_GENERAL_CONFIG_2\n",
			__func__);
		return ret;
	}

	dev_dbg(p->dev, "%s: PCM Streaming mode: %d, Reg 0x23: (0x%08x)\n",
		__func__, mode, cur_config);

	return 0;
}

static int dbmdx_calc_amodel_checksum(struct dbmdx_private *p,
				      const char *amodel,
				      unsigned long len, unsigned long *chksum)
{
	unsigned long sum = 0;
	u16 val;
	unsigned long i;
	u32 pos = 0, chunk_len;
	int err = -1;

	*chksum = 0;

	while (pos < len) {
		val = *(u16 *)(&amodel[pos]);
		pos += 2;
		if (pos >= len) {
			dev_dbg(p->dev, "%s:%d %u", __func__,
				__LINE__, pos);
			return err;
		}

		if (val == 0x025a) {
			sum += 0x5a + 0x02;

			chunk_len = *(u32 *)(&amodel[pos]);
			pos += 4;
			if (pos >= len) {
				dev_dbg(p->dev, "%s:%d %u", __func__,
					__LINE__, pos);
				return err;
			}

			sum += chunk_len;

			sum += *(u32 *)(&amodel[pos]);
			pos += 4;

			if ((pos + (chunk_len * 2)) > len) {
				dev_dbg(p->dev, "%s:%d %u, %u",
					__func__, __LINE__, pos, chunk_len);
				return err;
			}

			for (i = 0; i < chunk_len; i++) {
				sum += *(u16 *)(&amodel[pos]);
				pos += 2;
			}
		} else
			continue;
	}

	sum += 0x5A + 0x0e;
	*chksum = sum;

	return 0;
}

static ssize_t dbmdx_acoustic_model_build_gram_net_no_headers(
					struct dbmdx_private *p,
					const u8 *gram_data,
					size_t gram_size,
					u32 gram_addr,
					const u8 *net_data,
					size_t net_size,
					char	*amodel_buf,
					ssize_t *amodel_size)
{
	unsigned char head[DBMDX_AMODEL_HEADER_SIZE] = { 0 };
	size_t pos;
	unsigned long checksum;
	int ret;
	u32 net_addr = 0x2;
	ssize_t head_size = DBMDX_AMODEL_HEADER_SIZE;

	pos = 0;
	if (gram_addr == 0x1) {
		head[0] = 0x0;
		head[1] = 0x0;
		head[2] = 0x5A;
		head[3] = 0x02;
		head[4] =  (gram_size/2)        & 0xff;
		head[5] = ((gram_size/2) >>  8) & 0xff;
		head[6] = ((gram_size/2) >> 16) & 0xff;
		head[7] = ((gram_size/2) >> 24) & 0xff;
		head[8] = (gram_addr)        & 0xff;
		head[9] = ((gram_addr) >>  8) & 0xff;
		head[10] = ((gram_addr) >> 16) & 0xff;
		head[11] = ((gram_addr) >> 24) & 0xff;
	} else {
		head[0] = 0x5A;
		head[1] = 0x02;
		head[2] =  ((gram_size/2)+1)        & 0xff;
		head[3] = (((gram_size/2)+1) >>  8) & 0xff;
		head[4] = (((gram_size/2)+1) >> 16) & 0xff;
		head[5] = (((gram_size/2)+1) >> 24) & 0xff;
		head[6] = (gram_addr)        & 0xff;
		head[7] = ((gram_addr) >>  8) & 0xff;
		head[8] = ((gram_addr) >> 16) & 0xff;
		head[9] = ((gram_addr) >> 24) & 0xff;
		head[10] =  (gram_size/2)        & 0xff;
		head[11] = ((gram_size/2) >>  8) & 0xff;
	}

	memcpy(amodel_buf, head, head_size);

	pos += head_size;

	if (pos + gram_size > MAX_AMODEL_SIZE) {
		dev_err(p->dev,
			"%s: adding gram exceeds max size %zd>%d\n",
			__func__, pos + gram_size + 6, MAX_AMODEL_SIZE);
		ret = -EINVAL;
		goto out;
	}

	memcpy(amodel_buf + pos, gram_data, gram_size);

	pos += gram_size;

	if (gram_addr != 0x1)
		net_addr = gram_addr + (gram_size)/2 + 1;

	if (gram_addr == 0x1) {
		head[0] = 0x0;
		head[1] = 0x0;
		head[2] = 0x5A;
		head[3] = 0x02;
		head[4] =  (net_size/2)        & 0xff;
		head[5] = ((net_size/2) >>  8) & 0xff;
		head[6] = ((net_size/2) >> 16) & 0xff;
		head[7] = ((net_size/2) >> 24) & 0xff;
		head[8] = (net_addr)        & 0xff;
		head[9] = ((net_addr) >>  8) & 0xff;
		head[10] = ((net_addr) >> 16) & 0xff;
		head[11] = ((net_addr) >> 24) & 0xff;
	} else {
		head[0] = 0x5A;
		head[1] = 0x02;
		head[2] =  ((net_size/2)+1)        & 0xff;
		head[3] = (((net_size/2)+1) >>  8) & 0xff;
		head[4] = (((net_size/2)+1) >> 16) & 0xff;
		head[5] = (((net_size/2)+1) >> 24) & 0xff;
		head[6] = (net_addr)        & 0xff;
		head[7] = ((net_addr) >>  8) & 0xff;
		head[8] = ((net_addr) >> 16) & 0xff;
		head[9] = ((net_addr) >> 24) & 0xff;
		head[10] =  (net_size/2)        & 0xff;
		head[11] = ((net_size/2) >>  8) & 0xff;
	}

	memcpy(amodel_buf + pos, head, head_size);

	pos += head_size;

	if (pos + net_size + 6 > MAX_AMODEL_SIZE) {
		dev_err(p->dev,
			"%s: adding net exceeds max size %zd>%d\n",
			__func__, pos + net_size + 6, MAX_AMODEL_SIZE);
		ret = -EINVAL;
		goto out;
	}

	memcpy(amodel_buf + pos, net_data, net_size);

	ret = dbmdx_calc_amodel_checksum(p,
					(char *)amodel_buf,
					pos + net_size,
					&checksum);
	if (ret) {
		dev_err(p->dev, "%s: failed to calculate Amodel checksum\n",
			__func__);
		ret = -EINVAL;
		goto out;
	}

	*(unsigned long *)(amodel_buf + pos + net_size) = checksum;

	*amodel_size = (ssize_t)(pos + net_size + 4);

	ret = *amodel_size;

out:
	return ret;
}
static ssize_t dbmdx_acoustic_model_build_single_no_headers(
					struct dbmdx_private *p,
					const u8 *model_data,
					size_t model_size,
					u32 addr,
					char	*amodel_buf,
					ssize_t *amodel_size)
{
	unsigned char head[DBMDX_AMODEL_HEADER_SIZE] = { 0 };
	size_t pos;
	unsigned long checksum;
	int ret;
	ssize_t head_size = DBMDX_AMODEL_HEADER_SIZE;

	pos = 0;
	head[0] = 0x0;
	head[1] = 0x0;
	head[2] = 0x5A;
	head[3] = 0x02;
	head[4] =  (model_size/2)        & 0xff;
	head[5] = ((model_size/2) >>  8) & 0xff;
	head[6] = ((model_size/2) >> 16) & 0xff;
	head[7] = ((model_size/2) >> 24) & 0xff;
	head[8] = (addr)        & 0xff;
	head[9] = ((addr) >>  8) & 0xff;
	head[10] = ((addr) >> 16) & 0xff;
	head[11] = ((addr) >> 24) & 0xff;

	memcpy(amodel_buf, head, head_size);

	pos += head_size;

	if (pos + model_size > MAX_AMODEL_SIZE) {
		dev_err(p->dev,
			"%s: model exceeds max size %zd>%d\n",
			__func__, pos + model_size + 6, MAX_AMODEL_SIZE);
		ret = -EINVAL;
		goto out;
	}

	memcpy(amodel_buf + pos, model_data, model_size);

	pos += model_size;

	ret = dbmdx_calc_amodel_checksum(p,
					(char *)amodel_buf,
					pos,
					&checksum);
	if (ret) {
		dev_err(p->dev, "%s: failed to calculate Amodel checksum\n",
			__func__);
		ret = -EINVAL;
		goto out;
	}

	*(unsigned long *)(amodel_buf + pos) = checksum;

	*amodel_size = (ssize_t)(pos + 4);

	ret = *amodel_size;

out:
	return ret;
}


static ssize_t dbmdx_acoustic_model_build_from_multichunk_file(
					struct dbmdx_private *p,
					const u8 *file_data,
					ssize_t file_size,
					char	*amodel_buf,
					ssize_t *amodel_size,
					int	*num_of_amodel_chunks,
					ssize_t *amodel_chunks_size)
{
	unsigned char head[DBMDX_AMODEL_HEADER_SIZE] = { 0 };
	size_t target_pos, src_pos;
	unsigned long checksum;
	int ret;
	ssize_t head_size = DBMDX_AMODEL_HEADER_SIZE;
	ssize_t enc_head_size = DBMDX_AMODEL_HEADER_SIZE - 2;
	size_t encoded_size;

	src_pos = 0;
	target_pos = 0;
	*num_of_amodel_chunks = 0;

	while (src_pos < file_size) {

		if ((file_size - src_pos) < enc_head_size)
			break;

		if (*num_of_amodel_chunks >= DBMDX_AMODEL_MAX_CHUNKS) {
			dev_warn(p->dev,
				"%s: Reached Max number of Amodel chunks\n",
				__func__);
			break;
		}

		if (file_data[src_pos] != 0x5A ||
				file_data[src_pos+1] != 0x02) {
			src_pos += 2;
			continue;
		}

		head[0] = 0x0;
		head[1] = 0x0;

		memcpy(head + 2, file_data + src_pos, enc_head_size);

		encoded_size = (size_t)(head[4] | (head[5]<<8) |
					(head[6]<<16) | (head[7]<<24)) * 2;

		src_pos += enc_head_size;

		if (encoded_size > (file_size - src_pos)) {
			dev_err(p->dev,	"%s: Encoded size > File size\n",
					__func__);
			ret = -EINVAL;
			goto out;
		}

		memcpy(amodel_buf + target_pos, head, head_size);

		target_pos += head_size;

		if (target_pos + encoded_size + 6 > MAX_AMODEL_SIZE) {
			dev_err(p->dev,
				"%s: adding chunk exceeds max size %zd>%d\n",
				__func__,
				target_pos + encoded_size + 6, MAX_AMODEL_SIZE);
			ret = -EINVAL;
			goto out;
		}

		memcpy(amodel_buf + target_pos, file_data + src_pos,
			encoded_size);


		src_pos += encoded_size;
		target_pos += encoded_size;
		amodel_chunks_size[*num_of_amodel_chunks] = encoded_size;

		dev_info(p->dev,
			"%s: Added chunk #%d, (%d bytes), target_pos=%d\n",
				__func__, *num_of_amodel_chunks,
				(int)encoded_size, (int)target_pos);

		*num_of_amodel_chunks = *num_of_amodel_chunks + 1;

	}

	ret = dbmdx_calc_amodel_checksum(p,
					(char *)amodel_buf,
					target_pos,
					&checksum);
	if (ret) {
		dev_err(p->dev, "%s: failed to calculate Amodel checksum\n",
			__func__);
		ret = -EINVAL;
		goto out;
	}

	*(unsigned long *)(amodel_buf + target_pos) = checksum;

	*amodel_size = (ssize_t)(target_pos + 4);

	ret = *amodel_size;

out:
	return ret;
}

static ssize_t dbmdx_acoustic_model_build_from_svt_multichunk_file(
					struct dbmdx_private *p,
					const u8 *file_data,
					ssize_t file_size,
					char	*amodel_buf,
					ssize_t *amodel_size,
					int	*num_of_amodel_chunks,
					ssize_t *amodel_chunks_size)
{
	unsigned char head[DBMDX_AMODEL_HEADER_SIZE] = { 0 };
	size_t target_pos;
	unsigned long checksum;
	int ret;
	ssize_t head_size = DBMDX_AMODEL_HEADER_SIZE;
	size_t gram_size, net_size;
	u32 gram_addr = 0x1;
	u32 net_addr = 0x2;
	int i = 0;

	target_pos = 0;

	if (file_size < 4) {
		dev_err(p->dev, "%s: File size is too small\n", __func__);
		ret = -EINVAL;
		goto out;
	}

	 /* Check if it is special file that contains all zeroes for loading
	  * dummy model
	  */
	for (i = 0; i < file_size; i++)
		if (file_data[i])
			break;

	if (i == file_size) {
		dev_info(p->dev, "%s: Detected svt dummy model file\n",
				__func__);
		return dbmdx_va_amodel_load_dummy_model(p,
							0x1,
							amodel_buf,
							amodel_size,
							num_of_amodel_chunks,
							amodel_chunks_size);
	}

	 /* File format is:
	 * 4 bytes net_size + net_data + 4 bytes gram_size + gram_data
	 */

	/* The size is encoded in bytes */
	net_size = (size_t)(file_data[0] | (file_data[1]<<8) |
				(file_data[2]<<16) | (file_data[3]<<24));

	/* File size should be at least:
	 * net_data_size + 4 bytes net_size + 4 bytes gram_size
	 */
	if (net_size > (file_size - 8)) {
		dev_err(p->dev,	"%s: Net Encoded size > File size\n", __func__);
		ret = -EINVAL;
		goto out;
	}

	gram_size = (size_t)(file_data[net_size+4] |
				(file_data[net_size+5]<<8) |
				(file_data[net_size+6]<<16) |
				(file_data[net_size+7]<<24));

	/* File size should be at least:
	 * gram_data_size + net_data_size + 4 bytes net_size + 4 bytes gram_size
	 */
	if ((net_size + gram_size + 8) > file_size) {
		dev_err(p->dev,	"%s: Encoded size > File size\n", __func__);
		ret = -EINVAL;
		goto out;
	}

	if (net_size + gram_size +
			DBMDX_AMODEL_HEADER_SIZE*2 > MAX_AMODEL_SIZE) {
		dev_err(p->dev,
				"%s: Amodel exceeds max amodel size %zd>%d\n",
				__func__,
				net_size + gram_size +
				DBMDX_AMODEL_HEADER_SIZE*2, MAX_AMODEL_SIZE);
		ret = -EINVAL;
		goto out;
	}

	*num_of_amodel_chunks = 2;

	head[0] = 0x0;
	head[1] = 0x0;
	head[2] = 0x5A;
	head[3] = 0x02;
	head[4] =  (gram_size/2)        & 0xff;
	head[5] = ((gram_size/2) >>  8) & 0xff;
	head[6] = ((gram_size/2) >> 16) & 0xff;
	head[7] = ((gram_size/2) >> 24) & 0xff;
	head[8] = (gram_addr)        & 0xff;
	head[9] = ((gram_addr) >>  8) & 0xff;
	head[10] = ((gram_addr) >> 16) & 0xff;
	head[11] = ((gram_addr) >> 24) & 0xff;

	memcpy(amodel_buf + target_pos, head, head_size);

	target_pos += head_size;

	memcpy(amodel_buf + target_pos, file_data + net_size + 8, gram_size);

	target_pos += gram_size;

	head[0] = 0x0;
	head[1] = 0x0;
	head[2] = 0x5A;
	head[3] = 0x02;
	head[4] =  (net_size/2)        & 0xff;
	head[5] = ((net_size/2) >>  8) & 0xff;
	head[6] = ((net_size/2) >> 16) & 0xff;
	head[7] = ((net_size/2) >> 24) & 0xff;
	head[8] = (net_addr)        & 0xff;
	head[9] = ((net_addr) >>  8) & 0xff;
	head[10] = ((net_addr) >> 16) & 0xff;
	head[11] = ((net_addr) >> 24) & 0xff;

	memcpy(amodel_buf + target_pos, head, head_size);

	target_pos += head_size;

	memcpy(amodel_buf + target_pos, file_data + 4, net_size);

	target_pos += net_size;

	amodel_chunks_size[0] = gram_size;
	amodel_chunks_size[1] = net_size;

	ret = dbmdx_calc_amodel_checksum(p,
					(char *)amodel_buf,
					target_pos,
					&checksum);
	if (ret) {
		dev_err(p->dev, "%s: failed to calculate Amodel checksum\n",
			__func__);
		ret = -EINVAL;
		goto out;
	}

	*(unsigned long *)(amodel_buf + target_pos) = checksum;

	*amodel_size = (ssize_t)(target_pos + 4);

	ret = *amodel_size;

out:
	return ret;
}

static int dbmdx_acoustic_model_build(struct dbmdx_private *p,
					int num_of_amodel_files,
					const u8 **amodel_files_data,
					ssize_t *amodel_files_size,
					u32 gram_addr,
					u32 amodel_options,
					char	*amodel_buf,
					ssize_t *amodel_size,
					int	*num_of_amodel_chunks,
					ssize_t *amodel_chunks_size)
{
	if (amodel_options & DBMDX_AMODEL_INCLUDES_HEADERS) {
		if (amodel_options & DBMDX_AMODEL_SVT_ENCODING)
			return
			dbmdx_acoustic_model_build_from_svt_multichunk_file(p,
					amodel_files_data[0],
					amodel_files_size[0],
					amodel_buf,
					amodel_size,
					num_of_amodel_chunks,
					amodel_chunks_size);
		else
			return
			dbmdx_acoustic_model_build_from_multichunk_file(p,
					amodel_files_data[0],
					amodel_files_size[0],
					amodel_buf,
					amodel_size,
					num_of_amodel_chunks,
					amodel_chunks_size);

	} else {
		if (amodel_options & DBMDX_AMODEL_SINGLE_FILE_NO_HEADER) {
			*num_of_amodel_chunks = 1;
			amodel_chunks_size[0] = amodel_files_size[0];
			return dbmdx_acoustic_model_build_single_no_headers(
						p,
						amodel_files_data[0],
						amodel_files_size[0],
						gram_addr,
						amodel_buf,
						amodel_size);
		} else {
			*num_of_amodel_chunks = 2;
			amodel_chunks_size[0] = amodel_files_size[0];
			amodel_chunks_size[1] = amodel_files_size[1];
			return dbmdx_acoustic_model_build_gram_net_no_headers(
						p,
						amodel_files_data[0],
						amodel_files_size[0],
						gram_addr,
						amodel_files_data[1],
						amodel_files_size[1],
						amodel_buf,
						amodel_size);
		}
	}

}

#ifdef EXTERNAL_SOC_AMODEL_LOADING_ENABLED
static int dbmdx_acoustic_model_build_from_external(struct dbmdx_private *p,
							const u8 *amodel_data,
							unsigned int size)
{
	enum dbmdx_load_amodel_mode amode;
	int cur_val;
	u32 amodel_options;
	int num_of_amodel_files;
	const u8 *files_data[DBMDX_AMODEL_MAX_CHUNKS];
	ssize_t amodel_files_size[DBMDX_AMODEL_MAX_CHUNKS];
	int chunk_idx;
	size_t off = 0;
	struct amodel_info *cur_amodel = NULL;
	int ret = 0;
	enum dbmdx_model_detection_type det_type;
	enum dbmdx_vt_engine_type eng_type;
	struct vt_engine *sel_vt_eng = NULL;

	if (!p)
		return -EAGAIN;

	/* Engine type */
	cur_val = amodel_data[0];
	off += 1;

	if (cur_val >= DBMDX_VT_ENGINE_TYPE_MAX) {
		dev_err(p->dev,	"%s: invalid VTE type: %d\n", __func__,
			cur_val);
		ret = -EINVAL;
		goto out;
	}

	eng_type = (enum dbmdx_vt_engine_type)cur_val;

	ret = dbmdx_get_vte_from_model_type(p, eng_type, &sel_vt_eng);
	if (ret < 0) {
		dev_err(p->dev,	"%s: VTE Type %d is not supported\n",
				__func__, cur_val);
		ret = -EINVAL;
		goto out;
	}

	if (!(sel_vt_eng->enabled)) {
		dev_err(p->dev,	"%s: VTE %d is disabled\n",
				__func__, cur_val);
		ret = -EINVAL;
		goto out;
	}

	dev_dbg(p->dev, "%s: VTE Type %d (%s)\n", __func__,
			cur_val, dbmdx_vte_names[sel_vt_eng->eng_type]);

	/* Detection type */
	cur_val = amodel_data[off];
	off += 1;

	if (cur_val >= DETECTION_TYPE_MAX) {
		dev_err(p->dev,	"%s: Invalid Detection Type: %d\n", __func__,
			cur_val);
		ret = -EINVAL;
		goto out;
	}

	det_type = (enum dbmdx_model_detection_type)cur_val;

	/* Model type */
	cur_val = amodel_data[off];
	off += 1;
	if ((cur_val != LOAD_AMODEL_PRIMARY) &&
		(cur_val != LOAD_AMODEL_2NDARY)) {
		dev_err(p->dev,	"%s: invalid Amodel mode: %d\n", __func__,
			cur_val);
		ret = -EINVAL;
		goto out;
	}

	amode = (enum dbmdx_load_amodel_mode)cur_val;

	switch (det_type) {
	case DETECTION_TYPE_VOICE_COMMAND:
		if (!(sel_vt_eng->mode_support_mask &
					DBMDX_VT_MODE_SUPPORT_VOICE_COMMAND)) {
			dev_err(p->dev,
				"%s: VT Engine doesn't support Command mode",
								__func__);
			ret = -EINVAL;
			goto out;
		}
		cur_amodel = &(sel_vt_eng->vc_model);
		break;
	case DETECTION_TYPE_DUAL:
		if (!(sel_vt_eng->mode_support_mask &
					DBMDX_VT_MODE_SUPPORT_VOICE_DUAL)) {
			dev_err(p->dev,
				"%s: VT Engine doesn't support Dual mode",
								__func__);
			ret = -EINVAL;
			goto out;
		}
		if (amode == LOAD_AMODEL_PRIMARY)
			cur_amodel = &(sel_vt_eng->primary_model);
		else
			cur_amodel = &(sel_vt_eng->secondary_model);
		break;
	case DETECTION_TYPE_VOICE_ENERGY:
		if (!(sel_vt_eng->mode_support_mask &
					DBMDX_VT_MODE_SUPPORT_VOICE_ENERGY)) {
			dev_err(p->dev,
				"%s: VT Engine doesn't support VE mode",
								__func__);
			ret = -EINVAL;
			goto out;
		}
		cur_amodel = &(sel_vt_eng->ve_model);
		break;
	case DETECTION_TYPE_PHRASE:
		if (!(sel_vt_eng->mode_support_mask &
					DBMDX_VT_MODE_SUPPORT_PHRASE)) {
			dev_err(p->dev,
				"%s: VT Engine doesn't support Phrase mode",
								__func__);
			ret = -EINVAL;
			goto out;
		}
		cur_amodel = &(sel_vt_eng->primary_model);
		break;
	default:
		dev_err(p->dev,
			"%s: Error unknown detection mode:%d", __func__,
								det_type);
		ret = -EINVAL;
		goto out;
	}

	dev_dbg(p->dev, "%s: VTE Detection Type %d (%s), Amode (%d)\n",
				__func__, cur_val,
				dbmdx_vte_det_type_names[det_type],
				(int)amode);

	amodel_options = amodel_data[off];
	off += 1;
	dev_dbg(p->dev, "%s: Amodel options %d\n", __func__, amodel_options);

	num_of_amodel_files = amodel_data[off];
	off += 1;

	dev_dbg(p->dev, "%s: Num Of Amodel files %d\n", __func__,
							num_of_amodel_files);

	for (chunk_idx = 0; chunk_idx < num_of_amodel_files; chunk_idx++)
		files_data[chunk_idx] = NULL;

	for (chunk_idx = 0; chunk_idx < num_of_amodel_files; chunk_idx++) {

		/* The size is encoded in bytes */
		amodel_files_size[chunk_idx] = (ssize_t)(amodel_data[off] |
						(amodel_data[off+1]<<8) |
						(amodel_data[off+2]<<16) |
						(amodel_data[off+3]<<24));
		off += 4;
		dev_dbg(p->dev, "%s: Chunk size %d\n", __func__,
			(int)(amodel_files_size[chunk_idx]));

		/* File size should be at least:
		 * net_data_size + 4 bytes net_size + 4 bytes gram_size
		 */
		if (amodel_files_size[chunk_idx] > (size - off)) {
			dev_err(p->dev,	"%s: Chunk size exceeds buffer size\n",
					__func__);
			ret = -EINVAL;
			goto out;
		}

		if (amodel_files_size[chunk_idx] == 0) {
			dev_warn(p->dev, "%s Chunk size is 0. Ignore...\n",
				__func__);
			ret = -ENOENT;
			goto out;
		}

		files_data[chunk_idx] = &(amodel_data[off]);
		off += amodel_files_size[chunk_idx];

		dev_dbg(p->dev, "%s Chunk #%d size=%zu bytes\n",
			__func__, chunk_idx, amodel_files_size[chunk_idx]);
	}

	if (cur_amodel == NULL) {
		dev_err(p->dev,	"%s: amodel loading mode is not supported\n",
			__func__);
		ret = -EINVAL;
		goto out;
	}

	if (cur_amodel->amodel_buf == NULL) {
		cur_amodel->amodel_buf  = vmalloc(MAX_AMODEL_SIZE);
		if (!cur_amodel->amodel_buf) {
			ret = -ENOMEM;
			goto out;
		}
	}

	cur_amodel->amodel_loaded = false;

	ret = dbmdx_acoustic_model_build(p,
					num_of_amodel_files,
					files_data,
					amodel_files_size,
					0x1,
					amodel_options,
					cur_amodel->amodel_buf,
					&cur_amodel->amodel_size,
					&cur_amodel->num_of_amodel_chunks,
					cur_amodel->amodel_chunks_size);

	if (ret <= 0) {
		dev_err(p->dev,	"%s: amodel build failed: %d\n",
			__func__, ret);
		ret = -EIO;
		goto out;
	}

	sel_vt_eng->amodel_options = amodel_options;

	ret = 0;

	cur_amodel->amodel_loaded = true;
out:
	return ret;
}
#endif

static void dbmdx_get_firmware_version(const char *data, size_t size,
				       char *buf, size_t buf_size)
{
	int i, j;

	buf[0] = 0;
	i = size - 58;
	/* VA FW */
	for (i = size - 13; i > 0; i--) {
		if ((data[i]   == 'v') && (data[i+2]  == 'e') &&
		    (data[i+4] == 'r') && (data[i+6]  == 's') &&
		    (data[i+8] == 'i') && (data[i+10] == 'o')) {
			for (j = 0; i + j < size; j++) {
				if (j == buf_size - 1)
					break;
				buf[j] = data[i];
				i += 2;
				if (((buf[j] > 0) && (buf[j] < 32))
				    || (buf[j] > 126))
					return;
				if (buf[j] == 0)
					buf[j] = ' ';
			}
			buf[j] = 0;
			return;
		}
	}
}

static int dbmdx_firmware_ready(const struct firmware *fw,
				struct dbmdx_private *p)
{
	const u8 *fw_file_checksum;
	char fw_version[200];
	int ret;

	if (!fw) {
		dev_err(p->dev, "%s: firmware request failed\n", __func__);
		return -EIO;
	}

	if (fw->size <= 4) {
		dev_err(p->dev, "%s: firmware size (%zu) invalid\n",
			__func__, fw->size);
		goto out_err;
	}

	fw_file_checksum = &fw->data[fw->size - 4];

	/*
	 *  read firmware version from file, not sure if this is the same
	 *  for VA firmware
	 */
	memset(fw_version, 0, 200);
	dbmdx_get_firmware_version(fw->data, fw->size, fw_version, 200);
	if (strlen(fw_version) > 15)
		dev_info(p->dev, "%s: firmware: %s\n", __func__, fw_version);

	/* check if the chip interface is ready to boot */
	ret = p->chip->can_boot(p);
	if (ret)
		goto out_err;

	/* prepare boot if required */
	ret = p->chip->prepare_boot(p);
	if (ret)
		goto out_err;

	/* enable high speed clock for boot */
	p->clk_enable(p, DBMDX_CLK_MASTER);

	/* boot */
	ret = p->chip->boot(fw->data, fw->size, p, fw_file_checksum, 4, 1);
	if (ret)
		goto out_disable_hs_clk;

	/* disable high speed clock after boot */
	p->clk_disable(p, DBMDX_CLK_MASTER);

	/* finish boot if required */
	ret = p->chip->finish_boot(p);
	if (ret)
		goto out_err;

	ret = 0;
	goto out;

out_disable_hs_clk:
	p->clk_disable(p, DBMDX_CLK_MASTER);
out_err:
	dev_err(p->dev, "%s: firmware request failed\n", __func__);
	ret = -EIO;
out:
	return ret;
}

int dbmdx_indirect_register_read(struct dbmdx_private *p,
				u16 addr, u16 *presult)
{
	u16 val;
	int ret = 0;

	dev_dbg(p->dev, "%s\n", __func__);

	if (!p)
		return -EAGAIN;

	if (!p->device_ready) {
		dev_err(p->dev, "%s: device not ready\n", __func__);
		return -EAGAIN;
	}

	*presult = 0;

	ret = dbmdx_send_cmd(p,	DBMDX_REGN_INDIRECT_ACCESS_REG_NUMBER |
				(u16)addr, NULL);

	if (ret < 0) {
		dev_err(p->dev, "%s: set_param_addr error(1)\n", __func__);
		ret = -EIO;
		goto out;
	}

	ret = dbmdx_send_cmd(p, DBMDX_REGN_INDIRECT_ACCESS_REG_READ, &val);

	if (ret < 0) {
		dev_err(p->dev, "%s: get param error\n", __func__);
		ret = -EIO;
		goto out;
	}

	*presult = val;
out:
	return ret;
}

int dbmdx_indirect_register_write(struct dbmdx_private *p,
				u16 addr, u16 val)
{
	int ret = 0;

	dev_dbg(p->dev, "%s\n", __func__);

	if (!p)
		return -EAGAIN;

	if (!p->device_ready) {
		dev_err(p->dev, "%s: device not ready\n", __func__);
		return -EAGAIN;
	}

	ret = dbmdx_send_cmd(p, DBMDX_REGN_INDIRECT_ACCESS_REG_NUMBER |
					(u16)addr, NULL);

	if (ret < 0) {
		dev_err(p->dev, "%s: set_param_addr error(1)\n", __func__);
		ret = -EIO;
		goto out;
	}

	ret = dbmdx_send_cmd(p, DBMDX_REGN_INDIRECT_ACCESS_REG_WRITE | (u16)val,
				NULL);

	if (ret < 0) {
		dev_err(p->dev, "%s: set param error\n", __func__);
		ret = -EIO;
		goto out;
	}
out:
	return ret;
}

int dbmdx_io_register_read(struct dbmdx_private *p,
				u32 addr, u32 *presult)
{
	int ret = 0;

	dev_dbg(p->dev, "%s\n", __func__);

	if (!p)
		return -EAGAIN;

	*presult = 0;
	/* Write to IO Port Low the 32 bit address*/
	ret = dbmdx_send_cmd_32(p, DBMDX_REGN_IO_PORT_ADDR_LO, addr, NULL);
	if (ret < 0) {
		dev_err(p->dev, "%s: io_addr_read error(1)\n", __func__);
		ret = -1;
		goto out;
	}

	/* Read from IO Port value the content returned
	 *( start with write and wait for response )
	 */
	ret = dbmdx_send_cmd_32(p, DBMDX_REGN_IO_PORT_VALUE_LO, addr, presult);
	if (ret < 0) {
		dev_err(p->dev, "%s: get reg %u error\n",
				__func__, DBMDX_REGN_IO_PORT_VALUE_LO);
		ret = -1;
		goto out;
	}

	dev_dbg(p->dev, "%s: addr=0x%08x, val = 0x%08x\n",
		__func__, addr, *presult);

out:
	return ret;
}

int dbmdx_io_register_write(struct dbmdx_private *p,
				u32 addr, u32 value)
{
	int ret = 0;
	dev_dbg(p->dev, "%s\n", __func__);

	if (!p)
		return -EAGAIN;

	/* set indirect address */
	ret = dbmdx_send_cmd_32(p, DBMDX_REGN_IO_PORT_ADDR_LO, addr, NULL);
	if (ret < 0) {
		dev_err(p->dev, "%s: io_addr_write error(1)\n", __func__);
		ret = -1;
		goto out;
	}

	ret = dbmdx_send_cmd_32(p, DBMDX_REGN_IO_PORT_VALUE_LO, value, NULL);
	if (ret < 0) {
		dev_err(p->dev, "%s: io_addr_write error(2)\n", __func__);
		ret = -1;
		goto out;
	}

	dev_dbg(p->dev, "%s: addr=0x%08x was set to 0x%08x\n",
		__func__, addr, value);

out:
	return ret;
}


static int verify_dep_divisors(u32 output_clk,
				u32 *dep_divisors, int num_dep_divisors)
{
	int i;

	if (dep_divisors == NULL || !num_dep_divisors)
		return 0;

	for (i = 0; i < num_dep_divisors; i++) {
		if (output_clk % dep_divisors[i])
			return 1;
	}

	return 0;
}

static int calc_clk_div(struct dbmdx_private *p, u32 ref_clk,
				u32 low_clk_steps,
				struct clock_config *chip_clock_conf,
				u32 max_pll,
				u32 *dep_divisors, int num_dep_divisors,
				u16 *pll_step, u16 *tl3_div,
				u16 *ahb_div, u16 *apb_div,
				u16 *post_pll_div)
{
	u32 output_clk = 0;
	u32 ref_clk_c;
	u32 tl3_div_c = 0, pll_step_c = 0;
	u32 wanted_ahb_clk = 0, wanted_apb_clk = 0;
	u32 ahb_div_c = 0, apb_div_c = 0;
	u32 ahb_out = 0, apb_out = 0, global_out = 0, tl3_out = 0;
	u32 wanted_pll = chip_clock_conf->wanted_pll;
	u32 post_pll_div_c = 0;
	u32 multiplier = 1;

	if (ref_clk == 32768)
		ref_clk_c = 32768000;
	else
		ref_clk_c = ref_clk;

	if (wanted_pll == 0)
		wanted_pll = chip_clock_conf->wanted_tl3_clk;
	else if (wanted_pll < ref_clk_c) {
		dev_err(p->dev,
			"%s Wanted PLL freq. cannot be lower than input CLK\n",
			__func__);
		return -EINVAL;
	}


	if (wanted_pll < ref_clk_c)
		multiplier = 2;

	dev_dbg(p->dev, "%s: Wanted Pll: %u Hz\t RefClk_c %u Hz\tMult=%u\n",
			__func__, wanted_pll, ref_clk_c, multiplier);

	if (ref_clk_c == 32768000 || ref_clk_c == 49152000 ||
						ref_clk_c == 12288000) {
		if (multiplier*wanted_pll < ref_clk_c)
			pll_step_c = 0;
		else
			pll_step_c = DIV_ROUND_UP(multiplier*wanted_pll -
						ref_clk_c, low_clk_steps);
		dev_dbg(p->dev, "%s: (1)Current Pll Step: %u\n",
							__func__, pll_step_c);

		while (1) {
			output_clk = ref_clk_c + (pll_step_c * low_clk_steps);
			dev_dbg(p->dev, "%s: (1) Current output_clk: %u\n",
							__func__, output_clk);

			if (output_clk > max_pll) {
				dev_err(p->dev,
					"%s: Cannot match divisors, Max PLL was reached\n",
					__func__);
				return -EINVAL;
			}
			if (verify_dep_divisors(output_clk,
					dep_divisors, num_dep_divisors) == 0)
				break;

			pll_step_c = pll_step_c + 1;
		}
	} else {
		pll_step_c = DIV_ROUND_UP(multiplier*wanted_pll, ref_clk_c);
		dev_dbg(p->dev, "%s: (2)Current Pll Step: %u\n",
						__func__, pll_step_c);

		while (1) {
			output_clk = ref_clk_c * pll_step_c;
			dev_dbg(p->dev, "%s: (2) Current output_clk: %u\n",
							__func__, output_clk);

			if (output_clk > max_pll) {
				dev_err(p->dev,
					"%s: Cannot match divisors, Max PLL was reached\n",
					__func__);
				return -EINVAL;
			}
			if (verify_dep_divisors(output_clk,
					dep_divisors, num_dep_divisors) == 0)
				break;

			pll_step_c = pll_step_c + 1;
		}

	}

	if (chip_clock_conf->use_pll_post_div) {
		post_pll_div_c = (u32)(output_clk /
					chip_clock_conf->wanted_tl3_clk);
		tl3_div_c = 1;
	} else {
		tl3_div_c = (u32)(output_clk / chip_clock_conf->wanted_tl3_clk);
	}

	dev_dbg(p->dev,
		"%s: Pll Step: %u\tPost Pll Div: %u\tTL3 Div: %u\n",
		__func__, pll_step_c, post_pll_div_c, tl3_div_c);

	tl3_div_c = tl3_div_c - 1;
	if (chip_clock_conf->wanted_ahb_clk == 0)
		wanted_ahb_clk = chip_clock_conf->wanted_tl3_clk;
	if (chip_clock_conf->wanted_apb_clk == 0)
		wanted_apb_clk = wanted_ahb_clk;

	ahb_div_c = (chip_clock_conf->wanted_tl3_clk / wanted_ahb_clk) - 1;
	apb_div_c = (wanted_ahb_clk / wanted_apb_clk) - 1;
	global_out =  output_clk / (post_pll_div_c + 1);
	tl3_out = global_out / (tl3_div_c + 1);
	ahb_out = tl3_out / (ahb_div_c + 1);
	apb_out = ahb_out / (apb_div_c + 1);

	*pll_step = (u16)pll_step_c;
	*tl3_div = (u16)tl3_div_c;
	*ahb_div = (u16)ahb_div_c;
	*apb_div = (u16)apb_div_c;
	*post_pll_div = (u16)post_pll_div_c;

	dev_dbg(p->dev,
		"%s: Selected Clk Freq: PLL: %u(Hz) LB: %u(Hz) TL3: %u(Hz) AHB: %u(Hz) APB: %u(Hz)\n",
		__func__, output_clk, global_out, tl3_out, ahb_out, apb_out);

	dev_dbg(p->dev,
		"%s: Dividers: PLL Step: 0x%x, TL3_DIV: 0x%x AHB_DIV: 0x%x APB_DIV: 0x%x POST_PLL_DIV: 0x%x\n",
		__func__, pll_step_c, tl3_div_c, ahb_div_c,
		apb_div_c, post_pll_div_c);

	return 0;
}

static int configure_chip_clock(struct dbmdx_private *p,
				enum dbmdx_chip chip,
				enum dbmdx_clock_op clock_op,
				enum TDM_CLK_FREQ_CONF freq,
				int num_of_bits,
				u32 m_clk,
				u32 ref_clk,
				u32 low_clk_steps,
				struct clock_config *chip_clock_conf,
				u32 max_pll,
				u32 *dep_divisors,
				int num_dep_divisors)
{
	u16 pll_step, tl3_div;
	u16 ahb_div, apb_div;
	u16 post_pll_div;
	u16 switch_clk;
	u16 clk_src;
	u16 cur_config;
	u16 new_config;
	int ret = 0;

	dev_dbg(p->dev,
		"%s: Chip: %s, MCLK: %u(Hz)\n",
		__func__, chip == DBMDX_CHIP_VA ? "VA" : "VA_VE", m_clk);

	dev_dbg(p->dev,	"%s: VA Clock Operation:\t%s\n", __func__,
		(clock_op == DBMDX_CLOCK_OP_SWITCH_TO_MASTER_CLOCK) ?
			"Switch To Master CLK" :
		(clock_op == DBMDX_CLOCK_OP_SWITCH_TO_TDM_CLOCK) ?
			"Switch To TDM CLK" : "Configure Master CLK");
	dev_dbg(p->dev,
		"%s: RefClk: %u(Hz) LowClockSteps: %u(Hz) PLL: %u(Hz) TL3: %u(Hz) AHB: %u(Hz) APB: %u(Hz) UsePllDiv: %s\n",
		__func__, ref_clk,
		low_clk_steps,
		chip_clock_conf->wanted_pll,
		chip_clock_conf->wanted_tl3_clk,
		chip_clock_conf->wanted_ahb_clk,
		chip_clock_conf->wanted_apb_clk,
		chip_clock_conf->use_pll_post_div ? "ON" : "OFF");

	if (num_dep_divisors > 0) {
		int i;

		dev_dbg(p->dev, "%s: Dep. Divisors:\n", __func__);

		for (i = 0; i < num_dep_divisors; i++)
			dev_dbg(p->dev, "%s:\tDep. Div[%d]: %u\n", __func__,
							i, dep_divisors[i]);

	}

	ret = calc_clk_div(p, ref_clk, low_clk_steps,
				chip_clock_conf, max_pll,
				dep_divisors, num_dep_divisors,
				&pll_step, &tl3_div,
				&ahb_div, &apb_div, &post_pll_div);
	if (ret < 0) {
		dev_err(p->dev, "%s: Error calculating CLK DIV\n", __func__);
		ret = -EIO;
		goto out;
	}

	if (chip == DBMDX_CHIP_VA) {
		ret = dbmdx_switch_to_va_chip_interface(p,
							DBMDX_CMD_INTERFACE);
		if (ret) {
			dev_err(p->dev,
				"%s Error switching to (VA) CMD interface\n",
				__func__);
			ret = -EIO;
			goto out;
		}
	} else {
		ret = dbmdx_switch_to_va_ve_chip_interface(p,
							DBMDX_CMD_INTERFACE);
		if (ret) {
			dev_err(p->dev,
				"%s Error switching to (VA_VE) CMD interface\n",
				__func__);
			ret = -EIO;
			goto out;
		}

	}

	if (clock_op == DBMDX_CLOCK_OP_SWITCH_TO_TDM_CLOCK) {
		u32 tdm_clk_freq = (u32)(freq * num_of_bits * 2) / 1000;

		switch_clk = DBMDX_REGV_SRC_SET_CONF_CLK_ACCORDING_TO_CLK_SEL;
		clk_src = DBMDX_REGV_CLK_SEL_TDM0_SCLK;
		ret = dbmdx_send_cmd(p, DBMDX_REGN_TDM_SCLK_CLOCK_FREQUENCY |
						tdm_clk_freq, NULL);
		if (ret < 0) {
			dev_err(p->dev,
				"%s: failed to set tdm sclk freq\n",
				__func__);
			ret = -EIO;
			goto out;
		}

	} else if (clock_op == DBMDX_CLOCK_OP_SWITCH_TO_MASTER_CLOCK) {
		u16 mclk_to_set = DBMDX_REGV_MCLK_24576000Hz;

		switch_clk = DBMDX_REGV_SRC_SET_CONF_CLK_ACCORDING_TO_CLK_SEL;

		clk_src = DBMDX_REGV_CLK_SEL_MCLK_IN;

		if (m_clk <= 32768)
			mclk_to_set = DBMDX_REGV_MCLK_32768Hz;
		else if (m_clk <= 12288000)
			mclk_to_set = DBMDX_REGV_MCLK_12288000Hz;
		else if (m_clk <= 19200000)
			mclk_to_set = DBMDX_REGV_MCLK_19200000Hz;
		else
			mclk_to_set = DBMDX_REGV_MCLK_24576000Hz;

		ret = dbmdx_send_cmd(p,	DBMDX_REGN_MASTER_CLOCK_FREQUENCY |
					mclk_to_set, NULL);
		if (ret < 0) {
			dev_err(p->dev, "%s: failed to set MCLK freq\n",
				__func__);
			ret = -EIO;
			goto out;
		}
	} else {
		switch_clk = DBMDX_REGV_SRC_SET_KEEP_CUR_CLK_SRC;
		clk_src = DBMDX_REGV_CLK_SEL_MCLK_IN;
	}

	/* read configuration */
	ret = dbmdx_send_cmd(p, DBMDX_REGN_GENERAL_CONFIG_2, &cur_config);
	if (ret < 0) {
		dev_err(p->dev,
			"%s: failed to read DBMDX_VA_GENERAL_CONFIG_2\n",
			__func__);
		ret = -EIO;
		goto out;
	}

	if (chip_clock_conf->use_pll_post_div)
		new_config &= (0xFFE0 | (u16)(post_pll_div-1));
	else
		new_config &= 0xFFE0;

	if (cur_config != new_config) {
		ret = dbmdx_send_cmd(p,
				     DBMDX_REGN_GENERAL_CONFIG_2 | new_config,
				     NULL);
		if (ret < 0) {
			dev_err(p->dev,
				"%s: failed to set DBMDX_VA_GENERAL_CONFIG_2\n",
				__func__);
			ret = -EIO;
			goto out;
		}
	}

	if ((pll_step > 14 &&
			(clock_op != DBMDX_CLOCK_OP_SWITCH_TO_TDM_CLOCK)) ||
		(clock_op == DBMDX_CLOCK_OP_SWITCH_TO_MASTER_CLOCK)) {

		ret = dbmdx_send_cmd(p,	(DBMDX_REGN_DSP_CLOCK_CONFIG |
					DBMDX_REGV_USE_PLL_STEP_FROM_REG_0x1E |
					tl3_div << 8	|
					apb_div	<< 4	|
					ahb_div), NULL);
		if (ret < 0) {
			dev_err(p->dev,	"%s: failed to set tdm sclk freq\n",
					__func__);
			ret = -EIO;
			goto out;
		}

		if (m_clk <= 32768)
			msleep(DBMDX_MSLEEP_CONFIG_HOST_CLOCK_32KHZ);
		else
			usleep_range(DBMDX_USLEEP_CONFIG_HOST_CLOCK,
					DBMDX_USLEEP_CONFIG_HOST_CLOCK + 1000);

		ret = dbmdx_send_cmd(p,	(DBMDX_REGN_DSP_CLOCK_CONFIG_EXTENSION |
					switch_clk |
					clk_src |
					DBMDX_REGV_PLL_OSC_SEL_USE_PLL |
					pll_step), NULL);
		if (ret < 0) {
			dev_err(p->dev,	"%s: failed to set tdm sclk freq\n",
					__func__);
			ret = -EIO;
			goto out;
		}

		if (m_clk <= 32768)
			msleep(DBMDX_MSLEEP_CONFIG_HOST_CLOCK_32KHZ);
		else
			usleep_range(DBMDX_USLEEP_CONFIG_HOST_CLOCK,
					DBMDX_USLEEP_CONFIG_HOST_CLOCK + 1000);

	} else if (clock_op != DBMDX_CLOCK_OP_SWITCH_TO_TDM_CLOCK) {

		ret = dbmdx_send_cmd(p,	(DBMDX_REGN_DSP_CLOCK_CONFIG |
					pll_step << 12	|
					tl3_div << 8	|
					apb_div	<< 4	|
					ahb_div), NULL);
		if (ret < 0) {
			dev_err(p->dev,	"%s: failed to set tdm sclk freq\n",
					__func__);
			ret = -EIO;
			goto out;
		}

		if (m_clk <= 32768)
			msleep(DBMDX_MSLEEP_CONFIG_HOST_CLOCK_32KHZ);
		else
			usleep_range(DBMDX_USLEEP_CONFIG_HOST_CLOCK,
					DBMDX_USLEEP_CONFIG_HOST_CLOCK + 1000);

	} else {

		ret = dbmdx_send_cmd(p,	(DBMDX_REGN_DSP_CLOCK_CONFIG |
					DBMDX_REGV_USE_PLL_STEP_FROM_REG_0x1E |
					tl3_div << 8	|
					apb_div	<< 4	|
					ahb_div), NULL);
		if (ret < 0) {
			dev_err(p->dev,	"%s: failed to set tdm sclk freq\n",
					__func__);
			ret = -EIO;
			goto out;
		}

		usleep_range(DBMDX_USLEEP_CONFIG_HOST_CLOCK,
					DBMDX_USLEEP_CONFIG_HOST_CLOCK + 1000);

		ret = dbmdx_send_cmd(p,	(DBMDX_REGN_DSP_CLOCK_CONFIG_EXTENSION |
					switch_clk |
					clk_src |
					DBMDX_REGV_PLL_OSC_SEL_USE_PLL |
					pll_step), NULL);
		if (ret < 0) {
			dev_err(p->dev,
				"%s: failed to set clock ext\n",
					__func__);
				ret = -EIO;
			goto out;
		}

		if (m_clk <= 32768)
			msleep(DBMDX_MSLEEP_CONFIG_HOST_CLOCK_32KHZ);
		else
			usleep_range(DBMDX_USLEEP_CONFIG_HOST_CLOCK,
					DBMDX_USLEEP_CONFIG_HOST_CLOCK + 1000);

	}

	if (clk_src == DBMDX_REGV_CLK_SEL_MCLK_IN) {
		if (chip == DBMDX_CHIP_VA)
			p->cur_va_clock = DBMDX_CLOCK_FROM_MASTER;
		else
			p->cur_va_ve_clock = DBMDX_CLOCK_FROM_MASTER;
	}

	if (clock_op == DBMDX_CLOCK_OP_SWITCH_TO_TDM_CLOCK) {
		int retry;
		u16 val;
		u16 expected_val;
		u32 num_of_sensing_retries =
			((p->pdata->change_clock_src_enabled >> 8) & 0xff);

		for (retry = 0; retry < num_of_sensing_retries; retry++) {

			ret = dbmdx_send_cmd(p,
					DBMDX_REGN_DSP_CLOCK_CONFIG_EXTENSION,
					&val);
			if (ret < 0) {
				dev_err(p->dev,
					"%s: failed to read clock ext\n",
					__func__);
				ret = -EIO;
				goto out;
			}

			expected_val = (u16)(DBMDX_REGV_CLK_SEL_TDM0_SCLK |
					DBMDX_REGV_PLL_OSC_SEL_USE_PLL) +
						pll_step;

			if (expected_val == val)
				break;

			dev_dbg(p->dev,
				"%s: No clock: exp:0x%08x, recv:0x%08x\n",
				__func__, expected_val, val);

			ret = dbmdx_send_cmd(p,
					(DBMDX_REGN_DSP_CLOCK_CONFIG_EXTENSION |
						switch_clk |
						clk_src |
						DBMDX_REGV_PLL_OSC_SEL_USE_PLL |
						pll_step), NULL);
			if (ret < 0) {
				dev_err(p->dev,
					"%s: failed to set clock ext\n",
					__func__);
				ret = -EIO;
				goto out;
			}

			msleep(DBMDX_MSLEEP_CONFIG_TDM_CLOCK);

		}

		if (num_of_sensing_retries && retry >= num_of_sensing_retries) {
			dev_err(p->dev,
				"%s: FW didn't detect clock on TDM\n",
				__func__);
			if (chip == DBMDX_CHIP_VA)
				p->cur_va_clock = DBMDX_CLOCK_FROM_MASTER;
			else
				p->cur_va_ve_clock = DBMDX_CLOCK_FROM_MASTER;
			ret = -EIO;
			goto out;
		} else {
			if (chip == DBMDX_CHIP_VA)
				p->cur_va_clock = DBMDX_CLOCK_FROM_TDM;
			else
				p->cur_va_ve_clock = DBMDX_CLOCK_FROM_TDM;

		}

	} else {
		ret = dbmdx_va_alive(p);
		if (ret) {
			dev_err(p->dev, "%s: VA_VE Fw is dead\n", __func__);
			ret = -EIO;
			goto out;
		}
	}

out:
	return ret;


}


static int configure_usecase_clocks(struct dbmdx_private *p,
					struct usecase_config *usecase)
{
	enum dbmdx_clock_op clock_op;
	enum TDM_CLK_FREQ_CONF freq;
	int num_of_bits;
	u32 ref_clk;
	u32 low_clk_steps;
	u32 wanted_pll;
	u32 wanted_tl3_clk;
	u32 dep_divisors[2] = {0};
	u32 m_clk_d4, m_clk_d2;
	int ret = -EINVAL;
	int ret2 = 0;
	bool perform_config = false;
	struct clock_config *chip_clock_conf;
	int cur_divisor_index = 0;

	if (!p)
		return -EAGAIN;

	if (!p->device_ready) {
		dev_err(p->dev, "%s: device not ready\n", __func__);
		return -EAGAIN;
	}

	if (!usecase) {
		dev_err(p->dev, "%s: NULL PTR usecase\n", __func__);
		return -EINVAL;
	}

	m_clk_d4 = (u32)(p->clk_get_rate(p, DBMDX_CLK_MASTER));


	if (p->va_chip_enabled) {

		clock_op = usecase->clock_op_va;
		chip_clock_conf = &(usecase->clock_config_va);

		if (m_clk_d4 == 19200000) {
			ref_clk = 12288000;
			low_clk_steps = 6144000;
		} else {
			ref_clk = m_clk_d4;
			low_clk_steps = 4096000;
		}

		if (chip_clock_conf->wanted_tl3_clk == 0)
			wanted_tl3_clk = p->pdata->default_va_clock;
		else
			wanted_tl3_clk = chip_clock_conf->wanted_tl3_clk;

		if (chip_clock_conf->wanted_pll == 0)
			wanted_pll = wanted_tl3_clk;
		else
			wanted_pll = chip_clock_conf->wanted_pll;

		if (usecase->i2s_master_clock_va) {
			u32 i2s_clock = (u32)(usecase->tdm_clock_freq *
						usecase->number_of_bits * 2) /
						1000;
			dep_divisors[0] = i2s_clock;
			cur_divisor_index++;
		}

		if (!(p->mics_connected_to_va_ve_chip) && usecase->mic_freq) {
			dep_divisors[cur_divisor_index] = usecase->mic_freq;
			cur_divisor_index++;
		}


		if (clock_op == DBMDX_CLOCK_OP_SWITCH_TO_TDM_CLOCK) {

			if (!(p->pdata->change_clock_src_enabled &
				DBMDX_CHANGE_TO_HOST_CLOCK_D4_ENABLED)) {
				dev_info(p->dev,
					"%s: VA Switch to TDM CLK disabled\n",
					__func__);
			} else if (p->cur_va_clock == DBMDX_CLOCK_FROM_TDM) {
				dev_warn(p->dev,
					"%s: VA is already on TDM CLK\n",
					__func__);
			} else if (usecase->tdm_clock_freq == TDM_CLK_FREQ_0) {
				dev_warn(p->dev,
					"%s: Cannot switch to TDM Freq 0\n",
					__func__);
			} else {
				freq = usecase->tdm_clock_freq;
				num_of_bits = usecase->number_of_bits;
				ref_clk = (u32)(freq * num_of_bits * 2);
				perform_config = true;

			}
		} else if ((clock_op ==
				DBMDX_CLOCK_OP_SWITCH_TO_MASTER_CLOCK) ||
			(clock_op == DBMDX_CLOCK_OP_CONFIG_MASTER_CLOCK)) {

			if (p->cur_va_clock == DBMDX_CLOCK_FROM_MASTER &&
				(clock_op ==
				DBMDX_CLOCK_OP_SWITCH_TO_MASTER_CLOCK)) {
				dev_info(p->dev,
					"%s: VA is already on MASTER CLK, changing command\n",
					__func__);
				clock_op = DBMDX_CLOCK_OP_CONFIG_MASTER_CLOCK;
			}

			if (!(p->pdata->change_clock_src_enabled &
				DBMDX_CHANGE_TO_INTERNAL_CLOCK_D4_ENABLED) &&
				(clock_op ==
				DBMDX_CLOCK_OP_SWITCH_TO_MASTER_CLOCK)) {
				dev_info(p->dev,
					"%s: VA Switch to Master CLK disabled\n",
					__func__);
			} else if (p->cur_va_clock == DBMDX_CLOCK_FROM_TDM &&
				(clock_op ==
					DBMDX_CLOCK_OP_CONFIG_MASTER_CLOCK)) {
				dev_info(p->dev,
					"%s: VA cannot configure Master CLK when running on TDM CLK\n",
					__func__);
			} else {
				freq = TDM_CLK_FREQ_0;
				num_of_bits = 0;
				perform_config = true;
			}
		}

		if (perform_config) {
			ret = configure_chip_clock(p, DBMDX_CHIP_VA,
						clock_op,
						freq,
						num_of_bits,
						m_clk_d4,
						ref_clk,
						low_clk_steps,
						chip_clock_conf,
						DBMDX_MAX_VA_PLL,
						dep_divisors,
						cur_divisor_index);

			if (ret) {
				dev_err(p->dev,
					"%s: VA Error Performing CLK operation\n",
						__func__);
				ret = -EIO;
				goto out;
			}
		}
	}

	if (p->va_ve_chip_enabled) {
		perform_config = false;
		cur_divisor_index = 0;
		chip_clock_conf = &(usecase->clock_config_va_ve);
		clock_op = usecase->clock_op_va_ve;

		if (p->pdata->va_ve_separate_clocks)
			m_clk_d2 = (u32)(p->clk_get_rate(p,
				DBMDX_CLK_MASTER_VA_VE));
		else
			m_clk_d2 = m_clk_d4;

		if ((m_clk_d2 == 32768) || (m_clk_d2 == 19200000))
			ref_clk = 49152000;
		else
			ref_clk = m_clk_d2;
		low_clk_steps = 6144;

		if (chip_clock_conf->wanted_tl3_clk == 0)
			wanted_tl3_clk = p->pdata->default_va_ve_clock;
		else
			wanted_tl3_clk = chip_clock_conf->wanted_tl3_clk;

		if (chip_clock_conf->wanted_pll == 0)
			wanted_pll = wanted_tl3_clk;
		else
			wanted_pll = chip_clock_conf->wanted_pll;

		if (usecase->i2s_master_clock_va_ve) {
			u32 i2s_clock = (u32)(usecase->tdm_clock_freq *
						usecase->number_of_bits * 2) /
						1000;
			dep_divisors[0] = i2s_clock;
			cur_divisor_index++;
		}

		if (p->mics_connected_to_va_ve_chip && usecase->mic_freq) {
			dep_divisors[cur_divisor_index] = usecase->mic_freq;
			cur_divisor_index++;
		}


		if (clock_op == DBMDX_CLOCK_OP_SWITCH_TO_TDM_CLOCK) {

			if (!(p->pdata->change_clock_src_enabled &
				DBMDX_CHANGE_TO_HOST_CLOCK_D2_ENABLED)) {
				dev_info(p->dev,
					"%s: VA_VE Switch to TDM CLK disabled\n",
					__func__);
			} else if (p->cur_va_ve_clock == DBMDX_CLOCK_FROM_TDM) {
				dev_warn(p->dev,
					"%s: VA_VE is already on TDM CLK\n",
					__func__);
			} else if (usecase->tdm_clock_freq == TDM_CLK_FREQ_0) {
				dev_warn(p->dev,
					"%s: Cannot switch to TDM Freq 0\n",
					__func__);
			} else {

				freq = usecase->tdm_clock_freq;
				num_of_bits = usecase->number_of_bits;
				ref_clk = (u32)(freq * num_of_bits * 2);
				perform_config = true;
			}
		} else if ((clock_op ==
				DBMDX_CLOCK_OP_SWITCH_TO_MASTER_CLOCK) ||
			(clock_op == DBMDX_CLOCK_OP_CONFIG_MASTER_CLOCK)) {

			if (p->cur_va_ve_clock == DBMDX_CLOCK_FROM_MASTER &&
				(clock_op ==
				DBMDX_CLOCK_OP_SWITCH_TO_MASTER_CLOCK)) {
				dev_info(p->dev,
					"%s: VA_VE is already on MASTER CLK, changing command\n",
					__func__);
				clock_op = DBMDX_CLOCK_OP_CONFIG_MASTER_CLOCK;
			}

			if (!(p->pdata->change_clock_src_enabled &
				DBMDX_CHANGE_TO_INTERNAL_CLOCK_D2_ENABLED) &&
				(clock_op ==
				DBMDX_CLOCK_OP_SWITCH_TO_MASTER_CLOCK)) {
				dev_info(p->dev,
					"%s: VA_VE Switch to Master CLK disabled\n",
					__func__);
			} else if (p->cur_va_ve_clock == DBMDX_CLOCK_FROM_TDM &&
				(clock_op ==
					DBMDX_CLOCK_OP_CONFIG_MASTER_CLOCK)) {
				dev_info(p->dev,
					"%s: VA_VE cannot configure Master CLK when running on TDM CLK\n",
					__func__);
			} else {
				freq = TDM_CLK_FREQ_0;
				num_of_bits = 0;
				perform_config = true;
			}
		}

		if (perform_config) {
			ret = configure_chip_clock(p, DBMDX_CHIP_VA_VE,
						clock_op,
						freq,
						num_of_bits,
						m_clk_d2,
						ref_clk,
						low_clk_steps,
						chip_clock_conf,
						DBMDX_MAX_VA_VE_PLL,
						dep_divisors,
						cur_divisor_index);

			if (ret) {
				dev_err(p->dev,
					"%s: VA_VE Error Performing CLK operation\n",
					__func__);
				ret = -EIO;
				goto out;
			}
		}

	}

out:
	if (p->va_chip_enabled) {
		ret2 = dbmdx_switch_to_va_chip_interface(p,
							DBMDX_CMD_INTERFACE);
		if (ret2) {
			dev_err(p->dev,
				"%s Error switching to (VA) CMD interface\n",
				__func__);
			ret = -EIO;
		}
	} else {
		ret2 = dbmdx_switch_to_va_ve_chip_interface(p,
						DBMDX_CMD_INTERFACE);
		if (ret2) {
			dev_err(p->dev,
				"%s Error switching to (VA_VE) CMD interface\n",
				__func__);
			ret = -EIO;
		}
	}
	return ret;


}
static void get_uc_command_str(u32 seq_cmd, char *cmd_str, int len)
{
	u32 cmd;
	u16 val;

	cmd = (u32)((seq_cmd) & 0x0fff0000);
	val = (u16)((seq_cmd) & 0xffff);

	switch (cmd) {
	case DBMDX_UC_SEQ_CMD_IDLE:
		snprintf(cmd_str, len, "DO NOTHING");
		break;
	case DBMDX_UC_SEQ_CMD_USLEEP:
		snprintf(cmd_str, len, "USLEEP %d usec", (int)val);
		break;
	case DBMDX_UC_SEQ_CMD_MSLEEP:
		snprintf(cmd_str, len, "MSLEEP %d ms", (int)val);
		break;
	case DBMDX_UC_SEQ_CMD_CHANGE_CLK_SRC:
		snprintf(cmd_str, len, "Change Clock Src");
		break;
	case DBMDX_UC_SEQ_CMD_LOAD_ASRP_PARAMS:
		snprintf(cmd_str, len, "Load ASRP Params");
		break;
	case DBMDX_UC_SEQ_CMD_SET_ASRP_DELAY:
		snprintf(cmd_str, len, "Set ASRP Delay");
		break;
	case DBMDX_UC_SEQ_CMD_SET_ASRP_GAIN:
		snprintf(cmd_str, len, "Set ASRP Gain");
		break;
	case DBMDX_UC_SEQ_CMD_SET_AUDIO_ROUTING:
		snprintf(cmd_str, len, "Configure Audio Routing");
		break;
	case DBMDX_UC_SEQ_CMD_CONFIG_MICS:
		snprintf(cmd_str, len, "Configure Mics");
		break;
	case DBMDX_UC_SEQ_CMD_CONFIG_TDM:
		snprintf(cmd_str, len, "Configure Tdm Index %d",
								(int)val);
		break;
	case DBMDX_UC_SEQ_CMD_SET_IOM:
		snprintf(cmd_str, len, "Set Iom type %d", (int)val);
		break;
	case DBMDX_UC_SEQ_CMD_COMPLEX_CLB_1:
		snprintf(cmd_str, len, "Invoke Complex Callback 1");
		break;
	case DBMDX_UC_SEQ_CMD_COMPLEX_CLB_2:
		snprintf(cmd_str, len, "Invoke Complex Callback 2");
		break;
	case DBMDX_UC_SEQ_CMD_COMPLEX_CLB_3:
		snprintf(cmd_str, len, "Invoke Complex Callback 3");
		break;
	case DBMDX_UC_SEQ_CMD_HOOK_ON_INIT:
		snprintf(cmd_str, len,
				"Invoke Customer Hook On Init");
		break;
	case DBMDX_UC_SEQ_CMD_HOOK_VA_VE_REG:
		snprintf(cmd_str, len,
					"Invoke Customer Hook On VA_VE_REG");
		break;
	case DBMDX_UC_SEQ_CMD_HOOK_VA_REG:
		snprintf(cmd_str, len,
					"Invoke Customer Hook On VA_REG");
		break;
	case DBMDX_UC_SEQ_CMD_HOOK_PRE_TDM:
		snprintf(cmd_str, len,
					"Invoke Customer Hook On PRE_TDM");
		break;
	case DBMDX_UC_SEQ_CMD_HOOK_POST_TDM:
		snprintf(cmd_str, len,
					"Invoke Customer Hook On POST_TDM");
		break;
	case DBMDX_UC_SEQ_CMD_HOOK_POST_TDM_VA_VE:
		snprintf(cmd_str, len,
				"Invoke Customer Hook On POST_TDM_VA_VE");
		break;
	case DBMDX_UC_SEQ_CMD_HOOK_POST_TDM_VA:
		snprintf(cmd_str, len,
					"Invoke Customer Hook On POST_TDM_VA");
		break;
	case DBMDX_UC_SEQ_CMD_HOOK_ON_EXIT:
		snprintf(cmd_str, len,
					"Invoke Customer Hook On Exit");
		break;
	default:
		snprintf(cmd_str, len, "Set Register 0x%03x to 0x%04x",
			((cmd >> 16) & 0x0fff), val);
	}
}
static ssize_t dbmdx_dump_usecase(struct dbmdx_private *p,
				struct usecase_config *usecase, char *buf)
{
	int off = 0;
	int i, j;
	struct tdm_config *tdm_conf;
	char cmd_str[100] = {0};

	if (!usecase)
		return -EINVAL;

	off += snprintf(buf + off, PAGE_SIZE - off,
					"\n\t=======Usecase Dump========\n");

	off += snprintf(buf + off, PAGE_SIZE - off,
				"\tUsecase name:\t%s\n", usecase->usecase_name);

	off += snprintf(buf + off, PAGE_SIZE - off, "\tUsecase ID:\t%d\n",
		usecase->id);

	off += snprintf(buf + off, PAGE_SIZE - off, "\tUsecase HW REV:\t%d\n",
		usecase->hw_rev);

	if (p->va_chip_enabled) {
		off += snprintf(buf + off, PAGE_SIZE - off,
				"\tSend VA ASRP Params:\t%s\n",
				usecase->send_va_asrp_parms ? "ON" : "OFF");
		off += snprintf(buf + off, PAGE_SIZE - off,
				"\tVA ASRP Param File Name:\t%s\n",
				usecase->va_asrp_params_file_name);
	}
	if (p->va_ve_chip_enabled) {
		off += snprintf(buf + off, PAGE_SIZE - off,
				"\tSend VA_VE ASRP Params:\t%s\n",
				usecase->send_va_ve_asrp_parms ? "ON" : "OFF");
		off += snprintf(buf + off, PAGE_SIZE - off,
				"\tVA_VE ASRP Param File Name:\t%s\n",
				usecase->va_ve_asrp_params_file_name);
	}

	off += snprintf(buf + off, PAGE_SIZE - off,
			"\tMics Configuration Type:\t%d\n",
			(int)(usecase->config_mics));

	off += snprintf(buf + off, PAGE_SIZE - off,
			"\tMic Freq:\t%u (Hz)\n",
			(int)(usecase->mic_freq));

	for (i = 0; i < 4; i++)
		off += snprintf(buf + off, PAGE_SIZE - off,
				"\tMic cfg 0x%8.8x: 0x%8.8x\n",
				i, usecase->mic_config[i]);

	for (i = 0; i < NUM_OF_AUDIO_ROUTING_CONF; i++)
		off += snprintf(buf + off, PAGE_SIZE - off,
				"\tAudio route cfg 0x%8.8x: 0x%8.8x\n",
				i, usecase->audio_routing_config[i]);
	if (p->va_chip_enabled) {
		off += snprintf(buf + off, PAGE_SIZE - off,
				"\tNumber of VA configs values:\t%d\n",
				usecase->num_of_va_cfg_values);

		for (i = 0; i < usecase->num_of_va_cfg_values; i++) {
			get_uc_command_str(usecase->va_cfg_values[i],
						cmd_str, sizeof(cmd_str));
			off += snprintf(buf + off, PAGE_SIZE - off,
					"\tVA cfg %8.8x: 0x%8.8x\t(%s)\n",
					i, usecase->va_cfg_values[i],
					cmd_str);
		}

	}
	if (p->va_ve_chip_enabled) {
		off += snprintf(buf + off, PAGE_SIZE - off,
				"\tNumber of VA_VE configs values:\t%d\n",
				usecase->num_of_va_ve_cfg_values);


		for (i = 0; i < usecase->num_of_va_ve_cfg_values; i++) {
			get_uc_command_str(usecase->va_ve_cfg_values[i],
						cmd_str, sizeof(cmd_str));
			off += snprintf(buf + off, PAGE_SIZE - off,
					"\tVA cfg %8.8x: 0x%8.8x\t(%s)\n",
					i, usecase->va_ve_cfg_values[i],
					cmd_str);
		}
	}

	if (p->va_chip_enabled) {
		off += snprintf(buf + off, PAGE_SIZE - off,
				"\tSend VA Start Command:\t%s\n",
				usecase->send_va_start_cmd ? "ON" : "OFF");

		off += snprintf(buf + off, PAGE_SIZE - off,
				"\tSend VA Start Command Type:\t%s\n",
				(usecase->va_start_cmd_type ==
					START_CMD_TYPE_TDM) ? "TDM" : "OPMODE");

		off += snprintf(buf + off, PAGE_SIZE - off,
				"\tVA Start Command: 0x%8.8x\n",
				usecase->va_start_cmd);
	}
	if (p->va_ve_chip_enabled) {
		off += snprintf(buf + off, PAGE_SIZE - off,
				"\tSend VA_VE Start Command:\t%s\n",
				usecase->send_va_ve_start_cmd ? "ON" : "OFF");

		off += snprintf(buf + off, PAGE_SIZE - off,
				"\tSend VA_VE Start Command Type:\t%s\n",
				(usecase->va_ve_start_cmd_type ==
					START_CMD_TYPE_TDM) ? "TDM" : "OPMODE");

		off += snprintf(buf + off, PAGE_SIZE - off,
				"\tVA_VE Start Command: 0x%8.8x\n",
				usecase->va_ve_start_cmd);
	}

	if (p->va_chip_enabled) {
		off += snprintf(buf + off, PAGE_SIZE - off,
				"\tVA I2S Master:\t%s\n",
				usecase->i2s_master_clock_va ? "ON" : "OFF");
		off += snprintf(buf + off, PAGE_SIZE - off,
				"\tVA Clock Operation:\t%s\n",
				(usecase->clock_op_va ==
					DBMDX_CLOCK_OP_SWITCH_TO_MASTER_CLOCK) ?
				"Switch To Master CLK" :
				(usecase->clock_op_va ==
					DBMDX_CLOCK_OP_SWITCH_TO_TDM_CLOCK) ?
				"Switch To TDM CLK" : "Configure Master CLK");
		off += snprintf(buf + off, PAGE_SIZE - off,
				"\tVA Clock Config\n");
		off += snprintf(buf + off, PAGE_SIZE - off,
				"\t\tPLL:\t%u Hz\n",
				usecase->clock_config_va.wanted_pll);
		off += snprintf(buf + off, PAGE_SIZE - off,
				"\t\tTL3 PLL:\t%u Hz\n",
				usecase->clock_config_va.wanted_tl3_clk);
		off += snprintf(buf + off, PAGE_SIZE - off,
				"\t\tAHB Clock:\t%u Hz\n",
				usecase->clock_config_va.wanted_ahb_clk);
		off += snprintf(buf + off, PAGE_SIZE - off,
				"\t\tAPB Clock:\t%u Hz\n",
				usecase->clock_config_va.wanted_apb_clk);
		off += snprintf(buf + off, PAGE_SIZE - off,
				"\t\tUse Pll Divider:\t%s\n",
				usecase->clock_config_va.use_pll_post_div ?
				"ON" : "OFF");
	}
	if (p->va_ve_chip_enabled) {
		off += snprintf(buf + off, PAGE_SIZE - off,
				"\t VA_VE I2S Master:\t%s\n",
				usecase->i2s_master_clock_va_ve ? "ON" : "OFF");
		off += snprintf(buf + off, PAGE_SIZE - off,
				"\tVA_VE Clock Operation:\t%s\n",
				(usecase->clock_op_va_ve ==
					DBMDX_CLOCK_OP_SWITCH_TO_MASTER_CLOCK) ?
				"Switch To Master CLK" :
				(usecase->clock_op_va ==
					DBMDX_CLOCK_OP_SWITCH_TO_TDM_CLOCK) ?
				"Switch To TDM CLK" : "Configure Master CLK");

		off += snprintf(buf + off, PAGE_SIZE - off,
				"\tVA_VE Clock Config\n");
		off += snprintf(buf + off, PAGE_SIZE - off,
				"\t\tPLL:\t%u Hz\n",
				usecase->clock_config_va_ve.wanted_pll);
		off += snprintf(buf + off, PAGE_SIZE - off,
				"\t\tTL3 PLL:\t%u Hz\n",
				usecase->clock_config_va_ve.wanted_tl3_clk);
		off += snprintf(buf + off, PAGE_SIZE - off,
				"\t\tAHB Clock:\t%u Hz\n",
				usecase->clock_config_va_ve.wanted_ahb_clk);
		off += snprintf(buf + off, PAGE_SIZE - off,
				"\t\tAPB Clock:\t%u Hz\n",
				usecase->clock_config_va_ve.wanted_apb_clk);
		off += snprintf(buf + off, PAGE_SIZE - off,
				"\t\tUse Pll Divider:\t%s\n",
				usecase->clock_config_va_ve.use_pll_post_div ?
				"ON" : "OFF");

	}
	off += snprintf(buf + off, PAGE_SIZE - off,
				"\tTDM Clock Frequence:\t%d Hz\n",
				usecase->tdm_clock_freq);

	off += snprintf(buf + off, PAGE_SIZE - off,
				"\tNumber of bits:\t%d\n",
				usecase->number_of_bits);

	off += snprintf(buf + off, PAGE_SIZE - off,
				"\tUsecase requires acoustic model:\t%s\n",
				usecase->usecase_requires_amodel ?
					"ON" : "OFF");

	off += snprintf(buf + off, PAGE_SIZE - off,
				"\tAcoustic model detection mode:\t%d\n",
				usecase->usecase_amodel_mode);

	off += snprintf(buf + off, PAGE_SIZE - off,
				"\tUsecase sets detection mode:\t%s\n",
				usecase->usecase_sets_detection_mode ?
					"ON" : "OFF");
	off += snprintf(buf + off, PAGE_SIZE - off,
				"\tUsecase supports userspace buffering:\t%s\n",
				usecase->usecase_supports_us_buffering ?
					"ON" : "OFF");

	if (p->va_chip_enabled)
		off += snprintf(buf + off, PAGE_SIZE - off,
				"\tVA Chip Low Power Mode:\t%s\n",
				usecase->va_chip_low_power_mode ?
					"ON" : "OFF");
	if (p->va_ve_chip_enabled)
		off += snprintf(buf + off, PAGE_SIZE - off,
				"\tVA_VE Chip Low Power Mode:\t%s\n",
				usecase->va_ve_chip_low_power_mode ?
					"ON" : "OFF");

	off += snprintf(buf + off, PAGE_SIZE - off,
					"\t=======TDM Configuration========\n");

	if (p->va_chip_enabled) {

		for (i = 0; i < NUMBER_OF_TDM_CONFIGS_VA; i++) {

			tdm_conf = &(usecase->tdm_configs_va[i]);

			if (!(tdm_conf->enabled))
				continue;

			off += snprintf(buf + off, PAGE_SIZE - off,
					"\tDBMD4/8:TDM%d %s Configuration\n",
					tdm_conf->tdm_index,
					(tdm_conf->tdm_type ==
						TDM_TYPE_TX) ? "TX" : "RX");

			off += snprintf(buf + off, PAGE_SIZE - off,
					"\t\tTDM Start Command 0x%8.8x\n",
					tdm_conf->tdm_reg_config);
			for (j = 0; j < tdm_conf->num_of_io_reg_configs; j++)
				off += snprintf(buf + off, PAGE_SIZE - off,
				"\t\tIO Reg 0x%8.8x\tIO Value 0x%8.8x\n",
					tdm_conf->io_reg_configs[j].addr,
					tdm_conf->io_reg_configs[j].value);
		}
	}

	if (p->va_ve_chip_enabled) {

		for (i = 0; i < NUMBER_OF_TDM_CONFIGS_VA_VE; i++) {

			tdm_conf = &(usecase->tdm_configs_va_ve[i]);

			if (!(tdm_conf->enabled))
				continue;

			off += snprintf(buf + off, PAGE_SIZE - off,
					"\tDBMD2:TDM%d %s Configuration\n",
					tdm_conf->tdm_index,
					(tdm_conf->tdm_type ==
						TDM_TYPE_TX) ? "TX" : "RX");

			off += snprintf(buf + off, PAGE_SIZE - off,
					"\t\tTDM Start Command 0x%8.8x\n",
					tdm_conf->tdm_reg_config);
			for (j = 0; j < tdm_conf->num_of_io_reg_configs; j++)
				off += snprintf(buf + off, PAGE_SIZE - off,
				"\t\tIO Reg 0x%8.8x\tIO Value 0x%8.8x\n",
					tdm_conf->io_reg_configs[j].addr,
					tdm_conf->io_reg_configs[j].value);
		}
	}

	return off;
}
#ifdef DBMDX_LOAD_USECASE_FROM_FILE_SUPPORTED

static void read_u32_le(u32 *value, const void *data)
{
	u8 *data_ptr = (u8 *)data;
	*value = (u32)(((u32)(data_ptr[0])     & 0x000000FF) |
			((u32)(data_ptr[1]<<8)  & 0x0000FF00) |
			((u32)(data_ptr[2]<<16) & 0x00FF0000) |
			((u32)(data_ptr[3]<<24) & 0xFF000000));
}
static void read_tdm_config(struct dbmdx_private *p,
				struct tdm_config *tdm_config, const void *data)
{
	int i;
	u32 val;
	ssize_t cur_offset = 0;


	read_u32_le(&val, data + cur_offset);
	tdm_config->tdm_index = val;
	cur_offset += sizeof(u32);

	read_u32_le(&val, data + cur_offset);
	tdm_config->tdm_type = (enum TDM_TYPE_TX_RX)val;
	cur_offset += sizeof(u32);

	read_u32_le(&val, data + cur_offset);
	tdm_config->tdm_reg_config = val;
	cur_offset += sizeof(u32);

	read_u32_le(&val, data + cur_offset);
	tdm_config->tdm_interface = (enum TDM_INTERFACE)val;
	cur_offset += sizeof(u32);

	read_u32_le(&val, data + cur_offset);
	tdm_config->num_of_io_reg_configs = val;
	cur_offset += sizeof(u32);

	if (tdm_config->num_of_io_reg_configs > NUM_OF_TDM_CONF_REGS) {
		dev_warn(p->dev,
			"%s: num_of_io_reg_configs exceeds maximum (%d)\n",
			__func__, tdm_config->num_of_io_reg_configs);

		tdm_config->num_of_io_reg_configs = NUM_OF_TDM_CONF_REGS;
	}

	for (i = 0; i < NUM_OF_TDM_CONF_REGS; i++) {
		read_u32_le(&val, data + cur_offset);
		tdm_config->io_reg_configs[i].addr = val;
		cur_offset += sizeof(u32);

		read_u32_le(&val, data + cur_offset);
		tdm_config->io_reg_configs[i].value = val;
		cur_offset += sizeof(u32);
	}

}
#endif

static void free_external_usecase_memory(struct usecase_config *usecase)
{

	kfree(usecase->va_ve_cfg_values);
	kfree(usecase->va_cfg_values);
	kfree((char *)(usecase->va_asrp_params_file_name));
	kfree((char *)(usecase->va_ve_asrp_params_file_name));
	kfree((char *)(usecase->usecase_name));

}

#ifdef DBMDX_LOAD_USECASE_FROM_FILE_SUPPORTED
static int load_usecase_from_buffer(struct dbmdx_private *p,
		const void *usecase_data, ssize_t size,
		struct usecase_config *usecase)
{

	int rc = -1;
	int i;
	ssize_t cur_offset = RESERVED_OFFSET;
	ssize_t cur_limit;
	ssize_t cur_length;
	char temp_buffer[MAX_USECASE_FILENAME_LEN+1];
	char *str_buffer;
	u32 val;
	u8 *data = (u8 *)usecase_data;

	if (cur_offset >= size) {
		dev_err(p->dev, "%s: Error: buffer size it too short\n",
			__func__);
		return -1;
	}

	if (cur_offset + MAX_USECASE_NAME_LEN + 1 > size)
		cur_limit = size;
	else
		cur_limit = cur_offset + MAX_USECASE_NAME_LEN + 1;

	for (i = cur_offset; i < cur_limit; i++) {
		temp_buffer[i-cur_offset] = data[i];
		if (data[i] == '\0')
			break;
	}

	if (i >= cur_limit) {
		dev_err(p->dev, "%s: Error: usecase name wasn't found\n",
			__func__);
		rc = -1;
		goto out;
	}
	cur_length = i - cur_offset + 1;
	str_buffer = kzalloc(cur_length*sizeof(u8), GFP_KERNEL);
	if (!str_buffer) {
		rc = -ENOMEM;
		goto out;
	}

	memcpy(str_buffer, temp_buffer, cur_length*sizeof(u8));
	usecase->usecase_name = (const char *)str_buffer;

	cur_offset += cur_length;

	if (cur_offset + sizeof(u32) >= size) {
		dev_err(p->dev, "%s: Error: buffer size it too short\n",
			__func__);
		rc = -1;
		goto err_free;
	}

	read_u32_le(&val, data + cur_offset);
	usecase->id = val;
	cur_offset += sizeof(u32);

	if (cur_offset + sizeof(u32) >= size) {
		dev_err(p->dev, "%s: Error: buffer size it too short\n",
			__func__);
		rc = -1;
		goto err_free;
	}

	read_u32_le(&val, data + cur_offset);
	usecase->hw_rev = val;
	cur_offset += sizeof(u32);

	if (cur_offset + sizeof(u32) >= size) {
		dev_err(p->dev, "%s: Error: buffer size it too short\n",
			__func__);
		rc = -1;
		goto err_free;
	}

	read_u32_le(&val, data + cur_offset);
	usecase->send_va_asrp_parms = (bool)val;
	cur_offset += sizeof(u32);

	if (cur_offset + MAX_USECASE_FILENAME_LEN + 1 > size)
		cur_limit = size;
	else
		cur_limit = cur_offset + MAX_USECASE_FILENAME_LEN + 1;

	for (i = cur_offset; i < cur_limit; i++) {
		temp_buffer[i-cur_offset] = data[i];
		if (data[i] == '\0')
			break;
	}

	if (i >= cur_limit) {
		dev_err(p->dev,
			"%s: Error: va asrp params filename wasn't found\n",
			__func__);
		rc = -1;
		goto err_free;
	}
	cur_length = i - cur_offset + 1;
	str_buffer = kzalloc(cur_length*sizeof(u8), GFP_KERNEL);
	if (!str_buffer) {
		rc = -ENOMEM;
		goto err_free;
	}

	memcpy(str_buffer, temp_buffer, cur_length*sizeof(u8));
	usecase->va_asrp_params_file_name = (const char *)str_buffer;

	cur_offset += cur_length;

	if (cur_offset + sizeof(u32) >= size) {
		dev_err(p->dev, "%s: Error: buffer size it too short\n",
			__func__);
		rc = -1;
		goto err_free2;
	}

	read_u32_le(&val, data + cur_offset);
	usecase->send_va_ve_asrp_parms = (bool)val;
	cur_offset += sizeof(u32);

	if (cur_offset + MAX_USECASE_FILENAME_LEN + 1 > size)
		cur_limit = size;
	else
		cur_limit = cur_offset + MAX_USECASE_FILENAME_LEN + 1;

	for (i = cur_offset; i < cur_limit; i++) {
		temp_buffer[i-cur_offset] = data[i];
		if (data[i] == '\0')
			break;
	}

	if (i >= cur_limit) {
		dev_err(p->dev,
			"%s: Error: va_ve asrp params filename wasn't found\n",
			__func__);
		rc = -1;
		goto err_free2;
	}
	cur_length = i - cur_offset + 1;
	str_buffer = kzalloc(cur_length*sizeof(u8), GFP_KERNEL);
	if (!str_buffer) {
		rc = -ENOMEM;
		goto err_free2;
	}

	memcpy(str_buffer, temp_buffer, cur_length*sizeof(u8));
	usecase->va_ve_asrp_params_file_name = (const char *)str_buffer;

	cur_offset += cur_length;

	if (cur_offset + sizeof(u32) >= size) {
		dev_err(p->dev, "%s: Error: buffer size it too short\n",
			__func__);
		rc = -1;
		goto err_free3;
	}



	read_u32_le(&val, data + cur_offset);
	usecase->config_mics = (enum MIC_CONFIG_TYPE)val;
	cur_offset += sizeof(u32);


	if (cur_offset + 4*sizeof(u32) >= size) {
		dev_err(p->dev, "%s: Error: buffer size it too short\n",
			__func__);
		rc = -1;
		goto err_free3;
	}

	for (i = 0; i < 4; i++) {
		read_u32_le(&val, data + cur_offset);
		usecase->mic_config[i] = val;
		cur_offset += sizeof(u32);
	}

	if (cur_offset + NUM_OF_AUDIO_ROUTING_CONF*sizeof(u32) >= size) {
		dev_err(p->dev, "%s: Error: buffer size it too short\n",
			__func__);
		rc = -1;
		goto err_free3;
	}

	for (i = 0; i < NUM_OF_AUDIO_ROUTING_CONF; i++) {
		read_u32_le(&val, data + cur_offset);
		usecase->audio_routing_config[i] = val;
		cur_offset += sizeof(u32);
	}
	read_u32_le(&val, data + cur_offset);
	usecase->num_of_tdm_configs = val;
	cur_offset += sizeof(u32);

	if (cur_offset + usecase->num_of_tdm_configs*sizeof(struct tdm_config)
		>= size) {
		dev_err(p->dev, "%s: Error: buffer size it too short\n",
			__func__);
		rc = -1;
		goto err_free3;
	}

	for (i = 0; i < usecase->num_of_tdm_configs; i++) {
		read_tdm_config(p,
			&(usecase->tdm_configs[i]), data + cur_offset);
		cur_offset += sizeof(struct tdm_config);
	}

	if (cur_offset + sizeof(u32) >= size) {
		dev_err(p->dev, "%s: Error: buffer size it too short\n",
			__func__);
		rc = -1;
		goto err_free3;
	}
	read_u32_le(&val, data + cur_offset);
	usecase->num_of_va_cfg_values = val;
	cur_offset += sizeof(u32);

	if (cur_offset + usecase->num_of_va_cfg_values*sizeof(u32) >= size) {
		dev_err(p->dev, "%s: Error: buffer size it too short\n",
			__func__);
		rc = -1;
		goto err_free3;
	}

	usecase->va_cfg_values =
		kzalloc(usecase->num_of_va_cfg_values*sizeof(u32),
			GFP_KERNEL);
	if (!(usecase->va_cfg_values)) {
		rc = -ENOMEM;
		goto err_free3;
	}

	for (i = 0; i < usecase->num_of_va_cfg_values; i++) {
		read_u32_le(&val, data + cur_offset);
		usecase->va_cfg_values[i] = val;
		cur_offset += sizeof(u32);
	}

	if (cur_offset + sizeof(u32) >= size) {
		dev_err(p->dev, "%s: Error: buffer size it too short\n",
			__func__);
		rc = -1;
		goto err_free4;
	}

	read_u32_le(&val, data + cur_offset);
	usecase->num_of_va_ve_cfg_values = val;
	cur_offset += sizeof(u32);

	if (cur_offset + usecase->num_of_va_ve_cfg_values*sizeof(u32) >= size) {
		dev_err(p->dev, "%s: Error: buffer size it too short\n",
			__func__);
		rc = -1;
		goto err_free4;
	}

	usecase->va_ve_cfg_values =
		kzalloc(usecase->num_of_va_ve_cfg_values*sizeof(u32),
			GFP_KERNEL);
	if (!(usecase->va_ve_cfg_values)) {
		rc = -ENOMEM;
		goto err_free4;
	}

	for (i = 0; i < usecase->num_of_va_ve_cfg_values; i++) {
		read_u32_le(&val, data + cur_offset);
		usecase->va_ve_cfg_values[i] = val;
		cur_offset += sizeof(u32);
	}

	if (cur_offset + sizeof(u32) >= size) {
		dev_err(p->dev, "%s: Error: buffer size it too short\n",
			__func__);
		rc = -1;
		goto err_free5;
	}

	read_u32_le(&val, data + cur_offset);
	usecase->send_va_start_cmd = (bool)val;
	cur_offset += sizeof(u32);

	if (cur_offset + sizeof(u32) >= size) {
		dev_err(p->dev, "%s: Error: buffer size it too short\n",
			__func__);
		rc = -1;
		goto err_free5;
	}

	read_u32_le(&val, data + cur_offset);
	usecase->va_start_cmd_type = (enum START_CMD_TYPE)val;
	cur_offset += sizeof(u32);

	if (cur_offset + sizeof(u32) >= size) {
		dev_err(p->dev, "%s: Error: buffer size it too short\n",
			__func__);
		rc = -1;
		goto err_free5;
	}


	read_u32_le(&val, data + cur_offset);
	usecase->va_start_cmd = val;
	cur_offset += sizeof(u32);

	if (cur_offset + sizeof(u32) >= size) {
		dev_err(p->dev, "%s: Error: buffer size it too short\n",
			__func__);
		rc = -1;
		goto err_free5;
	}

	read_u32_le(&val, data + cur_offset);
	usecase->send_va_ve_start_cmd = (bool)val;
	cur_offset += sizeof(u32);

	if (cur_offset + sizeof(u32) >= size) {
		dev_err(p->dev, "%s: Error: buffer size it too short\n",
			__func__);
		rc = -1;
		goto err_free5;
	}

	read_u32_le(&val, data + cur_offset);
	usecase->va_ve_start_cmd_type = (enum START_CMD_TYPE)val;
	cur_offset += sizeof(u32);

	if (cur_offset + sizeof(u32) >= size) {
		dev_err(p->dev, "%s: Error: buffer size it too short\n",
			__func__);
		rc = -1;
		goto err_free5;
	}

	read_u32_le(&val, data + cur_offset);
	usecase->va_ve_start_cmd = val;
	cur_offset += sizeof(u32);

	if (cur_offset + sizeof(u32) >= size) {
		dev_err(p->dev, "%s: Error: buffer size it too short\n",
			__func__);
		rc = -1;
		goto err_free5;
	}

	read_u32_le(&val, data + cur_offset);
	usecase->change_clock_src = (bool)val;
	cur_offset += sizeof(u32);

	if (cur_offset + sizeof(u32) >= size) {
		dev_err(p->dev, "%s: Error: buffer size it too short\n",
			__func__);
		rc = -1;
		goto err_free5;
	}

	read_u32_le(&val, data + cur_offset);
	usecase->tdm_clock_freq = (enum TDM_CLK_FREQ_CONF)val;
	cur_offset += sizeof(u32);

	if (cur_offset + sizeof(u32) >= size) {
		dev_err(p->dev, "%s: Error: buffer size it too short\n",
			__func__);
		rc = -1;
		goto err_free5;
	}

	read_u32_le(&val, data + cur_offset);
	usecase->number_of_bits = val;
	cur_offset += sizeof(u32);

	if (cur_offset + sizeof(u32) >= size) {
		dev_err(p->dev, "%s: Error: buffer size it too short\n",
			__func__);
		rc = -1;
		goto err_free5;
	}

	read_u32_le(&val, data + cur_offset);
	usecase->usecase_requires_amodel = (bool)val;
	cur_offset += sizeof(u32);

	if (cur_offset + sizeof(u32) >= size) {
		dev_err(p->dev, "%s: Error: buffer size it too short\n",
			__func__);
		rc = -1;
		goto err_free5;
	}

	read_u32_le(&val, data + cur_offset);
	usecase->usecase_amodel_mode = val;
	cur_offset += sizeof(u32);

	if (cur_offset + sizeof(u32) > size) {
		dev_err(p->dev, "%s: Error: buffer size it too short\n",
			__func__);
		rc = -1;
		goto err_free5;
	}

	read_u32_le(&val, data + cur_offset);
	usecase->usecase_sets_detection_mode = (bool)val;
	cur_offset += sizeof(u32);

	if (cur_offset + sizeof(u32) > size) {
		dev_err(p->dev, "%s: Error: buffer size it too short\n",
			__func__);
		rc = -1;
		goto err_free5;
	}

	read_u32_le(&val, data + cur_offset);
	usecase->usecase_supports_us_buffering = (bool)val;
	cur_offset += sizeof(u32);

	if (cur_offset + sizeof(u32) > size) {
		dev_err(p->dev, "%s: Error: buffer size it too short\n",
			__func__);
		rc = -1;
		goto err_free5;
	}

	read_u32_le(&val, data + cur_offset);
	usecase->va_chip_low_power_mode = (bool)val;
	cur_offset += sizeof(u32);

	if (cur_offset + sizeof(u32) > size) {
		dev_err(p->dev, "%s: Error: buffer size it too short\n",
			__func__);
		rc = -1;
		goto err_free5;
	}

	read_u32_le(&val, data + cur_offset);
	usecase->va_ve_chip_low_power_mode = (bool)val;
	cur_offset += sizeof(u32);

	rc = 0;
	goto out;

err_free5:
	kfree(usecase->va_ve_cfg_values);
err_free4:
	kfree(usecase->va_cfg_values);
err_free3:
	kfree((char *)(usecase->va_ve_asrp_params_file_name));
err_free2:
	kfree((char *)(usecase->va_asrp_params_file_name));
err_free:
	kfree((char *)(usecase->usecase_name));
out:
	dev_dbg(p->dev, "%s: done (%d)\n", __func__, rc);

	return rc;
}
#endif

int dbmdx_asrp_indirect_param_write(struct dbmdx_private *p,
					u32 block_id, u16 offset, u16 value)
{
	int ret = 0;

	dev_dbg(p->dev, "%s\n", __func__);

	if (!p)
		return -EAGAIN;

	ret = dbmdx_send_cmd(p,
		(DBMDX_REGN_ASRP_BLOCKID_LOW_PART | (u16)block_id), NULL);
	if (ret < 0) {
		dev_err(p->dev, "%s: Error setting asrp low block id\n",
			__func__);
		ret = -EIO;
		goto out;
	}

	ret = dbmdx_send_cmd(p, (DBMDX_REGN_ASRP_BLOCKID_HIGH_PART |
			(u16)((u32)(block_id >> 16) & 0xffff)), NULL);
	if (ret < 0) {
		dev_err(p->dev, "%s: Error setting asrp hi block id\n",
			__func__);
		ret = -EIO;
		goto out;
	}

	ret = dbmdx_send_cmd(p, (DBMDX_REGN_ASRP_ENABLE_OFFSET | offset), NULL);
	if (ret < 0) {
		dev_err(p->dev, "%s: Error setting asrp offset\n", __func__);
		ret = -EIO;
		goto out;
	}

	ret = dbmdx_send_cmd(p, (DBMDX_REGN_ASRP_VALUE | value), NULL);
	if (ret < 0) {
		dev_err(p->dev, "%s: Error setting asrp value\n", __func__);
		ret = -EIO;
		goto out;
	}

	dev_dbg(p->dev,
		"%s: ASRP Block_ID=0x%08x Offset=0x%04x was set to 0x%04x\n",
		__func__, block_id, offset, value);

out:
	return ret;
}
static int dbmdx_va_ve_reset_tdm(struct dbmdx_private *p)
{
	int ret = 0, ind;
	int tdm_enable_mask = 0;

	dev_dbg(p->dev, "%s: Reset TDM lines\n", __func__);

#ifdef VA_VE_I2S_MASTER
	if (p->va_chip_enabled && (p->tdm_enable & DBMDX_TDM_1_VA)) {

		ret = dbmdx_switch_to_va_chip_interface(p, DBMDX_CMD_INTERFACE);
		if (ret) {
			dev_err(p->dev,
				"%s Error switching to (VA) CMD interface\n",
				__func__);
			ret = -EIO;
		}

		ret = dbmdx_send_cmd(p, DBMDX_REGN_TDM_ACTIVATION_CTL |
						DBMDX_REGV_CONFIG_TDM_1, NULL);
		if (ret < 0) {
			dev_err(p->dev, "%s: failed reset tdm\n", __func__);
			ret = -EIO;
			goto out;
		}

		msleep(DBMDX_MSLEEP_TDM_DEACTIVATION);

	}

	ret = configure_usecase_clocks(p, p->idle_usecase);
	if (ret < 0) {
		dev_err(p->dev, "%s: failed to change clk src\n", __func__);
		ret = -EIO;
		goto out;
	}

	if (p->va_ve_chip_enabled && (p->tdm_enable & DBMDX_TDM_2_VA_VE)) {

		ret = dbmdx_switch_to_va_ve_chip_interface(p,
							DBMDX_CMD_INTERFACE);
		if (ret) {
			dev_err(p->dev,
				"%s Error switching to (VA_VE) CMD interface\n",
				__func__);
			ret = -EIO;
		}

		ret = dbmdx_send_cmd(p, DBMDX_REGN_TDM_ACTIVATION_CTL |
						DBMDX_REGV_CONFIG_TDM_2, NULL);
		if (ret < 0) {
			dev_err(p->dev, "%s: failed reset tdm\n", __func__);
			ret = -EIO;
			goto out;
		}

		msleep(DBMDX_MSLEEP_TDM_DEACTIVATION*2);
	}

#else
	if (p->va_ve_chip_enabled && (p->tdm_enable & DBMDX_TDM_2_VA_VE)) {

		ret = dbmdx_switch_to_va_ve_chip_interface(p,
							DBMDX_CMD_INTERFACE);
		if (ret) {
			dev_err(p->dev,
				"%s Error switching to (VA_VE) CMD interface\n",
				__func__);
			ret = -EIO;
		}

		ret = dbmdx_send_cmd(p, DBMDX_REGN_TDM_ACTIVATION_CTL |
						DBMDX_REGV_CONFIG_TDM_2, NULL);
		if (ret < 0) {
			dev_err(p->dev, "%s: failed reset tdm\n", __func__);
			ret = -EIO;
			goto out;
		}

		msleep(DBMDX_MSLEEP_TDM_DEACTIVATION);
	}

	if (p->va_chip_enabled && (p->tdm_enable & DBMDX_TDM_1_VA)) {

		ret = dbmdx_switch_to_va_chip_interface(p, DBMDX_CMD_INTERFACE);
		if (ret) {
			dev_err(p->dev,
				"%s Error switching to (VA) CMD interface\n",
				__func__);
			ret = -EIO;
		}

		ret = dbmdx_send_cmd(p, DBMDX_REGN_TDM_ACTIVATION_CTL |
						DBMDX_REGV_CONFIG_TDM_1, NULL);
		if (ret < 0) {
			dev_err(p->dev, "%s: failed reset tdm\n", __func__);
			ret = -EIO;
			goto out;
		}

		msleep(DBMDX_MSLEEP_TDM_DEACTIVATION);

	}
#endif
	if (p->va_ve_chip_enabled) {

		ret = dbmdx_switch_to_va_ve_chip_interface(p,
							DBMDX_CMD_INTERFACE);
		if (ret) {
			dev_err(p->dev,
				"%s Error switching to (VA_VE) CMD interface\n",
				__func__);
			ret = -EIO;
		}

		for (ind = 0; ind < 4; ind++) {

			tdm_enable_mask = (DBMDX_TDM_VA_VE_BASE << ind);

			if (!(p->tdm_enable & tdm_enable_mask))
				continue;

			ret = dbmdx_send_cmd(p,
				     DBMDX_REGN_TDM_ACTIVATION_CTL | ind, NULL);
			if (ret < 0) {
				dev_err(p->dev,
					"%s: failed to tdm index\n", __func__);
				ret = -EIO;
				goto out;
			}

			ret = dbmdx_send_cmd(p, DBMDX_REGN_TDM_TX_CONFIG |
						DBMDX_REGV_CLEAR_TDM_CFG, NULL);
			if (ret < 0) {
				dev_err(p->dev,
					"%s: failed to reset tdm config\n",
					__func__);
				ret = -EIO;
				goto out;
			}

			ret = dbmdx_send_cmd(p, DBMDX_REGN_TDM_RX_CONFIG |
						DBMDX_REGV_CLEAR_TDM_CFG, NULL);
			if (ret < 0) {
				dev_err(p->dev,
					"%s: failed to reset tdm config\n",
					__func__);
				ret = -EIO;
				goto out;
			}

		}
	}

	if (p->va_chip_enabled) {

		ret = dbmdx_switch_to_va_chip_interface(p, DBMDX_CMD_INTERFACE);
		if (ret) {
			dev_err(p->dev,
				"%s Error switching to (VA) CMD interface\n",
				__func__);
			ret = -EIO;
		}
		for (ind = 0; ind < 2; ind++) {

			tdm_enable_mask = (DBMDX_TDM_VA_BASE << ind);

			if (!(p->tdm_enable & tdm_enable_mask))
				continue;

			ret = dbmdx_send_cmd(p,
				DBMDX_REGN_TDM_ACTIVATION_CTL | ind, NULL);
			if (ret < 0) {
				dev_err(p->dev,
					"%s: failed to tdm index\n", __func__);
				ret = -EIO;
				goto out;
			}

			ret = dbmdx_send_cmd(p, DBMDX_REGN_TDM_TX_CONFIG |
						DBMDX_REGV_CLEAR_TDM_CFG, NULL);
			if (ret < 0) {
				dev_err(p->dev,
					"%s: failed to reset tdm config\n",
					__func__);
				ret = -EIO;
				goto out;
			}

			ret = dbmdx_send_cmd(p, DBMDX_REGN_TDM_RX_CONFIG |
						DBMDX_REGV_CLEAR_TDM_CFG, NULL);
			if (ret < 0) {
				dev_err(p->dev,
					"%s: failed to reset tdm config\n",
					__func__);
				ret = -EIO;
				goto out;
			}
		}
	}

out:
	if (!ret)
		p->va_ve_flags.tdm_is_reset = true;
	return ret;

}
static int dbmdx_va_ve_send_idle_usecase_config(struct dbmdx_private *p,
							bool allow_sleep)
{
	int ret, ret1;
	u16 cur_config = 0xffff;
	u16 cur_mic_config;

	if (!p->pdata->feature_va_ve) {
		dev_err(p->dev, "%s: VA_VE feature not enabled\n", __func__);
		return -EINVAL;
	}

	if (p->va_ve_flags.idle_usecase_active) {
		dev_info(p->dev,
			"%s IDLE usecase has been already configured\n",
			__func__);
		return 0;
	}

	ret = customer_dbmdx_idle_usecase_config_on_init(p);
	if (ret < 0) {
		dev_err(p->dev, "%s: custom UC_IDLE_CFG_ONINIT error\n",
			__func__);
		ret = -EIO;
	}
	if (!(p->va_ve_flags.tdm_is_reset)) {
		ret = dbmdx_va_ve_reset_tdm(p);
		if (ret) {
			dev_err(p->dev,
				"%s Error resetting TDM lines\n", __func__);
			goto out;
		}
	} else {
		if (p->va_ve_chip_enabled) {
			ret = dbmdx_switch_to_va_ve_chip_interface(p,
							DBMDX_CMD_INTERFACE);
			if (ret) {
				dev_err(p->dev,
				"%s Error switching to (VA_VE) CMD interface\n",
					__func__);
				ret = -EIO;
				goto out;
			}

			ret = dbmdx_send_cmd(p,
				     DBMDX_REGN_OPERATION_MODE | DBMDX_IDLE,
				     NULL);
			if (ret < 0) {
				dev_err(p->dev, "%s: failed to set idle mode\n",
					__func__);
				goto out;
			}
		}
		if (p->va_chip_enabled) {
			ret = dbmdx_switch_to_va_chip_interface(p,
							DBMDX_CMD_INTERFACE);
			if (ret) {
				dev_err(p->dev,
				"%s Error switching to (VA_) CMD interface\n",
					__func__);
				ret = -EIO;
				goto out;
			}

			ret = dbmdx_send_cmd(p,
				     DBMDX_REGN_OPERATION_MODE | DBMDX_IDLE,
				     NULL);
			if (ret < 0) {
				dev_err(p->dev, "%s: failed to set idle mode\n",
					__func__);
				goto out;
			}
		}

		msleep(DBMDX_MSLEEP_SET_MODE);
	}

	if (p->va_ve_chip_enabled) {

		ret = dbmdx_switch_to_va_ve_chip_interface(p,
							DBMDX_CMD_INTERFACE);
		if (ret) {
			dev_err(p->dev,
				"%s Error switching to (VA_VE) CMD interface\n",
				__func__);
			ret = -EIO;
		}

		ret = dbmdx_send_cmd(p,
				DBMDX_REGN_GENERAL_CONFIG_2, NULL);
		if (ret < 0) {
			dev_err(p->dev,
				"%s: failed to reset tdm config\n", __func__);
			ret = -EIO;
			goto out;
		}

		/* read current configuration */
		ret = dbmdx_send_cmd(p,
				     DBMDX_REGN_AUDIO_PROCESSING_CONFIG,
				     &cur_config);
		if (ret < 0) {
			dev_err(p->dev,
				"%s: failed to read AUDIO_PROCESSING_CONFIG\n",
				__func__);
			ret = -EIO;
			goto out;
		}

		if (cur_config != 0) {
			ret = dbmdx_send_cmd(p,
					DBMDX_REGN_AUDIO_PROCESSING_CONFIG,
					NULL);

			if (ret < 0) {
				dev_err(p->dev,
					"%s: failed to reset APC\n", __func__);
				ret = -EIO;
				goto out;
			}
			usleep_range(DBMDX_USLEEP_AUDIO_PROCESSING_CONFIG,
				DBMDX_USLEEP_AUDIO_PROCESSING_CONFIG + 1000);
		}
	}

	if (p->va_chip_enabled) {

		ret = dbmdx_switch_to_va_chip_interface(p, DBMDX_CMD_INTERFACE);
		if (ret) {
			dev_err(p->dev,
				"%s Error switching to (VA) CMD interface\n",
				__func__);
			ret = -EIO;
		}

		ret = dbmdx_send_cmd(p,
				DBMDX_REGN_GENERAL_CONFIG_2, NULL);
		if (ret < 0) {
			dev_err(p->dev,
				"%s: failed to reset tdm config\n", __func__);
			ret = -EIO;
			goto out;
		}

		/* read current configuration */
		ret = dbmdx_send_cmd(p,
				     DBMDX_REGN_AUDIO_PROCESSING_CONFIG,
				     &cur_config);
		if (ret < 0) {
			dev_err(p->dev,
				"%s: failed to read AUDIO_PROCESSING_CONFIG\n",
				__func__);
			ret = -EIO;
			goto out;
		}

		if (cur_config != 0) {
			ret = dbmdx_send_cmd(p,
					DBMDX_REGN_AUDIO_PROCESSING_CONFIG,
					NULL);

			if (ret < 0) {
				dev_err(p->dev,
					"%s: failed to reset APC\n", __func__);
				ret = -EIO;
				goto out;
			}
			usleep_range(DBMDX_USLEEP_AUDIO_PROCESSING_CONFIG,
				DBMDX_USLEEP_AUDIO_PROCESSING_CONFIG + 1000);
		}


	}


	if (p->mics_connected_to_va_ve_chip) {
		ret = dbmdx_switch_to_va_ve_chip_interface(p,
							DBMDX_CMD_INTERFACE);
		if (ret) {
			dev_err(p->dev,
				"%s Error switching to (VA_VE) CMD interface\n",
				__func__);
			ret = -EIO;
			goto out;
		}
	} else {
		ret = dbmdx_switch_to_va_chip_interface(p, DBMDX_CMD_INTERFACE);
		if (ret) {
			dev_err(p->dev,
				"%s Error switching to (VA) CMD interface\n",
				__func__);
			ret = -EIO;
			goto out;
		}
	}

	/* Disable all mics */

	ret = dbmdx_send_cmd(p, DBMDX_REGN_FOURTH_MICROPHONE_CONFIG,
				      &cur_mic_config);

	if (ret < 0) {
		dev_err(p->dev, "%s: failed to read microphone4 value\n",
			__func__);
		ret = -EIO;
		goto out;
	}

	if (cur_mic_config != DBMDX_MIC_DISABLE_VAL &&	cur_mic_config != 0) {

		ret = dbmdx_send_cmd(p, DBMDX_REGN_FOURTH_MICROPHONE_CONFIG |
							DBMDX_MIC_DISABLE_VAL,
				      NULL);

		if (ret < 0) {
			dev_err(p->dev, "%s: failed to microphone4 mode 0x%x\n",
				__func__, 0);
			ret = -EIO;
			goto out;
		}
		usleep_range(DBMDX_USLEEP_MIC_CONFIG,
					DBMDX_USLEEP_MIC_CONFIG + 1000);
	}

	ret = dbmdx_send_cmd(p, DBMDX_REGN_THIRD_MICROPHONE_CONFIG,
				      &cur_mic_config);

	if (ret < 0) {
		dev_err(p->dev, "%s: failed to read microphone3 value\n",
			__func__);
		ret = -EIO;
		goto out;
	}

	if (cur_mic_config != DBMDX_MIC_DISABLE_VAL &&	cur_mic_config != 0) {

		ret = dbmdx_send_cmd(p, DBMDX_REGN_THIRD_MICROPHONE_CONFIG |
							DBMDX_MIC_DISABLE_VAL,
				      NULL);

		if (ret < 0) {
			dev_err(p->dev, "%s: failed to microphone3 mode 0x%x\n",
				__func__, 0);
			ret = -EIO;
			goto out;
		}
		usleep_range(DBMDX_USLEEP_MIC_CONFIG,
					DBMDX_USLEEP_MIC_CONFIG + 1000);
	}
	ret = dbmdx_send_cmd(p, DBMDX_REGN_SECOND_MICROPHONE_CONFIG,
				      &cur_mic_config);

	if (ret < 0) {
		dev_err(p->dev, "%s: failed to read microphone2 value\n",
			__func__);
		ret = -EIO;
		goto out;
	}

	if (cur_mic_config != DBMDX_MIC_DISABLE_VAL &&	cur_mic_config != 0) {

		ret = dbmdx_send_cmd(p, DBMDX_REGN_SECOND_MICROPHONE_CONFIG |
							DBMDX_MIC_DISABLE_VAL,
				      NULL);

		if (ret < 0) {
			dev_err(p->dev, "%s: failed to microphone2 mode 0x%x\n",
				__func__, 0);
			ret = -EIO;
			goto out;
		}
		usleep_range(DBMDX_USLEEP_MIC_CONFIG,
					DBMDX_USLEEP_MIC_CONFIG + 1000);
	}

	ret = dbmdx_send_cmd(p, DBMDX_REGN_FIRST_MICROPHONE_CONFIG,
				      &cur_mic_config);

	if (ret < 0) {
		dev_err(p->dev, "%s: failed to read microphone1 value\n",
			__func__);
		ret = -EIO;
		goto out;
	}

	if (cur_mic_config != DBMDX_MIC_DISABLE_VAL &&	cur_mic_config != 0) {

		ret = dbmdx_send_cmd(p, DBMDX_REGN_FIRST_MICROPHONE_CONFIG |
							DBMDX_MIC_DISABLE_VAL,
				      NULL);

		if (ret < 0) {
			dev_err(p->dev, "%s: failed to microphone1 mode 0x%x\n",
				__func__, 0);
			ret = -EIO;
			goto out;
		}
		usleep_range(DBMDX_USLEEP_MIC_CONFIG,
					DBMDX_USLEEP_MIC_CONFIG + 1000);
	}

#ifndef VA_VE_I2S_MASTER
	ret = configure_usecase_clocks(p, p->idle_usecase);
	if (ret < 0) {
		dev_err(p->dev, "%s: failed to change clk src\n", __func__);
		ret = -EIO;
		goto out;
	}
#endif

	ret = customer_dbmdx_idle_usecase_config_on_exit(p);
	if (ret < 0) {
		dev_err(p->dev, "%s: custom UC_IDLE_CFG_ONEXIT error\n",
			__func__);
		ret = -EIO;
	}


	p->va_flags.mode = DBMDX_IDLE;
	p->va_ve_flags.mode = DBMDX_IDLE;
	p->va_ve_flags.usecase_mode = DBMDX_IDLE;
	p->va_flags.irq_inuse = 0;

	if (allow_sleep) {
		p->va_flags.sleep_not_allowed = false;
		p->va_ve_flags.sleep_not_allowed = false;

		/* Try to go to Low Power */
		dbmdx_set_power_mode(p, DBMDX_PM_FALLING_ASLEEP);
	}
	p->va_ve_flags.idle_usecase_active = 1;
out:
	p->va_ve_flags.active_usecase_id = 0;
	p->va_ve_flags.cur_usecase = NULL;

	if (p->va_chip_enabled) {
		ret1 = dbmdx_switch_to_va_chip_interface(p,
						DBMDX_CMD_INTERFACE);
		if (ret1) {
			dev_err(p->dev,
				"%s Error switching to (VA) CMD interface\n",
				__func__);
			ret = -EIO;
		}
	} else {
		ret1 = dbmdx_switch_to_va_ve_chip_interface(p,
						DBMDX_CMD_INTERFACE);
		if (ret1) {
			dev_err(p->dev,
				"%s Error switching to (VA_VE) CMD interface\n",
				__func__);
			ret = -EIO;
		}
	}

	return ret;
}

static int dbmdx_va_ve_get_mic_config_from_mask(struct dbmdx_private *p,
						u32 mic_mask,
						u32 *cur_mic_config,
						u32 *masked_mic_config)
{
	int cur_mic = 0;
	bool mic_selected;
	int index;


	for (index = 0; index < 4; index++) {
		mic_selected = (bool)((mic_mask & (0x0001 << index)) != 0);
		if (mic_selected)
			masked_mic_config[cur_mic++] = cur_mic_config[index];
	}

	for (index = cur_mic; index < 4; index++)
		masked_mic_config[index] = 0x0;

	for (index = 0; index < 4; index++)
		dev_dbg(p->dev, "%s:\tMasked Mic cfg 0x%8.8x: 0x%8.8x\n",
				__func__, index, masked_mic_config[index]);

	return 0;
}
#if defined(DBMDX_PSAD_SUPPORTED)
static int dbmdx_va_ve_update_psad_data_regs(struct dbmdx_private *p,
						u16 trigger_len_in_frames,
						u16 trigger_offset_mode,
						u16 trigger_mode)
{

	int ret;

	if (dbmdx_switch_to_va_ve_chip_interface(p, DBMDX_CMD_INTERFACE)) {
		dev_err(p->dev,
			"%s Error switching to (VA) CMD interface\n",
			__func__);
		return -EIO;
	}

	ret = dbmdx_send_cmd(p,	DBMDX_REGN_ASRP_PSAD_TRIGGER_LENGTH |
					trigger_len_in_frames, NULL);
	if (ret < 0) {
		dev_err(p->dev,	"%s: set 0x%x reg error\n", __func__,
				DBMDX_REGN_ASRP_PSAD_TRIGGER_LENGTH);
		return -EIO;
	}

	ret = dbmdx_send_cmd(p,	DBMDX_REGN_ASRP_PSAD_TRIGGER_OFFSET_MODE |
					trigger_offset_mode, NULL);
	if (ret < 0) {
		dev_err(p->dev,	"%s: set 0x%x reg error\n", __func__,
				DBMDX_REGN_ASRP_PSAD_TRIGGER_OFFSET_MODE);
		return -EIO;
	}

	ret = dbmdx_send_cmd(p,	DBMDX_REGN_ASRP_PSAD_TRIGGER_MODE |
					trigger_mode, NULL);
	if (ret < 0) {
		dev_err(p->dev,	"%s: set 0x%x reg error\n", __func__,
				DBMDX_REGN_ASRP_PSAD_TRIGGER_MODE);
		return -EIO;
	}

	if (dbmdx_switch_to_va_chip_interface(p, DBMDX_CMD_INTERFACE)) {
		dev_err(p->dev,	"%s Error switching to (VA) CMD interface\n",
					__func__);
		return -EIO;
	}

	dev_info(p->dev,
		"%s: PSAD Data was updated: Trigger Len: 0x%x Frames\n",
		__func__, trigger_len_in_frames);

	return 0;

}


static int dbmdx_va_ve_update_psad_data_variable(struct dbmdx_private *p,
					u32 trigger_length_in_samples,
					u32 frame_length_in_samples)
{

	int ret;
	u32 irq_delay_in_mframes = DBMDX_INTERRUPT_DELAY_TIME;
	u32 frame_length_in_ms = (16*1024) / (frame_length_in_samples);
	u32 interrupt_delay_frames = ((irq_delay_in_mframes / 1000) /
						(frame_length_in_ms / 1000)) +
					((7*204) / frame_length_in_samples);
	u32 trigger_len_in_ms = trigger_length_in_samples / 16;
	u32 trigger_len_in_frames = trigger_len_in_ms / frame_length_in_ms;
	u32 trigger_offset_mode = DBMDX_SYSTEM_FRAME_OFFSET +
						interrupt_delay_frames;

	ret = dbmdx_va_ve_update_psad_data_regs(p, (u16)trigger_len_in_frames,
						(u16)trigger_offset_mode, 1);
	if (ret < 0) {
		dev_err(p->dev,	"%s: Error updating PSAD regs\n", __func__);
		return -EIO;
	}

	return 0;
}
static int dbmdx_get_trigger_length(struct dbmdx_private *p,
					u32 *trigger_length_in_samples,
					u32 model_idx)
{

	if (model_idx == DBMDX_OKG_MODEL_SELECTED) {
		*trigger_length_in_samples =
				DBMDX_OKG_TRIGGER_LENGTH_IN_SAMPLES;
		return 0;
	}

	dev_err(p->dev, "%s: PSAD Model %d is not supported\n",
				__func__, (int)model_idx);
	*trigger_length_in_samples = 0;
	return -EINVAL;
}

static int dbmdx_va_ve_update_psad_data(struct dbmdx_private *p,
					bool use_preset_vals,
					u32 preset_idx)
{
	int ret;
	struct usecase_config *cur_usecase;

	cur_usecase = p->va_ve_flags.cur_usecase;

	if (cur_usecase == NULL)
		return -EINVAL;

	if (!p->va_ve_chip_enabled) {
		dev_err(p->dev, "%s: VA_VE chip is not enabled\n", __func__);
		return -EINVAL;
	}

	if (!(cur_usecase->send_va_ve_asrp_parms)) {
		dev_dbg(p->dev,
			"%s: ASRP is not running, no need to set PSAD Data\n",
			__func__);
		return 0;
	}

	if (p->psad_disabled) {
		dev_dbg(p->dev,	"%s: PSAD is disabled\n", __func__);
		return 0;
	}

	if (use_preset_vals) {
		if (preset_idx == DBMDX_OKG_MODEL_SELECTED) {
			ret = dbmdx_va_ve_update_psad_data_regs(p,
							0x10, 0x5, 0x1);
			if (ret < 0) {
				dev_err(p->dev,
					"%s: Error updating PSAD data\n",
				__func__);
				return -EIO;
			}
		} else {
			dev_err(p->dev, "%s: PSAD Preset %d is not supported\n",
				__func__, (int)preset_idx);
			return -EINVAL;
		}
	} else {
		u32 trigger_length_in_samples;
		u32 frame_length_in_samples = DBMDX_FRAME_LENGTH;

		ret = dbmdx_get_trigger_length(p, &trigger_length_in_samples,
						preset_idx);

		if (ret < 0) {
			dev_err(p->dev, "%s: Error getting trigger length\n",
				__func__);
			return -EIO;
		}

		ret = dbmdx_va_ve_update_psad_data_variable(p,
					trigger_length_in_samples,
					frame_length_in_samples);
		if (ret < 0) {
			dev_err(p->dev,	"%s: Error updating PSAD data\n",
				__func__);
			return -EIO;
		}

	}

	return 0;
}
#endif

static int dbmdx_va_ve_set_asrp_gain(struct dbmdx_private *p, u32 gain_type,
								bool mute)
{

	int ret;
	u16 asrp_tx_out_gain = 0;
	u16 asrp_vcpf_out_gain = 0;
	u16 asrp_rx_out_gain = 0;

	if (!mute) {
		asrp_tx_out_gain =  (u16)(p->pdata->asrp_tx_out_gain);
		asrp_vcpf_out_gain = (u16)(p->pdata->asrp_vcpf_out_gain);
		asrp_rx_out_gain = (u16)(p->pdata->asrp_rx_out_gain);
	}

	dev_dbg(p->dev,	"%s: Usecase ASRP Gain Type: 0x%x Mute: %d\n", __func__,
					gain_type, mute);
	if ((gain_type & ASRP_TX_OUT_GAIN) && (asrp_tx_out_gain > 0 || mute)) {

		ret = dbmdx_send_cmd(p,	DBMDX_REGN_ASRP_OUTPUT_GAIN_DEST_SEL |
					DBMDX_REGV_ASRP_TX_OUT, NULL);
		if (ret < 0) {
			dev_err(p->dev,	"%s: set 0x%x reg error\n", __func__,
					DBMDX_REGN_ASRP_OUTPUT_GAIN_DEST_SEL);
			return -EIO;
		}

		ret = dbmdx_send_cmd(p,	DBMDX_REGN_ASRP_OUTPUT_GAIN_DEST_VALUE |
						asrp_tx_out_gain, NULL);
		if (ret < 0) {
			dev_err(p->dev,	"%s: set 0x%x reg error\n", __func__,
					DBMDX_REGN_ASRP_OUTPUT_GAIN_DEST_VALUE);
			return -EIO;
		}
	}
	if ((gain_type & ASRP_VCPF_OUT_GAIN) &&
				(asrp_vcpf_out_gain > 0 || mute)) {

		ret = dbmdx_send_cmd(p,	DBMDX_REGN_ASRP_OUTPUT_GAIN_DEST_SEL |
					DBMDX_REGV_ASRP_VCPF_OUT, NULL);
		if (ret < 0) {
			dev_err(p->dev,	"%s: set 0x%x reg error\n", __func__,
					DBMDX_REGN_ASRP_OUTPUT_GAIN_DEST_SEL);
			return -EIO;
		}

		ret = dbmdx_send_cmd(p,	DBMDX_REGN_ASRP_OUTPUT_GAIN_DEST_VALUE |
						asrp_vcpf_out_gain, NULL);
		if (ret < 0) {
			dev_err(p->dev,	"%s: set 0x%x reg error\n", __func__,
					DBMDX_REGN_ASRP_OUTPUT_GAIN_DEST_VALUE);
			return -EIO;
		}
	}

	if ((gain_type & ASRP_RX_OUT_GAIN) && (asrp_rx_out_gain > 0 || mute)) {

		ret = dbmdx_send_cmd(p,	DBMDX_REGN_ASRP_OUTPUT_GAIN_DEST_SEL |
					DBMDX_REGV_ASRP_RX_OUT, NULL);
		if (ret < 0) {
			dev_err(p->dev,	"%s: set 0x%x reg error\n", __func__,
					DBMDX_REGN_ASRP_OUTPUT_GAIN_DEST_SEL);
			return -EIO;
		}

		ret = dbmdx_send_cmd(p,	DBMDX_REGN_ASRP_OUTPUT_GAIN_DEST_VALUE |
							asrp_rx_out_gain, NULL);
		if (ret < 0) {
			dev_err(p->dev,	"%s: set 0x%x reg error\n", __func__,
					DBMDX_REGN_ASRP_OUTPUT_GAIN_DEST_VALUE);
			return -EIO;
		}
	}

	return 0;

}

static int dbmdx_mute_current_usecase(struct dbmdx_private *p, bool mute)
{
#ifdef RR_CHECK
	int ret =0;
	struct usecase_config *cur_usecase;

	cur_usecase = p->va_ve_flags.cur_usecase;

	if (cur_usecase == NULL)
		return -EINVAL;

	if (p->asrp_runs_on_vt) {
		if (!cur_usecase->send_va_asrp_parms) {
			dev_err(p->dev,	"%s No need to MUTE LP Usecase\n",
				__func__);
			return 0;
		}
		ret = dbmdx_switch_to_va_chip_interface(p,
							DBMDX_CMD_INTERFACE);
		if (ret) {
			dev_err(p->dev,
				"%s Error switching to (VA) CMD interface\n",
				__func__);
			return -EIO;
		}

	} else {
		/* Check for Low Power mode */
		if ((cur_usecase->va_ve_start_cmd_type ==
						START_CMD_TYPE_OPMODE) &&
			((cur_usecase->va_ve_start_cmd == DBMDX_IDLE) ||
			 (cur_usecase->va_ve_start_cmd == DBMDX_HIBERNATE))) {
			dev_err(p->dev,	"%s No need to MUTE LP Usecase\n",
				__func__);
			return 0;

		}

		ret = dbmdx_switch_to_va_ve_chip_interface(p,
							DBMDX_CMD_INTERFACE);
		if (ret) {
			dev_err(p->dev,
				"%s Error switching to (VA_VE) CMD interface\n",
				__func__);
			return -EIO;
		}

	}

	ret = dbmdx_va_ve_set_asrp_gain(p, cur_usecase->asrp_output_gain_type,
						mute);

	if (ret < 0) {
		dev_err(p->dev,	"%s: Error setting ASRP gain\n", __func__);
		return -EIO;
	}

	dev_info(p->dev, "%s Current usecase was %s\n",
				__func__, mute ? "MUTED" : "UNMUTED");
#endif
	return 0;
}
static int configure_usecase_iom(struct dbmdx_private *p)
{
	int ret = 0;

#if defined(DBMDX_I2S_STREAMING_SUPPORTED) || \
		defined(DBMDX_VC_I2S_STREAMING_SUPPORTED)
	u32 cur_io_reg_val;

	ret = dbmdx_switch_to_va_ve_chip_interface(p,
							DBMDX_CMD_INTERFACE);
	if (ret) {
		dev_err(p->dev, "%s Error switching to (VA_VE) CMD interface\n",
			__func__);
		return -EIO;
	}

	ret = dbmdx_io_register_read(p, DBMDX_HW_REGN_IOM1, &cur_io_reg_val);
	if (ret < 0) {
		dev_err(p->dev,	"%s: failed to read IOM1\n", __func__);
		return -EIO;
	}

	ret = dbmdx_io_register_write(p, DBMDX_HW_REGN_IOM1,
#if defined(VC_RX_PROCESS_SUPPORTED)
				(cur_io_reg_val | 0x08080000));
#else
				(cur_io_reg_val | 0x08000000));
#endif
	if (ret < 0) {
		dev_err(p->dev,	"%s: failed to update IOM1\n", __func__);
		return -EIO;
	}

#endif
	return ret;
}

static int configure_usecase_iom_2(struct dbmdx_private *p)
{
	int ret = 0;

#if !defined(VA_I2S_MASTER)
	u32 cur_io_reg_val;

	ret = dbmdx_switch_to_va_chip_interface(p,
							DBMDX_CMD_INTERFACE);
	if (ret) {
		dev_err(p->dev, "%s Error switching to (VA) CMD interface\n",
			__func__);
		return -EIO;
	}

	ret = dbmdx_io_register_read(p, DBMDX_HW_REGN_IOM1, &cur_io_reg_val);
	if (ret < 0) {
		dev_err(p->dev,	"%s: failed to read IOM1\n", __func__);
		return -EIO;
	}

	ret = dbmdx_io_register_write(p, DBMDX_HW_REGN_IOM1,
#if defined(VC_RX_PROCESS_SUPPORTED)
				(cur_io_reg_val &  0xcfcfffff));
#else
				(cur_io_reg_val & 0xcfffffff));
#endif
	if (ret < 0) {
		dev_err(p->dev,	"%s: failed to update IOM1\n", __func__);
		return -EIO;
	}

#endif
	return ret;
}

static int configure_usecase_tdm(struct dbmdx_private *p,
					struct usecase_config *usecase,
					enum dbmdx_chip chip,
					u16 tdm_index)
{
	struct tdm_config *cur_tdm_cfg = NULL;
	int ret, reg_ind;
	struct io_register *cur_io_reg;


	switch (tdm_index) {
	case DBMDX_UC_SEQ_CMD_VAL_TDM0_VA_TX:
		cur_tdm_cfg = &(usecase->tdm_configs_va[0]);
		break;
	case DBMDX_UC_SEQ_CMD_VAL_TDM0_VA_RX:
		cur_tdm_cfg = &(usecase->tdm_configs_va[1]);
		break;
	case DBMDX_UC_SEQ_CMD_VAL_TDM1_VA_TX:
		cur_tdm_cfg = &(usecase->tdm_configs_va[2]);
		break;
	case DBMDX_UC_SEQ_CMD_VAL_TDM1_VA_RX:
		cur_tdm_cfg = &(usecase->tdm_configs_va[3]);
		break;
	case DBMDX_UC_SEQ_CMD_VAL_TDM0_VA_VE_TX:
		cur_tdm_cfg = &(usecase->tdm_configs_va_ve[0]);
		break;
	case DBMDX_UC_SEQ_CMD_VAL_TDM0_VA_VE_RX:
		cur_tdm_cfg = &(usecase->tdm_configs_va_ve[1]);
		break;
	case DBMDX_UC_SEQ_CMD_VAL_TDM1_VA_VE_TX:
		cur_tdm_cfg = &(usecase->tdm_configs_va_ve[2]);
		break;
	case DBMDX_UC_SEQ_CMD_VAL_TDM1_VA_VE_RX:
		cur_tdm_cfg = &(usecase->tdm_configs_va_ve[3]);
		break;
	case DBMDX_UC_SEQ_CMD_VAL_TDM2_VA_VE_TX:
		cur_tdm_cfg = &(usecase->tdm_configs_va_ve[4]);
		break;
	case DBMDX_UC_SEQ_CMD_VAL_TDM2_VA_VE_RX:
		cur_tdm_cfg = &(usecase->tdm_configs_va_ve[5]);
		break;
	case DBMDX_UC_SEQ_CMD_VAL_TDM3_VA_VE_TX:
		cur_tdm_cfg = &(usecase->tdm_configs_va_ve[6]);
		break;
	case DBMDX_UC_SEQ_CMD_VAL_TDM3_VA_VE_RX:
		cur_tdm_cfg = &(usecase->tdm_configs_va_ve[7]);
		break;
	default:
		dev_err(p->dev, "%s BAD TDM Index: %u\n", __func__, tdm_index);
		return -EINVAL;
	}

	if (!(cur_tdm_cfg->enabled)) {
		dev_warn(p->dev,
		"%s: TDM Is disabled Chip: %s, TDM Index: %u\n",
		__func__, chip == DBMDX_CHIP_VA ? "VA" : "VA_VE", tdm_index);
		return 0;
	}

	if (cur_tdm_cfg->tdm_interface == TDM_INTERFACE_VA) {

		if (chip != DBMDX_CHIP_VA) {
			dev_err(p->dev,
				"%s: TDM Chip: VA, but conf belongs to VA_VE\n",
				__func__);
			return 0;

		}
		if (!p->va_chip_enabled) {
			dev_err(p->dev,
				"%s: TDM Chip: VA, but VA chip is disabled\n",
				__func__);
			return 0;

		}
		ret = dbmdx_switch_to_va_chip_interface(p, DBMDX_CMD_INTERFACE);
		if (ret) {
			dev_err(p->dev,
				"%s Error switching to (VA) CMD interface\n",
					__func__);
			return -EIO;
		}
	} else {

		if (chip != DBMDX_CHIP_VA_VE) {
			dev_err(p->dev,
				"%s: TDM Chip: VA_VE, but conf belongs to VA\n",
				__func__);
			return 0;
		}
		if (!p->va_ve_chip_enabled) {
			dev_err(p->dev,
				"%s: TDM Chip: VA_VE, but VA_VE chip is disabled\n",
				__func__);
			return 0;

		}
		ret = dbmdx_switch_to_va_ve_chip_interface(p,
							DBMDX_CMD_INTERFACE);
		if (ret) {
			dev_err(p->dev,
				"%s Error switching to (VA_VE) CMD interface\n",
					__func__);
			return -EIO;
		}
	}

	ret = dbmdx_send_cmd(p,
			DBMDX_REGN_TDM_ACTIVATION_CTL | cur_tdm_cfg->tdm_index,
			NULL);
	if (ret < 0) {
		dev_err(p->dev, "%s: failed to tdm index\n", __func__);
		return -EIO;

	}

	if (cur_tdm_cfg->tdm_type == TDM_TYPE_TX)
		ret = dbmdx_send_cmd(p,	DBMDX_REGN_TDM_TX_CONFIG |
					cur_tdm_cfg->tdm_reg_config,
					NULL);
	else
		ret = dbmdx_send_cmd(p,	DBMDX_REGN_TDM_RX_CONFIG |
					cur_tdm_cfg->tdm_reg_config,
					NULL);

	if (ret < 0) {
		dev_err(p->dev, "%s: failed to set tdm config\n", __func__);
		return -EIO;

	}

	for (reg_ind = 0; reg_ind < cur_tdm_cfg->num_of_io_reg_configs;
								reg_ind++) {
		cur_io_reg = &(cur_tdm_cfg->io_reg_configs[reg_ind]);
		ret = dbmdx_io_register_write(p, cur_io_reg->addr,
						 cur_io_reg->value);
		if (ret < 0) {
			dev_err(p->dev, "%s: failed to set io reg\n", __func__);
			return -EIO;
		}
	}

	return 0;
}

static int configure_usecase_mics(struct dbmdx_private *p,
					struct usecase_config *usecase,
					enum dbmdx_chip chip)
{
	int ret = 0;
	u32 *cur_mic_config;
	u32 user_selected_mic_config[4];


	if (p->mics_connected_to_va_ve_chip) {

		if (chip == DBMDX_CHIP_VA) {
			dev_err(p->dev,
				"%s Mics attached to VA_VE, but cmd is VA\n",
				__func__);
			return -EINVAL;
		}

		ret = dbmdx_switch_to_va_ve_chip_interface(p,
							DBMDX_CMD_INTERFACE);
		if (ret) {
			dev_err(p->dev,
				"%s Error switching to (VA_VE) CMD interface\n",
				__func__);
			return -EIO;
		}
	} else {
		if (chip == DBMDX_CHIP_VA_VE) {
			dev_err(p->dev,
				"%s Mics attached to VA, but cmd is VA_VE\n",
				__func__);
			return -EINVAL;
		}

		ret = dbmdx_switch_to_va_chip_interface(p, DBMDX_CMD_INTERFACE);
		if (ret) {
			dev_err(p->dev,
				"%s Error switching to (VA) CMD interface\n",
				__func__);
			return -EIO;
		}
	}

	if (usecase->config_mics == DO_NOT_CONFIG_MICS)
		return 0;

	if (p->va_ve_mic_mode == 0)
		cur_mic_config = usecase->mic_config;
	else
		cur_mic_config = p->pdata->va_ve_mic_config;

	if (usecase->config_mics == MIC_CONFIG_BY_USER_MASK) {
		dbmdx_va_ve_get_mic_config_from_mask(p,
						p->pdata->va_ve_mic_mask,
						cur_mic_config,
						user_selected_mic_config);
		cur_mic_config = user_selected_mic_config;
	}

	if (cur_mic_config[0] != DBMDX_MIC_DISABLE_VAL &&
					cur_mic_config[0] != 0) {

		ret = dbmdx_send_cmd(p,
				DBMDX_REGN_FIRST_MICROPHONE_CONFIG |
				cur_mic_config[0],
				NULL);
		if (ret < 0) {
			dev_err(p->dev,
				"%s: failed to set microphone1 mode 0x%x\n",
				__func__, cur_mic_config[0]);
			return -EIO;
		}
		usleep_range(DBMDX_USLEEP_MIC_CONFIG,
					DBMDX_USLEEP_MIC_CONFIG + 1000);
	}

	if (cur_mic_config[1] != DBMDX_MIC_DISABLE_VAL &&
					cur_mic_config[1] != 0) {

		ret = dbmdx_send_cmd(p,
				DBMDX_REGN_SECOND_MICROPHONE_CONFIG |
				cur_mic_config[1],
				NULL);
		if (ret < 0) {
			dev_err(p->dev,
				"%s: failed to set microphone2 mode 0x%x\n",
				__func__, cur_mic_config[1]);
			return -EIO;
		}
		usleep_range(DBMDX_USLEEP_MIC_CONFIG,
					DBMDX_USLEEP_MIC_CONFIG + 1000);
	}

	if (cur_mic_config[2] != DBMDX_MIC_DISABLE_VAL &&
					cur_mic_config[2] != 0) {

		ret = dbmdx_send_cmd(p,
				DBMDX_REGN_THIRD_MICROPHONE_CONFIG |
				cur_mic_config[2],
				NULL);

		if (ret < 0) {
			dev_err(p->dev,
				"%s: failed to set microphone3 mode 0x%x\n",
				__func__, cur_mic_config[2]);
			return -EIO;
		}

		usleep_range(DBMDX_USLEEP_MIC_CONFIG,
					DBMDX_USLEEP_MIC_CONFIG + 1000);
	}

	if (cur_mic_config[3] != DBMDX_MIC_DISABLE_VAL &&
					cur_mic_config[3] != 0) {

		ret = dbmdx_send_cmd(p,
				DBMDX_REGN_FOURTH_MICROPHONE_CONFIG |
				cur_mic_config[3],
				NULL);
		if (ret < 0) {
			dev_err(p->dev,
				"%s: failed to set microphone4 mode 0x%x\n",
				__func__, cur_mic_config[3]);
			return -EIO;
		}

		usleep_range(DBMDX_USLEEP_MIC_CONFIG,
					DBMDX_USLEEP_MIC_CONFIG + 1000);
	}

	return 0;
}

static int load_usecase_asrp_params(struct dbmdx_private *p,
					struct usecase_config *usecase,
					enum dbmdx_chip chip)
{
	int ret = 0;
	u16 cur_val;

	if (chip == DBMDX_CHIP_VA && p->va_chip_enabled &&
					usecase->send_va_asrp_parms) {

		ret = dbmdx_switch_to_va_chip_interface(p, DBMDX_CMD_INTERFACE);
		if (ret) {
			dev_err(p->dev,
				"%s Error switching to (VA) CMD interface\n",
				__func__);
			return -EIO;
		}
		if (!(p->va_load_asrp_params_options &
			DBMDX_ASRP_PARAMS_OPTIONS_ALWAYS_RELOAD) &&
			p->va_ve_flags.va_last_loaded_asrp_params_file_name &&
		!strcmp(p->va_ve_flags.va_last_loaded_asrp_params_file_name,
			usecase->va_asrp_params_file_name)) {
			dev_dbg(p->dev,
				"%s VA ASRP params have been already loaded\n",
				__func__);

		} else {
			/* read current configuration */
			ret = dbmdx_send_cmd(p,
					     DBMDX_REGN_AUDIO_PROCESSING_CONFIG,
					     &cur_val);
			if (ret < 0) {
				dev_err(p->dev,
				"%s: failed to read AUDIO_PROCESSING_CONFIG\n",
					__func__);
				return -EIO;
			}

			if (cur_val != 0) {
				ret = dbmdx_send_cmd(p,
					DBMDX_REGN_AUDIO_PROCESSING_CONFIG,
						NULL);

				if (ret < 0) {
					dev_err(p->dev,
						"%s: failed to reset APC\n",
						__func__);
					return -EIO;
				}
				usleep_range(
					DBMDX_USLEEP_AUDIO_PROCESSING_CONFIG,
					DBMDX_USLEEP_AUDIO_PROCESSING_CONFIG +
					1000);
			}

			ret = dbmdx_va_load_asrp_params(p,
				usecase->va_asrp_mem_loc,
				usecase->va_asrp_params_file_name);
			if (ret < 0) {
				dev_err(p->dev,
					"%s: failed to update VA ASRP params\n",
					__func__);
				return -EIO;
			}
			dev_dbg(p->dev,
			"%s VA ASRP params were successfully updated.\n",
				__func__);

			p->va_ve_flags.va_last_loaded_asrp_params_file_name =
				usecase->va_asrp_params_file_name;
		}
	} else if (chip == DBMDX_CHIP_VA && p->va_ve_chip_enabled &&
					usecase->send_va_ve_asrp_parms) {
		ret = dbmdx_switch_to_va_ve_chip_interface(p,
							DBMDX_CMD_INTERFACE);
		if (ret) {
			dev_err(p->dev,
				"%s Error switching to (VA_VE) CMD interface\n",
				__func__);
			return -EIO;
		}
		if (!(p->va_ve_load_asrp_params_options &
			DBMDX_ASRP_PARAMS_OPTIONS_ALWAYS_RELOAD) &&
		p->va_ve_flags.va_ve_last_loaded_asrp_params_file_name &&
		!strcmp(p->va_ve_flags.va_ve_last_loaded_asrp_params_file_name,
			usecase->va_ve_asrp_params_file_name)) {
			dev_dbg(p->dev,
			"%s VA_VE ASRP params have been already loaded\n",
				__func__);

		} else {
			/* read current configuration */
			ret = dbmdx_send_cmd(p,
					     DBMDX_REGN_AUDIO_PROCESSING_CONFIG,
					     &cur_val);
			if (ret < 0) {
				dev_err(p->dev,
				"%s: failed to read AUDIO_PROCESSING_CONFIG\n",
					__func__);
				return -EIO;
			}

			if (cur_val != 0) {
				ret = dbmdx_send_cmd(p,
					DBMDX_REGN_AUDIO_PROCESSING_CONFIG,
						NULL);

				if (ret < 0) {
					dev_err(p->dev,
						"%s: failed to reset APC\n",
						__func__);
					return -EIO;
				}
				usleep_range(
					DBMDX_USLEEP_AUDIO_PROCESSING_CONFIG,
					DBMDX_USLEEP_AUDIO_PROCESSING_CONFIG +
					1000);
			}

			ret = dbmdx_va_load_asrp_params(p,
				usecase->va_ve_asrp_mem_loc,
				usecase->va_ve_asrp_params_file_name);
			if (ret < 0) {
				dev_err(p->dev,
				"%s: failed to update VA_VE ASRP params\n",
					__func__);
				return -EIO;
			}

			dev_dbg(p->dev,
			"%s VA_VE ASRP params were successfully updated.\n",
				__func__);

			p->va_ve_flags.va_ve_last_loaded_asrp_params_file_name =
				usecase->va_ve_asrp_params_file_name;

		}
	}

	return 0;
}


static int process_usecase_cfg_seq_cmd(struct dbmdx_private *p,
					struct usecase_config *usecase,
					enum dbmdx_chip chip,
					u32 seq_cmd)
{
	u32 cmd;
	u16 arg;
	u16 cur_val;
	int ret = 0;
	int i;
	bool asrp_cmd_enabled = false;

	cmd = (u32)((seq_cmd) & 0x0fff0000);
	arg = (u16)((seq_cmd) & 0xffff);

	if (chip == DBMDX_CHIP_VA_VE) {
		ret = dbmdx_switch_to_va_ve_chip_interface(p,
							DBMDX_CMD_INTERFACE);
		if (ret) {
			dev_err(p->dev,
				"%s Error switching to (VA_VE) CMD interface\n",
				__func__);
			return -EIO;
		}
		asrp_cmd_enabled = true;
	} else {
		ret = dbmdx_switch_to_va_chip_interface(p, DBMDX_CMD_INTERFACE);
		if (ret) {
			dev_err(p->dev,
				"%s Error switching to (VA) CMD interface\n",
				__func__);
			return -EIO;
		}

		if (p->asrp_runs_on_vt)
			asrp_cmd_enabled = true;
	}

	switch (cmd) {
	case DBMDX_UC_SEQ_CMD_IDLE:
		break;
	case DBMDX_UC_SEQ_CMD_USLEEP:
		usleep_range(cur_val, cur_val + 100);
		break;
	case DBMDX_UC_SEQ_CMD_MSLEEP:
		msleep(cur_val);
		break;
	case DBMDX_UC_SEQ_CMD_CHANGE_CLK_SRC:
		if (usecase->change_clock_src) {
			ret = configure_usecase_clocks(p, usecase);
			if (ret < 0) {
				dev_err(p->dev,
					"%s: failed to change clk src\n",
					__func__);
				ret = -EIO;
				goto out;
			}
			dev_dbg(p->dev,
				"%s Clock source was successfully updated.\n",
				__func__);
		}
		break;
	case DBMDX_UC_SEQ_CMD_LOAD_ASRP_PARAMS:
		ret = load_usecase_asrp_params(p, usecase, chip);
		if (ret < 0) {
			dev_err(p->dev,
					"%s: failed load UC ASRP Params\n",
					__func__);
				ret = -EIO;
				goto out;
		}
		break;
	case DBMDX_UC_SEQ_CMD_SET_ASRP_DELAY:
		if (asrp_cmd_enabled && p->pdata->asrp_delay > 0) {
			ret = dbmdx_send_cmd(p,
					DBMDX_REGN_ASRP_USER_DEFINED_DELAY |
					(u16)(p->pdata->asrp_delay), NULL);
			if (ret < 0) {
				dev_err(p->dev, "%s: set 0x%x reg error\n",
						__func__,
					DBMDX_REGN_ASRP_USER_DEFINED_DELAY);
				ret = -EIO;
				goto out;
			}
		}
		break;
	case DBMDX_UC_SEQ_CMD_SET_ASRP_GAIN:
		if (usecase ->id == 0x920) {
			ret = dbmdx_send_cmd(p,	DBMDX_REGN_ASRP_OUTPUT_GAIN_DEST_SEL |
			DBMDX_REGV_ASRP_TX_OUT, NULL);
			if (ret < 0) {
				dev_err(p->dev,	"%s: set 0x%x reg error\n", __func__,
						DBMDX_REGN_ASRP_OUTPUT_GAIN_DEST_SEL);
				return -EIO;
			}

			dev_info(p->dev,"usecase_ide = 0x%x, set ASRP_Gain = 0x59f\n",usecase ->id );
			ret = dbmdx_send_cmd(p,	DBMDX_REGN_ASRP_OUTPUT_GAIN_DEST_VALUE |
							0x59f, NULL);
			if (ret < 0) {
				dev_err(p->dev,	"%s: set 0x%x reg error\n", __func__,
						DBMDX_REGN_ASRP_OUTPUT_GAIN_DEST_VALUE);
				return -EIO;
			}

		}
		else {
			if (asrp_cmd_enabled) {
				ret = dbmdx_va_ve_set_asrp_gain(p,
							usecase->asrp_output_gain_type,
							false);
				if (ret < 0) {
					dev_err(p->dev,	"%s: Error setting ASRP gain\n",
								__func__);
					ret = -EIO;
					goto out;
				}
			}
		}
		break;
	case DBMDX_UC_SEQ_CMD_SET_AUDIO_ROUTING:
		for (i = 0; i < NUM_OF_AUDIO_ROUTING_CONF; i++) {

			cur_val = (u16)(((usecase->audio_routing_config[i])
								& 0xf000));
			cur_val = cur_val >> 12;

			/* The index should be in the range [0:7] */
			if (cur_val >= NUM_OF_AUDIO_ROUTING_CONF)
				continue;

			ret = dbmdx_send_cmd(p,
				DBMDX_REGN_AUDIO_PROCESSING_ROUTING |
				(u32)usecase->audio_routing_config[i], NULL);
			if (ret < 0) {
				dev_err(p->dev,
					"%s: failed config VA_VE register\n",
					__func__);
				ret = -EIO;
				goto out;
			}
		}
		break;
	case DBMDX_UC_SEQ_CMD_CONFIG_MICS:
		ret = configure_usecase_mics(p, usecase, chip);
		if (ret < 0) {
			dev_err(p->dev,	"%s: failed to config UC Mics\n",
					__func__);
			ret = -EIO;
			goto out;
		}
		break;
	case DBMDX_UC_SEQ_CMD_CONFIG_TDM:
		ret = configure_usecase_tdm(p, usecase, chip, arg);
		p->va_ve_flags.tdm_is_reset = false;
		if (ret < 0) {
			dev_err(p->dev,	"%s: failed to config UC TDM\n",
					__func__);
			ret = -EIO;
			goto out;
		}
		break;
	case DBMDX_UC_SEQ_CMD_SET_IOM:
		if (arg == 2)
			ret = configure_usecase_iom_2(p);
		else
			ret = configure_usecase_iom(p);
		if (ret < 0) {
			dev_err(p->dev,	"%s: failed to config IOM\n",
					__func__);
			ret = -EIO;
			goto out;
		}
		break;
	case DBMDX_UC_SEQ_CMD_COMPLEX_CLB_1:
		if (!(usecase->complex_clb_1)) {
			dev_err(p->dev,	"%s: CLB_1 is not set\n", __func__);
			ret = -EIO;
			goto out;
		}
		ret = usecase->complex_clb_1(p, usecase);
		if (ret < 0) {
			dev_err(p->dev,	"%s: Error during CLB_1\n", __func__);
			ret = -EIO;
			goto out;
		}
		break;
	case DBMDX_UC_SEQ_CMD_COMPLEX_CLB_2:
		if (!(usecase->complex_clb_2)) {
			dev_err(p->dev,	"%s: CLB_2 is not set\n", __func__);
			ret = -EIO;
			goto out;
		}
		ret = usecase->complex_clb_2(p, usecase);
		if (ret < 0) {
			dev_err(p->dev,	"%s: Error during CLB_2\n", __func__);
			ret = -EIO;
			goto out;
		}
		break;
	case DBMDX_UC_SEQ_CMD_COMPLEX_CLB_3:
		if (!(usecase->complex_clb_3)) {
			dev_err(p->dev,	"%s: CLB_3 is not set\n", __func__);
			ret = -EIO;
			goto out;
		}
		ret = usecase->complex_clb_3(p, usecase);
		if (ret < 0) {
			dev_err(p->dev,	"%s: Error during CLB_3\n", __func__);
			ret = -EIO;
			goto out;
		}
		break;
	case DBMDX_UC_SEQ_CMD_HOOK_ON_INIT:
		ret = customer_dbmdx_send_usecase_config_on_init(p, usecase);
		if (ret < 0) {
			dev_err(p->dev, "%s: custom UC_CFG_ONINIT error\n",
				__func__);
			ret = -EIO;
			goto out;
		}
		break;
	case DBMDX_UC_SEQ_CMD_HOOK_VA_VE_REG:
		ret =
		customer_dbmdx_send_usecase_config_on_va_ve_reg_cfg(p, usecase);
		if (ret < 0) {
			dev_err(p->dev,
				"%s: custom UC_CFG_ON_VE_VA_REG_CFG error\n",
				__func__);
			ret = -EIO;
			goto out;
		}
		break;
	case DBMDX_UC_SEQ_CMD_HOOK_VA_REG:
		ret = customer_dbmdx_send_usecase_config_on_va_reg_cfg(p,
								usecase);
		if (ret < 0) {
			dev_err(p->dev,
				"%s: custom UC_CFG_ON_VA_REG_CFG error\n",
				__func__);
			ret = -EIO;
			goto out;
		}
		break;
	case DBMDX_UC_SEQ_CMD_HOOK_PRE_TDM:
		ret = customer_dbmdx_send_usecase_config_on_pre_tdm(p, usecase);
		if (ret < 0) {
			dev_err(p->dev,
				"%s: custom UC_CFG_PRETDM error\n",
				__func__);
			ret = -EIO;
			goto out;
		}
		break;
	case DBMDX_UC_SEQ_CMD_HOOK_POST_TDM:
		ret = customer_dbmdx_send_usecase_config_on_post_tdm(p,
								usecase);
		if (ret < 0) {
			dev_err(p->dev,
				"%s: custom UC_CFG_POSTTDM error\n",
				__func__);
			ret = -EIO;
			goto out;
		}
		break;
	case DBMDX_UC_SEQ_CMD_HOOK_POST_TDM_VA_VE:
		ret =
		customer_dbmdx_send_usecase_config_on_post_tdm_va_ve_reg(p,
								usecase);
		if (ret < 0) {
			dev_err(p->dev,
				"%s: custom UC_CFG_ON_POSTTDM_VA_VE_REG err\n",
				__func__);
			ret = -EIO;
			goto out;
		}
		break;
	case DBMDX_UC_SEQ_CMD_HOOK_POST_TDM_VA:
		ret = customer_dbmdx_send_usecase_config_on_post_tdm_va_reg(p,
								usecase);
		if (ret < 0) {
			dev_err(p->dev,
				"%s: custom UC_CFG_ON_POSTTDM_VA_REG err\n",
				__func__);
			ret = -EIO;
			goto out;
		}
		break;
	case DBMDX_UC_SEQ_CMD_HOOK_ON_EXIT:
		ret = customer_dbmdx_send_usecase_config_on_exit(p, usecase);
		if (ret < 0) {
			dev_err(p->dev,	"%s: custom UC_CFG_ON_EXIT error\n",
				__func__);
			ret = -EIO;
			goto out;
		}
		break;
	default:
		ret = dbmdx_send_cmd(p, seq_cmd, NULL);
		if (ret < 0) {
			dev_err(p->dev,
				"%s: failed to config register\n",
				__func__);
			ret = -EIO;
			goto out;
		}
		break;

	}
out:
	return ret;
}


static int dbmdx_va_ve_send_usecase_config(struct dbmdx_private *p,
					struct usecase_config *usecase)
{
	unsigned int i;
	int ret, ret2;

	if (!p->pdata->feature_va_ve) {
		dev_err(p->dev, "%s: VA_VE feature not enabled\n", __func__);
		return -EINVAL;
	}

	dev_dbg(p->dev, "%s Configure usecase %d (%s)\n", __func__,
		usecase->id, usecase->usecase_name);

	p->va_flags.sleep_not_allowed = true;
	p->va_ve_flags.sleep_not_allowed = true;


	if (p->va_ve_chip_enabled) {
		if (usecase->num_of_va_ve_cfg_values > 0) {
			ret = dbmdx_switch_to_va_ve_chip_interface(p,
							DBMDX_CMD_INTERFACE);
			if (ret) {
				dev_err(p->dev,
				"%s Error switching to (VA_VE) CMD interface\n",
					__func__);
				ret = -EIO;
				goto out;
			}
		}

		for (i = 0; i < usecase->num_of_va_ve_cfg_values; i++) {

			ret = process_usecase_cfg_seq_cmd(p, usecase,
						DBMDX_CHIP_VA_VE,
						usecase->va_ve_cfg_values[i]);

			if (ret < 0) {
				dev_err(p->dev,
				"%s: Failed to process VA_VE usecase seq cmd\n",
					__func__);
				ret = -EIO;
				goto out;
			}
		}
	}

	if (p->va_chip_enabled) {

		if (usecase->num_of_va_cfg_values > 0) {
			ret = dbmdx_switch_to_va_chip_interface(p,
							DBMDX_CMD_INTERFACE);
			if (ret) {
				dev_err(p->dev,
				"%s Error switching to (VA) CMD interface\n",
					__func__);
				ret = -EIO;
				goto out;
			}
		}

		for (i = 0; i < usecase->num_of_va_cfg_values; i++) {

			ret = process_usecase_cfg_seq_cmd(p, usecase,
						DBMDX_CHIP_VA,
						usecase->va_cfg_values[i]);
			if (ret < 0) {
				dev_err(p->dev,
				"%s: Failed to process VA usecase seq cmd\n",
					__func__);
				ret = -EIO;
				goto out;
			}
		}
	}

out:
	if (p->va_chip_enabled) {
		ret2 = dbmdx_switch_to_va_chip_interface(p,
						DBMDX_CMD_INTERFACE);
		if (ret2) {
			dev_err(p->dev,
				"%s Error switching to (VA) CMD interface\n",
				__func__);
			ret = -EIO;
		}
	} else {
		ret2 = dbmdx_switch_to_va_ve_chip_interface(p,
						DBMDX_CMD_INTERFACE);
		if (ret2) {
			dev_err(p->dev,
				"%s Error switching to (VA_VE) CMD interface\n",
				__func__);
			ret = -EIO;
		}
	}

	return ret;
}

static int dbmdx_va_ve_start_usecase(struct dbmdx_private *p,
					bool force_usecase_start,
					struct usecase_config *usecase)
{
	int ret = 0, ret2 = 0;
	u16 num_of_errors_va_exit, num_of_errors_va;
	u16 num_of_errors_va_ve_exit, num_of_errors_va_ve;
	u32 err_code;

	if (!p->pdata->feature_va_ve) {
		dev_err(p->dev, "%s: VA_VE feature not enabled\n", __func__);
		return -EINVAL;
	}
	if (p->start_usecase_disabled && !force_usecase_start) {
		dev_info(p->dev,
			"%s Usecase start is disabled! Please start manually\n",
			__func__);
		return 0;

	}
	if (usecase == NULL) {
		dev_err(p->dev, "%s: Usecase is NULL\n", __func__);
		return -EINVAL;

	}

	dev_dbg(p->dev, "%s Starting usecase %d (%s)\n", __func__,
		usecase->id, usecase->usecase_name);

	if (p->va_chip_enabled) {
		ret = dbmdx_get_fw_errors(p, DBMDX_CHIP_VA,
					&num_of_errors_va, &err_code);
		if (ret < 0) {
			dev_err(p->dev,
				"%s: Error reading num of VA FW errors\n",
				__func__);
			goto out;
		}
	}
	if (p->va_ve_chip_enabled) {
		ret = dbmdx_get_fw_errors(p, DBMDX_CHIP_VA_VE,
					&num_of_errors_va_ve, &err_code);
		if (ret < 0) {
			dev_err(p->dev,
				"%s: Error reading num of VA_VE FW errors\n",
				__func__);
			goto out;
		}
	}


	p->va_flags.mode = DBMDX_IDLE;
	p->va_ve_flags.mode = DBMDX_IDLE;

	ret = customer_dbmdx_start_usecase_on_init(p, usecase);
	if (ret < 0) {
		dev_err(p->dev, "%s: custom UC_START_ONINIT error\n",
			__func__);
		ret = -EIO;
		goto out;
	}

	ret = dbmdx_restore_gain_normalization(p);
	if (ret < 0) {
		dev_err(p->dev,	"%s: failed restore gain norm.\n", __func__);
		ret = -EIO;
		goto out;
	}

#ifdef VA_VE_I2S_MASTER
	if (p->va_ve_chip_enabled && usecase->send_va_ve_start_cmd) {
		ret = dbmdx_switch_to_va_ve_chip_interface(p,
				DBMDX_CMD_INTERFACE);
		if (ret) {
			dev_err(p->dev,
				"%s Error switching to (VA_VE) CMD interface\n",
				__func__);
			ret = -EIO;
			goto out;
		}

		if (usecase->va_ve_start_cmd_type == START_CMD_TYPE_TDM) {
			ret = dbmdx_send_cmd(p, DBMDX_REGN_TDM_ACTIVATION_CTL |
				usecase->va_ve_start_cmd, NULL);
			p->va_ve_flags.mode =
				(u32)((usecase->va_ve_start_cmd >> 2) & 0x3);
			p->va_ve_flags.tdm_is_reset = false;
			msleep(DBMDX_MSLEEP_TDM_ACTIVATION);

		} else {
			ret = dbmdx_send_cmd(p, DBMDX_REGN_OPERATION_MODE |
				usecase->va_ve_start_cmd, NULL);
			p->va_ve_flags.mode = usecase->va_ve_start_cmd;
			usleep_range(DBMDX_USLEEP_SET_MODE,
					DBMDX_USLEEP_SET_MODE + 1000);

		}
		if (ret < 0) {
			dev_err(p->dev,
			 "%s: failed to config va_ve tdm\n", __func__);
			ret = -EIO;
			goto out;
		}
	}
#endif
	if (p->va_chip_enabled && usecase->send_va_start_cmd) {
		ret = dbmdx_switch_to_va_chip_interface(p, DBMDX_CMD_INTERFACE);
		if (ret) {
			dev_err(p->dev,
				"%s Error switching to (VA) CMD interface\n",
				__func__);
			ret = -EIO;
			goto out;
		}

#ifdef VA_VE_I2S_MASTER
		if (usecase->change_clock_src) {
			ret = configure_usecase_clocks(p, usecase);
			if (ret < 0) {
				dev_err(p->dev,
					"%s: failed to change clk src\n",
					__func__);
				ret = -EIO;
				goto out;
			}
			dev_dbg(p->dev,
				"%s Clock source was successfully updated.\n",
				__func__);
		}
#endif
		if (usecase->va_start_cmd_type == START_CMD_TYPE_TDM) {
			ret = dbmdx_send_cmd(p, DBMDX_REGN_TDM_ACTIVATION_CTL |
				usecase->va_start_cmd, NULL);
			p->va_flags.mode =
				(u32)((usecase->va_start_cmd >> 2) & 0x3);
			p->va_ve_flags.tdm_is_reset = false;
			msleep(DBMDX_MSLEEP_TDM_ACTIVATION);
		} else {
			ret = dbmdx_send_cmd(p, DBMDX_REGN_OPERATION_MODE |
				usecase->va_start_cmd, NULL);
			p->va_flags.mode = usecase->va_start_cmd;
			usleep_range(DBMDX_USLEEP_SET_MODE,
					DBMDX_USLEEP_SET_MODE + 1000);
		}

		if (ret < 0) {
			dev_err(p->dev,
			 "%s: failed to config va tdm\n", __func__);
			ret = -EINVAL;
			goto out;
		}
	}
#ifndef VA_VE_I2S_MASTER
	if (p->va_ve_chip_enabled && usecase->send_va_ve_start_cmd) {
		ret = dbmdx_switch_to_va_ve_chip_interface(p,
				DBMDX_CMD_INTERFACE);
		if (ret) {
			dev_err(p->dev,
				"%s Error switching to (VA_VE) CMD interface\n",
				__func__);
			ret = -EIO;
			goto out;
		}

		if (usecase->va_ve_start_cmd_type == START_CMD_TYPE_TDM) {
			ret = dbmdx_send_cmd(p, DBMDX_REGN_TDM_ACTIVATION_CTL |
				usecase->va_ve_start_cmd, NULL);
			p->va_ve_flags.mode =
				(u32)((usecase->va_ve_start_cmd >> 2) & 0x3);
			p->va_ve_flags.tdm_is_reset = false;
			msleep(DBMDX_MSLEEP_TDM_ACTIVATION);
		} else {
			ret = dbmdx_send_cmd(p, DBMDX_REGN_OPERATION_MODE |
				usecase->va_ve_start_cmd, NULL);
			p->va_ve_flags.mode = usecase->va_ve_start_cmd;
			msleep(DBMDX_MSLEEP_SET_MODE);
		}
		if (ret < 0) {
			dev_err(p->dev,
			 "%s: failed to config va_ve tdm\n", __func__);
			ret = -EIO;
			goto out;
		}
	}
#endif
	ret = customer_dbmdx_start_usecase_on_exit(p, usecase);
	if (ret < 0) {
		dev_err(p->dev, "%s: custom UC_START_ONEXIT error\n",
			__func__);
		ret = -EIO;
		goto out;
	}

	if (p->va_chip_enabled) {
		ret = dbmdx_get_fw_errors(p, DBMDX_CHIP_VA,
					&num_of_errors_va_exit,
					&err_code);
		if (ret < 0) {
			dev_err(p->dev,
				"%s: Error reading num of VA FW errors\n",
				__func__);
			p->unlock(p);
			goto out;
		}
		if (num_of_errors_va_exit != num_of_errors_va) {
			dev_warn(p->dev,
				"%s: OS Error detected on VA CHIP !!!!\n",
				__func__);

			dev_warn(p->dev,
				"%s: OS Errors On Entry:\t%u, On Exit:\t%u, error 0x%08x\n",
				__func__, num_of_errors_va,
				num_of_errors_va_exit, err_code);

		}
	}
	if (p->va_ve_chip_enabled) {
		ret = dbmdx_get_fw_errors(p, DBMDX_CHIP_VA_VE,
					&num_of_errors_va_ve_exit,
					&err_code);
		if (ret < 0) {
			dev_err(p->dev,
				"%s: Error reading num of VA_VE FW errors\n",
				__func__);
			p->unlock(p);
			goto out;
		}
		if (num_of_errors_va_ve_exit != num_of_errors_va_ve) {
			dev_warn(p->dev,
				"%s: OS Error detected on VA_VE CHIP !!!!\n",
				__func__);

			dev_warn(p->dev,
				"%s: OS Errors On Entry:\t%u, On Exit:\t%u, error 0x%08x\n",
				__func__, num_of_errors_va_ve,
				num_of_errors_va_ve_exit, err_code);

		}

	}

	if (usecase->usecase_sets_detection_mode) {
		p->va_ve_flags.usecase_mode = DBMDX_DETECTION;
		p->va_flags.irq_inuse = 1;
	}

	if (p->va_chip_enabled && usecase->va_chip_low_power_mode &&
		p->va_flags.mode == DBMDX_IDLE)
		p->va_flags.sleep_not_allowed = false;

	if (p->va_ve_chip_enabled && usecase->va_ve_chip_low_power_mode &&
		p->va_ve_flags.mode == DBMDX_IDLE)
		p->va_ve_flags.sleep_not_allowed = false;

	if ((p->va_flags.sleep_not_allowed == false) ||
		(p->va_ve_flags.sleep_not_allowed == false))
		/* Try to go to Low Power */
		dbmdx_set_power_mode(p, DBMDX_PM_FALLING_ASLEEP);

out:
	if (p->va_chip_enabled) {
		ret2 = dbmdx_switch_to_va_chip_interface(p,
						DBMDX_CMD_INTERFACE);
		if (ret2) {
			dev_err(p->dev,
				"%s Error switching to (VA) CMD interface\n",
				__func__);
			ret = -EIO;
		}
	} else {
		ret2 = dbmdx_switch_to_va_ve_chip_interface(p,
						DBMDX_CMD_INTERFACE);
		if (ret2) {
			dev_err(p->dev,
				"%s Error switching to (VA_VE) CMD interface\n",
				__func__);
			ret = -EIO;
		}
	}

	return ret;
}

static int dbmdx_va_ve_start_usecase_in_mode(struct dbmdx_private *p,
					struct usecase_config *usecase,
					int mode, bool reconfigure_tdm)
{
	int ret = 0, ret2 = 0;
	int fw_mode_mask = 0xC;
	int new_mode;
	int va_ve_start_cmd = 0;
	int va_start_cmd = 0;
	int va_mode = mode;
	int va_ve_mode = mode;
	u16 cur_mode;
	u16 num_of_errors_va_exit, num_of_errors_va;
	u16 num_of_errors_va_ve_exit, num_of_errors_va_ve;
	u32 err_code;

	if (!p->pdata->feature_va_ve) {
		dev_err(p->dev, "%s: VA_VE feature not enabled\n", __func__);
		return -EINVAL;
	}

	if (usecase == NULL) {
		dev_err(p->dev, "%s: Usecase is NULL\n", __func__);
		return -EINVAL;

	}
	if (!(usecase->send_va_ve_start_cmd) && !(usecase->send_va_start_cmd)) {
		dev_err(p->dev, "%s: Usecase doesn't send start cmd\n",
			__func__);
		return -EINVAL;
	}

	if (p->va_chip_enabled) {
		ret = dbmdx_get_fw_errors(p, DBMDX_CHIP_VA,
					&num_of_errors_va, &err_code);
		if (ret < 0) {
			dev_err(p->dev,
				"%s: Error reading num of VA FW errors\n",
				__func__);
			goto out;
		}
	}
	if (p->va_ve_chip_enabled) {
		ret = dbmdx_get_fw_errors(p, DBMDX_CHIP_VA_VE,
					&num_of_errors_va_ve, &err_code);
		if (ret < 0) {
			dev_err(p->dev,
				"%s: Error reading num of VA_VE FW errors\n",
				__func__);
			goto out;
		}
	}

	if (mode == DBMDX_IDLE) {
		new_mode = 0;
		va_start_cmd = 0;
		va_ve_start_cmd = 0;
		va_mode = DBMDX_IDLE;
		va_ve_mode = DBMDX_IDLE;
	} else if (mode == DBMDX_DETECTION) {
		new_mode = ((0x1 << 2) & fw_mode_mask);
		va_start_cmd = usecase->va_start_cmd;
		va_start_cmd &= ~fw_mode_mask;
		va_start_cmd |= new_mode;
		va_ve_start_cmd = usecase->va_ve_start_cmd;
		va_mode = DBMDX_DETECTION;
		/* Check for Low Power mode */
		if ((usecase->va_ve_start_cmd_type ==  START_CMD_TYPE_OPMODE) &&
			(va_ve_start_cmd == DBMDX_IDLE))
			va_ve_mode = DBMDX_IDLE;
		else
			va_ve_mode = DBMDX_BUFFERING;
	} else if (mode == DBMDX_BUFFERING || mode == DBMDX_STREAMING) {
		if (!(p->pdata->alsa_streaming_options &
				DBMDX_ALSA_STREAMING_DISABLED) &&
			usecase->usecase_supports_i2s_buffering &&
			usecase->va_start_i2s_buffering_cmd) {
			va_start_cmd = usecase->va_start_i2s_buffering_cmd;
		} else {
			new_mode = ((0x2 << 2) & fw_mode_mask);
			va_start_cmd = usecase->va_start_cmd;
			va_start_cmd &= ~fw_mode_mask;
			va_start_cmd |= new_mode;
		}
		va_ve_start_cmd = usecase->va_ve_start_cmd;
		va_mode = DBMDX_BUFFERING;
		/* Check for Low Power mode */
		if ((usecase->va_ve_start_cmd_type ==  START_CMD_TYPE_OPMODE) &&
			(va_ve_start_cmd == DBMDX_IDLE))
			va_ve_mode = DBMDX_IDLE;
		else
			va_ve_mode = DBMDX_BUFFERING;
	} else {
		dev_err(p->dev, "%s: Usecase doesn't support mode %d\n",
			__func__, mode);
		return -EINVAL;
	}

	dev_dbg(p->dev, "%s Starting usecase %d (%s) in mode %d\n", __func__,
		usecase->id, usecase->usecase_name, mode);

	p->va_flags.mode = DBMDX_IDLE;
	p->va_ve_flags.mode = DBMDX_IDLE;

	dbmdx_set_power_mode(p, DBMDX_PM_ACTIVE);

	ret = customer_dbmdx_start_usecase_in_mode_on_init(p, usecase);
	if (ret < 0) {
		dev_err(p->dev, "%s: custom UC_START_IN_MODE_ONINIT error\n",
			__func__);
		ret = -EIO;
		goto out;
	}

	if (p->va_chip_enabled && usecase->send_va_start_cmd) {
		ret = dbmdx_switch_to_va_chip_interface(p, DBMDX_CMD_INTERFACE);
		if (ret) {
			dev_err(p->dev,
				"%s Error switching to (VA) CMD interface\n",
				__func__);
			ret = -EIO;
			goto out;
		}

		if (reconfigure_tdm &&
			(usecase->va_start_cmd_type == START_CMD_TYPE_TDM)) {
			ret = dbmdx_send_cmd(p, (DBMDX_REGN_TDM_ACTIVATION_CTL |
							va_start_cmd), NULL);

			if (ret < 0) {
				dev_err(p->dev,
					"%s: failed to config va tdm\n",
					__func__);
				ret = -EIO;
				goto out;
			}
			p->va_flags.mode = (u32)((va_start_cmd >> 2) & 0x3);
		} else if (!(p->pdata->alsa_streaming_options &
				DBMDX_ALSA_STREAMING_DISABLED) &&
			usecase->usecase_supports_i2s_buffering)	{
			ret = dbmdx_send_cmd(p, (DBMDX_REGN_TDM_ACTIVATION_CTL |
							va_start_cmd), NULL);
		} else {
			ret = dbmdx_send_cmd(p,
					DBMDX_REGN_OPERATION_MODE, &cur_mode);

			if (ret < 0) {
				dev_err(p->dev,
					"%s: failed to read va opmode\n",
					__func__);
				ret = -EIO;
				goto out;
			}

			if (cur_mode != va_mode) {
				ret = dbmdx_send_cmd(p,
						DBMDX_REGN_OPERATION_MODE |
						va_mode, NULL);

				if (ret < 0) {
					dev_err(p->dev,
					"%s: failed to config va opmode\n",
						__func__);
					ret = -EIO;
					goto out;
				}
			}
			p->va_flags.mode = va_mode;
		}
	}

	if (p->va_ve_chip_enabled && usecase->send_va_ve_start_cmd) {
		ret = dbmdx_switch_to_va_ve_chip_interface(p,
				DBMDX_CMD_INTERFACE);
		if (ret) {
			dev_err(p->dev,
				"%s Error switching to (VA_VE) CMD interface\n",
				__func__);
			ret = -EIO;
			goto out;
		}
		if (reconfigure_tdm &&
			(usecase->va_ve_start_cmd_type == START_CMD_TYPE_TDM)) {
			ret = dbmdx_send_cmd(p,
				DBMDX_REGN_TDM_ACTIVATION_CTL |
				va_ve_start_cmd, NULL);

			if (ret < 0) {
				dev_err(p->dev,
					"%s: failed to config va_ve tdm\n",
					__func__);
				ret = -EIO;
				goto out;
			}
			p->va_ve_flags.mode =
					(u32)((va_ve_start_cmd >> 2) & 0x3);
		} else {
			ret = dbmdx_send_cmd(p,
					DBMDX_REGN_OPERATION_MODE, &cur_mode);

			if (ret < 0) {
				dev_err(p->dev,
					"%s: failed to read va_ve opmode\n",
					__func__);
				ret = -EIO;
				goto out;
			}
			if (cur_mode != va_ve_mode) {
				ret = dbmdx_send_cmd(p,
					DBMDX_REGN_OPERATION_MODE | va_ve_mode,
					NULL);

				if (ret < 0) {
					dev_err(p->dev,
					"%s: failed to config va_ve opmode\n",
						__func__);
					ret = -EIO;
					goto out;
				}
			}
			p->va_ve_flags.mode = va_ve_mode;
		}
	}

	ret = customer_dbmdx_start_usecase_in_mode_on_exit(p, usecase);
	if (ret < 0) {
		dev_err(p->dev, "%s: custom UC_START_IN_MODE_ONEXIT error\n",
			__func__);
		ret = -EIO;
		goto out;
	}

	if (p->va_chip_enabled) {
		ret = dbmdx_get_fw_errors(p, DBMDX_CHIP_VA,
					&num_of_errors_va_exit,
					&err_code);
		if (ret < 0) {
			dev_err(p->dev,
				"%s: Error reading num of VA FW errors\n",
				__func__);
			p->unlock(p);
			goto out;
		}
		if (num_of_errors_va_exit != num_of_errors_va) {
			dev_warn(p->dev,
				"%s: OS Error detected on VA CHIP !!!!\n",
				__func__);

			dev_warn(p->dev,
				"%s: OS Errors On Entry:\t%u, On Exit:\t%u, error 0x%08x\n",
				__func__, num_of_errors_va,
				num_of_errors_va_exit, err_code);

		}
	}
	if (p->va_ve_chip_enabled) {
		ret = dbmdx_get_fw_errors(p, DBMDX_CHIP_VA_VE,
					&num_of_errors_va_ve_exit,
					&err_code);
		if (ret < 0) {
			dev_err(p->dev,
				"%s: Error reading num of VA_VE FW errors\n",
				__func__);
			p->unlock(p);
			goto out;
		}
		if (num_of_errors_va_ve_exit != num_of_errors_va_ve) {
			dev_warn(p->dev,
				"%s: OS Error detected on VA_VE CHIP !!!!\n",
				__func__);

			dev_warn(p->dev,
				"%s: OS Errors On Entry:\t%u, On Exit:\t%u, error 0x%08x\n",
				__func__, num_of_errors_va_ve,
				num_of_errors_va_ve_exit, err_code);

		}

	}

	if (mode == DBMDX_DETECTION)
		p->va_flags.irq_inuse = 1;

	p->va_ve_flags.usecase_mode = mode;

	p->va_flags.sleep_not_allowed = true;
	p->va_ve_flags.sleep_not_allowed = true;

	if (p->va_chip_enabled && usecase->va_chip_low_power_mode &&
		p->va_flags.mode == DBMDX_IDLE)
		p->va_flags.sleep_not_allowed = false;

	if (p->va_ve_chip_enabled && usecase->va_ve_chip_low_power_mode &&
		p->va_ve_flags.mode == DBMDX_IDLE &&
		p->va_flags.mode != DBMDX_BUFFERING &&
		p->va_flags.mode != DBMDX_STREAMING)
		p->va_ve_flags.sleep_not_allowed = false;

	if ((p->va_flags.sleep_not_allowed == false) ||
		(p->va_ve_flags.sleep_not_allowed == false))
		/* Try to go to Low Power */
		dbmdx_set_power_mode(p, DBMDX_PM_FALLING_ASLEEP);

out:
	if (p->va_chip_enabled) {
		ret2 = dbmdx_switch_to_va_chip_interface(p,
						DBMDX_CMD_INTERFACE);
		if (ret2) {
			dev_err(p->dev,
				"%s Error switching to (VA) CMD interface\n",
				__func__);
			ret = -EIO;
		}
	} else {
		ret2 = dbmdx_switch_to_va_ve_chip_interface(p,
						DBMDX_CMD_INTERFACE);
		if (ret2) {
			dev_err(p->dev,
				"%s Error switching to (VA_VE) CMD interface\n",
				__func__);
			ret = -EIO;
		}
	}

	return ret;
}

int dbmdx_va_usecase_update(struct dbmdx_private *p,
		struct usecase_config *usecase)
{
	int ret = 0;
	int ret2 = 0;

	/* flush pending buffering works if any */
	p->va_flags.buffering = 0;
	flush_work(&p->sv_work);

	/* IDLE Usecase */
	if (usecase == NULL) {

		if (p->va_ve_flags.idle_usecase_active) {
			dev_info(p->dev,
				"%s IDLE usecase has been already configured\n",
				__func__);
			return 0;
		}

		p->lock(p);

		dbmdx_set_power_mode(p, DBMDX_PM_ACTIVE);

		ret = dbmdx_va_ve_send_idle_usecase_config(p, true);

		p->unlock(p);

		if (ret < 0) {
			dev_err(p->dev,
				"%s: Error Sending Usecase\n", __func__);
		}

		p->va_ve_flags.active_usecase_id = 0;
		p->va_ve_flags.cur_usecase = NULL;

		goto out;

	}

	p->lock(p);

	dbmdx_set_power_mode(p, DBMDX_PM_ACTIVE);

	ret = dbmdx_va_ve_send_idle_usecase_config(p, false);

	p->va_ve_flags.idle_usecase_active = 0;

	p->unlock(p);

	if (ret < 0) {
		dev_err(p->dev,	"%s: Error Sending IDLE Usecase\n", __func__);
		goto out;
	}

	p->lock(p);

	ret = dbmdx_va_ve_send_usecase_config(p, usecase);

	p->unlock(p);

	if (ret < 0) {
		dev_err(p->dev,	"%s: Error Sending Usecase %d\n", __func__,
			usecase->id);
		goto out;
	}

	p->lock(p);

	ret = dbmdx_va_ve_start_usecase(p, false, usecase);

	p->unlock(p);

	if (ret < 0) {
		dev_err(p->dev,	"%s: Error Starting Usecase %d\n", __func__,
			usecase->id);
		goto out;
	}

	p->va_ve_flags.active_usecase_id = usecase->id;
	p->va_ve_flags.cur_usecase = usecase;


out:
	if (p->va_chip_enabled) {
		ret2 = dbmdx_switch_to_va_chip_interface(p,
						DBMDX_CMD_INTERFACE);
		if (ret2) {
			dev_err(p->dev,
				"%s Error switching to (VA) CMD interface\n",
				__func__);
			ret = -EIO;
		}
	} else {
		ret2 = dbmdx_switch_to_va_ve_chip_interface(p,
						DBMDX_CMD_INTERFACE);
		if (ret2) {
			dev_err(p->dev,
				"%s Error switching to (VA_VE) CMD interface\n",
				__func__);
			ret = -EIO;
		}
	}


	return ret;
}

int dbmdx_set_usecase_mode(struct dbmdx_private *p, int mode)
{
	int ret = 0;
	unsigned int cur_opmode = p->va_ve_flags.usecase_mode;
	bool reconfigure_tdm = false;
	int usecase_mode = (mode & DBMDX_USECASE_MODE_MASK);
	bool buffering_with_backlog = false;
	struct usecase_config *cur_usecase = p->va_ve_flags.cur_usecase;

	if (mode == DBMDX_DEFAULT_USECASE_MODE) {
		dev_dbg(p->dev, "%s: default usecase mode was requested\n",
			__func__);
	} else if (usecase_mode == DBMDX_IDLE ||
		usecase_mode == DBMDX_DETECTION ||
		usecase_mode == DBMDX_BUFFERING ||
		usecase_mode == DBMDX_STREAMING) {
		dev_dbg(p->dev, "%s: new requested usecase mode: %d (%s)\n",
			__func__, mode, dbmdx_state_names[usecase_mode]);
	} else {
		dev_dbg(p->dev, "%s: Unsupported usecase mode: %d\n",
			__func__, usecase_mode);
		return -EINVAL;
	}

	if (cur_usecase == NULL) {
		dev_err(p->dev, "%s: No active usecase\n", __func__);
		return -EINVAL;
	}

	if (mode == DBMDX_DEFAULT_USECASE_MODE) {

		ret = dbmdx_va_ve_start_usecase(p, true, cur_usecase);
		if (ret < 0) {
			dev_err(p->dev,
				"%s: Error Starting Usecase %d\n",
				__func__, cur_usecase->id);
			ret = -EIO;
		}

		goto out;
	}

	if ((p->va_ve_flags.usecase_mode == DBMDX_IDLE) ||
		(usecase_mode == DBMDX_IDLE))
		reconfigure_tdm = true;

	p->va_flags.buffering = 0;

	p->va_flags.irq_inuse = 0;
	if (usecase_mode == DBMDX_IDLE) {
		/* Stop PCM streaming work */
		p->va_flags.pcm_worker_active = 0;

		ret = dbmdx_va_ve_start_usecase_in_mode(p, cur_usecase,
						usecase_mode, reconfigure_tdm);
		if (ret) {
			dev_err(p->dev,
				"%s Error setting IDLE usecase mode\n",
				__func__);
			ret = -EIO;
			goto out;
		}

		p->va_ve_flags.usecase_mode = usecase_mode;
	} else if (usecase_mode == DBMDX_BUFFERING) {
		/* Stop PCM streaming work */
		p->va_flags.pcm_worker_active = 0;
		if (mode & DBMDX_BUFFERING_WITH_BACKLOG_MASK)
			buffering_with_backlog = true;

		/* Reset backlog position to 0
		 * if buffering with backlog flag is not set
		 */
		if (!buffering_with_backlog) {
			p->va_cur_backlog_length = 0;

			ret = dbmdx_switch_to_va_chip_interface(p,
							DBMDX_CMD_INTERFACE);

			if (ret) {
				dev_err(p->dev,
				"%s Error switching to (VA) CMD interface\n",
					__func__);
				ret = -EIO;
				goto out;
			}

			ret = dbmdx_send_cmd(p,
					DBMDX_REGN_VT_AUDIO_HISTORY_CFG,
					NULL);

			if (ret < 0) {
				dev_err(p->dev,
						"%s: failed to reset backlog pos.\n",
						__func__);
				ret = -EIO;
				goto out;
			}
		}

		if (p->va_flags.number_of_vt_since_start < 2) {
			ret = dbmdx_disable_gain_normalization(p);

			if (ret < 0) {
				dev_err(p->dev,
					"%s: failed disable gain norm.\n",
						__func__);
				ret = -EIO;
				goto out;
			}
		}

		ret = dbmdx_va_ve_start_usecase_in_mode(p, cur_usecase,
							DBMDX_BUFFERING,
							reconfigure_tdm);
		if (ret) {
			dev_err(p->dev,
				"%s Error setting BUFFERING usecase mode\n",
				__func__);
			ret = -EIO;
			goto out;
		}

		/* Do not call sv_work in case of i2s_buffering */
		if ((cur_usecase->usecase_supports_us_buffering) &&
			((p->pdata->alsa_streaming_options &
				DBMDX_ALSA_STREAMING_DISABLED) ||
			!(cur_usecase->usecase_supports_i2s_buffering))) {
			p->va_flags.buffering_paused = 0;
			p->va_flags.buffering = 1;
			dbmdx_schedule_work(p, &p->sv_work);
		}

	} else if (usecase_mode == DBMDX_STREAMING) {
		int channels;
		int usecase_channels;

		channels = p->audio_pcm_channels;
		usecase_channels = cur_usecase->num_of_output_channels;

		/* Stop PCM streaming work */
		p->va_flags.pcm_worker_active = 0;

		if (usecase_channels < 1)
			usecase_channels = 1;
		if (channels == usecase_channels)
			p->pcm_achannel_op = AUDIO_CHANNEL_OP_COPY;
		else {
			if (channels == 1 && usecase_channels == 2)
				p->pcm_achannel_op =
					AUDIO_CHANNEL_OP_TRUNCATE_2_TO_1;
			else if (channels == 2 && usecase_channels == 1)
				p->pcm_achannel_op =
					AUDIO_CHANNEL_OP_DUPLICATE_1_TO_2;
			else {
				dev_err(p->dev,
				"%s: DAI channels %d not matching usecase channels %d\n",
				__func__, channels, usecase_channels);
				ret = -EINVAL;
				goto out;
			}
		}

		p->pdata->va_audio_channels = usecase_channels;

		if (mode & DBMDX_BUFFERING_WITH_BACKLOG_MASK)
			buffering_with_backlog = true;

		/* Reset backlog position to 0
		 * if buffering with backlog flag is not set
		 */
		if (!buffering_with_backlog) {
			p->va_cur_backlog_length = 0;

			ret = dbmdx_switch_to_va_chip_interface(p,
							DBMDX_CMD_INTERFACE);

			if (ret) {
				dev_err(p->dev,
				"%s Error switching to (VA) CMD interface\n",
					__func__);
				ret = -EIO;
				goto out;
			}

			ret = dbmdx_send_cmd(p,
					DBMDX_REGN_VT_AUDIO_HISTORY_CFG,
					NULL);

			if (ret < 0) {
				dev_err(p->dev,
						"%s: failed to reset backlog pos.\n",
						__func__);
				ret = -EIO;
				goto out;
			}
		}

		if (p->va_flags.number_of_vt_since_start < 3) {
			ret = dbmdx_disable_gain_normalization(p);

			if (ret < 0) {
				dev_err(p->dev,
					"%s: failed disable gain norm.\n",
						__func__);
				ret = -EIO;
				goto out;
			}
		}

		ret = dbmdx_va_ve_start_usecase_in_mode(p, cur_usecase,
							DBMDX_STREAMING,
							reconfigure_tdm);
		if (ret) {
			dev_err(p->dev,
				"%s Error setting BUFFERING usecase mode\n",
				__func__);
			ret = -EIO;
			goto out;
		}


		/* Do not call sv_work in case of i2s_buffering */
		if ((cur_usecase->usecase_supports_us_buffering) &&
			((p->pdata->alsa_streaming_options &
				DBMDX_ALSA_STREAMING_DISABLED) ||
			!(cur_usecase->usecase_supports_i2s_buffering))) {
			p->va_flags.pcm_worker_active = 1;
			dbmdx_schedule_work(p, &p->pcm_streaming_work);
		}

	} else if (usecase_mode == DBMDX_DETECTION) {
		if (!cur_usecase->usecase_sets_detection_mode) {
			dev_err(p->dev,
				"%s Usecase doesn't support detection mode\n",
				__func__);
			ret = -EIO;
			goto out;
		}

		/* Restore backlog position */
		ret = dbmdx_switch_to_va_chip_interface(p,
							DBMDX_CMD_INTERFACE);
		if (ret) {
			dev_err(p->dev,
				"%s Error switching to (VA) CMD interface\n",
				__func__);
			ret = -EIO;
			goto out;
		}
#ifdef RR_TOFIX
		/* restore models backlog */
#endif
		ret = dbmdx_restore_gain_normalization(p);

		if (ret < 0) {
			dev_err(p->dev,
				"%s: failed restore gain norm.\n",
								__func__);
			ret = -EIO;
			goto out;
		}

		ret = dbmdx_va_ve_start_usecase_in_mode(p, cur_usecase,
							DBMDX_DETECTION,
							reconfigure_tdm);
		if (ret) {
			dev_err(p->dev,
				"%s Error setting Detection usecase mode\n",
				__func__);
			ret = -EIO;
			goto out;
		}
		p->va_flags.irq_inuse = 1;
		p->va_flags.mode = DBMDX_DETECTION;

	}

	p->va_ve_flags.usecase_mode = usecase_mode;


	ret = 0;

	dev_dbg(p->dev,
		"%s: Successful transition from usecase mode %d ==> %d\n",
		__func__, cur_opmode, p->va_ve_flags.usecase_mode);
out:
	return ret;
}


int dbmdx_restart_current_usecase(struct dbmdx_private *p)
{
	int ret = 0;
	struct usecase_config *cur_usecase;

	if (!p)
		return -EAGAIN;

	if (!p->device_ready) {
		dev_err(p->dev, "%s: device not ready\n", __func__);
		return -EAGAIN;
	}

	cur_usecase = p->va_ve_flags.cur_usecase;

	if (p->va_ve_flags.idle_usecase_active || !cur_usecase) {
		dev_info(p->dev, "%s No Active Usecase\n", __func__);
		return 0;
	}

	ret = dbmdx_va_usecase_update(p, cur_usecase);
	if (ret < 0) {
		dev_err(p->dev,	"%s Error while restarting current Usecase\n",
				__func__);
		return ret;
	}

	return ret;
}





static int usecase_manager_load_amodel(struct dbmdx_private *p,
					unsigned int cmd,
					bool *uc_reload_required)
{
	int ret = 0;
	unsigned int usecase_command_mask = 0;

	unsigned int model_operation = 0;
	unsigned int model_type = 0;
	unsigned int model_mode = 0;
	unsigned int model_options = 0;
	unsigned int model_custom_params = 0;
	unsigned int model_prefix_index = 0;
	struct vt_engine *sel_vt_eng = NULL;
	enum dbmdx_vt_engine_type type;


	usecase_command_mask =
		(((unsigned int)cmd & DBMDX_USECASE_MGR_CMD_MASK) >>
						DBMDX_USECASE_MGR_CMD_BITS);

	if (!(usecase_command_mask & DBMDX_USECASE_MGR_CMD_LOAD_AMODEL)) {

		dev_err(p->dev,	"%s: The UC command is not Load AModel\n",
				__func__);
		return -EINVAL;
	}

	model_operation =
		(((unsigned int)cmd &
			DBMDX_USECASE_MGR_LM_MODEL_OPERATION_MASK) >>
				DBMDX_USECASE_MGR_LM_MODEL_OPERATION_BITS);
	model_type =
		(((unsigned int)cmd & DBMDX_USECASE_MGR_LM_MODEL_TYPE_MASK) >>
					DBMDX_USECASE_MGR_LM_MODEL_TYPE_BITS);
	model_mode =
		(((unsigned int)cmd & DBMDX_USECASE_MGR_LM_MODEL_MODE_MASK) >>
					DBMDX_USECASE_MGR_LM_MODEL_MODE_BITS);
	model_options =
		(((unsigned int)cmd &
			DBMDX_USECASE_MGR_LM_MODEL_OPTIONS_MASK) >>
				DBMDX_USECASE_MGR_LM_MODEL_OPTIONS_BITS);
	model_custom_params =
		(((unsigned int)cmd &
			DBMDX_USECASE_MGR_LM_MODEL_CUSTOM_PARAMS_MASK) >>
				DBMDX_USECASE_MGR_LM_MODEL_CUSTOM_PARAMS_BITS);
	model_prefix_index =
		(((unsigned int)cmd &
			DBMDX_USECASE_MGR_LM_MODEL_PREFIX_MASK) >>
				DBMDX_USECASE_MGR_LM_MODEL_PREFIX_BITS);

	if (model_type == DBMDX_VT_ENGINE_TYPE_UNDEFINED ||
		model_type >= DBMDX_VT_ENGINE_TYPE_MAX) {
		dev_err(p->dev, "%s: Invalid engine type %d\n", __func__,
			(int)model_type);
		return -EINVAL;
	}

	type = (enum dbmdx_vt_engine_type)model_type;

	ret = dbmdx_get_vte_from_model_type(p, type, &sel_vt_eng);

	if (ret < 0) {
		dev_err(p->dev,	"%s: Model %d is not supported\n",
				__func__, type);
		return -EINVAL;
	}

	if (!(sel_vt_eng->enabled)) {
		dev_err(p->dev,	"%s: Model %s is disabled\n",
				__func__,
				dbmdx_vte_names[sel_vt_eng->eng_type]);
		return -EINVAL;

	}

	dev_dbg(p->dev,
		"%s:Model: %s\tDet.mode: 0x%x\tOptions: 0x%x\tParams 0x%x\tPrefix 0x%x\n",
		__func__, dbmdx_vte_names[sel_vt_eng->eng_type],
		model_mode, model_options, model_custom_params,
		model_prefix_index);

	*uc_reload_required = false;

	switch (model_operation) {
	case DBMDX_USECASE_MGR_LM_OP_STOP_DETECTION:
		ret = dbmdx_va_disable_vte_detection(p, sel_vt_eng,
			(enum dbmdx_model_detection_type)model_mode);
		if (ret < 0) {
			dev_err(p->dev,	"%s: Error stopping vte detection\n",
				__func__);

			return -EIO;
		}
		dev_dbg(p->dev,	"%s: Detection stopped for: %s\n", __func__,
				dbmdx_vte_names[sel_vt_eng->eng_type]);
		break;
	case DBMDX_USECASE_MGR_LM_OP_START_DETECTION:
		if (model_mode >= DETECTION_TYPE_MAX) {
			dev_err(p->dev,	"%s: Invalid Detection Type: %u\n",
				__func__, model_type);
			return -EINVAL;
		}
		ret = dbmdx_va_enable_vte_detection(p,
				sel_vt_eng,
				(enum dbmdx_model_detection_type)model_mode,
				model_custom_params,
				uc_reload_required);
		if (ret < 0) {
			dev_err(p->dev,	"%s: Error starting vte detection\n",
				__func__);

			return -EIO;
		}
		if (*uc_reload_required)
			dev_dbg(p->dev,
				"%s: Detection request registered, UC should restart for: %s\n",
				__func__,
				dbmdx_vte_names[sel_vt_eng->eng_type]);
		else
			dev_dbg(p->dev,	"%s: Detection started for: %s\n",
				__func__,
				dbmdx_vte_names[sel_vt_eng->eng_type]);

		break;
	case DBMDX_USECASE_MGR_LM_OP_LOAD_MODEL:
		model_options |= DBMDX_DO_NOT_SEND_TO_FW;
		if (model_mode >= DETECTION_TYPE_MAX) {
			dev_err(p->dev,	"%s: Invalid Detection Type: %u\n",
				__func__, model_type);
			return -EINVAL;
		}
		ret = dbmdx_va_load_amodel(p, sel_vt_eng,
				(enum dbmdx_model_detection_type)model_mode,
				model_options, model_prefix_index);
		if (ret < 0) {
			dev_err(p->dev,	"%s: Error loading model for %s\n",
				__func__,
				dbmdx_vte_names[sel_vt_eng->eng_type]);

			return -EIO;
		}
		if (sel_vt_eng->user_state != MODEL_STATE_STARTED)
			sel_vt_eng->user_state = MODEL_STATE_LOADED;
		dev_dbg(p->dev,	"%s: Model Loaded for: %s\n", __func__,
				dbmdx_vte_names[sel_vt_eng->eng_type]);

		break;
	case DBMDX_USECASE_MGR_LM_OP_UNLOAD_MODEL:
		if (p->va_ve_flags.idle_usecase_active) {
			if (model_mode >= DETECTION_TYPE_MAX) {
				dev_err(p->dev,
					"%s: Invalid Detection Type: %u\n",
					__func__,
					model_mode);
				return -EINVAL;
			}
			ret = dbmdx_va_unload_amodel(p, sel_vt_eng,
				(enum dbmdx_model_detection_type)model_mode);
			if (ret < 0) {
				dev_err(p->dev,	"%s: Error unloading model\n",
					__func__);

				sel_vt_eng->user_state = MODEL_STATE_UNLOADED;
				return -EIO;
			}
			sel_vt_eng->user_state = MODEL_STATE_UNLOADED;
			dev_dbg(p->dev,	"%s: Model Unloaded for: %s\n",
				__func__,
				dbmdx_vte_names[sel_vt_eng->eng_type]);

		} else {
			sel_vt_eng->user_state = MODEL_STATE_UNLOADED;
			dev_dbg(p->dev,
				"%s: Model was marked as Unloaded for: %s\n",
				__func__,
				dbmdx_vte_names[sel_vt_eng->eng_type]);

		}
		break;
	case DBMDX_USECASE_MGR_LM_OP_UPDATE_PARAMS:
		sel_vt_eng->custom_params = model_custom_params;
		dev_dbg(p->dev,	"%s: Model Params updated for: %s\n",
				__func__,
				dbmdx_vte_names[sel_vt_eng->eng_type]);
		break;
	default:
		dev_err(p->dev,	"%s: Unknown UCM_LM command: 0x%x\n",
				__func__, model_operation);
		return -EIO;
	}

	return 0;
}


static int dbmdx_usecase_manager(struct dbmdx_private *p, unsigned int cmd)
{
	int ret = 0;
	unsigned int usecase_command_mask = 0;
	unsigned int usecase_mode = 0;
	unsigned int usecase_id = 0;

	struct usecase_config *cur_usecase = p->va_ve_flags.cur_usecase;
	bool uc_reload_required = false;

	usecase_command_mask =
		(((unsigned int)cmd & DBMDX_USECASE_MGR_CMD_MASK) >>
						DBMDX_USECASE_MGR_CMD_BITS);

	/* flush pending buffering works if any */
	p->va_flags.buffering = 0;
	flush_work(&p->sv_work);

	if (usecase_command_mask == DBMDX_USECASE_MGR_CMD_SET_IDLE) {
		ret = dbmdx_va_set_usecase_id(p, 0);
		if (ret < 0) {
			dev_err(p->dev, "%s: Error configuring IDLE usecase\n",
				__func__);
			ret = -EIO;
			goto out;
		}
	}

	if (usecase_command_mask & DBMDX_USECASE_MGR_CMD_LOAD_AMODEL) {

		p->lock(p);

		dbmdx_set_power_mode(p, DBMDX_PM_ACTIVE);

		ret = usecase_manager_load_amodel(p, cmd, &uc_reload_required);
		if (ret < 0) {
			dev_err(p->dev,
				"%s: Error processing UCM Load Amodel cmd\n",
				__func__);
			ret = -EIO;
			dbmdx_set_power_mode(p, DBMDX_PM_FALLING_ASLEEP);
			p->unlock(p);
			goto out;
		}
		if (!uc_reload_required) {
			ret = 0;
			dbmdx_set_power_mode(p, DBMDX_PM_FALLING_ASLEEP);
			p->unlock(p);
			goto out;
		}

		p->unlock(p);

		/* Usecase should be restarted */
		ret = dbmdx_restart_current_usecase(p);


		if (ret < 0) {
			dev_err(p->dev, "%s: Error restarting current UC\n",
				__func__);
			ret = -EIO;
			goto out;
		}

		p->lock(p);

		dbmdx_set_power_mode(p, DBMDX_PM_FALLING_ASLEEP);

		p->unlock(p);

	}

	usecase_mode =
		(((unsigned int)cmd & DBMDX_USECASE_MGR_MODE_MASK) >>
						DBMDX_USECASE_MGR_MODE_BITS);
	usecase_id =
		(((unsigned int)cmd & DBMDX_USECASE_MGR_UID_MASK) >>
						DBMDX_USECASE_MGR_UID_BITS);

	dev_dbg(p->dev,
		"%s:Usecase: CmdMask 0x%x\tMode: 0x%x\tID: 0x%x\n",
		__func__, usecase_command_mask, usecase_mode, usecase_id);

	if (usecase_command_mask & DBMDX_USECASE_MGR_CMD_LOAD) {
		bool cur_start_usecase_disabled = p->start_usecase_disabled;

		if ((p->va_ve_flags.active_usecase_id != usecase_id) ||
			(p->va_ve_flags.usecase_mode == DBMDX_IDLE)) {
			p->start_usecase_disabled = true;

			ret = dbmdx_va_set_usecase_id(p, usecase_id);

			p->start_usecase_disabled = cur_start_usecase_disabled;

			if (ret < 0) {
				dev_err(p->dev,
					"%s: Error loading usecase 0x%x\n",
					__func__, usecase_id);
				ret = -EIO;
				goto out;
			}

		}

	}

	if (usecase_command_mask & DBMDX_USECASE_MGR_CMD_START) {
		if (p->va_ve_flags.active_usecase_id != usecase_id) {
			dev_err(p->dev, "%s: Usecase 0x%x was not loaded\n",
				__func__, usecase_id);
			ret = -EINVAL;
			goto out;
		}

		cur_usecase = p->va_ve_flags.cur_usecase;

		p->lock(p);

		ret = dbmdx_mute_current_usecase(p, false);

		if (ret < 0) {
			dev_err(p->dev,	"%s: Error Unmuting current usecase\n",
						__func__);
			ret = -EIO;
			p->unlock(p);
			goto out;
		}
		if (p->va_ve_flags.usecase_mode == DBMDX_IDLE)
			ret = dbmdx_va_ve_start_usecase(p, true, cur_usecase);

		p->unlock(p);

		if (ret < 0) {
			dev_err(p->dev, "%s: Error starting usecase 0x%x\n",
				__func__, usecase_id);
			ret = -EIO;
			goto out;
		}
	}

	if (usecase_command_mask & DBMDX_USECASE_MGR_CMD_SET_MODE) {
		if ((p->va_ve_flags.active_usecase_id != usecase_id) &&
			(usecase_id != 0)) {
			dev_err(p->dev, "%s: Usecase 0x%x was not loaded\n",
				__func__, usecase_id);
			ret = -EINVAL;
			goto out;
		}
		if (p->va_ve_flags.cur_usecase == NULL)  {
			if ((usecase_mode == DBMDX_BUFFERING) &&
				p->pdata->default_streaming_usecase &&
				(p->pdata->alsa_streaming_options &
				DBMDX_ALSA_STREAMING_SET_DEFAULT_USECASE)) {

				ret = dbmdx_set_va_usecase_name(p,
					p->pdata->default_streaming_usecase);
				if (ret) {
					dev_err(p->dev,
						"%s: Failed to configure def. streaming usecase\n",
						__func__);
					ret = -EIO;
					goto out;
				}
			} else {
				dev_err(p->dev,	"%s: No Active Usecase\n",
						__func__);
				ret = -EINVAL;
				goto out;
			}
		}

		dev_info(p->dev,
			"%s: Required mode: (%x), Current mode: (%x)\n",
			__func__, (int)usecase_mode,
			p->va_ve_flags.usecase_mode);

		/* flush pending buffering work if any */
		p->va_flags.buffering = 0;
		flush_work(&p->sv_work);

		p->lock(p);

		dbmdx_set_power_mode(p, DBMDX_PM_ACTIVE);

		ret = dbmdx_mute_current_usecase(p, false);

		if (ret < 0) {
			dev_err(p->dev,	"%s: Error Unmuting current usecase\n",
						__func__);
			ret = -EIO;
			p->unlock(p);
			goto out;
		}

		ret = dbmdx_set_usecase_mode(p, usecase_mode);

		p->unlock(p);

		if (ret < 0) {
			dev_err(p->dev,
				"%s: Error setting mode for usecase 0x%x\n",
				__func__, usecase_id);
			ret = -EIO;
			goto out;
		}
	}
out:
	return ret;
}

static int dbmdx_config_va_mode(struct dbmdx_private *p)
{
	unsigned int i;
	int ret;
	u16 fwver = 0xffff;
	u16 cur_reg;
	u16 cur_val;

	dev_dbg(p->dev, "%s\n", __func__);

	ret = dbmdx_send_cmd(p, DBMDX_REGN_CHIP_ID_NUMBER, &cur_val);
	if (ret < 0) {
		dev_err(p->dev, "%s: could not read Firmware ID\n",
		__func__);
		ret = -EIO;
		goto out;
	}

	dev_info(p->dev, "%s: Reported FW ID is: 0x%x\n", __func__, cur_val);

	usleep_range(DBMDX_USLEEP_BEFORE_INIT_CONFIG,
		DBMDX_USLEEP_BEFORE_INIT_CONFIG + 1000);

	ret = customer_dbmdx_va_pre_init(p);
	if (ret < 0) {
		dev_err(p->dev, "%s: custom VA_PREINIT error\n",
			__func__);
		ret = -EIO;
		goto out;
	}

	for (i = 0; i < p->pdata->va_cfg_values; i++) {

		cur_reg = (u16)((p->pdata->va_cfg_value[i] >> 16) & 0x0fff);
		cur_val = (u16)((p->pdata->va_cfg_value[i]) & 0xffff);

		if (cur_reg == 0)
			continue;
		else if (cur_reg == DBMDX_VA_USLEEP_FLAG) {
			usleep_range(cur_val, cur_val + 100);
			continue;
		} else if  (cur_reg == DBMDX_VA_MSLEEP_FLAG) {
			msleep(cur_val);
			continue;
		}

		if ((p->va_debug_mode != DBMDX_DEBUG_MODE_OFF) &&
			(p->active_interface != DBMDX_INTERFACE_UART) &&
			(cur_reg ==
		(u16)((DBMDX_REGN_HOST_INTERFACE_SUPPORT >> 16) & 0xff))) {
			ret = dbmdx_send_cmd(p,
				(p->pdata->va_cfg_value[i] | 0x1000), NULL);
			if (ret < 0) {
				dev_err(p->dev,
					"%s: failed to send cmd\n", __func__);
				ret = -EIO;
				goto out;
			}

			if (!(p->va_debug_mode & DBMDX_DEBUG_MODE_FW_LOG)) {
				ret = dbmdx_send_cmd(p,
					(DBMDX_REGN_FW_DEBUG_REGISTER |
					DBMDX_REGV_TOGGLE_UART_DEBUG_PRINTS),
					NULL);
				if (ret < 0) {
					dev_err(p->dev,
						"%s: failed to send cmd\n",
						__func__);
					ret = -EIO;
					goto out;
				}
			}

			continue;
		}

		ret = dbmdx_send_cmd(p, p->pdata->va_cfg_value[i], NULL);
		if (ret < 0) {
			dev_err(p->dev, "%s: failed to send cmd\n", __func__);
			ret = -EIO;
			goto out;
		}
	}

	/* Give to PLL enough time for stabilization */
	msleep(DBMDX_MSLEEP_CONFIG_VA_MODE_REG);

	p->chip->transport_enable(p, true);

	/* read firmware version */
	ret = dbmdx_send_cmd(p, DBMDX_REGN_FW_VERSION_NUMBER, &fwver);
	if (ret < 0) {
		dev_err(p->dev, "%s: could not read firmware version\n",
			__func__);
		goto out;
	}
	p->va_flags.microphones_enabled = true;
	p->pdata->va_audio_channels = 1;

	ret = dbmdx_read_fw_vad_settings(p);

	if (ret < 0) {
		dev_err(p->dev, "%s: failed to read fw vad settings\n",
			__func__);
		goto out;
	}
	ret = dbmdx_verify_model_support(p);
	if (ret < 0) {
		dev_err(p->dev, "%s: failed to verify amodel support\n",
			__func__);
		goto out;
	}

	ret = customer_dbmdx_va_post_init(p);
	if (ret < 0) {
		dev_err(p->dev, "%s: custom VA_POSTINIT error\n",
			__func__);
		goto out;
	}

	dev_info(p->dev, "%s: VA firmware 0x%x ready\n", __func__, fwver);

	ret = 0;
out:
	return ret;
}

static int dbmdx_config_va_ve_mode(struct dbmdx_private *p)
{
	unsigned int i;
	int ret;
	u16 fwver = 0xffff;
	u16 cur_reg;
	u16 cur_val;

	dev_dbg(p->dev, "%s\n", __func__);

	ret = dbmdx_send_cmd(p, DBMDX_REGN_CHIP_ID_NUMBER, &cur_val);
	if (ret < 0) {
		dev_err(p->dev, "%s: could not read Firmware ID\n",
		__func__);
		ret = -EIO;
		goto out;
	}

	dev_info(p->dev, "%s: Reported FW ID is: 0x%x\n", __func__, cur_val);

	usleep_range(DBMDX_USLEEP_BEFORE_INIT_CONFIG,
		DBMDX_USLEEP_BEFORE_INIT_CONFIG + 1000);

	ret = customer_dbmdx_va_ve_pre_init(p);
	if (ret < 0) {
		dev_err(p->dev, "%s: custom VA_VE_PREINIT error\n",
			__func__);
		ret = -EIO;
		goto out;
	}

	for (i = 0; i < p->pdata->va_ve_cfg_values; i++) {

		cur_reg = (u16)((p->pdata->va_ve_cfg_value[i] >> 16) & 0x0fff);
		cur_val = (u16)((p->pdata->va_ve_cfg_value[i]) & 0xffff);

		if (cur_reg == 0)
			continue;
		else if (cur_reg == DBMDX_VA_USLEEP_FLAG) {
			usleep_range(cur_val, cur_val + 100);
			continue;
		} else if  (cur_reg == DBMDX_VA_MSLEEP_FLAG) {
			msleep(cur_val);
			continue;
		}

		if ((p->va_debug_mode != DBMDX_DEBUG_MODE_OFF) &&
			(p->active_interface != DBMDX_INTERFACE_UART) &&
			(cur_reg ==
		(u16)((DBMDX_REGN_HOST_INTERFACE_SUPPORT >> 16) & 0xff))) {
			ret = dbmdx_send_cmd(p,
				(p->pdata->va_ve_cfg_value[i] | 0x1000), NULL);
			if (ret < 0) {
				dev_err(p->dev,
					"%s: failed to send cmd\n", __func__);
				ret = -EIO;
				goto out;
			}
			if (!(p->va_debug_mode & DBMDX_DEBUG_MODE_FW_LOG)) {
				ret = dbmdx_send_cmd(p,
					(DBMDX_REGN_FW_DEBUG_REGISTER |
					DBMDX_REGV_TOGGLE_UART_DEBUG_PRINTS),
					NULL);
				if (ret < 0) {
					dev_err(p->dev,
						"%s: failed to send cmd\n",
						__func__);
					ret = -EIO;
					goto out;
				}
			}
			continue;
		}

		ret = dbmdx_send_cmd(p, p->pdata->va_ve_cfg_value[i], NULL);
		if (ret < 0) {
			dev_err(p->dev, "%s: failed to send cmd\n", __func__);
			ret = -EIO;
			goto out;
		}
	}


	/* Give to PLL enough time for stabilization */
	msleep(DBMDX_MSLEEP_CONFIG_VA_MODE_REG);

	p->chip->transport_enable(p, true);

	/* read firmware version */
	ret = dbmdx_send_cmd(p, DBMDX_REGN_FW_VERSION_NUMBER, &fwver);
	if (ret < 0) {
		dev_err(p->dev, "%s: could not read firmware version\n",
			__func__);
		goto out;
	}

	ret = customer_dbmdx_va_ve_post_init(p);
	if (ret < 0) {
		dev_err(p->dev, "%s: custom VA_VE_POSTINIT error\n",
			__func__);
		goto out;
	}

	dev_info(p->dev, "%s: VA_VE firmware 0x%x ready\n", __func__, fwver);

	ret = 0;
out:
	return ret;
}

static int dbmdx_va_ve_firmware_ready(struct dbmdx_private *p)
{
	int ret;
	u32 cur_va_boot_mode = DBMDX_BOOT_MODE_NORMAL_BOOT;
	u32 cur_va_ve_boot_mode = DBMDX_BOOT_MODE_NORMAL_BOOT;

	dev_dbg(p->dev, "%s\n", __func__);

	/* Boot VA_VE chip */

	p->boot_mode = DBMDX_BOOT_MODE_NORMAL_BOOT;

	if (p->va_ve_chip_enabled &&
		(p->pdata->boot_options_va_ve & DBMDX_BOOT_OPT_SEND_PREBOOT)) {

		ret = dbmdx_switch_to_va_ve_chip_interface(p,
			DBMDX_PREBOOT_INTERFACE);
		if (ret) {
			dev_err(p->dev,
				"%s Error switching to (VA_VE) interface\n",
				__func__);
			return ret;
		}

		dbmdx_reset_sequence(p);

		msleep(DBMDX_MSLEEP_I2C_D2_AFTER_RESET_32K);

		ret = p->chip->prepare_boot(p);
		if (ret) {
			dev_err(p->dev,
				"%s Error preparing (VA_VE) interface\n",
				__func__);
			return ret;
		}

		/* preboot */
		ret = p->chip->write(p, p->va_ve_preboot_fw->data,
			p->va_ve_preboot_fw->size);

		if (ret != p->va_ve_preboot_fw->size) {
			dev_err(p->dev,
				"%s Error sending the Preboot FW (VA_VE)\n",
				__func__);
			return -EIO;
		}

		dev_err(p->dev, "%s Preboot was sent successfully (VA_VE)\n",
			__func__);

		cur_va_ve_boot_mode |= DBMDX_BOOT_MODE_RESET_DISABLED;

		if (p->pdata->reset_gpio_shared)
			cur_va_boot_mode |= DBMDX_BOOT_MODE_RESET_DISABLED;
	}
	/* Boot VA chip */
	if (p->va_chip_enabled &&
		(p->pdata->boot_options & DBMDX_BOOT_OPT_SEND_PREBOOT)) {

		ret = dbmdx_switch_to_va_chip_interface(p,
			DBMDX_PREBOOT_INTERFACE);
		if (ret) {
			dev_err(p->dev,
				"%s Error switching to (VA) interface\n",
				__func__);
			return ret;
		}

		/* If reset gpio is shared reset both chips */
		if (!(cur_va_boot_mode & DBMDX_BOOT_MODE_RESET_DISABLED)) {

			dbmdx_reset_sequence(p);

			msleep(DBMDX_MSLEEP_I2C_D2_AFTER_RESET_32K);

			cur_va_boot_mode |= DBMDX_BOOT_MODE_RESET_DISABLED;

			if (p->pdata->reset_gpio_shared)
				cur_va_ve_boot_mode |=
					DBMDX_BOOT_MODE_RESET_DISABLED;
		}
		ret = p->chip->prepare_boot(p);
		if (ret) {
			dev_err(p->dev,
				"%s Error preparing (VA) interface\n",
				__func__);
			return ret;
		}

		/* preboot */
		ret = p->chip->write(p, p->va_preboot_fw->data,
			p->va_preboot_fw->size);

		if (ret != p->va_preboot_fw->size) {
			dev_err(p->dev,
				"%s Error sending the Preboot FW (VA)\n",
				__func__);
			return -EIO;
		}

		dev_err(p->dev, "%s Preboot was sent successfully (VA)\n",
			__func__);

	}

	if (p->va_ve_chip_enabled) {

		ret = dbmdx_switch_to_va_ve_chip_interface(p,
							DBMDX_BOOT_INTERFACE);
		if (ret) {
			dev_err(p->dev,
			"%s Error switching to (VA_VE) BOOT interface\n",
				__func__);
			return ret;
		}

		p->boot_mode = cur_va_ve_boot_mode;

		/* BOOT VA_VE  */
		ret = dbmdx_firmware_ready(p->va_ve_fw, p);
		if (ret != 0) {
			dev_err(p->dev, "%s: could not load VA_VE firmware\n",
				__func__);
			p->boot_mode = DBMDX_BOOT_MODE_NORMAL_BOOT;
			return -EIO;
		}

		dbmdx_set_va_ve_active(p);

		p->boot_mode = DBMDX_BOOT_MODE_NORMAL_BOOT;

		dev_dbg(p->dev, "%s VA_VE FW was loaded successfully\n",
			__func__);

		ret = dbmdx_switch_to_va_ve_chip_interface(p,
							DBMDX_CMD_INTERFACE);
		if (ret) {
			dev_err(p->dev,
				"%s Error switching to (VA_VE) CMD interface\n",
				__func__);
			return ret;
		}

		ret = dbmdx_config_va_ve_mode(p);
		if (ret != 0) {
			dev_err(p->dev,
				"%s: could not configure VA_VE firmware\n",
				__func__);
			return -EIO;
		}

		ret = p->chip->set_va_firmware_ready(p);
		if (ret) {
			dev_err(p->dev,
				"%s: could not set to ready VA_VE firmware\n",
				__func__);
			return -EIO;
		}

		if (p->pdata->reset_gpio_shared)
			cur_va_boot_mode |= DBMDX_BOOT_MODE_RESET_DISABLED;

		p->usr_selected_chip = DBMDX_CHIP_VA_VE;
	}

	if (p->va_chip_enabled) {
		ret = dbmdx_switch_to_va_chip_interface(p,
							DBMDX_BOOT_INTERFACE);
		if (ret) {
			dev_err(p->dev,
				"%s Error switching to (VA) BOOT interface\n",
				__func__);
			return ret;
		}

		p->boot_mode = cur_va_boot_mode;

		/* BOOT VA interface */
		ret = dbmdx_firmware_ready(p->va_fw, p);
		if (ret != 0) {
			dev_err(p->dev, "%s: could not load VA firmware\n",
				__func__);
			p->boot_mode = DBMDX_BOOT_MODE_NORMAL_BOOT;
			return -EIO;
		}

		p->boot_mode = DBMDX_BOOT_MODE_NORMAL_BOOT;

		dev_dbg(p->dev, "%s VA FW was loaded successfully\n", __func__);

		dbmdx_set_va_active(p);

		ret = dbmdx_switch_to_va_chip_interface(p, DBMDX_CMD_INTERFACE);
		if (ret) {
			dev_err(p->dev,
				"%s Error switching to (VA) CMD interface\n",
			__func__);
			return ret;
		}

		ret = dbmdx_config_va_mode(p);
		if (ret != 0) {
			dev_err(p->dev, "%s: could not configure VA firmware\n",
				__func__);
			return -EIO;
		}

		ret = p->chip->set_va_firmware_ready(p);
		if (ret) {
			dev_err(p->dev,
				"%s: could not set to ready VA firmware\n",
				__func__);
			return -EIO;
		}

		if (p->pdata->reset_gpio_shared)
			cur_va_ve_boot_mode |= DBMDX_BOOT_MODE_RESET_DISABLED;

		p->usr_selected_chip = DBMDX_CHIP_VA;
	}

	return 0;
}

static int dbmdx_va_firmware_ready(struct dbmdx_private *p)
{
	int ret;

	dev_dbg(p->dev, "%s\n", __func__);

	p->boot_mode = DBMDX_BOOT_MODE_NORMAL_BOOT;

	/* Boot VA chip */
	if (p->pdata->boot_options & DBMDX_BOOT_OPT_SEND_PREBOOT) {

		ret = dbmdx_switch_to_va_chip_interface(p,
			DBMDX_PREBOOT_INTERFACE);
		if (ret) {
			dev_err(p->dev,
				"%s Error switching to (VA) interface\n",
				__func__);
			return ret;
		}

		dbmdx_reset_sequence(p);

		msleep(DBMDX_MSLEEP_I2C_D2_AFTER_RESET_32K);

		/* preboot */
		ret = p->chip->write(p, p->va_preboot_fw->data,
			p->va_preboot_fw->size);

		if (ret != p->va_preboot_fw->size) {
			dev_err(p->dev,
				"%s Error sending the Preboot FW (VA)\n",
				__func__);
			return -EIO;
		}

		dev_err(p->dev, "%s Preboot was sent successfully (VA)\n",
			__func__);

		p->boot_mode = DBMDX_BOOT_MODE_RESET_DISABLED;
	}

	ret = dbmdx_switch_to_va_chip_interface(p, DBMDX_BOOT_INTERFACE);
	if (ret) {
		dev_err(p->dev,
			"%s Error switching to (VA) BOOT interface\n",
			__func__);
		p->boot_mode = DBMDX_BOOT_MODE_NORMAL_BOOT;
		return ret;
	}

	/* common boot */
	ret = dbmdx_firmware_ready(p->va_fw, p);
	if (ret != 0) {
		dev_err(p->dev, "%s: could not load VA firmware\n", __func__);
		return -EIO;
	}

	p->boot_mode = DBMDX_BOOT_MODE_NORMAL_BOOT;

	dbmdx_set_va_active(p);

	ret = dbmdx_switch_to_va_chip_interface(p, DBMDX_CMD_INTERFACE);
	if (ret) {
		dev_err(p->dev,
			"%s Error switching to (VA) CMD interface\n",
			__func__);
		return ret;
	}

	ret = dbmdx_config_va_mode(p);
	if (ret != 0) {
		dev_err(p->dev, "%s: could not configure VA firmware\n",
			__func__);
		return -EIO;
	}

	ret = p->chip->set_va_firmware_ready(p);
	if (ret) {
		dev_err(p->dev, "%s: could not set to ready VA firmware\n",
			__func__);
		return -EIO;
	}

	return 0;
}

static int dbmdx_switch_to_va_firmware(struct dbmdx_private *p, bool do_reset)
{
	int ret;

	if (p->active_fw == DBMDX_FW_VA)
		return 0;

	if (!p->pdata->feature_va) {
		dev_err(p->dev, "%s: VA feature not enabled\n", __func__);
		return -EINVAL;
	}

	if (do_reset)
		dbmdx_set_boot_active(p);

	dbmdx_set_power_mode(p, DBMDX_PM_BOOTING);

	dev_dbg(p->dev, "%s: switching to VA firmware\n", __func__);

	p->device_ready = false;

	ret = dbmdx_va_firmware_ready(p);
	if (ret)
		return ret;

	p->device_ready = true;
	dbmdx_set_power_mode(p, DBMDX_PM_ACTIVE);

	return 0;
}
static int dbmdx_switch_to_va_ve_firmware(struct dbmdx_private *p,
					  bool do_reset)
{
	int ret = 0;
	int retry = RETRY_COUNT;

	if (p->active_fw == DBMDX_FW_VA || p->active_fw == DBMDX_FW_VA_VE)
		return 0;

	if (!p->pdata->feature_va_ve) {
		dev_err(p->dev, "%s: VA_VE feature not enabled\n", __func__);
		return -EINVAL;
	}

	if (do_reset) {
		dbmdx_set_boot_active_va_ve_chip(p);
		dbmdx_set_boot_active(p);
	}

	dbmdx_set_power_mode(p, DBMDX_PM_BOOTING);

	dev_dbg(p->dev, "%s: switching to VA VE firmware\n", __func__);

	p->device_ready = false;

	while (retry--) {
		ret = dbmdx_va_ve_firmware_ready(p);
		if (!ret)
			break;

		dbmdx_set_boot_active_va_ve_chip(p);
		dbmdx_set_boot_active(p);
	}

	if (ret)
		return ret;

	p->device_ready = true;

	ret = dbmdx_va_ve_send_idle_usecase_config(p, false);
	if (ret != 0) {
		dev_err(p->dev, "%s: could set idle usecase\n",	__func__);
		return -EIO;
	}

	dbmdx_set_power_mode(p, DBMDX_PM_ACTIVE);

	return 0;
}

static int dbmdx_request_and_load_fw_va_ve_mode(struct dbmdx_private *p)
{
	int ret;
	int retries = 30;

	if (!p->pdata->feature_va_ve) {
		dev_err(p->dev, "%s: VA_VE feature not enabled\n", __func__);
		return -EINVAL;
	}

	p->lock(p);

	dbmdx_set_boot_active(p);
	dbmdx_set_boot_active_va_ve_chip(p);

	if (p->va_fw) {
		release_firmware(p->va_fw);
		p->va_fw = NULL;
	}

	if (p->va_ve_fw) {
		release_firmware(p->va_ve_fw);
		p->va_ve_fw = NULL;
	}

	if (p->va_preboot_fw) {
		release_firmware(p->va_preboot_fw);
		p->va_preboot_fw = NULL;
	}

	if (p->va_ve_preboot_fw) {
		release_firmware(p->va_ve_preboot_fw);
		p->va_ve_preboot_fw = NULL;
	}

	ret = dbmdx_set_power_mode(p, DBMDX_PM_BOOTING);
	if (ret != 0) {
		dev_err(p->dev, "%s: could not change to DBMDX_PM_BOOTING\n",
			__func__);
		goto out;
	}

	do {
		if (p->va_chip_enabled) {
			dev_info(p->dev, "%s: request VA firmware - %s\n",
				__func__, p->pdata->va_firmware_name);
			ret =
			request_firmware((const struct firmware **)&p->va_fw,
				       p->pdata->va_firmware_name, p->dev);
			if (ret != 0) {
				dev_err(p->dev,
					"%s: failed to request VA firmware\n",
					__func__);
				msleep(DBMDX_MSLEEP_REQUEST_FW_FAIL);
				continue;
			}
			if (p->pdata->boot_options &
						DBMDX_BOOT_OPT_SEND_PREBOOT) {
				dev_info(p->dev,
				"%s: request VA preboot firmware - %s\n",
					 __func__,
					 p->pdata->va_preboot_firmware_name);
				ret = request_firmware(
				(const struct firmware **)&p->va_preboot_fw,
				p->pdata->va_preboot_firmware_name, p->dev);

				if (ret != 0) {
					dev_err(p->dev,
					"%s: failed to request VA preboot fw\n",
					__func__);
					retries = 0;
					break;
				}
			}
		}

		if (p->va_ve_chip_enabled) {
			dev_info(p->dev, "%s: request VA_VE firmware - %s\n",
				__func__, p->pdata->va_ve_firmware_name);
			ret =
			request_firmware((const struct firmware **)&p->va_ve_fw,
				       p->pdata->va_ve_firmware_name, p->dev);
			if (ret != 0) {
				dev_err(p->dev,
					"%s: failed to request VA_VE firmware\n",
					__func__);
				retries = 0;
				break;
			}

			if (p->pdata->boot_options_va_ve &
				DBMDX_BOOT_OPT_SEND_PREBOOT) {
				dev_info(p->dev,
				"%s: request VA_VE  preboot firmware - %s\n",
					 __func__,
					 p->pdata->va_ve_preboot_firmware_name);
				ret = request_firmware(
				(const struct firmware **)&p->va_ve_preboot_fw,
				p->pdata->va_ve_preboot_firmware_name, p->dev);

				if (ret != 0) {
					dev_err(p->dev,
					"%s: failed to request VA_VE preboot fw\n",
					__func__);
					retries = 0;
					break;
				}
			}
		}

		break;
	} while (--retries);

	if (retries == 0) {
		dev_err(p->dev, "%s: failed to request firmware\n",
			__func__);
		ret = -EIO;
		goto out_err;
	}
	if (p->va_ve_chip_enabled) {
		ret = dbmdx_switch_to_va_ve_chip_interface(p,
			DBMDX_PREBOOT_INTERFACE);
		if (ret) {
			dev_err(p->dev,
				"%s Error switching to (VA_VE) interface\n",
				__func__);
			ret = -EIO;
			goto out_err;
		}
	} else {
		ret = dbmdx_switch_to_va_chip_interface(p,
			DBMDX_PREBOOT_INTERFACE);
		if (ret) {
			dev_err(p->dev,
				"%s Error switching to (VA) interface\n",
				__func__);
			ret = -EIO;
			goto out_err;
		}
	}


	ret = dbmdx_switch_to_va_ve_firmware(p, 1);
	if (ret != 0) {
		dev_err(p->dev, "%s: failed to boot VA firmware\n",
			__func__);
		ret = -EIO;
		goto out_err;
	}

	ret = customer_dbmdx_load_va_ve_fw_on_exit(p);
	if (ret != 0) {
		dev_err(p->dev, "%s: custom LOAD_VA_VE_FW_ONEXIT error\n",
			__func__);
		ret = -EIO;
		goto out_err;
	}

	/* fall asleep by default after boot */
	ret = dbmdx_set_power_mode(p, DBMDX_PM_FALLING_ASLEEP);
	if (ret != 0) {
		dev_err(p->dev,
			"%s: could not change to DBMDX_PM_FALLING_ASLEEP\n",
			__func__);
		goto out_err;
	}
	p->device_ready = true;
	ret = 0;
	goto out;

out_err:
	if (p->va_fw) {
		release_firmware(p->va_fw);
		p->va_fw = NULL;
	}
	if (p->va_ve_fw) {
		release_firmware(p->va_fw);
		p->va_fw = NULL;
	}
	if (p->va_preboot_fw) {
		release_firmware(p->va_preboot_fw);
		p->va_preboot_fw = NULL;
	}

	if (p->va_ve_preboot_fw) {
		release_firmware(p->va_ve_preboot_fw);
		p->va_ve_preboot_fw = NULL;
	}

out:
	p->unlock(p);
	return ret;
}

static int dbmdx_request_and_load_fw(struct dbmdx_private *p, int va)
{
	int ret;
	int retries = 30;

	dev_dbg(p->dev, "%s %s\n",
		__func__, va ? "VA" : "-");

	p->lock(p);

	dbmdx_set_boot_active(p);

	if (va && p->va_fw) {
		release_firmware(p->va_fw);
		p->va_fw = NULL;
	}

	if (va && p->va_preboot_fw) {
		release_firmware(p->va_preboot_fw);
		p->va_preboot_fw = NULL;
	}

	ret = dbmdx_set_power_mode(p, DBMDX_PM_BOOTING);
	if (ret != 0) {
		dev_err(p->dev, "%s: could not change to DBMDX_PM_BOOTING\n",
			__func__);
		goto out;
	}

	if (p->pdata->feature_va && va) {
		/* request VA firmware */
		do {
			dev_info(p->dev, "%s: request VA firmware - %s\n",
				 __func__, p->pdata->va_firmware_name);
			ret =
			request_firmware((const struct firmware **)&p->va_fw,
					       p->pdata->va_firmware_name,
					       p->dev);
			if (ret != 0) {
				dev_err(p->dev,
					"%s: failed to request VA firmware\n",
					__func__);
				msleep(DBMDX_MSLEEP_REQUEST_FW_FAIL);
				continue;
			}
			if (p->pdata->boot_options &
					DBMDX_BOOT_OPT_SEND_PREBOOT) {
				dev_info(p->dev,
				"%s: request VA preboot firmware - %s\n",
				__func__,
				p->pdata->va_preboot_firmware_name);

				ret = request_firmware(
				(const struct firmware **)&p->va_preboot_fw,
				p->pdata->va_preboot_firmware_name, p->dev);

				if (ret != 0) {
					dev_err(p->dev,
					"%s: failed to request VA preboot fw\n",
						__func__);
					retries = 0;
					break;
				}
			}
			break;
		} while (--retries);

		if (retries == 0) {
			dev_err(p->dev, "%s: failed to request VA firmware\n",
				__func__);
			ret = -EIO;
			goto out;
		}

	}

	if (p->pdata->feature_va && va) {
		ret = dbmdx_switch_to_va_chip_interface(p,
							DBMDX_BOOT_INTERFACE);
		if (ret) {
			dev_err(p->dev,
				"%s Error switching to (VA) BOOT interface\n",
				__func__);
			ret = -EIO;
			goto out_err;
		}

		ret = dbmdx_switch_to_va_firmware(p, 1);
		if (ret != 0) {
			dev_err(p->dev, "%s: failed to boot VA firmware\n",
				__func__);
			ret = -EIO;
			goto out_err;
		}
	}
	/* fall asleep by default after boot */
	ret = dbmdx_set_power_mode(p, DBMDX_PM_FALLING_ASLEEP);
	if (ret != 0) {
		dev_err(p->dev,
			"%s: could not change to DBMDX_PM_FALLING_ASLEEP\n",
			__func__);
		goto out_err;
	}
	p->device_ready = true;
	ret = 0;
	goto out;

out_err:
	if (p->va_fw) {
		release_firmware(p->va_fw);
		p->va_fw = NULL;
	}
out:
	p->unlock(p);
	return ret;
}

int dbmdx_set_va_usecase_name(struct dbmdx_private *p, const char *buf)
{
	int ret;
	int ind;
	int num_usecases = sizeof(usecases_map) /
				sizeof(struct usecase_config *);
	struct usecase_config *cur_usecase = NULL;
	int n;

	if (!p)
		return -EAGAIN;

	if (!p->device_ready) {
		dev_err(p->dev, "%s: device not ready\n", __func__);
		return -EAGAIN;
	}

	dev_dbg(p->dev, "%s: usecase name - %s\n", __func__, buf);
	n = strlen(buf);

	if (!n) {
		dev_err(p->dev,	"%s: No usecase name supplied\n", __func__);
		ret = -EINVAL;
		goto out;
	}

	if (buf[n-1] == '\n')
		n = n - 1;

	if (!n) {
		dev_err(p->dev,	"%s: No usecase name supplied\n", __func__);
		ret = -EINVAL;
		goto out;
	}

	/* IDLE Usecase */
	if (!strncmp(buf, "idle", 4)) {
		cur_usecase = NULL;
	} else if (!strncmp(buf, "external", 8)) {
		if (!(p->external_usecase_loaded)) {
			dev_err(p->dev,	"%s: External usecase is not loaded\n",
				__func__);
			ret = -EINVAL;
			goto out;
		}
		cur_usecase = &config_usecase_external;
	} else {

		for (ind = 0; ind < num_usecases; ind++) {
			cur_usecase = usecases_map[ind];

			if (!cur_usecase)
				continue;
			if (n != strlen(cur_usecase->usecase_name))
				continue;
			if (!strncmp(buf, cur_usecase->usecase_name, n)) {
				/* Verify HW Rev */
				if (p->pdata->hw_rev == DBMDX_DEFAULT_HW_REV)
					break;
				if (p->pdata->hw_rev != cur_usecase->hw_rev)
					continue;
				break;
			}
		}

		if (ind >= num_usecases || !cur_usecase) {
			dev_err(p->dev,
				"%s: Unsupported usecase %s\n", __func__, buf);
			ret = -EINVAL;
			goto out;
		}
	}

	ret = dbmdx_va_usecase_update(p, cur_usecase);

	if (ret < 0 && ret != -EINVAL && ret != -ENOENT &&
					!p->pdata->va_recovery_disabled) {
		int recovery_res;

		if (p->device_ready && (dbmdx_va_ve_alive_with_lock(p) == 0)) {
			dev_err(p->dev,
				"%s: DBMDX response has been verified\n",
				__func__);
			goto out;
		}

		dev_err(p->dev, "%s: Performing recovery #1\n", __func__);

		recovery_res = dbmdx_perform_recovery(p);

		if (recovery_res) {
			dev_err(p->dev, "%s: recovery failed\n", __func__);
			goto out;
		}

		ret = dbmdx_va_usecase_update(p, cur_usecase);

		if (ret == 0) {
			dev_err(p->dev,
				"%s: Usecase was set after succesfull recovery\n",
				__func__);
			goto out;
		}

		if (p->device_ready && (dbmdx_va_ve_alive_with_lock(p) == 0)) {
			dev_err(p->dev,
				"%s: DBMDX response has been verified\n",
				__func__);
			goto out;
		}

		dev_err(p->dev, "%s: Performing recovery #2\n", __func__);

		recovery_res = dbmdx_perform_recovery(p);

		if (recovery_res) {
			dev_err(p->dev, "%s: recovery failed\n", __func__);
			goto out;
		}

	}
out:
	return ret;
}

int dbmdx_va_get_usecase_by_id(struct dbmdx_private *p, int usecase_id,
				struct usecase_config **usecase)
{
	int ind;
	int num_usecases = sizeof(usecases_map) /
				sizeof(struct usecase_config *);
	struct usecase_config *cur_usecase = NULL;

	if (!p)
		return -EAGAIN;

	if (!p->device_ready) {
		dev_err(p->dev, "%s: device not ready\n", __func__);
		return -EAGAIN;
	}

	if (!usecase) {
		dev_err(p->dev, "%s: NULL PTR usecase\n", __func__);
		return -EINVAL;
	}

	*usecase = NULL;

	/* IDLE Usecase */
	if (usecase_id == 0) {
		cur_usecase = NULL;
		return 0;
	}

	for (ind = 0; ind < num_usecases; ind++) {
		cur_usecase = usecases_map[ind];

		if (!cur_usecase)
			continue;
		if (usecase_id == cur_usecase->id) {
			*usecase = cur_usecase;
			break;
		}
	}

	if (ind >= num_usecases || !cur_usecase) {
		dev_err(p->dev,
			"%s: Unsupported usecase id:%d\n", __func__,
							usecase_id);
		return -EINVAL;
	}

	return 0;
}

int dbmdx_va_set_usecase_id(struct dbmdx_private *p, int usecase_id)
{
	int ret = 0;
	int ind;
	int num_usecases = sizeof(usecases_map) /
				sizeof(struct usecase_config *);
	struct usecase_config *cur_usecase = NULL;

	if (!p)
		return -EAGAIN;

	if (!p->device_ready) {
		dev_err(p->dev, "%s: device not ready\n", __func__);
		return -EAGAIN;
	}

	dev_dbg(p->dev, "%s: Usecase ID: %d\n", __func__, usecase_id);

	/* IDLE Usecase */
	if (usecase_id == 0) {
		cur_usecase = NULL;
	} else {

		for (ind = 0; ind < num_usecases; ind++) {
			cur_usecase = usecases_map[ind];

			if (!cur_usecase)
				continue;
			if (usecase_id == cur_usecase->id)
				break;
		}

		if (ind >= num_usecases || !cur_usecase) {
			dev_err(p->dev,
				"%s: Unsupported usecase id:%d\n", __func__,
								usecase_id);
			ret = -EINVAL;
			goto out;
		}
	}

	ret = dbmdx_va_usecase_update(p, cur_usecase);

	if (ret < 0 && ret != -EINVAL && ret != -ENOENT &&
					!p->pdata->va_recovery_disabled) {
		int recovery_res;

		if (p->device_ready && (dbmdx_va_ve_alive_with_lock(p) == 0)) {
			dev_err(p->dev,
				"%s: DBMDX response has been verified\n",
				__func__);
			goto out;
		}

		dev_err(p->dev, "%s: Performing recovery #1\n", __func__);

		recovery_res = dbmdx_perform_recovery(p);

		if (recovery_res) {
			dev_err(p->dev, "%s: recovery failed\n", __func__);
			goto out;
		}

		ret = dbmdx_va_usecase_update(p, cur_usecase);

		if (ret == 0) {
			dev_err(p->dev,
				"%s: Usecase was set after succesfull recovery\n",
				__func__);
			goto out;
		}

		if (p->device_ready && (dbmdx_va_ve_alive_with_lock(p) == 0)) {
			dev_err(p->dev,
				"%s: DBMDX response has been verified\n",
				__func__);
			goto out;
		}

		dev_err(p->dev, "%s: Performing recovery #2\n", __func__);

		recovery_res = dbmdx_perform_recovery(p);

		if (recovery_res) {
			dev_err(p->dev, "%s: recovery failed\n", __func__);
			goto out;
		}

	}
out:
	return ret;
}


/* ------------------------------------------------------------------------
 * sysfs attributes
 * ------------------------------------------------------------------------
 */

static ssize_t dbmdx_reg_show_va(struct device *dev, u32 command,
			      struct device_attribute *attr, char *buf)
{
	struct dbmdx_private *p = dev_get_drvdata(dev);
	int ret, ret2;
	u16 val = 0;
	enum dbmdx_firmware_active	cur_active_fw;

	cur_active_fw = p->active_fw;

	if (!p)
		return -EAGAIN;

	if (!p->device_ready) {
		dev_err(p->dev, "%s: device not ready\n", __func__);
		return -EAGAIN;
	}

	if (!p->va_chip_enabled) {
		dev_err(p->dev, "%s: VA Chip is not enabled\n", __func__);
		return -EAGAIN;
	}

	dev_dbg(p->dev, "%s: get VA reg %x\n", __func__, command);

	if (((p->active_fw == DBMDX_FW_VA) || (p->active_fw == DBMDX_FW_VA_VE))
		&& !(command & DBMDX_VA_CMD_MASK)) {
		dev_err(p->dev, "%s: VA_VE mode is not active\n", __func__);
		return -ENODEV;
	}

	p->lock(p);
	dbmdx_set_power_mode(p, DBMDX_PM_ACTIVE);

	ret = dbmdx_switch_to_va_chip_interface(p, DBMDX_CMD_INTERFACE);
	if (ret) {
		dev_err(p->dev,	"%s Error switching to VA interface\n",
					__func__);
		ret = -EIO;
		goto out_unlock;
	}

	ret = dbmdx_send_cmd(p, command, &val);
	if (ret < 0) {
		dev_err(p->dev, "%s: get reg %x error\n",
				__func__, command);
		goto out_unlock;
	}

	ret = snprintf(buf, PAGE_SIZE, "0x%x\n", val);

out_unlock:
	dbmdx_set_power_mode(p, DBMDX_PM_FALLING_ASLEEP);
	if (p->va_ve_chip_enabled && (cur_active_fw == DBMDX_FW_VA_VE)) {
		ret2 = dbmdx_switch_to_va_ve_chip_interface(p,
						DBMDX_CMD_INTERFACE);
		if (ret2)
			dev_err(p->dev,
				"%s Error switching to (VA_VE) interface\n",
				__func__);

	}

	p->unlock(p);
	return ret;
}

static ssize_t dbmdx_reg_show_va_ve(struct device *dev, u32 command,
			      struct device_attribute *attr, char *buf)
{
	struct dbmdx_private *p = dev_get_drvdata(dev);
	int ret, ret2;
	u16 val = 0;
	enum dbmdx_firmware_active	cur_active_fw;

	cur_active_fw = p->active_fw;

	if (!p)
		return -EAGAIN;

	if (!p->device_ready) {
		dev_err(p->dev, "%s: device not ready\n", __func__);
		return -EAGAIN;
	}

	if (!p->va_ve_chip_enabled) {
		dev_err(p->dev, "%s: VA_VE Chip is not enabled\n", __func__);
		return -EAGAIN;
	}

	dev_dbg(p->dev, "%s: get VA_VE reg %x\n", __func__, command);

	if (((p->active_fw == DBMDX_FW_VA) || (p->active_fw == DBMDX_FW_VA_VE))
		&& !(command & DBMDX_VA_CMD_MASK)) {
		dev_err(p->dev, "%s: VA_VE mode is not active\n", __func__);
		return -ENODEV;
	}

	p->lock(p);
	dbmdx_set_power_mode(p, DBMDX_PM_ACTIVE);

	ret = dbmdx_switch_to_va_ve_chip_interface(p, DBMDX_CMD_INTERFACE);
	if (ret) {
		dev_err(p->dev,	"%s Error switching to VA_VE interface\n",
					__func__);
		ret = -EIO;
		goto out_unlock;
	}

	ret = dbmdx_send_cmd(p, command, &val);
	if (ret < 0) {
		dev_err(p->dev, "%s: get reg %x error\n",
				__func__, command);
		goto out_unlock;
	}

	ret = snprintf(buf, PAGE_SIZE, "0x%x\n", val);

out_unlock:
	dbmdx_set_power_mode(p, DBMDX_PM_FALLING_ASLEEP);
	if (p->va_chip_enabled && (cur_active_fw == DBMDX_FW_VA)) {
		ret2 = dbmdx_switch_to_va_chip_interface(p,
						DBMDX_CMD_INTERFACE);
		if (ret2)
			dev_err(p->dev,
				"%s Error switching to (VA) interface\n",
				__func__);

	}

	p->unlock(p);
	return ret;
}

static ssize_t dbmdx_reg_show(struct device *dev, u32 command,
			      struct device_attribute *attr, char *buf)
{
	struct dbmdx_private *p = dev_get_drvdata(dev);
	int ret;
	u16 val = 0;

	if (!p)
		return -EAGAIN;

	if (!p->device_ready) {
		dev_err(p->dev, "%s: device not ready\n", __func__);
		return -EAGAIN;
	}

	dev_dbg(p->dev, "%s: get reg %x\n", __func__, command);

	if (((p->active_fw == DBMDX_FW_VA) || (p->active_fw == DBMDX_FW_VA_VE))
		&& !(command & DBMDX_VA_CMD_MASK)) {
		dev_err(p->dev, "%s: Bad command format mask\n", __func__);
		return -ENODEV;
	}

	p->lock(p);
	dbmdx_set_power_mode(p, DBMDX_PM_ACTIVE);

	ret = dbmdx_send_cmd(p, command, &val);
	if (ret < 0) {
		dev_err(p->dev, "%s: get reg %x error\n",
				__func__, command);
		goto out_unlock;
	}

	ret = snprintf(buf, PAGE_SIZE, "0x%x\n", val);

out_unlock:
	dbmdx_set_power_mode(p, DBMDX_PM_FALLING_ASLEEP);
	p->unlock(p);
	return ret;
}

static ssize_t dbmdx_reg_show_long(struct device *dev,
				u32 command,
				struct device_attribute *attr, char *buf)
{
	struct dbmdx_private *p = dev_get_drvdata(dev);
	int ret;
	u32 result;

	if (!p)
		return -EAGAIN;

	if (!p->device_ready) {
		dev_err(p->dev, "%s: device not ready\n", __func__);
		return -EAGAIN;
	}

	p->lock(p);
	ret = dbmdx_switch_to_user_selected_chip_interface(p);
	if (ret) {
		p->unlock(p);
		return snprintf(buf, PAGE_SIZE,
				"Error switching to user sel. chip iface\n");
	}
	dbmdx_set_power_mode(p, DBMDX_PM_ACTIVE);

	ret = dbmdx_send_cmd_32(p, command, 0, &result);
	if (ret < 0) {
		dev_err(p->dev, "%s: get reg %u error\n",
				__func__, command);
		goto out_unlock;
	}

	dev_dbg(p->dev, "%s: val = 0x%08x\n", __func__, result);
	ret = snprintf(buf, PAGE_SIZE, "0x%x\n", result);

out_unlock:

	dbmdx_set_power_mode(p, DBMDX_PM_FALLING_ASLEEP);
	if (dbmdx_switch_to_drv_selected_chip_interface(p)) {
		p->unlock(p);
		return snprintf(buf, PAGE_SIZE,
				"Error switching to user sel. chip iface\n");
	}

	p->unlock(p);
	return ret;
}

static ssize_t dbmdx_reg_store(struct device *dev, u32 command,
			       struct device_attribute *attr,
			       const char *buf, size_t size, int fw)
{
	struct dbmdx_private *p = dev_get_drvdata(dev);
	unsigned long val;
	int ret = 0;

	if (!p)
		return -EAGAIN;

	if (!p->device_ready) {
		dev_err(p->dev, "%s: device not ready\n", __func__);
		return -EAGAIN;
	}

	ret = kstrtoul(buf, 0, &val);
	if (ret)
		return -EINVAL;

	p->lock(p);

	dbmdx_set_power_mode(p, DBMDX_PM_ACTIVE);

	if (fw != p->active_fw) {
		if (fw == DBMDX_FW_VA && p->va_chip_enabled) {
			ret = dbmdx_switch_to_va_chip_interface(p,
							DBMDX_CMD_INTERFACE);
			if (ret) {
				dev_err(p->dev,
				"%s Error switching to (VA) CMD interface\n",
					__func__);
				ret = -EIO;
				goto out_unlock;
			}
		} else if (fw == DBMDX_FW_VA_VE && p->va_ve_chip_enabled) {
			ret = dbmdx_switch_to_va_ve_chip_interface(p,
							DBMDX_CMD_INTERFACE);
			if (ret) {
				dev_err(p->dev,
				"%s Error switching to (VA_VE) CMD interface\n",
					__func__);
				ret = -EIO;
				goto out_unlock;
			}
		} else {
			ret = -EINVAL;
			goto out_unlock;
		}
	}
	if (p->active_fw == DBMDX_FW_VA) {
		if (command == DBMDX_REGN_VT_AUDIO_HISTORY_CFG) {
			ret = dbmdx_set_backlog_len(p, val);
			if (ret < 0) {
				dev_err(p->dev, "%s: set history error\n",
					__func__);
				size = ret;
			}
			goto out_pm_mode;
		}

		ret = dbmdx_send_cmd(p, command | (u32)val, NULL);
		if (ret < 0) {
			dev_err(p->dev, "%s: set VA reg error\n", __func__);
			size = ret;
			goto out_pm_mode;
		}

		if (command == DBMDX_REGN_DIGITAL_GAIN) {
			if (p->va_active_mic_config != DBMDX_MIC_MODE_ANALOG)
				p->va_cur_digital_mic_digital_gain = (int)val;
			else
				p->va_cur_analog_mic_digital_gain = (int)val;
		} else if (command == DBMDX_REGN_MICROPHONE_ANALOG_GAIN)
			p->va_cur_analog_mic_analog_gain = (int)val;
	}

out_pm_mode:
	dbmdx_set_power_mode(p, DBMDX_PM_FALLING_ASLEEP);

out_unlock:
	p->unlock(p);
	return size;
}

static ssize_t dbmdx_reg_store_long(struct device *dev, u32 command,
				    struct device_attribute *attr,
				    const char *buf, size_t size)
{
	struct dbmdx_private *p = dev_get_drvdata(dev);
	unsigned long val;
	int ret;

	if (!p)
		return -EAGAIN;

	if (!p->device_ready) {
		dev_err(p->dev, "%s: device not ready\n", __func__);
		return -EAGAIN;
	}

	ret = kstrtoul(buf, 0, &val);
	if (ret)
		return -EINVAL;

	p->lock(p);
	ret = dbmdx_switch_to_user_selected_chip_interface(p);
	if (ret) {
		p->unlock(p);
		dev_err(p->dev, "%s: Error switching to user sel. chip iface\n",
			__func__);
		return -EIO;
	}

	dbmdx_set_power_mode(p, DBMDX_PM_ACTIVE);
		dev_err(p->dev, "%s: command= %x val = %x\n",
				__func__, command, (int)val);
	ret = dbmdx_send_cmd_32(p, command, val, NULL);
	if (ret < 0) {
		dev_err(p->dev, "%s: set reg error\n", __func__);
		size = ret;
		goto out_unlock;
	}
	dev_dbg(p->dev, "%s: val = 0x%08x\n", __func__, (u32)val);

out_unlock:
	dbmdx_set_power_mode(p, DBMDX_PM_FALLING_ASLEEP);

	ret = dbmdx_switch_to_drv_selected_chip_interface(p);
	if (ret) {
		p->unlock(p);
		dev_err(p->dev, "%s: Error switching to drv sel. chip iface\n",
			__func__);
		return -EIO;
	}

	p->unlock(p);
	return size;
}

static ssize_t dbmdx_fw_ver_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct dbmdx_private *p = dev_get_drvdata(dev);
	ssize_t off = 0;
	int ret;
	if (p->pdata->feature_va && p->active_fw == DBMDX_FW_VA)
		return dbmdx_reg_show(dev,
				DBMDX_REGN_FW_VERSION_NUMBER, attr, buf);
	if (p->pdata->feature_va_ve) {
		enum dbmdx_firmware_active	cur_active_fw;
		cur_active_fw = p->active_fw;
		if (p->va_chip_enabled) {

			if (p->active_interface_type_va != DBMDX_CMD_INTERFACE)
				return snprintf(buf, PAGE_SIZE,
					"VA chip is not in a cmd mode\n");

				ret = dbmdx_switch_to_va_chip_interface(p,
					DBMDX_CMD_INTERFACE);
				if (ret) {
					dev_err(p->dev,
					"%s Error switching to VA interface\n",
					__func__);
					return -EIO;
				}
				off += snprintf(buf + off, PAGE_SIZE - off,
					"%s version information\n",
					dbmdx_fw_type_to_str(p->active_fw));
				off += snprintf(buf + off, PAGE_SIZE - off,
					"===============================\n");
				off += snprintf(buf + off, PAGE_SIZE - off,
					"VA Chip Version: ");

				off += dbmdx_reg_show(dev,
						DBMDX_REGN_FW_VERSION_NUMBER,
						attr, buf + off);
		}
		if (p->va_ve_chip_enabled) {

			if (p->active_interface_type_va_ve !=
							DBMDX_CMD_INTERFACE)
				return snprintf(buf, PAGE_SIZE,
					"VA_VE chip is not in a cmd mode\n");

				ret = dbmdx_switch_to_va_ve_chip_interface(p,
					DBMDX_CMD_INTERFACE);
				if (ret) {
					dev_err(p->dev,
					"%s Error switching to (VA_VE) int.\n",
					__func__);
					return -EIO;
				}

				off += snprintf(buf + off, PAGE_SIZE - off,
					"VA_VE Chip Version:");

				off += dbmdx_reg_show(dev,
						DBMDX_REGN_FW_VERSION_NUMBER,
						attr, buf + off);
		}

		if (p->va_chip_enabled && (cur_active_fw == DBMDX_FW_VA)) {
			ret = dbmdx_switch_to_va_chip_interface(p,
				DBMDX_CMD_INTERFACE);
			if (ret) {
				dev_err(p->dev,
				"%s Error switching to (VA) interface\n",
					__func__);
				return -EIO;
			}

		}

		return off;

	}

	return snprintf(buf, PAGE_SIZE,
		"Unknown firmware (%d) loaded\n", p->active_fw);
}


static ssize_t dbmdx_va_clockcfg_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	return dbmdx_reg_show(dev,
			DBMDX_REGN_DSP_CLOCK_CONFIG, attr, buf);
}

static ssize_t dbmdx_va_clockcfg_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t size)
{
	return dbmdx_reg_store(dev,
				DBMDX_REGN_DSP_CLOCK_CONFIG, attr,
			    buf, size, DBMDX_FW_VA);
}


static ssize_t dbmdx_reboot_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t size)
{
	struct dbmdx_private *p = dev_get_drvdata(dev);
	int va = 0;
	int shutdown = 0;
	int va_resume = 0;
	int va_debug = DBMDX_DEBUG_MODE_OFF;
	int ret = 0;

	if  (!strncmp(buf, "shutdown", min_t(int, size, 8)))
		shutdown = 1;
	else if  (!strncmp(buf, "va_resume", min_t(int, size, 8))) {
		va = 1;
		va_resume = 1;
	} else if  (!strncmp(buf, "va_ve_record", min_t(int, size, 12))) {
		va = 1;
		va_debug = DBMDX_DEBUG_MODE_RECORD;
	} else if  (!strncmp(buf, "va_ve_debug", min_t(int, size, 11))) {
		va = 1;
		va_debug = (DBMDX_DEBUG_MODE_RECORD | DBMDX_DEBUG_MODE_FW_LOG);
	} else if (!strncmp(buf, "va_ve", min_t(int, size, 5))) {
		va = 1;
	} else if  (!strncmp(buf, "va_debug", min_t(int, size, 7))) {
		va = 1;
		va_debug = (DBMDX_DEBUG_MODE_RECORD | DBMDX_DEBUG_MODE_FW_LOG);
	} else if (!strncmp(buf, "va", min_t(int, size, 2)))
		va = 1;
	else if (!strncmp(buf, "help", min_t(int, size, 4))) {
		dev_info(p->dev,
			"%s: Commands: va_ve | va_ve_debug | va_ve_record | help\n",
			__func__);

		return size;
	}

	if (shutdown) {
		dev_info(p->dev, "%s: Shutting down DBMDX...\n", __func__);
		ret = dbmdx_shutdown(p);
		if (ret != 0) {
			dev_err(p->dev, "%s: Error shutting down DBMDX\n",
				__func__);
			return -EIO;
		}
		dev_info(p->dev, "%s: DBMDX was shut down\n", __func__);
		return size;
	}

	if (!va) {
		dev_warn(p->dev, "%s: not valid mode requested: %s\n",
			__func__, buf);
		return size;
	}

	if (va && (!(p->pdata->feature_va) && !(p->pdata->feature_va_ve))) {
		dev_dbg(p->dev, "%s: VA feature not enabled\n", __func__);
		va = 0;
	}

	if (va_resume) {
		if (p->active_fw == DBMDX_FW_POWER_OFF_VA) {
			dev_info(p->dev, "%s: DBMDX Resume Start\n", __func__);
			ret = dbmdx_perform_recovery(p);
			if (ret) {
				dev_err(p->dev, "%s: DBMDX resume failed\n",
					__func__);
				return -EIO;
			}
			dev_info(p->dev, "%s: Resume Done\n", __func__);
			return size;
		}
	}

	dev_info(p->dev, "%s: Reboot Start\n", __func__);


	/* flush pending buffering work if any */
	p->va_flags.buffering = 0;
	flush_work(&p->sv_work);
	p->va_flags.pcm_worker_active = 0;
	flush_work(&p->pcm_streaming_work);

	p->wakeup_release(p);

	p->va_debug_mode = va_debug;

	if (p->pdata->feature_va_ve)
		ret = dbmdx_request_and_load_fw_va_ve_mode(p);
	else
		ret = dbmdx_request_and_load_fw(p, va);

	if (ret != 0)
		return -EIO;

	return size;
}

static ssize_t dbmdx_va_debug_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t size)
{
	struct dbmdx_private *p = dev_get_drvdata(dev);
	int ret = -EINVAL;

	if (!strncmp(buf, "clk_output", min_t(int, size, 10)))
		ret = dbmdx_set_fw_debug_mode(p, FW_DEBUG_OUTPUT_CLK);
	else if (!strncmp(buf, "uart_dbg", min_t(int, size, 8)))
		ret = dbmdx_set_fw_debug_mode(p, FW_DEBUG_OUTPUT_UART);
	else if (!strncmp(buf, "clk_uart_output", min_t(int, size, 15)))
		ret = dbmdx_set_fw_debug_mode(p, FW_DEBUG_OUTPUT_CLK_UART);
	else if (!strncmp(buf, "disable_dbg", min_t(int, size, 11)))
		ret = dbmdx_set_fw_debug_mode(p, FW_DEBUG_OUTPUT_NONE);
	else if (!strncmp(buf, "mic_disable_on", min_t(int, size, 14))) {
		p->mic_disabling_blocked = false;
		ret = 0;
	} else if (!strncmp(buf, "mic_disable_off", min_t(int, size, 15))) {
		p->mic_disabling_blocked = true;
		ret = 0;
	} else if (!strncmp(buf, "pm_suspend", min_t(int, size, 10))) {
		if (p->chip->suspend)
			p->chip->suspend(p);
		ret = 0;
	} else if (!strncmp(buf, "pm_resume", min_t(int, size, 9))) {
		if (p->chip->resume)
			p->chip->resume(p);
		ret = 0;
	} else if (!strncmp(buf, "disable_sleep", min_t(int, size, 13))) {
		p->sleep_disabled = true;
		ret = 0;
	} else if (!strncmp(buf, "enable_sleep", min_t(int, size, 12))) {
		p->sleep_disabled = false;
		ret = 0;
	} else if (!strncmp(buf, "help", min_t(int, size, 4))) {
		dev_info(p->dev,
			"%s: Commands: clk_output | uart_dbg | clk_uart_output | disable_dbg | pm_suspend | pm_resume | disable_sleep | enable_sleep | help\n",
			__func__);
		ret = 0;
	}


	if (ret)
		return ret;

	return size;
}

static ssize_t dbmdx_va_debug_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	int ret;

	ret = snprintf(buf, PAGE_SIZE,
		"Supported Commands: [ clk_output | uart_dbg | clk_uart_output | disable_dbg | pm_suspend | pm_resume | disable_sleep | enable_sleep | mic_disable_on | mic_disable_off | help ]\n");

	return ret;
}

static ssize_t dbmdx_va_ve_debug_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t size)
{
	struct dbmdx_private *p = dev_get_drvdata(dev);
	int ret = -EINVAL;

	if (!strncmp(buf, "mic_override_enable", min_t(int, size, 19))) {
		p->va_ve_mic_mode = 1;
		ret = 0;
	} else if (!strncmp(buf, "mic_override_disable",
					min_t(int, size, 20))) {
		p->va_ve_mic_mode = 0;
		ret = 0;
	} else if (!strncmp(buf, "start_usecase_enable",
					min_t(int, size, 20))) {
		p->start_usecase_disabled = false;
		ret = 0;
	} else if (!strncmp(buf, "start_usecase_disable",
					min_t(int, size, 21))) {
		p->start_usecase_disabled = true;
		ret = 0;
	} else if (!strncmp(buf, "change_to_host_clk_enable_d4",
					min_t(int, size, 28))) {
		p->pdata->change_clock_src_enabled |=
				DBMDX_CHANGE_TO_HOST_CLOCK_D4_ENABLED;
		ret = 0;
	} else if (!strncmp(buf, "change_to_host_clk_disable_d4",
					min_t(int, size, 29))) {
		p->pdata->change_clock_src_enabled &=
				~DBMDX_CHANGE_TO_HOST_CLOCK_D4_ENABLED;
		ret = 0;

	} else if (!strncmp(buf, "change_to_int_clk_enable_d4",
					min_t(int, size, 27))) {
		p->pdata->change_clock_src_enabled |=
				DBMDX_CHANGE_TO_INTERNAL_CLOCK_D4_ENABLED;
		ret = 0;
	} else if (!strncmp(buf, "change_to_int_clk_disable_d4",
					min_t(int, size, 28))) {
		p->pdata->change_clock_src_enabled &=
				~DBMDX_CHANGE_TO_INTERNAL_CLOCK_D4_ENABLED;
		ret = 0;
	} else if (!strncmp(buf, "change_clk_src_enable_d4",
					min_t(int, size, 24))) {
		p->pdata->change_clock_src_enabled |=
				DBMDX_CHANGE_CLOCK_D4_ENABLED;
		ret = 0;
	} else if (!strncmp(buf, "change_clk_src_disable_d4",
					min_t(int, size, 25))) {
		p->pdata->change_clock_src_enabled &=
				~DBMDX_CHANGE_CLOCK_D4_ENABLED;
		ret = 0;
	} else if (!strncmp(buf, "change_to_host_clk_enable_d2",
					min_t(int, size, 28))) {
		p->pdata->change_clock_src_enabled |=
				DBMDX_CHANGE_TO_HOST_CLOCK_D2_ENABLED;
		ret = 0;
	} else if (!strncmp(buf, "change_to_host_clk_disable_d2",
					min_t(int, size, 29))) {
		p->pdata->change_clock_src_enabled &=
				~DBMDX_CHANGE_TO_HOST_CLOCK_D2_ENABLED;
		ret = 0;

	} else if (!strncmp(buf, "change_to_int_clk_enable_d2",
					min_t(int, size, 27))) {
		p->pdata->change_clock_src_enabled |=
				DBMDX_CHANGE_TO_INTERNAL_CLOCK_D2_ENABLED;
		ret = 0;
	} else if (!strncmp(buf, "change_to_int_clk_disable_d2",
					min_t(int, size, 28))) {
		p->pdata->change_clock_src_enabled &=
				~DBMDX_CHANGE_TO_INTERNAL_CLOCK_D2_ENABLED;
		ret = 0;
	} else if (!strncmp(buf, "change_clk_src_enable_d2",
					min_t(int, size, 24))) {
		p->pdata->change_clock_src_enabled |=
				DBMDX_CHANGE_CLOCK_D2_ENABLED;
		ret = 0;
	} else if (!strncmp(buf, "change_clk_src_disable_d2",
					min_t(int, size, 25))) {
		p->pdata->change_clock_src_enabled &=
				~DBMDX_CHANGE_CLOCK_D2_ENABLED;
		ret = 0;
	} else if (!strncmp(buf, "change_clk_src_enable",
					min_t(int, size, 21))) {
		p->pdata->change_clock_src_enabled |=
						DBMDX_CHANGE_CLOCK_ENABLED;
		ret = 0;
	} else if (!strncmp(buf, "change_clk_src_disable",
					min_t(int, size, 22))) {
		p->pdata->change_clock_src_enabled =
						DBMDX_CHANGE_CLOCK_DISABLED;
		ret = 0;
#if defined(DBMDX_PSAD_SUPPORTED)
	} else if (!strncmp(buf, "psad_disable",
					min_t(int, size, 12))) {
		p->psad_disabled = true;
		ret = 0;
	} else if (!strncmp(buf, "psad_enable",
					min_t(int, size, 11))) {
		p->psad_disabled = false;
		ret = 0;
#endif
	} else if (!strncmp(buf, "help", min_t(int, size, 4))) {
		dev_info(p->dev,
			"%s: Commands: mic_override_enable | mic_override_disable | start_usecase_enable | start_usecase_disable | change_to_host_clk_enable_d4 | change_to_host_clk_disable_d4 | change_to_int_clk_enable_d4 | change_to_int_clk_disable_d4 | change_clk_src_enable_d4 | change_clk_src_disable_d4 | change_to_host_clk_enable_d2 | change_to_host_clk_disable_d2 | change_to_int_clk_enable_d2 | change_to_int_clk_disable_d2 | change_clk_src_enable_d2 | change_clk_src_disable_d2 | change_clk_src_enable | change_clk_src_disable | help\n",
			__func__);
		ret = 0;
	}

	if (ret)
		return ret;

	return size;
}

static ssize_t dbmdx_va_ve_debug_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	int off = 0;
	struct dbmdx_private *p = dev_get_drvdata(dev);

	off += snprintf(buf, PAGE_SIZE,
		"Supported Commands: [ mic_override_enable | mic_override_disable | start_usecase_enable | start_usecase_disable | change_to_host_clk_enable_d4 | change_to_host_clk_disable_d4 | change_to_int_clk_enable_d4 | change_to_int_clk_disable_d4 | change_clk_src_enable_d4 | change_clk_src_disable_d4 | change_to_host_clk_enable_d2 | change_to_host_clk_disable_d2 | change_to_int_clk_enable_d2 | change_to_int_clk_disable_d2 | change_clk_src_enable_d2 | change_clk_src_disable_d2 | change_clk_src_enable | change_clk_src_disable | help]\n");

	off += snprintf(buf + off, PAGE_SIZE - off,
					"\t=======Current State======\n");

	off += snprintf(buf + off, PAGE_SIZE - off,
				"\tMic Override Enabled:\t%s\n",
				p->va_ve_mic_mode ? "ON" : "OFF");

	off += snprintf(buf + off, PAGE_SIZE - off,
				"\tStart Usecase Enabled:\t%s\n",
				p->start_usecase_disabled ? "OFF" : "ON");

	off += snprintf(buf + off, PAGE_SIZE - off,
			"\tDBMD4: Switch to the Host Clock:\t%s\n",
				(p->pdata->change_clock_src_enabled &
					DBMDX_CHANGE_TO_HOST_CLOCK_D4_ENABLED) ?
				"ON" : "OFF");
	off += snprintf(buf + off, PAGE_SIZE - off,
		"	\tDBMD4: Switch to the Internal Clock:\t%s\n",
				(p->pdata->change_clock_src_enabled &
				DBMDX_CHANGE_TO_INTERNAL_CLOCK_D4_ENABLED) ?
				"ON" : "OFF");
	off += snprintf(buf + off, PAGE_SIZE - off,
			"\tDBMD2: Switch to the Host Clock:\t%s\n",
				(p->pdata->change_clock_src_enabled &
					DBMDX_CHANGE_TO_HOST_CLOCK_D2_ENABLED) ?
				"ON" : "OFF");
	off += snprintf(buf + off, PAGE_SIZE - off,
			"\tDBMD2: Switch to the Internal Clock:\t%s\n",
				(p->pdata->change_clock_src_enabled &
				DBMDX_CHANGE_TO_INTERNAL_CLOCK_D2_ENABLED) ?
				"ON" : "OFF");
	return off;
}

static ssize_t dbmdx_va_speed_cfg_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t size)
{
	struct dbmdx_private *p = dev_get_drvdata(dev);
	struct dbmdx_platform_data *pdata;
	char *str_p;
	char *args =  (char *)buf;
	unsigned long val;
	u32 index = 0;
	u32 type = 0;
	u32 new_value = 0;
	bool index_set = false, type_set = false, value_set = false;
	int i;

	int ret = -EINVAL;

	if (!p)
		return -EAGAIN;

	pdata = p->pdata;

	while ((str_p = strsep(&args, " \t")) != NULL) {

		if (!*str_p)
			continue;

		if (strncmp(str_p, "index=", 6) == 0) {
			ret = kstrtoul((str_p+6), 0, &val);
			if (ret) {
				dev_err(p->dev, "%s: bad index\n", __func__);
				ret = -EINVAL;
				goto print_usage;
			} else if (val > 2) {
				dev_err(p->dev, "%s: index out of range: %d\n",
					__func__, (int)val);
				ret = -EINVAL;
				goto print_usage;
			}
			index = (u32)val;
			index_set = true;
			continue;
		}
		if (strncmp(str_p, "type=", 5) == 0) {
			if (strncmp(str_p+5, "cfg", 3) == 0)
				type = 0;
			else if (strncmp(str_p+5, "uart", 4) == 0)
				type = 1;
			else if (strncmp(str_p+5, "i2c", 3) == 0)
				type = 2;
			else if (strncmp(str_p+5, "spi", 3) == 0)
				type = 3;
			else {
				dev_err(p->dev, "%s: invalid type\n",
					__func__);
				ret = -EINVAL;
				goto print_usage;
			}
			type_set = true;
			continue;
		}
		if (strncmp(str_p, "value=", 6) == 0) {
			ret = kstrtoul((str_p+6), 0, &val);
			if (ret) {
				dev_err(p->dev, "%s: bad value\n", __func__);
				ret = -EINVAL;
				goto print_usage;
			}

			new_value = (u32)val;
			value_set = true;
			continue;
		}
	}

	if (!index_set) {
		dev_err(p->dev, "%s: index is not set\n", __func__);
		ret = -EINVAL;
		goto print_usage;
	} else if (!type_set) {
		dev_err(p->dev, "%s: type is not set\n", __func__);
		ret = -EINVAL;
		goto print_usage;
	} else if (!value_set) {
		dev_err(p->dev, "%s: value is not set\n", __func__);
		ret = -EINVAL;
		goto print_usage;
	}

	p->lock(p);

	if (type == 0) {
		p->pdata->va_speed_cfg[index].cfg = new_value;
		dev_info(p->dev, "%s: va_speed_cfg[%u].cfg was set to %8.8x\n",
			__func__, index, new_value);
	} else if (type == 1) {
		p->pdata->va_speed_cfg[index].uart_baud = new_value;
		dev_info(p->dev, "%s: va_speed_cfg[%u].uart_baud was set to %u\n",
			__func__, index, new_value);
	} else if (type == 2) {
		p->pdata->va_speed_cfg[index].i2c_rate = new_value;
		dev_info(p->dev, "%s: va_speed_cfg[%u].i2c_rate was set to %u\n",
			__func__, index, new_value);
	} else if (type == 3) {
		p->pdata->va_speed_cfg[index].spi_rate = new_value;
		dev_info(p->dev, "%s: va_speed_cfg[%u].spi_rate was set to %u\n",
			__func__, index, new_value);
	}

	p->unlock(p);

	for (i = 0; i < DBMDX_VA_NR_OF_SPEEDS; i++)
		dev_info(dev, "%s: VA speed cfg %8.8x: 0x%8.8x %u %u %u\n",
			__func__,
			i,
			pdata->va_speed_cfg[i].cfg,
			pdata->va_speed_cfg[i].uart_baud,
			pdata->va_speed_cfg[i].i2c_rate,
			pdata->va_speed_cfg[i].spi_rate);

	return size;
print_usage:
	dev_info(p->dev,
		"%s: Usage: index=[0/1/2] type=[cfg/uart/i2c/spi] value=newval\n",
		__func__);
	return ret;
}

static ssize_t dbmdx_va_speed_cfg_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct dbmdx_private *p = dev_get_drvdata(dev);
	int off = 0;
	struct dbmdx_platform_data *pdata;
	int i;

	if (!p)
		return -EAGAIN;

	pdata = p->pdata;

	for (i = 0; i < DBMDX_VA_NR_OF_SPEEDS; i++)
		off += snprintf(buf + off, PAGE_SIZE - off,
			"\tVA speed cfg %8.8x: 0x%8.8x %u %u %u\n",
			i,
			pdata->va_speed_cfg[i].cfg,
			pdata->va_speed_cfg[i].uart_baud,
			pdata->va_speed_cfg[i].i2c_rate,
			pdata->va_speed_cfg[i].spi_rate);

	return off;
}

static ssize_t dbmdx_va_cfg_values_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t size)
{
	struct dbmdx_private *p = dev_get_drvdata(dev);
	struct dbmdx_platform_data *pdata;
	char *str_p;
	char *args =  (char *)buf;
	unsigned long val;
	u32 index = 0;
	u32 new_value = 0;
	bool index_set = false, value_set = false;
	int i;

	int ret = -EINVAL;

	if (!p)
		return -EAGAIN;

	pdata = p->pdata;

	while ((str_p = strsep(&args, " \t")) != NULL) {

		if (!*str_p)
			continue;

		if (strncmp(str_p, "index=", 6) == 0) {
			ret = kstrtoul((str_p+6), 0, &val);
			if (ret) {
				dev_err(p->dev, "%s: bad index\n", __func__);
				ret = -EINVAL;
				goto print_usage;
			} else if (val >=  pdata->va_cfg_values) {
				dev_err(p->dev, "%s: index out of range: %d\n",
					__func__, (int)val);
				ret = -EINVAL;
				goto print_usage;
			}
			index = (u32)val;
			index_set = true;
			continue;
		}
		if (strncmp(str_p, "value=", 6) == 0) {
			ret = kstrtoul((str_p+6), 0, &val);
			if (ret) {
				dev_err(p->dev, "%s: bad value\n", __func__);
				ret = -EINVAL;
				goto print_usage;
			}

			new_value = (u32)val;
			value_set = true;
			continue;
		}
	}

	if (!index_set) {
		dev_err(p->dev, "%s: index is not set\n", __func__);
		ret = -EINVAL;
		goto print_usage;
	} else if (!value_set) {
		dev_err(p->dev, "%s: value is not set\n", __func__);
		ret = -EINVAL;
		goto print_usage;
	}
	p->lock(p);

	p->pdata->va_cfg_value[index] = new_value;

	dev_info(p->dev, "%s: va_cfg_value[%u] was set to %u\n",
		__func__, index, new_value);

	p->unlock(p);

	for (i = 0; i < pdata->va_cfg_values; i++)
		dev_dbg(dev, "%s: VA cfg %8.8x: 0x%8.8x\n",
			__func__, i, pdata->va_cfg_value[i]);

	return size;
print_usage:
	dev_info(p->dev,
		"%s: Usage: index=[0-%u] value=newval\n",
		__func__, (u32)(pdata->va_cfg_values));
	return ret;
}


static ssize_t dbmdx_va_cfg_values_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct dbmdx_private *p = dev_get_drvdata(dev);
	int off = 0;
	struct dbmdx_platform_data *pdata;
	int i;

	if (!p)
		return -EAGAIN;

	pdata = p->pdata;

	for (i = 0; i < pdata->va_cfg_values; i++)
		off += snprintf(buf + off, PAGE_SIZE - off,
				"\tVA cfg %8.8x: 0x%8.8x\n",
				i, pdata->va_cfg_value[i]);

	return off;
}

static ssize_t dbmdx_va_mic_cfg_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t size)
{
	struct dbmdx_private *p = dev_get_drvdata(dev);
	struct dbmdx_platform_data *pdata;
	char *str_p;
	char *args =  (char *)buf;
	unsigned long val;
	u32 index = 0;
	u32 new_value = 0;
	bool index_set = false, value_set = false;
	int i;

	int ret = -EINVAL;

	if (!p)
		return -EAGAIN;

	pdata = p->pdata;

	while ((str_p = strsep(&args, " \t")) != NULL) {

		if (!*str_p)
			continue;

		if (strncmp(str_p, "index=", 6) == 0) {
			ret = kstrtoul((str_p+6), 0, &val);
			if (ret) {
				dev_err(p->dev, "%s: bad index\n", __func__);
				ret = -EINVAL;
				goto print_usage;
			} else if (val >  (VA_MIC_CONFIG_SIZE - 1)) {
				dev_err(p->dev, "%s: index out of range: %d\n",
					__func__, (int)val);
				ret = -EINVAL;
				goto print_usage;
			}
			index = (u32)val;
			index_set = true;
			continue;
		}
		if (strncmp(str_p, "value=", 6) == 0) {
			ret = kstrtoul((str_p+6), 0, &val);
			if (ret) {
				dev_err(p->dev, "%s: bad value\n", __func__);
				ret = -EINVAL;
				goto print_usage;
			}

			new_value = (u32)val;
			value_set = true;
			continue;
		}
	}

	if (!index_set) {
		dev_err(p->dev, "%s: index is not set\n", __func__);
		ret = -EINVAL;
		goto print_usage;
	} else if (!value_set) {
		dev_err(p->dev, "%s: value is not set\n", __func__);
		ret = -EINVAL;
		goto print_usage;
	}
	p->lock(p);

	p->pdata->va_mic_config[index] = new_value;

	dev_info(p->dev, "%s: va_mic_config[%u] was set to %u\n",
		__func__, index, new_value);

	p->unlock(p);

	for (i = 0; i < VA_MIC_CONFIG_SIZE; i++)
		dev_dbg(dev, "%s: VA mic cfg %8.8x: 0x%8.8x\n",
			__func__, i, pdata->va_mic_config[i]);

	return size;
print_usage:
	dev_info(p->dev,
		"%s: Usage: index=[0-%d] value=newval\n",
		__func__, (VA_MIC_CONFIG_SIZE - 1));
	return ret;
}


static ssize_t dbmdx_va_mic_cfg_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct dbmdx_private *p = dev_get_drvdata(dev);
	int off = 0;
	struct dbmdx_platform_data *pdata;
	int i;

	if (!p)
		return -EAGAIN;

	pdata = p->pdata;

	for (i = 0; i < VA_MIC_CONFIG_SIZE; i++)
		off += snprintf(buf + off, PAGE_SIZE - off,
				"\tVA mic cfg %8.8x: 0x%8.8x\n",
				i, pdata->va_mic_config[i]);

	return off;
}


static ssize_t dbmdx_va_trigger_level_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct dbmdx_private *p = dev_get_drvdata(dev);
	u32 vt_reg;

	if (!p)
		return -EAGAIN;

	if (!p->device_ready) {
		dev_err(p->dev, "%s: device not ready\n", __func__);
		return -EAGAIN;
	}

	if (p->usr_selected_vte == 3)
		vt_reg = DBMDX_REGN_VT3_REGS_OFFSET;
	else if (p->usr_selected_vte == 2)
		vt_reg = DBMDX_REGN_VT2_REGS_OFFSET;
	else
		vt_reg = DBMDX_REGN_VT1_REGS_OFFSET;

	return dbmdx_reg_show(dev, vt_reg |  DBMDX_REGN_VT_REG_TH,
								attr, buf);
}

static ssize_t dbmdx_va_trigger_level_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t size)
{
	struct dbmdx_private *p = dev_get_drvdata(dev);
	u32 vt_reg;

	if (!p)
		return -EAGAIN;

	if (!p->device_ready) {
		dev_err(p->dev, "%s: device not ready\n", __func__);
		return -EAGAIN;
	}

	if (p->usr_selected_vte == 3)
		vt_reg = DBMDX_REGN_VT3_REGS_OFFSET;
	else if (p->usr_selected_vte == 2)
		vt_reg = DBMDX_REGN_VT2_REGS_OFFSET;
	else
		vt_reg = DBMDX_REGN_VT1_REGS_OFFSET;

	return dbmdx_reg_store(dev, vt_reg |  DBMDX_REGN_VT_REG_TH, attr,
			       buf, size, DBMDX_FW_VA);
}

static ssize_t dbmdx_va_verification_level_show(struct device *dev,
					     struct device_attribute *attr,
					     char *buf)
{
	struct dbmdx_private *p = dev_get_drvdata(dev);
	u32 vt_reg;

	if (!p)
		return -EAGAIN;

	if (!p->device_ready) {
		dev_err(p->dev, "%s: device not ready\n", __func__);
		return -EAGAIN;
	}

	if (p->usr_selected_vte == 3)
		vt_reg = DBMDX_REGN_VT3_REGS_OFFSET;
	else if (p->usr_selected_vte == 2)
		vt_reg = DBMDX_REGN_VT2_REGS_OFFSET;
	else
		vt_reg = DBMDX_REGN_VT1_REGS_OFFSET;

	return dbmdx_reg_show(dev, vt_reg |  DBMDX_REGN_VT_REG_LL,
								attr, buf);
}

static ssize_t dbmdx_va_verification_level_store(struct device *dev,
					      struct device_attribute *attr,
					      const char *buf, size_t size)
{
	struct dbmdx_private *p = dev_get_drvdata(dev);
	u32 vt_reg;

	if (!p)
		return -EAGAIN;

	if (!p->device_ready) {
		dev_err(p->dev, "%s: device not ready\n", __func__);
		return -EAGAIN;
	}

	if (p->usr_selected_vte == 3)
		vt_reg = DBMDX_REGN_VT3_REGS_OFFSET;
	else if (p->usr_selected_vte == 2)
		vt_reg = DBMDX_REGN_VT2_REGS_OFFSET;
	else
		vt_reg = DBMDX_REGN_VT1_REGS_OFFSET;

	return dbmdx_reg_store(dev, vt_reg |  DBMDX_REGN_VT_REG_LL,
					attr, buf, size, DBMDX_FW_VA);
}

static ssize_t dbmdx_va_digital_gain_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return dbmdx_reg_show(dev, DBMDX_REGN_DIGITAL_GAIN, attr, buf);
}

static ssize_t dbmdx_va_digital_gain_store(struct device *dev,
					     struct device_attribute *attr,
					     const char *buf, size_t size)
{
	return dbmdx_reg_store(dev, DBMDX_REGN_DIGITAL_GAIN, attr,
			       buf, size, DBMDX_FW_VA);
}

static ssize_t dbmdx_io_addr_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	return dbmdx_reg_show_long(dev,
				  DBMDX_REGN_IO_PORT_ADDR_LO, attr, buf);
}

static ssize_t dbmdx_io_addr_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t size)
{
	return dbmdx_reg_store_long(dev,
				   DBMDX_REGN_IO_PORT_ADDR_LO, attr, buf, size);
}
#ifdef RR_KEEP_OBSOLETE_FUNC
static int dbmdx_set_sv_recognition_mode(struct dbmdx_private *p,
	enum dbmdx_sv_recognition_mode mode)
{

	u16 cur_val = 0;
	if (!p->sv_a_model_support) {
		dev_warn(p->dev, "%s: SV model isn't supported.\n",
			__func__);
		return 0;
	}
	if ((p->va_detection_mode_custom_params !=
		DBMDX_NO_EXT_DETECTION_MODE_PARAMS) &&
		(mode != SV_RECOGNITION_MODE_DISABLED))
		cur_val = p->va_detection_mode_custom_params;
	else if (mode == SV_RECOGNITION_MODE_VOICE_PHRASE_OR_CMD)
		cur_val = 1;
	else if  (mode == SV_RECOGNITION_MODE_VOICE_ENERGY)
		cur_val = 2;

	if (dbmdx_send_cmd(p, DBMDX_REGN_VT_ENGINE_RECOGNITION_MODE | cur_val,
					NULL) < 0) {
		dev_err(p->dev,
		"%s: failed to set DBMDX_REGN_VT_ENGINE_RECOGNITION_MODE to %d\n",
			__func__, cur_val);
		return -EIO;
	}

	p->va_flags.sv_recognition_mode = mode;

	dev_info(p->dev, "%s: SV amodel mode was set to %d\n",
		 __func__, cur_val);
	return 0;
}
#endif

static int dbmdx_check_asrp_params_loaded_correctly(struct dbmdx_private *p)
{
	int ret;
	u16 val;

	if (!p)
		return -EAGAIN;

	if (!p->device_ready) {
		dev_err(p->dev, "%s: device not ready\n", __func__);
		return -EAGAIN;
	}

	dev_dbg(p->dev, "%s:\n", __func__);

	ret = dbmdx_va_alive(p);
	if (ret) {
		dev_err(p->dev, "%s: fw is dead\n", __func__);
		return -EIO;
	}

	ret = dbmdx_send_cmd(p, DBMDX_REGN_ASRP_PARAM_SIZE, &val);
	if (ret < 0) {
		dev_err(p->dev,
			"%s: failed to read ASRP size after loading params\n",
			__func__);
		return -EIO;
	}

	if (val == 0 || val == 0xffff) {
		dev_err(p->dev,
			"%s: Reported ASRP params size is invalid: %d\n",
			__func__, (int)val);
		return -EIO;
	}

	ret = dbmdx_send_cmd(p, DBMDX_REGN_ASRP_CONTROL, &val);
	if (ret < 0) {
		dev_err(p->dev,
			"%s: failed to read ASRP control register\n",
			__func__);
		return -EIO;
	}

	dev_info(p->dev, "%s: ASRP Control: 0x%x\n", __func__, val);

	dev_info(p->dev, "%s: ASRP Params sent successfully\n",
		__func__);

	return ret;
}


static int dbmdx_va_load_asrp_params(struct dbmdx_private *p,
			enum dbmdx_mem_loc model_mem_loc,
			const char *dbmdx_asrp_name)
{
	int ret, ret2;
	struct amodel_info cur_amodel;

	if (!p)
		return -EAGAIN;

	if (!p->device_ready) {
		dev_err(p->dev, "%s: device not ready\n", __func__);
		return -EAGAIN;
	}

	dev_dbg(p->dev, "%s:\n", __func__);

	cur_amodel.amodel_buf = vmalloc(MAX_AMODEL_SIZE);
	if (!(cur_amodel.amodel_buf)) {
		dev_err(p->dev,	"%s: Cannot allocate memory for ASRP model\n",
			__func__);
		return -ENOMEM;
	}

	ret = dbmdx_va_amodel_load_file(p,
					1,
					&dbmdx_asrp_name,
					NULL,
					1,
					DBMDX_AMODEL_INCLUDES_HEADERS,
					cur_amodel.amodel_buf,
					&cur_amodel.amodel_size,
					&cur_amodel.num_of_amodel_chunks,
					cur_amodel.amodel_chunks_size);
	if (ret < 0) {
		dev_err(p->dev, "%s: failed to load ASRP amodel(%d)\n",
			__func__, ret);
		goto out_mem;
	}

	p->device_ready = false;

	/* prepare the chip interface for A-Model loading */
	ret = p->chip->prepare_amodel_loading(p);
	if (ret != 0) {
		dev_err(p->dev, "%s: failed to prepare A-Model loading\n",
			__func__);
		p->device_ready = true;
		goto out_mem;
	}

	ret = dbmdx_va_amodel_send(p,
			cur_amodel.amodel_buf,
			cur_amodel.amodel_size - 4,
			cur_amodel.num_of_amodel_chunks,
			cur_amodel.amodel_chunks_size,
			0,
			model_mem_loc,
			&(cur_amodel.amodel_buf[cur_amodel.amodel_size - 4]),
			4,
			LOAD_AMODEL_ASRP);
	if (ret) {
		dev_err(p->dev, "%s: sending ASRP params failed\n",
			__func__);
		ret = -EIO;
		goto out_finish_loading;
	}

	p->device_ready = true;

	ret = dbmdx_check_asrp_params_loaded_correctly(p);
	if (ret) {
		dev_err(p->dev, "%s: ASRP parames loading error\n", __func__);
		ret = -EIO;
		goto out_finish_loading;
	}

out_finish_loading:
	/* finish A-Model loading */
	ret2 = p->chip->finish_amodel_loading(p);
	if (ret2 != 0)
		dev_err(p->dev, "%s: failed to finish A-Model loading\n",
			__func__);

	p->device_ready = true;
out_mem:
	vfree(cur_amodel.amodel_buf);
	return ret;
}

static int dbmdx_get_vte_from_model_type(struct dbmdx_private *p,
						enum dbmdx_vt_engine_type type,
						struct vt_engine **vte_eng)
{
	int index;

	*vte_eng = NULL;

	if (!p)
		return -EAGAIN;

	if (type == DBMDX_VT_ENGINE_TYPE_UNDEFINED ||
		type >= DBMDX_VT_ENGINE_TYPE_MAX) {
		dev_err(p->dev, "%s: Invalid engine type %d\n", __func__,
			(int)type);
		return -EINVAL;
	}

	for (index = 0; index < DBMDX_MAX_SUPPORTED_VT_ENGINES; index++) {
		if (p->vt_engines[index].eng_type == type) {
			*vte_eng = &(p->vt_engines[index]);
			return 0;
		}
	}

	return -EINVAL;
}

static int check_if_model_loaded(struct dbmdx_private *p,
				struct vt_engine *vte,
				enum dbmdx_model_detection_type det_type,
				bool *is_loaded_to_mem,
				bool *is_loaded_to_fw)
{
	struct amodel_info *model = NULL;
	struct amodel_info *model_sec = NULL;



	*is_loaded_to_mem = false;
	*is_loaded_to_fw = false;

	if (!p)
		return -EAGAIN;

	if (!p->device_ready) {
		dev_err(p->dev, "%s: device not ready\n", __func__);
		return -EAGAIN;
	}

	if (!vte) {
		dev_err(p->dev, "%s: VT Engine was not allocated\n", __func__);
		return -EINVAL;
	}

	if (!vte->enabled) {
		dev_dbg(p->dev, "%s: VT Engine is not enabled\n", __func__);
		return 0;
	}

	switch (det_type) {
	case DETECTION_TYPE_VOICE_COMMAND:
		model = &(vte->vc_model);
		break;
	case DETECTION_TYPE_VOICE_ENERGY:
		model = &(vte->ve_model);
		break;
	case DETECTION_TYPE_PHRASE:
		model = &(vte->primary_model);
		break;
	case DETECTION_TYPE_DUAL:
		model = &(vte->primary_model);
		model_sec = &(vte->secondary_model);
	default:
		dev_err(p->dev,
			"%s: Unsupported detection mode:%d",
						__func__, det_type);
		return -EINVAL;
	}
	if (det_type != DETECTION_TYPE_DUAL) {
		if (model->amodel_loaded)
			*is_loaded_to_mem = true;
	} else {
		if (model->amodel_loaded && model_sec->amodel_loaded)
			*is_loaded_to_mem = true;
	}

	if (*is_loaded_to_mem && (vte->detection_type == det_type) &&
		((vte->internal_state == MODEL_STATE_LOADED) ||
			(vte->internal_state == MODEL_STATE_STARTED)))
		*is_loaded_to_fw = true;

	dev_dbg(p->dev,
		"%s: Model Type %s, Det Type %s, Loaded To Mem: %d, To Fw: %d",
		__func__, dbmdx_vte_names[vte->eng_type],
		dbmdx_vte_det_type_names[det_type],
		(int)(*is_loaded_to_mem), (int)(*is_loaded_to_fw));

	return 0;
}

static int check_if_model_started(struct dbmdx_private *p,
				struct vt_engine *vte,
				enum dbmdx_model_detection_type det_type,
				bool *is_started)
{
	struct amodel_info *model = NULL;
	struct amodel_info *model_sec = NULL;
	bool is_loaded_to_mem = false;

	*is_started = false;

	if (!p)
		return -EAGAIN;

	if (!p->device_ready) {
		dev_err(p->dev, "%s: device not ready\n", __func__);
		return -EAGAIN;
	}

	if (!vte) {
		dev_err(p->dev, "%s: VT Engine was not allocated\n", __func__);
		return -EINVAL;
	}

	if (!vte->enabled) {
		dev_dbg(p->dev, "%s: VT Engine is not enabled\n", __func__);
		return 0;
	}

	switch (det_type) {
	case DETECTION_TYPE_VOICE_COMMAND:
		model = &(vte->vc_model);
		break;
	case DETECTION_TYPE_VOICE_ENERGY:
		model = &(vte->ve_model);
		break;
	case DETECTION_TYPE_PHRASE:
		model = &(vte->primary_model);
		break;
	case DETECTION_TYPE_DUAL:
		model = &(vte->primary_model);
		model_sec = &(vte->secondary_model);
	default:
		dev_err(p->dev,
			"%s: Unsupported detection mode:%d",
						__func__, det_type);
		return -EINVAL;
	}

	if (det_type != DETECTION_TYPE_DUAL) {
		if (model->amodel_loaded)
			is_loaded_to_mem = true;
	} else {
		if (model->amodel_loaded && model_sec->amodel_loaded)
			is_loaded_to_mem = true;
	}

	if (is_loaded_to_mem && (vte->detection_type == det_type) &&
		(vte->internal_state == MODEL_STATE_STARTED))
		*is_started = true;

	dev_dbg(p->dev,
		"%s: Model Type %s, Det Type %s, Started: %d\n",
		__func__, dbmdx_vte_names[vte->eng_type],
		dbmdx_vte_det_type_names[det_type], (int)(*is_started));

	return 0;
}


static int dbmdx_va_enable_vte_detection(struct dbmdx_private *p,
				struct vt_engine *vte,
				enum dbmdx_model_detection_type det_type,
				u16 model_custom_params,
				bool *uc_reload_required)
{
	int ret = 0;
	u16 vt_num;
	bool is_loaded_to_mem = false;
	bool is_loaded_to_fw = false;
	struct usecase_config *cur_usecase;

	*uc_reload_required = false;

	if (!p)
		return -EAGAIN;

	if (!p->device_ready) {
		dev_err(p->dev, "%s: device not ready\n", __func__);
		return -EAGAIN;
	}

	if (!vte) {
		dev_err(p->dev, "%s: VT Engine was not allocated\n", __func__);
		return -EINVAL;
	}

	if (!vte->enabled) {
		dev_err(p->dev, "%s: VT Engine is not enabled\n", __func__);
		return -EINVAL;
	}

	ret = check_if_model_loaded(p, vte, det_type, &is_loaded_to_mem,
					&is_loaded_to_fw);

	if (ret < 0) {
		dev_err(p->dev,
			"%s: Error checking if model is loaded for %s\n",
			__func__, dbmdx_vte_names[vte->eng_type]);
		return -EIO;
	}

	if (!is_loaded_to_mem) {
		dev_err(p->dev, "%s: Model is not loaded for %s\n", __func__,
			dbmdx_vte_names[vte->eng_type]);
		return -EINVAL;

	}

	vte->user_state = MODEL_STATE_STARTED;
	vte->user_detection_type = det_type;

	/* If IDLE usecase is active we only mark the user request */
	if (p->va_ve_flags.idle_usecase_active ||
				!(p->va_ve_flags.cur_usecase)) {
		dev_dbg(p->dev,	"%s: Detection request registered for : %s\n",
			__func__, dbmdx_vte_names[vte->eng_type]);
		return 0;
	}
	cur_usecase = p->va_ve_flags.cur_usecase;
	if (!(cur_usecase->usecase_requires_amodel)) {
		dev_dbg(p->dev,
			"%s: Detection request registered for : %s\n",
			__func__, dbmdx_vte_names[vte->eng_type]);
		return 0;
	}

	if (!is_loaded_to_fw) {
		dev_dbg(p->dev,
			"%s: Model is not loaded to FW, UC reload is required for : %s\n",
			__func__, dbmdx_vte_names[vte->eng_type]);
		*uc_reload_required = true;
		return 0;
	}

	if (vte->vte_index == 1)
		vt_num = DBMDX_REGV_SELECT_VT_EN_1;
	else if (vte->vte_index == 2)
		vt_num = DBMDX_REGV_SELECT_VT_EN_2;
	else
		vt_num = DBMDX_REGV_SELECT_VT_EN_3;

	ret = dbmdx_send_cmd(p,
				DBMDX_REGN_VT_GENERAL_CONFIG |
				vt_num |
				DBMDX_REGV_VT_EN_EN |
				DBMDX_REGV_VT_ACTIVE,
				NULL);

	if (ret < 0) {
		dev_err(p->dev, "%s: failed to enable engine for %s\n",
				__func__,
				dbmdx_vte_names[vte->eng_type]);
		return -EIO;
	}

	ret = dbmdx_send_cmd(p,
				DBMDX_REGN_VT_AUDIO_HISTORY_CFG |
				vt_num |
				*(vte->history),
				NULL);
	if (ret < 0) {
		dev_err(p->dev, "%s: Error setting history for %s\n",
			__func__, dbmdx_vte_names[vte->eng_type]);
		return -EIO;
	}

	vte->internal_state = MODEL_STATE_STARTED;


	dev_info(p->dev, "%s: Detection enabled for: %s\n", __func__,
				dbmdx_vte_names[vte->eng_type]);

	return 0;
}

static int dbmdx_va_disable_vte_detection(struct dbmdx_private *p,
				struct vt_engine *vte,
				enum dbmdx_model_detection_type det_type)
{
	int ret = 0;
	u16 vt_num;
	bool is_loaded_to_mem = false;
	bool is_loaded_to_fw = false;
	bool is_started = false;

	if (!p)
		return -EAGAIN;

	if (!p->device_ready) {
		dev_err(p->dev, "%s: device not ready\n", __func__);
		return -EAGAIN;
	}

	if (!vte) {
		dev_err(p->dev, "%s: VT Engine was not allocated\n", __func__);
		return -EINVAL;
	}

	if (!vte->enabled) {
		dev_err(p->dev, "%s: VT Engine is not enabled\n", __func__);
		return -EINVAL;
	}

	ret = check_if_model_loaded(p, vte, det_type, &is_loaded_to_mem,
					&is_loaded_to_fw);

	if (ret < 0) {
		dev_err(p->dev,
			"%s: Error checking if model is loaded for %s\n",
			__func__, dbmdx_vte_names[vte->eng_type]);
		return -EIO;
	}

	if (!is_loaded_to_mem) {
		dev_dbg(p->dev, "%s: Model is not loaded for %s\n",
			__func__, dbmdx_vte_names[vte->eng_type]);
		return 0;

	}

	ret = check_if_model_started(p, vte, det_type, &is_started);

	if (ret < 0) {
		dev_dbg(p->dev,
			"%s: Error checking if model is started for %s\n",
			__func__, dbmdx_vte_names[vte->eng_type]);
		return -EIO;
	}

	if (!is_started) {
		dev_dbg(p->dev, "%s: Model is not started for %s\n",
			__func__, dbmdx_vte_names[vte->eng_type]);
		vte->user_state = MODEL_STATE_LOADED;
		return 0;

	}

	vte->user_state = MODEL_STATE_LOADED;

	/* If IDLE usecase is active we only mark the user request */
	if (p->va_ve_flags.idle_usecase_active) {
		dev_dbg(p->dev,	"%s: Stop request registered for : %s\n",
			__func__, dbmdx_vte_names[vte->eng_type]);
		return 0;
	}

	if (vte->vte_index == 1)
		vt_num = DBMDX_REGV_SELECT_VT_EN_1;
	else if (vte->vte_index == 2)
		vt_num = DBMDX_REGV_SELECT_VT_EN_2;
	else
		vt_num = DBMDX_REGV_SELECT_VT_EN_3;

	ret = dbmdx_send_cmd(p,
				DBMDX_REGN_VT_GENERAL_CONFIG |
				vt_num |
				DBMDX_REGV_VT_EN_EN |
				DBMDX_REGV_VT_PAUSE,
				NULL);

	if (ret < 0) {
		dev_err(p->dev, "%s: failed to pause engine %s\n",
				__func__,
				dbmdx_vte_names[vte->eng_type]);
		return -EIO;
	}

	vte->internal_state = MODEL_STATE_LOADED;

	dev_info(p->dev, "%s: Detection disabled for: %s\n", __func__,
				dbmdx_vte_names[vte->eng_type]);


	return 0;
}

int uc_load_models_general(struct dbmdx_private *p,
					struct usecase_config *uc)
{
	int ret = 0;
	int i;
	struct vt_engine *vte;

	if (!p)
		return -EAGAIN;

	if (!p->device_ready) {
		dev_err(p->dev, "%s: device not ready\n", __func__);
		return -EAGAIN;
	}

	for (i = 0; i < DBMDX_MAX_SUPPORTED_VT_ENGINES; i++) {
		vte = &(p->vt_engines[i]);
		if (vte->enabled &&
			(vte->user_state != MODEL_STATE_UNLOADED)) {
			/* Model should be already loaded to memory */
			vte->mem_loc = DBMDX_MEM_LOC_DCTM;
			ret = dbmdx_va_load_amodel(p,
					vte,
					vte->user_detection_type,
					(DBMDX_LOAD_MODEL_FROM_MEMORY |
						DBMDX_DO_NOT_RELOAD_MODEL),
					0);
			if (ret < 0) {
				dev_err(p->dev,
					"%s: Error loading VT Engine model\n",
					__func__);
				return -EIO;
			}
			dev_info(p->dev,
				"%s: VTE (%s) model was successfully loaded\n",
				__func__,
				dbmdx_vte_names[vte->eng_type]);
			vte->internal_state = MODEL_STATE_LOADED;
		}
	}

	return 0;

}

int uc_load_models_unload_all(struct dbmdx_private *p,
					struct usecase_config *uc)
{
#ifdef DBMDX_UNLOAD_MODELS_ON_ENTER
	int ret = 0;
	int i;
	struct vt_engine *vte;

	if (!p)
		return -EAGAIN;

	if (!p->device_ready) {
		dev_err(p->dev, "%s: device not ready\n", __func__);
		return -EAGAIN;
	}

	for (i = 0; i < DBMDX_MAX_SUPPORTED_VT_ENGINES; i++) {
		vte = &(p->vt_engines[i]);
		if (vte->enabled &&
			((vte->internal_state == MODEL_STATE_LOADED) ||
				(vte->internal_state == MODEL_STATE_STARTED))) {

			ret = dbmdx_va_unload_amodel(p, vte,
						vte->detection_type);

			if (ret < 0) {
				dev_err(p->dev,
					"%s: Error unloading VT Engine model\n",
					__func__);
				return -EIO;
			}
			dev_info(p->dev,
				"%s: VTE:(%s) model was successfully unloaded\n",
				__func__,
				dbmdx_vte_names[vte->eng_type]);
			vte->internal_state = MODEL_STATE_UNLOADED;
		}
	}
#endif
	return 0;

}
int uc_stop_all_models(struct dbmdx_private *p,
					struct usecase_config *uc)
{
	int ret = 0;
	int i;
	struct vt_engine *vte;

	if (!p)
		return -EAGAIN;

	if (!p->device_ready) {
		dev_err(p->dev, "%s: device not ready\n", __func__);
		return -EAGAIN;
	}
	ret = dbmdx_send_cmd(p,	(DBMDX_REGN_VT_GENERAL_CONFIG |
					DBMDX_REGV_SELECT_VT_EN_GLOBAL |
					DBMDX_REGV_VT_EN_DIS), NULL);
	if (ret < 0) {
		dev_err(p->dev,	"%s:Error disabling VTE\n", __func__);
		return -EIO;
	}
	msleep(DBMDX_MSLEEP_DISABLE_VTE);

	for (i = 0; i < DBMDX_MAX_SUPPORTED_VT_ENGINES; i++) {
		vte = &(p->vt_engines[i]);
		if (vte->enabled &&
			(vte->internal_state == MODEL_STATE_STARTED))
			vte->internal_state = MODEL_STATE_LOADED;
	}

	return 0;

}

static int uc_start_loaded_models_general(struct dbmdx_private *p,
						struct usecase_config *uc)
{
	struct vt_engine *vte;
	int ret = 0;
	u16 vt_num;
	int i;

	if (!p)
		return -EAGAIN;

	if (!p->device_ready) {
		dev_err(p->dev, "%s: device not ready\n", __func__);
		return -EAGAIN;
	}

	for (i = 0; i < DBMDX_MAX_SUPPORTED_VT_ENGINES; i++) {
		vte = &(p->vt_engines[i]);
		if (vte->vte_index == 1)
			vt_num = DBMDX_REGV_SELECT_VT_EN_1;
		else if (p->vt_engines[i].vte_index == 2)
			vt_num = DBMDX_REGV_SELECT_VT_EN_2;
		else
			vt_num = DBMDX_REGV_SELECT_VT_EN_3;

		if (vte->enabled &&
			(vte->user_state == MODEL_STATE_STARTED) &&
			(vte->internal_state ==	MODEL_STATE_LOADED)) {
			ret = dbmdx_send_cmd(p,
					DBMDX_REGN_VT_GENERAL_CONFIG |
					vt_num |
					DBMDX_REGV_VT_EN_EN |
					DBMDX_REGV_VT_ACTIVE |
					DBMDX_REGV_SET_IN_CP |
					DBMDX_REGV_IN_CP_NUM_0,
					NULL);
			if (ret < 0) {
				dev_err(p->dev,
					"%s: Error enabling detection for %s\n",
					__func__,
					dbmdx_vte_names[vte->eng_type]);
				return -EIO;
			}
			dev_info(p->dev,
				"%s: Detecton for VTE (%s) was enabled\n",
				__func__,
				dbmdx_vte_names[vte->eng_type]);
			vte->internal_state = MODEL_STATE_STARTED;

			msleep(DBMDX_MSLEEP_ENABLE_VTE_DETECTION);

			ret = dbmdx_send_cmd(p,
					DBMDX_REGN_VT_AUDIO_HISTORY_CFG |
					vt_num |
					*(vte->history),
					NULL);
			if (ret < 0) {
				dev_err(p->dev,
					"%s: Error setting history for %s\n",
					__func__,
					dbmdx_vte_names[vte->eng_type]);
				return -EIO;
			}

		} else if (vte->enabled &&
			(vte->user_state == MODEL_STATE_LOADED) &&
			(vte->internal_state ==	MODEL_STATE_LOADED)) {
			ret = dbmdx_send_cmd(p,
					DBMDX_REGN_VT_GENERAL_CONFIG |
					vt_num |
					DBMDX_REGV_VT_EN_EN |
					DBMDX_REGV_VT_PAUSE |
					DBMDX_REGV_SET_IN_CP |
					DBMDX_REGV_IN_CP_NUM_0,
					NULL);
			if (ret < 0) {
				dev_err(p->dev,
					"%s: Error enabling detection for %s\n",
					__func__,
					dbmdx_vte_names[vte->eng_type]);
				return -EIO;
			}
			dev_info(p->dev,
				"%s: Detecton for VTE (%s) was enabled\n",
				__func__,
				dbmdx_vte_names[vte->eng_type]);
			vte->internal_state = MODEL_STATE_STARTED;

			msleep(DBMDX_MSLEEP_ENABLE_VTE_DETECTION);

			ret = dbmdx_send_cmd(p,
					DBMDX_REGN_VT_AUDIO_HISTORY_CFG |
					vt_num |
					*(vte->history),
					NULL);
			if (ret < 0) {
				dev_err(p->dev,
					"%s: Error setting history for %s\n",
					__func__,
					dbmdx_vte_names[vte->eng_type]);
				return -EIO;
			}

		} else {
			if (vte->enabled) {
				ret = dbmdx_send_cmd(p,
						DBMDX_REGN_VT_GENERAL_CONFIG |
						vt_num |
						DBMDX_REGV_VT_EN_DIS,
						NULL);
				if (ret < 0) {
					dev_err(p->dev,
					"%s: Error disabling detection for %s\n",
						__func__,
						dbmdx_vte_names[vte->eng_type]);
					return -EIO;
				}

				dev_info(p->dev,
				"	%s: Detecton for VTE (%s) was disabled\n",
					__func__,
					dbmdx_vte_names[vte->eng_type]);
			}
		}
	}

	return 0;

}
int melon_uc_lp_model_config(struct dbmdx_private *p,
				struct usecase_config *uc)
{
	return uc_start_loaded_models_general(p, uc);
}

int melon_uc_nr_model_config(struct dbmdx_private *p,
				struct usecase_config *uc)
{
	return uc_start_loaded_models_general(p, uc);
}


int melon_uc_barge_in_model_config(struct dbmdx_private *p,
				struct usecase_config *uc)
{
#ifdef DBMDX_MELON_BARGEIN_GA_MODE
	return 0;
#else
	return uc_start_loaded_models_general(p, uc);
#endif
}


int melon_uc_barge_in_load_models(struct dbmdx_private *p,
				struct usecase_config *uc)
{
#ifdef DBMDX_MELON_BARGEIN_GA_MODE
	return 0;
#else
	return uc_load_models_general(p, uc);
#endif

}


static int dbmdx_va_amodel_load_file(struct dbmdx_private *p,
			int num_of_amodel_files,
			const char **amodel_fnames,
			const char *amodel_prefix,
			u32 gram_addr,
			u32 amodel_options,
			char	*amodel_buf,
			ssize_t *amodel_size,
			int *num_of_amodel_chunks,
			ssize_t *amodel_chunks_size)
{
	int ret;
	struct firmware	*amodel_chunk_fw[DBMDX_AMODEL_MAX_CHUNKS];
	const u8 *files_data[DBMDX_AMODEL_MAX_CHUNKS];
	ssize_t amodel_files_size[DBMDX_AMODEL_MAX_CHUNKS];
	int chunk_idx;
	const char *cur_chunk_name;
	char chunk_name_buf[NAME_MAX + 1] = {0};

	if (!p)
		return -EAGAIN;
	for (chunk_idx = 0; chunk_idx < num_of_amodel_files; chunk_idx++)
		amodel_chunk_fw[chunk_idx] = NULL;

	for (chunk_idx = 0; chunk_idx < num_of_amodel_files; chunk_idx++) {

		if (amodel_prefix == NULL) {
			cur_chunk_name = amodel_fnames[chunk_idx];
		} else {
			if (strlen(amodel_fnames[chunk_idx]) >
				(NAME_MAX - strlen(amodel_prefix))) {
				dev_err(p->dev,
					"%s: Name length out of bounds\n",
					__func__);
				ret = -EINVAL;
				goto release;

			}
			strncpy(chunk_name_buf, amodel_prefix, NAME_MAX);
			strncat(chunk_name_buf, amodel_fnames[chunk_idx],
				NAME_MAX - strlen(amodel_prefix));
			cur_chunk_name = chunk_name_buf;
		}

		dev_dbg(p->dev, "%s: loading %s\n", __func__, cur_chunk_name);

		ret = request_firmware(
			(const struct firmware **)(&amodel_chunk_fw[chunk_idx]),
			cur_chunk_name,
			p->dbmdx_dev);

		if (ret < 0) {
			dev_err(p->dev,
				"%s: failed to request Amodel firmware(%d)\n",
				__func__, ret);
			ret = -ENOENT;
			goto release;
		}

		dev_dbg(p->dev, "%s: %s firmware successfully requested\n",
			__func__, amodel_fnames[chunk_idx]);

		dev_dbg(p->dev, "%s FW size=%zu bytes\n",
			__func__, amodel_chunk_fw[chunk_idx]->size);

		if (!amodel_chunk_fw[chunk_idx]->size) {
			dev_warn(p->dev, "%s FW size is 0. Ignore...\n",
				__func__);
			ret = -ENOENT;
			goto release;
		}
		files_data[chunk_idx] = amodel_chunk_fw[chunk_idx]->data;
		amodel_files_size[chunk_idx] =
				(ssize_t)amodel_chunk_fw[chunk_idx]->size;
	}

	ret = dbmdx_acoustic_model_build(p,
					num_of_amodel_files,
					files_data,
					amodel_files_size,
					gram_addr,
					amodel_options,
					amodel_buf,
					amodel_size,
					num_of_amodel_chunks,
					amodel_chunks_size);

	if (ret <= 0) {
		dev_err(p->dev,	"%s: amodel build failed: %d\n",
			__func__, ret);
		ret = -EIO;
		goto release;
	}

release:
	for (chunk_idx = 0; chunk_idx < num_of_amodel_files; chunk_idx++) {
		if (amodel_chunk_fw[chunk_idx])
			release_firmware(amodel_chunk_fw[chunk_idx]);
	}

	return ret;
}

#define DUMMY_MODEL_DATA_SIZE	4
static int dbmdx_va_amodel_load_dummy_model(struct dbmdx_private *p,
			u32 gram_addr,
			char	*amodel_buf,
			ssize_t *amodel_size,
			int *num_of_amodel_chunks,
			ssize_t *amodel_chunks_size)
{
	unsigned char head[DBMDX_AMODEL_HEADER_SIZE] = { 0 };
	unsigned char dummy_data[DUMMY_MODEL_DATA_SIZE] = { 0 };
	size_t target_pos = 0;
	unsigned long checksum;
	int ret = 0;
	ssize_t head_size = DBMDX_AMODEL_HEADER_SIZE;
	size_t gram_size = DUMMY_MODEL_DATA_SIZE;

	dev_dbg(p->dev, "%s\n", __func__);

	*num_of_amodel_chunks = 1;

	head[0] = 0x0;
	head[1] = 0x0;
	head[2] = 0x5A;
	head[3] = 0x02;
	head[4] =  (gram_size/2)        & 0xff;
	head[5] = ((gram_size/2) >>  8) & 0xff;
	head[6] = ((gram_size/2) >> 16) & 0xff;
	head[7] = ((gram_size/2) >> 24) & 0xff;
	head[8] = (gram_addr)        & 0xff;
	head[9] = ((gram_addr) >>  8) & 0xff;
	head[10] = ((gram_addr) >> 16) & 0xff;
	head[11] = ((gram_addr) >> 24) & 0xff;

	memcpy(amodel_buf + target_pos, head, head_size);

	target_pos += head_size;

	memcpy(amodel_buf + target_pos, dummy_data, gram_size);

	target_pos += gram_size;

	amodel_chunks_size[0] = gram_size;

	ret = dbmdx_calc_amodel_checksum(p,
					(char *)amodel_buf,
					target_pos,
					&checksum);
	if (ret) {
		dev_err(p->dev, "%s: failed to calculate Amodel checksum\n",
			__func__);
		ret = -EINVAL;
		goto out;
	}

	*(unsigned long *)(amodel_buf + target_pos) = checksum;

	*amodel_size = (ssize_t)(target_pos + 4);

	ret = *amodel_size;

out:
	return ret;
}

static int dbmdx_va_amodel_send(struct dbmdx_private *p,  const void *data,
			   size_t size, int num_of_chunks, size_t *chunk_sizes,
			   u16 model_vt_num,
			   enum dbmdx_mem_loc model_mem_loc,
			   const void *checksum, size_t chksum_len,
			   enum dbmdx_load_amodel_mode model_type)
{
	int retry = RETRY_COUNT;
	int ret;
	ssize_t send_bytes;
	size_t cur_pos;
	size_t cur_size;
	bool fw_in_boot_mode = false;
	int chunk_ind;
	u16 mem_loc;
	u16 model_ftype;
	u16 blk_start;
	u16 vt_num;

	if (model_mem_loc == DBMDX_MEM_LOC_DCTM)
		mem_loc = DBMDX_REGV_LOAD_MODEL_TO_DTCM;
	else
		mem_loc = DBMDX_REGV_LOAD_MODEL_TO_AHB;

	if (model_type == LOAD_AMODEL_ASRP) {
		vt_num = DBMDX_REGV_LOAD_ENGINE_TYPE_ASRP;
		blk_start = DBMDX_REGV_BLK_START_NUM_0;
		model_ftype = LOAD_AMODEL_ASRP;
	} else {
		blk_start = DBMDX_REGV_BLK_START_NUM_1;

		if (model_vt_num == 1)
			vt_num = DBMDX_REGV_LOAD_ENGINE_TYPE_VT1;
		else if (model_vt_num == 2)
			vt_num = DBMDX_REGV_LOAD_ENGINE_TYPE_VT2;
		else
			vt_num = DBMDX_REGV_LOAD_ENGINE_TYPE_VT3;

		if (model_type == LOAD_AMODEL_PRIMARY)
			model_ftype = DBMDX_REGV_LOAD_PRIMARY_VT_MODEL;
		else if (model_type == LOAD_AMODEL_2NDARY)
			model_ftype = DBMDX_REGV_LOAD_SECONDARY_VT_MODEL;
		else if (model_type == LOAD_AMODEL_ASRP_2ND_INIT)
			model_ftype = DBMDX_REGV_ASRP_2ND_INIT;

	}

	if (!p)
		return -EAGAIN;

	dev_dbg(p->dev, "%s Model FType: %u, VT Idx: %u, Mem, Mem Loc: %u\n",
			__func__, model_type, model_vt_num, model_mem_loc);

	while (retry--) {

		if (fw_in_boot_mode) {
			/* send boot command */
			ret =  p->chip->send_cmd_boot(p, DBMDX_FIRMWARE_BOOT);
			if (ret < 0) {
				dev_err(p->dev,
					"%s: booting the firmware failed\n",
					__func__);
				continue;
			}
			fw_in_boot_mode = false;
		}

		ret = dbmdx_send_cmd(
				p,
				DBMDX_REGN_LOAD_NEW_ACOUSTIC_MODEL |
				vt_num |
				DBMDX_REGV_OP_TYPE_LOAD_FILE |
				DBMDX_REGV_INIT_KILL_SET |
				blk_start |
				mem_loc |
				model_ftype,
				NULL);

		if (ret < 0) {
			dev_err(p->dev,
				"%s: failed to set fw to receive new amodel\n",
				__func__);
			continue;
		}

		fw_in_boot_mode = true;

		dev_info(p->dev,
			"%s: ---------> acoustic model download start\n",
			__func__);

		cur_pos = 0;
		ret = 0;

		p->wakeup_toggle(p);

		for (chunk_ind = 0; chunk_ind < num_of_chunks; chunk_ind++) {
			dev_info(p->dev,
				"%s: Sending amodel chunk %d (%d bytes)\n",
			__func__, chunk_ind, (int)chunk_sizes[chunk_ind]);

			if (chunk_sizes[chunk_ind] == 0)
				continue;

			cur_size = DBMDX_AMODEL_HEADER_SIZE;

			/* Send Gram Header */
			send_bytes = p->chip->write(p, data + cur_pos,
				cur_size);

			if (send_bytes != cur_size) {
				dev_err(p->dev,
				"%s: sending of acoustic model data failed\n",
					__func__);
				ret = -1;
				break;
			}

			/* wait for FW to process the header */
			usleep_range(DBMDX_USLEEP_AMODEL_HEADER,
				DBMDX_USLEEP_AMODEL_HEADER + 1000);

			cur_pos += DBMDX_AMODEL_HEADER_SIZE;
			cur_size = chunk_sizes[chunk_ind];

			/* Send Gram Data */
			send_bytes = p->chip->write(p,
						data + cur_pos, cur_size);

			if (send_bytes != cur_size) {
				dev_err(p->dev,
				"%s: sending of acoustic model data failed\n",
					__func__);
				ret = -1;
				break;
			}

			cur_pos += cur_size;
		}

		/* Check if error occurred during acoustic model transfer */
		if (ret < 0)
			continue;

		/* verify checksum */
		if (checksum) {
			ret = p->chip->verify_boot_checksum(p, checksum,
				chksum_len);
			if (ret < 0) {
				dev_err(p->dev,
					"%s: could not verify checksum\n",
					__func__);
				continue;
			}
		}
		break;
	}

	if (fw_in_boot_mode) {
		/* send boot command */
		ret =  p->chip->send_cmd_boot(p, DBMDX_FIRMWARE_BOOT);
		if (ret < 0) {
			dev_err(p->dev,
				"%s: booting the firmware failed\n", __func__);
			return -EIO;
		}
	}

	/* no retries left, failed to load acoustic */
	if (retry < 0) {
		dev_err(p->dev, "%s: failed to load acoustic model\n",
			__func__);
		return -EIO;
	}

	/* wait some time */
	usleep_range(DBMDX_USLEEP_AFTER_LOAD_AMODEL,
		DBMDX_USLEEP_AFTER_LOAD_AMODEL + 1000);

	return 0;
}

static int dbmdx_model_loaded_correctly(struct dbmdx_private *p, u16 vt_index)
{
	int ret;
	u16 val;
	u32 vt_reg;

	if (!p)
		return -EAGAIN;

	dev_dbg(p->dev, "%s: index=%u\n", __func__, vt_index);

	if (vt_index == 1) {
		vt_reg = DBMDX_REGN_VT1_REGS_OFFSET;
	} else if (vt_index == 2) {
		vt_reg = DBMDX_REGN_VT2_REGS_OFFSET;
	} else if (vt_index == 3) {
		vt_reg = DBMDX_REGN_VT3_REGS_OFFSET;
	} else {
		dev_err(p->dev,	"%s: Unknown VT Index\n", __func__);
		return  -EINVAL;
	}

	ret = dbmdx_va_alive(p);
	if (ret) {
		dev_err(p->dev, "%s: fw is dead\n", __func__);
		return -EIO;
	}

	ret = dbmdx_send_cmd(p, vt_reg | DBMDX_REGN_VT_REG_INIT, &val);
	if (ret < 0) {
		dev_err(p->dev, "%s: Error reading status\n", __func__);
		return -EIO;
	}

	if (val != 1) {
		dev_err(p->dev,
			"%s: Error reported by FW after loading amodel\n",
			__func__);
		return -EIO;
	}

	dev_info(p->dev, "%s: Acoustic model loaded successfully to ind: %u\n",
		__func__, vt_index);

	return 0;
}


static int dbmdx_va_amodel_load_single(struct dbmdx_private *p,
				struct vt_engine *vte,
				enum dbmdx_model_detection_type det_type,
				const char *amodel_prefix,
				bool to_load_from_memory,
				bool do_not_reload_model,
				bool send_to_fw)
{
	int ret, ret2;
	struct amodel_info *model = NULL;
	u32 cur_model_checksum;

	if (!p)
		return -EAGAIN;

	if (!p->device_ready) {
		dev_err(p->dev, "%s: device not ready\n", __func__);
		return -EAGAIN;
	}

	if (!vte) {
		dev_err(p->dev, "%s: VT Engine was not allocated\n", __func__);
		return -EINVAL;
	}

	switch (det_type) {
	case DETECTION_TYPE_VOICE_COMMAND:
		model = &(vte->vc_model);
		break;
	case DETECTION_TYPE_VOICE_ENERGY:
		model = &(vte->ve_model);
		break;
	case DETECTION_TYPE_PHRASE:
		model = &(vte->primary_model);
		break;
	default:
		dev_err(p->dev,
			"%s: Unsupported detection mode:%d",
						__func__, det_type);
		return -EINVAL;
	}


	if (!model) {
		dev_err(p->dev, "%s: No model allocated\n", __func__);
		return -EINVAL;
	}

	if (to_load_from_memory && !(model->amodel_loaded)) {

		dev_err(p->dev, "%s: Model was not loaded to memory\n",
			__func__);
		return -EAGAIN;
	}

	cur_model_checksum = model->amodel_checksum;

	if (!to_load_from_memory) {
		if (model->amodel_buf == NULL) {
			model->amodel_buf  = vmalloc(MAX_AMODEL_SIZE);
			if (!model->amodel_buf)
				return -ENOMEM;
		}

		if (vte->amodel_options & DBMDX_VE_SEND_DUMMY_AMODEL_4B &&
			det_type == DETECTION_TYPE_VOICE_ENERGY)
			ret = dbmdx_va_amodel_load_dummy_model(p,
					0x1,
					model->amodel_buf,
					&model->amodel_size,
					&model->num_of_amodel_chunks,
					model->amodel_chunks_size);
		else
			ret = dbmdx_va_amodel_load_file(p,
					vte->num_of_amodel_files[det_type],
					vte->amodel_fnames[det_type],
					amodel_prefix,
					0x1,
					vte->amodel_options,
					model->amodel_buf,
					&model->amodel_size,
					&model->num_of_amodel_chunks,
					model->amodel_chunks_size);

		if (ret < 0) {
			dev_err(p->dev, "%s: failed to load amodel(%d)\n",
				__func__, ret);
			ret = -EFAULT;
			goto out;
		}

		model->amodel_loaded = true;

		memcpy((u8 *)(&(model->amodel_checksum)),
			&(model->amodel_buf[model->amodel_size - 4]),
			4);

		model->det_type = det_type;

		model->eng_type = vte->eng_type;
		model->amode = LOAD_AMODEL_PRIMARY;
	}

	if (!send_to_fw) {
		ret = 0;
		goto out;
	}

	/* Check if the model is same */
	if (do_not_reload_model &&
		vte->loaded_to_fw && (vte->detection_type == det_type) &&
		model->amodel_loaded &&  (model->eng_type == vte->eng_type) &&
		(model->amodel_checksum == cur_model_checksum)) {

		dev_info(p->dev, "%s: No need to reload model\n", __func__);
		ret = 0;
		goto out;

	}

	p->device_ready = false;

	/* prepare the chip interface for A-Model loading */
	ret = p->chip->prepare_amodel_loading(p);
	if (ret != 0) {
		dev_err(p->dev, "%s: failed to prepare A-Model loading\n",
			__func__);
		p->device_ready = true;
		ret = -EIO;
		goto out;
	}

	vte->loaded_to_fw = false;

	ret = dbmdx_va_amodel_send(p,
				model->amodel_buf,
				model->amodel_size - 4,
				model->num_of_amodel_chunks,
				model->amodel_chunks_size,
				vte->vte_index,
				vte->mem_loc,
				&(model->amodel_buf[model->amodel_size - 4]),
				4,
				model->amode);
	if (ret) {
		dev_err(p->dev, "%s: sending amodel failed\n",
			__func__);
		ret = -EIO;
		goto out_finish_loading;
	}

	if (vte->verify_model_load) {

		ret = vte->verify_model_load(p, vte->vte_index);

		if (ret) {
			dev_err(p->dev, "%s: Error loading acoustic model\n",
				__func__);
			ret = -EIO;
			goto out_finish_loading;
		}
	} else {
		dev_warn(p->dev, "%s: No model verification function\n",
				__func__);

	}

	dev_info(p->dev, "%s: acoustic model sent successfully\n", __func__);

	vte->loaded_to_fw = true;
	vte->detection_type = det_type;
out_finish_loading:
	/* finish A-Model loading */
	ret2 = p->chip->finish_amodel_loading(p);
	if (ret2 != 0)
		dev_err(p->dev, "%s: failed to finish A-Model loading\n",
			__func__);

	p->device_ready = true;

out:
	return ret;
}

static int dbmdx_va_amodel_load_dual(struct dbmdx_private *p,
				struct vt_engine *vte,
				enum dbmdx_model_detection_type det_type,
				const char *amodel_prefix,
				bool to_load_from_memory,
				bool do_not_reload_model,
				bool send_to_fw)
{
	int ret, ret2;
	struct amodel_info *model_pr = NULL;
	struct amodel_info *model_sec = NULL;
	struct amodel_info *cur_model = NULL;
	u32 cur_model_checksum_pri;
	u32 cur_model_checksum_sec;

	if (!p)
		return -EAGAIN;

	if (!p->device_ready) {
		dev_err(p->dev, "%s: device not ready\n", __func__);
		return -EAGAIN;
	}

	if (!vte) {
		dev_err(p->dev, "%s: VT Engine was not allocated\n", __func__);
		return -EINVAL;
	}
	if (det_type != DETECTION_TYPE_DUAL) {
		dev_err(p->dev,
			"%s: Unsupported detection mode:%d",
							__func__, det_type);
		return -EINVAL;

	}

	model_pr = &(vte->primary_model);
	model_sec = &(vte->secondary_model);

	if (!model_pr || !model_sec) {
		dev_err(p->dev, "%s: No model allocated\n", __func__);
		return -EINVAL;
	}

	if (to_load_from_memory &&
		(!(model_pr->amodel_loaded) || !(model_sec->amodel_loaded))) {

		dev_err(p->dev, "%s: Model was not loaded to memory\n",
			__func__);
		return -EAGAIN;
	}

	cur_model_checksum_pri = model_pr->amodel_checksum;
	cur_model_checksum_sec = model_sec->amodel_checksum;

	if (!to_load_from_memory) {

		cur_model = model_pr;

		if (cur_model->amodel_buf == NULL) {
			cur_model->amodel_buf  = vmalloc(MAX_AMODEL_SIZE);
			if (!cur_model->amodel_buf)
				return -ENOMEM;
		}

		ret = dbmdx_va_amodel_load_file(p,
					vte->num_of_amodel_files[det_type],
					vte->amodel_fnames[det_type],
					amodel_prefix,
					0x1,
					vte->amodel_options,
					cur_model->amodel_buf,
					&cur_model->amodel_size,
					&cur_model->num_of_amodel_chunks,
					cur_model->amodel_chunks_size);

		if (ret < 0) {
			dev_err(p->dev,
				"%s: failed to load primary amodel(%d)\n",
				__func__, ret);
			ret = -EFAULT;
			goto out;
		}
		cur_model->amodel_loaded = true;

		memcpy((u8 *)(&(cur_model->amodel_checksum)),
			&(cur_model->amodel_buf[cur_model->amodel_size - 4]),
			4);

		cur_model->det_type = det_type;

		cur_model->eng_type = vte->eng_type;
		cur_model->amode = LOAD_AMODEL_PRIMARY;

		cur_model = model_sec;

		if (cur_model->amodel_buf == NULL) {
			cur_model->amodel_buf  = vmalloc(MAX_AMODEL_SIZE);
			if (!cur_model->amodel_buf)
				return -ENOMEM;
		}

		ret = dbmdx_va_amodel_load_file(p,
					vte->num_of_amodel_files_sec,
					vte->amodel_fnames_sec,
					amodel_prefix,
					0x1,
					vte->amodel_options,
					cur_model->amodel_buf,
					&cur_model->amodel_size,
					&cur_model->num_of_amodel_chunks,
					cur_model->amodel_chunks_size);

		if (ret < 0) {
			dev_err(p->dev,
				"%s: failed to load secondary amodel(%d)\n",
				__func__, ret);
			ret = -EFAULT;
			goto out;
		}
		cur_model->amodel_loaded = true;

		memcpy((u8 *)(&(cur_model->amodel_checksum)),
			&(cur_model->amodel_buf[cur_model->amodel_size - 4]),
			4);

		cur_model->det_type = det_type;

		cur_model->eng_type = vte->eng_type;
		cur_model->amode = LOAD_AMODEL_2NDARY;

		cur_model->amodel_loaded = true;
	}

	if (!send_to_fw) {
		ret = 0;
		goto out;
	}

	/* Check if the model is same */
	if (do_not_reload_model &&
		vte->loaded_to_fw && (vte->detection_type == det_type) &&
		model_pr->amodel_loaded &&
		(model_pr->eng_type == vte->eng_type) &&
		model_sec->amodel_loaded &&
		(model_sec->eng_type == vte->eng_type) &&
		(model_pr->amodel_checksum == cur_model_checksum_pri) &&
		(model_sec->amodel_checksum == cur_model_checksum_sec)) {

		dev_info(p->dev, "%s: No need to reload model\n", __func__);
		ret = 0;
		goto out;

	}

	p->device_ready = false;

	/* prepare the chip interface for A-Model loading */
	ret = p->chip->prepare_amodel_loading(p);
	if (ret != 0) {
		dev_err(p->dev, "%s: failed to prepare A-Model loading\n",
			__func__);
		p->device_ready = true;
		ret = -EIO;
		goto out;
	}

	vte->loaded_to_fw = false;

	cur_model = model_pr;

	ret = dbmdx_va_amodel_send(p,
			cur_model->amodel_buf,
			cur_model->amodel_size - 4,
			cur_model->num_of_amodel_chunks,
			cur_model->amodel_chunks_size,
			vte->vte_index,
			vte->mem_loc,
			&(cur_model->amodel_buf[cur_model->amodel_size - 4]),
			4,
			cur_model->amode);

	if (ret) {
		dev_err(p->dev, "%s: sending amodel failed\n",
			__func__);
		ret = -EIO;
		goto out_finish_loading;
	}

	cur_model = model_sec;

	/* load A-Model and verify checksum */
	ret = dbmdx_va_amodel_send(p,
			cur_model->amodel_buf,
			cur_model->amodel_size - 4,
			cur_model->num_of_amodel_chunks,
			cur_model->amodel_chunks_size,
			vte->vte_index,
			vte->mem_loc,
			&(cur_model->amodel_buf[cur_model->amodel_size - 4]),
			4,
			cur_model->amode);

	if (vte->verify_model_load) {

		ret = vte->verify_model_load(p, vte->vte_index);

		if (ret) {
			dev_err(p->dev, "%s: Error loading acoustic model\n",
				__func__);
			ret = -EIO;
			goto out_finish_loading;
		}
	} else {
		dev_warn(p->dev, "%s: No model verification function\n",
				__func__);

	}


	dev_info(p->dev, "%s: acoustic model sent successfully\n", __func__);

	vte->loaded_to_fw = true;
	vte->detection_type = det_type;

out_finish_loading:
	/* finish A-Model loading */
	ret2 = p->chip->finish_amodel_loading(p);
	if (ret2 != 0)
		dev_err(p->dev, "%s: failed to finish A-Model loading\n",
			__func__);

	p->device_ready = true;

out:
	return ret;
}

static int dbmdx_va_amodel_unload_single(struct dbmdx_private *p,
				struct vt_engine *vte,
				enum dbmdx_model_detection_type det_type)
{
	int ret;
	struct amodel_info *model = NULL;
	u16 vt_num;


	if (!p)
		return -EAGAIN;

	if (!p->device_ready) {
		dev_err(p->dev, "%s: device not ready\n", __func__);
		return -EAGAIN;
	}

	if (!vte) {
		dev_err(p->dev, "%s: VT Engine was not allocated\n", __func__);
		return -EINVAL;
	}

	switch (det_type) {
	case DETECTION_TYPE_VOICE_COMMAND:
		model = &(vte->vc_model);
		break;
	case DETECTION_TYPE_VOICE_ENERGY:
		model = &(vte->ve_model);
		break;
	case DETECTION_TYPE_PHRASE:
		model = &(vte->primary_model);
		break;
	default:
		dev_err(p->dev,
			"%s: Unsupported detection mode:%d",
						__func__, det_type);
		return -EINVAL;
	}


	if (!model) {
		dev_err(p->dev, "%s: No model allocated\n", __func__);
		return -EINVAL;
	}

	if (!(model->amodel_loaded) || !(vte->loaded_to_fw)) {
		dev_dbg(p->dev,	"%s: Model is not loaded in FW\n", __func__);
		return 0;
	}

	if (vte->vte_index == 1)
		vt_num = DBMDX_REGV_LOAD_ENGINE_TYPE_VT1;
	else if (vte->vte_index == 2)
		vt_num = DBMDX_REGV_LOAD_ENGINE_TYPE_VT2;
	else
		vt_num = DBMDX_REGV_LOAD_ENGINE_TYPE_VT3;

	ret = dbmdx_send_cmd(p,
				DBMDX_REGN_LOAD_NEW_ACOUSTIC_MODEL |
				vt_num |
				DBMDX_REGV_OP_TYPE_RELEASE_FILE |
				DBMDX_REGV_INIT_KILL_KILL |
				DBMDX_REGV_LOAD_PRIMARY_VT_MODEL,
				NULL);

	if (ret < 0) {
		dev_err(p->dev, "%s: failed to release model in fw\n",
				__func__);
		ret = -EIO;
		goto out;
	}


	vte->loaded_to_fw = false;
	dev_info(p->dev, "%s: acoustic model successfully released\n",
								__func__);
out:
	return ret;
}

static int dbmdx_va_amodel_unload_dual(struct dbmdx_private *p,
				struct vt_engine *vte,
				enum dbmdx_model_detection_type det_type)
{
	int ret;
	struct amodel_info *model_pr = NULL;
	struct amodel_info *model_sec = NULL;
	u16 vt_num;


	if (!p)
		return -EAGAIN;

	if (!p->device_ready) {
		dev_err(p->dev, "%s: device not ready\n", __func__);
		return -EAGAIN;
	}

	if (!vte) {
		dev_err(p->dev, "%s: VT Engine was not allocated\n", __func__);
		return -EINVAL;
	}

	if (det_type != DETECTION_TYPE_DUAL) {
		dev_err(p->dev,
			"%s: Unsupported detection mode:%d",
							__func__, det_type);
		return -EINVAL;

	}

	model_pr = &(vte->primary_model);
	model_sec = &(vte->secondary_model);


	if (!model_pr || !model_sec) {
		dev_err(p->dev, "%s: No model allocated\n", __func__);
		return -EINVAL;
	}

	if (!(model_pr->amodel_loaded) || !(model_sec->amodel_loaded)
						|| !(vte->loaded_to_fw)) {
		dev_dbg(p->dev,	"%s: Models are not loaded in FW\n", __func__);
		return 0;
	}

	if (vte->vte_index == 1)
		vt_num = DBMDX_REGV_LOAD_ENGINE_TYPE_VT1;
	else if (vte->vte_index == 2)
		vt_num = DBMDX_REGV_LOAD_ENGINE_TYPE_VT2;
	else
		vt_num = DBMDX_REGV_LOAD_ENGINE_TYPE_VT3;

	ret = dbmdx_send_cmd(p,
				DBMDX_REGN_LOAD_NEW_ACOUSTIC_MODEL |
				vt_num |
				DBMDX_REGV_OP_TYPE_RELEASE_FILE |
				DBMDX_REGV_LOAD_SECONDARY_VT_MODEL,
				NULL);

	if (ret < 0) {
		dev_err(p->dev, "%s: failed to release secondary model in fw\n",
				__func__);
		ret = -EIO;
		goto out;
	}


	ret = dbmdx_send_cmd(p,
				DBMDX_REGN_LOAD_NEW_ACOUSTIC_MODEL |
				vt_num |
				DBMDX_REGV_OP_TYPE_RELEASE_FILE |
				DBMDX_REGV_INIT_KILL_KILL |
				DBMDX_REGV_LOAD_PRIMARY_VT_MODEL,
				NULL);

	if (ret < 0) {
		dev_err(p->dev, "%s: failed to release primary model in fw\n",
				__func__);
		ret = -EIO;
		goto out;
	}


	vte->loaded_to_fw = false;
	dev_info(p->dev, "%s: acoustic model successfully released\n",
								__func__);
out:
	return ret;
}


static int dbmdx_va_load_amodel(struct dbmdx_private *p,
				struct vt_engine *vte,
				enum dbmdx_model_detection_type det_type,
				u32 model_options, int param)
{
	int ret;
	bool do_not_reload_model = false;
	bool load_model_from_memory = false;
	bool send_to_fw = true;
	char *amodel_prefix = NULL;
	char amodel_prefix_buf[16];


	if (model_options &  DBMDX_DO_NOT_RELOAD_MODEL)
		do_not_reload_model = true;

	if (model_options &  DBMDX_LOAD_MODEL_FROM_MEMORY)
		load_model_from_memory = true;

	if (model_options &  DBMDX_DO_NOT_SEND_TO_FW)
		send_to_fw = false;

	if ((model_options &  DBMDX_ADD_MODEL_PREFIX) &&
		!load_model_from_memory && (param > 0)) {
		if (param > MAX_AMODEL_PREFIX_INDEX) {
			dev_err(p->dev,
				"%s: Model prefix out of range %d > %d",
				__func__, param, MAX_AMODEL_PREFIX_INDEX);
			return -EINVAL;
		}
		snprintf(amodel_prefix_buf, 16, "%03d_", param);
		amodel_prefix = amodel_prefix_buf;
	}

	dev_dbg(p->dev,
		"%s: Model Opt.: Reload: %s, Load from Mem: %s Send To FW: %s Amodel Prefix: %s\n",
		__func__,
		do_not_reload_model ? "ON" : "OFF",
		load_model_from_memory ? "ON" : "OFF",
		send_to_fw ? "ON" : "OFF",
		amodel_prefix ? amodel_prefix : "OFF");


	switch (det_type) {
	case DETECTION_TYPE_VOICE_COMMAND:
		if (!(vte->mode_support_mask &
					DBMDX_VT_MODE_SUPPORT_VOICE_COMMAND)) {
			dev_err(p->dev,
				"%s: VT Engine doesn't support Command mode",
								__func__);
			return -EINVAL;
		}
		break;
	case DETECTION_TYPE_DUAL:
		if (!(vte->mode_support_mask &
					DBMDX_VT_MODE_SUPPORT_VOICE_DUAL)) {
			dev_err(p->dev,
				"%s: VT Engine doesn't support Dual mode",
								__func__);
			return -EINVAL;
		}
		break;
	case DETECTION_TYPE_VOICE_ENERGY:
		if (!(vte->mode_support_mask &
				DBMDX_VT_MODE_SUPPORT_VOICE_ENERGY)) {
			dev_err(p->dev,
				"%s: VT Engine doesn't support VE mode",
								__func__);
			return -EINVAL;
		}
		break;
	case DETECTION_TYPE_PHRASE:
		if (!(vte->mode_support_mask & DBMDX_VT_MODE_SUPPORT_PHRASE)) {
			dev_err(p->dev,
				"%s: VT Engine doesn't support Phrase mode",
								__func__);
			return -EINVAL;
		}
		break;
	default:
		dev_err(p->dev,
			"%s: Error unknown detection mode:%d", __func__,
								det_type);
		return -EINVAL;
	}

	if (det_type == DETECTION_TYPE_DUAL) {
		ret = dbmdx_va_amodel_load_dual(p,
						vte,
						det_type,
						amodel_prefix,
						load_model_from_memory,
						do_not_reload_model,
						send_to_fw);
		if (ret < 0) {
			dev_err(p->dev,
				"%s: Error loading dual acoustic model:%d\n",
				__func__, ret);
			return ret;
		}
	} else {
		ret = dbmdx_va_amodel_load_single(p,
						vte,
						det_type,
						amodel_prefix,
						load_model_from_memory,
						do_not_reload_model,
						send_to_fw);
		if (ret < 0) {
			dev_err(p->dev,	"%s: Error loading acoustic model:%d\n",
					__func__, ret);
			return ret;
		}
	}

	return ret;
}


static int dbmdx_va_unload_amodel(struct dbmdx_private *p,
				struct vt_engine *vte,
				enum dbmdx_model_detection_type det_type)
{
	int ret;
	struct amodel_info *model = NULL;

	switch (det_type) {
	case DETECTION_TYPE_VOICE_COMMAND:
		if (!(vte->mode_support_mask &
					DBMDX_VT_MODE_SUPPORT_VOICE_COMMAND)) {
			dev_err(p->dev,
				"%s: VT Engine doesn't support Command mode",
								__func__);
			return -EINVAL;
		}
		model = &(vte->vc_model);
		break;
	case DETECTION_TYPE_DUAL:
		if (!(vte->mode_support_mask &
				DBMDX_VT_MODE_SUPPORT_VOICE_DUAL)) {
			dev_err(p->dev,
				"%s: VT Engine doesn't support Dual mode",
								__func__);
			return -EINVAL;
		}
		model = &(vte->vc_model);
		break;
	case DETECTION_TYPE_VOICE_ENERGY:
		if (!(vte->mode_support_mask &
				DBMDX_VT_MODE_SUPPORT_VOICE_ENERGY)) {
			dev_err(p->dev,
				"%s: VT Engine doesn't support VE mode",
								__func__);
			return -EINVAL;
		}
		model = &(vte->ve_model);
		break;
	case DETECTION_TYPE_PHRASE:
		if (!(vte->mode_support_mask & DBMDX_VT_MODE_SUPPORT_PHRASE)) {
			dev_err(p->dev,
				"%s: VT Engine doesn't support Phrase mode",
								__func__);
			return -EINVAL;
		}
		model = &(vte->primary_model);
		break;
	default:
		dev_err(p->dev,
			"%s: Error unknown detection mode:%d", __func__,
								det_type);
		return -EINVAL;
	}

	if (!model || !(model->amodel_loaded) || !(vte->loaded_to_fw)) {
		dev_dbg(p->dev,	"%s: Model is not loaded in FW\n", __func__);
		return 0;
	}

	if (det_type == DETECTION_TYPE_DUAL) {
		ret = dbmdx_va_amodel_unload_dual(p,
						vte,
						det_type);
		if (ret < 0) {
			dev_err(p->dev,
				"%s: Error loading dual acoustic model:%d\n",
				__func__, ret);
			return ret;
		}
	} else {
		ret = dbmdx_va_amodel_unload_single(p,
						vte,
						det_type);
		if (ret < 0) {
			dev_err(p->dev,	"%s: Error unloading acoustic model:%d\n",
					__func__, ret);
			return ret;
		}
	}

	return ret;
}

static ssize_t dbmdx_va_usecase_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct dbmdx_private *p = dev_get_drvdata(dev);

	if (!p)
		return -EAGAIN;

	if (!p->device_ready) {
		dev_err(p->dev, "%s: device not ready\n", __func__);
		return -EAGAIN;
	}

	return snprintf(buf, PAGE_SIZE, "%d\n",
				p->va_ve_flags.active_usecase_id);
}

static ssize_t dbmdx_va_usecase_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t size)
{
	int ret;
	struct dbmdx_private *p = dev_get_drvdata(dev);

	int val = dbmdx_buf_to_int(buf);

	dev_dbg(p->dev, "%s: val - %d\n", __func__, val);

	p->usr_op_lock(p);

	ret = dbmdx_va_set_usecase_id(p, val);

	p->usr_op_unlock(p);

	return ret < 0 ? ret : size;
}

static ssize_t dbmdx_va_usecase_name_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct dbmdx_private *p = dev_get_drvdata(dev);

	if (!p)
		return -EAGAIN;

	if (!p->device_ready) {
		dev_err(p->dev, "%s: device not ready\n", __func__);
		return -EAGAIN;
	}

	return snprintf(buf, PAGE_SIZE, "%s\n",
		p->va_ve_flags.cur_usecase == NULL ? "NONE" :
		p->va_ve_flags.cur_usecase->usecase_name);
}

static ssize_t dbmdx_va_usecase_name_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t size)
{
	int ret;
	struct dbmdx_private *p = dev_get_drvdata(dev);

	p->usr_op_lock(p);

	ret = dbmdx_set_va_usecase_name(p, buf);

	p->usr_op_unlock(p);

	return ret < 0 ? ret : size;
}

static ssize_t dbmdx_usecase_manager_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t size)
{
	struct dbmdx_private *p = dev_get_drvdata(dev);
	unsigned long val;
	int ret = 0;

	if (!p)
		return -EAGAIN;

	if (!p->device_ready) {
		dev_err(p->dev, "%s: device not ready\n", __func__);
		return -EAGAIN;
	}

	ret = kstrtoul(buf, 0, &val);
	if (ret)
		return -EINVAL;

	p->usr_op_lock(p);

	ret = dbmdx_usecase_manager(p, (unsigned int)val);

	p->usr_op_unlock(p);

	return ret < 0 ? ret : size;
}

static ssize_t dbmdx_va_load_external_usecase_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t size)
{
#ifdef DBMDX_LOAD_USECASE_FROM_FILE_SUPPORTED
	int ret;
	struct dbmdx_private *p = dev_get_drvdata(dev);
	struct firmware	*usecase_fw = NULL;
	char fw_name[MAX_USECASE_FILENAME_LEN+1];
	int n;

	if (!p)
		return -EAGAIN;

	if (!p->device_ready) {
		dev_err(p->dev, "%s: device not ready\n", __func__);
		return -EAGAIN;
	}

	dev_dbg(p->dev, "%s: usecase file name - %s\n", __func__, buf);
	n = strlen(buf);

	if (!n) {
		dev_err(p->dev,	"%s: No usecase filename supplied\n", __func__);
		ret = -EINVAL;
		goto out;
	}

	if (buf[n-1] == '\n')
		n = n - 1;

	if (!n) {
		dev_err(p->dev,	"%s: No usecase filename supplied\n", __func__);
		ret = -EINVAL;
		goto out;
	}

	if (n > MAX_USECASE_FILENAME_LEN) {
		dev_err(p->dev,	"%s: usecase filename is too long\n", __func__);
		ret = -EINVAL;
		goto out;
	}

	strncpy(fw_name, buf, n);
	fw_name[n] = '\0';

	ret = request_firmware((const struct firmware **)&usecase_fw,
				       fw_name, p->dev);
	if (ret != 0) {
		dev_err(p->dev, "%s: failed to request usecase file\n",
				__func__);
		ret = -EIO;
		goto out;
	}

	if (p->external_usecase_loaded)
		free_external_usecase_memory(&config_usecase_external);

	p->external_usecase_loaded = false;

	ret = load_usecase_from_buffer(p, usecase_fw->data, usecase_fw->size,
		&config_usecase_external);

	release_firmware(usecase_fw);

	if (ret != 0) {
		dev_err(p->dev, "%s: failed to load external usecase file\n",
				__func__);
		ret = -EIO;
		goto out;
	}

	p->external_usecase_loaded = true;

	dev_info(p->dev, "%s: External usecase was loaded successfully\n",
				__func__);
	ret = 0;

out:
	return ret < 0 ? ret : size;
#else
	return size;
#endif
}

static ssize_t dbmdx_va_usecase_mode_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct dbmdx_private *p = dev_get_drvdata(dev);
	unsigned int	usecase_mode;

	if (!p)
		return -EAGAIN;

	if (!p->device_ready) {
		dev_err(p->dev, "%s: device not ready\n", __func__);
		return -EAGAIN;
	}

	usecase_mode =  p->va_ve_flags.usecase_mode;

	if (usecase_mode >= 0 && usecase_mode < DBMDX_NR_OF_STATES) {
		dev_dbg(p->dev, "%s: Current Usecase mode: %s (%x)\n",
			__func__,
			dbmdx_state_names[usecase_mode],
			usecase_mode);

		return snprintf(buf, PAGE_SIZE, "%s (%x)\n",
				dbmdx_state_names[usecase_mode],
				usecase_mode);
	} else {
		dev_dbg(p->dev, "%s: Current Usecase mode: Invalid (%x)\n",
			__func__, usecase_mode);
		return snprintf(buf, PAGE_SIZE, "%s Invalid (%x)\n",
				__func__, usecase_mode);
	}

	return 0;
}

static ssize_t dbmdx_va_usecase_mode_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t size)
{
	struct dbmdx_private *p = dev_get_drvdata(dev);
	unsigned long val;
	int ret = 0;

	if (!p)
		return -EAGAIN;

	if (!p->device_ready) {
		dev_err(p->dev, "%s: device not ready\n", __func__);
		return -EAGAIN;
	}

	ret = kstrtoul(buf, 0, &val);
	if (ret)
		return -EINVAL;

	dev_info(p->dev, "%s: Required mode: (%x), Current mode: (%x)\n",
			__func__, (int)val, p->va_ve_flags.usecase_mode);

	/* flush pending buffering work if any */
	p->va_flags.buffering = 0;
	flush_work(&p->sv_work);

	p->lock(p);

	dbmdx_set_power_mode(p, DBMDX_PM_ACTIVE);
	ret = dbmdx_set_usecase_mode(p, val);

	if (ret)
		size = ret;

	p->unlock(p);

	return size;
}


static ssize_t dbmdx_va_usecase_info_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t size)
{
	int ret;
	struct dbmdx_private *p = dev_get_drvdata(dev);
	int n;

	if (!p)
		return -EAGAIN;

	if (!p->device_ready) {
		dev_err(p->dev, "%s: device not ready\n", __func__);
		return -EAGAIN;
	}

	dev_dbg(p->dev, "%s: usecase name - %s\n", __func__, buf);
	n = strlen(buf);

	if (!n) {
		dev_err(p->dev,	"%s: No usecase name supplied\n", __func__);
		ret = -EINVAL;
		goto out;
	}

	if (buf[n-1] == '\n')
		n = n - 1;

	if (!n) {
		dev_err(p->dev,	"%s: No usecase name supplied\n", __func__);
		ret = -EINVAL;
		goto out;
	}

	if (n > MAX_USECASE_NAME_LEN) {
		dev_err(p->dev,	"%s: usecase name is too long\n", __func__);
		ret = -EINVAL;
		goto out;
	}

	strncpy(p->usecase_info, buf, n);
	p->usecase_info[n] = '\0';

	ret = 0;
out:
	return ret < 0 ? ret : size;
}

static ssize_t dbmdx_va_usecase_info_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	int ret;
	int ind;
	struct dbmdx_private *p = dev_get_drvdata(dev);
	int num_usecases = sizeof(usecases_map) /
				sizeof(struct usecase_config *);
	struct usecase_config *cur_usecase = NULL;
	int n;

	if (!p)
		return -EAGAIN;

	if (!p->device_ready) {
		dev_err(p->dev, "%s: device not ready\n", __func__);
		return -EAGAIN;
	}

	dev_dbg(p->dev, "%s: usecase info - %s\n", __func__, p->usecase_info);
	n = strlen(p->usecase_info);

	if (!n) {
		dev_err(p->dev,	"%s: No usecase name supplied for info\n",
			__func__);
		ret = -EINVAL;
		goto out;
	}

	if (!strncmp(p->usecase_info, "external", 8)) {
		if (!(p->external_usecase_loaded)) {
			dev_err(p->dev,	"%s: External usecase is not loaded\n",
				__func__);
			ret = -EINVAL;
			goto out;
		}
		cur_usecase = &config_usecase_external;
	} else {
		for (ind = 0; ind < num_usecases; ind++) {
			cur_usecase = usecases_map[ind];

			if (!cur_usecase)
				continue;
			if (n != strlen(cur_usecase->usecase_name))
				continue;
			if (!strncmp(p->usecase_info,
					cur_usecase->usecase_name, n)) {
				/* Verify HW Rev */
				if (p->pdata->hw_rev == DBMDX_DEFAULT_HW_REV)
					break;
				if (p->pdata->hw_rev != cur_usecase->hw_rev)
					continue;
				break;
			}
		}

		if (ind >= num_usecases || !cur_usecase) {
			dev_err(p->dev,
				"%s: Unsupported usecase %s\n", __func__,
				p->usecase_info);
			ret = -EINVAL;
			goto out;
		}
	}

	return dbmdx_dump_usecase(p, cur_usecase, buf);
out:
	return ret;
}


static ssize_t dbmdx_user_selected_chip_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct dbmdx_private *p = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "User seletect chip: %s\n",
		p->usr_selected_chip == DBMDX_CHIP_VA ? "VA" : "VA_VE");
}

static ssize_t dbmdx_user_selected_chip_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t size)
{
	struct dbmdx_private *p = dev_get_drvdata(dev);

	if  (!strncmp(buf, "va_ve", min_t(int, size, 5))) {
		if (p && p->va_ve_chip_enabled)
			p->usr_selected_chip = DBMDX_CHIP_VA_VE;
		else {
			dev_info(p->dev, "%s: VA_VE chip disabled\n", __func__);
			return -EINVAL;
		}
	} else if (!strncmp(buf, "va", min_t(int, size, 2))) {
		if (p && p->va_chip_enabled)
			p->usr_selected_chip = DBMDX_CHIP_VA;
		else {
			dev_info(p->dev, "%s: VA chip disabled\n", __func__);
			return -EINVAL;
		}
	} else
		dev_info(p->dev, "%s: Chip types: va | va_ve\n", __func__);

	return size;

}

static ssize_t dbmdx_va_ve_mic_cfg_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t size)
{
	struct dbmdx_private *p = dev_get_drvdata(dev);
	struct dbmdx_platform_data *pdata;
	char *str_p;
	char *args =  (char *)buf;
	unsigned long val;
	u32 index = 0;
	u32 new_value = 0;
	bool index_set = false, value_set = false;
	int i;

	int ret = -EINVAL;

	if (!p)
		return -EAGAIN;

	pdata = p->pdata;

	while ((str_p = strsep(&args, " \t")) != NULL) {

		if (!*str_p)
			continue;

		if (strncmp(str_p, "index=", 6) == 0) {
			ret = kstrtoul((str_p+6), 0, &val);
			if (ret) {
				dev_err(p->dev, "%s: bad index\n", __func__);
				ret = -EINVAL;
				goto print_usage;
			} else if (val >  (VA_VE_MIC_CONFIG_SIZE - 1)) {
				dev_err(p->dev, "%s: index out of range: %d\n",
					__func__, (int)val);
				ret = -EINVAL;
				goto print_usage;
			}
			index = (u32)val;
			index_set = true;
			continue;
		}
		if (strncmp(str_p, "value=", 6) == 0) {
			ret = kstrtoul((str_p+6), 0, &val);
			if (ret) {
				dev_err(p->dev, "%s: bad value\n", __func__);
				ret = -EINVAL;
				goto print_usage;
			}

			new_value = (u32)val;
			value_set = true;
			continue;
		}
	}

	if (!index_set) {
		dev_err(p->dev, "%s: index is not set\n", __func__);
		ret = -EINVAL;
		goto print_usage;
	} else if (!value_set) {
		dev_err(p->dev, "%s: value is not set\n", __func__);
		ret = -EINVAL;
		goto print_usage;
	}
	p->lock(p);

	p->pdata->va_ve_mic_config[index] = new_value;

	dev_info(p->dev, "%s: va_ve_mic_config[%u] was set to %u\n",
		__func__, index, new_value);

	p->unlock(p);

	for (i = 0; i < VA_VE_MIC_CONFIG_SIZE; i++)
		dev_dbg(dev, "%s: VA_VE mic cfg %8.8x: 0x%8.8x\n",
			__func__, i, pdata->va_ve_mic_config[i]);

	return size;
print_usage:
	dev_info(p->dev,
		"%s: Usage: index=[0-%d] value=newval\n",
		__func__, (VA_VE_MIC_CONFIG_SIZE - 1));
	return ret;
}


static ssize_t dbmdx_va_ve_mic_cfg_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct dbmdx_private *p = dev_get_drvdata(dev);
	int off = 0;
	struct dbmdx_platform_data *pdata;
	int i;

	if (!p)
		return -EAGAIN;

	pdata = p->pdata;

	for (i = 0; i < VA_VE_MIC_CONFIG_SIZE; i++)
		off += snprintf(buf + off, PAGE_SIZE - off,
				"\tVA_VE mic cfg %8.8x: 0x%8.8x\n",
				i, pdata->va_ve_mic_config[i]);

	return off;
}

static ssize_t dbmdx_va_ve_boot_options_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct dbmdx_private *p = dev_get_drvdata(dev);
	int ret;

	if (!p)
		return -EAGAIN;

	ret = snprintf(buf, PAGE_SIZE, "VA_VE Boot options: 0x%x\n",
		p->pdata->boot_options_va_ve);

	return ret;
}

static ssize_t dbmdx_va_ve_boot_options_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t size)
{
	struct dbmdx_private *p = dev_get_drvdata(dev);
	unsigned long val;
	int ret;

	ret = kstrtoul(buf, 0, &val);
	if (ret)
		return -EINVAL;

	if (!p)
		return -EAGAIN;

	 p->pdata->boot_options_va_ve = val;

	return size;
}

static ssize_t dbmdx_va_ve_mic_mask_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct dbmdx_private *p = dev_get_drvdata(dev);
	int off = 0;

	if (!p)
		return -EAGAIN;

	off += snprintf(buf + off, PAGE_SIZE - off,
					"\t=======Mic Mask ======\n");

	off += snprintf(buf + off, PAGE_SIZE, "VA_VE Mic Mask: 0x%x\n",
		p->pdata->va_ve_mic_mask);

	off += snprintf(buf + off, PAGE_SIZE - off,
			"\tMic1 Selected:\t%s\n",
			(p->pdata->va_ve_mic_mask & 0x0001) ? "ON" : "OFF");

	off += snprintf(buf + off, PAGE_SIZE - off,
			"\tMic2 Selected:\t%s\n",
			(p->pdata->va_ve_mic_mask & 0x0002) ? "ON" : "OFF");

	off += snprintf(buf + off, PAGE_SIZE - off,
			"\tMic3 Selected:\t%s\n",
			(p->pdata->va_ve_mic_mask & 0x0004) ? "ON" : "OFF");

	off += snprintf(buf + off, PAGE_SIZE - off,
			"\tMic4 Selected:\t%s\n",
			(p->pdata->va_ve_mic_mask & 0x0008) ? "ON" : "OFF");


	return off;
}

static ssize_t dbmdx_va_ve_mic_mask_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t size)
{
	struct dbmdx_private *p = dev_get_drvdata(dev);
	unsigned long val;
	int ret;
	int number_of_selected_mics = 0;

	ret = kstrtoul(buf, 0, &val);
	if (ret)
		return -EINVAL;

	if (!p)
		return -EAGAIN;

	number_of_selected_mics = (val & 0x0001) +
				((val & 0x0002) >> 1) +
				((val & 0x0004) >> 2) +
				((val & 0x0008) >> 3);

	if (number_of_selected_mics > 2) {
		dev_err(p->dev, "%s: Number of selected mics > 2\n", __func__);
		return -EINVAL;
	}

	p->pdata->va_ve_mic_mask = val;

	return size;
}

static ssize_t dbmdx_asrp_delay_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct dbmdx_private *p = dev_get_drvdata(dev);
	int ret;

	if (!p)
		return -EAGAIN;

	ret = snprintf(buf, PAGE_SIZE, "ASRP Delay: 0x%x\n",
		p->pdata->asrp_delay);

	return ret;
}

static ssize_t dbmdx_asrp_delay_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t size)
{
	struct dbmdx_private *p = dev_get_drvdata(dev);
	unsigned long val;
	int ret;

	ret = kstrtoul(buf, 0, &val);
	if (ret)
		return -EINVAL;

	if (!p)
		return -EAGAIN;

	p->pdata->asrp_delay = val;

	dev_info(p->dev, "%s: ASRP Delay was set to 0x%x\n", __func__,
		p->pdata->asrp_delay);

	return size;
}

static ssize_t dbmdx_asrp_gain_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct dbmdx_private *p = dev_get_drvdata(dev);
	int off = 0;


	if (!p)
		return -EAGAIN;

	off += snprintf(buf, PAGE_SIZE, "ASRP TX OUT Gain: 0x%x\n",
		p->pdata->asrp_tx_out_gain);

	off += snprintf(buf + off, PAGE_SIZE, "ASRP VCPF OUT Gain: 0x%x\n",
		p->pdata->asrp_vcpf_out_gain);

	off += snprintf(buf + off, PAGE_SIZE, "ASRP RX OUT Gain: 0x%x\n",
		p->pdata->asrp_rx_out_gain);

	return off;
}

static ssize_t dbmdx_asrp_gain_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t size)
{
	struct dbmdx_private *p = dev_get_drvdata(dev);
	unsigned long val;
	int ret;

	ret = kstrtoul(buf, 0, &val);
	if (ret)
		return -EINVAL;

	if (!p)
		return -EAGAIN;

	if (val & 0x20000) {

		p->pdata->asrp_rx_out_gain = val & 0xFFFF;

		dev_info(p->dev, "%s: ASRP RX OUT Gain was set to 0x%x\n",
			__func__, p->pdata->asrp_rx_out_gain);


	} else if (val & 0x10000) {

		p->pdata->asrp_vcpf_out_gain = val & 0xFFFF;

		dev_info(p->dev, "%s: ASRP VCPF OUT Gain was set to 0x%x\n",
			__func__, p->pdata->asrp_vcpf_out_gain);

	} else {

		p->pdata->asrp_tx_out_gain = val;

		dev_info(p->dev, "%s: ASRP TX OUT Gain was set to 0x%x\n",
			__func__, p->pdata->asrp_tx_out_gain);
	}

	return size;
}

static ssize_t dbmdx_va_ve_cfg_values_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t size)
{
	struct dbmdx_private *p = dev_get_drvdata(dev);
	struct dbmdx_platform_data *pdata;
	char *str_p;
	char *args =  (char *)buf;
	unsigned long val;
	u32 index = 0;
	u32 new_value = 0;
	bool index_set = false, value_set = false;
	int i;

	int ret = -EINVAL;

	if (!p)
		return -EAGAIN;

	pdata = p->pdata;

	while ((str_p = strsep(&args, " \t")) != NULL) {

		if (!*str_p)
			continue;

		if (strncmp(str_p, "index=", 6) == 0) {
			ret = kstrtoul((str_p+6), 0, &val);
			if (ret) {
				dev_err(p->dev, "%s: bad index\n", __func__);
				ret = -EINVAL;
				goto print_usage;
			} else if (val >=  pdata->va_ve_cfg_values) {
				dev_err(p->dev, "%s: index out of range: %d\n",
					__func__, (int)val);
				ret = -EINVAL;
				goto print_usage;
			}
			index = (u32)val;
			index_set = true;
			continue;
		}
		if (strncmp(str_p, "value=", 6) == 0) {
			ret = kstrtoul((str_p+6), 0, &val);
			if (ret) {
				dev_err(p->dev, "%s: bad value\n", __func__);
				ret = -EINVAL;
				goto print_usage;
			}

			new_value = (u32)val;
			value_set = true;
			continue;
		}
	}

	if (!index_set) {
		dev_err(p->dev, "%s: index is not set\n", __func__);
		ret = -EINVAL;
		goto print_usage;
	} else if (!value_set) {
		dev_err(p->dev, "%s: value is not set\n", __func__);
		ret = -EINVAL;
		goto print_usage;
	}
	p->lock(p);

	p->pdata->va_ve_cfg_value[index] = new_value;

	dev_info(p->dev, "%s: va_ve_cfg_value[%u] was set to %u\n",
		__func__, index, new_value);

	p->unlock(p);

	for (i = 0; i < pdata->va_ve_cfg_values; i++)
		dev_dbg(dev, "%s: VA_VE cfg %8.8x: 0x%8.8x\n",
			__func__, i, pdata->va_ve_cfg_value[i]);

	return size;
print_usage:
	dev_info(p->dev,
		"%s: Usage: index=[0-%u] value=newval\n",
		__func__, (u32)(pdata->va_ve_cfg_values));
	return ret;
}


static ssize_t dbmdx_va_ve_cfg_values_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct dbmdx_private *p = dev_get_drvdata(dev);
	int off = 0;
	struct dbmdx_platform_data *pdata;
	int i;

	if (!p)
		return -EAGAIN;

	pdata = p->pdata;

	for (i = 0; i < pdata->va_ve_cfg_values; i++)
		off += snprintf(buf + off, PAGE_SIZE - off,
				"\tVA_VE cfg %8.8x: 0x%8.8x\n",
				i, pdata->va_ve_cfg_value[i]);

	return off;
}

static ssize_t dbmdx_ref_playback_active_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct dbmdx_private *p = dev_get_drvdata(dev);
	int ret;

	if (!p)
		return -EAGAIN;

	if (!p->device_ready) {
		dev_err(p->dev, "%s: device not ready\n", __func__);
		return -EAGAIN;
	}
	return snprintf(buf, PAGE_SIZE, "Reference Playback Active: %s\n",
		p->reference_playback_active ? "ON" : "OFF");

	return ret;
}

static ssize_t dbmdx_ref_playback_active_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t size)
{
	struct dbmdx_private *p = dev_get_drvdata(dev);
	unsigned long val;
	int ret;

	ret = kstrtoul(buf, 0, &val);
	if (ret)
		return -EINVAL;

	if (!p)
		return -EAGAIN;

	if (val > 1)
		val = 1;

	p->reference_playback_active = val;

	dev_info(p->dev, "%s: Reference Playback Active: %s\n",
		__func__, p->reference_playback_active ? "ON" : "OFF");

	return size;
}

static ssize_t dbmdx_va_alsa_streaming_options_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct dbmdx_private *p = dev_get_drvdata(dev);
	int ret;

	if (!p)
		return -EAGAIN;

	ret = snprintf(buf, PAGE_SIZE, "alsa_streaming_options: 0x%x\n",
		p->pdata->alsa_streaming_options);

	return ret;
}

static ssize_t dbmdx_va_alsa_streaming_options_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t size)
{
	struct dbmdx_private *p = dev_get_drvdata(dev);
	unsigned long val;
	int ret;

	ret = kstrtoul(buf, 0, &val);
	if (ret)
		return -EINVAL;

	if (!p)
		return -EAGAIN;

	p->pdata->alsa_streaming_options = val;

	dev_info(p->dev, "%s: alsa_streaming options was set to 0x%x\n",
		__func__, p->pdata->alsa_streaming_options);

	return size;
}

static ssize_t dbmdx_va_max_sample_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct dbmdx_private *p = dev_get_drvdata(dev);

	if (!p)
		return -EAGAIN;

	if (p->mics_connected_to_va_ve_chip)
		return dbmdx_reg_show_va_ve(dev, DBMDX_REGN_LAST_MAX_SMP_VALUE,
						attr, buf);
	else
		return dbmdx_reg_show_va(dev, DBMDX_REGN_LAST_MAX_SMP_VALUE,
						attr, buf);
}

static ssize_t dbmdx_io_value_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	return dbmdx_reg_show_long(dev,
				  DBMDX_REGN_IO_PORT_VALUE_LO, attr, buf);
}

static ssize_t dbmdx_io_value_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t size)
{
	return dbmdx_reg_store_long(dev,
				   DBMDX_REGN_IO_PORT_VALUE_LO, attr,
				   buf, size);
}

static ssize_t dbmdx_va_buffer_size_show(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	return dbmdx_reg_show(dev, DBMDX_REGN_AUDIO_BUFFER_SIZE, attr, buf);
}

static ssize_t dbmdx_va_buffer_size_store(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf, size_t size)
{
	return dbmdx_reg_store(dev,
			       DBMDX_REGN_AUDIO_BUFFER_SIZE,
			       attr,
			       buf,
			       size,
			       DBMDX_FW_VA);
}

static ssize_t dbmdx_va_buffsmps_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	return dbmdx_reg_show(dev, DBMDX_REGN_NUM_OF_SMP_IN_BUF, attr, buf);
}

static ssize_t dbmdx_va_capture_on_detect_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct dbmdx_private *p = dev_get_drvdata(dev);
	int ret;

	if (!p)
		return -EAGAIN;

	if (!p->device_ready) {
		dev_err(p->dev, "%s: device not ready\n", __func__);
		return -EAGAIN;
	}

	ret = snprintf(buf, PAGE_SIZE, "%d\n", p->va_capture_on_detect);

	return ret;
}

static ssize_t dbmdx_va_capture_on_detect_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t size)
{
	struct dbmdx_private *p = dev_get_drvdata(dev);
	unsigned long val;
	int ret;

	ret = kstrtoul(buf, 0, &val);
	if (ret)
		return -EINVAL;

	if (!p)
		return -EAGAIN;

	if (val > 2) {
		dev_err(p->dev, "%s: invalid captute on detection mode\n",
			__func__);
		return -EINVAL;
	}

	p->va_capture_on_detect = (bool)val;

	return size;
}

static ssize_t dbmdx_va_detection_after_buffering_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct dbmdx_private *p = dev_get_drvdata(dev);
	int ret;

	if (!p)
		return -EAGAIN;

	if (!p->device_ready) {
		dev_err(p->dev, "%s: device not ready\n", __func__);
		return -EAGAIN;
	}

	ret = snprintf(buf, PAGE_SIZE, "%d\n",
		p->pdata->detection_after_buffering);

	return ret;
}

static ssize_t dbmdx_va_detection_after_buffering_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t size)
{
	struct dbmdx_private *p = dev_get_drvdata(dev);
	unsigned long val;
	int ret;

	ret = kstrtoul(buf, 0, &val);
	if (ret)
		return -EINVAL;

	if (!p)
		return -EAGAIN;

	if (val >= DETECTION_AFTER_BUFFERING_MODE_MAX) {
		dev_err(p->dev, "%s: invalid detection_after_buffering mode\n",
			__func__);
		return -EINVAL;
	}

	p->pdata->detection_after_buffering = val;

	return size;
}

static ssize_t dbmdx_va_disable_recovery_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct dbmdx_private *p = dev_get_drvdata(dev);
	int ret;

	if (!p)
		return -EAGAIN;

	if (!p->device_ready) {
		dev_err(p->dev, "%s: device not ready\n", __func__);
		return -EAGAIN;
	}

	ret = snprintf(buf, PAGE_SIZE, "\tVA Disable Recovery:\t%s\n",
			p->pdata->va_recovery_disabled ? "ON" : "OFF");

	return ret;
}

static ssize_t dbmdx_va_disable_recovery_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t size)
{
	struct dbmdx_private *p = dev_get_drvdata(dev);
	unsigned long val;
	int ret;

	ret = kstrtoul(buf, 0, &val);
	if (ret)
		return -EINVAL;

	if (!p)
		return -EAGAIN;

	if (val != 1  && val != 0) {
		dev_err(p->dev, "%s: invalid recovery disable mode\n",
			__func__);
		return -EINVAL;
	}

	p->pdata->va_recovery_disabled = (unsigned int)val;

	return size;
}

static ssize_t dbmdx_va_boot_options_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct dbmdx_private *p = dev_get_drvdata(dev);
	int ret;

	if (!p)
		return -EAGAIN;

	ret = snprintf(buf, PAGE_SIZE, "VA Boot options: 0x%x\n",
		p->pdata->boot_options);

	return ret;
}

static ssize_t dbmdx_va_boot_options_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t size)
{
	struct dbmdx_private *p = dev_get_drvdata(dev);
	unsigned long val;
	int ret;

	ret = kstrtoul(buf, 0, &val);
	if (ret)
		return -EINVAL;

	if (!p)
		return -EAGAIN;

	 p->pdata->boot_options = val;

	return size;
}

static ssize_t dbmdx_va_amodel_options_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct dbmdx_private *p = dev_get_drvdata(dev);
	int ret;

	if (!p)
		return -EAGAIN;

	ret = snprintf(buf, PAGE_SIZE, "amodel_options: 0x%x\n",
		p->pdata->amodel_options);

	return ret;
}

static ssize_t dbmdx_va_amodel_options_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t size)
{
	struct dbmdx_private *p = dev_get_drvdata(dev);
	unsigned long val;
	int ret;

	ret = kstrtoul(buf, 0, &val);
	if (ret)
		return -EINVAL;

	if (!p)
		return -EAGAIN;

	 p->pdata->amodel_options = val;

	return size;
}


static ssize_t dbmdx_hw_revision_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct dbmdx_private *p = dev_get_drvdata(dev);
	int ret;

	if (!p)
		return -EAGAIN;

	ret = snprintf(buf, PAGE_SIZE, "Hardware Revision: 0x%x\n",
		p->pdata->hw_rev);

	return ret;
}

static ssize_t dbmdx_hw_revision_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t size)
{
	struct dbmdx_private *p = dev_get_drvdata(dev);
	unsigned long val;
	int ret;

	ret = kstrtoul(buf, 0, &val);
	if (ret)
		return -EINVAL;

	if (!p)
		return -EAGAIN;

	p->pdata->hw_rev = (u32)val;

	dev_dbg(p->dev, "%s: HW revision was set to 0x%8x\n",
		__func__, p->pdata->hw_rev);

	return size;
}

static ssize_t dbmdx_debug_delay_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct dbmdx_private *p = dev_get_drvdata(dev);
	int ret;

	if (!p)
		return -EAGAIN;

	ret = snprintf(buf, PAGE_SIZE, "Add. Debug Delay: %u uSec\n",
		p->debug_delay_usec);

	return ret;
}

static ssize_t dbmdx_debug_delay_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t size)
{
	struct dbmdx_private *p = dev_get_drvdata(dev);
	unsigned long val;
	int ret;

	ret = kstrtoul(buf, 0, &val);
	if (ret)
		return -EINVAL;

	if (!p)
		return -EAGAIN;

	p->debug_delay_usec = (u32)val;

	dev_dbg(p->dev, "%s: Add. Debug Delay was set %u uSec\n",
		__func__, p->debug_delay_usec);

	return size;
}

static ssize_t dbmdx_fw_err_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct dbmdx_private *p = dev_get_drvdata(dev);
	ssize_t off = 0;
	int ret = 0;
	u16 num_of_errors = 0;
	u32 err_code = 0;

	if (!p)
		return -EAGAIN;

	if (!p->device_ready) {
		dev_err(p->dev, "%s: device not ready\n", __func__);
		return -EAGAIN;
	}

	p->lock(p);

	dbmdx_set_power_mode(p, DBMDX_PM_ACTIVE);

	if (p->va_chip_enabled) {
		ret = dbmdx_get_fw_errors(p, DBMDX_CHIP_VA, &num_of_errors,
						&err_code);
		if (ret) {
			dev_err(p->dev,
				"%s Error reading VA FW errors\n",
				__func__);
			ret = -EIO;
			goto out_unlock;
		}

		off += snprintf(buf + off, PAGE_SIZE - off,
				"Number of errors VA Chip: %d\n",
				(int)num_of_errors);

		if (num_of_errors > 0)
			off += snprintf(buf + off, PAGE_SIZE - off,
					"VA Chip Error code: 0x%8x\n",
					err_code);
	}
	if (p->va_ve_chip_enabled) {

		ret = dbmdx_get_fw_errors(p, DBMDX_CHIP_VA_VE, &num_of_errors,
						&err_code);
		if (ret) {
			dev_err(p->dev,
				"%s Error reading VA_VE FW errors\n",
				__func__);
			ret = -EIO;
			goto out_unlock;
		}

		off += snprintf(buf + off, PAGE_SIZE - off,
				"Number of errors VA_VE Chip: %d\n",
				(int)num_of_errors);

		if (num_of_errors > 0)
			off += snprintf(buf + off, PAGE_SIZE - off,
					"VA_VE Chip Error code: 0x%8x\n",
					err_code);

	}

out_unlock:
	dbmdx_set_power_mode(p, DBMDX_PM_FALLING_ASLEEP);

	p->unlock(p);
	if (ret < 0)
		return ret;
	else
		return off;
}


static ssize_t dbmdx_va_analog_micgain_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	return dbmdx_reg_show(dev,
			DBMDX_REGN_MICROPHONE_ANALOG_GAIN, attr, buf);
}

static ssize_t dbmdx_va_analog_micgain_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t size)
{
	return dbmdx_reg_store(dev, DBMDX_REGN_MICROPHONE_ANALOG_GAIN, attr,
			       buf, size, DBMDX_FW_VA);
}

static ssize_t dbmdx_va_backlog_size_show(struct device *dev,
				      struct device_attribute *attr,
				      char *buf)
{
	struct dbmdx_private *p = dev_get_drvdata(dev);
	ssize_t off = 0;

	if (!p)
		return -EAGAIN;

	if (!p->device_ready) {
		dev_err(p->dev, "%s: device not ready\n", __func__);
		return -EAGAIN;
	}

#ifdef DMBDX_GENERAL_AMODEL_SUPPORT
	off += snprintf(buf + off, PAGE_SIZE - off,
			"Backlog length: %d\n",
			p->pdata->va_backlog_length);
#endif
#ifdef DMBDX_SENSORY_AMODEL_SUPPORT
	off += snprintf(buf + off, PAGE_SIZE - off,
			"Sensory Backlog length: %d\n",
			p->pdata->va_backlog_length_sensory);
#endif
#ifdef DMBDX_OKG_AMODEL_SUPPORT
	off += snprintf(buf + off, PAGE_SIZE - off,
			"Google Backlog length: %d\n",
			p->pdata->va_backlog_length_google);
#endif
#ifdef DMBDX_ALEXA_AMODEL_SUPPORT
	off += snprintf(buf + off, PAGE_SIZE - off,
			"Alexa Backlog length: %d\n",
			p->pdata->va_backlog_length_alexa);
#endif
#ifdef DMBDX_BIXBY_AMODEL_SUPPORT
	off += snprintf(buf + off, PAGE_SIZE - off,
			"Bixby Backlog length: %d\n",
			p->pdata->va_backlog_length_bixby);
#endif

	return off;
}

static ssize_t dbmdx_va_backlog_size_store(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf,
				       size_t size)
{
	return dbmdx_reg_store(dev,
			DBMDX_REGN_VT_AUDIO_HISTORY_CFG, attr, buf,
			size, DBMDX_FW_VA);
}

static ssize_t dbmdx_reset_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t size)
{
	struct dbmdx_private *p = dev_get_drvdata(dev);
	int ret;
	unsigned long val;

	if (!p)
		return -EAGAIN;

	ret = kstrtoul(buf, 0, &val);
	if (ret)
		return -EINVAL;

	dev_dbg(p->dev, "%s: Val = %d\n", __func__, (int)val);

	if (val == 0) {
		ret = dbmdx_perform_recovery(p);
		if (ret) {
			dev_err(p->dev, "%s: recovery failed\n", __func__);
			return -EIO;
		}
	} else if (val == 1) {
		p->va_flags.buffering = 0;
		flush_work(&p->sv_work);

		ret = dbmdx_suspend_pcm_streaming_work(p);
		if (ret < 0)
			dev_err(p->dev,
				"%s: Failed to suspend PCM Streaming Work\n",
				__func__);

		p->device_ready = false;

		dbmdx_reset_set(p);

		dev_info(p->dev, "%s: DBMDX Chip is in Reset state\n",
			__func__);

	} else
		return -EINVAL;

	return size;
}

static ssize_t dbmdx_param_addr_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t size)
{
	struct dbmdx_private *p = dev_get_drvdata(dev);
	unsigned long val;
	int ret;

	if (!p)
		return -EAGAIN;

	if (!p->device_ready) {
		dev_err(p->dev, "%s: device not ready\n", __func__);
		return -EAGAIN;
	}

	ret = kstrtoul(buf, 0, &val);
	if (ret)
		return -EINVAL;

	p->lock(p);
	ret = dbmdx_switch_to_user_selected_chip_interface(p);
	if (ret) {
		p->unlock(p);
		dev_err(p->dev, "%s: Error switching to usr sel. chip iface\n",
			__func__);
		return -EIO;
	}

	ret = dbmdx_wake(p);
	if (ret < 0) {
		dev_err(p->dev, "%s: unable to wake\n", __func__);
		p->unlock(p);
		return ret;
	}

	ret = dbmdx_send_cmd(p,
			DBMDX_REGN_INDIRECT_ACCESS_REG_NUMBER | (u32)val,
			NULL);

	if (ret < 0) {
		dev_err(p->dev, "%s: set paramaddr error\n", __func__);
		p->unlock(p);
		return ret;
	}
	ret = dbmdx_switch_to_drv_selected_chip_interface(p);
	if (ret) {
		p->unlock(p);
		dev_err(p->dev, "%s: Error switching to drv sel. chip iface\n",
			__func__);
		return -EIO;
	}
	p->unlock(p);
	return size;
}

static ssize_t dbmdx_param_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct dbmdx_private *p = dev_get_drvdata(dev);
	int ret;
	u16 val;

	if (!p)
		return -EAGAIN;

	if (!p->device_ready) {
		dev_err(p->dev, "%s: device not ready\n", __func__);
		return -EAGAIN;
	}

	p->lock(p);
	ret = dbmdx_switch_to_user_selected_chip_interface(p);
	if (ret) {
		p->unlock(p);
		return snprintf(buf, PAGE_SIZE,
				"Error switching to user sel. chip iface\n");
	}
	ret = dbmdx_wake(p);
	if (ret < 0) {
		dev_err(p->dev, "%s: unable to wake\n", __func__);
		p->unlock(p);
		return ret;
	}

	ret = dbmdx_send_cmd(p,
			DBMDX_REGN_INDIRECT_ACCESS_REG_READ, &val);

	if (ret < 0) {
		dev_err(p->dev, "%s: get param error\n", __func__);
		p->unlock(p);
		return ret;
	}
	ret = dbmdx_switch_to_drv_selected_chip_interface(p);
	if (ret) {
		p->unlock(p);
		return snprintf(buf, PAGE_SIZE,
				"Error switching to drv sel. chip iface\n");
	}

	p->unlock(p);
	return snprintf(buf, PAGE_SIZE, "%d\n", (s16)val);
}

static ssize_t dbmdx_param_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t size)
{
	struct dbmdx_private *p = dev_get_drvdata(dev);
	unsigned long val;
	int ret;

	if (!p)
		return -EAGAIN;

	if (!p->device_ready) {
		dev_err(p->dev, "%s: device not ready\n", __func__);
		return -EAGAIN;
	}

	ret = kstrtol(buf, 0, &val);
	if (ret)
		return -EINVAL;

	p->lock(p);
	ret = dbmdx_switch_to_user_selected_chip_interface(p);
	if (ret) {
		p->unlock(p);
		dev_err(p->dev, "%s: Error switching to user sel. chip iface\n",
			__func__);
		return -EIO;
	}
	ret = dbmdx_wake(p);
	if (ret < 0) {
		dev_err(p->dev, "%s: unable to wake\n", __func__);
		p->unlock(p);
		return ret;
	}

	ret = dbmdx_send_cmd(p, DBMDX_REGN_INDIRECT_ACCESS_REG_WRITE |
							(u32)val, NULL);

	if (ret < 0) {
		dev_err(p->dev, "%s: set param error\n", __func__);
		p->unlock(p);
		return ret;
	}

	ret = dbmdx_switch_to_drv_selected_chip_interface(p);
	if (ret) {
		p->unlock(p);
		dev_err(p->dev, "%s: Error switching to drv sel. chip iface\n",
			__func__);
		return -EIO;
	}
	p->unlock(p);
	return size;
}

static ssize_t dbmdx_va_direct_write_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t size)
{
	struct dbmdx_private *p = dev_get_drvdata(dev);
	unsigned long val;
	char *str_p;
	char *args =  (char *)buf;
	u32 reg = 0;
	u32 new_value = 0;
	bool reg_set = false, value_set = false;
	u32 value_to_send = 0;

	int ret = -EINVAL;

	if (!p)
		return -EAGAIN;

	if (!p->device_ready) {
		dev_err(p->dev, "%s: device not ready\n", __func__);
		return -EAGAIN;
	}

	while ((str_p = strsep(&args, " \t")) != NULL) {

		if (!*str_p)
			continue;

		if (strncmp(str_p, "reg=", 4) == 0) {
			ret = kstrtoul((str_p+4), 0, &val);
			if (ret) {
				dev_err(p->dev, "%s: bad reg\n", __func__);
				ret = -EINVAL;
				goto print_usage;
			}
			reg = (u32)val;
			reg_set = true;
			continue;
		}
		if (strncmp(str_p, "value=", 6) == 0) {
			ret = kstrtoul((str_p+6), 0, &val);
			if (ret) {
				dev_err(p->dev, "%s: bad value\n", __func__);
				ret = -EINVAL;
				goto print_usage;
			}

			new_value = (u32)val;
			value_set = true;
			continue;
		}
	}

	if (!reg_set) {
		dev_err(p->dev, "%s: reg is not set\n", __func__);
		ret = -EINVAL;
		goto print_usage;
	} else if (!value_set) {
		dev_err(p->dev, "%s: value is not set\n", __func__);
		ret = -EINVAL;
		goto print_usage;
	}

	p->lock(p);

	ret = dbmdx_switch_to_user_selected_chip_interface(p);
	if (ret) {
		p->unlock(p);
		dev_err(p->dev, "%s: Error switching to user sel. chip iface\n",
			__func__);
		return -EIO;
	}

	ret = dbmdx_wake(p);
	if (ret < 0) {
		dev_err(p->dev, "%s: unable to wake\n", __func__);
		p->unlock(p);
		return ret;
	}
	if (reg > 0x00ff)
		ret = dbmdx_indirect_register_write(p, reg, (u16)new_value);
	else {
		value_to_send = DBMDX_VA_CMD_MASK |
			((reg & 0xff) << 16) | (new_value & 0xffff);

		ret = dbmdx_send_cmd(p, (u32)value_to_send, NULL);
	}

	if (ret < 0) {
		dev_err(p->dev, "%s: direct write error\n", __func__);
		p->unlock(p);
		return ret;
	}

	ret = dbmdx_switch_to_drv_selected_chip_interface(p);
	if (ret) {
		p->unlock(p);
		dev_err(p->dev, "%s: Error switching to drv sel. chip iface\n",
			__func__);
		return -EIO;
	}
	p->unlock(p);

	dev_dbg(dev, "%s: Reg 0x%04x was set to 0x%04x\n",
			__func__, reg, new_value);

	return size;
print_usage:
	dev_info(p->dev,
		"%s: Usage: reg=regaddr value=newval\n", __func__);
	return ret;
}

static ssize_t dbmdx_va_direct_read_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t size)
{
	struct dbmdx_private *p = dev_get_drvdata(dev);
	unsigned long val;
	char *str_p;
	char *args =  (char *)buf;
	u32 reg = 0;
	bool reg_set = false;
	u32 value_to_send = 0;
	u16 resp;

	int ret = -EINVAL;

	if (!p)
		return -EAGAIN;

	if (!p->device_ready) {
		dev_err(p->dev, "%s: device not ready\n", __func__);
		return -EAGAIN;
	}

	while ((str_p = strsep(&args, " \t")) != NULL) {

		if (!*str_p)
			continue;

		if (strncmp(str_p, "reg=", 4) == 0) {
			ret = kstrtoul((str_p+4), 0, &val);
			if (ret) {
				dev_err(p->dev, "%s: bad reg\n", __func__);
				ret = -EINVAL;
				goto print_usage;
			}
			reg = (u32)val;
			reg_set = true;
			continue;
		}
	}

	if (!reg_set) {
		dev_err(p->dev, "%s: reg is not set\n", __func__);
		ret = -EINVAL;
		goto print_usage;
	}

	p->lock(p);

	ret = dbmdx_switch_to_user_selected_chip_interface(p);
	if (ret) {
		p->unlock(p);
		dev_err(p->dev, "%s: Error switching to usr sel. chip iface\n",
			__func__);
		return -EIO;
	}

	ret = dbmdx_wake(p);
	if (ret < 0) {
		dev_err(p->dev, "%s: unable to wake\n", __func__);
		p->unlock(p);
		return ret;
	}

	if (reg > 0x00ff)
		ret = dbmdx_indirect_register_read(p, reg, &resp);
	else {
		value_to_send = DBMDX_VA_CMD_MASK | ((reg & 0xff) << 16);

		ret = dbmdx_send_cmd(p, (u32)value_to_send, &resp);
	}

	if (ret < 0) {
		dev_err(p->dev, "%s: direct read error\n", __func__);
		p->unlock(p);
		return ret;
	}

	ret = dbmdx_switch_to_drv_selected_chip_interface(p);
	if (ret) {
		p->unlock(p);
		dev_err(p->dev, "%s: Error switching to drv sel. chip iface\n",
			__func__);
		return -EIO;
	}

	p->unlock(p);

	dev_dbg(dev, "%s: Reg 0x%04x value is 0x%04x\n",
			__func__, reg, resp);

	return size;
print_usage:
	dev_info(p->dev,
		"%s: Usage: reg=regaddr\n", __func__);
	return ret;
}

static ssize_t dbmdx_va_min_samples_chunk_size_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct dbmdx_private *p = dev_get_drvdata(dev);
	int ret;

	if (!p)
		return -EAGAIN;

	if (!p->device_ready) {
		dev_err(p->dev, "%s: device not ready\n", __func__);
		return -EAGAIN;
	}

	ret = snprintf(buf, PAGE_SIZE, "%u\n",
		       p->pdata->min_samples_chunk_size);

	return ret;
}

static ssize_t dbmdx_va_min_samples_chunk_size_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t size)
{
	struct dbmdx_private *p = dev_get_drvdata(dev);
	unsigned long val;
	int ret;

	ret = kstrtol(buf, 0, &val);
	if (ret)
		return -EINVAL;

	dev_dbg(p->dev, "%s: val - %d\n", __func__, (int)val);

	if (val < 0) {
		dev_err(p->dev,
			"%s: invalid min_samples_chunk_size value %d\n",
			__func__, (int)(val));

		return -EINVAL;
	}

	p->pdata->min_samples_chunk_size = (u32)val;

	return size;
}

static ssize_t dbmdx_va_max_detection_buffer_size_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct dbmdx_private *p = dev_get_drvdata(dev);
	int ret;

	if (!p)
		return -EAGAIN;

	if (!p->device_ready) {
		dev_err(p->dev, "%s: device not ready\n", __func__);
		return -EAGAIN;
	}

	ret = snprintf(buf, PAGE_SIZE, "%u\n",
		       p->pdata->max_detection_buffer_size);

	return ret;
}

static ssize_t dbmdx_va_max_detection_buffer_size_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t size)
{
	struct dbmdx_private *p = dev_get_drvdata(dev);
	unsigned long val;
	int ret;

	ret = kstrtol(buf, 0, &val);
	if (ret)
		return -EINVAL;

	dev_dbg(p->dev, "%s: val - %d\n", __func__, (int)val);

	if (val < 0) {
		dev_err(p->dev,
			"%s: invalid max_detection_buffer_size value %d\n",
			__func__, (int)(val));

		return -EINVAL;
	}

	p->pdata->max_detection_buffer_size = (u32)val;

	return size;
}

static ssize_t dbmdx_dump_state(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct dbmdx_private *p = dev_get_drvdata(dev);
	int off = 0;
	struct dbmdx_platform_data *pdata;
	int i;

	if (!p)
		return -EAGAIN;

	pdata = p->pdata;

	off += snprintf(buf + off, PAGE_SIZE - off,
				"\tDBMDX Driver Ver:\t%s\n", DRIVER_VERSION);

	off += snprintf(buf + off, PAGE_SIZE - off,
					"\t=======Interfaces Dump======\n");

	off += snprintf(buf + off, PAGE_SIZE - off,
					"\tMulti Interface Support:\t%s\n",
			p->pdata->multi_interface_support ? "ON" : "OFF");
	off += snprintf(buf + off, PAGE_SIZE - off,
				"\tNumber of supported interfaces:\t%d\n",
				p->nr_of_interfaces);

	for (i = 0; i < p->nr_of_interfaces; i++) {
		off += snprintf(buf + off, PAGE_SIZE - off,
				"\tInterface #%d:\n",
				i);
		if (p->interfaces[i]->dump)
			off += p->interfaces[i]->dump(p->interfaces[i],
				buf + off);
	}

	if (p->pdata->multi_interface_support) {
		off += snprintf(buf + off, PAGE_SIZE - off,
					"\t=======VA Interfaces======\n");

		off += snprintf(buf + off, PAGE_SIZE - off,
		"\tPreboot:\t%d\n\tBoot:\t%d\n\tCommand:\t%d\n\tDebug:\t%d\n",
			p->pdata->va_interfaces[DBMDX_PREBOOT_INTERFACE],
			p->pdata->va_interfaces[DBMDX_BOOT_INTERFACE],
			p->pdata->va_interfaces[DBMDX_CMD_INTERFACE],
			p->pdata->va_interfaces[DBMDX_DEBUG_INTERFACE]);
	}

	if (p->pdata->multi_interface_support) {
		off += snprintf(buf + off, PAGE_SIZE - off,
					"\t=======VA_VE Interfaces======\n");

		off += snprintf(buf + off, PAGE_SIZE - off,
		"\tPreboot:\t%d\n\tBoot:\t%d\n\tCommand:\t%d\n\tDebug:\t%d\n",
			p->pdata->va_ve_interfaces[DBMDX_PREBOOT_INTERFACE],
			p->pdata->va_ve_interfaces[DBMDX_BOOT_INTERFACE],
			p->pdata->va_ve_interfaces[DBMDX_CMD_INTERFACE],
			p->pdata->va_ve_interfaces[DBMDX_DEBUG_INTERFACE]);
	}

	/* check for features */
	if (p->pdata->feature_va || p->pdata->feature_va_ve) {
		off += snprintf(buf + off, PAGE_SIZE - off,
				"\t=======Initial VA Configuration======\n");


		off += snprintf(buf + off, PAGE_SIZE - off,
						"\tVA feature activated\n");
		off += snprintf(buf + off, PAGE_SIZE - off,
			"\tVA firmware name: %s\n", pdata->va_firmware_name);

		for (i = 0; i < pdata->va_cfg_values; i++)
			off += snprintf(buf + off, PAGE_SIZE - off,
						"\tVA cfg %8.8x: 0x%8.8x\n",
				i, pdata->va_cfg_value[i]);

		for (i = 0; i < VA_MIC_CONFIG_SIZE; i++)
			off += snprintf(buf + off, PAGE_SIZE - off,
					"\tVA mic cfg %8.8x: 0x%8.8x\n",
					i, pdata->va_mic_config[i]);

		off += snprintf(buf + off, PAGE_SIZE - off,
				"\tVA default mic config: 0x%8.8x\n",
				pdata->va_initial_mic_config);

		off += snprintf(buf + off, PAGE_SIZE - off,
			"\tVA init digital mic digital gain:\t0x%04X\n",
		 pdata->va_mic_gain_config[DBMDX_DIGITAL_MIC_DIGITAL_GAIN]);

		off += snprintf(buf + off, PAGE_SIZE - off,
			"\tVA init analog mic analog gain:\t\t0x%04X\n",
		 pdata->va_mic_gain_config[DBMDX_ANALOG_MIC_ANALOG_GAIN]);

		off += snprintf(buf + off, PAGE_SIZE - off,
			"\tVA init analog mic digital gain:\t0x%04X\n",
		 pdata->va_mic_gain_config[DBMDX_ANALOG_MIC_DIGITAL_GAIN]);

#ifdef DMBDX_GENERAL_AMODEL_SUPPORT
		off += snprintf(buf + off, PAGE_SIZE - off,
				"\tBacklog length: %d\n",
				p->pdata->va_backlog_length);
#endif
#ifdef DMBDX_SENSORY_AMODEL_SUPPORT
		off += snprintf(buf + off, PAGE_SIZE - off,
				"\tSensory Backlog length: %d\n",
				p->pdata->va_backlog_length_sensory);
#endif
#ifdef DMBDX_OKG_AMODEL_SUPPORT
		off += snprintf(buf + off, PAGE_SIZE - off,
				"\tGoogle Backlog length: %d\n",
				p->pdata->va_backlog_length_google);
#endif
#ifdef DMBDX_ALEXA_AMODEL_SUPPORT
		off += snprintf(buf + off, PAGE_SIZE - off,
				"\tAlexa Backlog length: %d\n",
				p->pdata->va_backlog_length_alexa);
#endif
#ifdef DMBDX_BIXBY_AMODEL_SUPPORT
		off += snprintf(buf + off, PAGE_SIZE - off,
				"\tBixby Backlog length: %d\n",
				p->pdata->va_backlog_length_bixby);
#endif

		off += snprintf(buf + off, PAGE_SIZE - off,
			"\tauto_buffering %d\n", pdata->auto_buffering);

		off += snprintf(buf + off, PAGE_SIZE - off,
			"\tauto_detection %d\n", pdata->auto_detection);

		off += snprintf(buf + off, PAGE_SIZE - off,
			"\tdetection_after_buffering %d\n",
			pdata->detection_after_buffering);

		off += snprintf(buf + off, PAGE_SIZE - off,
					"\tsend_uevent_on_detection %d\n",
				pdata->send_uevent_on_detection);

		off += snprintf(buf + off, PAGE_SIZE - off,
					"\tsend_uevent_after_buffering %d\n",
				pdata->send_uevent_after_buffering);

		off += snprintf(buf + off, PAGE_SIZE - off,
					"\tbuffering_timeout %d\n",
					pdata->buffering_timeout);

		off += snprintf(buf + off, PAGE_SIZE - off,
						"\tpcm streaming mode %d\n",
						pdata->pcm_streaming_mode);

		off += snprintf(buf + off, PAGE_SIZE - off,
					"\tmax_detection_buffer_size %d\n",
					pdata->max_detection_buffer_size);

		off += snprintf(buf + off, PAGE_SIZE - off,
					"\tmin_samples_chunk_size %d\n",
					pdata->min_samples_chunk_size);

		off += snprintf(buf + off, PAGE_SIZE - off,
					"\tdetection_buffer_channels %d\n",
					pdata->detection_buffer_channels);

		off += snprintf(buf + off, PAGE_SIZE - off,
						"\tva_buffering_pcm_rate %u\n",
					pdata->va_buffering_pcm_rate);

	} else
		off += snprintf(buf + off, PAGE_SIZE - off,
					"\tVA feature not activated\n");
	if (p->pdata->feature_va_ve) {
		off += snprintf(buf + off, PAGE_SIZE - off,
				"\t=======Initial VA_VE Configuration======\n");


		off += snprintf(buf + off, PAGE_SIZE - off,
						"\tVA_VE feature activated\n");
		off += snprintf(buf + off, PAGE_SIZE - off,
			"\tVA_VE firmware name: %s\n",
				pdata->va_ve_firmware_name);

		for (i = 0; i < pdata->va_ve_cfg_values; i++)
			off += snprintf(buf + off, PAGE_SIZE - off,
						"\tVA_VE cfg %8.8x: 0x%8.8x\n",
				i, pdata->va_ve_cfg_value[i]);

		for (i = 0; i < VA_VE_MIC_CONFIG_SIZE; i++)
			off += snprintf(buf + off, PAGE_SIZE - off,
					"\tVA_VE mic cfg %8.8x: 0x%8.8x\n",
					i, pdata->va_ve_mic_config[i]);

		off += snprintf(buf + off, PAGE_SIZE - off,
					"\tVA_VE mic mask 0x%8.8x\n",
					pdata->va_ve_mic_mask);

		off += snprintf(buf + off, PAGE_SIZE - off,
					"\tASRP Delay 0x%8.8x\n",
					pdata->asrp_delay);

		off += snprintf(buf + off, PAGE_SIZE - off,
					"\tASRP TX OUT Gain 0x%8.8x\n",
					pdata->asrp_tx_out_gain);

		off += snprintf(buf + off, PAGE_SIZE - off,
					"\tASRP VCPF OUT Gain 0x%8.8x\n",
					pdata->asrp_vcpf_out_gain);

		off += snprintf(buf + off, PAGE_SIZE - off,
					"\tASRP RX OUT Gain 0x%8.8x\n",
					pdata->asrp_rx_out_gain);

	} else
		off += snprintf(buf + off, PAGE_SIZE - off,
					"\tVA_VE feature not activated\n");

	off += snprintf(buf + off, PAGE_SIZE - off,
				"\t=======Common Configuration======\n");

	off += snprintf(buf + off, PAGE_SIZE - off,
			"\tHW Revision 0x%8x\n", pdata->hw_rev);

	for (i = 0; i < DBMDX_VA_NR_OF_SPEEDS; i++)
		off += snprintf(buf + off, PAGE_SIZE - off,
				"\tVA speed cfg %8.8x: 0x%8.8x %u %u %u\n",
				i,
				pdata->va_speed_cfg[i].cfg,
				pdata->va_speed_cfg[i].uart_baud,
				pdata->va_speed_cfg[i].i2c_rate,
				pdata->va_speed_cfg[i].spi_rate);



	off += snprintf(buf + off, PAGE_SIZE - off,
				"\tmaster-clk-rate of %dHZ\n",
				(int)(p->clk_get_rate(p, DBMDX_CLK_MASTER)));
	if (p->pdata->feature_va_ve && p->pdata->va_ve_separate_clocks) {
		off += snprintf(buf + off, PAGE_SIZE - off,
				"\tmaster_va_ve-clk-rate of %dHZ\n",
				(int)(p->clk_get_rate(p,
						DBMDX_CLK_MASTER_VA_VE)));

	}
	off += snprintf(buf + off, PAGE_SIZE - off,
					"\tuart_low_speed_enabled %d\n",
					pdata->uart_low_speed_enabled);

	off += snprintf(buf + off, PAGE_SIZE - off,
			"\tBoot options 0x%8x\n", pdata->boot_options);

	off += snprintf(buf + off, PAGE_SIZE - off,
			"\tAmodel options 0x%8x\n", pdata->amodel_options);

	off += snprintf(buf + off, PAGE_SIZE - off, "\tVA firmware_id 0x%8x\n",
		 p->pdata->firmware_id);

	if (p->pdata->feature_va_ve) {
		off += snprintf(buf + off, PAGE_SIZE - off,
			"\tBoot options va_ve 0x%8x\n",
			pdata->boot_options_va_ve);

		off += snprintf(buf + off, PAGE_SIZE - off,
			"\tVA_VE firmware_id 0x%8x\n",
			p->pdata->firmware_id_va_ve);

	}

	off += snprintf(buf + off, PAGE_SIZE - off,
					"\t=======GPIO======\n");

	off += snprintf(buf + off, PAGE_SIZE - off,
					"\t=====VA GPIO====\n");

	off += snprintf(buf + off, PAGE_SIZE - off,
			"\tReset GPIO: 0x%8x\n", p->pdata->gpio_reset);

	off += snprintf(buf + off, PAGE_SIZE - off,
			"\tWakeup GPIO: 0x%8x\n", p->pdata->gpio_wakeup);

	off += snprintf(buf + off, PAGE_SIZE - off,
			"\tSV GPIO: 0x%8x\n", p->pdata->gpio_sv);

	off += snprintf(buf + off, PAGE_SIZE - off,
					"\twakeup_disabled %d\n",
					pdata->wakeup_disabled);

	off += snprintf(buf + off, PAGE_SIZE - off,
					"\tuse_gpio_for_wakeup %d\n",
					pdata->use_gpio_for_wakeup);

	off += snprintf(buf + off, PAGE_SIZE - off, "\tsend_wakeup_seq %d\n",
		 pdata->send_wakeup_seq);

	off += snprintf(buf + off, PAGE_SIZE - off, "\twakeup_set_value %d\n",
		 pdata->wakeup_set_value);

	if (p->pdata->feature_va_ve) {

		off += snprintf(buf + off, PAGE_SIZE - off,
					"\t=====VA_VE GPIO====\n");


		off += snprintf(buf + off, PAGE_SIZE - off,
			"\tReset GPIO: 0x%8x\n", p->pdata->gpio_reset_va_ve);

		off += snprintf(buf + off, PAGE_SIZE - off,
			"\tWakeup GPIO: 0x%8x\n", p->pdata->gpio_wakeup_va_ve);

		off += snprintf(buf + off, PAGE_SIZE - off,
			"\tReset GPIO shared %d\n",
			pdata->reset_gpio_shared);

		off += snprintf(buf + off, PAGE_SIZE - off,
					"\twakeup_va_ve_disabled %d\n",
					pdata->wakeup_va_ve_disabled);

		off += snprintf(buf + off, PAGE_SIZE - off,
					"\tuse_gpio_for_wakeup va_ve %d\n",
					pdata->use_gpio_for_wakeup_va_ve);

		off += snprintf(buf + off, PAGE_SIZE - off,
			"\tsend_wakeup_seq va_ve %d\n",
			pdata->send_wakeup_va_ve_seq);

		off += snprintf(buf + off, PAGE_SIZE - off,
			"\twakeup_set_value va_ve %d\n",
			pdata->wakeup_va_ve_set_value);

		off += snprintf(buf + off, PAGE_SIZE - off,
			"\tTDM ENABLE: 0x%8x\n", p->tdm_enable);

		off += snprintf(buf + off, PAGE_SIZE - off,
				"\tVA Default Clock %u Hz\n",
				pdata->default_va_clock);

		off += snprintf(buf + off, PAGE_SIZE - off,
				"\tVA_VE Default Clock %u Hz\n",
				pdata->default_va_ve_clock);

		off += snprintf(buf + off, PAGE_SIZE - off,
				"\tDBMD4: Switch to the Host Clock:\t%s\n",
				(p->pdata->change_clock_src_enabled &
					DBMDX_CHANGE_TO_HOST_CLOCK_D4_ENABLED) ?
				"ON" : "OFF");
		off += snprintf(buf + off, PAGE_SIZE - off,
				"\tDBMD4: Switch to the Internal Clock:\t%s\n",
				(p->pdata->change_clock_src_enabled &
				DBMDX_CHANGE_TO_INTERNAL_CLOCK_D4_ENABLED) ?
				"ON" : "OFF");
		off += snprintf(buf + off, PAGE_SIZE - off,
				"\tDBMD2: Switch to the host Clock:\t%s\n",
				(p->pdata->change_clock_src_enabled &
					DBMDX_CHANGE_TO_HOST_CLOCK_D2_ENABLED) ?
				"ON" : "OFF");
		off += snprintf(buf + off, PAGE_SIZE - off,
				"\tDBMD2: Switch to the Internal Clock:\t%s\n",
				(p->pdata->change_clock_src_enabled &
				DBMDX_CHANGE_TO_INTERNAL_CLOCK_D2_ENABLED) ?
				"ON" : "OFF");
	}

	return off;
}

static ssize_t dbmdx_dump_current_state(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct dbmdx_private *p = dev_get_drvdata(dev);
	int off = 0;
	struct dbmdx_platform_data *pdata;

	if (!p)
		return -EAGAIN;

	pdata = p->pdata;

	off += snprintf(buf + off, PAGE_SIZE - off,
				"\tDBMDX Driver Ver:\t%s\n", DRIVER_VERSION);

	off += snprintf(buf + off, PAGE_SIZE - off, "\tActive firmware:\t%s\n",
		p->active_fw == DBMDX_FW_VA_VE ?
		"VA_VE" :
		p->active_fw == DBMDX_FW_VA ?
		"VA" : "PRE_BOOT");

	off += snprintf(buf + off,  PAGE_SIZE - off, "\tPower mode:\t\t%s\n",
				dbmdx_power_mode_names[p->power_mode]);
	off += snprintf(buf + off, PAGE_SIZE - off,
				"\tActive Interface :\t%s\n",
				p->active_interface == DBMDX_INTERFACE_I2C ?
				"I2C" :
				p->active_interface ==
					DBMDX_INTERFACE_SPI ?
				"SPI" :
				p->active_interface ==
					DBMDX_INTERFACE_UART ?
				"UART" : "NONE");

	off += snprintf(buf + off, PAGE_SIZE - off, "\tVA Device Ready:\t%s\n",
				p->device_ready ? "Yes" : "No");

	off += snprintf(buf + off, PAGE_SIZE - off,
					"\tVA Debug Mode:\t%s\n",
			p->va_debug_mode ? "ON" : "OFF");
	off += snprintf(buf + off, PAGE_SIZE - off,
					"\tVA Sleep disabled:\t%s\n",
			p->sleep_disabled ? "ON" : "OFF");
	off += snprintf(buf + off, PAGE_SIZE - off,
					"\tRecovery Disabled:\t%s\n",
			p->pdata->va_recovery_disabled ? "ON" : "OFF");
	if (p->pdata->feature_va || p->pdata->feature_va_ve) {
		off += snprintf(buf + off, PAGE_SIZE - off,
					"\t=======VA Dump==========\n");
		off += snprintf(buf + off, PAGE_SIZE - off,
				"\t-------VA Current Settings------\n");
#ifdef DMBDX_GENERAL_AMODEL_SUPPORT
		off += snprintf(buf + off, PAGE_SIZE - off,
				"\tBacklog length: %d\n",
				p->pdata->va_backlog_length);
#endif
#ifdef DMBDX_SENSORY_AMODEL_SUPPORT
		off += snprintf(buf + off, PAGE_SIZE - off,
				"\tSensory Backlog length: %d\n",
				p->pdata->va_backlog_length_sensory);
#endif
#ifdef DMBDX_OKG_AMODEL_SUPPORT
		off += snprintf(buf + off, PAGE_SIZE - off,
				"\tGoogle Backlog length: %d\n",
				p->pdata->va_backlog_length_google);
#endif
#ifdef DMBDX_ALEXA_AMODEL_SUPPORT
		off += snprintf(buf + off, PAGE_SIZE - off,
				"\tAlexa Backlog length: %d\n",
				p->pdata->va_backlog_length_alexa);
#endif
#ifdef DMBDX_BIXBY_AMODEL_SUPPORT
		off += snprintf(buf + off, PAGE_SIZE - off,
				"\tBixby Backlog length: %d\n",
				p->pdata->va_backlog_length_bixby);
#endif
		off += snprintf(buf + off, PAGE_SIZE - off,
					"\tVA Active Microphone conf:\t%d\n",
				p->va_active_mic_config);
		off += snprintf(buf + off, PAGE_SIZE - off,
					"\tVA Cur Microphone conf:\t%d\n",
				p->va_current_mic_config);
		off += snprintf(buf + off, PAGE_SIZE - off,
					"\tVA Microphones Enabled:\t%s\n",
			p->va_flags.microphones_enabled ? "ON" : "OFF");
		off += snprintf(buf + off, PAGE_SIZE - off,
				"\tVA Disabling Mics Not Allowed:\t%s\n",
			p->va_flags.disabling_mics_not_allowed ? "ON" : "OFF");
		off += snprintf(buf + off, PAGE_SIZE - off,
				"\tVA Disabling Mics Blocked:\t%s\n",
			p->mic_disabling_blocked ? "ON" : "OFF");
		off += snprintf(buf + off, PAGE_SIZE - off,
						"\tVA Audio Channels:\t%d\n",
				p->pdata->va_audio_channels);
		off += snprintf(buf + off, PAGE_SIZE - off,
					"\tVA PCM Audio Channels:\t%d\n",
				p->audio_pcm_channels);
		off += snprintf(buf + off, PAGE_SIZE - off,
					"\tVA PCM Channel Operation:\t%d\n",
				p->pcm_achannel_op);
		off += snprintf(buf + off, PAGE_SIZE - off,
				"\tVA Detection Channel Operation:\t%d\n",
				p->detection_achannel_op);
		off += snprintf(buf + off, PAGE_SIZE - off,
					"\tVA Detection KFIFO size:\t%d\n",
					p->detection_samples_kfifo_buf_size);
		off += snprintf(buf + off, PAGE_SIZE - off,
					"\tVA Capture On Detect:\t%s\n",
			p->va_capture_on_detect ? "ON" : "OFF");
		off += snprintf(buf + off, PAGE_SIZE - off,
			"\tSample size:\t\t%d\n", p->audio_mode);

		off += snprintf(buf + off, PAGE_SIZE - off,
			"\tVA current digital mic digital gain:\t0x%04X\n",
			p->va_cur_digital_mic_digital_gain);

		off += snprintf(buf + off, PAGE_SIZE - off,
			"\tVA current analog mic analog gain:\t0x%04X\n",
			p->va_cur_analog_mic_analog_gain);

		off += snprintf(buf + off, PAGE_SIZE - off,
			"\tVA current analog mic digital gain:\t0x%04X\n",
			p->va_cur_analog_mic_digital_gain);

		off += snprintf(buf + off, PAGE_SIZE - off,
						"\t-------VA Status------\n");
		off += snprintf(buf + off, PAGE_SIZE - off,
						"\tVA Operational mode:\t%s\n",
				dbmdx_state_names[p->va_flags.mode]);
		off += snprintf(buf + off, PAGE_SIZE - off,
				"\tFW VAD TYPE:\t\t%d\n", p->fw_vad_type);
		off += snprintf(buf + off, PAGE_SIZE - off,
						"\tCurrent PCM rate:\t\t%d\n",
				p->current_pcm_rate);
		off += snprintf(buf + off, PAGE_SIZE - off,
				"\tBuffering:\t\t%s\n",
			p->va_flags.buffering ? "ON" : "OFF");
		off += snprintf(buf + off, PAGE_SIZE - off,
					"\tPCM Streaming Active:\t%s\n",
			p->va_flags.pcm_streaming_active ? "ON" : "OFF");
		off += snprintf(buf + off, PAGE_SIZE - off,
						"\tPCM Worker Active:	\t%s\n",
			p->va_flags.pcm_worker_active ? "ON" : "OFF");
		off += snprintf(buf + off, PAGE_SIZE - off,
						"\tIRQ In USE:\t\t%s\n",
			p->va_flags.irq_inuse ? "ON" : "OFF");
	}
	if (p->pdata->feature_va_ve) {
		off += snprintf(buf + off, PAGE_SIZE - off,
					"\t=======VA_VE Dump==========\n");
		off += snprintf(buf + off, PAGE_SIZE - off,
				"\t-------VA_VE Current Settings------\n");
		off += snprintf(buf + off, PAGE_SIZE - off,
					"\tDriver selected chip:\t\t%s\n",
			(p->va_ve_flags.drv_seletected_chip ==
				DBMDX_CHIP_VA_VE) ? "VA_VE" : "VA");
		off += snprintf(buf + off, PAGE_SIZE - off,
					"\tUser selected chip:\t\t%s\n",
			(p->usr_selected_chip ==
				DBMDX_CHIP_VA_VE) ? "VA_VE" : "VA");
		off += snprintf(buf + off, PAGE_SIZE - off,
					"\tActive Usecase ID:\t%d\n",
				p->va_ve_flags.active_usecase_id);
		off += snprintf(buf + off, PAGE_SIZE - off,
				"\tActive Usecase Name:\t%s\n",
				p->va_ve_flags.cur_usecase == NULL ? "NONE" :
				p->va_ve_flags.cur_usecase->usecase_name);
		off += snprintf(buf + off, PAGE_SIZE - off,
						"\tUsecase mode:\t%s\n",
				p->va_ve_flags.cur_usecase == NULL ? "NONE" :
				dbmdx_state_names[p->va_ve_flags.usecase_mode]);

		off += snprintf(buf + off, PAGE_SIZE - off,
						"\tReference Playback:	\t%s\n",
			p->reference_playback_active ? "ON" : "OFF");


	}
	return off;
}

static ssize_t dbmdx_dump_amodel_info(struct dbmdx_private *p,
					struct amodel_info *amodel,
					const char *model_name,
					int off, char *buf,
					int *updated_off)
{
	if (!p)
		return -EAGAIN;

	if (!amodel)
		return 0;

	off += snprintf(buf + off, PAGE_SIZE - off,
					"\t\tAmodel Type:\t%s\n",
					model_name);

	off += snprintf(buf + off, PAGE_SIZE - off,
					"\t\tAmodel Loaded:\t%s\n",
			amodel->amodel_loaded ? "Loaded" : "Not Loaded");

	if (!amodel->amodel_loaded) {
		*updated_off = off;
		return 0;
	}

	off += snprintf(buf + off, PAGE_SIZE - off,
			"\t\tAcoustic model size:\t%d bytes\n",
			(int)(amodel->amodel_size));

	off += snprintf(buf + off, PAGE_SIZE - off,
			"\t\tNumber of chunks:\t%d\n",
			(int)(amodel->num_of_amodel_chunks));

	off += snprintf(buf + off, PAGE_SIZE - off,
			"\t\tAmodel Checksum:\t0x%8x\n",
			amodel->amodel_checksum);

	off += snprintf(buf + off, PAGE_SIZE - off,
			"\t\tAmodel Det Type:\t%s\n",
			dbmdx_vte_det_type_names[amodel->det_type]);

	off += snprintf(buf + off, PAGE_SIZE - off,
			"\t\tAmodel amode:\t%d\n", (int)(amodel->amode));

	off += snprintf(buf + off, PAGE_SIZE - off,
			"\t\tAmodel Engine Type:\t%s\n",
			dbmdx_vte_names[amodel->eng_type]);

	*updated_off = off;
	return 0;
}

static int dbmdx_dump_vte(struct dbmdx_private *p, struct vt_engine *vte,
				 int off, char *buf, int *updated_off)
{

	int ret;
	int model_updated_off = 0;

	if (!p)
		return -EAGAIN;

	*updated_off = off;

	if (!vte)
		return 0;

	off += snprintf(buf + off, PAGE_SIZE - off,
				"\tVTE index: %u\n", vte->vte_index);

	off += snprintf(buf + off, PAGE_SIZE - off,
					"\tVTE Enabled:\t%s\n",
			vte->enabled ? "ON" : "OFF");

	if (!vte->enabled) {
		*updated_off = off;
		return 0;
	}

	off += snprintf(buf + off, PAGE_SIZE - off, "\tVTE Type:\t%s\n",
						dbmdx_vte_names[vte->eng_type]);

	off += snprintf(buf + off, PAGE_SIZE - off,
			"\tAmodel options:\t0x%8x\n", vte->amodel_options);

	off += snprintf(buf + off, PAGE_SIZE - off,
			"\tVTE Passphrase Detection support:\t%s\n",
			(vte->mode_support_mask &
					DBMDX_VT_MODE_SUPPORT_PHRASE) ?
			"ON" : "OFF");

	off += snprintf(buf + off, PAGE_SIZE - off,
			"\tVTE Voice Command support:\t%s\n",
			(vte->mode_support_mask &
					DBMDX_VT_MODE_SUPPORT_VOICE_COMMAND) ?
			"ON" : "OFF");

	off += snprintf(buf + off, PAGE_SIZE - off,
			"\tVTE LPSD support:\t%s\n",
			(vte->mode_support_mask &
					DBMDX_VT_MODE_SUPPORT_VOICE_ENERGY) ?
			"ON" : "OFF");

	off += snprintf(buf + off, PAGE_SIZE - off,
			"\tVTE Dual model support:\t%s\n",
			(vte->mode_support_mask &
					DBMDX_VT_MODE_SUPPORT_VOICE_DUAL) ?
			"ON" : "OFF");

	if (vte->mode_support_mask & DBMDX_VT_MODE_SUPPORT_PHRASE) {
		off += snprintf(buf + off, PAGE_SIZE - off,
				"\tPassphrase Amodel Settings\n");

		off += snprintf(buf + off, PAGE_SIZE - off,
			"\t\tNumber of model files:\t%d\n",
			vte->num_of_amodel_files[DETECTION_TYPE_PHRASE]);
		if (vte->num_of_amodel_files[DETECTION_TYPE_PHRASE] > 0)
			off += snprintf(buf + off, PAGE_SIZE - off,
			"\t\tFile #1 Name:\t%s\n",
			vte->amodel_fnames[DETECTION_TYPE_PHRASE][0]);
		if (vte->num_of_amodel_files[DETECTION_TYPE_PHRASE] > 1)
			off += snprintf(buf + off, PAGE_SIZE - off,
			"\t\tFile #2 Name:\t%s\n",
			vte->amodel_fnames[DETECTION_TYPE_PHRASE][1]);
		if (vte->num_of_amodel_files[DETECTION_TYPE_PHRASE] > 2)
			off += snprintf(buf + off, PAGE_SIZE - off,
			"\t\tFile #3 Name:\t%s\n",
			vte->amodel_fnames[DETECTION_TYPE_PHRASE][2]);

		ret = dbmdx_dump_amodel_info(p, &(vte->primary_model),
					"Primary Model",
					off, buf, &model_updated_off);
		if (!ret)
			off = model_updated_off;
	}
	if (vte->mode_support_mask & DBMDX_VT_MODE_SUPPORT_VOICE_ENERGY) {
		off += snprintf(buf + off, PAGE_SIZE - off,
				"\tLPSD Amodel Settings\n");

		off += snprintf(buf + off, PAGE_SIZE - off,
			"\t\tNumber of model files:\t%d\n",
			vte->num_of_amodel_files[DETECTION_TYPE_VOICE_ENERGY]);
		if (vte->num_of_amodel_files[DETECTION_TYPE_VOICE_ENERGY] > 0)
			off += snprintf(buf + off, PAGE_SIZE - off,
			"\t\tFile #1 Name:\t%s\n",
			vte->amodel_fnames[DETECTION_TYPE_VOICE_ENERGY][0]);
		if (vte->num_of_amodel_files[DETECTION_TYPE_VOICE_ENERGY] > 1)
			off += snprintf(buf + off, PAGE_SIZE - off,
			"\t\tFile #2 Name:\t%s\n",
			vte->amodel_fnames[DETECTION_TYPE_VOICE_ENERGY][1]);
		if (vte->num_of_amodel_files[DETECTION_TYPE_VOICE_ENERGY] > 2)
			off += snprintf(buf + off, PAGE_SIZE - off,
			"\t\tFile #3 Name:\t%s\n",
			vte->amodel_fnames[DETECTION_TYPE_VOICE_ENERGY][2]);

		ret = dbmdx_dump_amodel_info(p, &(vte->ve_model),
						"LPSD Model",
						off, buf, &model_updated_off);
		if (!ret)
			off = model_updated_off;

	}
	if (vte->mode_support_mask & DBMDX_VT_MODE_SUPPORT_VOICE_COMMAND) {
		off += snprintf(buf + off, PAGE_SIZE - off,
				"\tVoice Command Amodel Settings\n");

		off += snprintf(buf + off, PAGE_SIZE - off,
			"\t\tNumber of model files:\t%d\n",
			vte->num_of_amodel_files[DETECTION_TYPE_VOICE_COMMAND]);
		if (vte->num_of_amodel_files[DETECTION_TYPE_VOICE_COMMAND] > 0)
			off += snprintf(buf + off, PAGE_SIZE - off,
			"\t\tFile #1 Name:\t%s\n",
			vte->amodel_fnames[DETECTION_TYPE_VOICE_COMMAND][0]);
		if (vte->num_of_amodel_files[DETECTION_TYPE_VOICE_COMMAND] > 1)
			off += snprintf(buf + off, PAGE_SIZE - off,
			"\t\tFile #2 Name:\t%s\n",
			vte->amodel_fnames[DETECTION_TYPE_VOICE_COMMAND][1]);
		if (vte->num_of_amodel_files[DETECTION_TYPE_VOICE_COMMAND] > 2)
			off += snprintf(buf + off, PAGE_SIZE - off,
			"\t\tFile #3 Name:\t%s\n",
			vte->amodel_fnames[DETECTION_TYPE_VOICE_COMMAND][2]);

		ret = dbmdx_dump_amodel_info(p, &(vte->vc_model),
					"VC Model",
					off, buf, &model_updated_off);
		if (!ret)
			off = model_updated_off;

	}
	if (vte->mode_support_mask & DBMDX_VT_MODE_SUPPORT_VOICE_DUAL) {
		off += snprintf(buf + off, PAGE_SIZE - off,
				"\tDual Amodel Settings\n");

		off += snprintf(buf + off, PAGE_SIZE - off,
			"\t\tNumber of model files for Primary Model:\t%d\n",
			vte->num_of_amodel_files[DETECTION_TYPE_DUAL]);
		if (vte->num_of_amodel_files[DETECTION_TYPE_DUAL] > 0)
			off += snprintf(buf + off, PAGE_SIZE - off,
			"\t\tFile #1 Name:\t%s\n",
			vte->amodel_fnames[DETECTION_TYPE_DUAL][0]);
		if (vte->num_of_amodel_files[DETECTION_TYPE_DUAL] > 1)
			off += snprintf(buf + off, PAGE_SIZE - off,
			"\t\tFile #2 Name:\t%s\n",
			vte->amodel_fnames[DETECTION_TYPE_DUAL][1]);
		if (vte->num_of_amodel_files[DETECTION_TYPE_DUAL] > 2)
			off += snprintf(buf + off, PAGE_SIZE - off,
			"\t\tFile #3 Name:\t%s\n",
			vte->amodel_fnames[DETECTION_TYPE_DUAL][2]);

		ret = dbmdx_dump_amodel_info(p, &(vte->primary_model),
					"Primary Model",
					off, buf, &model_updated_off);
			if (!ret)
				off = model_updated_off;

		off += snprintf(buf + off, PAGE_SIZE - off,
			"\t\tNumber of model files for Secondary Model:\t%d\n",
			vte->num_of_amodel_files_sec);
		if (vte->num_of_amodel_files_sec > 0)
			off += snprintf(buf + off, PAGE_SIZE - off,
			"\t\tFile #1 Name:\t%s\n",
			vte->amodel_fnames_sec[0]);
		if (vte->num_of_amodel_files_sec > 1)
			off += snprintf(buf + off, PAGE_SIZE - off,
			"\t\tFile #2 Name:\t%s\n",
			vte->amodel_fnames_sec[1]);
		if (vte->num_of_amodel_files_sec > 2)
			off += snprintf(buf + off, PAGE_SIZE - off,
			"\t\tFile #3 Name:\t%s\n",
			vte->amodel_fnames_sec[2]);

		ret = dbmdx_dump_amodel_info(p, &(vte->secondary_model),
					"Secondary Model",
					off, buf, &model_updated_off);
		if (!ret)
			off = model_updated_off;

	}

	off += snprintf(buf + off, PAGE_SIZE - off,
			"\tVTE Frame Size:\t%u bytes\n",
			vte->frame_size);

	off += snprintf(buf + off, PAGE_SIZE - off,
			"\tVTE CP:\t%u\n",
			vte->cp);

	off += snprintf(buf + off, PAGE_SIZE - off,
			"\tVTE Custom Params:\t0x%8x\n", vte->custom_params);

	off += snprintf(buf + off, PAGE_SIZE - off,
			"\tVTE Mem Loc:\t0x%8x\n", vte->mem_loc);

	off += snprintf(buf + off, PAGE_SIZE - off, "\tVTE User State:\t%s\n",
				dbmdx_model_state_names[vte->user_state]);

	off += snprintf(buf + off, PAGE_SIZE - off,
		"\tVTE User Detection Type:\t%s\n",
		dbmdx_vte_det_type_names[vte->user_detection_type]);

	off += snprintf(buf + off, PAGE_SIZE - off,
			"\tVTE Internal State:\t%s\n",
			dbmdx_model_state_names[vte->internal_state]);

	off += snprintf(buf + off, PAGE_SIZE - off,
		"\tVTE Internal Detection Type:\t%s\n",
		dbmdx_vte_det_type_names[vte->detection_type]);

	off += snprintf(buf + off, PAGE_SIZE - off,
					"\tAmodel Loaded to FW:\t%s\n",
			vte->loaded_to_fw ? "Loaded" : "Not Loaded");

	off += snprintf(buf + off, PAGE_SIZE - off, "\tBacklog length: %u\n",
							*(vte->history));


	*updated_off = off;

	return 0;
}

static ssize_t dbmdx_dump_vte_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct dbmdx_private *p = dev_get_drvdata(dev);
	int off = 0;
	struct dbmdx_platform_data *pdata;
	int updated_off = 0;
	int i;
	struct vt_engine *vte;
	int ret;

	if (!p)
		return -EAGAIN;

	pdata = p->pdata;

	for (i = 0; i < DBMDX_MAX_SUPPORTED_VT_ENGINES; i++) {
		vte = &(p->vt_engines[i]);
		ret = dbmdx_dump_vte(p, vte, off, buf, &updated_off);
		if (!ret)
			off = updated_off;
	}

	return off;
}

#define MAX_REGS_NUM 112 /*0x6F + 1*/
static ssize_t dbmdx_dump_reg_show(struct device *dev, u16 *reg_buf)
{
	struct dbmdx_private *p = dev_get_drvdata(dev);
	int ret, i;
	u16 val = 0;

	if (!p)
		return -EAGAIN;

	if (!p->device_ready) {
		dev_err(p->dev, "%s: device not ready\n", __func__);
		return -EAGAIN;
	}

	p->lock(p);
	ret = dbmdx_switch_to_user_selected_chip_interface(p);
	if (ret) {
		p->unlock(p);
		dev_err(p->dev, "%s: Error switching to usr sel. chip iface\n",
			__func__);
		return -EIO;
	}

	dbmdx_set_power_mode(p, DBMDX_PM_ACTIVE);

	for (i = 0; i < MAX_REGS_NUM; i++) {
		ret = dbmdx_send_cmd(p, (DBMDX_VA_CMD_MASK | i<<16), &val);
		if (ret < 0) {
			dev_err(p->dev, "%s: get reg %x error\n",
					__func__, DBMDX_VA_CMD_MASK | i);
			goto out_unlock;
		}

		reg_buf[i] = val;
	}

out_unlock:
	dbmdx_set_power_mode(p, DBMDX_PM_FALLING_ASLEEP);

	ret = dbmdx_switch_to_drv_selected_chip_interface(p);
	if (ret) {
		p->unlock(p);
		dev_err(p->dev, "%s: Error switching to drv sel. chip iface\n",
			__func__);
		return -EIO;
	}

	p->unlock(p);
	return ret;
}

static ssize_t dbmdx_va_regs_dump_state(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct dbmdx_private *p = dev_get_drvdata(dev);
	int off = 0;
	int i;
	struct dbmdx_platform_data *pdata;
	u16 reg_buf[MAX_REGS_NUM];

	if (!p)
		return -EAGAIN;

	pdata = p->pdata;

	dbmdx_dump_reg_show(dev, &reg_buf[0]);

	off += snprintf(buf + off, PAGE_SIZE - off, "\n\n");
	off += snprintf(buf + off, PAGE_SIZE - off, "dbmdx:\n");
	off += snprintf(buf + off, PAGE_SIZE - off, "Registers Dump:\n");
	off += snprintf(buf + off, PAGE_SIZE - off,
					"register HEX(dec) : value HEX\n");
	off += snprintf(buf + off, PAGE_SIZE - off, "\n");

	for (i = 0; i < MAX_REGS_NUM; i++) {

		if (i == 0x40)
			off += snprintf(buf + off, PAGE_SIZE - off,
				"\nSV parameters direct access registers:\n");

		off += snprintf(buf + off, PAGE_SIZE - off,
				"0x%02X(%02i) : 0x%04X    ", i, i, reg_buf[i]);

		if (!((i+1)%4))
			off += snprintf(buf + off, PAGE_SIZE - off, "\n");
	}

	off += snprintf(buf + off, PAGE_SIZE - off, "\n\n");

	return off;
}

static ssize_t dbmdx_va_rxsize_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct dbmdx_private *p = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%lu\n", p->rxsize);
}

static ssize_t dbmdx_va_rxsize_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t size)
{
	int ret;
	unsigned long val;
	struct dbmdx_private *p = dev_get_drvdata(dev);

	ret = kstrtoul(buf, 0, &val);
	if (ret)
		return -EINVAL;

	if (val % 16 != 0)
		return -EINVAL;

	p->rxsize = val;

	return size;
}

static ssize_t dbmdx_va_rsize_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct dbmdx_private *p = dev_get_drvdata(dev);

	if (!p)
		return -EAGAIN;

	return snprintf(buf, PAGE_SIZE, "%u\n",
				p->chip->get_read_chunk_size(p));

}

static ssize_t dbmdx_va_rsize_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t size)
{
	int ret;
	unsigned long val;
	struct dbmdx_private *p = dev_get_drvdata(dev);

	if (!p)
		return -EAGAIN;

	ret = kstrtoul(buf, 0, &val);
	if (ret)
		return -EINVAL;

	p->lock(p);
	ret = p->chip->set_read_chunk_size(p, (u32)val);
	p->unlock(p);

	if (ret < 0)
		return -EINVAL;

	return size;
}

static ssize_t dbmdx_va_wsize_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct dbmdx_private *p = dev_get_drvdata(dev);

	if (!p)
		return -EAGAIN;


	return snprintf(buf, PAGE_SIZE, "%u\n",
			p->chip->get_write_chunk_size(p));
}

static ssize_t dbmdx_va_wsize_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t size)
{
	int ret;
	unsigned long val;
	struct dbmdx_private *p = dev_get_drvdata(dev);

	if (!p)
		return -EAGAIN;

	ret = kstrtoul(buf, 0, &val);
	if (ret)
		return -EINVAL;

	p->lock(p);
	ret = p->chip->set_write_chunk_size(p, (u32)val);
	p->unlock(p);

	if (ret < 0)
		return -EINVAL;

	return size;
}

static ssize_t dbmdx_power_mode_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct dbmdx_private *p = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%s: %s (%d)\n",
		       p->active_fw == DBMDX_FW_VA ? "VA" : "-",
		       dbmdx_power_mode_names[p->power_mode],
		       p->power_mode);
}

static ssize_t dbmdx_cur_opmode_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct dbmdx_private *p = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%s (%x)\n",
				dbmdx_state_names[p->va_flags.mode],
				p->va_flags.mode);
}

static ssize_t dbmdx_wakeup_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct dbmdx_private *p = dev_get_drvdata(dev);
	int ret;
	int gpio_val;

	if (!p)
		return -EAGAIN;

	if (!p->device_ready) {
		dev_err(p->dev, "%s: device not ready\n", __func__);
		return -EAGAIN;
	}

	if (!dbmdx_can_wakeup(p))
		ret = snprintf(buf, PAGE_SIZE, "No WakeUp GPIO\n");
	else {
		gpio_val = gpio_get_value(p->pdata->gpio_wakeup);
		ret = snprintf(buf, PAGE_SIZE, "WakeUp GPIO: %d\n", gpio_val);
	}

	return ret;
}

static ssize_t dbmdx_wakeup_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t size)
{
	struct dbmdx_private *p = dev_get_drvdata(dev);
	int ret = 0;
	int ret2 = 0;
	enum dbmdx_chip	cur_active_chip = p->active_chip;

	if (p->va_chip_enabled) {

		ret = dbmdx_switch_to_va_chip_interface(p, DBMDX_CMD_INTERFACE);
		if (ret) {
			dev_err(p->dev,
				"%s Error switching to (VA) CMD interface\n",
				__func__);
			ret = -EIO;
			goto out;
		}
		ret = dbmdx_force_wake(p);
		if (ret) {
			dev_err(p->dev,	"%s Cannot wake (VA) chip\n", __func__);
			ret = -EIO;
			goto out;
		}

	}

	if (p->va_ve_chip_enabled) {

		ret = dbmdx_switch_to_va_ve_chip_interface(p,
							DBMDX_CMD_INTERFACE);
		if (ret) {
			dev_err(p->dev,
				"%s Error switching to (VA_VE) CMD interface\n",
				__func__);
			ret = -EIO;
			goto out;
		}

		ret = dbmdx_force_wake(p);
		if (ret) {
			dev_err(p->dev,	"%s Cannot wake (VA_VE) chip\n",
				__func__);
			ret = -EIO;
			goto out;
		}
	}
out:
	if (cur_active_chip == DBMDX_CHIP_VA) {
		ret2 = dbmdx_switch_to_va_chip_interface(p,
						DBMDX_CMD_INTERFACE);
		if (ret2) {
			dev_err(p->dev,
				"%s Error switching to (VA) CMD interface\n",
				__func__);
			ret = -EIO;
		}
	} else {
		ret2 = dbmdx_switch_to_va_ve_chip_interface(p,
						DBMDX_CMD_INTERFACE);
		if (ret2) {
			dev_err(p->dev,
				"%s Error switching to (VA_VE) CMD interface\n",
				__func__);
			ret = -EIO;
		}
	}
	if (ret < 0)
		return ret;
	else
		return size;
	return size;
}

static ssize_t dbmdx_raw_cmd_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct dbmdx_private *p = dev_get_drvdata(dev);
	ssize_t read, i, ret = 0;
#define __MAX_RAW_READ       16
	u8 values[__MAX_RAW_READ];

	read = p->chip->read(p, values, __MAX_RAW_READ);

	if (read > 0) {
		for (i = 0; i < read; i++)
			ret += snprintf(&buf[ret], PAGE_SIZE,
							" %02x", values[i]);
	}

	ret += snprintf(&buf[ret], PAGE_SIZE, "\n");
	return ret;
#undef __MAX_RAW_READ
}

static ssize_t dbmdx_raw_cmd_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t size)
{
	struct dbmdx_private *p = dev_get_drvdata(dev);
	const char *s = " ";
	char *mbuf;
	char *tok;
	ssize_t i = 0, o;
	int ret = 0;
#define __RAW_CMD_INPUT      500
	u8 values[__RAW_CMD_INPUT];


	if (size > (__RAW_CMD_INPUT * 3)) {
		dev_err(p->dev, "%s: too much input (limit is %u bytes)\n",
				 __func__, __RAW_CMD_INPUT * 3);
		return -EINVAL;
	}

	mbuf = kstrndup(buf, __RAW_CMD_INPUT * 3, GFP_KERNEL);
	if (!mbuf)
		return -ENOMEM;

	do {
		tok = strsep(&mbuf, s);
		if (tok) {
			ret = kstrtou8(tok, 16, &values[i]);
			if (ret)
				break;
			i++;
		}
	} while (tok);

	if (ret == 0) {
		o = dbmdx_send_data(p, values, i);
		if (o != i) {
			dev_err(p->dev, "%s: send %zd/%zd bytes\n",
					 __func__, o, i);
		}
	}
	return size;
#undef __RAW_CMD_INPUT
}


static DEVICE_ATTR(fwver, S_IRUGO,
		   dbmdx_fw_ver_show, NULL);
static DEVICE_ATTR(paramaddr, S_IWUSR,
		   NULL, dbmdx_param_addr_store);
static DEVICE_ATTR(param, S_IRUGO | S_IWUSR,
		   dbmdx_param_show, dbmdx_param_store);
static DEVICE_ATTR(dump,  S_IRUGO,
		   dbmdx_dump_state, NULL);
static DEVICE_ATTR(dump_cur_state,  S_IRUGO,
		   dbmdx_dump_current_state, NULL);
static DEVICE_ATTR(dump_vte,  S_IRUGO,
		   dbmdx_dump_vte_show, NULL);
static DEVICE_ATTR(io_addr, S_IRUGO | S_IWUSR,
		   dbmdx_io_addr_show, dbmdx_io_addr_store);
static DEVICE_ATTR(io_value, S_IRUGO | S_IWUSR,
		   dbmdx_io_value_show, dbmdx_io_value_store);
static DEVICE_ATTR(direct_write, S_IWUSR,
		   NULL, dbmdx_va_direct_write_store);
static DEVICE_ATTR(direct_read, S_IWUSR,
		   NULL, dbmdx_va_direct_read_store);
static DEVICE_ATTR(power_mode,  S_IRUGO,
		   dbmdx_power_mode_show, NULL);
static DEVICE_ATTR(reboot, S_IWUSR,
		   NULL, dbmdx_reboot_store);
static DEVICE_ATTR(reset, S_IWUSR,
		   NULL, dbmdx_reset_store);
static DEVICE_ATTR(va_dump_regs,  S_IRUGO,
		   dbmdx_va_regs_dump_state, NULL);
static DEVICE_ATTR(va_debug, S_IRUGO | S_IWUSR,
		   dbmdx_va_debug_show, dbmdx_va_debug_store);
static DEVICE_ATTR(va_speed_cfg, S_IRUGO | S_IWUSR,
		   dbmdx_va_speed_cfg_show, dbmdx_va_speed_cfg_store);
static DEVICE_ATTR(va_cfg_values, S_IRUGO | S_IWUSR,
		   dbmdx_va_cfg_values_show, dbmdx_va_cfg_values_store);
static DEVICE_ATTR(va_mic_cfg, S_IRUGO | S_IWUSR,
		   dbmdx_va_mic_cfg_show, dbmdx_va_mic_cfg_store);
static DEVICE_ATTR(va_backlog_size, S_IRUGO | S_IWUSR,
		   dbmdx_va_backlog_size_show, dbmdx_va_backlog_size_store);
static DEVICE_ATTR(va_buffsize, S_IRUGO | S_IWUSR,
		   dbmdx_va_buffer_size_show, dbmdx_va_buffer_size_store);
static DEVICE_ATTR(va_buffsmps, S_IRUGO,
		   dbmdx_va_buffsmps_show, NULL);
static DEVICE_ATTR(va_capture_on_detect, S_IRUGO | S_IWUSR,
		   dbmdx_va_capture_on_detect_show,
		   dbmdx_va_capture_on_detect_store);
static DEVICE_ATTR(va_detection_after_buffering, S_IRUGO | S_IWUSR,
		   dbmdx_va_detection_after_buffering_show,
		   dbmdx_va_detection_after_buffering_store);
static DEVICE_ATTR(va_disable_recovery, S_IRUGO | S_IWUSR,
		   dbmdx_va_disable_recovery_show,
		   dbmdx_va_disable_recovery_store);
static DEVICE_ATTR(va_digital_gain, S_IRUGO | S_IWUSR,
		   dbmdx_va_digital_gain_show,
		   dbmdx_va_digital_gain_store);
static DEVICE_ATTR(va_max_sample, S_IRUGO,
		   dbmdx_va_max_sample_show, NULL);
static DEVICE_ATTR(va_analog_micgain, S_IRUGO | S_IWUSR,
		   dbmdx_va_analog_micgain_show, dbmdx_va_analog_micgain_store);
static DEVICE_ATTR(va_cur_opmode,  S_IRUGO,
		   dbmdx_cur_opmode_show, NULL);
static DEVICE_ATTR(va_clockcfg,  S_IRUGO | S_IWUSR ,
		   dbmdx_va_clockcfg_show, dbmdx_va_clockcfg_store);
static DEVICE_ATTR(va_rsize, S_IRUGO | S_IWUSR,
		   dbmdx_va_rsize_show, dbmdx_va_rsize_store);
static DEVICE_ATTR(va_rxsize, S_IRUGO | S_IWUSR,
		   dbmdx_va_rxsize_show, dbmdx_va_rxsize_store);
static DEVICE_ATTR(va_trigger_level, S_IRUGO | S_IWUSR,
		   dbmdx_va_trigger_level_show, dbmdx_va_trigger_level_store);
static DEVICE_ATTR(va_verif_level, S_IRUGO | S_IWUSR,
		   dbmdx_va_verification_level_show,
		   dbmdx_va_verification_level_store);
static DEVICE_ATTR(va_wsize, S_IRUGO | S_IWUSR,
		   dbmdx_va_wsize_show, dbmdx_va_wsize_store);
static DEVICE_ATTR(va_min_samples_chunk_size, S_IRUGO | S_IWUSR,
		   dbmdx_va_min_samples_chunk_size_show,
		   dbmdx_va_min_samples_chunk_size_store);
static DEVICE_ATTR(va_max_detection_buffer_size, S_IRUGO | S_IWUSR,
		   dbmdx_va_max_detection_buffer_size_show,
		   dbmdx_va_max_detection_buffer_size_store);
static DEVICE_ATTR(wakeup, S_IRUGO | S_IWUSR,
		   dbmdx_wakeup_show, dbmdx_wakeup_store);
static DEVICE_ATTR(raw_cmd, S_IRUGO | S_IWUSR,
		   dbmdx_raw_cmd_show, dbmdx_raw_cmd_store);
static DEVICE_ATTR(va_boot_options, S_IRUGO | S_IWUSR,
		   dbmdx_va_boot_options_show,
		   dbmdx_va_boot_options_store);
static DEVICE_ATTR(va_amodel_options, S_IRUGO | S_IWUSR,
		   dbmdx_va_amodel_options_show,
		   dbmdx_va_amodel_options_store);
static DEVICE_ATTR(hw_revision, S_IRUGO | S_IWUSR,
		   dbmdx_hw_revision_show,
		   dbmdx_hw_revision_store);
static DEVICE_ATTR(debug_delay, S_IRUGO | S_IWUSR,
		   dbmdx_debug_delay_show,
		   dbmdx_debug_delay_store);
static DEVICE_ATTR(fwerr, S_IRUGO,
		   dbmdx_fw_err_show, NULL);

static DEVICE_ATTR(usecase_manager, S_IWUSR, NULL, dbmdx_usecase_manager_store);
static DEVICE_ATTR(va_usecase, S_IRUGO | S_IWUSR,
		   dbmdx_va_usecase_show, dbmdx_va_usecase_store);
static DEVICE_ATTR(va_usecase_name, S_IRUGO | S_IWUSR,
		   dbmdx_va_usecase_name_show, dbmdx_va_usecase_name_store);
static DEVICE_ATTR(va_usecase_mode, S_IRUGO | S_IWUSR,
		   dbmdx_va_usecase_mode_show,
		   dbmdx_va_usecase_mode_store);
static DEVICE_ATTR(va_usecase_info, S_IRUGO | S_IWUSR,
		   dbmdx_va_usecase_info_show,
		   dbmdx_va_usecase_info_store);
static DEVICE_ATTR(va_load_external_usecase, S_IWUSR,
		   NULL, dbmdx_va_load_external_usecase_store);
static DEVICE_ATTR(user_selected_chip, S_IRUGO | S_IWUSR,
		   dbmdx_user_selected_chip_show,
		   dbmdx_user_selected_chip_store);
static DEVICE_ATTR(va_ve_debug, S_IRUGO | S_IWUSR,
		   dbmdx_va_ve_debug_show, dbmdx_va_ve_debug_store);
static DEVICE_ATTR(va_ve_mic_cfg, S_IRUGO | S_IWUSR,
		   dbmdx_va_ve_mic_cfg_show, dbmdx_va_ve_mic_cfg_store);
static DEVICE_ATTR(va_ve_boot_options, S_IRUGO | S_IWUSR,
		   dbmdx_va_ve_boot_options_show,
		   dbmdx_va_ve_boot_options_store);
static DEVICE_ATTR(va_ve_cfg_values, S_IRUGO | S_IWUSR,
		   dbmdx_va_ve_cfg_values_show, dbmdx_va_ve_cfg_values_store);
static DEVICE_ATTR(ref_playback_active, S_IRUGO | S_IWUSR,
		   dbmdx_ref_playback_active_show,
		   dbmdx_ref_playback_active_store);
static DEVICE_ATTR(va_ve_mic_mask, S_IRUGO | S_IWUSR,
		   dbmdx_va_ve_mic_mask_show,
		   dbmdx_va_ve_mic_mask_store);
static DEVICE_ATTR(asrp_delay, S_IRUGO | S_IWUSR,
		   dbmdx_asrp_delay_show,
		   dbmdx_asrp_delay_store);
static DEVICE_ATTR(asrp_gain, S_IRUGO | S_IWUSR,
		   dbmdx_asrp_gain_show,
		   dbmdx_asrp_gain_store);
static DEVICE_ATTR(alsa_streaming_options, S_IRUGO | S_IWUSR,
		   dbmdx_va_alsa_streaming_options_show,
		   dbmdx_va_alsa_streaming_options_store);

static struct attribute *dbmdx_va_attributes[] = {
	&dev_attr_io_addr.attr,
	&dev_attr_io_value.attr,
	&dev_attr_direct_write.attr,
	&dev_attr_direct_read.attr,
	&dev_attr_va_debug.attr,
	&dev_attr_va_dump_regs.attr,
	&dev_attr_va_cfg_values.attr,
	&dev_attr_va_mic_cfg.attr,
	&dev_attr_va_backlog_size.attr,
	&dev_attr_va_buffsize.attr,
	&dev_attr_va_buffsmps.attr,
	&dev_attr_va_capture_on_detect.attr,
	&dev_attr_va_detection_after_buffering.attr,
	&dev_attr_va_digital_gain.attr,
	&dev_attr_va_max_sample.attr,
	&dev_attr_va_analog_micgain.attr,
	&dev_attr_va_cur_opmode.attr,
	&dev_attr_va_clockcfg.attr,
	&dev_attr_va_rsize.attr,
	&dev_attr_va_rxsize.attr,
	&dev_attr_va_trigger_level.attr,
	&dev_attr_va_verif_level.attr,
	&dev_attr_va_wsize.attr,
	&dev_attr_va_min_samples_chunk_size.attr,
	&dev_attr_va_max_detection_buffer_size.attr,
	&dev_attr_va_amodel_options.attr,
	NULL,
};

static struct attribute *dbmdx_va_ve_common_attributes[] = {
	&dev_attr_io_addr.attr,
	&dev_attr_io_value.attr,
	&dev_attr_direct_write.attr,
	&dev_attr_direct_read.attr,
	&dev_attr_va_debug.attr,
	&dev_attr_va_ve_debug.attr,
	&dev_attr_va_dump_regs.attr,
	&dev_attr_va_ve_mic_cfg.attr,
	&dev_attr_va_digital_gain.attr,
	&dev_attr_va_max_sample.attr,
	&dev_attr_va_analog_micgain.attr,
	&dev_attr_va_cur_opmode.attr,
	&dev_attr_va_rsize.attr,
	&dev_attr_va_rxsize.attr,
	&dev_attr_va_wsize.attr,
	&dev_attr_va_min_samples_chunk_size.attr,
	&dev_attr_usecase_manager.attr,
	&dev_attr_va_usecase.attr,
	&dev_attr_va_usecase_name.attr,
	&dev_attr_va_usecase_mode.attr,
	&dev_attr_va_usecase_info.attr,
	&dev_attr_va_load_external_usecase.attr,
	&dev_attr_user_selected_chip.attr,
	&dev_attr_ref_playback_active.attr,
	&dev_attr_va_ve_mic_mask.attr,
	&dev_attr_asrp_delay.attr,
	&dev_attr_asrp_gain.attr,
	&dev_attr_alsa_streaming_options.attr,
	NULL,
};
static struct attribute *dbmdx_va_ve_chip_va_attributes[] = {
	&dev_attr_dump_vte.attr,
	&dev_attr_va_cfg_values.attr,
	&dev_attr_va_backlog_size.attr,
	&dev_attr_va_detection_after_buffering.attr,
	&dev_attr_va_capture_on_detect.attr,
	&dev_attr_va_trigger_level.attr,
	&dev_attr_va_verif_level.attr,
	&dev_attr_va_max_detection_buffer_size.attr,
	&dev_attr_va_amodel_options.attr,
	NULL,
};

static struct attribute *dbmdx_va_ve_chip_va_ve_attributes[] = {
	&dev_attr_va_ve_boot_options.attr,
	&dev_attr_va_ve_cfg_values.attr,
	NULL,
};

static struct attribute *dbmdx_common_attributes[] = {
	&dev_attr_fwver.attr,
	&dev_attr_paramaddr.attr,
	&dev_attr_param.attr,
	&dev_attr_dump.attr,
	&dev_attr_dump_cur_state.attr,
	&dev_attr_power_mode.attr,
	&dev_attr_reboot.attr,
	&dev_attr_reset.attr,
	&dev_attr_va_speed_cfg.attr,
	&dev_attr_va_disable_recovery.attr,
	&dev_attr_wakeup.attr,
	&dev_attr_raw_cmd.attr,
	&dev_attr_va_boot_options.attr,
	&dev_attr_hw_revision.attr,
	&dev_attr_debug_delay.attr,
	&dev_attr_fwerr.attr,
	NULL,
};


static const struct attribute_group dbmdx_common_attribute_group = {
	.attrs = dbmdx_common_attributes,
};

static const struct attribute_group dbmdx_va_attribute_group = {
	/* .name = "VA", */
	.attrs = dbmdx_va_attributes,
};

static const struct attribute_group dbmdx_va_ve_chip_va_ve_attribute_group = {
	/* .name = "VA_VE", */
	.attrs = dbmdx_va_ve_chip_va_ve_attributes,
};
static const struct attribute_group dbmdx_va_ve_chip_va_attribute_group = {
	/* .name = "VA_VE", */
	.attrs = dbmdx_va_ve_chip_va_attributes,
};
static const struct attribute_group dbmdx_va_ve_common_attribute_group = {
	/* .name = "VA_VE", */
	.attrs = dbmdx_va_ve_common_attributes,
};


static int dbmdx_shutdown(struct dbmdx_private *p)
{
	int ret = 0;

	if (!p)
		return -EAGAIN;

	if (!p->device_ready) {
		dev_err(p->dev, "%s: device not ready\n", __func__);
		return 0;
	}

	/* flush pending sv work if any */
	p->va_flags.buffering = 0;
	flush_work(&p->sv_work);

	ret = dbmdx_suspend_pcm_streaming_work(p);
	if (ret < 0)
		dev_err(p->dev, "%s: Failed to suspend PCM Streaming Work\n",
			__func__);

	p->lock(p);

	p->device_ready = false;

	p->asleep = false;

	p->active_fw = DBMDX_FW_POWER_OFF_VA;

	p->unlock(p);

	return 0;
}

static int dbmdx_perform_recovery(struct dbmdx_private *p)
{
#ifdef RR_TOFIX
	int ret = 0;
	int active_fw = p->active_fw;
	int current_mode = p->va_flags.mode;
	int current_audio_channels;
	struct va_flags	saved_va_flags;

	dev_info(p->dev, "%s: active FW - %s\n", __func__,
			dbmdx_fw_type_to_str(active_fw));

	if (active_fw == DBMDX_FW_VA) {
		current_mode = p->va_flags.mode;
		current_audio_channels = p->pdata->va_audio_channels;
		p->va_flags.buffering = 0;
		flush_work(&p->sv_work);

		ret = dbmdx_suspend_pcm_streaming_work(p);
		if (ret < 0)
			dev_err(p->dev,
				"%s: Failed to suspend PCM Streaming Work\n",
				__func__);
		memcpy(&saved_va_flags, &(p->va_flags), sizeof(saved_va_flags));
		p->wakeup_release(p);
		if (p->pdata->feature_va_ve)
			ret = dbmdx_request_and_load_fw_va_ve_mode(p);
		else
			ret = dbmdx_request_and_load_fw(p, 1);

	} else if (active_fw == DBMDX_FW_POWER_OFF_VA) {
		current_mode = p->va_flags.mode;
		current_audio_channels = p->pdata->va_audio_channels;

		memcpy(&saved_va_flags, &(p->va_flags), sizeof(saved_va_flags));
		p->wakeup_release(p);
		if (p->pdata->feature_va_ve)
			ret = dbmdx_request_and_load_fw_va_ve_mode(p);
		else
			ret = dbmdx_request_and_load_fw(p, 1);

	}

	if (ret != 0) {
		dev_err(p->dev, "%s: Recovery failure\n", __func__);
		return -EIO;
	}

	p->wakeup_release(p);

	p->lock(p);

	active_fw = p->active_fw;

	if (active_fw == DBMDX_FW_VA) {

		bool sv_a_model_loaded = saved_va_flags.amodel_len > 0 &&
					saved_va_flags.a_model_downloaded_to_fw;
#ifdef DMBDX_OKG_AMODEL_SUPPORT
		bool okg_a_model_downloaded_to_fw =
				saved_va_flags.okg_a_model_enabled &&
				saved_va_flags.okg_amodel_len > 0 &&
				saved_va_flags.okg_a_model_downloaded_to_fw;
		bool model_loaded = (sv_a_model_loaded ||
			okg_a_model_downloaded_to_fw);

		p->va_flags.okg_a_model_enabled =
			saved_va_flags.okg_a_model_enabled;

		if (p->va_flags.okg_a_model_enabled)
			p->va_flags.okg_a_model_downloaded_to_fw =
				saved_va_flags.okg_a_model_downloaded_to_fw;
		p->va_flags.okg_recognition_mode =
					saved_va_flags.okg_recognition_mode;
#else
		bool model_loaded = sv_a_model_loaded;
#endif
		p->va_flags.sv_recognition_mode =
					saved_va_flags.sv_recognition_mode;

		p->va_flags.pcm_streaming_active =
			saved_va_flags.pcm_streaming_active;

		p->va_flags.pcm_streaming_pushing_zeroes =
			saved_va_flags.pcm_streaming_pushing_zeroes;

		if (model_loaded) {
			int amodel_mode = 0;
			unsigned int  model_select_mask = 0;
			unsigned int  model_options_mask = 0;
			unsigned int  model_custom_params =
				p->va_detection_mode_custom_params;
			p->unlock(p);

			p->va_flags.auto_detection_disabled = true;
#ifdef DMBDX_OKG_AMODEL_SUPPORT
			/* If SV model is not loaded OKG model should be
			 * reloaded explicitly, otherwise it will be loaded
			 * after SV model is reloaded
			 */
			if (!sv_a_model_loaded) {
				amodel_mode = DETECTION_MODE_PHRASE;
				model_select_mask = DBMDX_OKG_MODEL_SELECTED;
				model_options_mask =
						DBMDX_LOAD_MODEL_NO_DETECTION;
				if (p->okg_amodel.amodel_loaded)
					model_options_mask |=
						DBMDX_LOAD_MODEL_FROM_MEMORY;

				amodel_mode |=
					((model_select_mask << 4) & 0x30);

				amodel_mode |=
					((model_options_mask << 8) & 0xf00);

				ret = dbmdx_va_amodel_update(p, amodel_mode);
			} else {
				amodel_mode = p->va_detection_mode;
				model_select_mask = DBMDX_SV_MODEL_SELECTED;
				if (okg_a_model_downloaded_to_fw)
					model_select_mask |=
						DBMDX_OKG_MODEL_SELECTED;
				model_options_mask =
						DBMDX_LOAD_MODEL_NO_DETECTION;
				if (p->primary_amodel.amodel_loaded)
					model_options_mask |=
						DBMDX_LOAD_MODEL_FROM_MEMORY;

				amodel_mode |=
					((model_select_mask << 4) & 0x30);

				amodel_mode |=
					((model_options_mask << 8) & 0xf00);

				amodel_mode |=
					((model_custom_params << 12) & 0xf000);

				ret = dbmdx_va_amodel_update(p, amodel_mode);
			}
#else
			amodel_mode = p->va_detection_mode;
			model_select_mask = DBMDX_SV_MODEL_SELECTED;
			model_options_mask = DBMDX_LOAD_MODEL_NO_DETECTION;
			if (p->primary_amodel.amodel_loaded)
				model_options_mask |=
						DBMDX_LOAD_MODEL_FROM_MEMORY;

			amodel_mode |=	((model_select_mask << 4) & 0x30);

			amodel_mode |=	((model_options_mask << 8) & 0xf00);

			amodel_mode |=	((model_custom_params << 12) & 0xf000);

			ret = dbmdx_va_amodel_update(p, amodel_mode);
#endif
			p->va_flags.auto_detection_disabled = false;

			if (ret != 0) {
				dev_err(p->dev,
					"%s: Failed to reload amodel\n",
					__func__);
			}

			p->lock(p);
			if (current_mode == DBMDX_DETECTION ||
				current_mode == DBMDX_DETECTION_AND_STREAMING) {

				amodel_mode = p->va_detection_mode;
				model_select_mask = 0;
				if (saved_va_flags.sv_recognition_mode !=
					SV_RECOGNITION_MODE_DISABLED)
					model_select_mask |=
						DBMDX_SV_MODEL_SELECTED;
#ifdef DMBDX_OKG_AMODEL_SUPPORT
				if (saved_va_flags.okg_recognition_mode !=
					OKG_RECOGNITION_MODE_DISABLED)
					model_select_mask |=
						DBMDX_OKG_MODEL_SELECTED;
#endif
				model_options_mask =
						DBMDX_DO_NOT_RELOAD_MODEL;

				amodel_mode |=
					((model_select_mask << 4) & 0x30);

				amodel_mode |=
					((model_options_mask << 8) & 0xf00);

				amodel_mode |=
					((model_custom_params << 12) & 0xf000);

				p->unlock(p);

				ret = dbmdx_va_amodel_update(p, amodel_mode);

				p->lock(p);

				if (ret) {
					dev_err(p->dev,
					"%s: Failed to trigger detection\n",
						__func__);
				}

			} else if (current_mode == DBMDX_STREAMING) {
				ret = dbmdx_set_mode(p, DBMDX_STREAMING);

				if (ret < 0) {
					dev_err(p->dev,
					"%s: Failed to set DBMDX_STREAMING mode\n",
						__func__);
				}
			} else
				dbmdx_set_power_mode(p,
					DBMDX_PM_FALLING_ASLEEP);
		}

	}

	p->device_ready = true;

	ret = dbmdx_set_power_mode(p, DBMDX_PM_FALLING_ASLEEP);
	if (ret)
		goto out;

out:
	p->unlock(p);
	return ret;
#else
	return 0;
#endif
}

#ifndef ALSA_SOC_INTERFACE_NOT_SUPPORTED
/* ------------------------------------------------------------------------
 * Interface functions for platform driver
 * ------------------------------------------------------------------------
 */
#ifdef SND_SOC_COMPONENT
int dbmdx_get_samples(struct snd_soc_component *component, char *buffer,
	unsigned int samples)
#else
int dbmdx_get_samples(struct snd_soc_codec *codec, char *buffer,
	unsigned int samples)

#endif    
{
#if defined(CONFIG_SND_SOC_DBMDX_SND_CAPTURE)
#ifdef SND_SOC_COMPONENT
    struct dbmdx_private *p = snd_soc_component_get_drvdata(component);
#else    
	struct dbmdx_private *p = snd_soc_codec_get_drvdata(codec);
#endif    
#else
	struct dbmdx_private *p = dbmdx_data;
#endif
	int avail = kfifo_len(&p->pcm_kfifo);
	int samples_avail = avail / p->bytes_per_sample;
	int ret;
	int err = -1;

#ifdef DBMDX_VV_DEBUG
	pr_debug("%s Requested %u, Available %d\n", __func__, samples, avail);
#endif
	if (p->va_flags.pcm_streaming_pushing_zeroes)
		return err;

	if (samples_avail < samples)
		return err;

	ret = kfifo_out(&p->pcm_kfifo,
			buffer,
			samples * p->bytes_per_sample);

	return ret == samples * p->bytes_per_sample ? 0 : err;
}
EXPORT_SYMBOL(dbmdx_get_samples);

#ifdef SND_SOC_COMPONENT
int dbmdx_component_lock(struct snd_soc_component *component)
#else
int dbmdx_codec_lock(struct snd_soc_codec *codec)
#endif
{
#if defined(CONFIG_SND_SOC_DBMDX_SND_CAPTURE)
#ifdef SND_SOC_COMPONENT
	struct dbmdx_private *p = snd_soc_component_get_drvdata(component);
#else
	struct dbmdx_private *p = snd_soc_codec_get_drvdata(codec);
#endif    
#else
	struct dbmdx_private *p = dbmdx_data;
#endif

	if (!p)
		return -EAGAIN;

	if (!atomic_add_unless(&p->audio_owner, 1, 1))
		return -EBUSY;

	return 0;
}
#ifdef SND_SOC_COMPONENT
EXPORT_SYMBOL(dbmdx_component_lock);
#else
EXPORT_SYMBOL(dbmdx_codec_lock);
#endif

#ifdef SND_SOC_COMPONENT
int dbmdx_component_unlock(struct snd_soc_component *component)
#else
int dbmdx_codec_unlock(struct snd_soc_codec *codec)
#endif
{
#if defined(CONFIG_SND_SOC_DBMDX_SND_CAPTURE)
#ifdef SND_SOC_COMPONENT
	struct dbmdx_private *p = snd_soc_component_get_drvdata(component);
#else
	struct dbmdx_private *p = snd_soc_codec_get_drvdata(codec);
#endif
#else
	struct dbmdx_private *p = dbmdx_data;
#endif

	if (!p)
		return -EAGAIN;

	atomic_dec(&p->audio_owner);
	return 0;
}
#ifdef SND_SOC_COMPONENT
EXPORT_SYMBOL(dbmdx_component_unlock); 
#else
EXPORT_SYMBOL(dbmdx_codec_unlock);
#endif

#if defined(CONFIG_SND_SOC_DBMDX_SND_CAPTURE)

#ifdef SND_SOC_COMPONENT
int dbmdx_start_pcm_streaming(struct snd_soc_component *component,
	struct snd_pcm_substream *substream)
#else
int dbmdx_start_pcm_streaming(struct snd_soc_codec *codec,
	struct snd_pcm_substream *substream)

#endif    
{
	int ret = 0;
#ifdef SND_SOC_COMPONENT    
	struct dbmdx_private *p = snd_soc_component_get_drvdata(component);
#else
	struct dbmdx_private *p = snd_soc_codec_get_drvdata(codec);
#endif    
	struct usecase_config *cur_usecase;

	if (!p)
		return -EAGAIN;

	p->lock(p);

	if (!p->device_ready) {
		dev_err(p->dev, "%s: device not ready\n", __func__);
		p->unlock(p);
		return -EAGAIN;
	}

	dev_dbg(p->dev, "%s:\n", __func__);

	p->va_flags.pcm_streaming_active = 0;

	p->unlock(p);

	/* flush pending buffering work if any */
	p->va_flags.buffering = 0;
	flush_work(&p->sv_work);
	p->va_flags.pcm_worker_active = 0;
	flush_work(&p->pcm_streaming_work);

	cur_usecase = p->va_ve_flags.cur_usecase;
	p->active_substream = substream;


	if (cur_usecase && cur_usecase->usecase_supports_us_buffering) {
		p->lock(p);
		dbmdx_set_power_mode(p, DBMDX_PM_ACTIVE);
		if (p->pdata->alsa_streaming_options &
			DBMDX_ALSA_STREAMING_BUFFERING_WITH_HISTORY)
			ret = dbmdx_set_usecase_mode(p,
					(DBMDX_BUFFERING_WITH_BACKLOG_MASK |
					DBMDX_STREAMING));
		else
			ret = dbmdx_set_usecase_mode(p, DBMDX_STREAMING);
		p->unlock(p);
		if (ret) {
			dev_err(p->dev,
				"%s: Failed to set Usecase in Streaming mode\n",
				__func__);
			return -EIO;
		}

	} else if (p->pdata->default_streaming_usecase &&
			(p->pdata->alsa_streaming_options &
			DBMDX_ALSA_STREAMING_SET_DEFAULT_USECASE)) {
		ret = dbmdx_set_va_usecase_name(p,
				p->pdata->default_streaming_usecase);
		if (ret) {
			dev_err(p->dev,
				"%s: Failed to configure streaming usecase\n",
					__func__);
			return -EIO;
		}
		p->lock(p);
		dbmdx_set_power_mode(p, DBMDX_PM_ACTIVE);
		if (p->pdata->alsa_streaming_options &
			DBMDX_ALSA_STREAMING_BUFFERING_WITH_HISTORY)
			ret = dbmdx_set_usecase_mode(p,
					(DBMDX_BUFFERING_WITH_BACKLOG_MASK |
					DBMDX_STREAMING));
		else
			ret = dbmdx_set_usecase_mode(p, DBMDX_STREAMING);

		p->unlock(p);
		if (ret) {
			dev_err(p->dev,
				"%s: Failed to set Buffering usecase mode\n",
				__func__);
			return -EIO;
		}

	} else {
		dev_err(p->dev,
		"%s: Usecase that supports streaming wasn't configured\n",
			__func__);
		return -EAGAIN;

	}
	return 0;
}

EXPORT_SYMBOL(dbmdx_start_pcm_streaming);
#ifdef SND_SOC_COMPONENT
int dbmdx_stop_pcm_streaming(struct snd_soc_component *component)
#else
int dbmdx_stop_pcm_streaming(struct snd_soc_codec *codec)
#endif
{
	int ret = 0;
#ifdef SND_SOC_COMPONENT    
	struct dbmdx_private *p = snd_soc_component_get_drvdata(component);
#else
	struct dbmdx_private *p = snd_soc_codec_get_drvdata(codec);
#endif    
	struct usecase_config *cur_usecase;

	if (!p)
		return -EAGAIN;

	p->lock(p);

	if (!p->device_ready) {
		dev_err(p->dev, "%s: device not ready\n", __func__);
		p->unlock(p);
		return -EAGAIN;
	}

	dev_dbg(p->dev, "%s:\n", __func__);

	p->va_flags.pcm_streaming_active = 0;

	p->unlock(p);

	/* flush pending work if any */
	p->va_flags.pcm_worker_active = 0;
	flush_work(&p->pcm_streaming_work);
	p->active_substream = NULL;

	cur_usecase = p->va_ve_flags.cur_usecase;

	if (cur_usecase && p->va_ve_flags.usecase_mode == DBMDX_STREAMING) {
		if (p->pdata->alsa_streaming_options &
			DBMDX_ALSA_STREAMING_IDLE_AFTER_BUFFERING) {
			dev_dbg(p->dev, "%s: Setting IDLE mode\n", __func__);
			ret = dbmdx_set_va_usecase_name(p, "idle");
			if (ret) {
				dev_err(p->dev,
					"%s: Failed to set IDLE usecase\n",
					__func__);
				return -EIO;
			}
		} else {
			p->lock(p);
			dev_dbg(p->dev, "%s: Setting DET. mode\n", __func__);
			dbmdx_set_power_mode(p, DBMDX_PM_ACTIVE);
			ret = dbmdx_set_usecase_mode(p, DBMDX_DETECTION);
			if (ret)
				dev_err(p->dev,
				"%s: Failed to set Usecase in Detection Mode\n",
					__func__);
			p->unlock(p);
		}
	} else {
		dev_dbg(p->dev,
			"%s:Usecase is not streaming, UC Mode: %d\n",
			__func__, p->va_ve_flags.usecase_mode);
	}
	return ret;
}

EXPORT_SYMBOL(dbmdx_stop_pcm_streaming);

#endif

/* ------------------------------------------------------------------------
 * Codec driver section
 * ------------------------------------------------------------------------
 */

#define DUMMY_REGISTER 0

static int dbmdx_dai_hw_params(struct snd_pcm_substream *substream,
			       struct snd_pcm_hw_params *params,
			       struct snd_soc_dai *dai)
{
#ifdef SND_SOC_COMPONENT
	int ret = 0;
	struct snd_soc_component *component = dai->component;
	struct dbmdx_private *p = snd_soc_component_get_drvdata(component);
#else
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	int ret = 0;
	struct dbmdx_private *p = snd_soc_codec_get_drvdata(rtd->codec);

#endif    
	int channels;

	if (!p)
		return -EAGAIN;

	if (!p->device_ready) {
		dev_err(p->dev, "%s: device not ready\n", __func__);
		ret = -EAGAIN;
		goto out;
	}

	if (p->active_fw != DBMDX_FW_VA && p->active_fw != DBMDX_FW_VA_VE) {
		dev_dbg(p->dev, "%s: VA firmware not active\n", __func__);
		ret = -EAGAIN;
		goto out;
	}

	dev_dbg(p->dev, "%s:\n", __func__);

	channels = params_channels(params);
	p->audio_pcm_channels = channels;

	if (channels == p->pdata->va_audio_channels)
		p->pcm_achannel_op = AUDIO_CHANNEL_OP_COPY;
	else {
		if (channels == 1 && p->pdata->va_audio_channels == 2)
			p->pcm_achannel_op =
				AUDIO_CHANNEL_OP_TRUNCATE_2_TO_1;
		else if (channels == 2 && p->pdata->va_audio_channels == 1)
			p->pcm_achannel_op =
				AUDIO_CHANNEL_OP_DUPLICATE_1_TO_2;
#ifdef DBMDX_4CHANNELS_SUPPORT
		else if (channels == 1 && p->pdata->va_audio_channels == 4)
			p->pcm_achannel_op =
				AUDIO_CHANNEL_OP_TRUNCATE_4_TO_1;
		else if (channels == 2 && p->pdata->va_audio_channels == 4)
			p->pcm_achannel_op =
				AUDIO_CHANNEL_OP_TRUNCATE_4_TO_2;
		else if (channels == 4 && p->pdata->va_audio_channels == 1)
			p->pcm_achannel_op =
				AUDIO_CHANNEL_OP_DUPLICATE_1_TO_4;
		else if (channels == 4 && p->pdata->va_audio_channels == 2)
			p->pcm_achannel_op =
				AUDIO_CHANNEL_OP_DUPLICATE_2_TO_4;
#endif
		else {
			dev_err(p->dev,
			 "%s: DAI channels %d not matching hw channels %d\n",
			 __func__, channels, p->pdata->va_audio_channels);
			ret = -EINVAL;
			goto out;
		}
	}

	dev_info(p->dev, "%s: DAI channels %d, Channel operation set to %d\n",
		__func__, channels, p->pcm_achannel_op);

	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		dev_dbg(p->dev, "%s: set pcm format: SNDRV_PCM_FORMAT_S16_LE\n",
			__func__);
		break;
	default:
		ret = -EINVAL;
	}

	if (ret) {
		dev_err(p->dev, "%s: failed to set pcm format\n",
			__func__);
		goto out;
	}

	switch (params_rate(params)) {
#ifdef DBMDX_PCM_RATE_8000_SUPPORTED
	case 8000:
		/* Fall through */
#endif
	case 16000:
		/* Fall through */
#ifdef DBMDX_PCM_RATE_32000_SUPPORTED
	case 32000:
		/* Fall through */
#endif
#ifdef DBMDX_PCM_RATE_44100_SUPPORTED
	case 44100:
		/* Fall through */
#endif
	case 48000:
		p->audio_pcm_rate = params_rate(params);
		dev_dbg(p->dev, "%s: set pcm rate: %u\n",
			__func__, params_rate(params));
		break;
	default:
		ret = -EINVAL;
	}

	if (ret) {
		dev_err(p->dev, "%s: failed to set pcm rate: %u\n",
			__func__, params_rate(params));
		goto out;
	}

out:
	return ret;
}

static struct snd_soc_dai_ops dbmdx_dai_ops = {
	.hw_params = dbmdx_dai_hw_params,
};

/* DBMDX codec DAI: */
static struct snd_soc_dai_driver dbmdx_va_dais[] = {
	{
		.name = "DBMDX_codec_dai",
		.capture = {
			.stream_name	= "vs_buffer",
			.channels_min	= 1,
			.channels_max	= MAX_SUPPORTED_CHANNELS,
			.rates		=
#ifdef DBMDX_PCM_RATE_8000_SUPPORTED
					SNDRV_PCM_RATE_8000 |
#endif
					SNDRV_PCM_RATE_16000 |
#ifdef DBMDX_PCM_RATE_32000_SUPPORTED
					SNDRV_PCM_RATE_32000 |
#endif
#ifdef DBMDX_PCM_RATE_32000_SUPPORTED
					SNDRV_PCM_RATE_44100 |
#endif
					SNDRV_PCM_RATE_48000,
			.formats	= SNDRV_PCM_FMTBIT_S16_LE,
		},
		.ops = &dbmdx_dai_ops,
	},
};

/* ASoC controls */
#ifdef SND_SOC_COMPONENT
static unsigned int dbmdx_dev_read(struct snd_soc_component *component,
				   unsigned int reg)
#else

static unsigned int dbmdx_dev_read(struct snd_soc_codec *codec,
				   unsigned int reg)

#endif                   
{
#if defined(SOC_CONTROLS_FOR_DBMDX_CODEC_ONLY)
#ifdef SND_SOC_COMPONENT 
	struct dbmdx_private *p = snd_soc_component_get_drvdata(component);
#else
	struct dbmdx_private *p = snd_soc_codec_get_drvdata(codec);
#endif    
#else
	struct dbmdx_private *p = dbmdx_data;
#endif
	int ret;
	u16 val = 0;

	if (!p)
		return -EAGAIN;

	if (!p->device_ready) {
		dev_err(p->dev, "%s: device not ready\n", __func__);
		return -EAGAIN;
	}

	p->lock(p);

	/* VA controls */
	if (p->active_fw != DBMDX_FW_VA)
		goto out_unlock;

	if (reg == DUMMY_REGISTER)
		goto out_unlock;

	/* just return 0 - the user needs to wakeup first */
	if (dbmdx_sleeping(p)) {
		dev_err(p->dev, "%s: device sleeping\n", __func__);
		goto out_unlock;
	}

	if (p->va_flags.mode == DBMDX_DETECTION) {
		dev_dbg(p->dev, "%s: device in detection state\n", __func__);
		goto out_unlock;
	}

	dbmdx_set_power_mode(p, DBMDX_PM_ACTIVE);

	ret = dbmdx_send_cmd(p, reg, &val);
	if (ret < 0)
		dev_err(p->dev, "%s: read 0x%x error\n", __func__, reg);

	dbmdx_set_power_mode(p, DBMDX_PM_FALLING_ASLEEP);

out_unlock:
	p->unlock(p);
	return (unsigned int)val;
}

#ifdef SND_SOC_COMPONENT
static int dbmdx_dev_write(struct snd_soc_component *component, unsigned int reg,
			   unsigned int val)
#else
static int dbmdx_dev_write(struct snd_soc_codec *codec, unsigned int reg,
			   unsigned int val)
#endif
{
#if defined(SOC_CONTROLS_FOR_DBMDX_CODEC_ONLY)
#ifdef SND_SOC_COMPONENT
	struct dbmdx_private *p = snd_soc_component_get_drvdata(component);
#else
	struct dbmdx_private *p = snd_soc_codec_get_drvdata(codec);
#endif
#else
	struct dbmdx_private *p = dbmdx_data;
#endif
	int ret = -EIO;

	if (!p)
		return -EAGAIN;

	if (!p->device_ready) {
		dev_err(p->dev, "%s: device not ready\n", __func__);
		return -EAGAIN;
	}

	dev_dbg(p->dev, "%s: ------- VA control ------\n", __func__);
	if (p->active_fw != DBMDX_FW_VA) {
		dev_dbg(p->dev, "%s: VA firmware not active\n", __func__);
		goto out;
	}

	p->lock(p);

	ret = dbmdx_set_power_mode(p, DBMDX_PM_ACTIVE);
	if (reg == DUMMY_REGISTER)
		goto out_unlock;

	ret = dbmdx_send_cmd(p, (reg << 16) | (val & 0xffff), NULL);
	if (ret < 0)
		dev_err(p->dev, "%s: write 0x%x to 0x%x error\n",
			__func__, val, reg);

out_unlock:
	dbmdx_set_power_mode(p, DBMDX_PM_FALLING_ASLEEP);
	p->unlock(p);
out:
	return ret;
}

static int dbmdx_va_control_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
			(struct soc_mixer_control *)kcontrol->private_value;
	unsigned short val, reg = mc->reg;
	int max = mc->max;
	int mask = (1 << fls(max)) - 1;
	int ret;
	unsigned int va_reg = DBMDX_VA_CMD_MASK | ((reg & 0xff) << 16);
#if defined(SOC_CONTROLS_FOR_DBMDX_CODEC_ONLY)
#ifdef SND_SOC_COMPONENT
	struct snd_soc_component *component = snd_kcontrol_chip(kcontrol);
	struct dbmdx_private *p = snd_soc_component_get_drvdata(codec);
#else
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct dbmdx_private *p = snd_soc_codec_get_drvdata(codec);
#endif    
#else
	struct dbmdx_private *p = dbmdx_data;
#endif
	if (!p)
		return -EAGAIN;

	if (!p->device_ready) {
		dev_err(p->dev, "%s: device not ready\n", __func__);
		return -EAGAIN;
	}

	p->lock(p);

	/* VA controls */
	if (p->active_fw != DBMDX_FW_VA) {
		ucontrol->value.integer.value[0] = 0;
		dev_dbg(p->dev, "%s: VA firmware not active\n", __func__);
		goto out_unlock;
	}

	ret = dbmdx_set_power_mode(p, DBMDX_PM_ACTIVE);
	if (ret < 0) {
		dev_err(p->dev, "%s: failed to set PM_ACTIVE\n", __func__);
		ret = -EINVAL;
		goto out_unlock;
	}

	ret = dbmdx_send_cmd(p, DBMDX_VA_CMD_MASK | ((reg & 0xff) << 16),
				  &val);
	if (ret < 0)
		dev_err(p->dev, "%s: read 0x%x error\n", __func__, reg);

	val &= mask;

	if (va_reg == DBMDX_REGN_DIGITAL_GAIN)
		val = (unsigned short)((short)val + DIGITAL_GAIN_TLV_SHIFT);

	ucontrol->value.integer.value[0] = val;

out_unlock:
	dbmdx_set_power_mode(p, DBMDX_PM_FALLING_ASLEEP);
	p->unlock(p);

	return 0;
}

static int dbmdx_va_update_reg(struct dbmdx_private *p,
				unsigned short reg, unsigned short val)
{
	int ret;
	unsigned int va_reg = DBMDX_VA_CMD_MASK | ((reg & 0xff) << 16);

	p->lock(p);

	ret = dbmdx_set_power_mode(p, DBMDX_PM_ACTIVE);
	if (ret < 0) {
		dev_err(p->dev, "%s: failed to set PM_ACTIVE\n", __func__);
		ret = -EINVAL;
		goto out_unlock;
	}

	if (va_reg == DBMDX_REGN_VT_AUDIO_HISTORY_CFG) {
		ret = dbmdx_set_backlog_len(p, val);
		if (ret < 0) {
			dev_err(p->dev, "%s: set history error\n", __func__);
			ret = -EINVAL;
			goto out_unlock;
		}
	} else if (va_reg == DBMDX_REGN_DIGITAL_GAIN) {
		short sval = ((short)val - DIGITAL_GAIN_TLV_SHIFT) & 0x0fff;

		ret = dbmdx_send_cmd(p, va_reg | sval, NULL);
		if (ret < 0) {
			dev_err(p->dev, "%s: write 0x%x to 0x%x error\n",
				__func__, val, reg);
			ret = -EINVAL;
			goto out_unlock;
		}

		if (p->va_active_mic_config != DBMDX_MIC_MODE_ANALOG)
			p->va_cur_digital_mic_digital_gain = (int)sval;
		else
			p->va_cur_analog_mic_digital_gain = (int)sval;

	}  else if (va_reg == DBMDX_REGN_MICROPHONE_ANALOG_GAIN) {
		ret = dbmdx_send_cmd(p, va_reg | (val & 0xffff), NULL);
		if (ret < 0) {
			dev_err(p->dev, "%s: write 0x%x to 0x%x error\n",
				__func__, val, reg);
			ret = -EINVAL;
			goto out_unlock;
		}
		p->va_cur_analog_mic_analog_gain = (int)val;
	} else {
		ret = dbmdx_send_cmd(p, va_reg | (val & 0xffff), NULL);
		if (ret < 0) {
			dev_err(p->dev, "%s: write 0x%x to 0x%x error\n",
				__func__, val, reg);
			ret = -EINVAL;
			goto out_unlock;
		}
	}

	ret = 0;

out_unlock:
	dbmdx_set_power_mode(p, DBMDX_PM_FALLING_ASLEEP);
	p->unlock(p);
	return ret;
}


static int dbmdx_va_control_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
			(struct soc_mixer_control *)kcontrol->private_value;
	unsigned short val = ucontrol->value.integer.value[0];
	unsigned short reg = mc->reg;
	int max = mc->max;
	int mask = (1 << fls(max)) - 1;
	int ret;
#if defined(SOC_CONTROLS_FOR_DBMDX_CODEC_ONLY)
#ifdef SND_SOC_COMPONENT
	struct snd_soc_component *component = snd_kcontrol_chip(kcontrol);
	struct dbmdx_private *p = snd_soc_component_get_drvdata(component);

#else
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct dbmdx_private *p = snd_soc_codec_get_drvdata(codec);
#endif    
#else
	struct dbmdx_private *p = dbmdx_data;
#endif

	if (!p)
		return -EAGAIN;

	if (!p->device_ready) {
		dev_err(p->dev, "%s: device not ready\n", __func__);
		return -EAGAIN;
	}

	dev_dbg(p->dev, "%s: ------- VA control ------\n", __func__);
	if (p->active_fw != DBMDX_FW_VA) {
		dev_dbg(p->dev, "%s: VA firmware not active\n", __func__);
		return -EAGAIN;
	}

	val &= mask;

	ret = dbmdx_va_update_reg(p, reg, val);

	if (ret < 0 && !p->pdata->va_recovery_disabled) {

		int recovery_res;

		if (dbmdx_va_alive_with_lock(p) == 0) {
			dev_err(p->dev,
				"%s: DBMDX response has been verified\n",
				__func__);
			goto out;
		}

		dev_err(p->dev, "%s: Performing recovery #1\n", __func__);

		recovery_res = dbmdx_perform_recovery(p);

		if (recovery_res) {
			dev_err(p->dev, "%s: recovery failed\n", __func__);
			goto out;
		}

		ret = dbmdx_va_update_reg(p, reg, val);

		if (ret == 0) {
			dev_err(p->dev,
				"%s: Reg. was updated after succesfull recovery\n",
				__func__);
			goto out;
		}

		if (dbmdx_va_alive_with_lock(p) == 0) {
			dev_err(p->dev,
				"%s: DBMDX response has been verified\n",
				__func__);
			goto out;
		}

		dev_err(p->dev, "%s: Performing recovery #2\n", __func__);

		recovery_res = dbmdx_perform_recovery(p);

		if (recovery_res) {
			dev_err(p->dev, "%s: recovery failed\n", __func__);
			goto out;
		}

	}
out:
	return ret;
}

static const char *const dbmdx_operation_mode_texts[] = {
	"Idle",
	"Sleep_pll_on",
	"Sleep_pll_off",
	"Hibernate",
	"Detection",
	"Buffering",
	"Not_active",
};

static const struct soc_enum dbmdx_operation_mode_enum =
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(dbmdx_operation_mode_texts),
			    dbmdx_operation_mode_texts);

static const unsigned int dbmdx_digital_gain_tlv[] = {
	TLV_DB_RANGE_HEAD(1),
	DIGITAL_GAIN_TLV_MIN, DIGITAL_GAIN_TLV_MAX,
	TLV_DB_SCALE_ITEM(-6000, 50, 0),
};

#ifdef EXTERNAL_SOC_AMODEL_LOADING_ENABLED
#ifdef SOC_BYTES_EXT_HAS_KCONTROL_FIELD
static int dbmdx_external_amodel_put(struct snd_kcontrol *kcontrol,
				 const unsigned int __user *bytes,
				 unsigned int size)
{
#if defined(SOC_CONTROLS_FOR_DBMDX_CODEC_ONLY)
#ifdef SND_SOC_COMPONENT
	struct snd_soc_component *component = snd_kcontrol_chip(kcontrol);
	struct dbmdx_private *p = snd_soc_component_get_drvdata(component);

#else
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct dbmdx_private *p = snd_soc_codec_get_drvdata(codec);
#endif 
#else
	struct dbmdx_private *p = dbmdx_data;
#endif
#else /* SOC_BYTES_EXT_HAS_KCONTROL_FIELD */
static int dbmdx_external_amodel_put(const unsigned int __user *bytes,
				 unsigned int size)
{
	struct dbmdx_private *p = dbmdx_data;
#endif /* SOC_BYTES_EXT_HAS_KCONTROL_FIELD */
	int ret = 0;
	char *data_buf;

	if (!p)
		return -EAGAIN;

	if (!p->device_ready) {
		dev_err(p->dev, "%s: device not ready\n", __func__);
		return -EAGAIN;
	}

	/* Buffer format is:
	 * 8B:TLV Header + 1B:Model type + 1B:Model Options + 1B:Number of files
	 * 4B:1st File Len + 1st File Data + [4B:2nd File Len + 2nd File Data]..
	 */
#ifdef SOC_TLV_HEADER_ENABLED
	if (size < 15) {
#else
	if (size < 9) {
#endif
		dev_err(p->dev, "%s: Header is too short\n", __func__);
		return -EINVAL;
	}

	if (size > MAX_AMODEL_SIZE) {
		dev_err(p->dev, "%s: Size exceeds max amodel size\n", __func__);
		return -EINVAL;
	}

	data_buf = vmalloc(MAX_AMODEL_SIZE);

	if (!data_buf)
		return -ENOMEM;

	dev_info(p->dev, "%s: Buffer size is %d\n", __func__, size);

#ifdef SOC_TLV_HEADER_ENABLED
	/* Skips the TLV header. */
	bytes += 2;
#endif
	if (copy_from_user(data_buf, bytes, size)) {
		dev_err(p->dev,
			"%s: Error during copying data from user\n", __func__);
		ret = -EFAULT;
		goto out_mem;
	}

	if (!data_buf) {
		dev_err(p->dev, "%s: no data_buf\n", __func__);
		ret = -EFAULT;
		goto out_mem;
	}

	p->usr_op_lock(p);

	ret = dbmdx_acoustic_model_build_from_external(p, data_buf, size);
	if (ret < 0) {
		dev_err(p->dev,
			"%s: Error building amodel from provided buffer\n",
			__func__);
		p->usr_op_unlock(p);
		return -EFAULT;
	}

	p->usr_op_unlock(p);

out_mem:
	vfree(data_buf);
	return ret;

}
#endif

static int dbmdx_wordid_get(struct snd_kcontrol *kcontrol,
			    struct snd_ctl_elem_value *ucontrol)
{
#if defined(SOC_CONTROLS_FOR_DBMDX_CODEC_ONLY)
#ifdef SND_SOC_COMPONENT
	struct snd_soc_component *component = snd_kcontrol_chip(kcontrol);
	struct dbmdx_private *p = snd_soc_component_get_drvdata(component);

#else
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct dbmdx_private *p = snd_soc_codec_get_drvdata(codec);
#endif 
#else
	struct dbmdx_private *p = dbmdx_data;
#endif
	int ret = 0;


	if (!p)
		return -EAGAIN;

	if (!p->device_ready) {
		dev_err(p->dev, "%s: device not ready\n", __func__);
		return -EAGAIN;
	}


	/* VA controls */
	if (p->active_fw != DBMDX_FW_VA) {
		ucontrol->value.integer.value[0] = 0;
		dev_dbg(p->dev, "%s: VA firmware not active\n", __func__);
		goto out;
	}

	ucontrol->value.integer.value[0] = p->va_last_word_id;
out:
	return ret;
}

static int dbmdx_wordid_put(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
#if defined(SOC_CONTROLS_FOR_DBMDX_CODEC_ONLY)
#ifdef SND_SOC_COMPONENT
	struct snd_soc_component *component = snd_kcontrol_chip(kcontrol);
	struct dbmdx_private *p = snd_soc_component_get_drvdata(component);

#else
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct dbmdx_private *p = snd_soc_codec_get_drvdata(codec);
#endif 
#else
	struct dbmdx_private *p = dbmdx_data;
#endif

	if (!p)
		return -EAGAIN;

	if (!p->device_ready) {
		dev_err(p->dev, "%s: device not ready\n", __func__);
		return -EAGAIN;
	}

	dev_err(p->dev, "%s: WORDID is not writable register\n", __func__);

	return -EIO;
}
static int dbmdx_va_capture_on_detect_get(struct snd_kcontrol *kcontrol,
				     struct snd_ctl_elem_value *ucontrol)
{
#if defined(SOC_CONTROLS_FOR_DBMDX_CODEC_ONLY)
#ifdef SND_SOC_COMPONENT
	struct snd_soc_component *component = snd_kcontrol_chip(kcontrol);
	struct dbmdx_private *p = snd_soc_component_get_drvdata(component);

#else
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct dbmdx_private *p = snd_soc_codec_get_drvdata(codec);
#endif 
#else
	struct dbmdx_private *p = dbmdx_data;
#endif
	int ret = 0;

	if (!p)
		return -EAGAIN;

	if (!p->device_ready) {
		dev_err(p->dev, "%s: device not ready\n", __func__);
		return -EAGAIN;
	}

	p->usr_op_lock(p);

	ucontrol->value.integer.value[0] = p->va_capture_on_detect;

	p->usr_op_unlock(p);

	return ret;
}

static int dbmdx_va_capture_on_detect_set(struct snd_kcontrol *kcontrol,
				     struct snd_ctl_elem_value *ucontrol)
{
#if defined(SOC_CONTROLS_FOR_DBMDX_CODEC_ONLY)
#ifdef SND_SOC_COMPONENT
	struct snd_soc_component *component = snd_kcontrol_chip(kcontrol);
	struct dbmdx_private *p = snd_soc_component_get_drvdata(component);

#else
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct dbmdx_private *p = snd_soc_codec_get_drvdata(codec);
#endif 
#else
	struct dbmdx_private *p = dbmdx_data;
#endif
	int ret = 0;

	p->usr_op_lock(p);

	p->va_capture_on_detect = ucontrol->value.integer.value[0];

	p->usr_op_unlock(p);

	return ret;
}

static int dbmdx_usecase_manager_get(struct snd_kcontrol *kcontrol,
				 struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = 0x7FFFFFFF;
	return 0;
}

static int dbmdx_usecase_manager_set(struct snd_kcontrol *kcontrol,
				 struct snd_ctl_elem_value *ucontrol)
{
#if defined(SOC_CONTROLS_FOR_DBMDX_CODEC_ONLY)
#ifdef SND_SOC_COMPONENT
	struct snd_soc_component *component = snd_kcontrol_chip(kcontrol);
	struct dbmdx_private *p = snd_soc_component_get_drvdata(component);
#else
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct dbmdx_private *p = snd_soc_codec_get_drvdata(codec);
#endif
#else
	struct dbmdx_private *p = dbmdx_data;
#endif
	unsigned int value = ucontrol->value.integer.value[0];
	int ret;

	if (!p)
		return -EAGAIN;

	if (!p->device_ready) {
		dev_err(p->dev, "%s: device not ready\n", __func__);
		return -EAGAIN;
	}

	p->usr_op_lock(p);

	dev_dbg(p->dev,	"%s: Usecase Manager cmd=0x%x\n", __func__, value);

	ret = dbmdx_usecase_manager(p, value);
	if (ret < 0)
		dev_err(p->dev, "%s: Usecase manager reported error\n",
			__func__);

	if (((value & 0x0fff00) >> 8) == 0x620){
	     es7210_resume();
	}
	else if(value == 0x0){
	     es7210_suspend();
	}
	else{
	    dev_dbg(p->dev,"%s: invalid cmd=0x%x\n", __func__, value);
	}
	p->usr_op_unlock(p);

	return ret;
}

static const char * const dbmdx_microphone_mode_texts[] = {
	"DigitalLeft",
	"DigitalRight",
	"DigitalStereoTrigOnLeft",
	"DigitalStereoTrigOnRight",
	"Analog",
#ifdef DBMDX_4CHANNELS_SUPPORT
	"Digital4Channels",
#endif
	"Disable",
};

static const struct soc_enum dbmdx_microphone_mode_enum =
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(dbmdx_microphone_mode_texts),
			    dbmdx_microphone_mode_texts);

static const char * const dbmdx_va_off_on_texts[] = {
	"OFF",
	"ON",
};

static const struct soc_enum dbmdx_va_off_on_enum =
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(dbmdx_va_off_on_texts),
			    dbmdx_va_off_on_texts);

static const struct snd_kcontrol_new dbmdx_va_snd_controls[] = {
	/*
	 * VA mixer controls
	 */
	 SOC_SINGLE_EXT_TLV("Digital gain", 0x04, 0, DIGITAL_GAIN_TLV_MAX, 0,
		dbmdx_va_control_get, dbmdx_va_control_put,
		dbmdx_digital_gain_tlv),
	SOC_SINGLE_EXT("Analog gain", 0x16, 0, 0xff, 0,
		dbmdx_va_control_get, dbmdx_va_control_put),
	SOC_SINGLE_EXT("Word ID", 0, 0, 0x0fff, 0,
		dbmdx_wordid_get, dbmdx_wordid_put),
	SOC_SINGLE("Trigger Level",
		   VA_MIXER_REG(DBMDX_REGN_VT_REG_TH),
		   0, 0xffff, 0),
	SOC_SINGLE("Verification Level",
		   VA_MIXER_REG(DBMDX_REGN_VT_REG_LL),
		   0, 0xffff, 0),
	SOC_SINGLE_EXT("Backlog size", 0x12, 0, 0x0fff, 0,
		dbmdx_va_control_get, dbmdx_va_control_put),
	SOC_ENUM_EXT("Capture on detection", dbmdx_va_off_on_enum,
		dbmdx_va_capture_on_detect_get, dbmdx_va_capture_on_detect_set),
#ifdef EXTERNAL_SOC_AMODEL_LOADING_ENABLED
	SND_SOC_BYTES_TLV("Acoustic Model", MAX_AMODEL_SIZE, NULL,
				dbmdx_external_amodel_put),
#endif
};

static const struct snd_kcontrol_new dbmdx_va_ve_snd_controls[] = {
	/*
	 * VA_VE mixer controls
	 */
	SOC_SINGLE_EXT("Backlog size", 0x12, 0, 0x0fff, 0,
		dbmdx_va_control_get, dbmdx_va_control_put),
#ifdef EXTERNAL_SOC_AMODEL_LOADING_ENABLED
	SND_SOC_BYTES_TLV("Acoustic Model", MAX_AMODEL_SIZE, NULL,
				dbmdx_external_amodel_put),
#endif
	SOC_SINGLE_EXT("Usecase Manager", 0, 0, 0x7FFFFFFF, 0,
		dbmdx_usecase_manager_get, dbmdx_usecase_manager_set),
};


#ifdef SND_SOC_COMPONENT
static int dbmdx_set_bias_level(struct snd_soc_component *component,
 				enum snd_soc_bias_level level)
#else
static int dbmdx_set_bias_level(struct snd_soc_codec *codec,
				enum snd_soc_bias_level level)
#endif
{
#ifdef SND_SOC_COMPONENT
	dev_dbg(component->dev, "%s: level %d\n", __func__, (int)level);
#else
	dev_dbg(codec->dev, "%s: level %d\n", __func__, (int)level);
#endif
	switch (level) {
	case SND_SOC_BIAS_ON:
		break;
	case SND_SOC_BIAS_PREPARE:
		break;
	case SND_SOC_BIAS_STANDBY:
		break;
	case SND_SOC_BIAS_OFF:
		break;
	default:
		return -EINVAL;
	}

	/* change to new state , removed in 4.x kernel */
#ifdef SND_SOC_COMPONENT
	/* component->dapm.bias_level = level; */
#else
	/* codec->dapm.bias_level = level; */
#endif

	return 0;
}

static int dbmdx_get_dai_drivers(struct dbmdx_private *p)
{
	struct snd_soc_dai_driver *dais;
	unsigned int num_dais = 0;
	int ret = 0;

	/* construct dai driver array depending of features */
	if (p->pdata->feature_va_ve)
		num_dais += ARRAY_SIZE(dbmdx_va_dais);

	dais = devm_kzalloc(p->dev,
			    num_dais * sizeof(*dais),
			    GFP_KERNEL);

	if (!dais) {
		dev_err(p->dev, "%s: out of memory\n", __func__);
		ret = -ENOMEM;
		goto out_err;
	}

	dev_err(p->dev, "%s: num DAIs: %u\n", __func__, num_dais);

	p->num_dais = num_dais;
	p->dais = dais;
	num_dais = 0;
	if (p->pdata->feature_va_ve) {
		memcpy(dais, dbmdx_va_dais, sizeof(dbmdx_va_dais));
		num_dais += ARRAY_SIZE(dbmdx_va_dais);
	}

out_err:
	return ret;
}

#ifdef SND_SOC_COMPONENT
int dbmdx_dev_probe(struct snd_soc_component *component)
#else
int dbmdx_dev_probe(struct snd_soc_codec *codec)
#endif
{
	int ret;
#ifdef SND_SOC_COMPONENT
	struct dbmdx_private *p = dev_get_drvdata(component->dev);
#else
	struct dbmdx_private *p = dev_get_drvdata(codec->dev);
#endif     

#ifdef SND_SOC_COMPONENT
//	component->control_data = NULL;
#else
	codec->control_data = NULL;
#endif
#ifdef SND_SOC_COMPONENT
	dev_dbg(component->dev, "%s: DBMDX component probe\n", __func__);
#else
	dev_dbg(codec->dev, "%s: DBMDX Codec probe\n", __func__);
#endif

	if (!p || !p->pdata) {
#ifdef SND_SOC_COMPONENT
		dev_err(component->dev, "%s: Device not ready, defer...\n",
			__func__);
#else
		dev_err(codec->dev, "%s: Device not ready, defer...\n",
			__func__);

#endif
		return -EPROBE_DEFER;
	}

	if (p->pdata->feature_va) {
#ifdef SND_SOC_COMPONENT    
		ret = snd_soc_add_component_controls(component,
					dbmdx_va_snd_controls,
					ARRAY_SIZE(dbmdx_va_snd_controls));
#else
		ret = snd_soc_add_codec_controls(codec,
					dbmdx_va_snd_controls,
					ARRAY_SIZE(dbmdx_va_snd_controls));

#endif                    
		if (ret) {
#ifdef SND_SOC_COMPONENT        
			dev_err(component->dev,
				"%s: failed to register VA controls\n",
				__func__);
#else
			dev_err(codec->dev,
				"%s: failed to register VA controls\n",
				__func__);

#endif                
			goto out_err;
		}
#ifdef SND_SOC_COMPONENT        
		dev_info(component->dev,
			 "%s: %d VA controls registered\n",
			 __func__, (int)(ARRAY_SIZE(dbmdx_va_snd_controls)));
#else
		dev_info(codec->dev,
			 "%s: %d VA controls registered\n",
			 __func__, (int)(ARRAY_SIZE(dbmdx_va_snd_controls)));
#endif             
	}
	if (p->pdata->feature_va_ve) {
#ifdef SND_SOC_COMPONENT    
		ret = snd_soc_add_component_controls(component,
				dbmdx_va_ve_snd_controls,
				ARRAY_SIZE(dbmdx_va_ve_snd_controls));
#else
		ret = snd_soc_add_codec_controls(codec,
				dbmdx_va_ve_snd_controls,
				ARRAY_SIZE(dbmdx_va_ve_snd_controls));

#endif                
		if (ret) {
#ifdef SND_SOC_COMPONENT        
			dev_err(component->dev,
				"%s(): adding VA_VE controls failed\n",
				__func__);
#else
			dev_err(codec->dev,
				"%s(): adding VA_VE controls failed\n",
				__func__);

#endif                
			ret = -EIO;
		}
#ifdef SND_SOC_COMPONENT        
		dev_info(component->dev,
			 "%s: %d VA_VE controls added\n",
			 __func__, (int)(ARRAY_SIZE(dbmdx_va_ve_snd_controls)));
#else
		dev_info(codec->dev,
			 "%s: %d VA_VE controls added\n",
			 __func__, (int)(ARRAY_SIZE(dbmdx_va_ve_snd_controls)));

#endif             
	}
	ret = 0;

#ifdef SND_SOC_COMPONENT
    dev_info(component->dev, "%s: success\n", __func__);
#else    
	dev_info(codec->dev, "%s: success\n", __func__);
#endif    

out_err:
	return ret;
}

#ifdef SND_SOC_COMPONENT
void dbmdx_dev_remove(struct snd_soc_component *component)
{
	dev_dbg(component->dev, "%s\n", __func__);
//	return 0;
}

#else
int dbmdx_dev_remove(struct snd_soc_codec *codec)
{
	dev_dbg(codec->dev, "%s\n", __func__);
	return 0;
}
#endif

#ifdef CONFIG_PM
#ifdef SND_SOC_COMPONENT
static int dbmdx_dev_suspend(struct snd_soc_component *component)
{
	dev_dbg(component->dev, "%s\n", __func__);
	return 0;
}

#else
static int dbmdx_dev_suspend(struct snd_soc_codec *codec)
{
	dev_dbg(codec->dev, "%s\n", __func__);
	return 0;
}
#endif

#ifdef SND_SOC_COMPONENT
static int dbmdx_dev_resume(struct snd_soc_component *component)
{
	dev_dbg(component->dev, "%s\n", __func__);
	return 0;
}

#else
static int dbmdx_dev_resume(struct snd_soc_codec *codec)
{
	dev_dbg(codec->dev, "%s\n", __func__);
	return 0;
}
#endif
#else
#define dbmdx_dev_suspend NULL
#define dbmdx_dev_resume NULL
#endif

#ifdef SND_SOC_COMPONENT
static struct snd_soc_component_driver soc_component_dev_dbmdx = {
#else
static struct snd_soc_codec_driver soc_codec_dev_dbmdx = {
#endif
	.probe   = dbmdx_dev_probe,
	.remove  = dbmdx_dev_remove,
	.suspend = dbmdx_dev_suspend,
	.resume  = dbmdx_dev_resume,
	.set_bias_level = dbmdx_set_bias_level,
	.read = dbmdx_dev_read,
	.write = dbmdx_dev_write,
#ifdef SND_SOC_COMPONENT
#else
	.reg_cache_size = 0,
	.reg_word_size = 0,
	.reg_cache_default = NULL,
	.ignore_pmdown_time = true,
#endif
};


#if !defined(SOC_CONTROLS_FOR_DBMDX_CODEC_ONLY)
#ifdef SND_SOC_COMPONENT
int dbmdx_remote_add_component_controls(struct snd_soc_component *component)
#else
int dbmdx_remote_add_codec_controls(struct snd_soc_codec *codec)
#endif
{
	int ret = 0;
	int rc;
	struct dbmdx_private *p = dbmdx_data;

	if (!p || !p->pdata) {
#ifdef SND_SOC_COMPONENT    
		remote_component = component;
#else
		remote_codec = codec;
#endif        
		return 0;
	}

#ifdef SND_SOC_COMPONENT
	remote_component = component;
#else
	remote_codec = codec;
#endif

#ifdef SND_SOC_COMPONENT
	dev_dbg(component->dev, "%s start\n", __func__);
#else    
	dev_dbg(codec->dev, "%s start\n", __func__);
#endif
        
	if (p->pdata->feature_va) {
#ifdef SND_SOC_COMPONENT
		rc = snd_soc_add_component_controls(component, dbmdx_va_snd_controls,
				ARRAY_SIZE(dbmdx_va_snd_controls));

#else    
		rc = snd_soc_add_codec_controls(codec, dbmdx_va_snd_controls,
				ARRAY_SIZE(dbmdx_va_snd_controls));
#endif                
		if (rc) {
#ifdef SND_SOC_COMPONENT
			dev_err(component->dev, "%s(): adding VA controls failed\n",
				__func__);

#else        
			dev_err(codec->dev, "%s(): adding VA controls failed\n",
				__func__);
#endif                
			ret = -EIO;
		}
#ifdef SND_SOC_COMPONENT        
		dev_info(component->dev,
			 "%s: %d VA controls added\n",
			 __func__, (int)(ARRAY_SIZE(dbmdx_va_snd_controls)));
#else

		dev_info(codec->dev,
			 "%s: %d VA controls added\n",
			 __func__, (int)(ARRAY_SIZE(dbmdx_va_snd_controls)));

#endif
             
	}
	if (p->pdata->feature_va_ve) {

#ifdef SND_SOC_COMPONENT
		rc = snd_soc_add_component_controls(component, dbmdx_va_ve_snd_controls,
				ARRAY_SIZE(dbmdx_va_ve_snd_controls));
        
#else
		rc = snd_soc_add_codec_controls(codec, dbmdx_va_ve_snd_controls,
				ARRAY_SIZE(dbmdx_va_ve_snd_controls));
#endif            

    
		if (rc) {
#ifdef SND_SOC_COMPONENT
			dev_err(component->dev,
				"%s(): adding VA_VE controls failed\n",
				__func__);

#else                
			dev_err(codec->dev,
				"%s(): adding VA_VE controls failed\n",
				__func__);
#endif                
			ret = -EIO;
		}
#ifdef SND_SOC_COMPONENT        
		dev_info(component->dev,
			 "%s: %d VA_VE controls added\n",
			 __func__, (int)(ARRAY_SIZE(dbmdx_va_ve_snd_controls)));
#else
		dev_info(codec->dev,
			 "%s: %d VA_VE controls added\n",
			 __func__, (int)(ARRAY_SIZE(dbmdx_va_ve_snd_controls)));
#endif
        }

#ifdef SND_SOC_COMPONENT    
	p->remote_component_in_use = 1;
#else
	p->remote_codec_in_use = 1;
#endif    

	return ret;
}

#ifdef SND_SOC_COMPONENT
EXPORT_SYMBOL(dbmdx_remote_add_component_controls);
#else
EXPORT_SYMBOL(dbmdx_remote_add_codec_controls);
#endif

#else
#ifdef SND_SOC_COMPONENT
int dbmdx_remote_add_component_controls(struct snd_soc_component *component)
#else
int dbmdx_remote_add_codec_controls(struct snd_soc_codec *codec)
#endif
{
#ifdef SND_SOC_COMPONENT
	dev_dbg(component->dev, "%s\n", __func__);
#else
	dev_dbg(codec->dev, "%s\n", __func__);
#endif    

	return 0;
}
#endif

#endif /* ALSA_SOC_INTERFACE_NOT_SUPPORTED */

void dbmdx_remote_register_event_callback(event_cb func)
{
	if (dbmdx_data)
		dbmdx_data->event_callback = func;
	else
		g_event_callback = func;
}
EXPORT_SYMBOL(dbmdx_remote_register_event_callback);

void dbmdx_remote_register_set_i2c_freq_callback(set_i2c_freq_cb func)
{
	if (dbmdx_data)
		dbmdx_data->set_i2c_freq_callback = func;
	else
		g_set_i2c_freq_callback = func;
}
EXPORT_SYMBOL(dbmdx_remote_register_set_i2c_freq_callback);

static int dbmdx_process_detection_irq(struct dbmdx_private *p,
					bool to_start_buffering)
{
	u16 event_id = 23;
	u16 alt_event_id = 23;
	u16 vt_index_val;
	u32 vt_reg;
	int vt_index;
	u16 score = 0;
	struct vt_engine *det_vt_engine;
	u16 passphrase_length_in_samples = 0;
	int ret;

	p->va_flags.number_of_vt_since_start++;


#if !defined(DBMDX_PSAD_SUPPORTED)
	if (p->va_debug_mode & DBMDX_DEBUG_MODE_FW_LOG)
		msleep(DBMDX_MSLEEP_DBG_AFTER_DETECTION);
#endif

	/* Stop PCM streaming work */
	ret = dbmdx_suspend_pcm_streaming_work(p);
	if (ret < 0)
		dev_err(p->dev, "%s: Failed to suspend PCM Streaming Work\n",
			__func__);

	/* flush pending sv work if any */
	p->va_flags.buffering = 0;
	flush_work(&p->sv_work);

	p->lock(p);

	dbmdx_set_power_mode(p, DBMDX_PM_ACTIVE);

	if (dbmdx_switch_to_va_chip_interface(p, DBMDX_CMD_INTERFACE)) {
		dev_err(p->dev,
			"%s Error switching to (VA) CMD interface\n",
			__func__);
		return -EIO;
	}

	ret = dbmdx_send_cmd(p,	DBMDX_REGN_DETECT_AND_SYS_EVENTS,
								&vt_index_val);
	if (ret < 0) {
		dev_err(p->dev,	"%s: failed to read Event ID\n", __func__);
		ret = -EIO;
		goto out_unlock;
	}

	ret = dbmdx_send_cmd(p,	DBMDX_REGN_DETECT_AND_SYS_EVENTS |
					DBMDX_REGV_RESET_DET_STATUS, NULL);
	if (ret < 0) {
		dev_err(p->dev,	"%s: failed to Reset Event ID\n",
								__func__);
		ret = -EIO;
		goto out_unlock;
	}
	if (vt_index_val & DBMDX_REGV_VT1_DET) {
		vt_index = 0;
		vt_reg = DBMDX_REGN_VT1_REGS_OFFSET;
	} else if (vt_index_val & DBMDX_REGV_VT2_DET) {
		vt_index = 1;
		vt_reg = DBMDX_REGN_VT2_REGS_OFFSET;
	} else if (vt_index_val & DBMDX_REGV_VT3_DET) {
		vt_index = 2;
		vt_reg = DBMDX_REGN_VT3_REGS_OFFSET;
	} else {
		dev_err(p->dev,	"%s: Unknown VT Index in Status Reg\n",
								__func__);
		ret = -EIO;
		goto out_unlock;
	}

	det_vt_engine = &(p->vt_engines[vt_index]);

	if (!(det_vt_engine->enabled) ||
		!(det_vt_engine->internal_state ==  MODEL_STATE_STARTED)) {
		dev_err(p->dev,	"%s: Unexpected EVENT for VT Engine %d\n",
						__func__, vt_index + 1);
		ret = -EINVAL;
		goto out_unlock;

	}

	if (det_vt_engine->detection_type == DETECTION_TYPE_VOICE_ENERGY) {
		dev_info(p->dev, "%s: VOICE ENERGY (VT Engine %d)\n",
						__func__, vt_index + 1);
		event_id = 0;
	} else if (det_vt_engine->detection_type == DETECTION_TYPE_PHRASE)  {

		dev_info(p->dev, "%s: PASSPHRASE (VT Engine %d)\n",
						__func__, vt_index + 1);

		p->va_cur_backlog_length = *(det_vt_engine->history);

#if defined(DBMDX_PSAD_SUPPORTED)
		ret = dbmdx_va_ve_update_psad_data(p,
						true,
						DBMDX_OKG_MODEL_SELECTED);
		if (ret < 0) {
			dev_err(p->dev,	"%s: failed to update PSAD data\n",
					__func__);
			ret = -EIO;
			goto out_unlock;
		}
#endif
		ret = dbmdx_send_cmd(p,	vt_reg | DBMDX_REGN_VT_REG_WORD_ID,
								&event_id);
		if (ret < 0) {
			dev_err(p->dev,	"%s: failed reading event id\n",
								__func__);
			ret = -EIO;
			goto out_unlock;
		}

		ret = dbmdx_send_cmd(p, vt_reg | DBMDX_REGN_VT_REG_ALT_WORD_ID,
								&alt_event_id);

		if (ret < 0) {
			dev_err(p->dev, "%s: Failed reading WordID\n",
								__func__);
			ret = -EIO;
			goto out_unlock;
		}

		ret = dbmdx_send_cmd(p, vt_reg |
						DBMDX_REGN_VT_REG_PHRASE_LENGTH,
						&passphrase_length_in_samples);
		if (ret < 0) {
			dev_err(p->dev,
				"%s: failed reading passphrase length\n",
				__func__);
			ret = -EIO;
			goto out_unlock;
		}

		dev_info(p->dev,
			"%s: last WordID:\t%d, EventID:\t%d, VT Length:\t%u samples\n",
			__func__, event_id, (event_id & 0xff),
			passphrase_length_in_samples);

		event_id = (event_id & 0xff);

	} else if (det_vt_engine->detection_type ==
					DETECTION_TYPE_VOICE_COMMAND) {
		dev_info(p->dev, "%s: VOICE_COMMAND (VT Engine %d)\n",
						__func__, vt_index + 1);

		ret = dbmdx_send_cmd(p,	vt_reg | DBMDX_REGN_VT_REG_WORD_ID,
								&event_id);
		if (ret < 0) {
			dev_err(p->dev,	"%s: failed reading event id\n",
								__func__);
			ret = -EIO;
			goto out_unlock;
		}

		event_id = (event_id & 0xff);

		/*for determining voice command, mask should be up to 0x100 */
		event_id += 0x100;
		dev_info(p->dev, "%s: last WordID:%d\n", __func__, event_id);
	} else if (det_vt_engine->detection_type == DETECTION_TYPE_DUAL) {
		dev_info(p->dev, "%s: VOICE_DUAL\n", __func__);

		ret = dbmdx_send_cmd(p,	vt_reg | DBMDX_REGN_VT_REG_WORD_ID,
								&event_id);
		if (ret < 0) {
			dev_err(p->dev,	"%s: failed reading event id\n",
								__func__);
			ret = -EIO;
			goto out_unlock;
		}

		dev_info(p->dev, "%s: last WordID: %d, EventID: %d\n",
			__func__, event_id, (event_id & 0xff));

		event_id = (event_id & 0xff);
		to_start_buffering = false;
	}

	ret = dbmdx_send_cmd(p, vt_reg | DBMDX_REGN_VT_REG_FINAL_SCORE,	&score);

	if (ret < 0) {
		dev_err(p->dev,	"%s: failed reading Score\n", __func__);
		ret = -EIO;
		goto out_unlock;
	}

	dev_info(p->dev, "%s: Score:%d\n", __func__, score);

	p->va_last_word_id = event_id;

	if (p->pdata->detection_after_buffering !=
		DETECTION_AFTER_BUFFERING_MODE_ENABLE_ALL) {

		ret = dbmdx_va_disable_vte_detection(p, det_vt_engine,
						det_vt_engine->detection_type);

		if (ret < 0) {
			dev_err(p->dev,
				  "%s: failed to set VTE recognition mode\n",
				  __func__);
			ret = -EIO;
			goto out_unlock;
		}
	}

	/* Start Buffering before sending events,
	 * unless i2s-buffering is needed
	 */
	if ((to_start_buffering) &&
		((p->pdata->alsa_streaming_options &
				DBMDX_ALSA_STREAMING_DISABLED) ||
		!p->va_ve_flags.cur_usecase->usecase_supports_i2s_buffering)) {
		/* flush fifo */
		kfifo_reset(&p->detection_samples_kfifo);

		p->va_flags.buffering_paused = 0;
		p->va_flags.buffering = 1;
		dbmdx_schedule_work(p, &p->sv_work);
	}

	p->unlock(p);

	if (p->event_callback)
		p->event_callback(event_id);

	if (p->pdata->send_uevent_on_detection) {
		char uevent_buf[256];
		char * const envp[] = { uevent_buf, NULL };
#ifndef SHORT_DETECTION_STATS
		int backlog_in_samples;
		int backlog_mode;
		ssize_t off = 0;
		int act_passphrase_length_in_samples;
		int act_backlog_in_samples;
		int num_of_channels = p->pdata->detection_buffer_channels;
		if (det_vt_engine->detection_type == DETECTION_TYPE_PHRASE) {

			if (p->pdata->detection_buffer_channels == 0)
				num_of_channels = p->pdata->va_audio_channels;

			backlog_in_samples =
				((u32)(*(det_vt_engine->history) & 0xffc)) *
				p->pdata->va_buffering_pcm_rate *
				num_of_channels /
				1000;
			backlog_mode =
				(u32)(*(det_vt_engine->history) & 0x3);
			act_passphrase_length_in_samples =
					passphrase_length_in_samples *
					num_of_channels;
			if (backlog_mode == 2)
				act_backlog_in_samples = backlog_in_samples +
					act_passphrase_length_in_samples;
			else
				act_backlog_in_samples = backlog_in_samples +
					FW_BACKLOG_PADDING_IN_SAMPLES;

			off = snprintf(uevent_buf, sizeof(uevent_buf),
			"VOICE_WAKEUP EVENT_TYPE=Detection EVENT_ID=%d",
					event_id);

			off += snprintf(uevent_buf + off,
					sizeof(uevent_buf) - off,
					" SCORE=%d", score);

			off += snprintf(uevent_buf + off,
					sizeof(uevent_buf) - off,
					" BACKLOG_MODE=%d", backlog_mode);

			off += snprintf(uevent_buf + off,
					sizeof(uevent_buf) - off,
					" PASSPHRASE_LEN_IN_SAMPLES=%d",
					act_passphrase_length_in_samples);

			off += snprintf(uevent_buf + off,
					sizeof(uevent_buf) - off,
					" BACKLOG_LEN_IN_SAMPLES=%d",
					backlog_in_samples);

			off += snprintf(uevent_buf + off,
					sizeof(uevent_buf) - off,
					" FULL_BACKLOG_LEN_IN_SAMPLES=%d",
					act_backlog_in_samples);

		} else
			snprintf(uevent_buf, sizeof(uevent_buf),
				"VOICE_WAKEUP EVENT_TYPE=Detection EVENT_ID=%d",
				event_id);
#else
		snprintf(uevent_buf, sizeof(uevent_buf),
		"VOICE_WAKEUP EVENT_TYPE=Detection EVENT_ID=%d SCORE=%d",
			event_id, score);
#endif
		dev_info(p->dev, "%s: Sending uevent: %s\n",
			__func__, uevent_buf);

		ret = kobject_uevent_env(&p->dev->kobj, KOBJ_CHANGE,
			(char **)envp);

		if (ret)
			dev_err(p->dev,	"%s: Sending uevent failed %d\n",
				__func__, ret);
	}

	return 0;

out_unlock:
	p->unlock(p);
	return ret;
}

static void dbmdx_uevent_work(struct work_struct *work)
{
	struct dbmdx_private *p = container_of(
			work, struct dbmdx_private, uevent_work);
	int ret;

	if (!p)
		return;

	if (!p->device_ready) {
		dev_err(p->dev, "%s: device not ready\n", __func__);
		return;
	}

	ret = dbmdx_process_detection_irq(p, true);

	if (ret < 0)
		dev_err(p->dev,
			"%s: Error occurred during processing detection IRQ\n",
			__func__);

}

#define DBMDX_FW_AUDIO_BUFFER_MIN_SIZE	60

static void dbmdx_sv_work(struct work_struct *work)
{
	struct dbmdx_private *p = container_of(
			work, struct dbmdx_private, sv_work);
	int ret;
	int bytes_per_sample = p->bytes_per_sample;
	unsigned int bytes_to_read;
	u16 nr_samples, nr_samples_in_fw;
	size_t avail_samples;
	unsigned int total = 0;
	int kfifo_space = 0;
	int retries = 0;
	size_t data_offset;
	u16 cur_config = 0xffff;
	struct usecase_config *cur_usecase = NULL;

	dev_dbg(p->dev,
		"%s HW Channels %d, Channel operation: %d\n",
		__func__,
		p->pdata->va_audio_channels,
		p->detection_achannel_op);

	p->chip->transport_enable(p, true);

	if (p->va_debug_mode & DBMDX_DEBUG_MODE_FW_LOG)
		msleep(DBMDX_MSLEEP_DBG_AFTER_DETECTION);

	p->va_flags.irq_inuse = 0;

	/* Stop PCM streaming work */
	ret = dbmdx_suspend_pcm_streaming_work(p);
	if (ret < 0)
		dev_err(p->dev, "%s: Failed to suspend PCM Streaming Work\n",
			__func__);

	if (p->pdata->feature_va_ve) {
		cur_usecase = p->va_ve_flags.cur_usecase;
		if (cur_usecase == NULL) {
			dev_dbg(p->dev, "%s: no configured usecase, exit..\n",
				__func__);
			goto out;
		}
		if (!cur_usecase->usecase_supports_us_buffering) {
			dev_dbg(p->dev,
				"%s:Usecase doesn't support buffering, exit.\n",
				__func__);
			goto out;
		}
	}

	if (!p->va_capture_on_detect ||
		p->va_flags.sv_capture_on_detect_disabled) {
		dev_dbg(p->dev, "%s: no capture requested, exit..\n", __func__);
		goto out;
	}

	p->lock(p);

	/* flush fifo */
	kfifo_reset(&p->detection_samples_kfifo);

	if (p->mics_connected_to_va_ve_chip)
		ret = dbmdx_switch_to_va_ve_chip_interface(p,
							DBMDX_CMD_INTERFACE);
	else
		ret = dbmdx_switch_to_va_chip_interface(p, DBMDX_CMD_INTERFACE);
	/* prepare anything if needed (e.g. increase speed) */
	ret = p->chip->prepare_buffering(p);
	if (ret)
		goto out_fail_unlock;

	ret = dbmdx_set_pcm_streaming_mode(p, 0);

	if (ret < 0) {
		dev_err(p->dev, "%s: failed to set pcm streaming mode\n",
			__func__);
		goto out_fail_unlock;
	}

	/* read configuration */
	ret = dbmdx_send_cmd(p,
			     DBMDX_REGN_GENERAL_CONFIG_1,
			     &cur_config);
	if (ret < 0) {
		dev_err(p->dev,
			"%s: failed to read DBMDX_VA_GENERAL_CONFIG_1\n",
			__func__);
		goto out_fail_unlock;
	}

	/* Check whether buffering requires implicit trigger */
	if (cur_config & 0x80) {
		/* Reset position to backlog start */
		ret = dbmdx_send_cmd(p,
			(DBMDX_REGN_VT_AUDIO_HISTORY_CFG |
			p->va_cur_backlog_length |
			DBMDX_REGV_START_BUF_READ),
			NULL);

		if (ret < 0) {
			dev_err(p->dev,
				"%s: failed to reset backlog position\n",
				__func__);
			goto out_fail_unlock;
		}
	}

	p->unlock(p);

	p->va_flags.mode = DBMDX_BUFFERING;

	p->va_ve_flags.usecase_mode = DBMDX_BUFFERING;

	do {
		if (p->pdata->max_detection_buffer_size > 0 &&
			total >= p->pdata->max_detection_buffer_size) {
			dev_info(p->dev,
				"%s: buffering mode ended - reached requested max buffer size",
				__func__);
			break;
		}

		if (p->va_flags.buffering_paused) {
			msleep(DBMDX_MSLEEP_BUFFERING_PAUSED);
			dev_dbg(p->dev, "%s: buffering is paused...",
				__func__);
			continue;
		}


		p->lock(p);

		if (p->mics_connected_to_va_ve_chip)
			ret = dbmdx_switch_to_va_ve_chip_interface(p,
							DBMDX_CMD_INTERFACE);
		else
			ret = dbmdx_switch_to_va_chip_interface(p,
							DBMDX_CMD_INTERFACE);
		bytes_to_read = 0;
		/* read number of samples available in audio buffer */
		if (dbmdx_send_cmd(p,
				   DBMDX_REGN_NUM_OF_SMP_IN_BUF,
				   &nr_samples) < 0) {
			dev_err(p->dev,
				"%s: failed to read DBMDX_VA_NUM_OF_SMP_IN_BUF\n",
				__func__);
			p->unlock(p);
			goto out;
		}

		if (nr_samples == 0xffff) {
			dev_info(p->dev,
				"%s: buffering mode ended - fw in idle mode",
				__func__);
			p->unlock(p);
			break;
		}

#if DBMDX_FW_AUDIO_BUFFER_MIN_SIZE
		/* Do not go below the low border - can cause clicks */
		if (nr_samples < DBMDX_FW_AUDIO_BUFFER_MIN_SIZE)
			nr_samples = 0;
		else
			nr_samples -= DBMDX_FW_AUDIO_BUFFER_MIN_SIZE;
#endif

		nr_samples_in_fw = nr_samples;

		p->unlock(p);

		/* Now fill the kfifo. The user can access the data in
		 * parallel. The kfifo is safe for concurrent access of one
		 * reader (ALSA-capture/character device) and one writer (this
		 * work-queue).
		 */
		if (nr_samples) {
			bytes_to_read = nr_samples * 8 * bytes_per_sample;

			/* limit transaction size (no software flow control ) */
			if (bytes_to_read > p->rxsize && p->rxsize)
				bytes_to_read = p->rxsize;

			kfifo_space = kfifo_avail(&p->detection_samples_kfifo);

			if (p->detection_achannel_op ==
				AUDIO_CHANNEL_OP_DUPLICATE_1_TO_2)
				kfifo_space = kfifo_space / 2;
#ifdef DBMDX_4CHANNELS_SUPPORT
			else if (p->detection_achannel_op ==
				AUDIO_CHANNEL_OP_DUPLICATE_1_TO_4)
				kfifo_space = kfifo_space / 4;
			else if (p->detection_achannel_op ==
				AUDIO_CHANNEL_OP_DUPLICATE_2_TO_4)
				kfifo_space = kfifo_space / 2;
#endif
			if (bytes_to_read > kfifo_space)
				bytes_to_read = kfifo_space;

			/* recalculate number of samples */
			nr_samples = bytes_to_read / (8 * bytes_per_sample);

			if (!nr_samples) {
				/* not enough samples, wait some time */
				usleep_range(DBMDX_USLEEP_NO_SAMPLES,
					DBMDX_USLEEP_NO_SAMPLES + 1000);
				retries++;
				if (retries > p->pdata->buffering_timeout)
					break;
				continue;
			}
			/* get audio samples */
			p->lock(p);
			if (p->mics_connected_to_va_ve_chip)
				ret = dbmdx_switch_to_va_ve_chip_interface(p,
							DBMDX_CMD_INTERFACE);
			else
				ret = dbmdx_switch_to_va_chip_interface(p,
							DBMDX_CMD_INTERFACE);

			ret = p->chip->read_audio_data(p,
				p->read_audio_buf, nr_samples,
				false, &avail_samples, &data_offset);
			if (ret < 0) {
				dev_err(p->dev,
					"%s: failed to read block of audio data: %d\n",
					__func__, ret);
				p->unlock(p);
				break;
			} else if (ret > 0) {
				total += bytes_to_read;

				ret = dbmdx_add_audio_samples_to_kfifo(p,
					&p->detection_samples_kfifo,
					p->read_audio_buf + data_offset,
					bytes_to_read,
					p->detection_achannel_op);
			}

			retries = 0;

			p->unlock(p);

#if DBMDX_MSLEEP_IF_AUDIO_BUFFER_EMPTY
			if (nr_samples == nr_samples_in_fw)
				msleep(DBMDX_MSLEEP_IF_AUDIO_BUFFER_EMPTY);
#endif

		} else {
			usleep_range(DBMDX_USLEEP_NO_SAMPLES,
				DBMDX_USLEEP_NO_SAMPLES + 1000);
			retries++;
			if (retries > p->pdata->buffering_timeout)
				break;
		}

	} while (p->va_flags.buffering);

	dev_info(p->dev, "%s: audio buffer read, total of %u bytes\n",
		__func__, total);


out:
	p->lock(p);
	p->va_flags.buffering = 0;
	/* finish audio buffering (e.g. reduce speed) */
	ret = p->chip->finish_buffering(p);
	if (ret) {
		dev_err(p->dev, "%s: failed to finish buffering\n", __func__);
		goto out_fail_unlock;
	}

	if (p->pdata->send_uevent_after_buffering) {
		char uevent_buf[100];
		char * const envp[] = { uevent_buf, NULL };

		snprintf(uevent_buf, sizeof(uevent_buf),
			"VOICE_WAKEUP EVENT_TYPE=BufferingDone");

		dev_info(p->dev, "%s: Sending uevent: %s\n",
			__func__, uevent_buf);

		ret = kobject_uevent_env(&p->dev->kobj, KOBJ_CHANGE,
			(char **)envp);

		if (ret)
			dev_err(p->dev,
					"%s: Sending uevent failed %d\n",
						__func__, ret);
	}

	if (p->pdata->detection_after_buffering !=
					DETECTION_AFTER_BUFFERING_OFF) {
		cur_usecase = p->va_ve_flags.cur_usecase;
		if ((cur_usecase != NULL) &&
			cur_usecase->usecase_sets_detection_mode) {
			ret = dbmdx_set_usecase_mode(p,
						DBMDX_DETECTION);
			if (ret) {
				dev_err(p->dev,
				"%s: failed to set detection usecase mode\n",
						__func__);
				goto out_fail_unlock;
			}
		}
	} else if (p->va_flags.pcm_streaming_active) {
#ifdef RR_TODO
		ret = dbmdx_set_mode(p, DBMDX_STREAMING);

		if (ret < 0) {
			dev_err(p->dev,
				"%s: failed to set DBMDX_STREAMING mode\n",
					__func__);
			goto out_fail_unlock;
		}
#endif
	}

out_fail_unlock:
	p->unlock(p);
	dev_dbg(p->dev, "%s done\n", __func__);
}

#if defined(CONFIG_SND_SOC_DBMDX_SND_CAPTURE)

static void dbmdx_pcm_streaming_work_mod_0(struct work_struct *work)
{
	struct dbmdx_private *p = container_of(
			work, struct dbmdx_private, pcm_streaming_work);
	int ret;
	int bytes_per_sample = p->bytes_per_sample;
	unsigned int bytes_to_read;
	u16 nr_samples;
	size_t avail_samples;
	unsigned int total = 0;
	int kfifo_space = 0;
	int retries = 0;
	struct snd_pcm_substream *substream = p->active_substream;
	struct snd_soc_pcm_runtime *rtd;
#ifdef SND_SOC_COMPONENT
	struct snd_soc_component *component;
#else
	struct snd_soc_codec *codec;
#endif
	struct snd_pcm_runtime *runtime;
	unsigned long frame_in_bytes;
	u32 samples_chunk_size;
	bool direct_copy_enabled = false;
	u8 *local_samples_buf;
	u8 *read_samples_buf;
	unsigned int stream_buf_size;
	u32 pos;
	u32 missing_samples;
	u32 sleep_time_ms;
	size_t data_offset;
	u16 cur_config = 0xffff;

	if (substream == NULL) {
		dev_err(p->dev,	"%s Error No Active Substream\n", __func__);
		return;
	}

	rtd = substream->private_data;
#ifdef SND_SOC_COMPONENT
	component = rtd->codec_dai->component;
#else
	codec = rtd->codec_dai->codec;
#endif

	dev_dbg(p->dev,
		"%s PCM Channels %d, HW Channels %d, Channel operation: %d\n",
		__func__,
		p->audio_pcm_channels,
		p->pdata->va_audio_channels,
		p->pcm_achannel_op);

	ret = dbmdx_set_pcm_timer_mode(substream, false);

	if (ret < 0)
		dev_err(p->dev,	"%s Failed to stop timer mode\n", __func__);

	p->va_flags.pcm_streaming_pushing_zeroes = false;

	runtime = substream->runtime;
	frame_in_bytes = frames_to_bytes(runtime, runtime->period_size);
	stream_buf_size = snd_pcm_lib_buffer_bytes(substream);

	samples_chunk_size =
		(u32)(frame_in_bytes * p->pdata->va_audio_channels) /
		(8 * bytes_per_sample * runtime->channels);

	if (((u32)(frame_in_bytes * p->pdata->va_audio_channels) %
		(8 * bytes_per_sample * runtime->channels)))
		samples_chunk_size++;
	else if (p->pcm_achannel_op == AUDIO_CHANNEL_OP_COPY)
		direct_copy_enabled = true;

	bytes_to_read = samples_chunk_size * 8 * bytes_per_sample;

	dev_dbg(p->dev,
		"%s Frame Size: %d, Dir. Copy Mode: %d, Samples TX Thr.: %d\n",
		__func__,
		(int)frame_in_bytes,
		(int)direct_copy_enabled,
		(int)samples_chunk_size);

	local_samples_buf = kmalloc(bytes_to_read + 8, GFP_KERNEL);
	if (!local_samples_buf)
		return;

	read_samples_buf = local_samples_buf;

	p->lock(p);

	/* flush fifo */
	kfifo_reset(&p->pcm_kfifo);

	/* prepare anything if needed (e.g. increase speed) */
	ret = p->chip->prepare_buffering(p);
	if (ret)
		goto out_fail_unlock;

	/* Delay streaming start to stabilize the PLL and microphones */
	msleep(DBMDX_MSLEEP_PCM_STREAMING_WORK);

	ret = dbmdx_set_pcm_streaming_mode(p, 0);

	if (ret < 0) {
		dev_err(p->dev, "%s: failed to set pcm streaming mode\n",
			__func__);
		goto out_fail_unlock;
	}

	/* read configuration */
	ret = dbmdx_send_cmd(p,
			     DBMDX_REGN_GENERAL_CONFIG_1,
			     &cur_config);
	if (ret < 0) {
		dev_err(p->dev,
			"%s: failed to read DBMDX_VA_GENERAL_CONFIG_1\n",
			__func__);
		goto out_fail_unlock;
	}

	/* Check whether buffering requires implicit trigger */
	if (cur_config & 0x80) {
		/* Reset position to backlog start */
		ret = dbmdx_send_cmd(p,
			DBMDX_REGN_VT_AUDIO_HISTORY_CFG |
			DBMDX_REGV_START_BUF_READ,
			NULL);

		if (ret < 0) {
			dev_err(p->dev,
				"%s: failed to reset backlog position\n",
				__func__);
			goto out_fail_unlock;
		}
	}

	p->unlock(p);

	do {
		p->lock(p);

		if (!(p->va_flags.pcm_worker_active)) {
			p->unlock(p);
			break;
		}

		/* read number of samples available in audio buffer */
		if (dbmdx_send_cmd(p,
				   DBMDX_REGN_NUM_OF_SMP_IN_BUF,
				   &nr_samples) < 0) {
			dev_err(p->dev,
				"%s: failed to read DBMDX_VA_NUM_OF_SMP_IN_BUF\n",
				__func__);
			p->unlock(p);
			goto out;
		}

		if (nr_samples == 0xffff) {
			dev_info(p->dev,
				"%s: buffering mode ended - fw in idle mode",
				__func__);
			p->unlock(p);
			break;
		}

		p->unlock(p);

		/* Now fill the kfifo. The user can access the data in
		 * parallel. The kfifo is safe for concurrent access of one
		 * reader (ALSA-capture/character device) and one writer (this
		 * work-queue).
		 */
		if (nr_samples >= samples_chunk_size) {

			if (nr_samples >= 2*samples_chunk_size)
				sleep_time_ms = 0;
			else {
				missing_samples = 2*samples_chunk_size -
						nr_samples;
				sleep_time_ms =
					(u32)(missing_samples * 8 * 1000) /
					p->current_pcm_rate;
			}

			nr_samples = samples_chunk_size;

			if (!direct_copy_enabled) {
				kfifo_space = kfifo_avail(&p->pcm_kfifo);

				if (p->pcm_achannel_op ==
					AUDIO_CHANNEL_OP_DUPLICATE_1_TO_2)
					kfifo_space = kfifo_space / 2;
#ifdef DBMDX_4CHANNELS_SUPPORT
				else if (p->pcm_achannel_op ==
					AUDIO_CHANNEL_OP_DUPLICATE_1_TO_4)
					kfifo_space = kfifo_space / 4;
				else if (p->pcm_achannel_op ==
					AUDIO_CHANNEL_OP_DUPLICATE_2_TO_4)
					kfifo_space = kfifo_space / 2;
#endif
				if (bytes_to_read > kfifo_space)
					nr_samples = 0;
			}

			if (!nr_samples) {
				/* not enough samples, wait some time */
				usleep_range(DBMDX_USLEEP_NO_SAMPLES,
					DBMDX_USLEEP_NO_SAMPLES + 1000);
				retries++;
				if (retries > p->pdata->buffering_timeout)
					break;
				continue;
			}
			/* get audio samples */
			p->lock(p);

			ret = p->chip->read_audio_data(p,
				local_samples_buf, nr_samples,
				false, &avail_samples, &data_offset);

			if (ret < 0) {
				dev_err(p->dev,
					"%s: failed to read block of audio data: %d\n",
					__func__, ret);
				p->unlock(p);
				break;
			} else if (ret > 0) {
				total += bytes_to_read;

				pos = stream_get_position(substream);
				read_samples_buf = runtime->dma_area + pos;

				if (direct_copy_enabled)
					memcpy(read_samples_buf,
						local_samples_buf + data_offset,
						bytes_to_read);
				else {

					ret =
					dbmdx_add_audio_samples_to_kfifo(p,
						&p->pcm_kfifo,
						local_samples_buf + data_offset,
						bytes_to_read,
						p->pcm_achannel_op);

#ifdef SND_SOC_COMPONENT
					ret =
					dbmdx_get_samples(component,
						read_samples_buf,
						runtime->channels *
						runtime->period_size);
#else
					ret =
					dbmdx_get_samples(codec,
						read_samples_buf,
						runtime->channels *
						runtime->period_size);

#endif
					if (ret) {
						memset(read_samples_buf,
							0,
							frame_in_bytes);
						pr_debug("%s Inserting %d bytes of silence\n",
						__func__, (int)bytes_to_read);
					}
				}

				pos += frame_in_bytes;
				if (pos >= stream_buf_size)
					pos = 0;

				stream_set_position(substream, pos);

				snd_pcm_period_elapsed(substream);
			}

			retries = 0;

			p->unlock(p);

			if (sleep_time_ms > 0)
				msleep(sleep_time_ms);

		} else {
			u32 missing_samples = samples_chunk_size - nr_samples;
			u32 sleep_time_ms = (u32)(missing_samples * 8 * 1100) /
					p->current_pcm_rate;

			msleep(sleep_time_ms);

			retries++;
			if (retries > p->pdata->buffering_timeout)
				break;
		}

	} while (p->va_flags.pcm_worker_active);

	dev_info(p->dev, "%s: audio buffer read, total of %u bytes\n",
		__func__, total);

out:
	p->lock(p);
	p->va_flags.pcm_worker_active = 0;
	/* finish audio buffering (e.g. reduce speed) */
	ret = p->chip->finish_buffering(p);
	if (ret) {
		dev_err(p->dev, "%s: failed to finish buffering\n", __func__);
		goto out_fail_unlock;
	}
out_fail_unlock:
	kfree(local_samples_buf);

	p->unlock(p);
	dev_dbg(p->dev, "%s done\n", __func__);
}


#define DBMDX_EXTENDED_STREAMIN_DBG_INFO	0
#define DBMDX_MIN_SAMPLES_IN_FW			64

static void dbmdx_pcm_streaming_work_mod_1(struct work_struct *work)
{
	struct dbmdx_private *p = container_of(
			work, struct dbmdx_private, pcm_streaming_work);
	int ret;
	int bytes_per_sample = p->bytes_per_sample;
	unsigned int bytes_to_read;
	size_t avail_samples;
	unsigned int total = 0;
	int bytes_in_kfifo;
	int retries = 0;
	struct snd_pcm_substream *substream = p->active_substream;
	struct snd_soc_pcm_runtime *rtd;
#ifdef SND_SOC_COMPONENT
	struct snd_soc_component *component;
#else
	struct snd_soc_codec *codec;
#endif
	struct snd_pcm_runtime *runtime;
	unsigned long frame_in_bytes;
	u32 samples_chunk_size;
	bool direct_copy_enabled = false;
	u8 *local_samples_buf;
	u8 *read_samples_buf;
	unsigned int stream_buf_size;
	u32 pos;
	u32 missing_samples;
	u32 sleep_time_ms;
	size_t data_offset;

	if (substream == NULL) {
		dev_err(p->dev,	"%s Error No Active Substream\n", __func__);
		return;
	}

	rtd = substream->private_data;
#ifdef SND_SOC_COMPONENT
	component = rtd->codec_dai->component;
#else
	codec = rtd->codec_dai->codec;
#endif

	dev_dbg(p->dev,
		"%s PCM Channels %d, HW Channels %d, Channel operation: %d\n",
		__func__,
		p->audio_pcm_channels,
		p->pdata->va_audio_channels,
		p->pcm_achannel_op);

	ret = dbmdx_set_pcm_timer_mode(substream, false);

	if (ret < 0)
		dev_err(p->dev,	"%s Failed to stop timer mode\n", __func__);

	p->va_flags.pcm_streaming_pushing_zeroes = false;

	runtime = substream->runtime;
	frame_in_bytes = frames_to_bytes(runtime, runtime->period_size);
	stream_buf_size = snd_pcm_lib_buffer_bytes(substream);

	samples_chunk_size =
		(u32)(frame_in_bytes * p->pdata->va_audio_channels) /
		(8 * bytes_per_sample * runtime->channels);

	if (((u32)(frame_in_bytes * p->pdata->va_audio_channels) %
		(8 * bytes_per_sample * runtime->channels)))
		samples_chunk_size++;
	else if (p->pcm_achannel_op == AUDIO_CHANNEL_OP_COPY)
		direct_copy_enabled = true;

	bytes_to_read = samples_chunk_size * 8 * bytes_per_sample;

	if (p->pdata->va_audio_channels > 1 ||  runtime->channels >  1)
		p->va_flags.padded_cmds_disabled = true;

	dev_dbg(p->dev,
		"%s Frame Size: %d, Dir. Copy Mode: %d, Samples TX Thr.: %d\n",
		__func__,
		(int)frame_in_bytes,
		(int)direct_copy_enabled,
		(int)samples_chunk_size);

	/* Ensure that there enough space for metadata , add 8 bytes */
	local_samples_buf = kmalloc(bytes_to_read + 8, GFP_KERNEL);
	if (!local_samples_buf)
		return;

	read_samples_buf = local_samples_buf;

	p->lock(p);

	/* flush fifo */
	kfifo_reset(&p->pcm_kfifo);

	/* prepare anything if needed (e.g. increase speed) */
	ret = p->chip->prepare_buffering(p);
	if (ret)
		goto out_fail_unlock;

	/* Delay streaming start to stabilize the PLL and microphones */
	msleep(DBMDX_MSLEEP_PCM_STREAMING_WORK);

	ret = dbmdx_set_pcm_streaming_mode(p, 1);

	if (ret < 0) {
		dev_err(p->dev, "%s: failed to set pcm streaming mode\n",
			__func__);
		goto out_fail_unlock;
	}

	/* Start PCM streaming, FW should be in detection mode */
	ret = dbmdx_send_cmd(p,
			(DBMDX_REGN_VT_AUDIO_HISTORY_CFG |
			 DBMDX_REGV_START_BUF_READ), NULL);

	if (ret < 0) {
		dev_err(p->dev, "%s: failed start pcm streaming\n", __func__);
		goto out_fail_unlock;
	}

	p->unlock(p);

	/* Ensure that we will sleep till the first chunk is available */
	avail_samples = 0;

	do {
		bytes_in_kfifo = kfifo_len(&p->pcm_kfifo);

		if (bytes_in_kfifo >= frame_in_bytes)	{

#if DBMDX_EXTENDED_STREAMIN_DBG_INFO
			dev_dbg(p->dev,
				"%s Bytes in KFIFO (%d) >= Frame Size(%d)\n",
				__func__,
				(int)bytes_in_kfifo,
				(int)frame_in_bytes);
#endif
			pos = stream_get_position(substream);

			read_samples_buf = runtime->dma_area + pos;

#ifdef SND_SOC_COMPONENT
			ret = dbmdx_get_samples(component, read_samples_buf,
						runtime->channels *
						runtime->period_size);
#else
			ret = dbmdx_get_samples(codec, read_samples_buf,
						runtime->channels *
						runtime->period_size);
#endif
			pos += frame_in_bytes;
			if (pos >= stream_buf_size)
				pos = 0;

			stream_set_position(substream, pos);

			snd_pcm_period_elapsed(substream);

			continue;
		}

		samples_chunk_size =
			(u32)((frame_in_bytes - bytes_in_kfifo) *
				p->pdata->va_audio_channels) /
			(8 * bytes_per_sample * runtime->channels);

		if (((u32)((frame_in_bytes - bytes_in_kfifo) *
			p->pdata->va_audio_channels) %
			(8 * bytes_per_sample * runtime->channels)))
			samples_chunk_size++;

		bytes_to_read = samples_chunk_size * 8 * bytes_per_sample;

		if (avail_samples >= samples_chunk_size +
						DBMDX_MIN_SAMPLES_IN_FW)
			sleep_time_ms = 0;
		else {
			missing_samples = samples_chunk_size +
						DBMDX_MIN_SAMPLES_IN_FW -
						avail_samples;

			sleep_time_ms =
				(u32)(missing_samples * 8 * 1000) /
				p->current_pcm_rate;
		}

#if DBMDX_EXTENDED_STREAMIN_DBG_INFO
		dev_dbg(p->dev,
			"%s Frame Size(%d), Kfifo Bytes (%d), Samples Chunk (%d), Bytes to Read (%d), Avail. Samples (%d), Sleep Time (%d)ms\n",
			__func__,
			(int)frame_in_bytes,
			(int)bytes_in_kfifo,
			(int)samples_chunk_size,
			(int)bytes_to_read,
			(int)avail_samples,
			(int)sleep_time_ms);
#endif

		if (sleep_time_ms > 0)
			msleep(sleep_time_ms);

		/* get audio samples */
		p->lock(p);

		if (!(p->va_flags.pcm_worker_active)) {
			p->unlock(p);
			break;
		}

		ret = p->chip->read_audio_data(p,
				local_samples_buf, samples_chunk_size,
				true, &avail_samples, &data_offset);

		p->unlock(p);

		if (ret < 0) {
			dev_err(p->dev,
				"%s: failed to read block of audio data: %d\n",
				__func__, ret);
			break;
		}

		if (avail_samples == 0xffff) {
			dev_info(p->dev,
				"%s: buffering mode ended - fw in idle mode",
				__func__);
			break;
		} else if (avail_samples >= samples_chunk_size) {

			/* If all requested samples were read */

#if DBMDX_EXTENDED_STREAMIN_DBG_INFO
			dev_dbg(p->dev,
				"%s Chunk was read (big enough): Avail. Samples (%d),Samples Chunk (%d), Kfifo Bytes (%d)\n",
				__func__,
				(int)avail_samples,
				(int)samples_chunk_size,
				(int)bytes_in_kfifo);
#endif
			total += bytes_to_read;

			pos = stream_get_position(substream);
			read_samples_buf = runtime->dma_area + pos;

			if (direct_copy_enabled && !bytes_in_kfifo)
				memcpy(read_samples_buf,
					local_samples_buf + data_offset,
					bytes_to_read);
			else {
				ret = dbmdx_add_audio_samples_to_kfifo(p,
						&p->pcm_kfifo,
						local_samples_buf + data_offset,
						bytes_to_read,
						p->pcm_achannel_op);

#ifdef SND_SOC_COMPONENT
				ret = dbmdx_get_samples(component, read_samples_buf,		
						runtime->channels * 					
						runtime->period_size);
#else
				ret = dbmdx_get_samples(codec, read_samples_buf,
						runtime->channels *
						runtime->period_size);
#endif
				if (ret) {
					memset(read_samples_buf,
						0,
						frame_in_bytes);
					pr_debug("%s Inserting %d bytes of silence\n",
					__func__, (int)bytes_to_read);
				}
			}

			pos += frame_in_bytes;
			if (pos >= stream_buf_size)
				pos = 0;

			stream_set_position(substream, pos);

			snd_pcm_period_elapsed(substream);

			avail_samples -= samples_chunk_size;

			retries = 0;

		} else if (avail_samples > 0) {
			u32 bytes_read;
#if DBMDX_EXTENDED_STREAMIN_DBG_INFO
			dev_dbg(p->dev,
				"%s Chunk was read (not big enough): Avail. Samples (%d),Samples Chunk (%d), Kfifo Bytes (%d)\n",
				__func__,
				(int)avail_samples,
				(int)samples_chunk_size,
				(int)bytes_in_kfifo);
#endif

			bytes_read =
				(u32)(avail_samples * 8 * bytes_per_sample);

			total += bytes_read;

			ret = dbmdx_add_audio_samples_to_kfifo(p,
						&p->pcm_kfifo,
						local_samples_buf + data_offset,
						bytes_read,
						p->pcm_achannel_op);
			avail_samples = 0;

			retries = 0;

		} else {

#if DBMDX_EXTENDED_STREAMIN_DBG_INFO
			dev_dbg(p->dev,
			"%s Chunk was read (no samples): Avail. Samples (%d),Samples Chunk (%d), Kfifo Bytes (%d), Retries (%d)\n",
				__func__,
				(int)avail_samples,
				(int)samples_chunk_size,
				(int)bytes_in_kfifo,
				(int)retries+1);
#endif

			retries++;
			if (retries > p->pdata->buffering_timeout)
				break;
		}

	} while (p->va_flags.pcm_worker_active);

	dev_info(p->dev, "%s: audio buffer read, total of %u bytes\n",
		__func__, total);

	p->lock(p);
	p->va_flags.pcm_worker_active = 0;
	/* finish audio buffering (e.g. reduce speed) */
	ret = p->chip->finish_buffering(p);
	if (ret) {
		dev_err(p->dev, "%s: failed to finish buffering\n", __func__);
		goto out_fail_unlock;
	}
out_fail_unlock:
	kfree(local_samples_buf);
	p->va_flags.padded_cmds_disabled = false;
	p->unlock(p);
	dev_dbg(p->dev, "%s done\n", __func__);
}
#endif

static irqreturn_t dbmdx_sv_interrupt_hard(int irq, void *dev)
{
	struct dbmdx_private *p = (struct dbmdx_private *)dev;
	if (p && (p->device_ready) && (p->va_flags.irq_inuse))
		return IRQ_WAKE_THREAD;

	return IRQ_HANDLED;
}


static irqreturn_t dbmdx_sv_interrupt_soft(int irq, void *dev)
{
	struct dbmdx_private *p = (struct dbmdx_private *)dev;

	dev_dbg(p->dev, "%s\n", __func__);
	if ((p->device_ready) && (p->va_flags.irq_inuse)) {

#ifdef DBMDX_PROCESS_DETECTION_IRQ_WITHOUT_WORKER
		dbmdx_process_detection_irq(p, true);
#else
		dbmdx_schedule_work(p, &p->uevent_work);
#endif

		dev_info(p->dev, "%s - SV EVENT\n", __func__);
	}
	return IRQ_HANDLED;
}

static int dbmdx_fw_load_thread(void *data)
{
	struct dbmdx_private *p = (struct dbmdx_private *)data;

	if (p->pdata->feature_va)
		return dbmdx_request_and_load_fw(p, 1);
	else if (p->pdata->feature_va_ve)
		return dbmdx_request_and_load_fw_va_ve_mode(p);

	return -EINVAL;
}

static int dbmdx_get_va_resources(struct dbmdx_private *p)
{
	int ret;

	dev_dbg(p->dev, "%s\n", __func__);

	atomic_set(&p->audio_owner, 0);

#if defined(CONFIG_SND_SOC_DBMDX_SND_CAPTURE)
	if (p->pdata->pcm_streaming_mode == 0)
		INIT_WORK(&p->pcm_streaming_work,
			dbmdx_pcm_streaming_work_mod_0);
	else
		INIT_WORK(&p->pcm_streaming_work,
			dbmdx_pcm_streaming_work_mod_1);

#endif

	INIT_WORK(&p->sv_work, dbmdx_sv_work);
	INIT_WORK(&p->uevent_work, dbmdx_uevent_work);

	ret = kfifo_alloc(&p->pcm_kfifo, MAX_KFIFO_BUFFER_SIZE_MONO,
		GFP_KERNEL);

	if (ret)  {
		dev_err(p->dev, "%s: no kfifo memory\n", __func__);
		ret = -ENOMEM;
		goto err;
	}

	p->detection_samples_kfifo_buf_size = MAX_KFIFO_BUFFER_SIZE;

	p->detection_samples_kfifo_buf =
	kmalloc(MAX_KFIFO_BUFFER_SIZE, GFP_KERNEL);

	if (!p->detection_samples_kfifo_buf) {
		ret = -ENOMEM;
		goto err_kfifo_pcm_free;
	}

	kfifo_init(&p->detection_samples_kfifo,
		p->detection_samples_kfifo_buf, MAX_KFIFO_BUFFER_SIZE);


	/* Switch ON capture backlog by default */
	p->va_capture_on_detect = true;

	/* default mode is PCM mode */
	p->audio_mode = DBMDX_AUDIO_MODE_PCM;
	p->audio_pcm_rate = 16000;
	p->bytes_per_sample = 2;

	p->va_flags.irq_inuse = 0;
	if (p->sv_irq != -1) {
		ret = request_threaded_irq(p->sv_irq,
					  dbmdx_sv_interrupt_hard,
					  dbmdx_sv_interrupt_soft,
					  IRQF_TRIGGER_RISING,
					  "dbmdx_sv", p);

		if (ret < 0) {
			dev_err(p->dev, "%s: cannot get irq\n", __func__);
			goto err_kfifo_free;
		}

		ret = irq_set_irq_wake(p->sv_irq, 1);
		if (ret < 0)
			dev_err(p->dev, "%s: cannot set irq_set_irq_wake\n",
				__func__);
	}

	ret = sysfs_create_group(&p->dbmdx_dev->kobj,
				 &dbmdx_va_attribute_group);
	if (ret) {
		dev_err(p->dbmdx_dev, "%s: failed to create VA sysfs group\n",
			__func__);
		goto err_free_irq;
	}

	return 0;
err_free_irq:
	p->va_flags.irq_inuse = 0;
	irq_set_irq_wake(p->sv_irq, 0);
	free_irq(p->sv_irq, p);
err_kfifo_free:
	kfifo_free(&p->detection_samples_kfifo);
err_kfifo_pcm_free:
	kfifo_free(&p->pcm_kfifo);
err:
	return ret;
}

static void dbmdx_free_va_resources(struct dbmdx_private *p)
{
	dev_dbg(p->dev, "%s\n", __func__);
	free_vte(p, &(p->vt_engines[0]));
	free_vte(p, &(p->vt_engines[1]));
	free_vte(p, &(p->vt_engines[2]));
	kfifo_free(&p->pcm_kfifo);
	kfifo_free(&p->detection_samples_kfifo);
	p->va_flags.irq_inuse = 0;
	if (p->sv_irq != -1) {
		irq_set_irq_wake(p->sv_irq, 0);
		free_irq(p->sv_irq, p);
	}
#ifdef CONFIG_OF
	if (p->pdata->va_cfg_values)
		kfree(p->pdata->va_cfg_value);
#endif

	sysfs_remove_group(&p->dev->kobj, &dbmdx_va_attribute_group);
}


static int dbmdx_get_va_ve_chip_va_resources(struct dbmdx_private *p)
{
	int ret;

	dev_dbg(p->dev, "%s\n", __func__);

	atomic_set(&p->audio_owner, 0);

#if defined(CONFIG_SND_SOC_DBMDX_SND_CAPTURE)
	if (p->pdata->pcm_streaming_mode == 0)
		INIT_WORK(&p->pcm_streaming_work,
			dbmdx_pcm_streaming_work_mod_0);
	else
		INIT_WORK(&p->pcm_streaming_work,
			dbmdx_pcm_streaming_work_mod_1);

#endif

	INIT_WORK(&p->sv_work, dbmdx_sv_work);
	INIT_WORK(&p->uevent_work, dbmdx_uevent_work);

	ret = kfifo_alloc(&p->pcm_kfifo, MAX_KFIFO_BUFFER_SIZE_MONO,
		GFP_KERNEL);

	if (ret)  {
		dev_err(p->dev, "%s: no kfifo memory\n", __func__);
		ret = -ENOMEM;
		goto err;
	}

	p->detection_samples_kfifo_buf_size = MAX_KFIFO_BUFFER_SIZE;

	p->detection_samples_kfifo_buf =
	kmalloc(MAX_KFIFO_BUFFER_SIZE, GFP_KERNEL);

	if (!p->detection_samples_kfifo_buf) {
		dev_err(p->dev, "%s: no kfifo memory\n", __func__);
		ret = -ENOMEM;
		goto err_kfifo_pcm_free;
	}

	kfifo_init(&p->detection_samples_kfifo,
		p->detection_samples_kfifo_buf, MAX_KFIFO_BUFFER_SIZE);

	/* Switch ON capture backlog by default */
	p->va_capture_on_detect = true;

	/* default mode is PCM mode */
	p->audio_mode = DBMDX_AUDIO_MODE_PCM;
	p->audio_pcm_rate = 16000;
	p->bytes_per_sample = 2;

	p->va_flags.irq_inuse = 0;
	if (p->sv_irq != -1) {
		ret = request_threaded_irq(p->sv_irq,
					  dbmdx_sv_interrupt_hard,
					  dbmdx_sv_interrupt_soft,
					  IRQF_TRIGGER_RISING,
					  "dbmdx_sv", p);

		if (ret < 0) {
			dev_err(p->dev, "%s: cannot get irq\n", __func__);
			goto err_kfifo_free;
		}
		ret = irq_set_irq_wake(p->sv_irq, 1);
		if (ret < 0)
			dev_err(p->dev, "%s: cannot set irq_set_irq_wake\n",
				__func__);
	}

	ret = sysfs_create_group(&p->dbmdx_dev->kobj,
				 &dbmdx_va_ve_chip_va_attribute_group);
	if (ret) {
		dev_err(p->dbmdx_dev, "%s: failed to create VA sysfs group\n",
			__func__);
		goto err_free_irq;
	}

	return 0;
err_free_irq:
	p->va_flags.irq_inuse = 0;
	irq_set_irq_wake(p->sv_irq, 0);
	free_irq(p->sv_irq, p);
err_kfifo_free:
	kfifo_free(&p->detection_samples_kfifo);
err_kfifo_pcm_free:
	kfifo_free(&p->pcm_kfifo);
err:
	return ret;
}

static int dbmdx_get_va_ve_common_resources(struct dbmdx_private *p)
{
	int ret;

	dev_dbg(p->dev, "%s\n", __func__);


	ret = sysfs_create_group(&p->dbmdx_dev->kobj,
				 &dbmdx_va_ve_common_attribute_group);
	if (ret) {
		dev_err(p->dbmdx_dev,
			"%s: failed to create VA_VE common sysfs group\n",
			__func__);
		return ret;
	}

	return 0;
}

static int dbmdx_get_va_ve_chip_va_ve_resources(struct dbmdx_private *p)
{
	int ret;

	dev_dbg(p->dev, "%s\n", __func__);


	ret = sysfs_create_group(&p->dbmdx_dev->kobj,
				 &dbmdx_va_ve_chip_va_ve_attribute_group);
	if (ret) {
		dev_err(p->dbmdx_dev,
			"%s: failed to create VA_VE chip va_ve sysfs group\n",
			__func__);
		return ret;
	}

	return 0;
}

static void dbmdx_free_va_ve_common_resources(struct dbmdx_private *p)
{
	dev_dbg(p->dev, "%s\n", __func__);

	if (p->external_usecase_loaded)
		free_external_usecase_memory(&config_usecase_external);

	sysfs_remove_group(&p->dev->kobj, &dbmdx_va_ve_common_attribute_group);
}

static void dbmdx_free_va_ve_chip_va_resources(struct dbmdx_private *p)
{
	dev_dbg(p->dev, "%s\n", __func__);

	free_vte(p, &(p->vt_engines[0]));
	free_vte(p, &(p->vt_engines[1]));
	free_vte(p, &(p->vt_engines[2]));
	kfifo_free(&p->pcm_kfifo);
	kfifo_free(&p->detection_samples_kfifo);
	p->va_flags.irq_inuse = 0;
	if (p->sv_irq != -1) {
		irq_set_irq_wake(p->sv_irq, 0);
		free_irq(p->sv_irq, p);
	}

	sysfs_remove_group(&p->dev->kobj, &dbmdx_va_ve_chip_va_attribute_group);
}

static void dbmdx_free_va_ve_chip_va_ve_resources(struct dbmdx_private *p)
{
	dev_dbg(p->dev, "%s\n", __func__);

#ifdef CONFIG_OF
	if (p->pdata->va_ve_cfg_values)
		kfree(p->pdata->va_ve_cfg_value);
#endif

	sysfs_remove_group(&p->dev->kobj,
		&dbmdx_va_ve_chip_va_ve_attribute_group);
}

static int dbmdx_common_probe(struct dbmdx_private *p)
{
#ifdef CONFIG_OF
	struct device_node *np = p->dev->of_node;
#endif
	struct task_struct *boot_thread;
	int ret = 0;
	int fclk;

	dbmdx_data = p;
	dev_set_drvdata(p->dev, p);

#ifdef CONFIG_OF
	if (!np) {
		dev_err(p->dev, "%s: error no devicetree entry\n", __func__);
		ret = -ENODEV;
		goto err;
	}
#endif

	/* enable constant clock */
	p->clk_enable(p, DBMDX_CLK_CONSTANT);

	/* enable regulator if defined */
	if (p->vregulator) {
		ret = regulator_enable(p->vregulator);
		if (ret != 0) {
			dev_err(p->dev, "%s: Failed to enable regulator: %d\n",
				__func__, ret);
			goto err_clk_off;
		}
	}

	/* lock init */
	mutex_init(&p->p_lock);
	mutex_init(&p->p_usr_op_lock);

	/* set clock rates */
	if (p->clk_set_rate(p, DBMDX_CLK_MASTER) < 0) {
		dev_err(p->dev,
			"%s: could not set rate for master clock\n",
			__func__);
		goto err_clk_off;
	}

	if (p->pdata->va_ve_separate_clocks) {
		if (p->clk_set_rate(p, DBMDX_CLK_MASTER_VA_VE)) {
			dev_err(p->dev,
			"%s: could not set rate for master_va_ve clock\n",
			__func__);
			goto err_clk_off;
		}
	}

	if (p->clk_set_rate(p, DBMDX_CLK_CONSTANT) < 0) {
		dev_err(p->dev,
			"%s: could not set rate for constant clock\n",
			__func__);
		goto err_clk_off;
	}
	fclk = p->clk_get_rate(p, DBMDX_CLK_MASTER);

	p->master_pll_rate =  (unsigned int)((fclk / 6) * 17);

	switch (fclk) {
	case 32768:
		p->clk_type = DBMD2_XTAL_FREQ_32K_IMG7;
		p->master_pll_rate = 60000000;
		break;
	case 9600000:
		p->clk_type = DBMD2_XTAL_FREQ_9M_IMG4;
		break;
	case 24000000:
	default:
		p->clk_type = DBMD2_XTAL_FREQ_24M_IMG1;
	}

	p->ns_class = class_create(THIS_MODULE, "voicep");
	if (IS_ERR(p->ns_class)) {
		dev_err(p->dev, "%s: failed to create class\n", __func__);
		goto err_clk_off;
	}

	p->dbmdx_dev = device_create(p->ns_class, NULL, 0, p, "dbmdx");
	if (IS_ERR(p->dbmdx_dev)) {
		dev_err(p->dev, "%s: could not create device\n", __func__);
		goto err_class_destroy;
	}

	ret = sysfs_create_group(&p->dbmdx_dev->kobj,
				 &dbmdx_common_attribute_group);
	if (ret) {
		dev_err(p->dbmdx_dev, "%s: failed to create sysfs group\n",
			__func__);
		goto err_device_unregister;
	}

	if (p->pdata->feature_va) {
		ret = dbmdx_va_chip_probe(p);
		if (ret) {
			dev_err(p->dev,
				"%s: could not probe va chip\n", __func__);
			goto err_sysfs_remove_group;
		}
		ret = dbmdx_get_va_resources(p);
		if (ret) {
			dev_err(p->dev,
				"%s: could not get VA resources\n", __func__);
			goto err_chip_remove;
		}
	}
	else if (p->pdata->feature_va_ve) {
		if (p->va_chip_enabled) {
			ret = dbmdx_va_chip_probe(p);
			if (ret) {
				dev_err(p->dev,
					"%s: could not probe va chip\n",
					__func__);
				goto err_sysfs_remove_group;
			}
		}
		if (p->va_ve_chip_enabled) {
			ret = dbmdx_va_ve_chip_probe(p);
			if (ret) {
				dev_err(p->dev,
					"%s: could not probe va_ve chip\n",
					__func__);
				if (p->va_chip_enabled)
					dbmdx_va_chip_remove(p);
				goto err_sysfs_remove_group;
			}
		}

		ret = dbmdx_get_va_ve_common_resources(p);
		if (ret) {
			dev_err(p->dev,
				"%s: could not get VA_VE common resources\n",
				__func__);
			goto err_chip_remove;
		}

		if (p->va_chip_enabled) {
			ret = dbmdx_get_va_ve_chip_va_resources(p);
			if (ret) {
				dev_err(p->dev,
				"%s: couldn't get VA_VE chip VA resources\n",
					__func__);
				dbmdx_free_va_ve_common_resources(p);
				goto err_chip_remove;
			}
		}

		if (p->va_ve_chip_enabled) {
			ret = dbmdx_get_va_ve_chip_va_ve_resources(p);
			if (ret) {
				dev_err(p->dev,
				"%s: couldn't get VA_VE chip VA_VE resources\n",
					__func__);
				dbmdx_free_va_ve_common_resources(p);
				if (p->va_chip_enabled)
					dbmdx_free_va_ve_chip_va_resources(p);
				goto err_chip_remove;
			}

		}

#ifdef DBMDX_ALWAYS_RELOAD_ASRP_PARAMS
		p->va_load_asrp_params_options =
				DBMDX_ASRP_PARAMS_OPTIONS_ALWAYS_RELOAD;
		p->va_ve_load_asrp_params_options =
				DBMDX_ASRP_PARAMS_OPTIONS_ALWAYS_RELOAD;
#endif
	}

	p->cur_reset_gpio = p->pdata->gpio_reset;
	p->cur_wakeup_gpio = p->pdata->gpio_wakeup;

	ret = dbmdx_register_cdev(p);
	if (ret)
		goto err_free_resources;
#ifndef ALSA_SOC_INTERFACE_NOT_SUPPORTED
	ret = dbmdx_get_dai_drivers(p);
	if (ret)
		goto err_deregister_cdev;
#ifdef SND_SOC_COMPONENT
	/* register the component */
	ret = snd_soc_register_component(p->dev,
				     &soc_component_dev_dbmdx,
				     p->dais,
				     p->num_dais);
#else
	/* register the codec */
	ret = snd_soc_register_codec(p->dev,
				     &soc_codec_dev_dbmdx,
				     p->dais,
				     p->num_dais);

#endif
	if (ret != 0) {
		dev_err(p->dev,
			"%s: Failed to register codec and its DAI: %d\n",
			__func__, ret);
		goto err_free_dais;
	}
#endif
	boot_thread = kthread_run(dbmdx_fw_load_thread,
				  (void *)p,
				  "dbmdx probe thread");
	if (IS_ERR_OR_NULL(boot_thread)) {
		dev_err(p->dev,
			"%s: Cannot create DBMDX boot thread\n", __func__);
		ret = -EIO;
#ifndef ALSA_SOC_INTERFACE_NOT_SUPPORTED
#ifdef SND_SOC_COMPONENT
		goto err_component_unregister;
#else
		goto err_codec_unregister;
#endif
#else
		goto err_deregister_cdev;
#endif
	}

	ret = customer_dbmdx_common_probe(p);
	if (ret < 0) {
		dev_err(p->dev, "%s: custom COMMON_PROBE error\n",
			__func__);
		ret = -EIO;
#ifndef ALSA_SOC_INTERFACE_NOT_SUPPORTED
#ifdef SND_SOC_COMPONENT
		goto err_component_unregister;
#else
		goto err_codec_unregister;
#endif
#else
		goto err_deregister_cdev;
#endif
	}

	dev_info(p->dev, "%s: registered DBMDX codec driver version = %s\n",
		__func__, DRIVER_VERSION);

	return 0;

#ifndef ALSA_SOC_INTERFACE_NOT_SUPPORTED
#ifdef SND_SOC_COMPONENT
err_component_unregister:
	snd_soc_unregister_component(p->dev);
#else
err_codec_unregister:
	snd_soc_unregister_codec(p->dev);
#endif
err_free_dais:
	devm_kfree(p->dev, p->dais);
#endif
err_deregister_cdev:
	dbmdx_deregister_cdev(p);
err_free_resources:
	if (p->pdata->feature_va)
		dbmdx_free_va_resources(p);
	if (p->pdata->feature_va_ve) {
		dbmdx_free_va_ve_common_resources(p);
		if (p->va_chip_enabled)
			dbmdx_free_va_ve_chip_va_resources(p);
		if (p->va_ve_chip_enabled)
			dbmdx_free_va_ve_chip_va_ve_resources(p);

	}
err_chip_remove:
	if (p->pdata->feature_va)
		dbmdx_va_chip_remove(p);
	if (p->pdata->feature_va_ve) {
		if (p->va_chip_enabled)
			dbmdx_va_chip_remove(p);
		if (p->va_ve_chip_enabled)
			dbmdx_va_ve_chip_remove(p);
	}
err_sysfs_remove_group:
	sysfs_remove_group(&p->dev->kobj, &dbmdx_common_attribute_group);
err_class_destroy:
	class_destroy(p->ns_class);
err_device_unregister:
	device_unregister(p->dbmdx_dev);
err_clk_off:
	/* disable constant clock */
	p->clk_disable(p, DBMDX_CLK_CONSTANT);
#ifdef CONFIG_OF
err:
#endif
	return ret;
}

static void dbmdx_common_remove(struct dbmdx_private *p)
{
	int i;

	if (p->pdata->feature_va) {
		dbmdx_free_va_resources(p);
		dbmdx_va_chip_remove(p);
	}
	else if (p->pdata->feature_va_ve) {
		dbmdx_free_va_ve_common_resources(p);
		if (p->va_chip_enabled) {
			dbmdx_free_va_ve_chip_va_resources(p);
			dbmdx_va_chip_remove(p);
		}
		if (p->va_ve_chip_enabled) {
			dbmdx_free_va_ve_chip_va_ve_resources(p);
			dbmdx_va_ve_chip_remove(p);
		}

	}

#ifndef CONFIG_SND_SOC_DBMDX
	dbmdx_deinit_interface();
#if defined(CONFIG_SND_SOC_DBMDX_SND_CAPTURE)
	board_dbmdx_snd_exit();
	snd_dbmdx_pcm_exit();
#endif
#endif

	flush_workqueue(p->dbmdx_workq);
	destroy_workqueue(p->dbmdx_workq);
#ifndef ALSA_SOC_INTERFACE_NOT_SUPPORTED
#ifdef SND_SOC_COMPONENT
	snd_soc_unregister_component(p->dev);
#else
	snd_soc_unregister_codec(p->dev);
#endif
#endif
	dbmdx_deregister_cdev(p);
	sysfs_remove_group(&p->dev->kobj, &dbmdx_common_attribute_group);
	device_unregister(p->dbmdx_dev);
	class_destroy(p->ns_class);
	/* disable constant clock */
	p->clk_disable(p, DBMDX_CLK_CONSTANT);
	for (i = 0; i < DBMDX_NR_OF_CLKS; i++)
		if (p->clocks[i])
			clk_put(p->clocks[i]);
	devm_kfree(p->dev, p->dais);
	kfree(p);
}

static int dbmdx_va_chip_probe(struct dbmdx_private *p)
{
#ifdef CONFIG_OF
	struct device_node *np = p->dev->of_node;
#endif
	int ret = 0;

#ifdef CONFIG_OF
	if (!np) {
		dev_err(p->dev, "%s: error no devicetree entry\n", __func__);
		ret = -ENODEV;
		goto err;
	}
#endif

	ret = customer_dbmdx_va_chip_probe(p);
	if (ret < 0) {
		dev_err(p->dev, "%s: custom VA_CHIP_ONINIT error\n",
			__func__);
		ret = -EIO;
		goto err;
	}
	
#ifdef CONFIG_OF
	/* reset */
	p->pdata->gpio_reset = of_get_named_gpio(np, "reset-gpio", 0);
#endif
	if (!gpio_is_valid(p->pdata->gpio_reset)) {
		dev_err(p->dev, "%s: reset gpio invalid\n", __func__);
		ret = -EINVAL;
		goto err;
	}

	ret = gpio_request(p->pdata->gpio_reset, "DBMDX reset");
	if (ret < 0) {
		dev_err(p->dev, "%s: error requesting reset gpio\n", __func__);
		goto err;
	}
	gpio_direction_output(p->pdata->gpio_reset, 0);
	gpio_set_value(p->pdata->gpio_reset, 0);

	/* sv */
#ifdef CONFIG_OF
	p->pdata->gpio_sv = of_get_named_gpio(np, "sv-gpio", 0);
#endif
	if (!gpio_is_valid(p->pdata->gpio_sv)) {
		p->pdata->gpio_sv = -1;
		p->sv_irq = -1;
		dev_err(p->dev, "%s: sv gpio not available\n", __func__);
	} else {
		ret = gpio_request(p->pdata->gpio_sv, "DBMDX sv");
		if (ret < 0) {
			dev_err(p->dev, "%s: error requesting sv gpio\n",
				__func__);
			goto err_gpio_free;
		}
		gpio_direction_input(p->pdata->gpio_sv);

		/* interrupt gpio */
		p->sv_irq = ret = gpio_to_irq(p->pdata->gpio_sv);
		if (ret < 0) {
			dev_err(p->dev, "%s: cannot map gpio to irq\n",
				__func__);
			goto err_gpio_free;
		}
	}
	/* d2 strap 1 - only on some boards */
#ifdef CONFIG_OF
	p->pdata->gpio_d2strap1 = of_get_named_gpio(np, "d2-strap1", 0);
#endif
	if (gpio_is_valid(p->pdata->gpio_d2strap1)) {
		dev_info(p->dev,
			"%s: valid d2 strap1 gpio: %d\n",
			__func__,
			p->pdata->gpio_d2strap1);
		ret = gpio_request(p->pdata->gpio_d2strap1, "DBMDX STRAP 1");
		if (ret < 0) {
			dev_err(p->dev, "%s: error requesting d2 strap1 gpio\n",
				__func__);
			goto err_gpio_free;
		} else {
			/* keep it as input */
			if (gpio_direction_input(p->pdata->gpio_d2strap1)) {
				dev_err(p->dev,
					"%s: error setting d2 strap1 gpio to input\n",
					__func__);
				goto err_gpio_free;
			}
#ifdef CONFIG_OF
			ret = of_property_read_u32(np, "d2-strap1-rst-val",
					&p->pdata->gpio_d2strap1_rst_val);
			if (ret) {
				dev_dbg(p->dev,
					"%s: no d2-strap1-rst-val in dts\n",
					__func__);
				p->pdata->gpio_d2strap1_rst_val = 0;
			}

			/* normalize */
			if (p->pdata->gpio_d2strap1_rst_val > 1)
				p->pdata->gpio_d2strap1_rst_val = 1;
#endif
		}
	} else {
		dev_info(p->dev,
			"%s: missing or invalid d2 strap1 gpio: %d\n",
			__func__,
			p->pdata->gpio_d2strap1);
		p->pdata->gpio_d2strap1 = -1;
	}

	/* wakeup */
#ifdef CONFIG_OF
	p->pdata->gpio_wakeup = of_get_named_gpio(np, "wakeup-gpio", 0);
#endif
	if (!gpio_is_valid(p->pdata->gpio_wakeup)) {
		dev_info(p->dev, "%s: wakeup gpio not specified\n", __func__);
		p->pdata->gpio_wakeup = -1;
	} else {
		ret = gpio_request(p->pdata->gpio_wakeup, "DBMDX wakeup");
		if (ret < 0) {
			dev_err(p->dev, "%s: error requesting wakeup gpio\n",
				__func__);
			goto err_gpio_free;
		}
		/* keep the wakeup pin high */
		gpio_direction_output(p->pdata->gpio_wakeup, 1);
	}


	return 0;

err_gpio_free:
	if (p->pdata->gpio_wakeup >= 0)
		gpio_free(p->pdata->gpio_wakeup);
	if (p->pdata->gpio_d2strap1 >= 0)
		gpio_free(p->pdata->gpio_d2strap1);
	if (p->pdata->gpio_sv >= 0)
		gpio_free(p->pdata->gpio_sv);
	if (p->pdata->gpio_reset >= 0)
		gpio_free(p->pdata->gpio_reset);
err:
	return ret;
}

static void dbmdx_va_chip_remove(struct dbmdx_private *p)
{
	if (p->pdata->gpio_wakeup >= 0)
		gpio_free(p->pdata->gpio_wakeup);
	if (p->pdata->gpio_d2strap1 >= 0)
		gpio_free(p->pdata->gpio_d2strap1);
	if (p->pdata->gpio_sv >= 0)
		gpio_free(p->pdata->gpio_sv);
	if (p->pdata->gpio_reset >= 0)
		gpio_free(p->pdata->gpio_reset);
}

static int dbmdx_va_ve_chip_probe(struct dbmdx_private *p)
{
#ifdef CONFIG_OF
	struct device_node *np = p->dev->of_node;
#endif
	int ret = 0;

#ifdef CONFIG_OF
	if (!np) {
		dev_err(p->dev, "%s: error no devicetree entry\n", __func__);
		ret = -ENODEV;
		goto err;
	}
#endif

	ret = customer_dbmdx_va_ve_chip_probe(p);
	if (ret < 0) {
		dev_err(p->dev, "%s: custom VA_VE_CHIP_ONINIT error\n",
			__func__);
		ret = -EIO;
		goto err;
	}

#ifdef CONFIG_OF
	ret = of_property_read_u32(np, "reset_gpio_shared",
		&p->pdata->reset_gpio_shared);
	if ((ret && ret != -EINVAL)) {
		dev_err(p->dev, "%s: invalid 'reset_gpio_shared'\n",
			__func__);
		ret = -EINVAL;
		goto err;
	} else if (ret ==  -EINVAL) {
		dev_info(p->dev,
			"%s: no reset_gpio_shared definition in dev-tree\n",
			__func__);
		p->pdata->reset_gpio_shared = 0;
	} else {

		if (p->pdata->reset_gpio_shared > 1)
			p->pdata->reset_gpio_shared = 1;

		dev_info(p->dev,
			"%s: using reset_gpio_shared of %d from dev-tree\n",
			 __func__, p->pdata->reset_gpio_shared);

	}

	if (p->pdata->reset_gpio_shared)
		p->pdata->gpio_reset_va_ve = p->pdata->gpio_reset;
	/* reset va_ve*/
	else
		p->pdata->gpio_reset_va_ve = of_get_named_gpio(np,
						"reset-va_ve-gpio", 0);
#endif
	if (!(p->pdata->reset_gpio_shared)) {
		if (!gpio_is_valid(p->pdata->gpio_reset_va_ve)) {
			dev_err(p->dev, "%s: reset va_ve gpio invalid\n",
				__func__);
			ret = -EINVAL;
			goto err;

		}

		ret = gpio_request(p->pdata->gpio_reset_va_ve,
			"DBMDX reset va_ve");
		if (ret < 0) {
			dev_err(p->dev,
				"%s: error requesting reset va_ve gpio\n",
				__func__);
			goto err;
		}

		gpio_direction_output(p->pdata->gpio_reset_va_ve, 0);
		gpio_set_value(p->pdata->gpio_reset_va_ve, 0);
	}


	/* wakeup */
#ifdef CONFIG_OF
	p->pdata->gpio_wakeup_va_ve = of_get_named_gpio(np,
						"wakeup-va_ve-gpio", 0);
#endif
	if (!gpio_is_valid(p->pdata->gpio_wakeup_va_ve)) {
		dev_info(p->dev, "%s: wakeup va_ve gpio not specified\n",
			__func__);
		p->pdata->gpio_wakeup_va_ve = -1;
	} else {
		ret = gpio_request(p->pdata->gpio_wakeup_va_ve,
			"DBMDX va_ve wakeup");
		if (ret < 0) {
			dev_err(p->dev,
				"%s: error requesting wakeup va_ve gpio\n",
				__func__);
			goto err_gpio_free;
		}
		/* keep the wakeup pin high */
		gpio_direction_output(p->pdata->gpio_wakeup_va_ve, 1);
	}

	return 0;

err_gpio_free:
	if (p->pdata->gpio_wakeup_va_ve >= 0)
		gpio_free(p->pdata->gpio_wakeup_va_ve);
	if (p->pdata->gpio_reset_va_ve >= 0)
		gpio_free(p->pdata->gpio_reset_va_ve);
err:
	return ret;
}

static void dbmdx_va_ve_chip_remove(struct dbmdx_private *p)
{
	if (p->pdata->gpio_wakeup_va_ve >= 0)
		gpio_free(p->pdata->gpio_wakeup_va_ve);
	if (p->pdata->gpio_reset_va_ve >= 0)
		gpio_free(p->pdata->gpio_reset_va_ve);
}

#ifdef CONFIG_OF
static int of_dev_node_match(struct device *dev, void *data)
{
	return dev->of_node == data;
}

/* must call put_device() when done with returned i2c_client device */
struct platform_device *of_find_platform_device_by_node(
	struct device_node *node)
{
	struct device *dev;

	dev = bus_find_device(&platform_bus_type, NULL, node,
			      of_dev_node_match);
	if (!dev)
		return NULL;

	return container_of(dev, struct platform_device, dev);
}

static struct spi_device *of_find_spi_device_by_node(struct device_node *node)
{
	struct device *dev;

	dev = bus_find_device(&spi_bus_type, NULL, node, of_dev_node_match);

	if (!dev)
		return NULL;

	return to_spi_device(dev);
}

static int dbmdx_of_get_clk_info(struct dbmdx_private *p,
				 struct device_node *np,
				 enum dbmdx_clocks dbmdx_clk)
{
	int ret;
	int rate, rrate;
	struct clk *clk;

	ret = of_property_read_u32(np,
				   dbmdx_of_clk_rate_names[dbmdx_clk],
				   &rate);
	if (ret != 0) {
		dev_info(p->dev,
			 "%s: no %s definition in device-tree\n",
			 __func__,
			 dbmdx_of_clk_rate_names[dbmdx_clk]);
		rate = -1;
	} else
		dev_info(p->dev,
			 "%s: using %s at %dHZ from device-tree\n",
			 __func__,
			 dbmdx_of_clk_names[dbmdx_clk],
			 rate);

	clk = clk_get(p->dev, dbmdx_of_clk_names[dbmdx_clk]);
	if (IS_ERR(clk)) {
		dev_info(p->dev,
			 "%s: no %s definition in device-tree\n",
			 __func__,
			 dbmdx_of_clk_names[dbmdx_clk]);
		/* nothing in the device tree */
		clk = NULL;
	} else {
		/* If clock rate not specified in dts, try to detect */
		if (rate == -1) {
			rate = clk_get_rate(clk);
			dev_info(p->dev,
				 "%s: using %s at %dHZ\n",
				 __func__,
				 dbmdx_of_clk_names[dbmdx_clk],
				 rate);
		} else {
			/* verify which rate can be set */
			rrate = clk_round_rate(clk, rate);
			if (rrate !=  rate) {
				dev_info(p->dev,
					 "%s: rounded rate %d to %d\n",
					 __func__,
					 rate, rrate);
				rate = rrate;
			}
		}
	}
	p->clocks[dbmdx_clk] = clk;
	p->pdata->clock_rates[dbmdx_clk] = rate;

	return 0;
}

static int dbmdx_get_va_devtree_pdata(struct device *dev,
		struct dbmdx_private *p)
{
	struct dbmdx_platform_data *pdata = p->pdata;
	struct device_node *np;
	struct property *property = NULL;
	int ret = 0;
	int i;

	np = dev->of_node;

	if (!pdata->feature_va && !pdata->feature_va_ve)
		return 0;

	/* read file names for the various firmwares */
	/* read name of VA firmware */
	ret = of_property_read_string(np,
				      "va-firmware-name",
				      &pdata->va_firmware_name);
	if (ret != 0) {
		/* set default name */
		pdata->va_firmware_name = DBMD2_VA_FIRMWARE_NAME;
		dev_info(dev, "%s: using default VA firmware name: %s\n",
			 __func__, pdata->va_firmware_name);
	} else
		dev_info(dev, "%s: using device-tree VA firmware name: %s\n",
			__func__, pdata->va_firmware_name);

	ret = of_property_read_string(np,
				      "va-preboot-firmware-name",
				      &pdata->va_preboot_firmware_name);
	if (ret != 0) {
		/* set default name */
		pdata->va_preboot_firmware_name =
			DBMD4_VA_PREBOOT_FIRMWARE_NAME;
		dev_info(dev,
			"%s: using default VA preboot firmware name: %s\n",
			 __func__, pdata->va_preboot_firmware_name);
	} else
		dev_info(dev,
			"%s: using device-tree VA preboot firmware name: %s\n",
			__func__, pdata->va_preboot_firmware_name);

	ret = of_property_read_u32(np, "auto_buffering",
		&p->pdata->auto_buffering);

	if ((ret && ret != -EINVAL) || (p->pdata->auto_buffering != 0 &&
		p->pdata->auto_buffering != 1)) {
		dev_err(p->dev, "%s: invalid 'auto_buffering'\n", __func__);
		ret = -EINVAL;
		goto out_err;
	}

	ret = of_property_read_u32(np, "auto_detection",
		&p->pdata->auto_detection);
	if ((ret && ret != -EINVAL) || (p->pdata->auto_detection != 0 &&
			p->pdata->auto_detection != 1)) {
		dev_err(p->dev, "%s: invalid 'auto_detection'\n", __func__);
		ret = -EINVAL;
		goto out_err;
	}

	ret = of_property_read_u32(np, "detection_after_buffering",
				&p->pdata->detection_after_buffering);
	if ((ret && ret != -EINVAL) ||
			(p->pdata->detection_after_buffering >=
				DETECTION_AFTER_BUFFERING_MODE_MAX)) {
		dev_err(p->dev, "%s: invalid 'detection_after_buffering'\n",
			__func__);
		ret = -EINVAL;
		goto out_err;
	}

	ret = of_property_read_u32(np, "send_uevent_on_detection",
			&p->pdata->send_uevent_on_detection);
	if ((ret && ret != -EINVAL) ||
			(p->pdata->send_uevent_on_detection != 0 &&
			p->pdata->send_uevent_on_detection != 1)) {
		dev_err(p->dev, "%s: invalid 'send_uevent_on_detection'\n",
			__func__);
		ret = -EINVAL;
		goto out_err;
	}

	ret = of_property_read_u32(np, "send_uevent_after_buffering",
			&p->pdata->send_uevent_after_buffering);
	if ((ret && ret != -EINVAL) ||
			(p->pdata->send_uevent_after_buffering != 0 &&
			p->pdata->send_uevent_after_buffering != 1)) {
		dev_err(p->dev, "%s: invalid 'send_uevent_after_buffering'\n",
			__func__);
		ret = -EINVAL;
		goto out_err;
	}

	ret = of_property_read_u32(np, "buffering_timeout",
			&p->pdata->buffering_timeout);
	if (ret && ret != -EINVAL) {
		dev_err(p->dev, "%s: invalid 'buffering_timeout'\n",
			__func__);
		ret = -EINVAL;
		goto out_err;
	} else if (ret ==  -EINVAL) {
		dev_info(p->dev,
			"%s: no buffering_timeout, setting to %d\n",
				__func__, MAX_RETRIES_TO_WRITE_TOBUF);
		p->pdata->buffering_timeout = MAX_RETRIES_TO_WRITE_TOBUF;
	} else {

		if (p->pdata->buffering_timeout < MIN_RETRIES_TO_WRITE_TOBUF)
			p->pdata->buffering_timeout =
					MIN_RETRIES_TO_WRITE_TOBUF;

		dev_info(p->dev,
			"%s: using buffering_timeout of %u from dev tree\n",
			 __func__, p->pdata->buffering_timeout);
	}

	ret = of_property_read_u32(np, "detection_buffer_channels",
			&p->pdata->detection_buffer_channels);
	if ((ret && ret != -EINVAL) ||
		(p->pdata->detection_buffer_channels >
			MAX_SUPPORTED_CHANNELS)) {
		dev_err(p->dev, "%s: detection_buffer_channels'\n", __func__);
		ret = -EINVAL;
		goto out_err;
	}

	ret = of_property_read_u32(np, "min_samples_chunk_size",
			&p->pdata->min_samples_chunk_size);
	if ((ret && ret != -EINVAL)) {
		dev_err(p->dev, "%s: min_samples_chunk_size'\n", __func__);
		ret = -EINVAL;
		goto out_err;
	}

	dev_info(p->dev, "%s: using min_samples_chunk_size of %d\n",
			 __func__, p->pdata->min_samples_chunk_size);

	ret = of_property_read_u32(np, "max_detection_buffer_size",
			&p->pdata->max_detection_buffer_size);
	if ((ret && ret != -EINVAL)) {
		dev_err(p->dev, "%s: max_detection_buffer_size'\n", __func__);
		ret = -EINVAL;
		goto out_err;
	}

	dev_info(p->dev, "%s: using max_detection_buffer_size of %d\n",
			 __func__, p->pdata->max_detection_buffer_size);


	ret = of_property_read_u32(np, "va_buffering_pcm_rate",
			&p->pdata->va_buffering_pcm_rate);
	if (ret && ret != -EINVAL) {
		dev_err(p->dev, "%s: invalid 'va_buffering_pcm_rate'\n",
			__func__);
		ret = -EINVAL;
		goto out_err;
	} else if (ret ==  -EINVAL) {
		dev_info(p->dev,
			"%s: no va buffering pcm rate, setting to 16000\n",
				__func__);
		p->pdata->va_buffering_pcm_rate = 16000;
	} else {
		if (p->pdata->va_buffering_pcm_rate != 16000 &&
			p->pdata->va_buffering_pcm_rate != 32000)
			p->pdata->va_buffering_pcm_rate = 16000;

		dev_info(p->dev,
			"%s: using va_buffering_pcm_rate of %u from dev tree\n",
			 __func__, p->pdata->va_buffering_pcm_rate);
	}

	property = of_find_property(np, "va-config", &pdata->va_cfg_values);
	if (property) {
		pdata->va_cfg_value = kzalloc(pdata->va_cfg_values, GFP_KERNEL);
		if (!pdata->va_cfg_value) {
			ret = -ENOMEM;
			goto out_err;
		}
		pdata->va_cfg_values /= sizeof(u32);
		ret = of_property_read_u32_array(np,
						 "va-config",
						 pdata->va_cfg_value,
						 pdata->va_cfg_values);
		if (ret) {
			dev_err(dev, "%s: Could not read VA configuration\n",
				__func__);
			ret = -EIO;
			goto out_free_va_resources;
		}
		dev_info(dev,
			"%s: using %u VA configuration values from dev-tree\n",
				__func__, pdata->va_cfg_values);
		for (i = 0; i < pdata->va_cfg_values; i++)
			dev_dbg(dev, "%s: VA cfg %8.8x: 0x%8.8x\n",
				__func__, i, pdata->va_cfg_value[i]);
	}

	ret = of_property_read_u32_array(np,
					 "va-mic-config",
					 pdata->va_mic_config,
					 VA_MIC_CONFIG_SIZE);
	if (ret != 0) {
		dev_err(p->dev, "%s: invalid or missing 'va-mic-config'\n",
			__func__);
		goto out_free_va_resources;
	}
	dev_info(dev,
		"%s: using %u VA mic configuration values from device-tree\n",
		__func__, VA_MIC_CONFIG_SIZE);
	for (i = 0; i < VA_MIC_CONFIG_SIZE; i++)
		dev_dbg(dev, "%s: VA mic cfg %8.8x: 0x%8.8x\n",
			__func__, i, pdata->va_mic_config[i]);

	ret = of_property_read_u32(np,
				   "va-mic-mode",
				   &pdata->va_initial_mic_config);
	if (ret != 0) {
		dev_err(p->dev, "%s: invalid or missing 'va-mic-mode'\n",
			__func__);
		goto out_free_va_resources;
	}
	dev_info(dev, "%s: VA default mic config: 0x%8.8x\n",
		 __func__, pdata->va_initial_mic_config);


	ret = of_property_read_u32(np,
		"digital_mic_digital_gain",
		&pdata->va_mic_gain_config[DBMDX_DIGITAL_MIC_DIGITAL_GAIN]);
	if (ret != 0)
		pdata->va_mic_gain_config[DBMDX_DIGITAL_MIC_DIGITAL_GAIN]
		= 0x1000;
	else {
		dev_info(dev,
		"%s: using digital mic gain config 0x%04X from device-tree\n",
		__func__,
		pdata->va_mic_gain_config[DBMDX_DIGITAL_MIC_DIGITAL_GAIN]);
	}

	ret = of_property_read_u32(np,
		"analog_mic_analog_gain",
		&pdata->va_mic_gain_config[DBMDX_ANALOG_MIC_ANALOG_GAIN]);
	if (ret != 0)
		pdata->va_mic_gain_config[DBMDX_ANALOG_MIC_ANALOG_GAIN]
		= 0x1000;
	else {
		dev_info(dev,
		"%s: using analog mic gain config 0x%04X from device-tree\n",
		__func__,
		pdata->va_mic_gain_config[DBMDX_ANALOG_MIC_ANALOG_GAIN]);
	}

	ret = of_property_read_u32(np,
		"analog_mic_digital_gain",
		&pdata->va_mic_gain_config[DBMDX_ANALOG_MIC_DIGITAL_GAIN]);
	if (ret != 0)
		pdata->va_mic_gain_config[DBMDX_ANALOG_MIC_DIGITAL_GAIN]
		= 0x1000;
	else {
		dev_info(dev,
		"%s: using analog mic digital gain 0x%04X from device-tree\n",
		__func__,
		pdata->va_mic_gain_config[DBMDX_ANALOG_MIC_DIGITAL_GAIN]);
	}

#ifdef DMBDX_GENERAL_AMODEL_SUPPORT
	ret = of_property_read_u32(np,
				   "va_backlog_length",
				   &p->pdata->va_backlog_length);
	if (ret != 0) {
		dev_info(dev,
			"%s: no va_backlog_length definition in device-tree\n",
			__func__);
		p->pdata->va_backlog_length = 1;
	} else {
		if (p->pdata->va_backlog_length > 0xfff)
			p->pdata->va_backlog_length = 0xfff;
		dev_info(dev,
			"%s: using backlog length of %d from device-tree\n",
			 __func__, p->pdata->va_backlog_length);
	}
#endif
#ifdef DMBDX_SENSORY_AMODEL_SUPPORT
	ret = of_property_read_u32(np,
				   "va_backlog_length_sensory",
				   &p->pdata->va_backlog_length_sensory);
	if (ret != 0) {
		dev_info(dev,
			"%s: no va_backlog_length_sensory def. in device-tree\n",
			__func__);
		p->pdata->va_backlog_length_sensory = 1;
	} else {
		if (p->pdata->va_backlog_length_sensory > 0xfff)
			p->pdata->va_backlog_length_sensory = 0xfff;
		dev_info(dev,
			"%s: using Sensory backlog length of %d from device-tree\n",
			 __func__, p->pdata->va_backlog_length_sensory);
	}
#endif
#ifdef DMBDX_OKG_AMODEL_SUPPORT
	ret = of_property_read_u32(np,
				   "va_backlog_length_okg",
				   &p->pdata->va_backlog_length_google);
	if (ret != 0) {
		dev_info(dev,
			"%s: no va_backlog_length_okg def. in device-tree\n",
			__func__);
		p->pdata->va_backlog_length_google = 1000;
	} else {
		if (p->pdata->va_backlog_length_google > 0xfff)
			p->pdata->va_backlog_length_google = 0xfff;
		dev_info(dev,
			"%s: using OKG backlog length of %d from device-tree\n",
			 __func__, p->pdata->va_backlog_length_google);
	}
#endif
#ifdef DMBDX_ALEXA_AMODEL_SUPPORT
	ret = of_property_read_u32(np,
				   "va_backlog_length_alexa",
				   &p->pdata->va_backlog_length_alexa);
	if (ret != 0) {
		dev_info(dev,
			"%s: no va_backlog_length_alexa def. in device-tree\n",
			__func__);
		p->pdata->va_backlog_length_alexa = 500;
	} else {
		if (p->pdata->va_backlog_length_alexa > 0xfff)
			p->pdata->va_backlog_length_alexa = 0xfff;
		dev_info(dev,
			"%s: using Alexa backlog length of %d from device-tree\n",
			 __func__, p->pdata->va_backlog_length_alexa);
	}
#endif
#ifdef DMBDX_BIXBY_AMODEL_SUPPORT
	ret = of_property_read_u32(np,
				   "va_backlog_length_bixby",
				   &p->pdata->va_backlog_length_bixby);
	if (ret != 0) {
		dev_info(dev,
			"%s: no va_backlog_length_bixby def. in device-tree\n",
			__func__);
		p->pdata->va_backlog_length_bixby = 500;
	} else {
		if (p->pdata->va_backlog_length_bixby > 0xfff)
			p->pdata->va_backlog_length_bixby = 0xfff;
		dev_info(dev,
			"%s: using Bixby backlog length of %d from device-tree\n",
			 __func__, p->pdata->va_backlog_length_bixby);
	}
#endif
	ret = of_property_read_u32(np, "pcm_streaming_mode",
		&p->pdata->pcm_streaming_mode);
	if ((ret && ret != -EINVAL) ||
			(p->pdata->pcm_streaming_mode != 0 &&
				p->pdata->pcm_streaming_mode != 1)) {
		dev_err(p->dev, "%s: invalid 'pcm_streaming_mode'\n", __func__);
		goto out_free_va_resources;
	}

	ret = dbmdx_get_fw_interfaces(p, "va-interfaces",
		p->pdata->va_interfaces);
	if (ret)
		goto out_free_va_resources;

	return 0;

out_free_va_resources:
	if (pdata->va_cfg_values)
		kfree(pdata->va_cfg_value);
	pdata->va_cfg_values = 0;
out_err:
	return ret;
}

static int dbmdx_get_va_ve_devtree_pdata(struct device *dev,
		struct dbmdx_private *p)
{
	struct dbmdx_platform_data *pdata = p->pdata;
	struct device_node *np;
	struct property *property = NULL;
	int ret = 0;
	int i;

	np = dev->of_node;

	if (!pdata->feature_va_ve)
		return 0;

	/* read file names for the various firmwares */
	/* read name of VA_VE firmware */
	ret = of_property_read_string(np,
				      "va_ve-firmware-name",
				      &pdata->va_ve_firmware_name);
	if (ret != 0) {
		/* set default name */
		pdata->va_ve_firmware_name = DBMD2_VA_VE_FIRMWARE_NAME;
		dev_info(dev, "%s: using default VA_VE firmware name: %s\n",
			 __func__, pdata->va_ve_firmware_name);
	} else
		dev_info(dev, "%s: using device-tree VA_VE firmware name: %s\n",
			__func__, pdata->va_ve_firmware_name);

	ret = of_property_read_string(np,
				      "va_ve-preboot-firmware-name",
				      &pdata->va_ve_preboot_firmware_name);
	if (ret != 0) {
		/* set default name */
		pdata->va_ve_preboot_firmware_name =
			DBMD2_VA_PREBOOT_FIRMWARE_NAME;
		dev_info(dev,
			"%s: using default VA_VE preboot firmware name: %s\n",
			 __func__, pdata->va_ve_preboot_firmware_name);
	} else
		dev_info(dev,
			"%s: using device-tree VA_VE preboot fw. name: %s\n",
			__func__, pdata->va_ve_preboot_firmware_name);

	property = of_find_property(np, "va_ve-config",
		&pdata->va_ve_cfg_values);
	if (property) {
		pdata->va_ve_cfg_value = kzalloc(pdata->va_ve_cfg_values,
			GFP_KERNEL);
		if (!pdata->va_ve_cfg_value) {
			ret = -ENOMEM;
			goto out_err;
		}
		pdata->va_ve_cfg_values /= sizeof(u32);
		ret = of_property_read_u32_array(np,
						 "va_ve-config",
						 pdata->va_ve_cfg_value,
						 pdata->va_ve_cfg_values);
		if (ret) {
			dev_err(dev, "%s: Could not read VA_VE configuration\n",
				__func__);
			ret = -EIO;
			goto out_free_va_ve_resources;
		}
		dev_info(dev,
			"%s: using %u VA_VE conf. values from dev-tree\n",
				__func__, pdata->va_ve_cfg_values);
		for (i = 0; i < pdata->va_ve_cfg_values; i++)
			dev_dbg(dev, "%s: VA_VE cfg %8.8x: 0x%8.8x\n",
				__func__, i, pdata->va_ve_cfg_value[i]);
	}

	ret = of_property_read_u32_array(np,
					 "va_ve-mic-config",
					 pdata->va_ve_mic_config,
					 VA_VE_MIC_CONFIG_SIZE);
	if (ret != 0) {
		dev_err(p->dev, "%s: missing 'va_ve-mic-config'\n",
			__func__);
		for (i = 0; i < VA_VE_MIC_CONFIG_SIZE; i++)
			pdata->va_ve_mic_config[i] = 0;
	} else {
		dev_info(dev,
			"%s: using %u VA_VE mic conf. values from dev-tree\n",
			__func__, VA_VE_MIC_CONFIG_SIZE);
		for (i = 0; i < VA_VE_MIC_CONFIG_SIZE; i++)
			dev_dbg(dev, "%s: VA_VE mic cfg %8.8x: 0x%8.8x\n",
				__func__, i, pdata->va_ve_mic_config[i]);
	}

	ret = dbmdx_get_fw_interfaces(p, "va_ve-interfaces",
						p->pdata->va_ve_interfaces);
	if (ret)
		goto out_free_va_ve_resources;

	return 0;

out_free_va_ve_resources:
	if (pdata->va_ve_cfg_values)
		kfree(pdata->va_ve_cfg_value);
	pdata->va_ve_cfg_values = 0;
out_err:
	return ret;
}


static int dbmdx_get_devtree_pdata(struct device *dev,
		struct dbmdx_private *p)
{
	struct dbmdx_platform_data *pdata = p->pdata;
	struct property *property = NULL;
	struct device_node *np;
	int ret = 0;
	int len = 0;
	int i = 0;

	np = dev->of_node;

	ret = of_property_read_u32(np, "project_sub_type",
			&pdata->project_sub_type);
	if (ret && ret != -EINVAL) {
		dev_err(p->dev, "%s: invalid 'project_sub_type'\n",
			__func__);
		ret = -EINVAL;
		goto out_err;
	} else if (ret ==  -EINVAL) {
		dev_info(p->dev,
			"%s: no project_sub_type, setting to %d\n",
				__func__, 0);
		pdata->project_sub_type = DBMDX_VA_VE_PROJECT_SUB_TYPE_VT_ASRP;
	} else {

		dev_info(p->dev,
			"%s: using project_sub_type of %u from dev tree\n",
			 __func__, pdata->project_sub_type);
	}


	/* check for features */
	if (of_find_property(np, "feature-va", NULL)) {
		dev_info(dev, "%s: VA feature activated\n", __func__);
		pdata->feature_va = 1;
		p->va_chip_enabled = true;
	}

	if (of_find_property(np, "feature-va_ve", NULL)) {
		dev_info(dev, "%s: VA_VE feature activated\n", __func__);
		pdata->feature_va_ve = 1;

		if (pdata->project_sub_type ==
			DBMDX_VA_VE_PROJECT_SUB_TYPE_VT_ASRP) {
			/* Mango */
			p->va_chip_enabled = true;
			p->va_ve_chip_enabled = true;
			p->mics_connected_to_va_ve_chip = false;
			p->asrp_runs_on_vt = false;
			p->tdm_enable = (DBMDX_TDM_0_VA |
						DBMDX_TDM_1_VA |
						DBMDX_TDM_0_VA_VE |
						DBMDX_TDM_2_VA_VE |
						DBMDX_TDM_3_VA_VE);
#ifdef DBMDX_MANGO_USECASES_HW_REV_03_SUPPORTED
			p->idle_usecase = &config_uc_mango_idle;
#else
			p->idle_usecase = NULL;
#endif
		} else if (pdata->project_sub_type ==
			DBMDX_VA_VE_PROJECT_SUB_TYPE_VT_ONLY) {
			/* Melon */
			p->va_chip_enabled = true;
			p->va_ve_chip_enabled = false;
			p->mics_connected_to_va_ve_chip = false;
			p->asrp_runs_on_vt = true;
			p->tdm_enable = (DBMDX_TDM_0_VA |
						DBMDX_TDM_1_VA);
#ifdef DBMDX_MELON_USECASES_SUPPORTED
			p->idle_usecase = &config_uc_melon_idle;
#else
			p->idle_usecase = NULL;
#endif
		} else if (pdata->project_sub_type ==
			DBMDX_VA_VE_PROJECT_SUB_TYPE_ASRP_ONLY) {
			/* Orange */
			p->va_chip_enabled = false;
			p->va_ve_chip_enabled = true;
			p->mics_connected_to_va_ve_chip = true;
			p->asrp_runs_on_vt = false;
			p->tdm_enable = (DBMDX_TDM_0_VA_VE |
						DBMDX_TDM_2_VA_VE |
						DBMDX_TDM_3_VA_VE);
#ifdef DBMDX_ORANGE_USECASES_SUPPORTED
			p->idle_usecase = &config_uc_orange_idle;
#else
			p->idle_usecase = NULL;
#endif
		} else {
			dev_info(dev, "%s: Unknown project type %u\n", __func__,
				pdata->project_sub_type);
			p->va_chip_enabled = true;
			p->va_ve_chip_enabled = true;
			p->tdm_enable = (DBMDX_TDM_0_VA |
						DBMDX_TDM_1_VA |
						DBMDX_TDM_0_VA_VE |
						DBMDX_TDM_2_VA_VE |
						DBMDX_TDM_3_VA_VE);
#ifdef DBMDX_MANGO_USECASES_HW_REV_03_SUPPORTED
			p->idle_usecase = &config_uc_mango_idle;
#else
			p->idle_usecase = NULL;
#endif

		}

	}


	if (!pdata->feature_va_ve && !pdata->feature_va) {
		dev_err(dev, "%s: No feature activated\n", __func__);
		ret = -EINVAL;
		goto out_err;
	}

	if (pdata->feature_va_ve && pdata->feature_va) {
		dev_err(dev, "%s: Both VA and VA_VE features are activated\n",
			__func__);
		ret = -EINVAL;
		goto out_err;
	}
	if (of_find_property(np, "multi-interface-support", NULL)) {
		dev_info(dev, "%s: Multi Interface Probe is supported\n",
			__func__);
		pdata->multi_interface_support = 1;
	} else {
		dev_info(dev, "%s: Multi Interface Probe is not supported\n",
			__func__);
		pdata->multi_interface_support = 0;
	}

	if (pdata->feature_va) {
		/* read VA devtree data */
		ret = dbmdx_get_va_devtree_pdata(dev, p);
		if (ret != 0) {
			dev_err(dev, "%s: Error reading VA device tree data\n",
				__func__);
			ret = -EINVAL;
			goto out_err;
		}
	}
	if (pdata->feature_va_ve) {
		/* read VA devtree data */
		if (p->va_chip_enabled) {
			ret = dbmdx_get_va_devtree_pdata(dev, p);
			if (ret != 0) {
				dev_err(dev,
				"%s: Error reading VA device tree data\n",
					__func__);
				ret = -EINVAL;
				goto out_err;
			}
		}
		/* read VA_VE devtree data */
		if (p->va_ve_chip_enabled) {
			ret = dbmdx_get_va_ve_devtree_pdata(dev, p);
			if (ret != 0) {
				dev_err(dev,
				"%s: Error reading VA_VE device tree data\n",
				__func__);
				ret = -EINVAL;
				goto out_err;
			}
		}
	}

	property = of_find_property(np, "va-speeds", &len);
	if (property) {
		if (len < DBMDX_VA_NR_OF_SPEEDS * 4) {
			dev_err(dev,
				"%s: VA speed configuration table too short\n",
				__func__);
			ret = -EINVAL;
			goto out_err;
		}
		ret = of_property_read_u32_array(np,
						 "va-speeds",
						 (u32 *)&pdata->va_speed_cfg,
						 DBMDX_VA_NR_OF_SPEEDS * 4);
		if (ret) {
			dev_err(dev,
				"%s: Could not read VA speed configuration\n",
				__func__);
			ret = -EINVAL;
			goto out_err;
		}
		dev_info(dev,
			"%s: using %u VA speed configuration values from device-tree\n",
			__func__,
			DBMDX_VA_NR_OF_SPEEDS);
		for (i = 0; i < DBMDX_VA_NR_OF_SPEEDS; i++)
			dev_dbg(dev, "%s: VA speed cfg %8.8x: 0x%8.8x %u %u %u\n",
				__func__,
				i,
				pdata->va_speed_cfg[i].cfg,
				pdata->va_speed_cfg[i].uart_baud,
				pdata->va_speed_cfg[i].i2c_rate,
				pdata->va_speed_cfg[i].spi_rate);
	}

	ret = of_property_read_u32(np, "hw_revision",
		&p->pdata->hw_rev);
	if (ret && ret != -EINVAL) {
		dev_err(p->dev, "%s: invalid 'hw_revision'\n", __func__);
		ret = -EINVAL;
		goto out_err;
	} else if (ret ==  -EINVAL) {
		dev_info(p->dev,
			"%s: no hw_revision definition in device-tree.\n",
			__func__);
		p->pdata->hw_rev = DBMDX_DEFAULT_HW_REV;
	}

	ret = of_property_read_u32(np, "wakeup_disabled",
		&p->pdata->wakeup_disabled);
	if ((ret && ret != -EINVAL)) {
		dev_err(p->dev, "%s: invalid 'wakeup_disabled'\n",
			__func__);
		ret = -EINVAL;
		goto out_err;
	} else if (ret ==  -EINVAL) {
		dev_info(p->dev,
			"%s: no wakeup_disabled definition in dev-tree\n",
			__func__);
		p->pdata->wakeup_disabled = 0;
	} else {

		if (p->pdata->wakeup_disabled > 1)
			p->pdata->wakeup_disabled = 1;

		dev_info(p->dev,
			"%s: using wakeup_disabled of %d from dev-tree\n",
			 __func__, p->pdata->wakeup_disabled);

	}

	ret = of_property_read_u32(np, "use_gpio_for_wakeup",
		&p->pdata->use_gpio_for_wakeup);
	if ((ret && ret != -EINVAL)) {
		dev_err(p->dev, "%s: invalid 'use_gpio_for_wakeup'\n",
			__func__);
		ret = -EINVAL;
		goto out_err;
	} else if (ret ==  -EINVAL) {
		dev_info(p->dev,
			"%s: no use_gpio_for_wakeup definition in dev-tree\n",
			__func__);
		p->pdata->use_gpio_for_wakeup = 1;
	} else {

		if (p->pdata->use_gpio_for_wakeup > 1)
			p->pdata->use_gpio_for_wakeup = 1;

		dev_info(p->dev,
			"%s: using use_gpio_for_wakeup of %d from dev-tree\n",
			 __func__, p->pdata->use_gpio_for_wakeup);

	}

	ret = of_property_read_u32(np, "send_wakeup_seq",
		&p->pdata->send_wakeup_seq);
	if ((ret && ret != -EINVAL)) {
		dev_err(p->dev, "%s: invalid 'send_wakeup_seq'\n",
			__func__);
		ret = -EINVAL;
		goto out_err;
	} else if (ret ==  -EINVAL) {
		dev_info(p->dev,
			"%s: no send_wakeup_seq definition in device-tree\n",
			__func__);
		p->pdata->send_wakeup_seq = 0;
	} else {

		if (p->pdata->send_wakeup_seq > 1)
			p->pdata->send_wakeup_seq = 1;

		dev_info(p->dev,
			"%s: using send_wakeup_seq of %d from device-tree\n",
			 __func__, p->pdata->send_wakeup_seq);

	}

	ret = of_property_read_u32(np, "wakeup_set_value",
		&p->pdata->wakeup_set_value);
	if ((ret && ret != -EINVAL)) {
		dev_err(p->dev, "%s: invalid 'wakeup_set_value'\n",
			__func__);
		ret = -EINVAL;
		goto out_err;
	} else if (ret ==  -EINVAL) {
		dev_info(p->dev,
			"%s: no wakeup_set_value definition in device-tree\n",
			__func__);
		p->pdata->wakeup_set_value = 1;
	} else {

		if (p->pdata->wakeup_set_value > 1)
			p->pdata->wakeup_set_value = 1;

		dev_info(p->dev,
			"%s: using wakeup_set_value of %d from device-tree\n",
			 __func__, p->pdata->wakeup_set_value);

	}

	ret = of_property_read_u32(np, "firmware_id",
		&p->pdata->firmware_id);
	if (ret && ret != -EINVAL) {
		dev_err(p->dev, "%s: invalid 'firmware_id'\n", __func__);
		ret = -EINVAL;
		goto out_err;
	} else if (ret ==  -EINVAL) {
		dev_info(p->dev,
		"%s: no firmware_id definition in device-tree. assuming d2\n",
			__func__);
		p->pdata->firmware_id = DBMDX_FIRMWARE_ID_DBMD2;
	} else {
		dev_info(p->dev,
			"%s: using firmware_id of 0x%8x from device-tree\n",
			 __func__, p->pdata->firmware_id);
	}

	ret = of_property_read_u32(np, "boot_options",
		&p->pdata->boot_options);
	if (ret && ret != -EINVAL) {
		dev_err(p->dev, "%s: invalid 'boot_options'\n", __func__);
		ret = -EINVAL;
		goto out_err;
	} else if (ret ==  -EINVAL) {
		dev_info(p->dev,
			"%s: no boot_options definition in device-tree.\n",
			__func__);
		p->pdata->boot_options = DBMDX_BOOT_MODE_NORMAL_BOOT;
	}
#ifdef DMBDX_GENERAL_AMODEL_SUPPORT
	ret = of_property_read_u32(np, "amodel_options",
		&p->pdata->amodel_options);
	if (ret && ret != -EINVAL) {
		dev_err(p->dev, "%s: invalid 'amodel_options'\n", __func__);
		ret = -EINVAL;
		goto out_err;
	} else if (ret ==  -EINVAL) {
		dev_info(p->dev,
			"%s: no amodel_options definition in device-tree.\n",
			__func__);
		p->pdata->amodel_options = DBMDX_AMODEL_DEFAULT_OPTIONS;
	}

#endif

#ifdef DMBDX_GENERAL_AMODEL_SUPPORT
	ret = of_property_read_u32(np, "amodel_options_sensory",
		&p->pdata->amodel_options_sensory);
	if (ret && ret != -EINVAL) {
		dev_err(p->dev, "%s: invalid 'amodel_options_sensory'\n",
			__func__);
		ret = -EINVAL;
		goto out_err;
	} else if (ret ==  -EINVAL) {
		dev_info(p->dev,
			"%s: no amodel_options_sensory definition in device-tree.\n",
			__func__);
		p->pdata->amodel_options_sensory =
				DBMDX_AMODEL_SENSORY_DEFAULT_OPTIONS;
	}

#endif

	ret = of_property_read_u32(np, "wakeup_va_ve_disabled",
		&p->pdata->wakeup_va_ve_disabled);
	if ((ret && ret != -EINVAL)) {
		dev_err(p->dev, "%s: invalid 'wakeup_va_ve_disabled'\n",
			__func__);
		ret = -EINVAL;
		goto out_err;
	} else if (ret ==  -EINVAL) {
		dev_info(p->dev,
			"%s: no wakeup_va_ve_disabled def. in dev-tree\n",
			__func__);
		p->pdata->wakeup_va_ve_disabled = 0;
	} else {

		if (p->pdata->wakeup_va_ve_disabled > 1)
			p->pdata->wakeup_va_ve_disabled = 1;

		dev_info(p->dev,
		"%s: using wakeup_va_ve_disabled of %d from dev-tree\n",
			 __func__, p->pdata->wakeup_va_ve_disabled);

	}

	ret = of_property_read_u32(np, "use_gpio_for_wakeup_va_ve",
		&p->pdata->use_gpio_for_wakeup_va_ve);
	if ((ret && ret != -EINVAL)) {
		dev_err(p->dev, "%s: invalid 'use_gpio_for_wakeup_va_ve'\n",
			__func__);
		ret = -EINVAL;
		goto out_err;
	} else if (ret ==  -EINVAL) {
		dev_info(p->dev,
			"%s: no use_gpio_for_wakeup_va_ve def. in dev-tree\n",
			__func__);
		p->pdata->use_gpio_for_wakeup_va_ve = 1;
	} else {

		if (p->pdata->use_gpio_for_wakeup_va_ve > 1)
			p->pdata->use_gpio_for_wakeup_va_ve = 1;

		dev_info(p->dev,
		"%s: using use_gpio_for_wakeup_va_ve of %d from dev-tree\n",
			 __func__, p->pdata->use_gpio_for_wakeup_va_ve);

	}

	ret = of_property_read_u32(np, "send_wakeup_va_ve_seq",
		&p->pdata->send_wakeup_va_ve_seq);
	if ((ret && ret != -EINVAL)) {
		dev_err(p->dev, "%s: invalid 'send_wakeup_va_ve_seq'\n",
			__func__);
		ret = -EINVAL;
		goto out_err;
	} else if (ret ==  -EINVAL) {
		dev_info(p->dev,
			"%s: no send_wakeup_va_ve_seq def. in device-tree\n",
			__func__);
		p->pdata->send_wakeup_va_ve_seq = 0;
	} else {

		if (p->pdata->send_wakeup_va_ve_seq > 1)
			p->pdata->send_wakeup_va_ve_seq = 1;

		dev_info(p->dev,
		"%s: using send_wakeup_va_ve_seq of %d from device-tree\n",
			 __func__, p->pdata->send_wakeup_va_ve_seq);

	}

	ret = of_property_read_u32(np, "wakeup_va_ve_set_value",
		&p->pdata->wakeup_va_ve_set_value);
	if ((ret && ret != -EINVAL)) {
		dev_err(p->dev, "%s: invalid 'wakeup_va_ve_set_value'\n",
			__func__);
		ret = -EINVAL;
		goto out_err;
	} else if (ret ==  -EINVAL) {
		dev_info(p->dev,
			"%s: no wakeup_va_ve_set_value def. in device-tree\n",
			__func__);
		p->pdata->wakeup_va_ve_set_value = 1;
	} else {

		if (p->pdata->wakeup_va_ve_set_value > 1)
			p->pdata->wakeup_va_ve_set_value = 1;

		dev_info(p->dev,
		"%s: using wakeup_va_ve_set_value of %d from device-tree\n",
			 __func__, p->pdata->wakeup_va_ve_set_value);

	}

	ret = of_property_read_u32(np, "firmware_id_va_ve",
		&p->pdata->firmware_id_va_ve);
	if (ret && ret != -EINVAL) {
		dev_err(p->dev, "%s: invalid 'firmware_id_va_ve'\n", __func__);
		ret = -EINVAL;
		goto out_err;
	} else if (ret ==  -EINVAL) {
		dev_info(p->dev,
		"%s: no firmware_id_va_ve def. in device-tree. assuming d2\n",
			__func__);
		p->pdata->firmware_id_va_ve = DBMDX_FIRMWARE_ID_DBMD2;
	} else {
		dev_info(p->dev,
		"%s: using firmware_id_va_ve of 0x%8x from device-tree\n",
			 __func__, p->pdata->firmware_id_va_ve);
	}

	ret = of_property_read_u32(np, "boot_options_va_ve",
		&p->pdata->boot_options_va_ve);
	if (ret && ret != -EINVAL) {
		dev_err(p->dev, "%s: invalid 'boot_options_va_ve'\n", __func__);
		ret = -EINVAL;
		goto out_err;
	} else if (ret ==  -EINVAL) {
		dev_info(p->dev,
			"%s: no boot_options_va_ve def. in device-tree.\n",
			__func__);
		p->pdata->boot_options_va_ve = DBMDX_BOOT_MODE_NORMAL_BOOT;
	}

	ret = of_property_read_u32(np, "change_clock_src_options",
		&p->pdata->change_clock_src_enabled);
	if ((ret && ret != -EINVAL)) {
		dev_err(p->dev, "%s: invalid 'change_clock_src_enabled'\n",
			__func__);
		ret = -EINVAL;
		goto out_err;
	} else if (ret ==  -EINVAL) {
		dev_info(p->dev,
			"%s: no change_clock_src_enabled def. in dev-tree\n",
			__func__);
		p->pdata->change_clock_src_enabled =
					DBMDX_CHANGE_CLOCK_DISABLED;
	} else {

		dev_info(p->dev,
		"%s: using change_clock_src_enabled of: %0x02x from dev-tree\n",
			__func__, p->pdata->change_clock_src_enabled);
	}
	ret = of_property_read_u32(np, "va_ve_separate_clocks",
		&p->pdata->va_ve_separate_clocks);
	if ((ret && ret != -EINVAL)) {
		dev_err(p->dev, "%s: invalid 'va_ve_separate_clocks'\n",
			__func__);
		ret = -EINVAL;
		goto out_err;
	} else if (ret ==  -EINVAL) {
		dev_info(p->dev,
			"%s: no va_ve_separate_clocks def. in dev-tree\n",
			__func__);
		p->pdata->va_ve_separate_clocks = 0;
	} else {

		if (p->pdata->va_ve_separate_clocks > 1)
			p->pdata->va_ve_separate_clocks = 1;

		dev_info(p->dev,
		"%s: using va_ve_separate_clocks of: %0x02x from dev-tree\n",
			__func__, p->pdata->va_ve_separate_clocks);
	}

	ret = of_property_read_u32(np, "va_ve_mic_mask",
		&p->pdata->va_ve_mic_mask);
	if (ret && ret != -EINVAL) {
		dev_err(p->dev, "%s: invalid 'va_ve_mic_mask'\n", __func__);
		ret = -EINVAL;
		goto out_err;
	} else if (ret ==  -EINVAL) {
		dev_info(p->dev,
			"%s: no va_ve_mic_mask def. in device-tree.\n",
			__func__);
		p->pdata->va_ve_mic_mask = 0x0001;
	}
	ret = of_property_read_u32(np, "asrp_delay",
		&p->pdata->asrp_delay);
	if (ret && ret != -EINVAL) {
		dev_err(p->dev, "%s: invalid 'asrp_delay'\n", __func__);
		ret = -EINVAL;
		goto out_err;
	} else if (ret ==  -EINVAL) {
		dev_info(p->dev,
			"%s: no asrp_delay definition in device-tree.\n",
			__func__);
		p->pdata->asrp_delay = 0;
	} else {
		dev_info(p->dev,
		"%s: using asrp_delay of 0x%x from device-tree\n",
			 __func__, p->pdata->asrp_delay);
	}

	ret = of_property_read_u32(np, "asrp_tx_out_gain",
		&p->pdata->asrp_tx_out_gain);
	if (ret && ret != -EINVAL) {
		dev_err(p->dev, "%s: invalid 'asrp_tx_out_gain'\n", __func__);
		ret = -EINVAL;
		goto out_err;
	} else if (ret ==  -EINVAL) {
		dev_info(p->dev,
			"%s: no asrp_tx_out_gain definition in device-tree.\n",
			__func__);
		p->pdata->asrp_tx_out_gain = 0;
	} else {
		dev_info(p->dev,
		"%s: using asrp_tx_out_gain of 0x%x from device-tree\n",
			 __func__, p->pdata->asrp_tx_out_gain);
	}

	ret = of_property_read_u32(np, "asrp_vcpf_out_gain",
		&p->pdata->asrp_vcpf_out_gain);
	if (ret && ret != -EINVAL) {
		dev_err(p->dev, "%s: invalid 'asrp_vcpf_out_gain'\n", __func__);
		ret = -EINVAL;
		goto out_err;
	} else if (ret ==  -EINVAL) {
		dev_info(p->dev,
			"%s: no asrp_vcpf_out_gain def. in device-tree.\n",
			__func__);
		p->pdata->asrp_vcpf_out_gain = 0;
	} else {
		dev_info(p->dev,
		"%s: using asrp_vcpf_out_gain of 0x%x from device-tree\n",
			 __func__, p->pdata->asrp_vcpf_out_gain);
	}

	ret = of_property_read_u32(np, "asrp_rx_out_gain",
		&p->pdata->asrp_rx_out_gain);
	if (ret && ret != -EINVAL) {
		dev_err(p->dev, "%s: invalid 'asrp_rx_out_gain'\n", __func__);
		ret = -EINVAL;
		goto out_err;
	} else if (ret ==  -EINVAL) {
		dev_info(p->dev,
			"%s: no asrp_rx_out_gain definition in device-tree.\n",
			__func__);
		p->pdata->asrp_rx_out_gain = 0;
	} else {
		dev_info(p->dev,
		"%s: using asrp_rx_out_gain of 0x%x from device-tree\n",
			 __func__, p->pdata->asrp_rx_out_gain);
	}
	ret = of_property_read_string(np,
				      "default-streaming-usecase-name",
				      &pdata->default_streaming_usecase);
	if (ret != 0) {
		/* set default name */
		pdata->default_streaming_usecase = NULL;
		dev_info(dev, "%s: No default streaming usecase was set\n",
			 __func__);
	} else
		dev_info(dev,
			"%s: using device-tree default streaming usecase: %s\n",
			__func__, pdata->default_streaming_usecase);

	ret = of_property_read_u32(np, "default_va_clock",
		&p->pdata->default_va_clock);
	if (ret && ret != -EINVAL) {
		dev_err(p->dev, "%s: invalid 'default_va_clock'\n", __func__);
		ret = -EINVAL;
		goto out_err;
	} else if (ret ==  -EINVAL) {
		dev_info(p->dev,
			"%s: no default_va_clock def. in device-tree.\n",
			__func__);
		p->pdata->default_va_clock = DEFAULT_D4_CLOCK_HZ;
		dev_info(p->dev,
			"%s: using default_va_clock of %u Hz (default)\n",
			 __func__, p->pdata->default_va_clock);

	} else {
		if (p->pdata->default_va_clock == 0)
			p->pdata->default_va_clock = DEFAULT_D4_CLOCK_HZ;
		dev_info(p->dev,
		"%s: using default_va_clock of %u Hz from device-tree\n",
			 __func__, p->pdata->default_va_clock);
	}

	ret = of_property_read_u32(np, "default_va_ve_clock",
		&p->pdata->default_va_ve_clock);
	if (ret && ret != -EINVAL) {
		dev_err(p->dev, "%s: invalid 'default_va_ve_clock'\n",
			__func__);
		ret = -EINVAL;
		goto out_err;
	} else if (ret ==  -EINVAL) {
		dev_info(p->dev,
			"%s: no default_va_ve_clock def. in device-tree.\n",
			__func__);
		p->pdata->default_va_ve_clock = DEFAULT_D2_CLOCK_HZ;
		dev_info(p->dev,
			"%s: using default_va_ve_clock of %u Hz (default)\n",
			 __func__, p->pdata->default_va_ve_clock);

	} else {
		if (p->pdata->default_va_ve_clock == 0)
			p->pdata->default_va_ve_clock = DEFAULT_D2_CLOCK_HZ;
		dev_info(p->dev,
		"%s: using default_va_ve_clock of %u Hz from device-tree\n",
			 __func__, p->pdata->default_va_ve_clock);
	}

	ret = of_property_read_u32(np, "alsa_streaming_options",
		&p->pdata->alsa_streaming_options);
	if (ret && ret != -EINVAL) {
		dev_err(p->dev, "%s: invalid 'alsa_streaming_options'\n",
			__func__);
		ret = -EINVAL;
		goto out_err;
	} else if (ret ==  -EINVAL) {
		dev_info(p->dev,
			"%s: no alsa_streaming_options def. in device-tree.\n",
			__func__);
		p->pdata->alsa_streaming_options =
					DBMDX_ALSA_STREAMING_DEFAULT_OPTIONS;
		dev_info(p->dev,
		"%s: using default alsa_streaming_options of 0x%x\n",
			 __func__, p->pdata->alsa_streaming_options);

	} else {
		dev_info(p->dev,
		"%s: using alsa_streaming_options of 0x%x from device-tree\n",
			 __func__, p->pdata->alsa_streaming_options);
	}

	ret = of_property_read_u32(np, "disable_recovery",
		&p->pdata->va_recovery_disabled);
	if ((ret && ret != -EINVAL) ||
			(p->pdata->va_recovery_disabled != 0 &&
				p->pdata->va_recovery_disabled != 1)) {
		dev_err(p->dev, "%s: invalid 'va_recovery_disabled'\n",
			__func__);
		ret = -EINVAL;
		goto out_err;
	}

	ret = of_property_read_u32(np, "low_power_mode_disabled",
		&p->pdata->low_power_mode_disabled);
	if ((ret && ret != -EINVAL) ||
			(p->pdata->low_power_mode_disabled != 0 &&
				p->pdata->low_power_mode_disabled != 1)) {
		dev_err(p->dev, "%s: invalid 'low_power_mode_disabled'\n",
			__func__);
		ret = -EINVAL;
		goto out_err;
	}

	ret = of_property_read_u32(np, "uart_low_speed_enabled",
		&p->pdata->uart_low_speed_enabled);
	if ((ret && ret != -EINVAL) ||
			(p->pdata->uart_low_speed_enabled != 0 &&
				p->pdata->uart_low_speed_enabled != 1)) {
		dev_err(p->dev, "%s: invalid 'uart_low_speed_enabled'\n",
			__func__);
		ret = -EINVAL;
		goto out_err;
	}

	if (dbmdx_of_get_clk_info(p, np, DBMDX_CLK_MASTER)) {
		dev_err(dev,
			"%s: failed to get master clock information\n",
			 __func__);
	}

	if (dbmdx_of_get_clk_info(p, np, DBMDX_CLK_CONSTANT)) {
		dev_err(dev,
			"%s: failed to get constant clock information\n",
			 __func__);
	}
	if (p->pdata->va_ve_separate_clocks)
		if (dbmdx_of_get_clk_info(p, np, DBMDX_CLK_MASTER_VA_VE))
			dev_err(dev,
			"%s: failed to get master (VA_VE) clock information\n",
			 __func__);
	return 0;
out_err:
	return ret;
}

static int dbmdx_find_chip_interface(struct device_node *np,
			struct chip_interface **chip,
			enum dbmdx_bus_interface *active_interface)
{
	struct spi_device *spi_dev;
	struct i2c_client *i2c_client;
	struct platform_device *uart_client;
	struct chip_interface *c = NULL;

	*active_interface = DBMDX_INTERFACE_NONE;

	i2c_client = of_find_i2c_device_by_node(np);
	if (i2c_client) {
		/* got I2C command interface */
		c = i2c_get_clientdata(i2c_client);
		if (!c)
			return -EPROBE_DEFER;

		*active_interface = DBMDX_INTERFACE_I2C;
	}

	uart_client = of_find_platform_device_by_node(np);
	if (uart_client) {
		/* got UART command interface */
		c = dev_get_drvdata(&uart_client->dev);
		if (!c)
			return -EPROBE_DEFER;
		*active_interface = DBMDX_INTERFACE_UART;
	}

	spi_dev = of_find_spi_device_by_node(np);
	if (spi_dev) {
		/* got spi command interface */
		c = spi_get_drvdata(spi_dev);
		if (!c)
			return -EPROBE_DEFER;

		*active_interface = DBMDX_INTERFACE_SPI;
	}

	*chip = c;

	return c ? 0 : -EINVAL;
}

static int dbmdx_get_fw_interfaces(struct dbmdx_private *p,
				   const char *tag,
				   int *iarray)
{
	struct device_node *np = p->dev->of_node;
	struct property *property;
	int ret, i, nr_interfaces = 0;

	for (i = 0; i < DBMDX_MAX_INTERFACES; i++)
		iarray[i] = -1;

	/* If multiinterface is not supported just set all interfaces to 0 */
	if (!p->pdata->multi_interface_support) {
		for (i = 0; i < DBMDX_MAX_INTERFACES; i++)
			iarray[i] = 0;
		return 0;
	}

	property = of_find_property(np, tag, &nr_interfaces);
	if (!property) {
		dev_err(p->dev,
			"%s: no valid %s entry in devicetree\n",
			__func__, tag);
		return -EINVAL;
	}
	nr_interfaces /= sizeof(u32);
	if (nr_interfaces > DBMDX_MAX_INTERFACES ||
	    nr_interfaces == 0) {
		dev_err(p->dev,
			"%s: %s min entries is %d, max is %d\n",
			__func__, tag, 1, DBMDX_MAX_INTERFACES);
		return -EINVAL;
	}

	ret = of_property_read_u32_array(np, tag, iarray, nr_interfaces);
	if (ret) {
		dev_err(p->dev,
			"%s: could not read %s\n", __func__, tag);
		return -EIO;
	}

	dev_info(p->dev,
		"%s: %s uses %d interfaces from device-tree\n",
		__func__, tag, nr_interfaces);

	for (i = 0; i < DBMDX_MAX_INTERFACES; i++) {
		/* make sure all interfaces have a valid index */
		if (iarray[i] == -1)
			iarray[i] = iarray[0];
		dev_dbg(p->dev, "%s: interface %2.2x: 0x%2.2x\n",
			__func__, i, iarray[i]);
	}

	return 0;
}

static int dbmdx_interface_probe_single(struct dbmdx_private *p)
{
	int ret = 0;
	struct device_node *np = p->dev->of_node;
	struct device_node *interface_np;
	struct chip_interface *chip;
	enum dbmdx_bus_interface active_interface = DBMDX_INTERFACE_NONE;

	interface_np = of_parse_phandle(np, "cmd-interface", 0);
	if (!interface_np) {
		dev_err(p->dev, "%s: invalid command interface node\n",
			__func__);
		ret = -EINVAL;
		goto out;
	}

	ret = dbmdx_find_chip_interface(interface_np, &chip, &active_interface);
	if (ret == -EPROBE_DEFER)
		goto out;
	if (ret != 0) {
		dev_err(p->dev, "%s: invalid interface phandle\n", __func__);
		goto out;
	}

	p->nr_of_interfaces = 1;

	p->interfaces = kzalloc(sizeof(struct chip_interface *), GFP_KERNEL);

	if (!(p->interfaces)) {
		dev_err(p->dev, "%s: no memory for interfaces\n", __func__);
		goto out;
	}

	p->interface_types = kzalloc(sizeof(enum dbmdx_bus_interface),
				GFP_KERNEL);

	if (!(p->interface_types)) {
		dev_err(p->dev, "%s: no memory for interface types\n",
			__func__);
		goto out;
	}

	p->interfaces[0] = chip;
	p->interface_types[0] = active_interface;

	/* set chip interface */
	p->chip = chip;

	p->active_interface = active_interface;

	return 0;
out:
	kfree(p->interfaces);
	kfree(p->interface_types);
	return ret;
}

static int dbmdx_interface_probe_multi(struct dbmdx_private *p)
{
	int ret = 0;
	unsigned int nr_interfaces = 0;
	int i = 0;
	struct device_node *np = p->dev->of_node;
	struct device_node *interface_np;
	struct chip_interface *chip;
	struct chip_interface **interfaces = NULL;
	enum dbmdx_bus_interface	*interface_types = NULL;
	enum dbmdx_bus_interface	*new_interface_types;
	struct chip_interface **new_interfaces;
	enum dbmdx_bus_interface active_interface = DBMDX_INTERFACE_NONE;

	do {
		interface_np = of_parse_phandle(np, "cd-interfaces", i++);
		if (!interface_np)
			continue;

		ret = dbmdx_find_chip_interface(interface_np, &chip,
							&active_interface);
		if (ret == -EPROBE_DEFER)
			goto out;
		if (ret != 0) {
			dev_err(p->dev, "%s: invalid interface phandle\n",
				__func__);
			goto out;
		}

		new_interfaces = krealloc(interfaces,
					  sizeof(struct chip_interface *) *
					  (nr_interfaces + 1),
					  GFP_KERNEL);
		if (!new_interfaces) {
			dev_err(p->dev, "%s: no memory for interfaces\n",
				__func__);
			goto out;
		}

		new_interface_types = krealloc(interface_types,
					  sizeof(enum dbmdx_bus_interface) *
					  (nr_interfaces + 1),
					  GFP_KERNEL);
		if (!new_interface_types) {
			dev_err(p->dev, "%s: no memory for interface types\n",
				__func__);
			goto out;
		}

		interfaces = new_interfaces;
		interfaces[nr_interfaces] = chip;
		interface_types = new_interface_types;
		interface_types[nr_interfaces] = active_interface;
		nr_interfaces++;

	} while (interface_np);

	if (!nr_interfaces) {
		dev_err(p->dev, "%s: invalid nr of interfaces\n",
			__func__);
		ret = -EINVAL;
		goto out_free_interfaces;
	}

	p->nr_of_interfaces = nr_interfaces;
	p->interfaces = interfaces;
	p->interface_types = interface_types;

	dev_info(p->dev, "%s: found %u interfaces\n", __func__, nr_interfaces);


	return 0;
out_free_interfaces:
	kfree(interfaces);
	kfree(interface_types);
out:
	return ret;
}

static int dbmdx_interface_probe(struct dbmdx_private *p)
{
	/* check for features */
	if (p->pdata->multi_interface_support)
		return dbmdx_interface_probe_multi(p);

	return dbmdx_interface_probe_single(p);
}

#else
static int dbmdx_name_match(struct device *dev, void *dev_name)
{
	struct platform_device *pdev = to_platform_device(dev);

	if (!pdev || !pdev->name)
		return 0;

	return !strcmp(pdev->name, dev_name);
}

static int dbmdx_spi_name_match(struct device *dev, void *dev_name)
{
	struct spi_device *spi_dev = to_spi_device(dev);

	if (!spi_dev || !spi_dev->modalias)
		return 0;

	return !strcmp(spi_dev->modalias, dev_name);
}

static int dbmdx_i2c_name_match(struct device *dev, void *dev_name)
{
	struct i2c_client *i2c_dev = to_i2c_client(dev);

	if (!i2c_dev || !i2c_dev->name)
		return 0;

	return !strcmp(i2c_dev->name, dev_name);
}

struct i2c_client *dbmdx_find_i2c_device_by_name(const char *dev_name)
{
	struct device *dev;

	dev = bus_find_device(&i2c_bus_type, NULL, (void *)dev_name,
					 dbmdx_i2c_name_match);

	return dev ? to_i2c_client(dev) : NULL;
}

struct spi_device *dbmdx_find_spi_device_by_name(const char *dev_name)
{
	struct device *dev;

	dev = bus_find_device(&spi_bus_type, NULL,
				(void *)dev_name, dbmdx_spi_name_match);
	return dev ? to_spi_device(dev) : NULL;
}

struct platform_device *dbmdx_find_platform_device_by_name(const char *dev_name)
{
	struct device *dev;

	dev = bus_find_device(&platform_bus_type, NULL,
				(void *)dev_name, dbmdx_name_match);
	return dev ? to_platform_device(dev) : NULL;
}


static int dbmdx_platform_get_clk_info(struct dbmdx_private *p,
				 enum dbmdx_clocks dbmdx_clk)
{
	int rate, rrate;
	struct clk *clk;
	struct dbmdx_platform_data *pdata = p->pdata;

	rate = pdata->clock_rates[dbmdx_clk];
	dev_info(p->dev,
		 "%s: using %s at %dHZ\n",
		 __func__,
		 dbmdx_of_clk_names[dbmdx_clk],
		 rate);

	clk = clk_get(p->dev, dbmdx_of_clk_names[dbmdx_clk]);
	if (IS_ERR(clk)) {
		dev_info(p->dev,
			 "%s: no %s definition\n",
			 __func__,
			 dbmdx_of_clk_names[dbmdx_clk]);
		/* nothing in the device tree */
		clk = NULL;
	} else {
		/* If clock rate not specified in dts, try to detect */
		if (rate == -1) {
			rate = clk_get_rate(clk);
			dev_info(p->dev,
				 "%s: using %s at %dHZ\n",
				 __func__,
				 dbmdx_of_clk_names[dbmdx_clk],
				 rate);
		} else {
			/* verify which rate can be set */
			rrate = clk_round_rate(clk, rate);
			if (rrate !=  rate) {
				dev_info(p->dev,
					 "%s: rounded rate %d to %d\n",
					 __func__,
					 rate, rrate);
				rate = rrate;
			}
		}
	}
	p->clocks[dbmdx_clk] = clk;
	p->pdata->clock_rates[dbmdx_clk] = rate;

	return 0;
}


static int verify_platform_data(struct device *dev,
		struct dbmdx_private *p)
{
	struct dbmdx_platform_data *pdata = p->pdata;
	int i;

	/* check for features */
	if (pdata->feature_va) {
		dev_info(dev, "%s: VA feature activated\n", __func__);
		p->va_chip_enabled = true;
	} else
		dev_info(dev, "%s: VA feature not activated\n", __func__);
	if (pdata->feature_va_ve) {
		dev_info(dev, "%s: VA_VE feature activated\n", __func__);

		if (pdata->project_sub_type ==
			DBMDX_VA_VE_PROJECT_SUB_TYPE_VT_ASRP) {
			/* MANGO */
			p->va_chip_enabled = true;
			p->va_ve_chip_enabled = true;
		} else if (pdata->project_sub_type ==
			DBMDX_VA_VE_PROJECT_SUB_TYPE_VT_ONLY) {
			/* Melon */
			p->va_chip_enabled = true;
			p->va_ve_chip_enabled = false;

		} else if (pdata->project_sub_type ==
			/* ORANGE */
			DBMDX_VA_VE_PROJECT_SUB_TYPE_ASRP_ONLY) {
			p->va_chip_enabled = false;
			p->va_ve_chip_enabled = true;

		} else {
			dev_info(dev, "%s: Unknown project type %u\n", __func__,
				pdata->project_sub_type);
			p->va_chip_enabled = true;
			p->va_ve_chip_enabled = true;
		}

	} else
		dev_info(dev, "%s: VA_VE feature not activated\n", __func__);
/* check if enabled features make sense */
	if (!pdata->feature_va && !pdata->feature_va_ve) {
		dev_err(dev, "%s: No feature activated\n", __func__);
		return -EINVAL;
	}
	if (pdata->feature_va && pdata->feature_va_ve) {
		dev_err(dev,
			"%s: Both VA and VA_VE features are activated\n",
			__func__);
		return -EINVAL;
	}
	if (pdata->multi_interface_support > 1)
		pdata->multi_interface_support = 1;

	if (pdata->multi_interface_support)
		dev_info(dev, "%s: Multi Interface Probe is supported\n",
			__func__);
	else
		dev_info(dev, "%s: Multi Interface Probe is not supported\n",
			__func__);

	if ((pdata->feature_va) ||
		(pdata->feature_va_ve && p->va_chip_enabled)) {
		dev_info(dev, "%s: VA firmware name: %s\n",
				 __func__, pdata->va_firmware_name);

		dev_info(dev, "%s: VA preboot firmware name: %s\n",
				 __func__, pdata->va_preboot_firmware_name);

		if (pdata->auto_buffering != 0 &&
		    pdata->auto_buffering != 1)
			pdata->auto_buffering = 0;

		dev_info(dev, "%s: using auto_buffering of %d\n",
			 __func__, pdata->auto_buffering);

		if (pdata->auto_detection != 0 &&
			pdata->auto_detection != 1)
			pdata->auto_detection = 1;

		dev_info(dev, "%s: using auto_detection of %d\n",
			 __func__, pdata->auto_detection);

		if (pdata->detection_after_buffering < 0 ||
			pdata->detection_after_buffering >=
				DETECTION_AFTER_BUFFERING_MODE_MAX)
			pdata->detection_after_buffering =
					DETECTION_AFTER_BUFFERING_OFF;

		dev_info(dev, "%s: using detection_after_buffering of %d\n",
			 __func__, pdata->detection_after_buffering);

		if (pdata->send_uevent_on_detection != 0 &&
			pdata->send_uevent_on_detection != 1)
			pdata->send_uevent_on_detection = 0;

		dev_info(dev, "%s: using send_uevent_on_detection of %d\n",
			__func__, pdata->send_uevent_on_detection);

		if (pdata->send_uevent_after_buffering != 0 &&
			pdata->send_uevent_after_buffering != 1)
			pdata->send_uevent_after_buffering = 0;

		dev_info(dev, "%s: using send_uevent_after_buffering of %d\n",
			__func__, pdata->send_uevent_after_buffering);

		if (p->pdata->buffering_timeout < MIN_RETRIES_TO_WRITE_TOBUF)
			p->pdata->buffering_timeout =
					MIN_RETRIES_TO_WRITE_TOBUF;

		dev_info(p->dev,
			"%s: using buffering_timeout of %u\n",
			 __func__, p->pdata->buffering_timeout);

		if (pdata->detection_buffer_channels < 0 ||
		pdata->detection_buffer_channels > MAX_SUPPORTED_CHANNELS)
			pdata->detection_buffer_channels = 0;

		dev_info(p->dev, "%s: using detection_buffer_channels of %d\n",
			 __func__, pdata->detection_buffer_channels);

		dev_info(p->dev, "%s: using min_samples_chunk_size of %d\n",
			 __func__, pdata->min_samples_chunk_size);

		dev_info(p->dev, "%s: using max_detection_buffer_size of %d\n",
			 __func__, pdata->max_detection_buffer_size);

		if (pdata->va_buffering_pcm_rate != 16000 &&
			pdata->va_buffering_pcm_rate != 32000)
			pdata->va_buffering_pcm_rate = 16000;

		dev_info(p->dev, "%s: using va_buffering_pcm_rate of %u\n",
			__func__, pdata->va_buffering_pcm_rate);

		for (i = 0; i < pdata->va_cfg_values; i++)
			dev_dbg(dev, "%s: VA cfg %8.8x: 0x%8.8x\n",
					__func__, i, pdata->va_cfg_value[i]);

		for (i = 0; i < VA_MIC_CONFIG_SIZE; i++)
			dev_dbg(dev, "%s: VA mic cfg %8.8x: 0x%8.8x\n",
				__func__, i, pdata->va_mic_config[i]);

		dev_info(dev, "%s: VA default mic config: 0x%8.8x\n",
			 __func__, pdata->va_initial_mic_config);

		for (i = 0; i < 3; i++)
			dev_dbg(dev, "%s: VA mic gain cfg %i: 0x%04X\n",
				__func__, i, pdata->va_mic_gain_config[i]);

		if (pdata->va_backlog_length > 0xfff)
			pdata->va_backlog_length = 0xfff;

#ifdef DMBDX_GENERAL_AMODEL_SUPPORT
		dev_info(dev, "%s: using backlog length of %d\n",
			 __func__, pdata->va_backlog_length);
#endif
#ifdef DMBDX_SENSORY_AMODEL_SUPPORT
		if (pdata->va_backlog_length_sensory > 0xfff)
			pdata->va_backlog_length_sensory = 0xfff;

		dev_info(dev, "%s: using Sensory backlog length of %d\n",
			 __func__, pdata->va_backlog_length_sensory);
#endif
#ifdef DMBDX_OKG_AMODEL_SUPPORT
		if (pdata->va_backlog_length_google > 0xfff)
			pdata->va_backlog_length_google = 0xfff;

		dev_info(dev, "%s: using OKG backlog length of %d\n",
			 __func__, pdata->va_backlog_length_sensory);
#endif
#ifdef DMBDX_ALEXA_AMODEL_SUPPORT
		if (pdata->va_backlog_length_alexa > 0xfff)
			pdata->va_backlog_length_alexa = 0xfff;

		dev_info(dev, "%s: using ALEXA backlog length of %d\n",
			 __func__, pdata->va_backlog_length_alexa);
#endif
#ifdef DMBDX_BIXBY_AMODEL_SUPPORT
		if (pdata->va_backlog_length_bixby > 0xfff)
			pdata->va_backlog_length_bixby = 0xfff;

		dev_info(dev, "%s: using Bixby backlog length of %d\n",
			 __func__, pdata->va_backlog_length_bixby);
#endif

		if (pdata->pcm_streaming_mode != 0 &&
			pdata->pcm_streaming_mode != 1)
			pdata->pcm_streaming_mode = 0;

		dev_info(dev, "%s: using pcm_streaming_mode of %d\n",
			 __func__, pdata->pcm_streaming_mode);

		dev_dbg(p->dev, "va-interfaces:\n");

		for (i = 0; i < DBMDX_MAX_INTERFACES; i++)
			dev_dbg(p->dev, "%s: interface %d: %d\n",
				__func__, i, p->pdata->va_interfaces[i]);
	}

	if (pdata->feature_va_ve) {
		dev_info(dev, "%s: VA_VE firmware name: %s\n",
				 __func__, pdata->va_ve_firmware_name);

		dev_info(dev, "%s: VA_VE preboot firmware name: %s\n",
				 __func__, pdata->va_ve_preboot_firmware_name);

		for (i = 0; i < pdata->va_ve_cfg_values; i++)
				dev_dbg(dev, "%s: VA_VE cfg %8.8x: 0x%8.8x\n",
					__func__, i, pdata->va_ve_cfg_value[i]);

		for (i = 0; i < VA_VE_MIC_CONFIG_SIZE; i++)
			dev_dbg(dev, "%s: VA_VE mic cfg %8.8x: 0x%8.8x\n",
				__func__, i, pdata->va_ve_mic_config[i]);

		dev_dbg(p->dev, "va_ve-interfaces:\n");

		for (i = 0; i < DBMDX_MAX_INTERFACES; i++)
			dev_dbg(p->dev, "%s: interface %d: %d\n",
				__func__, i, p->pdata->va_ve_interfaces[i]);
	}

	for (i = 0; i < DBMDX_VA_NR_OF_SPEEDS; i++)
		dev_dbg(dev, "%s: VA speed cfg %8.8x: 0x%8.8x %u %u %u\n",
			__func__,
			i,
			pdata->va_speed_cfg[i].cfg,
			pdata->va_speed_cfg[i].uart_baud,
			pdata->va_speed_cfg[i].i2c_rate,
			pdata->va_speed_cfg[i].spi_rate);

	dev_info(dev, "%s: using hw revision of 0x%8x\n",
		 __func__, pdata->hw_rev);

	if (pdata->wakeup_disabled > 1)
		pdata->wakeup_disabled = 1;

	dev_info(dev, "%s: using wakeup_disabled of %d\n",
		 __func__, pdata->wakeup_disabled);


	if (pdata->use_gpio_for_wakeup > 1)
		pdata->use_gpio_for_wakeup = 1;

	dev_info(dev, "%s: using use_gpio_for_wakeup of %d\n",
		 __func__, pdata->use_gpio_for_wakeup);

	if (pdata->send_wakeup_seq > 1)
		pdata->send_wakeup_seq = 1;

	dev_info(dev, "%s: using send_wakeup_seq of %d\n",
		 __func__, pdata->send_wakeup_seq);

	if (pdata->wakeup_set_value > 1)
		pdata->wakeup_set_value = 1;

	dev_info(dev, "%s: using wakeup_set_value of %d\n",
		 __func__, pdata->wakeup_set_value);

	dev_info(dev, "%s: using firmware_id of 0x%8x\n",
		 __func__, pdata->firmware_id);

	dev_info(dev, "%s: using boot_options of 0x%8x\n",
		 __func__, pdata->boot_options);

	dev_info(dev, "%s: using amodel_options of 0x%8x\n",
		 __func__, pdata->amodel_options);

	if (pdata->wakeup_va_ve_disabled > 1)
		pdata->wakeup_va_ve_disabled = 1;

	dev_info(dev, "%s: using wakeup_va_ve_disabled of %d\n",
		 __func__, pdata->wakeup_va_ve_disabled);

	if (pdata->use_gpio_for_wakeup_va_ve > 1)
		pdata->use_gpio_for_wakeup_va_ve = 1;

	dev_info(dev, "%s: using use_gpio_for_wakeup_va_ve of %d\n",
		 __func__, pdata->use_gpio_for_wakeup_va_ve);

	if (pdata->send_wakeup_va_ve_seq > 1)
		pdata->send_wakeup_va_ve_seq = 1;

	dev_info(dev, "%s: using send_wakeup_va_ve_seq of %d\n",
		 __func__, pdata->send_wakeup_va_ve_seq);

	if (pdata->wakeup_va_ve_set_value > 1)
		pdata->wakeup_va_ve_set_value = 1;

	dev_info(dev, "%s: using wakeup_va_ve_set_value of %d\n",
		 __func__, pdata->wakeup_va_ve_set_value);

	dev_info(dev, "%s: using firmware_id_va_ve of 0x%8x\n",
		 __func__, pdata->firmware_id_va_ve);

	dev_info(dev, "%s: using boot_options_va_ve of 0x%8x\n",
		 __func__, pdata->boot_options_va_ve);

	pdata->change_clock_src_enabled &= DBMDX_CHANGE_CLOCK_MASK;

	dev_info(dev, "%s: using change_clock_src_enabled of: %0x02x\n",
		__func__, p->pdata->change_clock_src_enabled);

	if (pdata->va_ve_separate_clocks > 1)
		pdata->va_ve_separate_clocks = 1;

	dev_info(dev, "%s: using va_ve_separate_clocks of %d\n",
		 __func__, pdata->va_ve_separate_clocks);

	dev_info(dev, "%s: using va_ve_mic_mask of 0x%8x\n",
		 __func__, pdata->va_ve_mic_mask);

	dev_info(dev, "%s: using asrp_delay of 0x%8x\n",
		 __func__, pdata->asrp_delay);

	dev_info(dev, "%s: using asrp_tx_out_gain of 0x%8x\n",
		 __func__, pdata->asrp_tx_out_gain);

	dev_info(dev, "%s: using asrp_vcpf_out_gain of 0x%8x\n",
		 __func__, pdata->asrp_vcpf_out_gain);

	dev_info(dev, "%s: using asrp_rx_out_gain of 0x%8x\n",
		 __func__, pdata->asrp_rx_out_gain);

	if (pdata->default_va_clock == 0)
		p->pdata->default_va_clock = DEFAULT_D4_CLOCK_HZ;

	dev_info(p->dev, "%s: using default_va_clock of %u Hz\n",
			 __func__, pdata->default_va_clock);

	if (pdata->default_va_ve_clock == 0)
		p->pdata->default_va_ve_clock = DEFAULT_D2_CLOCK_HZ;

	dev_info(p->dev, "%s: using default_va_ve_clock of %u Hz\n",
			 __func__, pdata->default_va_ve_clock);

	dev_info(p->dev, "%s: using alsa_streaming_options of 0x%x\n",
			 __func__, pdata->alsa_streaming_options);

	if (pdata->va_recovery_disabled != 0 &&
		pdata->va_recovery_disabled != 1)
		pdata->va_recovery_disabled = 0;

	dev_info(dev,
		"%s: using va_recovery_disabled of %d\n",
		 __func__, pdata->va_recovery_disabled);

	if (pdata->low_power_mode_disabled != 0 &&
		pdata->low_power_mode_disabled != 1)
		pdata->low_power_mode_disabled = 0;

	dev_info(dev,
		"%s: using low_power_mode_disabled of %d\n",
		 __func__, pdata->low_power_mode_disabled);

	if (pdata->uart_low_speed_enabled != 0 &&
		pdata->uart_low_speed_enabled != 1)
		pdata->uart_low_speed_enabled = 0;

	dev_info(p->dev,
		"%s: using uart_low_speed_enabled of %d\n",
		 __func__, pdata->uart_low_speed_enabled);

	dev_info(p->dev,
		"%s: using min_samples_chunk_size of %d\n",
		 __func__, pdata->min_samples_chunk_size);

	dev_info(p->dev,
		"%s: using max_detection_buffer_size of %d\n",
		 __func__, pdata->max_detection_buffer_size);


	if (dbmdx_platform_get_clk_info(p, DBMDX_CLK_MASTER)) {
		dev_err(dev,
			"%s: failed to get master clock information\n",
			 __func__);
	}

	if (pdata->va_ve_separate_clocks)
		if (dbmdx_platform_get_clk_info(p, DBMDX_CLK_MASTER_VA_VE))
			dev_err(dev,
			"%s: failed to get master_va_ve clock information\n",
				__func__);

	if (dbmdx_platform_get_clk_info(p, DBMDX_CLK_CONSTANT)) {
		dev_err(dev,
			"%s: failed to get constant clock information\n",
			 __func__);
	}


	return 0;
}

static int dbmdx_find_chip_interface(struct chip_interface **chip,
				enum dbmdx_bus_interface *active_interface)
{
	struct spi_device *spi_dev;
	struct i2c_client *i2c_client;
	struct platform_device *uart_client;
	struct chip_interface *c = NULL;

	*active_interface = DBMDX_INTERFACE_NONE;

	i2c_client = dbmdx_find_i2c_device_by_name("dbmdx-i2c");

	if (!i2c_client)
		i2c_client = dbmdx_find_i2c_device_by_name("dbmd_4_6-i2c");

	if (!i2c_client)
		i2c_client = dbmdx_find_i2c_device_by_name("dbmd6-i2c");

	if (!i2c_client)
		i2c_client = dbmdx_find_i2c_device_by_name("dbmd4-i2c");

	if (!i2c_client)
		i2c_client = dbmdx_find_i2c_device_by_name("dbmd2-i2c");

	if (i2c_client) {
		/* got I2C command interface */
		c = i2c_get_clientdata(i2c_client);
		if (!c)
			return -EPROBE_DEFER;

		*active_interface = DBMDX_INTERFACE_I2C;
	}

	uart_client = dbmdx_find_platform_device_by_name("dbmdx-uart");

	if (!uart_client)
		uart_client =
			dbmdx_find_platform_device_by_name("dbmd_4_6-uart");

	if (!uart_client)
		uart_client = dbmdx_find_platform_device_by_name("dbmd6-uart");

	if (!uart_client)
		uart_client = dbmdx_find_platform_device_by_name("dbmd4-uart");

	if (!uart_client)
		uart_client = dbmdx_find_platform_device_by_name("dbmd2-uart");

	if (uart_client) {
		/* got UART command interface */
		c = dev_get_drvdata(&uart_client->dev);
		if (!c)
			return -EPROBE_DEFER;
		*active_interface = DBMDX_INTERFACE_UART;
	}

	spi_dev = dbmdx_find_spi_device_by_name("dbmdx-spi");

	if (!spi_dev)
		spi_dev = dbmdx_find_spi_device_by_name("dbmd_4_6-spi");

	if (!spi_dev)
		spi_dev = dbmdx_find_spi_device_by_name("dbmd6-spi");

	if (!spi_dev)
		spi_dev = dbmdx_find_spi_device_by_name("dbmd4-spi");

	if (!spi_dev)
		spi_dev = dbmdx_find_spi_device_by_name("dbmd2-spi");

	if (spi_dev) {
		/* got spi command interface */
		dev_info(&spi_dev->dev, "%s: spi interface node %p\n",
				__func__, spi_dev);

		/* got spi command interface */
		c = spi_get_drvdata(spi_dev);
		if (!c)
			return -EPROBE_DEFER;

		*active_interface = DBMDX_INTERFACE_SPI;
	}

	*chip = c;

	return c ? 0 : -EINVAL;
}
static int dbmdx_find_chip_interface_by_name(const char *iface_name,
				struct chip_interface **chip,
				enum dbmdx_bus_interface *active_interface)
{
	struct spi_device *spi_dev;
	struct i2c_client *i2c_client;
	struct platform_device *uart_client;
	struct chip_interface *c = NULL;

	*active_interface = DBMDX_INTERFACE_NONE;

	i2c_client = dbmdx_find_i2c_device_by_name(iface_name);

	if (i2c_client) {
		/* got I2C command interface */
		c = i2c_get_clientdata(i2c_client);
		if (!c)
			return -EPROBE_DEFER;

		*active_interface = DBMDX_INTERFACE_I2C;
		goto out;
	}

	uart_client = dbmdx_find_platform_device_by_name(iface_name);

	if (uart_client) {
		/* got UART command interface */
		c = dev_get_drvdata(&uart_client->dev);
		if (!c)
			return -EPROBE_DEFER;
		*active_interface = DBMDX_INTERFACE_UART;
		goto out;
	}

	spi_dev = dbmdx_find_spi_device_by_name(iface_name);

	if (spi_dev) {
		/* got spi command interface */
		dev_info(&spi_dev->dev, "%s: spi interface node %p\n",
				__func__, spi_dev);

		/* got spi command interface */
		c = spi_get_drvdata(spi_dev);
		if (!c)
			return -EPROBE_DEFER;

		*active_interface = DBMDX_INTERFACE_SPI;
		goto out;
	}
out:
	*chip = c;

	return c ? 0 : -EINVAL;
}

static int dbmdx_interface_probe_single(struct dbmdx_private *p)
{
	int ret = 0;
	struct chip_interface *chip;
	enum dbmdx_bus_interface active_interface = DBMDX_INTERFACE_NONE;

	ret = dbmdx_find_chip_interface(&chip, &active_interface);
	if (ret == -EPROBE_DEFER)
		goto out;
	if (ret != 0) {
		dev_err(p->dev, "%s: invalid interface phandle\n", __func__);
		goto out;
	}

	p->nr_of_interfaces = 1;

	p->interfaces = kzalloc(sizeof(struct chip_interface *), GFP_KERNEL);

	if (!(p->interfaces)) {
		dev_err(p->dev, "%s: no memory for interfaces\n", __func__);
		goto out;
	}

	p->interface_types = kzalloc(sizeof(enum dbmdx_bus_interface),
				GFP_KERNEL);

	if (!(p->interface_types)) {
		dev_err(p->dev, "%s: no memory for interface types\n",
			__func__);
		goto out;
	}

	p->interfaces[0] = chip;
	p->interface_types[0] = active_interface;

	/* set chip interface */
	p->chip = chip;

	p->active_interface = active_interface;

	return 0;
out:
	kfree(p->interfaces);
	kfree(p->interface_types);
	return ret;
}

static int dbmdx_interface_probe_multi(struct dbmdx_private *p)
{
	int ret = 0;
	unsigned int nr_interfaces = 0;
	int interface_ind;
	struct chip_interface *chip;
	struct chip_interface **interfaces = NULL;
	enum dbmdx_bus_interface	*interface_types = NULL;
	enum dbmdx_bus_interface	*new_interface_types;
	struct chip_interface **new_interfaces;
	enum dbmdx_bus_interface active_interface = DBMDX_INTERFACE_NONE;

	if (!p->pdata->cd_interfaces) {
		dev_err(p->dev, "%s: invalid interfaces array\n", __func__);
		ret = -EINVAL;
		goto out;
	}

	for (interface_ind = 0; p->pdata->cd_interfaces[interface_ind];
		++interface_ind) {

		const char *interface_name =
				p->pdata->cd_interfaces[interface_ind];

		if (!interface_name)
			break;

		ret = dbmdx_find_chip_interface_by_name(interface_name, &chip,
							&active_interface);
		if (ret == -EPROBE_DEFER)
			goto out;
		if (ret != 0) {
			dev_err(p->dev, "%s: invalid interface phandle [%s]\n",
				__func__, interface_name);
			goto out;
		}

		new_interfaces = krealloc(interfaces,
					  sizeof(struct chip_interface *) *
					  (nr_interfaces + 1),
					  GFP_KERNEL);
		if (!new_interfaces) {
			dev_err(p->dev, "%s: no memory for interfaces\n",
				__func__);
			goto out;
		}

		new_interface_types = krealloc(interface_types,
					  sizeof(enum dbmdx_bus_interface) *
					  (nr_interfaces + 1),
					  GFP_KERNEL);
		if (!new_interface_types) {
			dev_err(p->dev, "%s: no memory for interface types\n",
				__func__);
			goto out;
		}

		interfaces = new_interfaces;
		interfaces[nr_interfaces] = chip;
		interface_types = new_interface_types;
		interface_types[nr_interfaces] = active_interface;
		nr_interfaces++;

	}

	if (!nr_interfaces) {
		dev_err(p->dev, "%s: invalid nr of interfaces\n",
			__func__);
		ret = -EINVAL;
		goto out_free_interfaces;
	}

	p->nr_of_interfaces = nr_interfaces;
	p->interfaces = interfaces;
	p->interface_types = interface_types;

	dev_info(p->dev, "%s: found %u interfaces\n", __func__, nr_interfaces);


	return 0;
out_free_interfaces:
	kfree(interfaces);
	kfree(interface_types);
out:
	return ret;
}


static int dbmdx_interface_probe(struct dbmdx_private *p)
{
	/* check for features */
	if (p->pdata->multi_interface_support)
		return dbmdx_interface_probe_multi(p);

	return dbmdx_interface_probe_single(p);
}
#endif

static int dbmdx_platform_probe(struct platform_device *pdev)
{
	struct dbmdx_platform_data *pdata;
	struct dbmdx_private *p;
	int ret = 0;

	dev_info(&pdev->dev, "%s: DBMDX codec driver version = %s\n",
		__func__, DRIVER_VERSION);

	p = kzalloc(sizeof(*p), GFP_KERNEL);
	if (p == NULL) {
		ret = -ENOMEM;
		goto out;
	}

	p->dev = &pdev->dev;


#ifdef CONFIG_OF
	pdata = kzalloc(sizeof(struct dbmdx_platform_data), GFP_KERNEL);
	if (!pdata) {
		ret = -ENOMEM;
		goto out_err_free_private;
	}

	p->pdata = pdata;

	ret = dbmdx_get_devtree_pdata(p->dev, p);
	if (ret) {
		dev_err(p->dev, "%s: failed to read device tree data\n",
			__func__);
		goto out_err_free_pdata;
	}
#else
	pdata = dev_get_platdata(p->dev);

	if (pdata == NULL) {
		ret = -ENODEV;
		dev_err(&pdev->dev, "%s: Failed to get platform data\n",
			__func__);
		goto out_err_free_private;
	}

	p->pdata = pdata;

	ret = verify_platform_data(p->dev, p);
	if (ret) {
		dev_err(p->dev, "%s: Failed to verify platform data\n",
			__func__);
		goto out_err_free_pdata;
	}
#endif

#ifndef CONFIG_SND_SOC_DBMDX
	dbmdx_init_interface();
#endif

	ret = dbmdx_interface_probe(p);
	if (ret) {
		ret = -EPROBE_DEFER;
		goto out_err_free_pdata;
	}


	p->dev = &pdev->dev;

	p->vregulator = devm_regulator_get(p->dev, "dbmdx_regulator");
	if (IS_ERR(p->vregulator)) {
		dev_info(p->dev, "%s: Can't get voltage regulator\n",
			__func__);
		p->vregulator = NULL;
	}

	p->sleep_disabled = pdata->low_power_mode_disabled;

	/* set initial mic as it appears in the platform data */
	p->va_current_mic_config = pdata->va_initial_mic_config;
	p->va_active_mic_config = pdata->va_initial_mic_config;

	p->va_cur_digital_mic_digital_gain =
		pdata->va_mic_gain_config[DBMDX_DIGITAL_MIC_DIGITAL_GAIN];
	p->va_cur_analog_mic_analog_gain =
		pdata->va_mic_gain_config[DBMDX_ANALOG_MIC_ANALOG_GAIN];
	/* analog mic gain is a sum of analog gain & digital gain*/
	p->va_cur_analog_mic_digital_gain =
	pdata->va_mic_gain_config[DBMDX_ANALOG_MIC_DIGITAL_GAIN];

	/* initialize delayed pm work */
	INIT_DELAYED_WORK(&p->delayed_pm_work,
			  dbmdx_delayed_pm_work_hibernate_va_ve);

	p->dbmdx_workq = create_workqueue("dbmdx-wq");
	if (!p->dbmdx_workq) {
		dev_err(p->dev, "%s: Could not create workqueue\n",
			__func__);
		ret = -EIO;
		goto out_err_free_pdata;
	}

	/* set helper functions */
	p->reset_set = dbmdx_reset_set;
	p->reset_release = dbmdx_reset_release;
	p->reset_sequence = dbmdx_reset_sequence;
	p->wakeup_set = dbmdx_wakeup_set;
	p->wakeup_release = dbmdx_wakeup_release;
	p->wakeup_toggle = dbmdx_wakeup_toggle;
	p->lock = dbmdx_lock;
	p->unlock = dbmdx_unlock;
	p->usr_op_lock = dbmdx_usr_op_lock;
	p->usr_op_unlock = dbmdx_usr_op_unlock;
	p->verify_checksum = dbmdx_verify_checksum;
	p->va_set_speed = dbmdx_va_set_speed;
	p->clk_get_rate = dbmdx_clk_get_rate;
	p->clk_set_rate = dbmdx_clk_set_rate;
	p->clk_enable = dbmdx_clk_enable;
	p->clk_disable = dbmdx_clk_disable;

	/* set callbacks (if already set externally) */
	if (g_set_i2c_freq_callback)
		p->set_i2c_freq_callback = g_set_i2c_freq_callback;
	if (g_event_callback)
		p->event_callback = g_event_callback;

	p->rxsize = MAX_REQ_SIZE;

	ret = dbmdx_common_probe(p);
	if (ret < 0) {
		dev_err(p->dev, "%s: probe failed\n", __func__);
		goto out_err_destroy_workqueue;
	}
#ifndef ALSA_SOC_INTERFACE_NOT_SUPPORTED
#ifdef SND_SOC_COMPONENT
	if (remote_component && p->remote_component_in_use == 0)
		dbmdx_remote_add_component_controls(remote_component);
#else
	if (remote_codec && p->remote_codec_in_use == 0)
		dbmdx_remote_add_codec_controls(remote_codec);
#endif

#ifndef CONFIG_SND_SOC_DBMDX
#if defined(CONFIG_SND_SOC_DBMDX_SND_CAPTURE)
	board_dbmdx_snd_init();
	snd_dbmdx_pcm_init();
#endif
#endif
#endif

#if defined(CONFIG_SND_SOC_DBMDX_I2S_CAPTURE_DEVICE)
	dbmdx_snd_init(p);
#endif

	dev_info(p->dev, "%s: successfully probed\n", __func__);
	return 0;

out_err_destroy_workqueue:
	destroy_workqueue(p->dbmdx_workq);
out_err_free_pdata:
#ifdef CONFIG_OF
	kfree(pdata);
#endif
out_err_free_private:
	kfree(p);
out:
	return ret;
}

static int dbmdx_platform_remove(struct platform_device *pdev)
{
	struct dbmdx_private *p = dev_get_drvdata(&pdev->dev);

	dbmdx_common_remove(p);
#if defined(CONFIG_SND_SOC_DBMDX_I2S_CAPTURE_DEVICE)
	dbmdx_snd_deinit(p);
#endif

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id dbmdx_of_match[] = {
	{ .compatible = "dspg,dbmdx-codec", },
	{}
};
MODULE_DEVICE_TABLE(of, dbmdx_of_match);
#endif

static struct platform_driver dbmdx_platform_driver = {
	.driver = {
		.name = "dbmdx-codec",
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = dbmdx_of_match,
#endif
	},
	.probe =    dbmdx_platform_probe,
	.remove =   dbmdx_platform_remove,
};

static int __init dbmdx_modinit(void)
{
	return platform_driver_register(&dbmdx_platform_driver);
}
module_init(dbmdx_modinit);

static void __exit dbmdx_exit(void)
{
	platform_driver_unregister(&dbmdx_platform_driver);
}
module_exit(dbmdx_exit);

MODULE_VERSION(DRIVER_VERSION);
MODULE_DESCRIPTION("DSPG DBMDX codec driver");
MODULE_LICENSE("GPL");
