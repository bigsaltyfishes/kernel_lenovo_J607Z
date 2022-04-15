/*
 * linux/sound/cs35l41.h -- Platform data for CS35L41
 *
 * Copyright (c) 2018 Cirrus Logic Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __CS35L41_H
#define __CS35L41_H

struct classh_cfg {
	bool classh_bst_override;
	bool classh_algo_enable;
	int classh_bst_max_limit;
	int classh_mem_depth;
	int classh_release_rate;
	int classh_headroom;
	int classh_wk_fet_delay;
	int classh_wk_fet_thld;
};

struct irq_cfg {
	bool is_present;
	bool irq_pol_inv;
	bool irq_out_en;
	int irq_src_sel;
};

struct cs35l41_platform_data {
	bool sclk_frc;
	bool lrclk_frc;
	bool right_channel;
	bool amp_gain_zc;
	bool ng_enable;
	bool tuning_has_prefix;
	bool hibernate_enable;
	bool fwname_use_revid;
	bool handle_ssr;
	bool disable_wake_ctrl;
	int bst_ind;
	int bst_vctrl;
	int bst_ipk;
	int bst_cap;
	int temp_warn_thld;
	int ng_pcm_thld;
	int ng_delay;
	int dout_hiz;
	struct irq_cfg irq_config1;
	struct irq_cfg irq_config2;
	struct classh_cfg classh_config;
};

struct cs35l41_rst_cache {
	bool extclk_cfg;
	int asp_width;
	int asp_wl;
	int asp_fmt;
	int lrclk_fmt;
	int sclk_fmt;
	int slave_mode;
	int fs_cfg;
};

struct cs35l41_private {
	struct wm_adsp dsp; /* needs to be first member */
	struct snd_soc_codec *codec;
	struct cs35l41_platform_data pdata;
	struct device *dev;
	struct regmap *regmap;
	struct regulator_bulk_data supplies[2];
	int num_supplies;
	int irq;
	int clksrc;
	int extclk_freq;
	int extclk_cfg;
	int sclk;
	bool reload_tuning;
	unsigned int gpi_glob_en;
	int lrclk_fmt;
	int sclk_fmt;
	int amp_hibernate;
	bool dspa_mode;
	bool i2s_mode;
	bool swire_mode;
	bool halo_booted;
	bool halo_routed;
	bool skip_codec_probe;
	bool enabled;
	bool bus_spi;
	bool fast_switch_en;
	bool force_int;
	struct mutex rate_lock;
	bool hibernate_force_wake;
	bool restart_needed;
	/* GPIO for /RST */
	struct gpio_desc *reset_gpio;
	/* Run-time mixer */
	unsigned int fast_switch_file_idx;
	struct soc_enum fast_switch_enum;
	const char **fast_switch_names;
	struct mutex force_int_lock;
	struct delayed_work hb_work;
	struct work_struct restart_work;
	struct workqueue_struct *wq;
	struct mutex hb_lock;
	struct cs35l41_rst_cache reset_cache;
	unsigned int ctl_cache[CS35L41_CTRL_CACHE_SIZE];
};

int cs35l41_probe(struct cs35l41_private *cs35l41,
				struct cs35l41_platform_data *pdata);
int cs35l41_remove(struct cs35l41_private *cs35l41);

#endif /* __CS35L41_H */
