// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2014-2020, The Linux Foundation. All rights reserved.
 * Author yingangl@qti.qualcomm.com
 */

#include <asm/cacheflush.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/init.h>
#include <linux/export.h>
#include <linux/err.h>
#include <linux/of.h>
#include <linux/of_address.h>

#include <linux/of_device.h>
#include <linux/dma-mapping.h>
#include <linux/module.h>

#define LOG_LEN 0x10000
static int bootloader_log_probe(struct platform_device *pdev)
{
  	int ret;
  	int i = 0;

  	phys_addr_t region_start;
  	void *region = NULL;
  	u32 aligned_size = 0;
  	char *x;
  	char line[80];
  	pr_info("bootloader_log_probe \n");
 
  	ret = of_property_read_u32(pdev->dev.of_node, "qcom,bootloader_log_size", &aligned_size);
  	if (ret) {
  		pr_err("bootloader_log_size size (default) = %u\n", LOG_LEN);
  		aligned_size = LOG_LEN;
  	} else {
  		pr_err("bootloader_log_size = %u\n", aligned_size);
  	 }
 
  	region = dma_alloc_attrs(&pdev->dev, aligned_size,&region_start, GFP_KERNEL,DMA_ATTR_SKIP_ZEROING);

 	if(!region){
 			pr_err("bootloader_log alloc failure \n");
 			goto alloc_failure;
  	}
 	else{
  		pr_err("bootloader_log region alloc OK 0x%p\n",region);
  	}
 	pr_info("%s %d probe done \n", __func__, __LINE__);
 
  	for(i=0;i < aligned_size;i=i+80){
 	 	x=(char *)region;
 	 	x+=i;
	 	memcpy(line, x, 80);
	 	if (strstr(line, "UEFI End")){
			 pr_err("bootloader_log end \n");
			 break;
		 }
	 	printk(KERN_ERR"%s\n", line);
 	}
 
	alloc_failure:
  	return ret;
}

static const struct of_device_id bootloader_log_match_table[] = {
	 {.compatible = "qcom,bootloader_log",},
 	 {}
};

static struct platform_driver bootloader_log_driver = {
	.probe = bootloader_log_probe,
	.driver = {
		.name = "bootloader_log",
		.owner = THIS_MODULE,
		.of_match_table = bootloader_log_match_table,
	},
};


static int __init bootloader_log_init(void)
{
	 pr_info("bootloader_log_init \n");
	 return platform_driver_register(&bootloader_log_driver);
}

late_initcall(bootloader_log_init);
