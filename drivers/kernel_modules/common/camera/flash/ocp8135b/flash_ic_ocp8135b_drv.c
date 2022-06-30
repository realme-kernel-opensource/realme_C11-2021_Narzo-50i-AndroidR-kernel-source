/*
 * Copyright (C) 2015-2016 Spreadtrum Communications Inc.
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
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/vmalloc.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/module.h>
#include "sprd_img.h"
#include "flash_drv.h"

#ifdef pr_fmt
#undef pr_fmt
#endif
#define pr_fmt(fmt) "FLASH_OCP8135B: %d %d %s : " fmt, current->pid, __LINE__, __func__

#define FLASH_DRIVER_NAME "flash_ocp8135b"
#define FLASH_GPIO_MAX 2

/**
* flash max 750mA
* movie max 100mA
*/
#define DUTY_CYCLE_FLASH 1
#define DUTY_CYCLE_TORCH 0.75
#define FREQUENCY_PWM 20 //20KHZ

enum {
	GPIO_FLASH_TORCH_EN,    // 63 ENF
	GPIO_FLASH_EN,          // 62 ENM
};

struct flash_driver_data {
    int gpio_tab[FLASH_GPIO_MAX];
    u32 torch_led_index;
};

static const char *const flash_gpio_names[FLASH_GPIO_MAX] = {
	"flash-torch-en-gpios",	/* 63  for torch/flash mode */
	"flash-en-gpios",       /* 62  for enable ic pin */
};

static int sprd_flash_ocp8135b_deinit(void *drvd)
{
	int ret = 0;
	int gpio_id = 0;
	struct flash_driver_data *drv_data = (struct flash_driver_data *)drvd;
	if (!drv_data)
		return -EFAULT;

	gpio_id = drv_data->gpio_tab[GPIO_FLASH_TORCH_EN];
	if (gpio_is_valid(gpio_id)) {
		ret = gpio_direction_output(gpio_id, SPRD_FLASH_OFF);
		if (ret)
			goto exit;
	}

	gpio_id = drv_data->gpio_tab[GPIO_FLASH_EN];
	if (gpio_is_valid(gpio_id)) {
		ret = gpio_direction_output(gpio_id, SPRD_FLASH_OFF);
		if (ret)
			goto exit;
	}
exit:
	return ret;
}

static int sprd_flash_ocp8135b_open_torch(void *drvd, uint8_t idx)
{
	int ret = 0;
	int gpio_id = 0;
	int i = 0;
	struct flash_driver_data *drv_data = (struct flash_driver_data *)drvd;
    pr_info("torch_led_index:%d, idx:%d\n", drv_data->torch_led_index, idx);
	if (!drv_data)
		return -EFAULT;

	idx = drv_data->torch_led_index;
	if (SPRD_FLASH_LED0 & idx) {
		gpio_id = drv_data->gpio_tab[GPIO_FLASH_EN];
		if (gpio_is_valid(gpio_id)) {
			ret = gpio_direction_output(gpio_id, SPRD_FLASH_OFF);
			if (ret)
				goto exit;
		}

		gpio_id = drv_data->gpio_tab[GPIO_FLASH_TORCH_EN];
		if (gpio_is_valid(gpio_id)) {
			ret = gpio_direction_output(gpio_id, SPRD_FLASH_ON);
			udelay(6 * 1000);
			for (i = 0; i < 7; i++)
			{
			ret = gpio_direction_output(gpio_id, SPRD_FLASH_OFF);
			udelay(1000 * (1-DUTY_CYCLE_TORCH)/FREQUENCY_PWM);
			ret = gpio_direction_output(gpio_id, SPRD_FLASH_ON);
			udelay(1000 * DUTY_CYCLE_TORCH/FREQUENCY_PWM);
			}
		}
	}
exit:
    pr_info("x\n");
    return ret;
}

static int sprd_flash_ocp8135b_close_torch(void *drvd, uint8_t idx)
{
	int ret = 0;
	int gpio_id = 0;
	struct flash_driver_data *drv_data = (struct flash_driver_data *)drvd;
        pr_info("torch_led_index:%d, idx:%d\n", drv_data->torch_led_index, idx);

	if (!drv_data)
		return -EFAULT;

        idx = drv_data->torch_led_index;
	if (SPRD_FLASH_LED0 & idx) {
		gpio_id = drv_data->gpio_tab[GPIO_FLASH_TORCH_EN];
		if (gpio_is_valid(gpio_id)) {
			ret = gpio_direction_output(gpio_id, SPRD_FLASH_OFF);
			if (ret)
				goto exit;
		}
	}
exit:
	return ret;
}

static int sprd_flash_ocp8135b_open_preflash(void *drvd, uint8_t idx)
{
	int ret = 0;
	int gpio_id = 0;
	struct flash_driver_data *drv_data = (struct flash_driver_data *)drvd;

	if (!drv_data)
		return -EFAULT;

	pr_info("torch_led_index:%d, idx:%d\n", drv_data->torch_led_index, idx);
        idx = drv_data->torch_led_index;
	if (SPRD_FLASH_LED0 & idx) {
		gpio_id = drv_data->gpio_tab[GPIO_FLASH_EN];
		if (gpio_is_valid(gpio_id)) {
			ret = gpio_direction_output(gpio_id, SPRD_FLASH_ON);
			if (ret) {
				goto exit;
			}
		}
	}

exit:
	return ret;
}

static int sprd_flash_ocp8135b_close_preflash(void *drvd, uint8_t idx)
{
	int ret = 0;
	int gpio_id = 0;
	struct flash_driver_data *drv_data = (struct flash_driver_data *)drvd;

	if (!drv_data)
		return -EFAULT;

	pr_info("torch_led_index:%d, idx:%d\n", drv_data->torch_led_index, idx);
        idx = drv_data->torch_led_index;
	if (SPRD_FLASH_LED0 & idx) {
		gpio_id = drv_data->gpio_tab[GPIO_FLASH_EN];
		if (gpio_is_valid(gpio_id)) {
			ret = gpio_direction_output(gpio_id, SPRD_FLASH_OFF);
			if (ret)
				goto exit;
		}

		gpio_id = drv_data->gpio_tab[GPIO_FLASH_TORCH_EN];
		if (gpio_is_valid(gpio_id)) {
			ret = gpio_direction_output(gpio_id, SPRD_FLASH_OFF);
			if (ret)
				goto exit;
		}
	}

exit:
	return ret;
}

static int sprd_flash_ocp8135b_open_highlight(void *drvd, uint8_t idx)
{
	int ret = 0;
	int gpio_id = 0;
        int i = 0;
	struct flash_driver_data *drv_data = (struct flash_driver_data *)drvd;
        pr_info("torch_led_index:%d, idx:%d\n", drv_data->torch_led_index, idx);
	if (!drv_data)
        return -EFAULT;

    idx = drv_data->torch_led_index;
	if (SPRD_FLASH_LED0 & idx) {
		gpio_id = drv_data->gpio_tab[GPIO_FLASH_TORCH_EN];
		if (gpio_is_valid(gpio_id)) {
			for (i = 0; i < 3; i++)
			{
			ret = gpio_direction_output(gpio_id, SPRD_FLASH_ON);
			udelay(1000 * DUTY_CYCLE_FLASH/FREQUENCY_PWM);
			ret = gpio_direction_output(gpio_id, SPRD_FLASH_OFF);
			udelay(1000 * (1 - DUTY_CYCLE_FLASH)/FREQUENCY_PWM);
			}
		}

		gpio_id = drv_data->gpio_tab[GPIO_FLASH_EN];
		if (gpio_is_valid(gpio_id)) {
			ret = gpio_direction_output(gpio_id, SPRD_FLASH_ON);
			if (ret)
				goto exit;
		}

        gpio_id = drv_data->gpio_tab[GPIO_FLASH_TORCH_EN];
        if (gpio_is_valid(gpio_id)) {
            for (i = 0; i < 5; i++)
            {
                ret = gpio_direction_output(gpio_id, SPRD_FLASH_ON);
                udelay(1000 * DUTY_CYCLE_FLASH/FREQUENCY_PWM);
                ret = gpio_direction_output(gpio_id, SPRD_FLASH_OFF);
                udelay(1000* (1 - DUTY_CYCLE_FLASH)/FREQUENCY_PWM);
            }
        }
    }
exit:
	return ret;
}

static int sprd_flash_ocp8135b_close_highlight(void *drvd, uint8_t idx)
{
	int ret = 0;
	int gpio_id = 0;
	struct flash_driver_data *drv_data = (struct flash_driver_data *)drvd;
        pr_info("idx:%d\n",idx);
	if (!drv_data)
		return -EFAULT;

        idx = drv_data->torch_led_index;
	if (SPRD_FLASH_LED0 & idx) {
		gpio_id = drv_data->gpio_tab[GPIO_FLASH_EN];
		if (gpio_is_valid(gpio_id)) {
			ret = gpio_direction_output(gpio_id, SPRD_FLASH_OFF);
			if (ret)
				goto exit;
		}
	}
exit:
	return ret;
}


static int sprd_flash_ocp8135b_cfg_value_torch(void *drvd, uint8_t idx,
    struct sprd_flash_element *element) {
    int ret = 0;
    int gpio_id = 0;
    struct flash_driver_data *drv_data = (struct flash_driver_data *)drvd;
    pr_info("idx:%d\n",idx);
    idx = drv_data->torch_led_index;
	if (SPRD_FLASH_LED0 & idx) {
		gpio_id = drv_data->gpio_tab[GPIO_FLASH_TORCH_EN];
		if (gpio_is_valid(gpio_id)) {
			ret = gpio_direction_output(gpio_id, SPRD_FLASH_OFF);
			if (ret)
				goto exit;
		}
	}
exit:
	return ret;
}

static int sprd_flash_ocp8135b_cfg_value_preflash(void *drvd, uint8_t idx,
				struct sprd_flash_element *element) {
	int ret = 0;
	int gpio_id = 0;
	struct flash_driver_data *drv_data = (struct flash_driver_data *)drvd;
	pr_info("idx:%d\n", idx);
        idx = drv_data->torch_led_index;
	if (SPRD_FLASH_LED0 & idx) {
		gpio_id = drv_data->gpio_tab[GPIO_FLASH_TORCH_EN];
		if (gpio_is_valid(gpio_id)) {
			ret = gpio_direction_output(gpio_id, SPRD_FLASH_OFF);
			if (ret)
				goto exit;
		}
	}
exit:
	return ret;
}

static int sprd_flash_ocp8135b_cfg_value_highlight(void *drvd, uint8_t idx,
                               struct sprd_flash_element *element) {
	struct flash_driver_data *drv_data = (struct flash_driver_data *)drvd;
	if (!drv_data)
		return -EFAULT;
        pr_err("element->index:%d\n", element->index);
	return 0;
}


static const struct sprd_flash_driver_ops flash_gpio_ops = {
	.open_torch = sprd_flash_ocp8135b_open_torch,
	.close_torch = sprd_flash_ocp8135b_close_torch,
	.open_preflash = sprd_flash_ocp8135b_open_preflash,
	.close_preflash = sprd_flash_ocp8135b_close_preflash,
	.open_highlight = sprd_flash_ocp8135b_open_highlight,
	.close_highlight = sprd_flash_ocp8135b_close_highlight,
	.cfg_value_torch = sprd_flash_ocp8135b_cfg_value_torch,
	.cfg_value_preflash = sprd_flash_ocp8135b_cfg_value_preflash,
	.cfg_value_highlight = sprd_flash_ocp8135b_cfg_value_highlight,
};

static const struct of_device_id ocp8135b_flash_of_match_table[] = {
	{.compatible = "sprd,flash-ocp8135b", .data = &flash_gpio_ops},
};

static int sprd_flash_ocp8135b_probe(struct platform_device *pdev)
{
	int ret = 0;
	u32 gpio_node = 0;
	struct flash_driver_data *drv_data = NULL;
	int gpio[FLASH_GPIO_MAX];
	int j;
        pr_err("E\n");
	if (IS_ERR_OR_NULL(pdev))
		return -EINVAL;

	if (!pdev->dev.of_node) {
		pr_err("no device node %s", __func__);
		return -ENODEV;
	}

	ret = of_property_read_u32(pdev->dev.of_node, "flash-ic", &gpio_node);
	if (ret || gpio_node != 8135) {
		pr_err("no ocp8135b flash\n");
		return -ENODEV;
	}

	drv_data = devm_kzalloc(&pdev->dev, sizeof(*drv_data), GFP_KERNEL);
	if (!drv_data)
		return -ENOMEM;

	pdev->dev.platform_data = (void *)drv_data;

	ret = of_property_read_u32(pdev->dev.of_node,
				   "torch-led-idx", &drv_data->torch_led_index);
	if (ret)
		drv_data->torch_led_index = SPRD_FLASH_LED0;

	for (j = 0; j < FLASH_GPIO_MAX; j++) {
		gpio[j] = of_get_named_gpio(pdev->dev.of_node,
					    flash_gpio_names[j], 0);
		if (gpio_is_valid(gpio[j])) {
			ret = devm_gpio_request(&pdev->dev,
						gpio[j], flash_gpio_names[j]);
                        pr_err("gpio init gpio%d ,%d ,%c \n",j,gpio[j],flash_gpio_names[j]);
			if (ret) {
				pr_err("flash gpio err\n");
				goto exit;
			}

			ret = gpio_direction_output(gpio[j], SPRD_FLASH_OFF);

			if (ret) {
				pr_err("flash gpio output err\n");
				goto exit;
			}
		}
	}

	memcpy((void *)drv_data->gpio_tab, (void *)gpio, sizeof(gpio));

	ret = sprd_flash_register(&flash_gpio_ops, drv_data, SPRD_FLASH_REAR);

	if (ret)
		goto exit;
exit:
        pr_err("flash-ocp8135b probe x\n");
	return ret;
}

static int sprd_flash_ocp8135b_remove(struct platform_device *pdev)
{
	int ret = 0;

	ret = sprd_flash_ocp8135b_deinit(pdev->dev.platform_data);
	if (ret)
		pr_err("flash deinit err\n");

	return ret;
}

static const struct platform_device_id ocp8135b_flash_id[] = {
	{"flash_ocp8135b", 0},
	{},
};

static struct platform_driver sprd_flash_ocp8135b_driver = {
	.probe = sprd_flash_ocp8135b_probe,
	.remove = sprd_flash_ocp8135b_remove,
	.driver = {
		   .name = FLASH_DRIVER_NAME,
                   .owner = THIS_MODULE,
		   .of_match_table = of_match_ptr(ocp8135b_flash_of_match_table),
		   },
        .id_table = ocp8135b_flash_id,
};

module_platform_driver(sprd_flash_ocp8135b_driver);
MODULE_DESCRIPTION("Spreadtrum OCP8135 Camera Flash Driver");
MODULE_LICENSE("GPL");
