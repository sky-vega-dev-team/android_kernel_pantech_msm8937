/* SPDX-License-Identifier: GPL-2.0-only */
/* The integer GPIO lookups camera_v2 uses, gone from mainline */
#ifndef __CAMLM_OF_GPIO_H
#define __CAMLM_OF_GPIO_H

#include <linux/of.h>
#include <linux/gpio.h>

int camlm_of_get_named_gpio(const struct device_node *np, const char *name,
			    int index);

static inline int of_get_named_gpio(const struct device_node *np,
				    const char *name, int index)
{
	return camlm_of_get_named_gpio(np, name, index);
}

static inline int of_get_gpio(const struct device_node *np, int index)
{
	return camlm_of_get_named_gpio(np, "gpios", index);
}

static inline int of_gpio_named_count(const struct device_node *np,
				      const char *name)
{
	return of_count_phandle_with_args(np, name, "#gpio-cells");
}

static inline int of_gpio_count(const struct device_node *np)
{
	return of_gpio_named_count(np, "gpios");
}

#endif
