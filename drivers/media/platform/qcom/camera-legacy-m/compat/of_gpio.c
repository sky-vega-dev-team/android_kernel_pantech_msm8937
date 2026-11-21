// SPDX-License-Identifier: GPL-2.0-only
#include <linux/gpio/driver.h>
#include <linux/of.h>
#include "linux/of_gpio.h"

/* Global GPIO number of a "<&chip offset flags>" specifier */
int camlm_of_get_named_gpio(const struct device_node *np, const char *name,
			    int index)
{
	struct of_phandle_args args;
	struct gpio_device *gdev;
	int ret;

	ret = of_parse_phandle_with_args(np, name, "#gpio-cells", index, &args);
	if (ret)
		return ret;

	gdev = gpio_device_find_by_fwnode(of_fwnode_handle(args.np));
	of_node_put(args.np);
	if (!gdev)
		return -EPROBE_DEFER;

	ret = gpio_device_get_base(gdev) + args.args[0];
	gpio_device_put(gdev);

	return ret;
}
