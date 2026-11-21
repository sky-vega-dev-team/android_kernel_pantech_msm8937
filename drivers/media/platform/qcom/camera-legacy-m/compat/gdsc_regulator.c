// SPDX-License-Identifier: GPL-2.0-only
/*
 * camera_v2 switches the VFE, CPP and JPEG GDSCs as regulators, the way
 * the downstream clock drivers exposed them. Mainline models GDSCs as power
 * domains: present each one as a regulator that holds its power domain
 * on while enabled.
 */
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>

struct camlm_gdsc {
	struct device *dev;
	bool enabled;
};

static int camlm_gdsc_enable(struct regulator_dev *rdev)
{
	struct camlm_gdsc *gdsc = rdev_get_drvdata(rdev);
	int ret;

	ret = pm_runtime_resume_and_get(gdsc->dev);
	if (ret)
		return ret;
	gdsc->enabled = true;

	return 0;
}

static int camlm_gdsc_disable(struct regulator_dev *rdev)
{
	struct camlm_gdsc *gdsc = rdev_get_drvdata(rdev);

	gdsc->enabled = false;
	pm_runtime_put(gdsc->dev);

	return 0;
}

static int camlm_gdsc_is_enabled(struct regulator_dev *rdev)
{
	struct camlm_gdsc *gdsc = rdev_get_drvdata(rdev);

	return gdsc->enabled;
}

static const struct regulator_ops camlm_gdsc_ops = {
	.enable = camlm_gdsc_enable,
	.disable = camlm_gdsc_disable,
	.is_enabled = camlm_gdsc_is_enabled,
};

static int camlm_gdsc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct regulator_config cfg = {};
	struct regulator_desc *desc;
	struct regulator_dev *rdev;
	struct camlm_gdsc *gdsc;
	int ret;

	gdsc = devm_kzalloc(dev, sizeof(*gdsc), GFP_KERNEL);
	desc = devm_kzalloc(dev, sizeof(*desc), GFP_KERNEL);
	if (!gdsc || !desc)
		return -ENOMEM;
	gdsc->dev = dev;

	ret = devm_pm_runtime_enable(dev);
	if (ret)
		return ret;

	desc->name = dev_name(dev);
	desc->type = REGULATOR_VOLTAGE;
	desc->owner = THIS_MODULE;
	desc->ops = &camlm_gdsc_ops;

	cfg.dev = dev;
	cfg.driver_data = gdsc;
	cfg.of_node = dev->of_node;
	cfg.init_data = of_get_regulator_init_data(dev, dev->of_node, desc);

	rdev = devm_regulator_register(dev, desc, &cfg);
	return PTR_ERR_OR_ZERO(rdev);
}

static const struct of_device_id camlm_gdsc_of_match[] = {
	{ .compatible = "qcom,camlm-gdsc-regulator" },
	{ }
};

static struct platform_driver camlm_gdsc_driver = {
	.probe = camlm_gdsc_probe,
	.driver = {
		.name = "camlm-gdsc-regulator",
		.of_match_table = camlm_gdsc_of_match,
	},
};
module_platform_driver(camlm_gdsc_driver);
