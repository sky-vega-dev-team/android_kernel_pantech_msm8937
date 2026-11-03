// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2026 The LineageOS Project
 *
 * EF71 wireless charging input.
 *
 * The EF71 has a BQ51221 Qi/PMA receiver feeding the PMI8950 DCIN, and a
 * MAX14731 in front of USBIN and DCIN that reports which input has power
 * (USB_VBUS_DET_N, WLC_DET_N) and takes the OVP enable, P-CONNECT and OTG
 * enable inputs from the SoC. The charging itself is left to
 * qpnp-smbcharger, which handles DCIN as its "dc" power supply, and USB
 * host mode to msm_otg, which turns the smbcharger OTG boost on through
 * its vbus_otg regulator. This driver keeps OVP_EN_N and P-CONNECT low as
 * the stock firmware does, raises OTG_EN while that boost is on so the
 * MAX14731 passes it to the connector, and suspends DCIN while USB is
 * plugged in as well, so a cable always takes over from the pad.
 */

#define pr_fmt(fmt) "ef71-wireless: " fmt

#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/regulator/consumer.h>
#include <linux/workqueue.h>

#define DETECT_DEBOUNCE_MS	100
#define DC_PSY_RETRY_MS		1000

struct ef71_wireless {
	struct device		*dev;
	struct gpio_desc	*usb_det;
	struct gpio_desc	*wlc_det;
	struct gpio_desc	*wlc_pg;
	struct gpio_desc	*ovp_en_n;
	struct gpio_desc	*p_connect;
	struct gpio_desc	*otg_en;
	struct regulator	*vbus_otg;
	struct notifier_block	otg_nb;
	struct power_supply	*dc_psy;
	struct delayed_work	update_work;
	int			dc_enabled;
};

static void ef71_wireless_update(struct work_struct *work)
{
	struct ef71_wireless *chip = container_of(work, struct ef71_wireless,
						  update_work.work);
	union power_supply_propval val = {0, };
	bool usb, wlc;
	int rc;

	/* Both detect lines are active low */
	usb = !gpiod_get_value_cansleep(chip->usb_det);
	wlc = !gpiod_get_value_cansleep(chip->wlc_det);

	if (!chip->dc_psy) {
		chip->dc_psy = power_supply_get_by_name("dc");
		if (!chip->dc_psy) {
			schedule_delayed_work(&chip->update_work,
					      msecs_to_jiffies(DC_PSY_RETRY_MS));
			return;
		}
	}

	val.intval = !(usb && wlc);
	if (val.intval == chip->dc_enabled)
		goto out;

	rc = power_supply_set_property(chip->dc_psy,
				       POWER_SUPPLY_PROP_CHARGING_ENABLED, &val);
	if (rc < 0) {
		pr_err("Couldn't %s DCIN, rc=%d\n",
		       val.intval ? "resume" : "suspend", rc);
		return;
	}
	chip->dc_enabled = val.intval;
out:
	pr_debug("usb=%d wireless=%d pg=%d dcin %s\n", usb, wlc,
		 chip->wlc_pg ? gpiod_get_value_cansleep(chip->wlc_pg) : -1,
		 chip->dc_enabled ? "on" : "suspended");
}

static irqreturn_t ef71_wireless_detect_irq(int irq, void *data)
{
	struct ef71_wireless *chip = data;

	mod_delayed_work(system_wq, &chip->update_work,
			 msecs_to_jiffies(DETECT_DEBOUNCE_MS));
	return IRQ_HANDLED;
}

/* Follow the smbcharger OTG boost, as msm_otg did on msm-3.18 */
static int ef71_wireless_otg_event(struct notifier_block *nb,
				   unsigned long event, void *data)
{
	struct ef71_wireless *chip = container_of(nb, struct ef71_wireless,
						  otg_nb);

	if (event & (REGULATOR_EVENT_ENABLE | REGULATOR_EVENT_ABORT_DISABLE)) {
		gpiod_set_value(chip->otg_en, 1);
		pr_debug("OTG boost on\n");
	} else if (event & (REGULATOR_EVENT_PRE_DISABLE |
			    REGULATOR_EVENT_DISABLE)) {
		gpiod_set_value(chip->otg_en, 0);
		pr_debug("OTG boost off\n");
	}

	return NOTIFY_OK;
}

static int ef71_wireless_request_irq(struct ef71_wireless *chip,
				     struct gpio_desc *gpio, const char *name)
{
	int irq = gpiod_to_irq(gpio);

	if (irq < 0)
		return irq;

	return devm_request_threaded_irq(chip->dev, irq, NULL,
				ef71_wireless_detect_irq,
				IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING |
				IRQF_ONESHOT, name, chip);
}

static int ef71_wireless_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct ef71_wireless *chip;
	int rc;

	chip = devm_kzalloc(dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	chip->dev = dev;
	chip->dc_enabled = -1;
	INIT_DELAYED_WORK(&chip->update_work, ef71_wireless_update);

	chip->usb_det = devm_gpiod_get(dev, "usb-det", GPIOD_IN);
	if (IS_ERR(chip->usb_det))
		return PTR_ERR(chip->usb_det);

	chip->wlc_det = devm_gpiod_get(dev, "wlc-det", GPIOD_IN);
	if (IS_ERR(chip->wlc_det))
		return PTR_ERR(chip->wlc_det);

	chip->wlc_pg = devm_gpiod_get_optional(dev, "wlc-pg", GPIOD_IN);
	if (IS_ERR(chip->wlc_pg))
		return PTR_ERR(chip->wlc_pg);

	/* MAX14731 controls, low as the stock firmware leaves them */
	chip->ovp_en_n = devm_gpiod_get_optional(dev, "ovp-en-n", GPIOD_OUT_LOW);
	if (IS_ERR(chip->ovp_en_n))
		return PTR_ERR(chip->ovp_en_n);

	chip->p_connect = devm_gpiod_get_optional(dev, "p-connect", GPIOD_OUT_LOW);
	if (IS_ERR(chip->p_connect))
		return PTR_ERR(chip->p_connect);

	chip->otg_en = devm_gpiod_get(dev, "otg-en", GPIOD_OUT_LOW);
	if (IS_ERR(chip->otg_en))
		return PTR_ERR(chip->otg_en);

	chip->vbus_otg = devm_regulator_get(dev, "vbus-otg");
	if (IS_ERR(chip->vbus_otg))
		return PTR_ERR(chip->vbus_otg);

	chip->otg_nb.notifier_call = ef71_wireless_otg_event;
	rc = devm_regulator_register_notifier(chip->vbus_otg, &chip->otg_nb);
	if (rc < 0) {
		dev_err(dev, "Couldn't watch the OTG regulator, rc=%d\n", rc);
		return rc;
	}
	if (regulator_is_enabled(chip->vbus_otg) > 0)
		gpiod_set_value(chip->otg_en, 1);

	rc = ef71_wireless_request_irq(chip, chip->usb_det, "ef71_usb_det");
	if (rc < 0) {
		dev_err(dev, "Couldn't request the USB detect IRQ, rc=%d\n", rc);
		return rc;
	}

	rc = ef71_wireless_request_irq(chip, chip->wlc_det, "ef71_wlc_det");
	if (rc < 0) {
		dev_err(dev, "Couldn't request the wireless detect IRQ, rc=%d\n",
			rc);
		return rc;
	}

	platform_set_drvdata(pdev, chip);
	schedule_delayed_work(&chip->update_work, 0);

	return 0;
}

static int ef71_wireless_remove(struct platform_device *pdev)
{
	struct ef71_wireless *chip = platform_get_drvdata(pdev);

	cancel_delayed_work_sync(&chip->update_work);
	if (chip->dc_psy)
		power_supply_put(chip->dc_psy);

	return 0;
}

static const struct of_device_id ef71_wireless_match[] = {
	{ .compatible = "pantech,ef71-wireless" },
	{ }
};
MODULE_DEVICE_TABLE(of, ef71_wireless_match);

static struct platform_driver ef71_wireless_driver = {
	.driver = {
		.name = "ef71-wireless",
		.of_match_table = ef71_wireless_match,
	},
	.probe = ef71_wireless_probe,
	.remove = ef71_wireless_remove,
};
module_platform_driver(ef71_wireless_driver);

MODULE_DESCRIPTION("EF71 wireless charging input");
MODULE_LICENSE("GPL v2");
