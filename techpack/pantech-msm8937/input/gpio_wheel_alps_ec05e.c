/*
 * GPIO wheel APLS EC05E device driver module.
 *
 * Copyright (c) 2016 Pantech Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

#include <linux/module.h>

#include <linux/gpio_wheel_alps_ec05e.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#include <linux/pinctrl/consumer.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/miscdevice.h>
#ifdef CONFIG_PANTECH_ERR_CRASH_LOGGING
#include <mach/pantech_apanic.h>
#endif

#define EC05E_TERMINAL_A_DEV_NAME "terminal_a"
#define EC05E_TERMINAL_B_DEV_NAME "terminal_b"
#define EC05E_CW_DIRECTION -1
#define EC05E_CCW_DIRECTION 1

enum GPIO_WHEEL_ALPS_EC05E_IOCTL_CMD {
    SWITCH_EVENT_MODE = 0
};

enum GPIO_WHEEL_ALPS_EC05E_EVENT_MODE {
    KEY_VOLUME_MODE = 0,
    KEY_PAGE_MODE = 1,
    KEY_UP_DOWN_MODE = 2,
    REL_WHEEL_MODE = 3
};

static int terminal_a_gpio = 0;
static int terminal_b_gpio = 0;
static bool cw_flag = false;
static bool ccw_flag = false;
static int direction = 0;
static int event_mode = 0;
static bool report_rel_position_flag = false;

struct gpio_wheel_alps_ec05e_terminal_data {
    const struct gpio_wheel_alps_ec05e_terminal *terminal;
    struct input_dev *input;
    struct work_struct work;
    unsigned int irq;
};

struct gpio_wheel_alps_ec05e_drvdata {
    const struct gpio_wheel_alps_ec05e_platform_data *pdata;
    struct pinctrl *terminal_pinctrl;
    struct input_dev *input;
    struct gpio_wheel_alps_ec05e_terminal_data data[0];
};

static void gpio_wheel_alps_ec05e_report_event(struct input_dev *input)
{
    unsigned int type;
    unsigned int code;
    int value;
    
    switch (event_mode) {
        case KEY_VOLUME_MODE:
            type = EV_KEY;
            if (direction == EC05E_CW_DIRECTION) {
                pr_debug("[PAN_WHEEL] %s: Report volume down event\n", __func__);
                code = KEY_VOLUMEDOWN;
            } else if (direction == EC05E_CCW_DIRECTION) {
                pr_debug("[PAN_WHEEL] %s: Report volume up event\n", __func__);
                code = KEY_VOLUMEUP;
            } else {
                pr_err("[PAN_WHEEL] %s: Invaild direction value: %d\n", __func__, direction);
                return;
            }
            break;
        case KEY_PAGE_MODE:
            type = EV_KEY;
            if (direction == EC05E_CW_DIRECTION) {
                pr_debug("[PAN_WHEEL] %s: Report page down event\n", __func__);
                code = KEY_PAGEDOWN;
            } else if (direction == EC05E_CCW_DIRECTION) {
                pr_debug("[PAN_WHEEL] %s: Report page up event\n", __func__);
                code = KEY_PAGEUP;
            } else {
                pr_err("[PAN_WHEEL] %s: Invaild direction value: %d\n", __func__, direction);
                return;
            }
            break;
        case KEY_UP_DOWN_MODE:
            type = EV_KEY;
            if (direction == EC05E_CW_DIRECTION) {
                pr_debug("[PAN_WHEEL] %s: Report key down event\n", __func__);
                code = KEY_DOWN;
            } else if (direction == EC05E_CCW_DIRECTION) {
                pr_debug("[PAN_WHEEL] %s: Report key up event\n", __func__);
                code = KEY_UP;
            } else {
                pr_err("[PAN_WHEEL] %s: Invaild direction value: %d\n", __func__, direction);
                return;
            }
            break;
        case REL_WHEEL_MODE:
            type = EV_REL;
            code = REL_WHEEL;
            if (direction == EC05E_CW_DIRECTION) {
                pr_debug("[PAN_WHEEL] %s: Report wheel down event\n", __func__);
                value = -2;
            } else if (direction == EC05E_CCW_DIRECTION) {
                pr_debug("[PAN_WHEEL] %s: Report wheel up event\n", __func__);
                value = 2;
            } else {
                pr_err("[PAN_WHEEL] %s: Invaild direction value: %d\n", __func__, direction);
                return;
            }
            break;
        default:
            pr_err("[PAN_WHEEL] %s: Invaild event mode: %d, set event mode to %d\n", __func__, event_mode, KEY_VOLUME_MODE);
            event_mode = KEY_VOLUME_MODE;
            type = EV_KEY;
            if (direction == EC05E_CW_DIRECTION) {
                pr_debug("[PAN_WHEEL] %s: Report volume down event\n", __func__);
                code = KEY_VOLUMEDOWN;
            } else if (direction == EC05E_CCW_DIRECTION) {
                pr_debug("[PAN_WHEEL] %s: Report volume up event\n", __func__);
                code = KEY_VOLUMEUP;
            } else {
                pr_err("[PAN_WHEEL] %s: Invaild direction value: %d\n", __func__, direction);
                return;
            }
            break;
    }
    
    if (type == EV_KEY) {
        input_event(input, type, code, 1);
        input_sync(input);
        input_event(input, type, code, 0);
        input_sync(input);
    } else if (type == EV_REL) {
        if (report_rel_position_flag == false) {
            input_event(input, type, REL_X, 1);
            input_event(input, type, REL_Y, 1);
            report_rel_position_flag = true;
        }
        input_event(input, type, code, value);
        input_sync(input);
    }
    
#ifdef CONFIG_PANTECH_ERR_CRASH_LOGGING
    if (direction == EC05E_CW_DIRECTION) {
        pantech_force_dump_key(KEY_VOLUMEDOWN, 1); 
    } else if (direction == EC05E_CCW_DIRECTION) {
        pantech_force_dump_key(KEY_VOLUMEUP, 1); 
    }
#endif					
}

static void gpio_wheel_alps_ec05e_work_func(struct work_struct *work)
{
    struct gpio_wheel_alps_ec05e_terminal_data *tdata = 
        container_of(work, struct gpio_wheel_alps_ec05e_terminal_data, work);
    
    pr_debug("[PAN_WHEEL] %s: Enter\n", __func__);
    
    gpio_wheel_alps_ec05e_report_event(tdata->input);
    
    if (tdata->terminal->wakeup) {
        pr_debug("[PAN_WHEEL] %s: Request to relax PM\n", __func__);
        pm_relax(tdata->input->dev.parent);
    }
}

static irqreturn_t gpio_wheel_alps_ec05e_isr_terminal_a(int irq, void *dev_id)
{
    struct gpio_wheel_alps_ec05e_terminal_data *tdata = dev_id;
    const struct gpio_wheel_alps_ec05e_terminal *terminal = tdata->terminal;
    int terminal_a_state, terminal_b_state;

    pr_debug("[PAN_WHEEL] %s: Enter\n", __func__);

    if (tdata->terminal->wakeup) {
        pr_debug("[PAN_WHEEL] %s: Request to stay awake PM\n", __func__);
        pm_stay_awake(tdata->input->dev.parent);
    }

    terminal_a_state = __gpio_get_value(terminal->gpio);
    pr_debug("[PAN_WHEEL] %s: Terminal A GPIO state = %d\n", __func__, terminal_a_state);
    terminal_b_state = __gpio_get_value(terminal_b_gpio);
    pr_debug("[PAN_WHEEL] %s: Terminal B GPIO state = %d\n", __func__, terminal_b_state);

    if (terminal_a_state == 0 && terminal_b_state == 1) {
        cw_flag = true;
    } else if (terminal_a_state == 1 && terminal_b_state == 0 && cw_flag == true) {
        cw_flag = false;
        direction = EC05E_CW_DIRECTION;
        pr_debug("[PAN_WHEEL] %s: Call work queue to process CW work\n", __func__);
        schedule_work(&tdata->work);
    } else {
        cw_flag = false;
    }

    pr_debug("[PAN_WHEEL] %s: Exit\n", __func__);

    return IRQ_HANDLED;
}

static irqreturn_t gpio_wheel_alps_ec05e_isr_terminal_b(int irq, void *dev_id)
{
    struct gpio_wheel_alps_ec05e_terminal_data *tdata = dev_id;
    const struct gpio_wheel_alps_ec05e_terminal *terminal = tdata->terminal;
    int terminal_a_state, terminal_b_state;

    pr_debug("[PAN_WHEEL] %s: Enter\n", __func__);

    if (tdata->terminal->wakeup) {
        pr_debug("[PAN_WHEEL] %s: Request to stay awake PM\n", __func__);
        pm_stay_awake(tdata->input->dev.parent);
    }

    terminal_b_state = __gpio_get_value(terminal->gpio);
    pr_debug("[PAN_WHEEL] %s: Terminal B GPIO state = %d\n", __func__, terminal_b_state);
    terminal_a_state = __gpio_get_value(terminal_a_gpio);
    pr_debug("[PAN_WHEEL] %s: Terminal A GPIO state = %d\n", __func__, terminal_a_state);

    if (terminal_b_state == 0 && terminal_a_state == 1) {
        ccw_flag = true;
    } else if (terminal_b_state == 1 && terminal_a_state == 0 && ccw_flag == true) {
        ccw_flag = false;
        direction = EC05E_CCW_DIRECTION;
        pr_debug("[PAN_WHEEL] %s: Call work queue to process CCW work\n", __func__);
        schedule_work(&tdata->work);
    } else {
        ccw_flag = false;
    }

    pr_debug("[PAN_WHEEL] %s: Exit\n", __func__);

    return IRQ_HANDLED;
}

static int gpio_wheel_alps_ec05e_setup_terminal(struct platform_device *pdev,
    struct input_dev *input,
    struct gpio_wheel_alps_ec05e_terminal_data *tdata,
    const struct gpio_wheel_alps_ec05e_terminal *terminal)
{
    const char *desc = terminal->desc ? terminal->desc : "gpio_wheel_alps_ec05e";
    struct device *dev = &pdev->dev;
    irq_handler_t isr;
    unsigned long irqflags;
    int irq;
    int error;

    tdata->input = input;
    tdata->terminal = terminal;

    if (gpio_is_valid(terminal->gpio)) {
        error = devm_gpio_request_one(&pdev->dev, terminal->gpio, GPIOF_IN, desc);
        if (error < 0) {
            dev_err(dev, "[PAN_WHEEL] Failed to request GPIO %d, error %d\n", terminal->gpio, error);
            return error;
        }

        irq = gpio_to_irq(terminal->gpio);
        if (irq < 0) {
            error = irq;
            dev_err(dev, "[PAN_WHEEL] Unable to get irq number for GPIO %d, error %d\n", terminal->gpio, error);
            return error;
        }
        tdata->irq = irq;

        INIT_WORK(&tdata->work, gpio_wheel_alps_ec05e_work_func);

        if (!strcmp(desc, EC05E_TERMINAL_A_DEV_NAME)) {
            isr = gpio_wheel_alps_ec05e_isr_terminal_a;
            irqflags = IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING;

            error = request_irq(tdata->irq, isr, irqflags, desc, tdata);
            if (error < 0) {
                dev_err(dev, "[PAN_WHEEL] Unable to claim irq %d: error %d\n", tdata->irq, error);
                return error;
            }
        } else if (!strcmp(desc, EC05E_TERMINAL_B_DEV_NAME)) {
            isr = gpio_wheel_alps_ec05e_isr_terminal_b;
            irqflags = IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING;

            error = request_irq(tdata->irq, isr, irqflags, desc, tdata);
            if (error < 0) {
                dev_err(dev, "[PAN_WHEEL] Unable to claim irq %d: error %d\n", tdata->irq, error);
                return error;
            }
        }
    } else {
        pr_err("[PAN_WHEEL] %s: GPIO %d is invaild\n", __func__, terminal->gpio);
        return -EINVAL;
    }
    
    return 0;
}

static int gpio_wheel_alps_ec05e_pinctrl_configure(struct gpio_wheel_alps_ec05e_drvdata *ddata,
							bool active)
{
	struct pinctrl_state *set_state;
	int retval;

	if (active) {
		set_state = pinctrl_lookup_state(ddata->terminal_pinctrl, "tlmm_gpio_wheel_alps_ec05e_active");
		if (IS_ERR(set_state)) {
			dev_err(&ddata->input->dev, "[PAN_WHEEL] cannot get ts pinctrl active state\n");
			return PTR_ERR(set_state);
		}
	} else {
		set_state = pinctrl_lookup_state(ddata->terminal_pinctrl, "tlmm_gpio_wheel_alps_ec05e_suspend");
		if (IS_ERR(set_state)) {
			dev_err(&ddata->input->dev, "[PAN_WHEEL] cannot get gpiokey pinctrl sleep state\n");
			return PTR_ERR(set_state);
		}
	}
	retval = pinctrl_select_state(ddata->terminal_pinctrl, set_state);
	if (retval) {
		dev_err(&ddata->input->dev, "[PAN_WHEEL] cannot set ts pinctrl active state\n");
		return retval;
	}

	return 0;
}

static int gpio_wheel_alps_ec05e_open(struct input_dev *input)
{
    struct gpio_wheel_alps_ec05e_drvdata *ddata = input_get_drvdata(input);
    const struct gpio_wheel_alps_ec05e_platform_data *pdata = ddata->pdata;
    int error;

    if (pdata->enable) {
        printk("[PAN_WHEEL] %s: Enable platform data\n", __func__);
        error = pdata->enable(input->dev.parent);
        if (error) {
            pr_err("[PAN_WHEEL] %s: Failed to enable platform data\n", __func__);
            return error;
        }
    }

    return 0;
}

static void gpio_wheel_alps_ec05e_close(struct input_dev *input)
{
    struct gpio_wheel_alps_ec05e_drvdata *ddata = input_get_drvdata(input);
    const struct gpio_wheel_alps_ec05e_platform_data *pdata = ddata->pdata;

    if (pdata->disable) {
        printk("[PAN_WHEEL] %s: Disable platform data\n", __func__);
        pdata->disable(input->dev.parent);
    }
}

static long gpio_wheel_alps_ec05e_ioctl(struct file *filp, unsigned int cmd, unsigned long arg) {
    pr_debug("[PAN_WHEEL] %s: cmd = %d, arg = %d\n", __func__, cmd, (int)arg);
    switch (cmd) {
        case SWITCH_EVENT_MODE:
            if (arg == KEY_VOLUME_MODE) {
                pr_debug("[PAN_WHEEL] %s: Set event mode to %d\n", __func__, KEY_VOLUME_MODE);
                event_mode = KEY_VOLUME_MODE;
            } else if (arg == KEY_PAGE_MODE) {
                pr_debug("[PAN_WHEEL] %s: Set event mode to %d\n", __func__, KEY_PAGE_MODE);
                event_mode = KEY_PAGE_MODE;
            } else if (arg == KEY_UP_DOWN_MODE) {
                pr_debug("[PAN_WHEEL] %s: Set event mode to %d\n", __func__, KEY_UP_DOWN_MODE);
                event_mode = KEY_UP_DOWN_MODE;
            } else if (arg == REL_WHEEL_MODE) {
                pr_debug("[PAN_WHEEL] %s: Set event mode to %d\n", __func__, REL_WHEEL_MODE);
                event_mode = REL_WHEEL_MODE;
                report_rel_position_flag = false;
            } else {
                pr_err("[PAN_WHEEL] %s: Invaild argument : %d, set event mode to %d\n", __func__, (int)arg, KEY_VOLUME_MODE);
                event_mode = KEY_VOLUME_MODE;
            }
            break;
        default:
            pr_err("[PAN_WHEEL] %s: Unexpected command : %d\n", __func__, cmd);
            break;
	}
	
	return 0;
}

static struct file_operations gpio_wheel_alps_ec05e_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = gpio_wheel_alps_ec05e_ioctl,
};

static struct miscdevice gpio_wheel_alps_ec05e_miscdev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "gpio_wheel_alps_ec05e_dev",
	.fops = &gpio_wheel_alps_ec05e_fops,
};

static int gpio_wheel_alps_ec05e_miscdev_registered = 0;

#ifdef CONFIG_OF

static struct gpio_wheel_alps_ec05e_platform_data*
gpio_wheel_alps_ec05e_get_devtree_pdata(struct device *dev)
{
    struct device_node *node, *pp;
    struct gpio_wheel_alps_ec05e_platform_data *pdata;
    struct gpio_wheel_alps_ec05e_terminal *terminal;
    int error;
    int nTerminals;
    int i;

    node = dev->of_node;
    if (node == NULL) {
        pr_err("[PAN_WHEEL] %s: Dev node is NULL\n", __func__);
        return ERR_PTR(-ENODEV);
    }
    
    nTerminals = of_get_child_count(node);
    if (nTerminals == 0) {
        pr_err("[PAN_WHEEL] %s: Number of dev child node is 0\n", __func__);
        return ERR_PTR(-ENODEV);
    }

    pdata = devm_kzalloc(dev, sizeof(*pdata) + nTerminals * sizeof(*terminal), GFP_KERNEL);
    if (!pdata) {
        pr_err("[PAN_WHEEL] %s: Failed to allocate pdata\n", __func__);
        return ERR_PTR(-ENOMEM);
    }

    pdata->terminals= (struct gpio_wheel_alps_ec05e_terminal *)(pdata + 1);
    pdata->nTerminals = nTerminals;
    pr_debug("[PAN_WHEEL] %s: Number of terminal = %d\n", __func__, nTerminals);

    pdata->name = of_get_property(node, "input-name", NULL);
    
    i = 0;
    for_each_child_of_node(node, pp) {
        int gpio;
        enum of_gpio_flags flags;

        if (!of_find_property(pp, "gpios", NULL)) {
            pdata->nTerminals--;
            dev_warn(dev, "[PAN_WHEEL] Found button without gpios\n");
            continue;
        }
        
        gpio = of_get_gpio_flags(pp, 0, &flags);
        if (gpio < 0) {
            error = gpio;
            if (error != -EPROBE_DEFER) {
                dev_err(dev, "[PAN_WHEEL] Failed to get gpio flags, error: %d\n", error);
            }
            return ERR_PTR(error);
        }

        terminal = &pdata->terminals[i++];
        
        terminal->gpio = gpio;

        terminal->desc = of_get_property(pp, "label", NULL);

        terminal->wakeup = !!of_get_property(pp, "gpio-wheel,wakeup", NULL);

        if (!strcmp(terminal->desc, EC05E_TERMINAL_A_DEV_NAME)) {
            terminal_a_gpio = terminal->gpio;
        } else if (!strcmp(terminal->desc, EC05E_TERMINAL_B_DEV_NAME)) {
            terminal_b_gpio = terminal->gpio;
        }

        printk("[PAN_WHEEL] %s: label = %s, gpio = %d, wakeup = %d\n", __func__, terminal->desc, terminal->gpio, terminal->wakeup);
    }

    if (pdata->nTerminals== 0) {
        pr_err("[PAN_WHEEL] %s: Number of terminal is 0\n", __func__);
        return ERR_PTR(-EINVAL);
    }

    return pdata;
}

static const struct of_device_id gpio_wheel_alps_ec05e_of_match[] = {
    { .compatible = GPIO_WHEEL_ALPS_EC05E_DEV_NAME, },
    { },
};
MODULE_DEVICE_TABLE(of, gpio_wheel_alps_ec05e_of_match);

#else   /* CONFIG_OF */

static struct gpio_wheel_alps_ec05e_platform_data*
gpio_wheel_alps_ec05e_get_devtree_pdata(struct device *dev)
{
    pr_err("[PAN_WHEEL] %s: devtree is not available\n", __func__);
    return ERR_PTR(-ENODEV);
}

#endif  /* CONFIG_OF */

static int gpio_wheel_alps_ec05e_probe(struct platform_device *pdev)
{
    struct device *dev = &pdev->dev;
    const struct gpio_wheel_alps_ec05e_platform_data *pdata = dev_get_platdata(dev);
    struct gpio_wheel_alps_ec05e_drvdata *ddata;
    struct input_dev *input;
    size_t size;
    int error;
    int i;
    int wakeup = 0;
    struct pinctrl_state *set_state;

    printk("[PAN_WHEEL] %s: ALPS EC05E GPIO Wheel Driver Probing start\n", __func__);

    if (!pdata) {
        pdata = gpio_wheel_alps_ec05e_get_devtree_pdata(dev);
        if (IS_ERR(pdata)) {
            pr_err("[PAN_WHEEL] %s: Failed to get pdata form devtree\n", __func__);
            return PTR_ERR(pdata);
        }
    }

    size = sizeof(struct gpio_wheel_alps_ec05e_drvdata) + 
        pdata->nTerminals * sizeof(struct gpio_wheel_alps_ec05e_terminal_data);
    ddata = devm_kzalloc(dev, size, GFP_KERNEL);
    if (!ddata) {
        dev_err(dev, "[PAN_WHEEL] failed to allocate state\n");
        return -ENOMEM;
    }

    input = devm_input_allocate_device(dev);
    if (!input) {
        dev_err(dev, "[PAN_WHEEL] failed to allocate input device\n");
        return -ENOMEM;
    }

    ddata->pdata = pdata;
    ddata->input = input;
    
    platform_set_drvdata(pdev, ddata);
    input_set_drvdata(input, ddata);

    input->name = GPIO_WHEEL_ALPS_EC05E_DEV_NAME;
    input->phys = "gpio-wheel-alps-ec05e/input0";
    input->dev.parent = &pdev->dev;
    input->id.bustype = BUS_HOST;
    input->open = gpio_wheel_alps_ec05e_open;
    input->close = gpio_wheel_alps_ec05e_close;

    input_set_capability(input, EV_KEY, KEY_VOLUMEDOWN);
    input_set_capability(input, EV_KEY, KEY_VOLUMEUP);
    input_set_capability(input, EV_KEY, KEY_PAGEDOWN);
    input_set_capability(input, EV_KEY, KEY_PAGEUP);
    input_set_capability(input, EV_KEY, KEY_UP);
    input_set_capability(input, EV_KEY, KEY_DOWN);
	/*
    input_set_capability(input, EV_REL, REL_X);
    input_set_capability(input, EV_REL, REL_Y);
    input_set_capability(input, EV_KEY, BTN_MOUSE);
    input_set_capability(input, EV_REL, REL_WHEEL);
	*/

    ddata->terminal_pinctrl= devm_pinctrl_get(dev);
    if (IS_ERR(ddata->terminal_pinctrl)) {
        if (PTR_ERR(ddata->terminal_pinctrl) == -EPROBE_DEFER) {
            pr_err("[PAN_WHEEL] %s: Driver requests probe retry\n", __func__);
            return -EPROBE_DEFER;
        }
        pr_debug("[PAN_WHEEL] %s: Target does not use pinctrl\n", __func__);
        ddata->terminal_pinctrl = NULL;
    }
    
    if (ddata->terminal_pinctrl) {
        error = gpio_wheel_alps_ec05e_pinctrl_configure(ddata, true);
        if (error) {
            dev_err(dev, "[PAN_WHEEL] cannot set ts pinctrl active state, error = %d\n", error);
            return error;
        }
    }

    for (i = 0; i < pdata->nTerminals; i++) {
        const struct gpio_wheel_alps_ec05e_terminal *terminal = &pdata->terminals[i];
        struct gpio_wheel_alps_ec05e_terminal_data *tdata = &ddata->data[i];
        
        error = gpio_wheel_alps_ec05e_setup_terminal(pdev, input, tdata, terminal);
        if (error) {
            pr_err("[PAN_WHEEL] %s: failed to setup terminals\n", __func__);
            goto err_setup_terminal;
        }

        if (terminal->wakeup) {
            wakeup = 1;
        }
    }

    error = input_register_device(input);
    if (error) {
        dev_err(dev, "[PAN_WHEEL] Unable to register input device, error: %d\n", error);
        return error;
    }

    device_init_wakeup(&pdev->dev, wakeup);

    error = misc_register(&gpio_wheel_alps_ec05e_miscdev);
    if (error) {
        dev_err(dev, "[PAN_WHEEL] Could not register misc device, error: %d\n", error);
        return error;
    }
    else {
        gpio_wheel_alps_ec05e_miscdev_registered = 1;
    }

    printk("[PAN_WHEEL] %s: ALPS EC05E GPIO Wheel Driver Probing end\n", __func__);

    return 0;
    
err_setup_terminal:
    if (ddata->terminal_pinctrl) {
        set_state = pinctrl_lookup_state(ddata->terminal_pinctrl, "tlmm_gpio_wheel_alps_ec05e_suspend");
        if (IS_ERR(set_state)) {
            dev_err(dev, "[PAN_WHEEL] cannot get gpio_wheel_alps_ec05e pinctrl sleep state\n");
        }
        else {
            pinctrl_select_state(ddata->terminal_pinctrl, set_state);
        }
    }
    
    return error;
}

static int gpio_wheel_alps_ec05e_remove(struct platform_device *pdev)
{
    printk("[PAN_WHEEL] %s: ALPS EC05E GPIO Wheel Driver removing\n", __func__);

    if (gpio_wheel_alps_ec05e_miscdev_registered) {
        misc_deregister(&gpio_wheel_alps_ec05e_miscdev);
    }

    device_init_wakeup(&pdev->dev, 0);
    
    return 0;
}

#ifdef CONFIG_PM_SLEEP

static int gpio_wheel_alps_ec05e_suspend(struct device *dev)
{
    struct gpio_wheel_alps_ec05e_drvdata *ddata = dev_get_drvdata(dev);
    struct input_dev *input = ddata->input;
    int i, ret;

    if (ddata->terminal_pinctrl) {
        ret = gpio_wheel_alps_ec05e_pinctrl_configure(ddata, false);
        if (ret) {
            dev_err(dev, "[PAN_WHEEL] failed to put the pin in suspend state\n");
            return ret;
        }
    }

    if (device_may_wakeup(dev)) {
        for (i = 0; i < ddata->pdata->nTerminals; i++) {
            struct gpio_wheel_alps_ec05e_terminal_data *tdata = &ddata->data[i];
            if (tdata->terminal->wakeup) {
                enable_irq_wake(tdata->irq);
            }
        }
    } else {
        mutex_lock(&input->mutex);
        if (input->users) {
            gpio_wheel_alps_ec05e_close(input);
        }
        mutex_unlock(&input->mutex);
    }

    return 0;
}

static int gpio_wheel_alps_ec05e_resume(struct device *dev)
{
    struct gpio_wheel_alps_ec05e_drvdata *ddata = dev_get_drvdata(dev);
    struct input_dev *input = ddata->input;
    int error = 0;
    int i;

    if (ddata->terminal_pinctrl) {
        error = gpio_wheel_alps_ec05e_pinctrl_configure(ddata, true);
        if (error) {
            dev_err(dev, "[PAN_WHEEL] failed to put the pin in resume state\n");
            return error;
        }
    }

    if (device_may_wakeup(dev)) {
        for (i = 0; i < ddata->pdata->nTerminals; i++) {
            struct gpio_wheel_alps_ec05e_terminal_data *tdata = &ddata->data[i];
            if (tdata->terminal->wakeup) {
                disable_irq_wake(tdata->irq);
            }
        }
    } else {
        mutex_lock(&input->mutex);
        if (input->users) {
            error = gpio_wheel_alps_ec05e_open(input);
        }
        mutex_unlock(&input->mutex);
    }

    if (error) {
        return error;
    }

    return 0;
}

#else   /* CONFIG_PM_SLEEP */

static int gpio_wheel_alps_ec05e_suspend(struct device *dev)
{
    return 0;
}

static int gpio_wheel_alps_ec05e_resume(struct device *dev)
{
    return 0;
}

#endif  /* CONFIG_PM_SLEEP */

static SIMPLE_DEV_PM_OPS(gpio_wheel_alps_ec05e_pm_ops, gpio_wheel_alps_ec05e_suspend, gpio_wheel_alps_ec05e_resume);

static struct platform_driver gpio_wheel_alps_ec05e_driver = {
    .probe		= gpio_wheel_alps_ec05e_probe,
    .remove	= gpio_wheel_alps_ec05e_remove,
    .driver		= {
        .name	= GPIO_WHEEL_ALPS_EC05E_DEV_NAME,
        .owner	= THIS_MODULE,
        .pm	= &gpio_wheel_alps_ec05e_pm_ops,
        .of_match_table = of_match_ptr(gpio_wheel_alps_ec05e_of_match),
    }
};

static int __init gpio_wheel_alps_ec05e_init(void)
{
    //printk("[PAN_WHEEL] %s: ALPS EC05E GPIO Wheel Driver module init\n", __func__);
    
    return platform_driver_register(&gpio_wheel_alps_ec05e_driver);
}

static void __exit gpio_wheel_alps_ec05e_exit(void)
{
    printk("[PAN_WHEEL] %s: ALPS EC05E GPIO Wheel Driver module exit\n", __func__);
    
    platform_driver_unregister(&gpio_wheel_alps_ec05e_driver);
}

late_initcall(gpio_wheel_alps_ec05e_init);
module_exit(gpio_wheel_alps_ec05e_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("GPIO Wheel ALPS EC05E Driver");
