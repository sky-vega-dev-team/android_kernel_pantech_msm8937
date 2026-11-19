// SPDX-License-Identifier: GPL-2.0-only
/*
 * Novatek NT35695 1080p command mode panel of the Pantech VEGA IRON 2 (EF71S)
 *
 * Init sequence, timings and DCS backlight from the EF71 msm-3.18 panel
 * description (qcom,mdss_dsi_nt35695_1080p_cmd_ef71).
 */

#include <linux/backlight.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>

#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_panel.h>
#include <drm/drm_probe_helper.h>

#include <video/mipi_display.h>

#define NT35695_BL_MAX	216

struct ef71_nt35695 {
	struct drm_panel panel;
	struct mipi_dsi_device *dsi;
	struct gpio_desc *reset_gpio;
};

static inline struct ef71_nt35695 *to_ef71_nt35695(struct drm_panel *panel)
{
	return container_of_const(panel, struct ef71_nt35695, panel);
}

static void ef71_nt35695_reset(struct ef71_nt35695 *ctx)
{
	/* msm-3.18: <1 20>, <0 1>, <1 20> on the active low reset line */
	gpiod_set_value_cansleep(ctx->reset_gpio, 0);
	msleep(20);
	gpiod_set_value_cansleep(ctx->reset_gpio, 1);
	usleep_range(1000, 2000);
	gpiod_set_value_cansleep(ctx->reset_gpio, 0);
	msleep(20);
}

static int ef71_nt35695_on(struct ef71_nt35695 *ctx)
{
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = ctx->dsi };

	ctx->dsi->mode_flags |= MIPI_DSI_MODE_LPM;

	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xff, 0x10);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xfb, 0x01);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xba, 0x03);
	/* Tearing effect on VBlank only */
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x35, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x44, 0x03);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xbb, 0x10);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb0, 0x03);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xff, 0x10);
	/* CABC: user interface image */
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x55, 0x01);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xff, 0x23);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x01, 0x84);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xfb, 0x01);
	/* Backlight PWM at 40 kHz */
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x08, 0x02);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x46, 0x45);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xff, 0x10);
	mipi_dsi_dcs_exit_sleep_mode_multi(&dsi_ctx);
	mipi_dsi_msleep(&dsi_ctx, 110);
	/* Brightness control and backlight on */
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, MIPI_DCS_WRITE_CONTROL_DISPLAY, 0x24);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xff, 0x10);
	mipi_dsi_dcs_set_display_on_multi(&dsi_ctx);
	mipi_dsi_msleep(&dsi_ctx, 48);

	return dsi_ctx.accum_err;
}

static int ef71_nt35695_off(struct ef71_nt35695 *ctx)
{
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = ctx->dsi };

	ctx->dsi->mode_flags &= ~MIPI_DSI_MODE_LPM;

	mipi_dsi_dcs_set_display_off_multi(&dsi_ctx);
	mipi_dsi_msleep(&dsi_ctx, 150);
	mipi_dsi_dcs_enter_sleep_mode_multi(&dsi_ctx);
	mipi_dsi_msleep(&dsi_ctx, 150);

	return dsi_ctx.accum_err;
}

static int ef71_nt35695_prepare(struct drm_panel *panel)
{
	struct ef71_nt35695 *ctx = to_ef71_nt35695(panel);
	struct device *dev = &ctx->dsi->dev;
	int ret;

	ef71_nt35695_reset(ctx);

	ret = ef71_nt35695_on(ctx);
	if (ret < 0) {
		dev_err(dev, "Failed to initialize panel: %d\n", ret);
		gpiod_set_value_cansleep(ctx->reset_gpio, 1);
		return ret;
	}

	return 0;
}

static int ef71_nt35695_unprepare(struct drm_panel *panel)
{
	struct ef71_nt35695 *ctx = to_ef71_nt35695(panel);
	struct device *dev = &ctx->dsi->dev;
	int ret;

	ret = ef71_nt35695_off(ctx);
	if (ret < 0)
		dev_err(dev, "Failed to un-initialize panel: %d\n", ret);

	gpiod_set_value_cansleep(ctx->reset_gpio, 1);

	return 0;
}

static const struct drm_display_mode ef71_nt35695_mode = {
	.clock = (1080 + 180 + 10 + 142) * (1920 + 10 + 2 + 4) * 60 / 1000,
	.hdisplay = 1080,
	.hsync_start = 1080 + 180,
	.hsync_end = 1080 + 180 + 10,
	.htotal = 1080 + 180 + 10 + 142,
	.vdisplay = 1920,
	.vsync_start = 1920 + 10,
	.vsync_end = 1920 + 10 + 2,
	.vtotal = 1920 + 10 + 2 + 4,
	.width_mm = 59,
	.height_mm = 104,
	.type = DRM_MODE_TYPE_DRIVER,
};

static int ef71_nt35695_get_modes(struct drm_panel *panel,
				  struct drm_connector *connector)
{
	return drm_connector_helper_get_modes_fixed(connector, &ef71_nt35695_mode);
}

static const struct drm_panel_funcs ef71_nt35695_panel_funcs = {
	.prepare = ef71_nt35695_prepare,
	.unprepare = ef71_nt35695_unprepare,
	.get_modes = ef71_nt35695_get_modes,
};

static int ef71_nt35695_bl_update_status(struct backlight_device *bl)
{
	struct mipi_dsi_device *dsi = bl_get_data(bl);
	u8 brightness = backlight_get_brightness(bl);
	int ret;

	dsi->mode_flags &= ~MIPI_DSI_MODE_LPM;

	/* One parameter byte, not the 16 bit form the DCS helper sends */
	ret = mipi_dsi_dcs_write(dsi, MIPI_DCS_SET_DISPLAY_BRIGHTNESS,
				 &brightness, sizeof(brightness));

	dsi->mode_flags |= MIPI_DSI_MODE_LPM;

	return ret < 0 ? ret : 0;
}

static const struct backlight_ops ef71_nt35695_bl_ops = {
	.update_status = ef71_nt35695_bl_update_status,
};

static struct backlight_device *
ef71_nt35695_create_backlight(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	const struct backlight_properties props = {
		.type = BACKLIGHT_RAW,
		.brightness = 150,
		.max_brightness = NT35695_BL_MAX,
	};

	return devm_backlight_device_register(dev, dev_name(dev), dev, dsi,
					      &ef71_nt35695_bl_ops, &props);
}

static int ef71_nt35695_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct ef71_nt35695 *ctx;
	int ret;

	ctx = devm_drm_panel_alloc(dev, struct ef71_nt35695, panel,
				   &ef71_nt35695_panel_funcs,
				   DRM_MODE_CONNECTOR_DSI);
	if (IS_ERR(ctx))
		return PTR_ERR(ctx);

	ctx->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(ctx->reset_gpio),
				     "Failed to get reset-gpios\n");

	ctx->dsi = dsi;
	mipi_dsi_set_drvdata(dsi, ctx);

	dsi->lanes = 4;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_CLOCK_NON_CONTINUOUS;

	ctx->panel.prepare_prev_first = true;

	ctx->panel.backlight = ef71_nt35695_create_backlight(dsi);
	if (IS_ERR(ctx->panel.backlight))
		return dev_err_probe(dev, PTR_ERR(ctx->panel.backlight),
				     "Failed to create backlight\n");

	drm_panel_add(&ctx->panel);

	ret = mipi_dsi_attach(dsi);
	if (ret < 0) {
		drm_panel_remove(&ctx->panel);
		return dev_err_probe(dev, ret, "Failed to attach to DSI host\n");
	}

	return 0;
}

static void ef71_nt35695_remove(struct mipi_dsi_device *dsi)
{
	struct ef71_nt35695 *ctx = mipi_dsi_get_drvdata(dsi);
	int ret;

	ret = mipi_dsi_detach(dsi);
	if (ret < 0)
		dev_err(&dsi->dev, "Failed to detach from DSI host: %d\n", ret);

	drm_panel_remove(&ctx->panel);
}

static const struct of_device_id ef71_nt35695_of_match[] = {
	{ .compatible = "pantech,ef71-nt35695" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, ef71_nt35695_of_match);

static struct mipi_dsi_driver ef71_nt35695_driver = {
	.probe = ef71_nt35695_probe,
	.remove = ef71_nt35695_remove,
	.driver = {
		.name = "panel-pantech-ef71-nt35695",
		.of_match_table = ef71_nt35695_of_match,
	},
};
module_mipi_dsi_driver(ef71_nt35695_driver);

MODULE_DESCRIPTION("DRM driver for the Pantech EF71 NT35695 command mode panel");
MODULE_LICENSE("GPL");
