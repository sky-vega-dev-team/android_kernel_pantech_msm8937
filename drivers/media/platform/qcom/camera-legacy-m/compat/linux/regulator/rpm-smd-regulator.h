/* SPDX-License-Identifier: GPL-2.0-only */
/* RPM voltage corners, as camera_v2 names them */
#ifndef __CAMLM_RPM_SMD_REGULATOR_H
#define __CAMLM_RPM_SMD_REGULATOR_H

enum rpm_regulator_voltage_corner {
	RPM_REGULATOR_CORNER_NONE = 1,
	RPM_REGULATOR_CORNER_RETENTION,
	RPM_REGULATOR_CORNER_SVS_KRAIT,
	RPM_REGULATOR_CORNER_SVS_SOC,
	RPM_REGULATOR_CORNER_NORMAL,
	RPM_REGULATOR_CORNER_TURBO,
	RPM_REGULATOR_CORNER_SUPER_TURBO,
};

enum rpm_regulator_mode {
	RPM_REGULATOR_MODE_AUTO,
	RPM_REGULATOR_MODE_IPEAK,
	RPM_REGULATOR_MODE_HPM,
};

struct rpm_regulator;

/* No RPM-SMD regulator API: report none, the sensor does without */
static inline struct rpm_regulator *rpm_regulator_get(struct device *dev,
						      const char *supply)
{
	return NULL;
}

static inline int rpm_regulator_set_mode(struct rpm_regulator *regulator,
					 enum rpm_regulator_mode mode)
{
	return 0;
}

static inline void rpm_regulator_put(struct rpm_regulator *regulator)
{
}

#endif
