/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * msm_bus bandwidth votes: mainline has no msm_bus. Accept the votes and
 * drop them; the camera clocks are fixed by their rates.
 */
#ifndef __CAMLM_MSM_BUS_H
#define __CAMLM_MSM_BUS_H

#include <linux/of.h>

#include <linux/types.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

struct msm_bus_vectors {
	int src;
	int dst;
	u64 ab;
	u64 ib;
};

struct msm_bus_paths {
	int num_paths;
	struct msm_bus_vectors *vectors;
};

struct msm_bus_lat_vectors;

struct msm_bus_scale_pdata {
	struct msm_bus_paths *usecase;
	struct msm_bus_lat_vectors *usecase_lat;
	int num_usecases;
	const char *name;
	unsigned int active_only;
};

static inline uint32_t
msm_bus_scale_register_client(struct msm_bus_scale_pdata *pdata)
{
	return 1;
}

static inline int msm_bus_scale_client_update_request(uint32_t cl,
						      unsigned int index)
{
	return 0;
}

static inline void msm_bus_scale_unregister_client(uint32_t cl)
{
}

static inline struct msm_bus_scale_pdata *
msm_bus_cl_get_pdata(struct platform_device *pdev)
{
	/*
	 * Two use cases of one path each, the shape camera_v2 updates
	 * dynamically. Nothing reads the votes back.
	 */
	struct {
		struct msm_bus_scale_pdata pdata;
		struct msm_bus_paths paths[2];
		struct msm_bus_vectors vectors[2];
	} *bus;

	bus = devm_kzalloc(&pdev->dev, sizeof(*bus), GFP_KERNEL);
	if (!bus)
		return NULL;

	bus->paths[0].num_paths = 1;
	bus->paths[0].vectors = &bus->vectors[0];
	bus->paths[1].num_paths = 1;
	bus->paths[1].vectors = &bus->vectors[1];
	bus->pdata.usecase = bus->paths;
	bus->pdata.num_usecases = 2;
	bus->pdata.name = dev_name(&pdev->dev);

	return &bus->pdata;
}

static inline void msm_bus_cl_clear_pdata(struct msm_bus_scale_pdata *pdata)
{
}

#endif
