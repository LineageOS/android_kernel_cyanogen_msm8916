/* Copyright (c) 2012-2014, The Linux Foundation. All rights reserved.
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef _MSM_DTS_SRS_TM_CONFIG_H_
#define _MSM_DTS_SRS_TM_CONFIG_H_

#ifdef CONFIG_DTS_SRS_TM

#include <sound/soc.h>

#ifdef CONFIG_DTS_SRS_TM

union srs_trumedia_params_u {
	struct srs_trumedia_params srs_params;
	__u16 raw_params[1];
};

struct param_outband;

void msm_dts_srs_tm_ion_memmap(struct param_outband *po_);
void msm_dts_srs_tm_init(int port_id, int copp_idx);
void msm_dts_srs_tm_deinit(int port_id);
void msm_dts_srs_tm_add_controls(struct snd_soc_platform *platform);
#else
void msm_dts_srs_tm_init(int port_id, int copp_idx) { }
void msm_dts_srs_tm_deinit(int port_id) { }
void msm_dts_srs_tm_add_controls(struct snd_soc_platform *platform) { }

#endif

#endif

#endif

