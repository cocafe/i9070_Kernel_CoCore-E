/*
 * Copyright (C) ST-Ericsson SA 2010
 * Author: Pierre Peiffer <pierre.peiffer@stericsson.com> for ST-Ericsson.
 * License terms: GNU General Public License (GPL), version 2.
 */

/** \file configuration.c
 *
 * Nomadik Multiprocessing Framework Linux Driver
 *
 */

#include <linux/module.h>
#include <cm/engine/api/configuration_engine.h>
#include <cm/engine/configuration/inc/configuration_status.h>
#include <cm/engine/power_mgt/inc/power.h>
#include "osal-kernel.h"

/* Per-driver environment */
struct OsalEnvironment osalEnv =
{
	.mpc = {
		{
			.coreId           = SVA_CORE_ID,
			.name             = "sva",
			.base_phys        = (void*)U8500_SVA_BASE,
			.interrupt0       = IRQ_DB8500_SVA,
			.interrupt1       = IRQ_DB8500_SVA2,
			.mmdsp_regulator  = NULL,
			.pipe_regulator   = NULL,
			.monitor_tsk      = NULL,
			.hwmem_code       = NULL,
			.hwmem_data       = NULL,
			.trace_read_count = ATOMIC_INIT(0),
		},
		{
			.coreId           = SIA_CORE_ID,
			.name             = "sia",
			.base_phys        = (void*)U8500_SIA_BASE,
			.interrupt0       = IRQ_DB8500_SIA,
			.interrupt1       = IRQ_DB8500_SIA2,
			.mmdsp_regulator  = NULL,
			.pipe_regulator   = NULL,
			.monitor_tsk      = NULL,
			.hwmem_code       = NULL,
			.hwmem_data       = NULL,
			.trace_read_count = ATOMIC_INIT(0),
		}
	},
	.esram_regulator = { NULL, NULL},
	.dsp_sleep = {
		.sia_auto_pm_enable = PRCMU_AUTO_PM_OFF,
		.sia_power_on       = 0,
		.sia_policy         = PRCMU_AUTO_PM_POLICY_DSP_OFF_HWP_OFF,
		.sva_auto_pm_enable = PRCMU_AUTO_PM_OFF,
		.sva_power_on       = 0,
		.sva_policy         = PRCMU_AUTO_PM_POLICY_DSP_OFF_HWP_OFF,		
	},
	.dsp_idle = {
		.sia_auto_pm_enable = PRCMU_AUTO_PM_OFF,
		.sia_power_on       = 0,
		.sia_policy         = PRCMU_AUTO_PM_POLICY_DSP_OFF_HWP_OFF,
		.sva_auto_pm_enable = PRCMU_AUTO_PM_OFF,
		.sva_power_on       = 0,
		.sva_policy         = PRCMU_AUTO_PM_POLICY_DSP_OFF_HWP_OFF,		
	},
};

module_param_call(cm_debug_level, param_set_int, param_get_int,
                  &cm_debug_level, S_IWUSR|S_IRUGO);
MODULE_PARM_DESC(cm_debug_level, "Debug level of NMF Core");

module_param_call(cm_error_break, param_set_bool, param_get_bool,
                  &cm_error_break, S_IWUSR|S_IRUGO);
MODULE_PARM_DESC(cm_error_break, "Stop on error (in an infinite loop, for debugging purpose)");

module_param_call(cmIntensiveCheckState, param_set_bool, param_get_bool,
                  &cmIntensiveCheckState, S_IWUSR|S_IRUGO);
MODULE_PARM_DESC(cmIntensiveCheckState, "Add additional intensive checks");

DECLARE_MPC_PARAM(SVA, SDRAM_DATA_SIZE, "", 1);

DECLARE_MPC_PARAM(SIA, 0, "\n\t\t\t(0 means shared with SVA)", 2);

bool cfgCommunicationLocationInSDRAM = true;
module_param(cfgCommunicationLocationInSDRAM, bool, S_IRUGO);
MODULE_PARM_DESC(cfgCommunicationLocationInSDRAM, "Location of communications (SDRAM or ESRAM)");

bool cfgSemaphoreTypeHSEM = true;
module_param(cfgSemaphoreTypeHSEM, bool, S_IRUGO);
MODULE_PARM_DESC(cfgSemaphoreTypeHSEM, "Semaphore used (HSEM or LSEM)");

int cfgESRAMSize = ESRAM_SIZE;
module_param(cfgESRAMSize, uint, S_IRUGO);
MODULE_PARM_DESC(cfgESRAMSize, "Size of ESRAM used in the CM (in Kb)");

static int set_param_powerMode(const char *val, const struct kernel_param *kp)
{
	/* No equals means "set"... */
	if (!val) val = "1";

	/* One of =[yYnN01] */
	switch (val[0]) {
	case 'y': case 'Y': case '1':
		CM_ENGINE_SetMode(CM_CMD_DBG_MODE, 0);
		break;
	case 'n': case 'N': case '0':
		CM_ENGINE_SetMode(CM_CMD_DBG_MODE, 1);
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

module_param_call(powerMode, set_param_powerMode, param_get_bool, &powerMode, S_IWUSR|S_IRUGO);
MODULE_PARM_DESC(powerMode, "DSP power mode enable");

int init_config(void)
{
	if (cfgMpcSDRAMCodeSize_SVA == 0 || cfgMpcSDRAMCodeSize_SIA == 0) {
		pr_err("SDRAM code size must be greater than 0\n");
		return -EINVAL;
	}

	if (cfgMpcSDRAMDataSize_SVA == 0) {
		pr_err("SDRAM data size for SVA must be greater than 0\n");
		return -EINVAL;
	}

	osalEnv.mpc[SVA].nbYramBanks     = cfgMpcYBanks_SVA;
	osalEnv.mpc[SVA].eeId            = cfgSchedulerTypeHybrid_SVA ? HYBRID_EXECUTIVE_ENGINE : SYNCHRONOUS_EXECUTIVE_ENGINE;
	osalEnv.mpc[SVA].sdram_code.size = cfgMpcSDRAMCodeSize_SVA * ONE_KB;
	osalEnv.mpc[SVA].sdram_data.size = cfgMpcSDRAMDataSize_SVA * ONE_KB;
	osalEnv.mpc[SVA].base.size       = 128*ONE_KB; //we expose only TCM24
	spin_lock_init(&osalEnv.mpc[SVA].trace_reader_lock);

	osalEnv.mpc[SIA].nbYramBanks     = cfgMpcYBanks_SIA;
	osalEnv.mpc[SIA].eeId            = cfgSchedulerTypeHybrid_SIA ? HYBRID_EXECUTIVE_ENGINE : SYNCHRONOUS_EXECUTIVE_ENGINE;
	osalEnv.mpc[SIA].sdram_code.size = cfgMpcSDRAMCodeSize_SIA * ONE_KB;
	osalEnv.mpc[SIA].sdram_data.size = cfgMpcSDRAMDataSize_SIA * ONE_KB;
	osalEnv.mpc[SIA].base.size       = 128*ONE_KB; //we expose only TCM24
	spin_lock_init(&osalEnv.mpc[SIA].trace_reader_lock);

	return 0;
}
