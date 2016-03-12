/* Copyright (c) 2013-2015, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#define pr_fmt(fmt)	"CHG: %s: " fmt, __func__

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/spmi.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/interrupt.h>
#include <linux/power_supply.h>
#include <linux/qpnp/qpnp-adc.h>
#include <linux/alarmtimer.h>
#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/wakelock.h>
#define CREATE_MASK(NUM_BITS, POS) \
	((unsigned char) (((1 << (NUM_BITS)) - 1) << (POS)))
#define LBC_MASK(MSB_BIT, LSB_BIT) \
	CREATE_MASK(MSB_BIT - LSB_BIT + 1, LSB_BIT)

/* Interrupt offsets */
#define INT_RT_STS_REG				0x10
#define FAST_CHG_ON_IRQ                         BIT(5)
#define OVERTEMP_ON_IRQ				BIT(4)
#define BAT_TEMP_OK_IRQ                         BIT(1)
#define BATT_PRES_IRQ                           BIT(0)

/* USB CHARGER PATH peripheral register offsets */
#define USB_PTH_STS_REG				0x09
#define USB_IN_VALID_MASK			BIT(1)
#define USB_SUSP_REG				0x47
#define USB_SUSPEND_BIT				BIT(0)

/* CHARGER peripheral register offset */
#define CHG_OPTION_REG				0x08
#define CHG_OPTION_MASK				BIT(7)
#define CHG_STATUS_REG				0x09
#define CHG_VDD_LOOP_BIT			BIT(1)
#define CHG_VDD_MAX_REG				0x40
#define CHG_VDD_SAFE_REG			0x41
#define CHG_IBAT_MAX_REG			0x44
#define CHG_IBAT_SAFE_REG			0x45
#define CHG_VIN_MIN_REG				0x47
#define CHG_CTRL_REG				0x49
#define CHG_ENABLE				BIT(7)
#define CHG_FORCE_BATT_ON			BIT(0)
#define CHG_EN_MASK				(BIT(7) | BIT(0))
#define CHG_EN_MASK_1				BIT(0)
#define CHG_FAILED_REG				0x4A
#define CHG_FAILED_BIT				BIT(7)
#define CHG_VBAT_WEAK_REG			0x52
#define CHG_IBATTERM_EN_REG			0x5B
#define CHG_USB_ENUM_T_STOP_REG			0x4E
#define CHG_TCHG_MAX_EN_REG			0x60
#define CHG_TCHG_MAX_EN_BIT			BIT(7)
#define CHG_TCHG_MAX_MASK			LBC_MASK(6, 0)
#define CHG_TCHG_MAX_REG			0x61
#define CHG_WDOG_EN_REG				0x65
#define CHG_PERPH_RESET_CTRL3_REG		0xDA
#define CHG_COMP_OVR1				0xEE
#define CHG_VBAT_DET_OVR_MASK			LBC_MASK(1, 0)
#define OVERRIDE_0				0x2
#define OVERRIDE_NONE				0x0

/* BATTIF peripheral register offset */
#define BAT_IF_PRES_STATUS_REG			0x08
#define BATT_PRES_MASK				BIT(7)
#define BAT_IF_TEMP_STATUS_REG			0x09
#define BATT_TEMP_HOT_MASK			BIT(6)
#define BATT_TEMP_COLD_MASK			LBC_MASK(7, 6)
#define BATT_TEMP_OK_MASK			BIT(7)
#define BAT_IF_VREF_BAT_THM_CTRL_REG		0x4A
#define VREF_BATT_THERM_FORCE_ON		LBC_MASK(7, 6)
#define VREF_BAT_THM_ENABLED_FSM		BIT(7)
#define BAT_IF_BPD_CTRL_REG			0x48
#define BATT_BPD_CTRL_SEL_MASK			LBC_MASK(1, 0)
#define BATT_BPD_OFFMODE_EN			BIT(3)
#define BATT_THM_EN				BIT(1)
#define BATT_ID_EN				BIT(0)
#define BAT_IF_BTC_CTRL				0x49
#define BTC_COMP_EN_MASK			BIT(7)
#define BTC_COLD_MASK				BIT(1)
#define BTC_HOT_MASK				BIT(0)

/* MISC peripheral register offset */
#define MISC_REV2_REG				0x01
#define MISC_BOOT_DONE_REG			0x42
#define MISC_BOOT_DONE				BIT(7)
#define MISC_TRIM3_REG				0xF3
#define MISC_TRIM3_VDD_MASK			LBC_MASK(5, 4)
#define MISC_TRIM4_REG				0xF4
#define MISC_TRIM4_VDD_MASK			BIT(4)

#define PERP_SUBTYPE_REG			0x05
#define SEC_ACCESS                              0xD0

/* Linear peripheral subtype values */
#define LBC_CHGR_SUBTYPE			0x15
#define LBC_BAT_IF_SUBTYPE			0x16
#define LBC_USB_PTH_SUBTYPE			0x17
#define LBC_MISC_SUBTYPE			0x18

#define QPNP_CHG_I_MAX_MIN_90                   90

/* Feature flags */
#define VDD_TRIM_SUPPORTED			BIT(0)

#define QPNP_CHARGER_DEV_NAME	"qcom,qpnp-linear-charger"

/* usb_interrupts */

struct qpnp_lbc_irq {
	int		irq;
	unsigned long	disabled;
	bool            is_wake;
};

enum {
	USBIN_VALID = 0,
	USB_OVER_TEMP,
	USB_CHG_GONE,
	BATT_PRES,
	BATT_TEMPOK,
	CHG_DONE,
	CHG_FAILED,
	CHG_FAST_CHG,
	CHG_VBAT_DET_LO,
	MAX_IRQS,
};

enum {
	USER	= BIT(0),
	THERMAL = BIT(1),
	CURRENT = BIT(2),
	SOC	= BIT(3),
};

enum bpd_type {
	BPD_TYPE_BAT_ID,
	BPD_TYPE_BAT_THM,
	BPD_TYPE_BAT_THM_BAT_ID,
};

static const char * const bpd_label[] = {
	[BPD_TYPE_BAT_ID] = "bpd_id",
	[BPD_TYPE_BAT_THM] = "bpd_thm",
	[BPD_TYPE_BAT_THM_BAT_ID] = "bpd_thm_id",
};

enum btc_type {
	HOT_THD_25_PCT = 25,
	HOT_THD_35_PCT = 35,
	COLD_THD_70_PCT = 70,
	COLD_THD_80_PCT = 80,
};

static u8 btc_value[] = {
	[HOT_THD_25_PCT] = 0x0,
	[HOT_THD_35_PCT] = BIT(0),
	[COLD_THD_70_PCT] = 0x0,
	[COLD_THD_80_PCT] = BIT(1),
};

static inline int get_bpd(const char *name)
{
	int i = 0;
	for (i = 0; i < ARRAY_SIZE(bpd_label); i++) {
		if (strcmp(bpd_label[i], name) == 0)
			return i;
	}
	return -EINVAL;
}

static enum power_supply_property msm_batt_power_props[] = {
	POWER_SUPPLY_PROP_CHARGING_ENABLED,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_COOL_TEMP,
	POWER_SUPPLY_PROP_WARM_TEMP,
	POWER_SUPPLY_PROP_SYSTEM_TEMP_LEVEL,
	POWER_SUPPLY_PROP_INPUT_CURRENT_MAX,
};

static char *pm_batt_supplied_to[] = {
	"bms",
};

struct vddtrim_map {
	int			trim_uv;
	int			trim_val;
};

/*
 * VDDTRIM is a 3 bit value which is split across two
 * register TRIM3(bit 5:4)	-> VDDTRIM bit(2:1)
 * register TRIM4(bit 4)	-> VDDTRIM bit(0)
 */
#define TRIM_CENTER			4
#define MAX_VDD_EA_TRIM_CFG		8
#define VDD_TRIM3_MASK			LBC_MASK(2, 1)
#define VDD_TRIM3_SHIFT			3
#define VDD_TRIM4_MASK			BIT(0)
#define VDD_TRIM4_SHIFT			4
#define AVG(VAL1, VAL2)			((VAL1 + VAL2) / 2)

/*
 * VDDTRIM table containing map of trim voltage and
 * corresponding trim value.
 */
struct vddtrim_map vddtrim_map[] = {
	{36700,		0x00},
	{28000,		0x01},
	{19800,		0x02},
	{10760,		0x03},
	{0,		0x04},
	{-8500,		0x05},
	{-16800,	0x06},
	{-25440,	0x07},
};
static struct wake_lock 	dc_chg_wake_lock;
static bool dc_chg_lock=false;
static int runin_on  = 1;
static int temp_on = 1;
static int charging_on = 1;
static bool input_current =false;
static int limit_on = 1;
extern int recharge_trigger ;
extern int report_full ;
/*
 * struct qpnp_lbc_chip - device information
 * @dev:			device pointer to access the parent
 * @spmi:			spmi pointer to access spmi information
 * @chgr_base:			charger peripheral base address
 * @bat_if_base:		battery interface  peripheral base address
 * @usb_chgpth_base:		USB charge path peripheral base address
 * @misc_base:			misc peripheral base address
 * @bat_is_cool:		indicates that battery is cool
 * @bat_is_warm:		indicates that battery is warm
 * @chg_done:			indicates that charging is completed
 * @usb_present:		present status of USB
 * @batt_present:		present status of battery
 * @cfg_charging_disabled:	disable drawing current from USB.
 * @cfg_use_fake_battery:	flag to report default battery properties
 * @fastchg_on:			indicate charger in fast charge mode
 * @cfg_btc_disabled:		flag to disable btc (disables hot and cold
 *				irqs)
 * @cfg_max_voltage_mv:		the max volts the batt should be charged up to
 * @cfg_min_voltage_mv:		VIN_MIN configuration
 * @cfg_batt_weak_voltage_uv:	weak battery voltage threshold
 * @cfg_warm_bat_chg_ma:	warm battery maximum charge current in mA
 * @cfg_cool_bat_chg_ma:	cool battery maximum charge current in mA
 * @cfg_safe_voltage_mv:	safe voltage to which battery can charge
 * @cfg_warm_bat_mv:		warm temperature battery target voltage
 * @cfg_warm_bat_mv:		warm temperature battery target voltage
 * @cfg_cool_bat_mv:		cool temperature battery target voltage
 * @cfg_soc_resume_limit:	SOC at which battery resumes charging
 * @cfg_float_charge:		enable float charging
 * @charger_disabled:		maintain USB path state.
 * @cfg_charger_detect_eoc:	charger can detect end of charging
 * @cfg_disable_vbatdet_based_recharge:	keep VBATDET comparator overriden to
 *				low and VBATDET irq disabled.
 * @cfg_chgr_led_support:	support charger led work.
 * @cfg_safe_current:		battery safety current setting
 * @cfg_hot_batt_p:		hot battery threshold setting
 * @cfg_cold_batt_p:		eold battery threshold setting
 * @cfg_warm_bat_decidegc:	warm battery temperature in degree Celsius
 * @cfg_cool_bat_decidegc:	cool battery temperature in degree Celsius
 * @fake_battery_soc:		SOC value to be reported to userspace
 * @cfg_tchg_mins:		maximum allowed software initiated charge time
 * @chg_failed_count:		counter to maintained number of times charging
 *				failed
 * @cfg_bpd_detection:		battery present detection mechanism selection
 * @cfg_thermal_levels:		amount of thermal mitigation levels
 * @cfg_thermal_mitigation:	thermal mitigation level values
 * @therm_lvl_sel:		thermal mitigation level selection
 * @jeita_configure_lock:	lock to serialize jeita configuration request
 * @hw_access_lock:		lock to serialize access to charger registers
 * @ibat_change_lock:		lock to serialize ibat change requests from
 *				USB and thermal.
 * @irq_lock			lock to serialize enabling/disabling of irq
 * @supported_feature_flag	bitmask for all supported features
 * @vddtrim_alarm		alarm to schedule trim work at regular
 *				interval
 * @vddtrim_work		work to perform actual vddmax trimming
 * @init_trim_uv		initial trim voltage at bootup
 * @delta_vddmax_uv		current vddmax trim voltage
 * @chg_enable_lock:		lock to serialize charging enable/disable for
 *				SOC based resume charging
 * @usb_psy:			power supply to export information to
 *				userspace
 * @bms_psy:			power supply to export information to
 *				userspace
 * @batt_psy:			power supply to export information to
 *				userspace
 */
struct qpnp_lbc_chip {
	struct device			*dev;
	struct spmi_device		*spmi;
	u16				chgr_base;
	u16				bat_if_base;
	u16				usb_chgpth_base;
	u16				misc_base;
	bool				bat_is_cool;
	bool				bat_is_warm;
	bool				chg_done;
	bool				usb_present;
	bool				batt_present;
	bool				cfg_charging_disabled;
	bool				cfg_btc_disabled;
	bool				cfg_use_fake_battery;
	bool				fastchg_on;
	bool				cfg_use_external_charger;
	unsigned int			cfg_warm_bat_chg_ma;
	unsigned int			cfg_cool_bat_chg_ma;
	unsigned int			cfg_safe_voltage_mv;
	unsigned int			cfg_max_voltage_mv;
	unsigned int			cfg_min_voltage_mv;
	unsigned int			cfg_charger_detect_eoc;
	unsigned int			cfg_disable_vbatdet_based_recharge;
	unsigned int			cfg_batt_weak_voltage_uv;
	unsigned int			cfg_warm_bat_mv;
	unsigned int			cfg_cool_bat_mv;
	unsigned int			cfg_hot_batt_p;
	unsigned int			cfg_cold_batt_p;
	unsigned int			cfg_thermal_levels;
	unsigned int			therm_lvl_sel;
	unsigned int			*thermal_mitigation;
	unsigned int			cfg_safe_current;
	unsigned int			cfg_tchg_mins;
	unsigned int			chg_failed_count;
	unsigned int			cfg_disable_follow_on_reset;
	unsigned int			supported_feature_flag;
	int				cfg_bpd_detection;
	int				cfg_warm_bat_decidegc;
	int				cfg_cool_bat_decidegc;
	int				fake_battery_soc;
	int				cfg_soc_resume_limit;
	int				cfg_float_charge;
	int				charger_disabled;
	int				prev_max_ma;
	int				usb_psy_ma;
	int				delta_vddmax_uv;
	int				init_trim_uv;
	struct alarm			vddtrim_alarm;
	struct work_struct		vddtrim_work;
	struct qpnp_lbc_irq		irqs[MAX_IRQS];
	struct delayed_work		charger_work;
	struct delayed_work		batt_pres_work;
	struct mutex			jeita_configure_lock;
	struct mutex			chg_enable_lock;
	spinlock_t			ibat_change_lock;
	spinlock_t			hw_access_lock;
	spinlock_t			irq_lock;
	struct power_supply		*usb_psy;
	struct power_supply		*bms_psy;
	struct power_supply		batt_psy;
	struct qpnp_adc_tm_btm_param	adc_param;
	struct qpnp_vadc_chip		*vadc_dev;
	struct qpnp_adc_tm_chip		*adc_tm_dev;
};

static void qpnp_lbc_enable_irq(struct qpnp_lbc_chip *chip,
		struct qpnp_lbc_irq *irq)
{
	unsigned long flags;

	spin_lock_irqsave(&chip->irq_lock, flags);
	if (__test_and_clear_bit(0, &irq->disabled)) {
		pr_debug("number = %d\n", irq->irq);
		enable_irq(irq->irq);
		if (irq->is_wake)
			enable_irq_wake(irq->irq);
	}
	spin_unlock_irqrestore(&chip->irq_lock, flags);
}

static void qpnp_lbc_disable_irq(struct qpnp_lbc_chip *chip,
		struct qpnp_lbc_irq *irq)
{
	unsigned long flags;

	spin_lock_irqsave(&chip->irq_lock, flags);
	if (!__test_and_set_bit(0, &irq->disabled)) {
		pr_debug("number = %d\n", irq->irq);
		disable_irq_nosync(irq->irq);
		if (irq->is_wake)
			disable_irq_wake(irq->irq);
	}
	spin_unlock_irqrestore(&chip->irq_lock, flags);
}

static int __qpnp_lbc_read(struct spmi_device *spmi, u16 base,
		u8 *val, int count)
{
	int rc = 0;

	rc = spmi_ext_register_readl(spmi->ctrl, spmi->sid, base, val, count);
	if (rc)
		pr_err("SPMI read failed base=0x%02x sid=0x%02x rc=%d\n",
				base, spmi->sid, rc);

	return rc;
}

static int __qpnp_lbc_write(struct spmi_device *spmi, u16 base,
		u8 *val, int count)
{
	int rc;

	rc = spmi_ext_register_writel(spmi->ctrl, spmi->sid, base, val,
			count);
	if (rc)
		pr_err("SPMI write failed base=0x%02x sid=0x%02x rc=%d\n",
				base, spmi->sid, rc);

	return rc;
}

static int __qpnp_lbc_secure_write(struct spmi_device *spmi, u16 base,
		u16 offset, u8 *val, int count)
{
	int rc;
	u8 reg_val;

	reg_val = 0xA5;
	rc = __qpnp_lbc_write(spmi, base + SEC_ACCESS, &reg_val, 1);
	if (rc) {
		pr_err("SPMI read failed base=0x%02x sid=0x%02x rc=%d\n",
				base + SEC_ACCESS, spmi->sid, rc);
		return rc;
	}

	rc = __qpnp_lbc_write(spmi, base + offset, val, 1);
	if (rc)
		pr_err("SPMI write failed base=0x%02x sid=0x%02x rc=%d\n",
				base + SEC_ACCESS, spmi->sid, rc);

	return rc;
}

static int qpnp_lbc_read(struct qpnp_lbc_chip *chip, u16 base,
		u8 *val, int count)
{
	int rc = 0;
	struct spmi_device *spmi = chip->spmi;
	unsigned long flags;

	if (base == 0) {
		pr_err("base cannot be zero base=0x%02x sid=0x%02x rc=%d\n",
				base, spmi->sid, rc);
		return -EINVAL;
	}

	spin_lock_irqsave(&chip->hw_access_lock, flags);
	rc = __qpnp_lbc_read(spmi, base, val, count);
	spin_unlock_irqrestore(&chip->hw_access_lock, flags);

	return rc;
}

static int qpnp_lbc_write(struct qpnp_lbc_chip *chip, u16 base,
		u8 *val, int count)
{
	int rc = 0;
	struct spmi_device *spmi = chip->spmi;
	unsigned long flags;

	if (base == 0) {
		pr_err("base cannot be zero base=0x%02x sid=0x%02x rc=%d\n",
				base, spmi->sid, rc);
		return -EINVAL;
	}

	spin_lock_irqsave(&chip->hw_access_lock, flags);
	rc = __qpnp_lbc_write(spmi, base, val, count);
	spin_unlock_irqrestore(&chip->hw_access_lock, flags);

	return rc;
}

static int qpnp_lbc_masked_write(struct qpnp_lbc_chip *chip, u16 base,
		u8 mask, u8 val)
{
	int rc;
	u8 reg_val;
	struct spmi_device *spmi = chip->spmi;
	unsigned long flags;

	spin_lock_irqsave(&chip->hw_access_lock, flags);
	rc = __qpnp_lbc_read(spmi, base, &reg_val, 1);
	if (rc) {
		pr_err("spmi read failed: addr=%03X, rc=%d\n", base, rc);
		goto out;
	}
	pr_debug("addr = 0x%x read 0x%x\n", base, reg_val);

	reg_val &= ~mask;
	reg_val |= val & mask;

	pr_debug("writing to base=%x val=%x\n", base, reg_val);

	rc = __qpnp_lbc_write(spmi, base, &reg_val, 1);
	if (rc)
		pr_err("spmi write failed: addr=%03X, rc=%d\n", base, rc);

out:
	spin_unlock_irqrestore(&chip->hw_access_lock, flags);
	return rc;
}

static int __qpnp_lbc_secure_masked_write(struct spmi_device *spmi, u16 base,
		u16 offset, u8 mask, u8 val)
{
	int rc;
	u8 reg_val, reg_val1;

	rc = __qpnp_lbc_read(spmi, base + offset, &reg_val, 1);
	if (rc) {
		pr_err("spmi read failed: addr=%03X, rc=%d\n", base, rc);
		return rc;
	}
	pr_debug("addr = 0x%x read 0x%x\n", base, reg_val);

	reg_val &= ~mask;
	reg_val |= val & mask;
	pr_debug("writing to base=%x val=%x\n", base, reg_val);

	reg_val1 = 0xA5;
	rc = __qpnp_lbc_write(spmi, base + SEC_ACCESS, &reg_val1, 1);
	if (rc) {
		pr_err("SPMI read failed base=0x%02x sid=0x%02x rc=%d\n",
				base + SEC_ACCESS, spmi->sid, rc);
		return rc;
	}

	rc = __qpnp_lbc_write(spmi, base + offset, &reg_val, 1);
	if (rc) {
		pr_err("SPMI write failed base=0x%02x sid=0x%02x rc=%d\n",
				base + offset, spmi->sid, rc);
		return rc;
	}

	return rc;
}

static int qpnp_lbc_get_trim_voltage(u8 trim_reg)
{
	int i;

	for (i = 0; i < MAX_VDD_EA_TRIM_CFG; i++)
		if (trim_reg == vddtrim_map[i].trim_val)
			return vddtrim_map[i].trim_uv;

	pr_err("Invalid trim reg reg_val=%x\n", trim_reg);
	return -EINVAL;
}

static u8 qpnp_lbc_get_trim_val(struct qpnp_lbc_chip *chip)
{
	int i, sign;
	int delta_uv;

	sign = (chip->delta_vddmax_uv >= 0) ? -1 : 1;

	switch (sign) {
		case -1:
			for (i = TRIM_CENTER; i >= 0; i--) {
				if (vddtrim_map[i].trim_uv > chip->delta_vddmax_uv) {
					delta_uv = AVG(vddtrim_map[i].trim_uv,
							vddtrim_map[i + 1].trim_uv);
					if (chip->delta_vddmax_uv >= delta_uv)
						return vddtrim_map[i].trim_val;
					else
						return vddtrim_map[i + 1].trim_val;
				}
			}
			break;
		case 1:
			for (i = TRIM_CENTER; i <= 7; i++) {
				if (vddtrim_map[i].trim_uv < chip->delta_vddmax_uv) {
					delta_uv = AVG(vddtrim_map[i].trim_uv,
							vddtrim_map[i - 1].trim_uv);
					if (chip->delta_vddmax_uv >= delta_uv)
						return vddtrim_map[i - 1].trim_val;
					else
						return vddtrim_map[i].trim_val;
				}
			}
			break;
	}

	return vddtrim_map[i].trim_val;
}

static int qpnp_lbc_is_usb_chg_plugged_in(struct qpnp_lbc_chip *chip)
{
	u8 usbin_valid_rt_sts;
	int rc;

	rc = qpnp_lbc_read(chip, chip->usb_chgpth_base + INT_RT_STS_REG,
			&usbin_valid_rt_sts, 1);
	if (rc) {
		pr_err("spmi read failed: addr=%03X, rc=%d\n",
				chip->usb_chgpth_base + INT_RT_STS_REG, rc);
		return rc;
	}

	pr_debug("rt_sts 0x%x\n", usbin_valid_rt_sts);

	return (usbin_valid_rt_sts & USB_IN_VALID_MASK) ? 1 : 0;
}

static int qpnp_lbc_charger_enable(struct qpnp_lbc_chip *chip, int reason,
		int enable)
{
	int disabled = chip->charger_disabled;
	u8 reg_val;
	int rc = 0;
	u8 battery_present_rt_sts;

	pr_debug("reason=%d requested_enable=%d disabled_status=%d\n",
			reason, enable, disabled);
	if (enable)
		disabled &= ~reason;
	else
		disabled |= reason;
	rc = qpnp_lbc_read(chip, chip->bat_if_base + BAT_IF_PRES_STATUS_REG,
			&battery_present_rt_sts, 1);
	if (rc) {
		pr_err("spmi read failed: addr=%0X, rc=%d\n",
				battery_present_rt_sts, rc);
		return rc;
	}
	if(battery_present_rt_sts & 0x02){
		rc = qpnp_lbc_masked_write(chip, chip->chgr_base + CHG_CTRL_REG,
				CHG_EN_MASK_1, (0x00));
	}else{
		rc = qpnp_lbc_masked_write(chip, chip->chgr_base + CHG_CTRL_REG,
				CHG_EN_MASK_1, (0x01));
	}
	if (!!chip->charger_disabled == !!disabled)
		goto skip;

	reg_val = !!disabled ? CHG_FORCE_BATT_ON : CHG_ENABLE;
	rc = qpnp_lbc_masked_write(chip, chip->chgr_base + CHG_CTRL_REG,
			CHG_EN_MASK, reg_val);
	if (rc) {
		pr_err("Failed to %s charger rc=%d\n",
				reg_val ? "enable" : "disable", rc);
		return rc;
	}
skip:
	chip->charger_disabled = disabled;
	return rc;
}

static int qpnp_lbc_is_batt_present(struct qpnp_lbc_chip *chip)
{
	u8 batt_pres_rt_sts;
	int rc;

	rc = qpnp_lbc_read(chip, chip->bat_if_base + INT_RT_STS_REG,
			&batt_pres_rt_sts, 1);
	if (rc) {
		pr_err("spmi read failed: addr=%03X, rc=%d\n",
				chip->bat_if_base + INT_RT_STS_REG, rc);
		return rc;
	}

	return (batt_pres_rt_sts & BATT_PRES_IRQ) ? 1 : 0;
}

static int qpnp_lbc_bat_if_configure_btc(struct qpnp_lbc_chip *chip)
{
	u8 btc_cfg = 0, mask = 0, rc;

	/* Do nothing if battery peripheral not present */
	if (!chip->bat_if_base)
		return 0;

	if ((chip->cfg_hot_batt_p == HOT_THD_25_PCT)
			|| (chip->cfg_hot_batt_p == HOT_THD_35_PCT)) {
		btc_cfg |= btc_value[chip->cfg_hot_batt_p];
		mask |= BTC_HOT_MASK;
	}

	if ((chip->cfg_cold_batt_p == COLD_THD_70_PCT) ||
			(chip->cfg_cold_batt_p == COLD_THD_80_PCT)) {
		btc_cfg |= btc_value[chip->cfg_cold_batt_p];
		mask |= BTC_COLD_MASK;
	}
	if (chip->cfg_btc_disabled) {
		mask |= BTC_COMP_EN_MASK;
	}
	pr_debug("BTC configuration mask=%x\n", btc_cfg);

	rc = qpnp_lbc_masked_write(chip,
			chip->bat_if_base + BAT_IF_BTC_CTRL,
			mask, btc_cfg);
	if (rc)
		pr_err("Failed to configure BTC rc=%d\n", rc);

	return rc;
}

#define QPNP_LBC_VBATWEAK_MIN_UV        3000000
#define QPNP_LBC_VBATWEAK_MAX_UV        3581250
#define QPNP_LBC_VBATWEAK_STEP_UV       18750
static int qpnp_lbc_vbatweak_set(struct qpnp_lbc_chip *chip, int voltage)
{
	u8 reg_val;
	int rc;

	if (voltage < QPNP_LBC_VBATWEAK_MIN_UV ||
			voltage > QPNP_LBC_VBATWEAK_MAX_UV) {
		rc = -EINVAL;
	} else {
		reg_val = (voltage - QPNP_LBC_VBATWEAK_MIN_UV) /
			QPNP_LBC_VBATWEAK_STEP_UV;
		pr_debug("VBAT_WEAK=%d setting %02x\n",
				chip->cfg_batt_weak_voltage_uv, reg_val);
		rc = qpnp_lbc_write(chip, chip->chgr_base + CHG_VBAT_WEAK_REG,
				&reg_val, 1);
		if (rc)
			pr_err("Failed to set VBAT_WEAK rc=%d\n", rc);
	}

	return rc;
}

#define QPNP_LBC_VBAT_MIN_MV		4000
#define QPNP_LBC_VBAT_MAX_MV		4775
#define QPNP_LBC_VBAT_STEP_MV		25
static int qpnp_lbc_vddsafe_set(struct qpnp_lbc_chip *chip, int voltage)
{
	u8 reg_val;
	int rc;

	if (voltage < QPNP_LBC_VBAT_MIN_MV
			|| voltage > QPNP_LBC_VBAT_MAX_MV) {
		pr_err("bad mV=%d asked to set\n", voltage);
		return -EINVAL;
	}
	reg_val = (voltage - QPNP_LBC_VBAT_MIN_MV) / QPNP_LBC_VBAT_STEP_MV;
	pr_debug("voltage=%d setting %02x\n", voltage, reg_val);
	rc = qpnp_lbc_write(chip, chip->chgr_base + CHG_VDD_SAFE_REG,
			&reg_val, 1);
	if (rc)
		pr_err("Failed to set VDD_SAFE rc=%d\n", rc);

	return rc;
}

static int qpnp_lbc_vddmax_set(struct qpnp_lbc_chip *chip, int voltage)
{
	u8 reg_val;
	int rc, trim_val;
	unsigned long flags;

	if (voltage < QPNP_LBC_VBAT_MIN_MV
			|| voltage > QPNP_LBC_VBAT_MAX_MV) {
		pr_err("bad mV=%d asked to set\n", voltage);
		return -EINVAL;
	}

	spin_lock_irqsave(&chip->hw_access_lock, flags);
	reg_val = (voltage - QPNP_LBC_VBAT_MIN_MV) / QPNP_LBC_VBAT_STEP_MV;
	pr_debug("voltage=%d setting %02x\n", voltage, reg_val);
	rc = __qpnp_lbc_write(chip->spmi, chip->chgr_base + CHG_VDD_MAX_REG,
			&reg_val, 1);
	if (rc) {
		pr_err("Failed to set VDD_MAX rc=%d\n", rc);
		goto out;
	}

	/* Update trim value */
	if (chip->supported_feature_flag & VDD_TRIM_SUPPORTED) {
		trim_val = qpnp_lbc_get_trim_val(chip);
		reg_val = (trim_val & VDD_TRIM3_MASK) << VDD_TRIM3_SHIFT;
		rc = __qpnp_lbc_secure_masked_write(chip->spmi,
				chip->misc_base, MISC_TRIM3_REG,
				MISC_TRIM3_VDD_MASK, reg_val);
		if (rc) {
			pr_err("Failed to set MISC_TRIM3_REG rc=%d\n", rc);
			goto out;
		}

		reg_val = (trim_val & VDD_TRIM4_MASK) << VDD_TRIM4_SHIFT;
		rc = __qpnp_lbc_secure_masked_write(chip->spmi,
				chip->misc_base, MISC_TRIM4_REG,
				MISC_TRIM4_VDD_MASK, reg_val);
		if (rc) {
			pr_err("Failed to set MISC_TRIM4_REG rc=%d\n", rc);
			goto out;
		}

		chip->delta_vddmax_uv = qpnp_lbc_get_trim_voltage(trim_val);
		if (chip->delta_vddmax_uv == -EINVAL) {
			pr_err("Invalid trim voltage=%d\n",
					chip->delta_vddmax_uv);
			rc = -EINVAL;
			goto out;
		}

		pr_debug("VDD_MAX delta=%d trim value=%x\n",
				chip->delta_vddmax_uv, trim_val);
	}

out:
	spin_unlock_irqrestore(&chip->hw_access_lock, flags);
	return rc;
}

static int qpnp_lbc_set_appropriate_vddmax(struct qpnp_lbc_chip *chip)
{
	int rc;

	if (chip->bat_is_cool)
		rc = qpnp_lbc_vddmax_set(chip, chip->cfg_cool_bat_mv);
	else if (chip->bat_is_warm)
		rc = qpnp_lbc_vddmax_set(chip, chip->cfg_warm_bat_mv);
	else
		rc = qpnp_lbc_vddmax_set(chip, chip->cfg_max_voltage_mv);
	if (rc)
		pr_err("Failed to set appropriate vddmax rc=%d\n", rc);

	return rc;
}

#define QPNP_LBC_MIN_DELTA_UV			13000
static void qpnp_lbc_adjust_vddmax(struct qpnp_lbc_chip *chip, int vbat_uv)
{
	int delta_uv, prev_delta_uv, rc;

	prev_delta_uv =  chip->delta_vddmax_uv;
	delta_uv = (int)(chip->cfg_max_voltage_mv * 1000) - vbat_uv;

	/*
	 * If delta_uv is positive, apply trim if delta_uv > 13mv
	 * If delta_uv is negative always apply trim.
	 */
	if (delta_uv > 0 && delta_uv < QPNP_LBC_MIN_DELTA_UV) {
		pr_debug("vbat is not low enough to increase vdd\n");
		return;
	}

	pr_debug("vbat=%d current delta_uv=%d prev delta_vddmax_uv=%d\n",
			vbat_uv, delta_uv, chip->delta_vddmax_uv);
	chip->delta_vddmax_uv = delta_uv + chip->delta_vddmax_uv;
	pr_debug("new delta_vddmax_uv  %d\n", chip->delta_vddmax_uv);
	rc = qpnp_lbc_set_appropriate_vddmax(chip);
	if (rc) {
		pr_err("Failed to set appropriate vddmax rc=%d\n", rc);
		chip->delta_vddmax_uv = prev_delta_uv;
	}
}

#define QPNP_LBC_VINMIN_MIN_MV		4200
#define QPNP_LBC_VINMIN_MAX_MV		5037
#define QPNP_LBC_VINMIN_STEP_MV		27
static int qpnp_lbc_vinmin_set(struct qpnp_lbc_chip *chip, int voltage)
{
	u8 reg_val;
	int rc;

	if ((voltage < QPNP_LBC_VINMIN_MIN_MV)
			|| (voltage > QPNP_LBC_VINMIN_MAX_MV)) {
		pr_err("bad mV=%d asked to set\n", voltage);
		return -EINVAL;
	}

	reg_val = (voltage - QPNP_LBC_VINMIN_MIN_MV) / QPNP_LBC_VINMIN_STEP_MV;
	pr_debug("VIN_MIN=%d setting %02x\n", voltage, reg_val);
	rc = qpnp_lbc_write(chip, chip->chgr_base + CHG_VIN_MIN_REG,
			&reg_val, 1);
	if (rc)
		pr_err("Failed to set VIN_MIN rc=%d\n", rc);

	return rc;
}

#define QPNP_LBC_IBATSAFE_MIN_MA	90
#define QPNP_LBC_IBATSAFE_MAX_MA	1440
#define QPNP_LBC_I_STEP_MA		90
static int qpnp_lbc_ibatsafe_set(struct qpnp_lbc_chip *chip, int safe_current)
{
	u8 reg_val;
	int rc;

	if (safe_current < QPNP_LBC_IBATSAFE_MIN_MA
			|| safe_current > QPNP_LBC_IBATSAFE_MAX_MA) {
		pr_err("bad mA=%d asked to set\n", safe_current);
		return -EINVAL;
	}

	reg_val = (safe_current - QPNP_LBC_IBATSAFE_MIN_MA)
		/ QPNP_LBC_I_STEP_MA;
	pr_debug("Ibate_safe=%d setting %02x\n", safe_current, reg_val);

	rc = qpnp_lbc_write(chip, chip->chgr_base + CHG_IBAT_SAFE_REG,
			&reg_val, 1);
	if (rc)
		pr_err("Failed to set IBAT_SAFE rc=%d\n", rc);

	return rc;
}

#define QPNP_LBC_IBATMAX_MIN	90
#define QPNP_LBC_IBATMAX_MAX	1440
/*
 * Set maximum current limit from charger
 * ibat =  System current + charging current
 */
static int qpnp_lbc_ibatmax_set(struct qpnp_lbc_chip *chip, int chg_current)
{
	u8 reg_val;
	int rc;

	if (chg_current > QPNP_LBC_IBATMAX_MAX)
		pr_debug("bad mA=%d clamping current\n", chg_current);

	chg_current = clamp(chg_current, QPNP_LBC_IBATMAX_MIN,
			QPNP_LBC_IBATMAX_MAX);
	reg_val = (chg_current - QPNP_LBC_IBATMAX_MIN) / QPNP_LBC_I_STEP_MA;

	rc = qpnp_lbc_write(chip, chip->chgr_base + CHG_IBAT_MAX_REG,
			&reg_val, 1);
	if (rc)
		pr_err("Failed to set IBAT_MAX rc=%d\n", rc);
	else
		chip->prev_max_ma = chg_current;

	return rc;
}

#define QPNP_LBC_TCHG_MIN	4
#define QPNP_LBC_TCHG_MAX	512
#define QPNP_LBC_TCHG_STEP	4
static int qpnp_lbc_tchg_max_set(struct qpnp_lbc_chip *chip, int minutes)
{
	u8 reg_val = 0;
	int rc;

	minutes = clamp(minutes, QPNP_LBC_TCHG_MIN, QPNP_LBC_TCHG_MAX);

	/* Disable timer */
	rc = qpnp_lbc_masked_write(chip, chip->chgr_base + CHG_TCHG_MAX_EN_REG,
			CHG_TCHG_MAX_EN_BIT, 0);
	if (rc) {
		pr_err("Failed to write tchg_max_en rc=%d\n", rc);
		return rc;
	}

	reg_val = (minutes / QPNP_LBC_TCHG_STEP) - 1;

	pr_debug("TCHG_MAX=%d mins setting %x\n", minutes, reg_val);
	rc = qpnp_lbc_masked_write(chip, chip->chgr_base + CHG_TCHG_MAX_REG,
			CHG_TCHG_MAX_MASK, reg_val);
	if (rc) {
		pr_err("Failed to write tchg_max_reg rc=%d\n", rc);
		return rc;
	}

	/* Enable timer */
	rc = qpnp_lbc_masked_write(chip, chip->chgr_base + CHG_TCHG_MAX_EN_REG,
			CHG_TCHG_MAX_EN_BIT, CHG_TCHG_MAX_EN_BIT);
	if (rc) {
		pr_err("Failed to write tchg_max_en rc=%d\n", rc);
		return rc;
	}

	return rc;
}

static int qpnp_lbc_vbatdet_override(struct qpnp_lbc_chip *chip, int ovr_val)
{
	int rc;
	u8 reg_val;
	struct spmi_device *spmi = chip->spmi;
	unsigned long flags;

	spin_lock_irqsave(&chip->hw_access_lock, flags);

	rc = __qpnp_lbc_read(spmi, chip->chgr_base + CHG_COMP_OVR1,
			&reg_val, 1);
	if (rc) {
		pr_err("spmi read failed: addr=%03X, rc=%d\n",
				chip->chgr_base, rc);
		goto out;
	}
	pr_debug("addr = 0x%x read 0x%x\n", chip->chgr_base, reg_val);

	reg_val &= ~CHG_VBAT_DET_OVR_MASK;
	reg_val |= ovr_val & CHG_VBAT_DET_OVR_MASK;

	pr_debug("writing to base=%x val=%x\n", chip->chgr_base, reg_val);

	rc = __qpnp_lbc_secure_write(spmi, chip->chgr_base, CHG_COMP_OVR1,
			&reg_val, 1);
	if (rc)
		pr_err("spmi write failed: addr=%03X, rc=%d\n",
				chip->chgr_base, rc);

out:
	spin_unlock_irqrestore(&chip->hw_access_lock, flags);
	return rc;
}

#define index  5
static int get_prop_battery_voltage_now(struct qpnp_lbc_chip *chip)
{
	int rc = 0;
	int max=3800,mix=3800,sum=0;
	int count=0;
	struct qpnp_vadc_result results;
	rc = qpnp_vadc_read(chip->vadc_dev, VBAT_SNS, &results);
	if (rc) {
		pr_err("Unable to read vbat rc=%d\n", rc);
		return 0;
	}
	max = results.physical;
	mix = results.physical;
	for(count=0;count<index;count++)
	{
		mdelay(4);
		rc = qpnp_vadc_read(chip->vadc_dev, VBAT_SNS, &results);
		if (rc) {
			pr_err("Unable to read vbat rc=%d\n", rc);
			return 0;
		}
		if(results.physical > max)
		{
			max = results.physical;
		}else if(results.physical < mix){
			mix = results.physical;
		}
		sum+= results.physical;
	}
	results.physical = (sum-max-mix)/(count-2);
	return results.physical;
}

static int get_prop_batt_present(struct qpnp_lbc_chip *chip)
{
	u8 reg_val;
	int rc;

	rc = qpnp_lbc_read(chip, chip->bat_if_base + BAT_IF_PRES_STATUS_REG,
			&reg_val, 1);
	if (rc) {
		pr_err("Failed to read battery status read failed rc=%d\n",
				rc);
		return 0;
	}

	return (reg_val & BATT_PRES_MASK) ? 1 : 0;
}
#define USB_VALID_MASK		0xC0
#define USB_VALID_OVP_VALUE	0x40
#define BATT_TEMP_OVERHEAT   (61*10)
#define BATT_TEMP_OVERCOLD   (-1*10)
#define BATT_TEMP_COOL_DEGC        (10*10)
#define BTTE_TEMP_WARM_DEGC     (45*10)
static int get_prop_batt_health(struct qpnp_lbc_chip *chip)
{
	u8 reg_val;
	int rc;
	struct qpnp_vadc_result result;
	int batt_temp;
	u8 usbin_chg_rt_sts ;
	rc = qpnp_lbc_read(chip, chip->usb_chgpth_base + USB_PTH_STS_REG,
			&usbin_chg_rt_sts, 1);
	if ((usbin_chg_rt_sts & USB_VALID_MASK)
			== USB_VALID_OVP_VALUE) {
		pr_err("Over voltage charger inserted\n");
		return POWER_SUPPLY_HEALTH_OVERVOLTAGE;
	}

	rc = qpnp_lbc_read(chip, chip->bat_if_base + BAT_IF_TEMP_STATUS_REG,
			&reg_val, 1);
	if (rc) {
		pr_err("Failed to read battery health rc=%d\n", rc);
		return POWER_SUPPLY_HEALTH_UNKNOWN;
	}

	rc = qpnp_vadc_read(chip->vadc_dev, LR_MUX1_BATT_THERM, &result);
	if (rc) {
		pr_debug("CHG:error reading adc channel = %d, rc = %d\n",
				LR_MUX1_BATT_THERM, rc);
		return POWER_SUPPLY_HEALTH_UNKNOWN;
	}
	batt_temp = (int)result.physical;
	if((batt_temp >= BATT_TEMP_OVERHEAT) || (batt_temp <= BATT_TEMP_OVERCOLD)) {
		if(batt_temp >= BATT_TEMP_OVERHEAT) {
			if ((qpnp_lbc_is_usb_chg_plugged_in(chip))&&(get_prop_batt_present(chip))) {
				pr_info("POWER_SUPPLY_HEALTH_OVERHEAT \n");
				return POWER_SUPPLY_HEALTH_OVERHEAT;
			}
			else {
				return POWER_SUPPLY_HEALTH_GOOD;
			}
		}
		if(batt_temp <=BATT_TEMP_OVERCOLD) {
			if ((qpnp_lbc_is_usb_chg_plugged_in(chip))&&(get_prop_batt_present(chip))) {
				pr_info("POWER_SUPPLY_HEALTH_COLD \n");
				return POWER_SUPPLY_HEALTH_COLD;
			}
			else {
				return POWER_SUPPLY_HEALTH_GOOD;
			}
		}
	}
	return POWER_SUPPLY_HEALTH_GOOD;
}

static int get_prop_charge_type(struct qpnp_lbc_chip *chip)
{
	int rc;
	u8 reg_val;

	if (!get_prop_batt_present(chip))
		return POWER_SUPPLY_CHARGE_TYPE_NONE;

	rc = qpnp_lbc_read(chip, chip->chgr_base + INT_RT_STS_REG,
			&reg_val, 1);
	if (rc) {
		pr_err("Failed to read interrupt sts %d\n", rc);
		return POWER_SUPPLY_CHARGE_TYPE_NONE;
	}

	if (reg_val & FAST_CHG_ON_IRQ)
		return POWER_SUPPLY_CHARGE_TYPE_FAST;

	return POWER_SUPPLY_CHARGE_TYPE_NONE;
}

static int get_prop_batt_status(struct qpnp_lbc_chip *chip)
{
	int rc;
	u8 reg_val;

	if (qpnp_lbc_is_usb_chg_plugged_in(chip) && chip->chg_done)
		return POWER_SUPPLY_STATUS_FULL;

	rc = qpnp_lbc_read(chip, chip->chgr_base + INT_RT_STS_REG,
			&reg_val, 1);
	if (rc) {
		pr_err("Failed to read interrupt sts rc= %d\n", rc);
		return POWER_SUPPLY_CHARGE_TYPE_NONE;
	}

	if (reg_val & FAST_CHG_ON_IRQ)
		return POWER_SUPPLY_STATUS_CHARGING;

	return POWER_SUPPLY_STATUS_DISCHARGING;
}

static int get_prop_current_now(struct qpnp_lbc_chip *chip)
{
	union power_supply_propval ret = {0,};

	if (chip->bms_psy) {
		chip->bms_psy->get_property(chip->bms_psy,
				POWER_SUPPLY_PROP_CURRENT_NOW, &ret);
		return ret.intval;
	} else {
		pr_debug("No BMS supply registered return 0\n");
	}

	return 0;
}

#define BATTERYE_CHECK_PERIOD_MS	1000
static int present_status=1,flag=0;
	static void
qpnp_batt_pres_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct qpnp_lbc_chip *chip = container_of(dwork,
			struct qpnp_lbc_chip, batt_pres_work);
	int rc = -EINVAL;
	union power_supply_propval ret = {0,};


	if ( !get_prop_batt_present(chip))
	{
		present_status=0;
		pr_debug("present_status=0\n");
	}else{
		present_status =1;
	}
	if (chip->bms_psy == NULL)
		chip->bms_psy = power_supply_get_by_name("bms");
	if (chip->bms_psy) {
		rc = chip->bms_psy->get_property(chip->bms_psy,
				POWER_SUPPLY_PROP_PRESENT, &ret);
		if (rc) {
			pr_err("Unable to get battery 'STATUS' rc=%d\n", rc);
		}
		flag = 0;
	}
}

#define DEFAULT_CAPACITY	50
static int get_prop_capacity(struct qpnp_lbc_chip *chip)
{
	union power_supply_propval ret = {0,};
	int soc, battery_status, charger_in;

	if (chip->fake_battery_soc >= 0)
		return chip->fake_battery_soc;
	if ((chip->cfg_use_fake_battery || !get_prop_batt_present(chip))&&(flag==0))
	{
		flag=1;
		schedule_delayed_work(&chip->batt_pres_work,
				msecs_to_jiffies(BATTERYE_CHECK_PERIOD_MS));
	}

	if (chip->bms_psy) {
		chip->bms_psy->get_property(chip->bms_psy,
				POWER_SUPPLY_PROP_CAPACITY, &ret);
		mutex_lock(&chip->chg_enable_lock);
		if (chip->chg_done)
			chip->bms_psy->get_property(chip->bms_psy,
					POWER_SUPPLY_PROP_CAPACITY, &ret);
		battery_status = get_prop_batt_status(chip);
		charger_in = qpnp_lbc_is_usb_chg_plugged_in(chip);

		/* reset chg_done flag if capacity not 100% */
		if (ret.intval < 100 && chip->chg_done) {
			chip->chg_done = false;
			power_supply_changed(&chip->batt_psy);
		}

		if (battery_status != POWER_SUPPLY_STATUS_CHARGING
				&& charger_in
				&& !chip->cfg_charging_disabled
				&& chip->cfg_soc_resume_limit
				&& recharge_trigger) {
			pr_debug("resuming charging at %d%% soc\n",
					ret.intval);
			if (!chip->cfg_disable_vbatdet_based_recharge)
				qpnp_lbc_vbatdet_override(chip, OVERRIDE_0);
			if((charging_on == 1)&&(runin_on == 1)){
				qpnp_lbc_charger_enable(chip, SOC, 1);
			}
		}
		mutex_unlock(&chip->chg_enable_lock);

		soc = ret.intval;
		if (soc == 0) {
			if (!qpnp_lbc_is_usb_chg_plugged_in(chip))
				pr_warn_ratelimited("Batt 0, CHG absent\n");
		}
		return soc;
	} else {
		pr_debug("No BMS supply registered return %d\n",
				DEFAULT_CAPACITY);
	}

	/*
	 * Return default capacity to avoid userspace
	 * from shutting down unecessarily
	 */
	return DEFAULT_CAPACITY;
}

#define DEFAULT_TEMP		250
static int get_prop_batt_temp(struct qpnp_lbc_chip *chip)
{
	int rc = 0;
	struct qpnp_vadc_result results;

	if (chip->cfg_use_fake_battery || !get_prop_batt_present(chip))
		return DEFAULT_TEMP;

	rc = qpnp_vadc_read(chip->vadc_dev, LR_MUX1_BATT_THERM, &results);
	if (rc) {
		pr_debug("Unable to read batt temperature rc=%d\n", rc);
		return DEFAULT_TEMP;
	}
	pr_debug("get_bat_temp %d, %lld\n", results.adc_code,
			results.physical);

	return (int)results.physical;
}

static void qpnp_lbc_set_appropriate_current(struct qpnp_lbc_chip *chip)
{
	unsigned int chg_current = chip->usb_psy_ma;
	if (input_current) {
		chg_current = chip->prev_max_ma;
		pr_debug("charging function test charger current %d mA\n", chg_current);
		qpnp_lbc_ibatmax_set(chip, chg_current);
		return ;
	}
	if (chip->bat_is_cool && chip->cfg_cool_bat_chg_ma)
		chg_current = min(chg_current, chip->cfg_cool_bat_chg_ma);
	if (chip->bat_is_warm && chip->cfg_warm_bat_chg_ma)
		chg_current = min(chg_current, chip->cfg_warm_bat_chg_ma);
	if (chip->therm_lvl_sel != 0 && chip->thermal_mitigation)
		chg_current = min(chg_current,
				chip->thermal_mitigation[chip->therm_lvl_sel]);

	pr_debug("setting charger current %d mA\n", chg_current);
	qpnp_lbc_ibatmax_set(chip, chg_current);
}

#define QPNP_CHG_IBAT_ORT		700
static void qpnp_batt_external_power_changed(struct power_supply *psy)
{
	struct qpnp_lbc_chip *chip = container_of(psy, struct qpnp_lbc_chip,
			batt_psy);
	union power_supply_propval ret = {0,};
	int current_ma;
	unsigned long flags;

	spin_lock_irqsave(&chip->ibat_change_lock, flags);
	if (!chip->bms_psy)
		chip->bms_psy = power_supply_get_by_name("bms");

	if (qpnp_lbc_is_usb_chg_plugged_in(chip)) {
		chip->usb_psy->get_property(chip->usb_psy,
				POWER_SUPPLY_PROP_CURRENT_MAX, &ret);
		current_ma = ret.intval / 1000;

		if (current_ma == chip->prev_max_ma)
			goto skip_current_config;

		/* Disable charger in case of reset or suspend event */
		if (current_ma <= 2 && !chip->cfg_use_fake_battery
				&& get_prop_batt_present(chip)) {
			qpnp_lbc_charger_enable(chip, CURRENT, 1);
			chip->usb_psy_ma = 450;
			qpnp_lbc_set_appropriate_current(chip);
		} else {
			chip->usb_psy_ma = current_ma;
			if(temp_on ==0) {
				current_ma = min(QPNP_CHG_IBAT_ORT, ret.intval / 1000);
				chip->usb_psy_ma = current_ma;
			}
			qpnp_lbc_set_appropriate_current(chip);
			qpnp_lbc_charger_enable(chip, CURRENT, 1);
		}
	}

skip_current_config:
	spin_unlock_irqrestore(&chip->ibat_change_lock, flags);
	pr_debug("power supply changed batt_psy\n");
	power_supply_changed(&chip->batt_psy);
}

static int qpnp_lbc_system_temp_level_set(struct qpnp_lbc_chip *chip,
		int lvl_sel)
{
	int rc = 0;
	int prev_therm_lvl;
	unsigned long flags;

	if (!chip->thermal_mitigation) {
		pr_err("Thermal mitigation not supported\n");
		return -EINVAL;
	}

	if (lvl_sel < 0) {
		pr_err("Unsupported level selected %d\n", lvl_sel);
		return -EINVAL;
	}

	if (lvl_sel >= chip->cfg_thermal_levels) {
		pr_err("Unsupported level selected %d forcing %d\n", lvl_sel,
				chip->cfg_thermal_levels - 1);
		lvl_sel = chip->cfg_thermal_levels - 1;
	}

	if (lvl_sel == chip->therm_lvl_sel)
		return 0;

	spin_lock_irqsave(&chip->ibat_change_lock, flags);
	prev_therm_lvl = chip->therm_lvl_sel;
	chip->therm_lvl_sel = lvl_sel;
	if (chip->therm_lvl_sel == (chip->cfg_thermal_levels - 1)) {
		/* Disable charging if highest value selected by */
		rc = qpnp_lbc_charger_enable(chip, THERMAL, 0);
		if (rc < 0)
			dev_err(chip->dev,
					"Failed to set disable charging rc %d\n", rc);
		goto out;
	}

	qpnp_lbc_set_appropriate_current(chip);

	if (prev_therm_lvl == chip->cfg_thermal_levels - 1) {
		/*
		 * If previously highest value was selected charging must have
		 * been disabed. Enable charging.
		 */
		rc = qpnp_lbc_charger_enable(chip, THERMAL, 1);
		if (rc < 0) {
			dev_err(chip->dev,
					"Failed to enable charging rc %d\n", rc);
		}
	}
out:
	spin_unlock_irqrestore(&chip->ibat_change_lock, flags);
	return rc;
}

#define MIN_COOL_TEMP		-300
#define MAX_WARM_TEMP		1000
#define HYSTERISIS_DECIDEGC	5

static int qpnp_lbc_configure_jeita(struct qpnp_lbc_chip *chip,
		enum power_supply_property psp, int temp_degc)
{
	int rc = 0;

	if ((temp_degc < MIN_COOL_TEMP) || (temp_degc > MAX_WARM_TEMP)) {
		pr_err("Bad temperature request %d\n", temp_degc);
		return -EINVAL;
	}

	mutex_lock(&chip->jeita_configure_lock);
	switch (psp) {
		case POWER_SUPPLY_PROP_COOL_TEMP:
			if (temp_degc >=
					(chip->cfg_warm_bat_decidegc - HYSTERISIS_DECIDEGC)) {
				pr_err("Can't set cool %d higher than warm %d - hysterisis %d\n",
						temp_degc,
						chip->cfg_warm_bat_decidegc,
						HYSTERISIS_DECIDEGC);
				rc = -EINVAL;
				goto mutex_unlock;
			}
			if (chip->bat_is_cool)
				chip->adc_param.high_temp =
					temp_degc + HYSTERISIS_DECIDEGC;
			else if (!chip->bat_is_warm)
				chip->adc_param.low_temp = temp_degc;

			chip->cfg_cool_bat_decidegc = temp_degc;
			break;
		case POWER_SUPPLY_PROP_WARM_TEMP:
			if (temp_degc <=
					(chip->cfg_cool_bat_decidegc + HYSTERISIS_DECIDEGC)) {
				pr_err("Can't set warm %d higher than cool %d + hysterisis %d\n",
						temp_degc,
						chip->cfg_warm_bat_decidegc,
						HYSTERISIS_DECIDEGC);
				rc = -EINVAL;
				goto mutex_unlock;
			}
			if (chip->bat_is_warm)
				chip->adc_param.low_temp =
					temp_degc - HYSTERISIS_DECIDEGC;
			else if (!chip->bat_is_cool)
				chip->adc_param.high_temp = temp_degc;

			chip->cfg_warm_bat_decidegc = temp_degc;
			break;
		default:
			rc = -EINVAL;
			goto mutex_unlock;
	}

	if (qpnp_adc_tm_channel_measure(chip->adc_tm_dev, &chip->adc_param))
		pr_err("request ADC error\n");

mutex_unlock:
	mutex_unlock(&chip->jeita_configure_lock);
	return rc;
}

static int qpnp_batt_property_is_writeable(struct power_supply *psy,
		enum power_supply_property psp)
{
	switch (psp) {
		case POWER_SUPPLY_PROP_STATUS:
		case POWER_SUPPLY_PROP_CAPACITY:
		case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		case POWER_SUPPLY_PROP_COOL_TEMP:
		case POWER_SUPPLY_PROP_VOLTAGE_MIN:
		case POWER_SUPPLY_PROP_WARM_TEMP:
		case POWER_SUPPLY_PROP_SYSTEM_TEMP_LEVEL:
		case POWER_SUPPLY_PROP_INPUT_CURRENT_MAX:
			return 1;
		default:
			break;
	}

	return 0;
}

/*
 * End of charge happens only when BMS reports the battery status as full. For
 * charging to end the s/w must put the usb path in suspend. Note that there
 * is no battery fet and usb path suspend is the only control to prevent any
 * current going in to the battery (and the system)
 * Charging can begin only when VBATDET comparator outputs 0. This indicates
 * that the battery is a at a lower voltage than 4% of the vddmax value.
 * S/W can override this comparator to output a favourable value - this is
 * used while resuming charging when the battery hasnt fallen below 4% but
 * the SOC has fallen below the resume threshold.
 *
 * In short, when SOC resume happens:
 * a. overide the comparator to output 0
 * b. enable charging
 *
 * When vbatdet based resume happens:
 * a. enable charging
 *
 * When end of charge happens:
 * a. disable the overrides in the comparator
 *    (may be from a previous soc resume)
 * b. disable charging
 */
static int qpnp_batt_power_set_property(struct power_supply *psy,
		enum power_supply_property psp,
		const union power_supply_propval *val)
{
	struct qpnp_lbc_chip *chip = container_of(psy, struct qpnp_lbc_chip,
			batt_psy);
	int rc = 0;

	switch (psp) {
		case POWER_SUPPLY_PROP_STATUS:
			if (val->intval == POWER_SUPPLY_STATUS_FULL &&
					!chip->cfg_float_charge) {
				mutex_lock(&chip->chg_enable_lock);

				/* Disable charging */
				rc = qpnp_lbc_charger_enable(chip, SOC, 0);
				if (rc)
					pr_err("Failed to disable charging rc=%d\n",
							rc);
				else
					chip->chg_done = true;

				/*
				 * Enable VBAT_DET based charging:
				 * To enable charging when VBAT falls below VBAT_DET
				 * and device stays suspended after EOC.
				 */
				if (!chip->cfg_disable_vbatdet_based_recharge) {
					/* No override for VBAT_DET_LO comp */
					rc = qpnp_lbc_vbatdet_override(chip,
							OVERRIDE_NONE);
					if (rc)
						pr_err("Failed to override VBAT_DET rc=%d\n",
								rc);
					else
						qpnp_lbc_enable_irq(chip,
								&chip->irqs[CHG_VBAT_DET_LO]);
				}

				mutex_unlock(&chip->chg_enable_lock);
			}
			break;
		case POWER_SUPPLY_PROP_COOL_TEMP:
			rc = qpnp_lbc_configure_jeita(chip, psp, val->intval);
			break;
		case POWER_SUPPLY_PROP_WARM_TEMP:
			rc = qpnp_lbc_configure_jeita(chip, psp, val->intval);
			break;
		case POWER_SUPPLY_PROP_CAPACITY:
			chip->fake_battery_soc = val->intval;
			pr_debug("power supply changed batt_psy\n");
			break;
		case POWER_SUPPLY_PROP_CHARGING_ENABLED:
			chip->cfg_charging_disabled = !(val->intval);
			rc = qpnp_lbc_charger_enable(chip, USER,
					!chip->cfg_charging_disabled);
			if (rc)
				pr_err("Failed to disable charging rc=%d\n", rc);
			break;
		case POWER_SUPPLY_PROP_VOLTAGE_MIN:
			qpnp_lbc_vinmin_set(chip, val->intval / 1000);
			break;
		case POWER_SUPPLY_PROP_SYSTEM_TEMP_LEVEL:
			qpnp_lbc_system_temp_level_set(chip, val->intval);
			break;
		case POWER_SUPPLY_PROP_INPUT_CURRENT_MAX:
			qpnp_lbc_ibatmax_set(chip, val->intval / 1000);
			input_current = true;
			break;
		default:
			return -EINVAL;
	}

	power_supply_changed(&chip->batt_psy);
	return rc;
}

static int qpnp_batt_power_get_property(struct power_supply *psy,
		enum power_supply_property psp,
		union power_supply_propval *val)
{
	struct qpnp_lbc_chip *chip =
		container_of(psy, struct qpnp_lbc_chip, batt_psy);

	switch (psp) {
		case POWER_SUPPLY_PROP_STATUS:
			val->intval = get_prop_batt_status(chip);
			break;
		case POWER_SUPPLY_PROP_CHARGE_TYPE:
			val->intval = get_prop_charge_type(chip);
			break;
		case POWER_SUPPLY_PROP_HEALTH:
			val->intval = get_prop_batt_health(chip);
			break;
		case POWER_SUPPLY_PROP_PRESENT:
			val->intval = get_prop_batt_present(chip);
			if(val->intval){
				break;
			}
			else if(present_status ==1){
				val->intval = 1;
				break;
			}
			break;
		case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
			val->intval = chip->cfg_max_voltage_mv * 1000;
			break;
		case POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN:
			val->intval = chip->cfg_min_voltage_mv * 1000;
			break;
		case POWER_SUPPLY_PROP_VOLTAGE_NOW:
			val->intval = get_prop_battery_voltage_now(chip);
			break;
		case POWER_SUPPLY_PROP_TEMP:
			val->intval = get_prop_batt_temp(chip);
			break;
		case POWER_SUPPLY_PROP_COOL_TEMP:
			val->intval = chip->cfg_cool_bat_decidegc;
			break;
		case POWER_SUPPLY_PROP_WARM_TEMP:
			val->intval = chip->cfg_warm_bat_decidegc;
			break;
		case POWER_SUPPLY_PROP_CAPACITY:
			val->intval = get_prop_capacity(chip);
			break;
		case POWER_SUPPLY_PROP_CURRENT_NOW:
			val->intval = get_prop_current_now(chip);
			break;
		case POWER_SUPPLY_PROP_CHARGING_ENABLED:
			val->intval = !(chip->cfg_charging_disabled);
			break;
		case POWER_SUPPLY_PROP_SYSTEM_TEMP_LEVEL:
			val->intval = chip->therm_lvl_sel;
			break;
		case POWER_SUPPLY_PROP_INPUT_CURRENT_MAX:
			val->intval = chip->prev_max_ma * 1000;
			break;
		default:
			return -EINVAL;
	}

	return 0;
}


static void qpnp_lbc_jeita_adc_notification(enum qpnp_tm_state state, void *ctx)
{
	struct qpnp_lbc_chip *chip = ctx;
	bool bat_warm = 0, bat_cool = 0;
	int temp;
	if (state >= ADC_TM_STATE_NUM) {
		pr_err("invalid notification %d\n", state);
		return;
	}

	temp = get_prop_batt_temp(chip);

	pr_debug("temp = %d state = %s\n", temp,
			state == ADC_TM_WARM_STATE ? "warm" : "cool");

	if (state == ADC_TM_WARM_STATE) {
		if (temp >= chip->cfg_warm_bat_decidegc) {
			/* Normal to warm */
			bat_warm = true;
			bat_cool = false;
			chip->adc_param.low_temp =
				chip->cfg_warm_bat_decidegc
				- HYSTERISIS_DECIDEGC;
			chip->adc_param.state_request =
				ADC_TM_COOL_THR_ENABLE;
		} else if (temp >=
				chip->cfg_cool_bat_decidegc + HYSTERISIS_DECIDEGC) {
			/* Cool to normal */
			bat_warm = false;
			bat_cool = false;

			chip->adc_param.low_temp =
				chip->cfg_cool_bat_decidegc;
			chip->adc_param.high_temp =
				chip->cfg_warm_bat_decidegc;
			chip->adc_param.state_request =
				ADC_TM_HIGH_LOW_THR_ENABLE;
		}
	} else {
		if (temp <= chip->cfg_cool_bat_decidegc) {
			/* Normal to cool */
			bat_warm = false;
			bat_cool = true;
			chip->adc_param.high_temp =
				chip->cfg_cool_bat_decidegc
				+ HYSTERISIS_DECIDEGC;
			chip->adc_param.state_request =
				ADC_TM_WARM_THR_ENABLE;
		} else if (temp <= (chip->cfg_warm_bat_decidegc -
					HYSTERISIS_DECIDEGC)){
			/* Warm to normal */
			bat_warm = false;
			bat_cool = false;

			chip->adc_param.low_temp =
				chip->cfg_cool_bat_decidegc;
			chip->adc_param.high_temp =
				chip->cfg_warm_bat_decidegc;
			chip->adc_param.state_request =
				ADC_TM_HIGH_LOW_THR_ENABLE;
		}
	}
	pr_debug("warm %d, cool %d, low = %d deciDegC, high = %d deciDegC\n",
			chip->bat_is_warm, chip->bat_is_cool,
			chip->adc_param.low_temp, chip->adc_param.high_temp);

	if (qpnp_adc_tm_channel_measure(chip->adc_tm_dev, &chip->adc_param))
		pr_err("request ADC error\n");
}

#define IBAT_TERM_EN_MASK		BIT(3)
static int qpnp_lbc_chg_init(struct qpnp_lbc_chip *chip)
{
	int rc;
	u8 reg_val;

	qpnp_lbc_vbatweak_set(chip, chip->cfg_batt_weak_voltage_uv);
	rc = qpnp_lbc_vinmin_set(chip, chip->cfg_min_voltage_mv);
	if (rc) {
		pr_err("Failed  to set  vin_min rc=%d\n", rc);
		return rc;
	}
	rc = qpnp_lbc_vddsafe_set(chip, chip->cfg_safe_voltage_mv);
	if (rc) {
		pr_err("Failed to set vdd_safe rc=%d\n", rc);
		return rc;
	}
	rc = qpnp_lbc_vddmax_set(chip, chip->cfg_max_voltage_mv);
	if (rc) {
		pr_err("Failed to set vdd_safe rc=%d\n", rc);
		return rc;
	}
	rc = qpnp_lbc_ibatsafe_set(chip, chip->cfg_safe_current);
	if (rc) {
		pr_err("Failed to set ibat_safe rc=%d\n", rc);
		return rc;
	}

	if (of_property_read_bool(chip->spmi->dev.of_node, "qcom,tchg-mins")) {
		rc = qpnp_lbc_tchg_max_set(chip, chip->cfg_tchg_mins);
		if (rc) {
			pr_err("Failed to set tchg_mins rc=%d\n", rc);
			return rc;
		}
	}

	/*
	 * Override VBAT_DET comparator to enable charging
	 * irrespective of VBAT above VBAT_DET.
	 */
	rc = qpnp_lbc_vbatdet_override(chip, OVERRIDE_0);
	if (rc) {
		pr_err("Failed to override comp rc=%d\n", rc);
		return rc;
	}

	/*
	 * Disable iterm comparator of linear charger to disable charger
	 * detecting end of charge condition based on DT configuration
	 * and float charge configuration.
	 */
	if (!chip->cfg_charger_detect_eoc || chip->cfg_float_charge) {
		rc = qpnp_lbc_masked_write(chip,
				chip->chgr_base + CHG_IBATTERM_EN_REG,
				IBAT_TERM_EN_MASK, 0);
		if (rc) {
			pr_err("Failed to disable EOC comp rc=%d\n", rc);
			return rc;
		}
	}

	/* Disable charger watchdog */
	reg_val = 0;
	rc = qpnp_lbc_write(chip, chip->chgr_base + CHG_WDOG_EN_REG,
			&reg_val, 1);

	return rc;
}

static int qpnp_lbc_bat_if_init(struct qpnp_lbc_chip *chip)
{
	u8 reg_val;
	int rc;

	/* Select battery presence detection */
	switch (chip->cfg_bpd_detection) {
		case BPD_TYPE_BAT_THM:
			reg_val = BATT_THM_EN;
			break;
		case BPD_TYPE_BAT_ID:
			reg_val = BATT_ID_EN;
			break;
		case BPD_TYPE_BAT_THM_BAT_ID:
			reg_val = BATT_THM_EN | BATT_ID_EN;
			break;
		default:
			reg_val = BATT_THM_EN;
			break;
	}

	rc = qpnp_lbc_masked_write(chip,
			chip->bat_if_base + BAT_IF_BPD_CTRL_REG,
			BATT_BPD_CTRL_SEL_MASK, reg_val);
	if (rc) {
		pr_err("Failed to choose BPD rc=%d\n", rc);
		return rc;
	}

	/* Force on VREF_BAT_THM */
	reg_val = VREF_BATT_THERM_FORCE_ON;
	rc = qpnp_lbc_write(chip,
			chip->bat_if_base + BAT_IF_VREF_BAT_THM_CTRL_REG,
			&reg_val, 1);
	if (rc) {
		pr_err("Failed to force on VREF_BAT_THM rc=%d\n", rc);
		return rc;
	}

	return 0;
}

static int qpnp_lbc_usb_path_init(struct qpnp_lbc_chip *chip)
{
	int rc;
	u8 reg_val;

	if (qpnp_lbc_is_usb_chg_plugged_in(chip)) {
		reg_val = 0;
		rc = qpnp_lbc_write(chip,
				chip->usb_chgpth_base + CHG_USB_ENUM_T_STOP_REG,
				&reg_val, 1);
		if (rc) {
			pr_err("Failed to write enum stop rc=%d\n", rc);
			return -ENXIO;
		}
	}

	if (chip->cfg_charging_disabled) {
		rc = qpnp_lbc_charger_enable(chip, USER, 0);
		if (rc)
			pr_err("Failed to disable charging rc=%d\n", rc);
	} else {
		/*
		 * Enable charging explictly,
		 * because not sure the default behavior.
		 */
		reg_val = CHG_ENABLE;
		rc = qpnp_lbc_masked_write(chip, chip->chgr_base + CHG_CTRL_REG,
				CHG_EN_MASK, reg_val);
		if (rc)
			pr_err("Failed to enable charger rc=%d\n", rc);
	}

	return rc;
}

#define LBC_MISC_DIG_VERSION_1			0x01
static int qpnp_lbc_misc_init(struct qpnp_lbc_chip *chip)
{
	int rc;
	u8 reg_val, reg_val1, trim_center;

	/* Check if this LBC MISC version supports VDD trimming */
	rc = qpnp_lbc_read(chip, chip->misc_base + MISC_REV2_REG,
			&reg_val, 1);
	if (rc) {
		pr_err("Failed to read VDD_EA TRIM3 reg rc=%d\n", rc);
		return rc;
	}

	if (reg_val >= LBC_MISC_DIG_VERSION_1) {
		chip->supported_feature_flag |= VDD_TRIM_SUPPORTED;
		/* Read initial VDD trim value */
		rc = qpnp_lbc_read(chip, chip->misc_base + MISC_TRIM3_REG,
				&reg_val, 1);
		if (rc) {
			pr_err("Failed to read VDD_EA TRIM3 reg rc=%d\n", rc);
			return rc;
		}

		rc = qpnp_lbc_read(chip, chip->misc_base + MISC_TRIM4_REG,
				&reg_val1, 1);
		if (rc) {
			pr_err("Failed to read VDD_EA TRIM3 reg rc=%d\n", rc);
			return rc;
		}

		trim_center = ((reg_val & MISC_TRIM3_VDD_MASK)
				>> VDD_TRIM3_SHIFT)
			| ((reg_val1 & MISC_TRIM4_VDD_MASK)
					>> VDD_TRIM4_SHIFT);
		chip->init_trim_uv = qpnp_lbc_get_trim_voltage(trim_center);
		chip->delta_vddmax_uv = chip->init_trim_uv;
		pr_debug("Initial trim center %x trim_uv %d\n",
				trim_center, chip->init_trim_uv);
	}

	pr_debug("Setting BOOT_DONE\n");
	reg_val = MISC_BOOT_DONE;
	rc = qpnp_lbc_write(chip, chip->misc_base + MISC_BOOT_DONE_REG,
			&reg_val, 1);

	return rc;
}

#define OF_PROP_READ(chip, prop, qpnp_dt_property, retval, optional)	\
	do {									\
		if (retval)							\
		break;							\
		\
		retval = of_property_read_u32(chip->spmi->dev.of_node,		\
				"qcom," qpnp_dt_property,	\
				&chip->prop);			\
		\
		if ((retval == -EINVAL) && optional)				\
		retval = 0;						\
		else if (retval)						\
		pr_err("Error reading " #qpnp_dt_property		\
				" property rc = %d\n", rc);		\
	} while (0)

static int qpnp_charger_read_dt_props(struct qpnp_lbc_chip *chip)
{
	int rc = 0;
	const char *bpd;

	OF_PROP_READ(chip, cfg_max_voltage_mv, "vddmax-mv", rc, 0);
	OF_PROP_READ(chip, cfg_safe_voltage_mv, "vddsafe-mv", rc, 0);
	OF_PROP_READ(chip, cfg_min_voltage_mv, "vinmin-mv", rc, 0);
	OF_PROP_READ(chip, cfg_safe_current, "ibatsafe-ma", rc, 0);
	if (rc)
		pr_err("Error reading required property rc=%d\n", rc);

	OF_PROP_READ(chip, cfg_tchg_mins, "tchg-mins", rc, 1);
	OF_PROP_READ(chip, cfg_warm_bat_decidegc, "warm-bat-decidegc", rc, 1);
	OF_PROP_READ(chip, cfg_cool_bat_decidegc, "cool-bat-decidegc", rc, 1);
	OF_PROP_READ(chip, cfg_hot_batt_p, "batt-hot-percentage", rc, 1);
	OF_PROP_READ(chip, cfg_cold_batt_p, "batt-cold-percentage", rc, 1);
	OF_PROP_READ(chip, cfg_batt_weak_voltage_uv, "vbatweak-uv", rc, 1);
	OF_PROP_READ(chip, cfg_soc_resume_limit, "resume-soc", rc, 1);
	if (rc) {
		pr_err("Error reading optional property rc=%d\n", rc);
		return rc;
	}

	rc = of_property_read_string(chip->spmi->dev.of_node,
			"qcom,bpd-detection", &bpd);
	if (rc) {

		chip->cfg_bpd_detection = BPD_TYPE_BAT_THM;
		rc = 0;
	} else {
		chip->cfg_bpd_detection = get_bpd(bpd);
		if (chip->cfg_bpd_detection < 0) {
			pr_err("Failed to determine bpd schema rc=%d\n", rc);
			return -EINVAL;
		}
	}

	/*
	 * Look up JEITA compliance parameters if cool and warm temp
	 * provided
	 */
	if (chip->cfg_cool_bat_decidegc || chip->cfg_warm_bat_decidegc) {
		chip->adc_tm_dev = qpnp_get_adc_tm(chip->dev, "chg");
		if (IS_ERR(chip->adc_tm_dev)) {
			rc = PTR_ERR(chip->adc_tm_dev);
			if (rc != -EPROBE_DEFER)
				pr_err("Failed to get adc-tm rc=%d\n", rc);
			return rc;
		}

		OF_PROP_READ(chip, cfg_warm_bat_chg_ma, "ibatmax-warm-ma",
				rc, 1);
		OF_PROP_READ(chip, cfg_cool_bat_chg_ma, "ibatmax-cool-ma",
				rc, 1);
		OF_PROP_READ(chip, cfg_warm_bat_mv, "warm-bat-mv", rc, 1);
		OF_PROP_READ(chip, cfg_cool_bat_mv, "cool-bat-mv", rc, 1);
		if (rc) {
			pr_err("Error reading battery temp prop rc=%d\n", rc);
			return rc;
		}
	}

	/* Get the btc-disabled property */
	chip->cfg_btc_disabled = of_property_read_bool(
			chip->spmi->dev.of_node, "qcom,btc-disabled");

	chip->cfg_btc_disabled = true;

	/* Get the charging-disabled property */
	chip->cfg_charging_disabled =
		of_property_read_bool(chip->spmi->dev.of_node,
				"qcom,charging-disabled");

	/* Get the fake-batt-values property */
	chip->cfg_use_fake_battery =
		of_property_read_bool(chip->spmi->dev.of_node,
				"qcom,use-default-batt-values");

	/* Get peripheral reset configuration property */
	chip->cfg_disable_follow_on_reset =
		of_property_read_bool(chip->spmi->dev.of_node,
				"qcom,disable-follow-on-reset");

	/* Get the float charging property */
	chip->cfg_float_charge =
		of_property_read_bool(chip->spmi->dev.of_node,
				"qcom,float-charge");

	/* Get the charger EOC detect property */
	chip->cfg_charger_detect_eoc =
		of_property_read_bool(chip->spmi->dev.of_node,
				"qcom,charger-detect-eoc");

	/* Get the vbatdet disable property */
	chip->cfg_disable_vbatdet_based_recharge =
		of_property_read_bool(chip->spmi->dev.of_node,
				"qcom,disable-vbatdet-based-recharge");
	/* Disable charging when faking battery values */
	if (chip->cfg_use_fake_battery)
		chip->cfg_charging_disabled = true;

	chip->cfg_use_external_charger = of_property_read_bool(
			chip->spmi->dev.of_node, "qcom,use-external-charger");

	if (of_find_property(chip->spmi->dev.of_node,
				"qcom,thermal-mitigation",
				&chip->cfg_thermal_levels)) {
		chip->thermal_mitigation = devm_kzalloc(chip->dev,
				chip->cfg_thermal_levels,
				GFP_KERNEL);

		if (chip->thermal_mitigation == NULL) {
			pr_err("thermal mitigation kzalloc() failed.\n");
			return -ENOMEM;
		}

		chip->cfg_thermal_levels /= sizeof(int);
		rc = of_property_read_u32_array(chip->spmi->dev.of_node,
				"qcom,thermal-mitigation",
				chip->thermal_mitigation,
				chip->cfg_thermal_levels);
		if (rc) {
			pr_err("Failed to read threm limits rc = %d\n", rc);
			return rc;
		}
	}

	return rc;
}

#define CHG_CHECK_PERIOD_MS	10000

static irqreturn_t qpnp_lbc_usbin_valid_irq_handler(int irq, void *_chip)
{
	struct qpnp_lbc_chip *chip = _chip;
	int usb_present;
	unsigned long flags;

	usb_present = qpnp_lbc_is_usb_chg_plugged_in(chip);
	pr_err("usbin-valid triggered: %d\n", usb_present);

	if (chip->usb_present ^ usb_present) {
		chip->usb_present = usb_present;
		if (!usb_present) {
			qpnp_lbc_charger_enable(chip, CURRENT, 0);
			if (!qpnp_lbc_is_usb_chg_plugged_in(chip)) {
				if(dc_chg_lock){
					printk("%s,release dc_chg_wake_lock:finish charging\n",__func__);
					wake_unlock(&dc_chg_wake_lock);
					dc_chg_lock=false;
				}
			}
			spin_lock_irqsave(&chip->ibat_change_lock, flags);
			chip->usb_psy_ma = QPNP_CHG_I_MAX_MIN_90;
			qpnp_lbc_set_appropriate_current(chip);
			spin_unlock_irqrestore(&chip->ibat_change_lock,
					flags);
		} else {
			/*
			 * Override VBAT_DET comparator to start charging
			 * even if VBAT > VBAT_DET.
			 */
			if (!chip->cfg_disable_vbatdet_based_recharge)
				qpnp_lbc_vbatdet_override(chip, OVERRIDE_0);

			/*
			 * Enable SOC based charging to make sure
			 * charging gets enabled on USB insertion
			 * irrespective of battery SOC above resume_soc.
			 */
			schedule_delayed_work(&chip->charger_work,
					msecs_to_jiffies(CHG_CHECK_PERIOD_MS));
			input_current =false ;
			if(!dc_chg_lock){
				printk("%s,add dc_chg_wake_lock:begin charging\n",__func__);
				wake_lock(&dc_chg_wake_lock);
				dc_chg_lock=true;
			}
			qpnp_lbc_charger_enable(chip, SOC, 1);
		}

		pr_debug("Updating usb_psy PRESENT property\n");
		power_supply_set_present(chip->usb_psy, chip->usb_present);
	}

	return IRQ_HANDLED;
}

static int qpnp_lbc_is_batt_temp_ok(struct qpnp_lbc_chip *chip)
{
	u8 reg_val;
	int rc;

	rc = qpnp_lbc_read(chip, chip->bat_if_base + INT_RT_STS_REG,
			&reg_val, 1);
	if (rc) {
		pr_err("spmi read failed: addr=%03X, rc=%d\n",
				chip->bat_if_base + INT_RT_STS_REG, rc);
		return rc;
	}

	return (reg_val & BAT_TEMP_OK_IRQ) ? 1 : 0;
}

static irqreturn_t qpnp_lbc_batt_temp_irq_handler(int irq, void *_chip)
{
	struct qpnp_lbc_chip *chip = _chip;
	int batt_temp_good;

	batt_temp_good = qpnp_lbc_is_batt_temp_ok(chip);
	pr_debug("batt-temp triggered: %d\n", batt_temp_good);

	pr_debug("power supply changed batt_psy\n");
	power_supply_changed(&chip->batt_psy);
	return IRQ_HANDLED;
}

static irqreturn_t qpnp_lbc_batt_pres_irq_handler(int irq, void *_chip)
{
	struct qpnp_lbc_chip *chip = _chip;
	int batt_present;

	batt_present = qpnp_lbc_is_batt_present(chip);
	pr_debug("batt-pres triggered: %d\n", batt_present);

	if (chip->batt_present ^ batt_present) {
		chip->batt_present = batt_present;
		pr_debug("power supply changed batt_psy\n");
		power_supply_changed(&chip->batt_psy);

		if ((chip->cfg_cool_bat_decidegc
					|| chip->cfg_warm_bat_decidegc)
				&& batt_present) {
			pr_debug("enabling vadc notifications\n");
			if (qpnp_adc_tm_channel_measure(chip->adc_tm_dev,
						&chip->adc_param))
				pr_err("request ADC error\n");
		} else if ((chip->cfg_cool_bat_decidegc
					|| chip->cfg_warm_bat_decidegc)
				&& !batt_present) {
			qpnp_adc_tm_disable_chan_meas(chip->adc_tm_dev,
					&chip->adc_param);
			pr_debug("disabling vadc notifications\n");
		}
	}
	return IRQ_HANDLED;
}

static irqreturn_t qpnp_lbc_chg_failed_irq_handler(int irq, void *_chip)
{
	struct qpnp_lbc_chip *chip = _chip;
	int rc;
	u8 reg_val = CHG_FAILED_BIT;

	pr_debug("chg_failed triggered count=%u\n", ++chip->chg_failed_count);
	rc = qpnp_lbc_write(chip, chip->chgr_base + CHG_FAILED_REG,
			&reg_val, 1);
	if (rc)
		pr_err("Failed to write chg_fail clear bit rc=%d\n", rc);

	if (chip->bat_if_base) {
		pr_debug("power supply changed batt_psy\n");
		power_supply_changed(&chip->batt_psy);
	}

	return IRQ_HANDLED;
}

static int qpnp_lbc_is_fastchg_on(struct qpnp_lbc_chip *chip)
{
	u8 reg_val;
	int rc;

	rc = qpnp_lbc_read(chip, chip->chgr_base + INT_RT_STS_REG,
			&reg_val, 1);
	if (rc) {
		pr_err("Failed to read interrupt status rc=%d\n", rc);
		return rc;
	}
	pr_debug("charger status %x\n", reg_val);
	return (reg_val & FAST_CHG_ON_IRQ) ? 1 : 0;
}

#define TRIM_PERIOD_NS			(50LL * NSEC_PER_SEC)
static irqreturn_t qpnp_lbc_fastchg_irq_handler(int irq, void *_chip)
{
	ktime_t kt;
	struct qpnp_lbc_chip *chip = _chip;
	bool fastchg_on = false;

	fastchg_on = qpnp_lbc_is_fastchg_on(chip);

	pr_debug("FAST_CHG IRQ triggered, fastchg_on: %d\n", fastchg_on);

	if (chip->fastchg_on ^ fastchg_on) {
		chip->fastchg_on = fastchg_on;
		if (fastchg_on) {
			mutex_lock(&chip->chg_enable_lock);
			chip->chg_done = false;
			mutex_unlock(&chip->chg_enable_lock);
			/*
			 * Start alarm timer to periodically calculate
			 * and update VDD_MAX trim value.
			 */
			if (chip->supported_feature_flag &
					VDD_TRIM_SUPPORTED) {
				kt = ns_to_ktime(TRIM_PERIOD_NS);
				alarm_start_relative(&chip->vddtrim_alarm,
						kt);
			}
		}

		if (chip->bat_if_base) {
			pr_debug("power supply changed batt_psy\n");
			power_supply_changed(&chip->batt_psy);
		}
	}

	return IRQ_HANDLED;
}

static irqreturn_t qpnp_lbc_chg_done_irq_handler(int irq, void *_chip)
{
	struct qpnp_lbc_chip *chip = _chip;

	pr_debug("charging done triggered\n");

	chip->chg_done = true;
	pr_debug("power supply changed batt_psy\n");
	power_supply_changed(&chip->batt_psy);

	return IRQ_HANDLED;
}

static irqreturn_t qpnp_lbc_vbatdet_lo_irq_handler(int irq, void *_chip)
{
	struct qpnp_lbc_chip *chip = _chip;
	int rc;

	pr_debug("vbatdet-lo triggered\n");

	/*
	 * Disable vbatdet irq to prevent interrupt storm when VBAT is
	 * close to VBAT_DET.
	 */
	qpnp_lbc_disable_irq(chip, &chip->irqs[CHG_VBAT_DET_LO]);

	/*
	 * Override VBAT_DET comparator to 0 to fix comparator toggling
	 * near VBAT_DET threshold.
	 */
	qpnp_lbc_vbatdet_override(chip, OVERRIDE_0);

	/*
	 * Battery has fallen below the vbatdet threshold and it is
	 * time to resume charging.
	 */
	rc = qpnp_lbc_charger_enable(chip, SOC, 1);
	if (rc)
		pr_err("Failed to enable charging\n");

	return IRQ_HANDLED;
}

static int qpnp_lbc_is_overtemp(struct qpnp_lbc_chip *chip)
{
	u8 reg_val;
	int rc;

	rc = qpnp_lbc_read(chip, chip->usb_chgpth_base + INT_RT_STS_REG,
			&reg_val, 1);
	if (rc) {
		pr_err("Failed to read interrupt status rc=%d\n", rc);
		return rc;
	}

	pr_debug("OVERTEMP rt status %x\n", reg_val);
	return (reg_val & OVERTEMP_ON_IRQ) ? 1 : 0;
}

static irqreturn_t qpnp_lbc_usb_overtemp_irq_handler(int irq, void *_chip)
{
	struct qpnp_lbc_chip *chip = _chip;
	int overtemp = qpnp_lbc_is_overtemp(chip);

	pr_warn_ratelimited("charger %s temperature limit !!!\n",
			overtemp ? "exceeds" : "within");

	return IRQ_HANDLED;
}

static int qpnp_disable_lbc_charger(struct qpnp_lbc_chip *chip)
{
	int rc;
	u8 reg;

	reg = CHG_FORCE_BATT_ON;
	rc = qpnp_lbc_masked_write(chip, chip->chgr_base + CHG_CTRL_REG,
			CHG_EN_MASK, reg);
	/* disable BTC */
	rc |= qpnp_lbc_masked_write(chip, chip->bat_if_base + BAT_IF_BTC_CTRL,
			BTC_COMP_EN_MASK, 0);
	/* Enable BID and disable THM based BPD */
	reg = BATT_ID_EN | BATT_BPD_OFFMODE_EN;
	rc |= qpnp_lbc_write(chip, chip->bat_if_base + BAT_IF_BPD_CTRL_REG,
			&reg, 1);
	return rc;
}

#define SPMI_REQUEST_IRQ(chip, idx, rc, irq_name, threaded, flags, wake)\
	do {									\
		if (rc)								\
		break;							\
		if (chip->irqs[idx].irq) {					\
			if (threaded)						\
			rc = devm_request_threaded_irq(chip->dev,	\
					chip->irqs[idx].irq, NULL,		\
					qpnp_lbc_##irq_name##_irq_handler,	\
					flags, #irq_name, chip);		\
			else							\
			rc = devm_request_irq(chip->dev,		\
					chip->irqs[idx].irq,			\
					qpnp_lbc_##irq_name##_irq_handler,	\
					flags, #irq_name, chip);		\
			if (rc < 0) {						\
				pr_err("Unable to request " #irq_name " %d\n",	\
						rc);	\
			} else {						\
				rc = 0;						\
				if (wake) {					\
					enable_irq_wake(chip->irqs[idx].irq);	\
					chip->irqs[idx].is_wake = true;		\
				}						\
			}							\
		}								\
	} while (0)

#define SPMI_GET_IRQ_RESOURCE(chip, rc, resource, idx, name)		\
	do {									\
		if (rc)								\
		break;							\
		\
		rc = spmi_get_irq_byname(chip->spmi, resource, #name);		\
		if (rc < 0) {							\
			pr_err("Unable to get irq resource " #name "%d\n", rc);	\
		} else {							\
			chip->irqs[idx].irq = rc;				\
			rc = 0;							\
		}								\
	} while (0)

static int qpnp_lbc_request_irqs(struct qpnp_lbc_chip *chip)
{
	int rc = 0;

	SPMI_REQUEST_IRQ(chip, CHG_FAILED, rc, chg_failed, 0,
			IRQF_TRIGGER_RISING, 1);

	SPMI_REQUEST_IRQ(chip, CHG_FAST_CHG, rc, fastchg, 1,
			IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING
			| IRQF_ONESHOT, 1);

	SPMI_REQUEST_IRQ(chip, CHG_DONE, rc, chg_done, 0,
			IRQF_TRIGGER_RISING, 0);

	SPMI_REQUEST_IRQ(chip, CHG_VBAT_DET_LO, rc, vbatdet_lo, 0,
			IRQF_TRIGGER_FALLING, 1);

	SPMI_REQUEST_IRQ(chip, BATT_PRES, rc, batt_pres, 1,
			IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING
			| IRQF_ONESHOT, 1);

	SPMI_REQUEST_IRQ(chip, BATT_TEMPOK, rc, batt_temp, 0,
			IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING, 1);

	SPMI_REQUEST_IRQ(chip, USBIN_VALID, rc, usbin_valid, 0,
			IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING, 1);

	SPMI_REQUEST_IRQ(chip, USB_OVER_TEMP, rc, usb_overtemp, 0,
			IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING, 0);

	return 0;
}

static int qpnp_lbc_get_irqs(struct qpnp_lbc_chip *chip, u8 subtype,
		struct spmi_resource *spmi_resource)
{
	int rc = 0;

	switch (subtype) {
		case LBC_CHGR_SUBTYPE:
			SPMI_GET_IRQ_RESOURCE(chip, rc, spmi_resource,
					CHG_FAST_CHG, fast-chg-on);
			SPMI_GET_IRQ_RESOURCE(chip, rc, spmi_resource,
					CHG_FAILED, chg-failed);
			if (!chip->cfg_disable_vbatdet_based_recharge)
				SPMI_GET_IRQ_RESOURCE(chip, rc, spmi_resource,
						CHG_VBAT_DET_LO, vbat-det-lo);
			if (chip->cfg_charger_detect_eoc)
				SPMI_GET_IRQ_RESOURCE(chip, rc, spmi_resource,
						CHG_DONE, chg-done);
			break;
		case LBC_BAT_IF_SUBTYPE:
			SPMI_GET_IRQ_RESOURCE(chip, rc, spmi_resource,
					BATT_PRES, batt-pres);
			SPMI_GET_IRQ_RESOURCE(chip, rc, spmi_resource,
					BATT_TEMPOK, bat-temp-ok);
			break;
		case LBC_USB_PTH_SUBTYPE:
			SPMI_GET_IRQ_RESOURCE(chip, rc, spmi_resource,
					USBIN_VALID, usbin-valid);
			SPMI_GET_IRQ_RESOURCE(chip, rc, spmi_resource,
					USB_OVER_TEMP, usb-over-temp);
			break;
	};

	return 0;
}

/* Get/Set initial state of charger */
static void determine_initial_status(struct qpnp_lbc_chip *chip)
{
	u8 reg_val;
	int rc;

	chip->usb_present = qpnp_lbc_is_usb_chg_plugged_in(chip);
	power_supply_set_present(chip->usb_psy, chip->usb_present);
	/*
	 * Set USB psy online to avoid userspace from shutting down if battery
	 * capacity is at zero and no chargers online.
	 */
	if (chip->usb_present)
		power_supply_set_online(chip->usb_psy, 1);

	/*
	 * Configure peripheral reset control
	 * This is a workaround only for SLT testing.
	 */
	if (chip->cfg_disable_follow_on_reset) {
		reg_val = 0x0;
		rc = __qpnp_lbc_secure_write(chip->spmi, chip->chgr_base,
				CHG_PERPH_RESET_CTRL3_REG, &reg_val, 1);
		if (rc)
			pr_err("Failed to configure PERPH_CTRL3 rc=%d\n", rc);
		else
			pr_warn("Charger is not following PMIC reset\n");
	}
}

#define IBAT_TRIM			-300
static void qpnp_lbc_vddtrim_work_fn(struct work_struct *work)
{
	int rc, vbat_now_uv, ibat_now;
	u8 reg_val;
	ktime_t kt;
	struct qpnp_lbc_chip *chip = container_of(work, struct qpnp_lbc_chip,
			vddtrim_work);

	vbat_now_uv = get_prop_battery_voltage_now(chip);
	ibat_now = get_prop_current_now(chip) / 1000;
	pr_debug("vbat %d ibat %d capacity %d\n",
			vbat_now_uv, ibat_now, get_prop_capacity(chip));

	/*
	 * Stop trimming under following condition:
	 * USB removed
	 * Charging Stopped
	 */
	if (!qpnp_lbc_is_fastchg_on(chip) ||
			!qpnp_lbc_is_usb_chg_plugged_in(chip)) {
		pr_debug("stop trim charging stopped\n");
		if (!qpnp_lbc_is_usb_chg_plugged_in(chip)) {
			power_supply_set_online(chip->usb_psy, 0);
		}
		goto exit;
	} else {
		rc = qpnp_lbc_read(chip, chip->chgr_base + CHG_STATUS_REG,
				&reg_val, 1);
		if (rc) {
			pr_err("Failed to read chg status rc=%d\n", rc);
			goto out;
		}

		/*
		 * Update VDD trim voltage only if following conditions are
		 * met:
		 * If charger is in VDD loop AND
		 * If ibat is between 0 ma and -300 ma
		 */
		if ((reg_val & CHG_VDD_LOOP_BIT) &&
				((ibat_now < 0) && (ibat_now > IBAT_TRIM)))
			qpnp_lbc_adjust_vddmax(chip, vbat_now_uv);
	}

out:
	kt = ns_to_ktime(TRIM_PERIOD_NS);
	alarm_start_relative(&chip->vddtrim_alarm, kt);
exit:
	pm_relax(chip->dev);
}

static enum alarmtimer_restart vddtrim_callback(struct alarm *alarm,
		ktime_t now)
{
	struct qpnp_lbc_chip *chip = container_of(alarm, struct qpnp_lbc_chip,
			vddtrim_alarm);

	pm_stay_awake(chip->dev);
	schedule_work(&chip->vddtrim_work);

	return ALARMTIMER_NORESTART;
}

static void
qpnp_charging_function_work (struct qpnp_lbc_chip *chip)
{
	if(charging_on) {
		if (!runin_on) {
			qpnp_lbc_charger_enable(chip, USER, 0);
			pr_debug("runin discharging\n");
			return;
		}
		qpnp_lbc_charger_enable(chip, USER, 1);
		pr_debug("enable or runin charging\n");
	}
	else{
		qpnp_lbc_charger_enable(chip, USER, 0);
		pr_debug("disable charging\n");
	}
	return;
}

#define QPNP_CHG_IBAT_COOL		600
#define QPNP_CHG_IBAT_WARM		1500
#define QPNP_VDDMAX_ORT		4100
#define QPNP_VDDMAX_NORMAL		4350
u8 psy_health_sts = 0;
static int temp_rechar = 0;
	static void
qpnp_batt_temp_work(struct qpnp_lbc_chip *chip)
{
	int batt_temp;
	int rc;
	struct qpnp_vadc_result results;
	int iusb_current;
	union power_supply_propval ret = {0,};
	rc = qpnp_vadc_read(chip->vadc_dev, LR_MUX1_BATT_THERM, &results);

	if (rc) {
		pr_info("charging:error reading adc channel = %d, rc = %d\n",
				LR_MUX1_BATT_THERM, rc);
		return;
	}
	pr_debug("batt_temp = %lld \n", results.physical);
	batt_temp = get_prop_batt_temp(chip);
	chip->cfg_max_voltage_mv = QPNP_VDDMAX_NORMAL;
	chip->usb_psy->get_property(chip->usb_psy,
			POWER_SUPPLY_PROP_CURRENT_MAX, &ret);
	if((batt_temp >= BATT_TEMP_OVERHEAT) || (batt_temp <= BATT_TEMP_OVERCOLD) ||
		((temp_rechar == 1)&&(batt_temp > 540))) {
		if(temp_on == 1) {
			/* disable charging */
			qpnp_lbc_charger_enable(chip, THERMAL, 0);
			pr_debug("batt_temp = %lld \n", results.physical);
			if(chip->chg_done != false)
				chip->chg_done = false;

			if(batt_temp >= BATT_TEMP_OVERHEAT) {
				if (qpnp_lbc_is_usb_chg_plugged_in(chip)) {
					pr_info("POWER_SUPPLY_HEALTH_OVERHEAT \n");
					psy_health_sts = POWER_SUPPLY_HEALTH_OVERHEAT;
				}
				else {
					psy_health_sts = POWER_SUPPLY_HEALTH_GOOD;
				}
			}
			if(batt_temp <=BATT_TEMP_OVERCOLD) {
				if (qpnp_lbc_is_usb_chg_plugged_in(chip)) {
					pr_info("POWER_SUPPLY_HEALTH_COLD \n");
					psy_health_sts = POWER_SUPPLY_HEALTH_COLD;
				}
				else {
					psy_health_sts = POWER_SUPPLY_HEALTH_GOOD;
				}
			}
		}
		else {
			chip->bat_is_cool = false;
			chip->bat_is_warm= false;
			qpnp_lbc_charger_enable(chip, THERMAL, 1);
			iusb_current = min(QPNP_CHG_IBAT_ORT, ret.intval / 1000);
			qpnp_lbc_ibatmax_set(chip, iusb_current);
			chip->cfg_max_voltage_mv = QPNP_VDDMAX_ORT;
			qpnp_lbc_set_appropriate_vddmax(chip);
		}
		temp_rechar = 1;
	}
	else {
		/* enable charging */
		chip->bat_is_warm= false;
		chip->bat_is_cool= false;
		temp_rechar = 0;
		if ((batt_temp < BATT_TEMP_COOL_DEGC) || (batt_temp > BTTE_TEMP_WARM_DEGC)){
			if (batt_temp < BATT_TEMP_COOL_DEGC) {
				if ((limit_on == 1)) {
					chip->bat_is_cool = true;
				}
				else {
					chip->bat_is_cool = false;
				}

				chip->bat_is_warm= false;
				chip->cfg_cool_bat_chg_ma = QPNP_CHG_IBAT_COOL ;
			}
			else if (batt_temp > BTTE_TEMP_WARM_DEGC) {
				chip->bat_is_cool = false;
				chip->bat_is_warm= true;
				chip->cfg_cool_bat_chg_ma = QPNP_CHG_IBAT_WARM ;
			}
			qpnp_lbc_set_appropriate_current(chip);
			qpnp_lbc_set_appropriate_vddmax(chip);
		}
		else {
			if ((batt_temp >= BATT_TEMP_COOL_DEGC)  && (batt_temp < BTTE_TEMP_WARM_DEGC)) {
				chip->bat_is_cool = false;
				chip->bat_is_warm= false;
				qpnp_lbc_set_appropriate_current(chip);
				qpnp_lbc_set_appropriate_vddmax(chip);
			}
		}
		if (qpnp_lbc_is_usb_chg_plugged_in(chip)) {
			power_supply_set_online(chip->usb_psy, 1);
		}
		if (!chip->chg_done) {
			if((charging_on == 1)&&(runin_on == 1)){
				qpnp_lbc_charger_enable(chip, THERMAL, 1);
			}
		}
		pr_debug("POWER_SUPPLY_HEALTH_GOOD \n");
		psy_health_sts = POWER_SUPPLY_HEALTH_GOOD;
		pr_debug("psy changed usb_psy\n");
		power_supply_changed(chip->usb_psy);
	}
	power_supply_set_health_state(chip->usb_psy, psy_health_sts);
	return;

}

#define CONSECUTIVE_COUNT 3
#define CV_TIME_COUNT  0
	static void
qpnp_lbc_charging_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct qpnp_lbc_chip *chip = container_of(dwork,
			struct qpnp_lbc_chip, charger_work);
	int rc;
	static int fastchg_err_count = 0;
	int vbat_mv;
	int fast_on = 0;
	int capacity_bms = 0;

	capacity_bms = get_prop_capacity(chip);
	vbat_mv = get_prop_battery_voltage_now(chip) / 1000;
	fast_on = qpnp_lbc_is_fastchg_on(chip);
	qpnp_batt_temp_work(chip);  // temp check and temp protect command
	qpnp_charging_function_work(chip); //charging function close test
	pr_debug("vbat_mv = %d fast_on = %d capacity_bms=%d\n", vbat_mv,fast_on,capacity_bms);
	if (qpnp_lbc_is_fastchg_on(chip) &&	qpnp_lbc_is_usb_chg_plugged_in(chip)){
		if(capacity_bms == 100) {
			if (report_full) {
				pr_err("End of Charging\n");
				qpnp_lbc_charger_enable(chip, SOC, 0);
				chip->chg_done = true;
			}

		}

	}

	else if (!fast_on &&
			qpnp_lbc_is_usb_chg_plugged_in(chip) && (chip->chg_done == false) && (psy_health_sts == POWER_SUPPLY_HEALTH_GOOD)) {
		if((runin_on ==0) ||(charging_on==0))
			goto check_again_later;
		if (fastchg_err_count== CONSECUTIVE_COUNT) {
			pr_info("fastchg error return\n");
			fastchg_err_count = 0;
			mutex_lock(&chip->chg_enable_lock);

			/* Disable charging */
			rc = qpnp_lbc_charger_enable(chip, SOC, 0);
			if (rc)
				pr_err("Failed to disable charging rc=%d\n",	rc);
			else
				chip->chg_done = true;
			pr_err("vbat_mv=%d,chip->chg_done =%d\n",vbat_mv,chip->chg_done);

			/*
			 * Enable VBAT_DET based charging:
			 * To enable charging when VBAT falls below VBAT_DET
			 * and device stays suspended after EOC.
			 */
			if (!chip->cfg_disable_vbatdet_based_recharge) {
				/* No override for VBAT_DET_LO comp */
				rc = qpnp_lbc_vbatdet_override(chip,
						OVERRIDE_NONE);
				if (rc)
					pr_err("Failed to override VBAT_DET rc=%d\n",	rc);
				else
					qpnp_lbc_enable_irq(chip,
							&chip->irqs[CHG_VBAT_DET_LO]);
			}
			mutex_unlock(&chip->chg_enable_lock);
			goto check_again_later;
		}
		else {
			fastchg_err_count += 1;
			pr_info("fastchg_err_count = %d\n", fastchg_err_count);
			qpnp_lbc_vbatdet_override(chip, OVERRIDE_0);
			qpnp_lbc_charger_enable(chip, SOC, 1);
			goto check_again_later;
		}


	}
check_again_later:
	schedule_delayed_work(&chip->charger_work,
			msecs_to_jiffies(CHG_CHECK_PERIOD_MS));

	return;

}

int
msm_chg_usb_runin_switch(uint32_t  on)
{
	runin_on  = on;
	pr_err("runin_on =%d \n",runin_on);
	return runin_on ;
}
int
qpnp_chg_temp_protect_switch(uint32_t  on)
{
	temp_on  =on;
	pr_err("temp_on =%d \n",temp_on);
	return temp_on ;
}
int
qpnp_chg_charging_enabled_switch(uint32_t  on)
{
	charging_on = on;
	pr_err("charging_on =%d \n",charging_on);
	return charging_on ;
}
int
qpnp_chg_limit_current_switch(uint32_t  on)
{
	limit_on = on;
	pr_err("limit_on =%d \n",limit_on);
	return limit_on ;
}

static ssize_t runin_switch_on_show(struct device *dev,  struct device_attribute *attr, char *buf)
{
	int rc;
	char *s = buf;
	pr_debug("enter\n");
	rc = msm_chg_usb_runin_switch(1);
	if(rc>=0){
		s += sprintf(s, "%s\n", "on");
	}
	if (s != buf)
		/* convert the last space to a newline */
		*(s-1) = '\n';

	return (s - buf);
}

static ssize_t runin_switch_off_show(struct device *dev,  struct device_attribute *attr, char *buf)
{
	int rc;
	char *s = buf;
	pr_debug("enter\n");
	rc = msm_chg_usb_runin_switch(0);
	if(rc>=0){
		s += sprintf(s, "%s\n", "on");
	}
	if (s != buf)

		*(s-1) = '\n';

	return (s - buf);
}
static ssize_t temp_switch_on_show(struct device *dev,  struct device_attribute *attr, char *buf)
{
	int rc;
	char *s = buf;
	pr_debug("enter\n");
	rc = qpnp_chg_temp_protect_switch(1);
	if(rc>=0){
		s += sprintf(s, "%s\n", "on");
	}
	if (s != buf)
		/* convert the last space to a newline */
		*(s-1) = '\n';

	return (s - buf);
}

static ssize_t temp_switch_off_show(struct device *dev,  struct device_attribute *attr, char *buf)
{
	int rc;
	char *s = buf;
	pr_debug("enter\n");
	rc = qpnp_chg_temp_protect_switch(0);
	if(rc>=0){
		s += sprintf(s, "%s\n", "on");
	}
	if (s != buf)

		*(s-1) = '\n';

	return (s - buf);
}
static ssize_t enabled_switch_on_show(struct device *dev,  struct device_attribute *attr, char *buf)
{
	int rc;
	char *s = buf;
	pr_debug("enter\n");
	rc = qpnp_chg_charging_enabled_switch(1);
	if(rc>=0){
		s += sprintf(s, "%s\n", "on");
	}
	if (s != buf)

		*(s-1) = '\n';

	return (s - buf);
}
static ssize_t enabled_switch_off_show(struct device *dev,  struct device_attribute *attr, char *buf)
{
	int rc;
	char *s = buf;
	pr_debug("enter\n");
	rc = qpnp_chg_charging_enabled_switch(0);
	if(rc>=0){
		s += sprintf(s, "%s\n", "on");
	}
	if (s != buf)

		*(s-1) = '\n';

	return (s - buf);
}
static ssize_t limit_current_on_show(struct device *dev,  struct device_attribute *attr, char *buf)
{
	int rc;
	char *s = buf;
	pr_debug("enter\n");
	rc = qpnp_chg_limit_current_switch(1);
	if(rc>=0){
		s += sprintf(s, "%s\n", "on");
	}
	if (s != buf)

		*(s-1) = '\n';

	return (s - buf);
}
static ssize_t limit_current_off_show(struct device *dev,  struct device_attribute *attr, char *buf)
{
	int rc;
	char *s = buf;
	pr_debug("enter\n");
	rc = qpnp_chg_limit_current_switch(0);
	if(rc>=0){
		s += sprintf(s, "%s\n", "on");
	}
	if (s != buf)

		*(s-1) = '\n';

	return (s - buf);
}
static DEVICE_ATTR(runin_switch_on, S_IRUGO | S_IWUSR, runin_switch_on_show, NULL);
static DEVICE_ATTR(runin_switch_off, S_IRUGO | S_IWUSR, runin_switch_off_show, NULL);
static DEVICE_ATTR(temp_switch_on, S_IRUGO | S_IWUSR, temp_switch_on_show, NULL);
static DEVICE_ATTR(temp_switch_off, S_IRUGO | S_IWUSR, temp_switch_off_show, NULL);
static DEVICE_ATTR(enabled_switch_on, S_IRUGO | S_IWUSR, enabled_switch_on_show, NULL);
static DEVICE_ATTR(enabled_switch_off, S_IRUGO | S_IWUSR, enabled_switch_off_show, NULL);
static DEVICE_ATTR(limit_current_on, S_IRUGO | S_IWUSR, limit_current_on_show, NULL);
static DEVICE_ATTR(limit_current_off, S_IRUGO | S_IWUSR, limit_current_off_show, NULL);

static struct attribute *msm_chg_attrs[] = {
	&dev_attr_runin_switch_on.attr,
	&dev_attr_runin_switch_off.attr,
	&dev_attr_temp_switch_on.attr,
	&dev_attr_temp_switch_off.attr,
	&dev_attr_enabled_switch_on.attr,
	&dev_attr_enabled_switch_off.attr,
	&dev_attr_limit_current_on.attr,
	&dev_attr_limit_current_off.attr,
	NULL,
};

static struct attribute_group msm_chg_attr_grp = {
	.attrs = msm_chg_attrs,
};

static int qpnp_lbc_probe(struct spmi_device *spmi)
{
	u8 subtype;
	ktime_t kt;
	struct qpnp_lbc_chip *chip;
	struct resource *resource;
	struct spmi_resource *spmi_resource;
	struct power_supply *usb_psy;
	int rc = 0;

	usb_psy = power_supply_get_by_name("usb");
	if (!usb_psy) {
		pr_err("usb supply not found deferring probe\n");
		return -EPROBE_DEFER;
	}

	chip = devm_kzalloc(&spmi->dev, sizeof(struct qpnp_lbc_chip),
			GFP_KERNEL);
	if (!chip) {
		pr_err("memory allocation failed.\n");
		return -ENOMEM;
	}

	chip->usb_psy = usb_psy;
	chip->dev = &spmi->dev;
	chip->spmi = spmi;
	chip->fake_battery_soc = -EINVAL;
	dev_set_drvdata(&spmi->dev, chip);
	device_init_wakeup(&spmi->dev, 1);
	mutex_init(&chip->jeita_configure_lock);
	mutex_init(&chip->chg_enable_lock);
	spin_lock_init(&chip->hw_access_lock);
	spin_lock_init(&chip->ibat_change_lock);
	spin_lock_init(&chip->irq_lock);
	INIT_WORK(&chip->vddtrim_work, qpnp_lbc_vddtrim_work_fn);
	alarm_init(&chip->vddtrim_alarm, ALARM_REALTIME, vddtrim_callback);

	INIT_DELAYED_WORK(&chip->batt_pres_work, qpnp_batt_pres_work);
	/* Get all device-tree properties */
	rc = qpnp_charger_read_dt_props(chip);
	if (rc) {
		pr_err("Failed to read DT properties rc=%d\n", rc);
		return rc;
	}

	spmi_for_each_container_dev(spmi_resource, spmi) {
		if (!spmi_resource) {
			pr_err("spmi resource absent\n");
			rc = -ENXIO;
			goto fail_chg_enable;
		}

		resource = spmi_get_resource(spmi, spmi_resource,
				IORESOURCE_MEM, 0);
		if (!(resource && resource->start)) {
			pr_err("node %s IO resource absent!\n",
					spmi->dev.of_node->full_name);
			rc = -ENXIO;
			goto fail_chg_enable;
		}

		rc = qpnp_lbc_read(chip, resource->start + PERP_SUBTYPE_REG,
				&subtype, 1);
		if (rc) {
			pr_err("Peripheral subtype read failed rc=%d\n", rc);
			goto fail_chg_enable;
		}

		switch (subtype) {
			case LBC_CHGR_SUBTYPE:
				chip->chgr_base = resource->start;

				/* Get Charger peripheral irq numbers */
				rc = qpnp_lbc_get_irqs(chip, subtype, spmi_resource);
				if (rc) {
					pr_err("Failed to get CHGR irqs rc=%d\n", rc);
					goto fail_chg_enable;
				}
				break;
			case LBC_USB_PTH_SUBTYPE:
				chip->usb_chgpth_base = resource->start;
				rc = qpnp_lbc_get_irqs(chip, subtype, spmi_resource);
				if (rc) {
					pr_err("Failed to get USB_PTH irqs rc=%d\n",
							rc);
					goto fail_chg_enable;
				}
				break;
			case LBC_BAT_IF_SUBTYPE:
				chip->bat_if_base = resource->start;
				chip->vadc_dev = qpnp_get_vadc(chip->dev, "chg");
				if (IS_ERR(chip->vadc_dev)) {
					rc = PTR_ERR(chip->vadc_dev);
					if (rc != -EPROBE_DEFER)
						pr_err("vadc prop missing rc=%d\n",
								rc);
					goto fail_chg_enable;
				}
				/* Get Charger Batt-IF peripheral irq numbers */
				rc = qpnp_lbc_get_irqs(chip, subtype, spmi_resource);
				if (rc) {
					pr_err("Failed to get BAT_IF irqs rc=%d\n", rc);
					goto fail_chg_enable;
				}
				break;
			case LBC_MISC_SUBTYPE:
				chip->misc_base = resource->start;
				break;
			default:
				pr_err("Invalid peripheral subtype=0x%x\n", subtype);
				rc = -EINVAL;
		}
	}

	if (chip->cfg_use_external_charger) {
		pr_warn("Disabling Linear Charger (e-external-charger = 1)\n");
		rc = qpnp_disable_lbc_charger(chip);
		if (rc)
			pr_err("Unable to disable charger rc=%d\n", rc);
		return -ENODEV;
	}

	/* Initialize h/w */
	rc = qpnp_lbc_misc_init(chip);
	if (rc) {
		pr_err("unable to initialize LBC MISC rc=%d\n", rc);
		return rc;
	}
	rc = qpnp_lbc_chg_init(chip);
	if (rc) {
		pr_err("unable to initialize LBC charger rc=%d\n", rc);
		return rc;
	}
	rc = qpnp_lbc_bat_if_init(chip);
	if (rc) {
		pr_err("unable to initialize LBC BAT_IF rc=%d\n", rc);
		return rc;
	}
	rc = qpnp_lbc_usb_path_init(chip);
	if (rc) {
		pr_err("unable to initialize LBC USB path rc=%d\n", rc);
		return rc;
	}

	if (chip->bat_if_base) {
		chip->batt_present = qpnp_lbc_is_batt_present(chip);
		chip->batt_psy.name = "battery";
		chip->batt_psy.type = POWER_SUPPLY_TYPE_BATTERY;
		chip->batt_psy.properties = msm_batt_power_props;
		chip->batt_psy.num_properties =
			ARRAY_SIZE(msm_batt_power_props);
		chip->batt_psy.get_property = qpnp_batt_power_get_property;
		chip->batt_psy.set_property = qpnp_batt_power_set_property;
		chip->batt_psy.property_is_writeable =
			qpnp_batt_property_is_writeable;
		chip->batt_psy.external_power_changed =
			qpnp_batt_external_power_changed;
		chip->batt_psy.supplied_to = pm_batt_supplied_to;
		chip->batt_psy.num_supplicants =
			ARRAY_SIZE(pm_batt_supplied_to);
		rc = power_supply_register(chip->dev, &chip->batt_psy);
		if (rc < 0) {
			pr_err("batt failed to register rc=%d\n", rc);
			goto fail_chg_enable;
		}
	}

	if ((chip->cfg_cool_bat_decidegc || chip->cfg_warm_bat_decidegc)
			&& chip->bat_if_base) {
		chip->adc_param.low_temp = chip->cfg_cool_bat_decidegc;
		chip->adc_param.high_temp = chip->cfg_warm_bat_decidegc;
		chip->adc_param.timer_interval = ADC_MEAS1_INTERVAL_1S;
		chip->adc_param.state_request = ADC_TM_HIGH_LOW_THR_ENABLE;
		chip->adc_param.btm_ctx = chip;
		chip->adc_param.threshold_notification =
			qpnp_lbc_jeita_adc_notification;
		chip->adc_param.channel = LR_MUX1_BATT_THERM;

		if (get_prop_batt_present(chip)) {
			rc = qpnp_adc_tm_channel_measure(chip->adc_tm_dev,
					&chip->adc_param);
			if (rc) {
				pr_err("request ADC error rc=%d\n", rc);
				goto unregister_batt;
			}
		}
	}

	rc = qpnp_lbc_bat_if_configure_btc(chip);
	if (rc) {
		pr_err("Failed to configure btc rc=%d\n", rc);
		goto unregister_batt;
	}
	INIT_DELAYED_WORK(&chip->charger_work, qpnp_lbc_charging_work);
	wake_lock_init(&dc_chg_wake_lock, WAKE_LOCK_SUSPEND, "dc_chg_lock");
	/* Get/Set charger's initial status */
	determine_initial_status(chip);
	rc = sysfs_create_group(&chip->dev->kobj, &msm_chg_attr_grp);
	if (rc < 0) {
		pr_err("%s: Failed to create the sysfs entry\n", __func__);
	}
	rc = qpnp_lbc_request_irqs(chip);
	if (rc) {
		pr_err("unable to initialize LBC MISC rc=%d\n", rc);
		goto unregister_batt;
	}

	if (chip->cfg_charging_disabled && !get_prop_batt_present(chip))
		pr_info("Battery absent and charging disabled !!!\n");

	/* Configure initial alarm for VDD trim */
	if ((chip->supported_feature_flag & VDD_TRIM_SUPPORTED) &&
			qpnp_lbc_is_fastchg_on(chip)) {
		kt = ns_to_ktime(TRIM_PERIOD_NS);
		alarm_start_relative(&chip->vddtrim_alarm, kt);
	}

	if (qpnp_lbc_is_usb_chg_plugged_in(chip)){
		schedule_delayed_work(&chip->charger_work,
				msecs_to_jiffies(CHG_CHECK_PERIOD_MS));
		if(!dc_chg_lock){
			printk("%s,add dc_chg_wake_lock:begin charging\n",__func__);
			wake_lock(&dc_chg_wake_lock);
			dc_chg_lock=true;
		}
	}

	pr_info("Probe chg_dis=%d bpd=%d usb=%d batt_pres=%d batt_volt=%d soc=%d\n",
			chip->cfg_charging_disabled,
			chip->cfg_bpd_detection,
			qpnp_lbc_is_usb_chg_plugged_in(chip),
			get_prop_batt_present(chip),
			get_prop_battery_voltage_now(chip),
			get_prop_capacity(chip));

	return 0;

unregister_batt:
	if (chip->bat_if_base)
		power_supply_unregister(&chip->batt_psy);
fail_chg_enable:
	dev_set_drvdata(&spmi->dev, NULL);
	return rc;
}

static int qpnp_lbc_remove(struct spmi_device *spmi)
{
	struct qpnp_lbc_chip *chip = dev_get_drvdata(&spmi->dev);

	if (chip->supported_feature_flag & VDD_TRIM_SUPPORTED) {
		alarm_cancel(&chip->vddtrim_alarm);
		cancel_work_sync(&chip->vddtrim_work);
	}
	if (chip->bat_if_base)
		power_supply_unregister(&chip->batt_psy);
	cancel_delayed_work_sync(&chip->charger_work);
	mutex_destroy(&chip->jeita_configure_lock);
	mutex_destroy(&chip->chg_enable_lock);
	dev_set_drvdata(&spmi->dev, NULL);
	return 0;
}

static struct of_device_id qpnp_lbc_match_table[] = {
	{ .compatible = QPNP_CHARGER_DEV_NAME, },
	{}
};

static struct spmi_driver qpnp_lbc_driver = {
	.probe		= qpnp_lbc_probe,
	.remove		= qpnp_lbc_remove,
	.driver		= {
		.name		= QPNP_CHARGER_DEV_NAME,
		.owner		= THIS_MODULE,
		.of_match_table	= qpnp_lbc_match_table,
	},
};

/*
 * qpnp_lbc_init() - register spmi driver for qpnp-chg
 */
static int __init qpnp_lbc_init(void)
{
	return spmi_driver_register(&qpnp_lbc_driver);
}
module_init(qpnp_lbc_init);

static void __exit qpnp_lbc_exit(void)
{
	spmi_driver_unregister(&qpnp_lbc_driver);
}
module_exit(qpnp_lbc_exit);

MODULE_DESCRIPTION("QPNP Linear charger driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:" QPNP_CHARGER_DEV_NAME);
