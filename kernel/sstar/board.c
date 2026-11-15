// SPDX-License-Identifier: GPL-2.0
/*
 * Board level helpers of the SigmaStar BSP used by mhal.ko: padmux
 * (drivers/sstar/padmux), GPIO (gpio/mdrv_gpio*.c), core voltage
 * (voltage/voltage_ctrl.c) and the temperature sensor
 * (cpufreq/infinity6e/cpufreq.c).
 *
 * GPIO: the vendor PAD_* numbers (include/infinity6e/gpio.h) are mapped to
 * the lines of the mainline sstar,ssc338q-pinctrl gpio_chip and driven through
 * gpiolib with raw values (the vendor passes physical levels). padmux reports
 * "not used" so mhal skips its own pad mux path.
 */
#include <linux/err.h>
#include <linux/gpio/consumer.h>
#include <linux/gpio/driver.h>
#include <linux/gpio/machine.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/printk.h>
#include <linux/thermal.h>

#include "osm_riu.h"
#include "osm_sstar.h"

#define PAD_UNKNOWN		0xffff
#define CORE_VOLTAGE_MV		900
#define TEMP_FALLBACK_C		50

int mdrv_padmux_active(void)
{
	return 0;
}
EXPORT_SYMBOL(mdrv_padmux_active);

int mdrv_padmux_getpad(int puse)
{
	return PAD_UNKNOWN;
}
EXPORT_SYMBOL(mdrv_padmux_getpad);

int mdrv_padmux_getmode(int puse)
{
	return PAD_UNKNOWN;
}
EXPORT_SYMBOL(mdrv_padmux_getmode);

/*
 * Vendor pad numbers (infinity6e/gpio.h) to mainline gpio_chip lines
 * (dt-bindings/gpio/sstar,ssc338q-gpio.h).
 */
#define VPAD_PM_GPIO0		6
#define VPAD_PM_GPIO10		16
#define VPAD_SAR_GPIO0		23
#define VPAD_SAR_GPIO3		26
#define VPAD_SR0_IO00		67
#define VPAD_SR0_IO19		86

#define LINE_SR0_IO00		52
#define LINE_PM_GPIO0		92
#define LINE_SAR_GPIO0		103
#define LINE_COUNT		107

static struct gpio_device *gdev;
static struct gpio_desc *descs[LINE_COUNT];
static DEFINE_MUTEX(gpio_lock);

static int vpad_to_line(unsigned int pad)
{
	if (pad >= VPAD_SR0_IO00 && pad <= VPAD_SR0_IO19)
		return pad - VPAD_SR0_IO00 + LINE_SR0_IO00;
	if (pad >= VPAD_PM_GPIO0 && pad <= VPAD_PM_GPIO10)
		return pad - VPAD_PM_GPIO0 + LINE_PM_GPIO0;
	if (pad >= VPAD_SAR_GPIO0 && pad <= VPAD_SAR_GPIO3)
		return pad - VPAD_SAR_GPIO0 + LINE_SAR_GPIO0;
	return -EINVAL;
}

static int match_pinctrl(struct gpio_chip *gc, const void *data)
{
	return of_device_is_compatible(dev_of_node(gc->parent),
				       "sstar,ssc338q-pinctrl");
}

/* Request (once) the line of a vendor pad, keeping the descriptor */
static struct gpio_desc *pad_desc(unsigned int pad)
{
	struct gpio_desc *desc;
	int line = vpad_to_line(pad);

	if (line < 0) {
		pr_warn_once("osm_sstar: GPIO pad %u not mapped\n", pad);
		return NULL;
	}

	mutex_lock(&gpio_lock);
	if (!gdev)
		gdev = gpio_device_find(NULL, match_pinctrl);
	if (!gdev) {
		mutex_unlock(&gpio_lock);
		pr_warn_once("osm_sstar: pinctrl gpio chip not found\n");
		return NULL;
	}

	desc = descs[line];
	if (!desc) {
		desc = gpiochip_request_own_desc(gpio_device_get_chip(gdev),
						 line, "osm_sstar", 0,
						 GPIOD_ASIS);
		if (IS_ERR(desc)) {
			pr_err("osm_sstar: GPIO pad %u (line %d) request failed: %pe\n",
			       pad, line, desc);
			desc = NULL;
		} else {
			descs[line] = desc;
		}
	}
	mutex_unlock(&gpio_lock);

	return desc;
}

/* Release the GPIO lines, call from the module exit */
void osm_board_exit(void)
{
	int i;

	for (i = 0; i < LINE_COUNT; i++) {
		if (descs[i])
			gpiochip_free_own_desc(descs[i]);
		descs[i] = NULL;
	}
	if (gdev)
		gpio_device_put(gdev);
	gdev = NULL;
}

/*
 * The BSP sets the pad mux of a pad here. Mainline has no per-pad mux
 * for the sensor pads: PADTOP pads are plain GPIOs unless a function group
 * claims them, PM/SAR pads are switched to GPIO by the chip's request().
 */
int MDrv_GPIO_PadVal_Set(u8 u8IndexGPIO, u32 u32PadMode)
{
	pr_info("osm_sstar: pad %u mode %#x\n", u8IndexGPIO, u32PadMode);
	return 0;
}
EXPORT_SYMBOL(MDrv_GPIO_PadVal_Set);

/* chip is unused (NULL from mhal), offset is the vendor pad number */
int camdriver_gpio_direction_output(struct gpio_chip *chip, unsigned int offset,
				    int value)
{
	struct gpio_desc *desc = pad_desc(offset);

	if (!desc)
		return 0;

	pr_info("osm_sstar: GPIO pad %u output %d\n", offset, value);
	return gpiod_direction_output_raw(desc, value);
}
EXPORT_SYMBOL(camdriver_gpio_direction_output);

void camdriver_gpio_set(struct gpio_chip *chip, unsigned int offset, int value)
{
	struct gpio_desc *desc = pad_desc(offset);

	if (desc)
		pr_info("osm_sstar: GPIO pad %u set %d\n", offset, value);
	if (desc)
		gpiod_set_raw_value(desc, value);
}
EXPORT_SYMBOL(camdriver_gpio_set);

/* demander is a VOLTAGE_DEMANDER_E, int-sized on ARM EABI */
void set_core_voltage(unsigned int demander, int mV)
{
}
EXPORT_SYMBOL(set_core_voltage);

int get_core_voltage(void)
{
	return CORE_VOLTAGE_MV;
}
EXPORT_SYMBOL(get_core_voltage);

/* Degrees C, like the BSP (the thermal core reports millidegrees) */
int ms_get_temp(void)
{
	struct thermal_zone_device *tz;
	int temp;

	tz = thermal_zone_get_zone_by_name("soc-thermal");
	if (IS_ERR_OR_NULL(tz) || thermal_zone_get_temp(tz, &temp))
		return TEMP_FALLBACK_C;

	return temp / 1000;
}
EXPORT_SYMBOL(ms_get_temp);
