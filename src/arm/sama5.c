/*
 * Author: Norbert Wesp <nwesp@phytec.de>
 * Copyright (c) 2016 Phytec Messtechnik GmbH.
 *
 * Based on src/arm/beaglebone.c
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
 * LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
 * OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
 * WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <mraa/types.h>

#include "common.h"
#include "arm/sama5.h"

#define PLATFORM_NAME_SAMA5 "Atmel SAMA5"

mraa_result_t
mraa_sam5d_uart_init_pre(int index)
{
    char devpath[MAX_SIZE];

    sprintf(devpath, "/dev/ttyS%u", index);
    if (!mraa_file_exist(devpath)) {
	syslog(LOG_ERR, "uart: Device not initialized");
    } else {
	plat->uart_dev[index].device_path = devpath;
	return MRAA_SUCCESS;
    }
    return MRAA_ERROR_NO_RESOURCES;
}

mraa_result_t
mraa_sam5d_spi_init_pre(int index)
{
    mraa_result_t ret = MRAA_ERROR_INVALID_PARAMETER;
    char devpath[MAX_SIZE];
    int deviceindex = 0;

    if ((index == 0) && mraa_link_targets("/sys/class/spidev/spidev3.0", "48030000")) {
	printf("spidev..\n");
	deviceindex = 3;
    }
    if (deviceindex == 0) {
	deviceindex = 3;
    }
    snprintf(devpath, MAX_SIZE, "/dev/spidev%u.0", deviceindex);
    if (mraa_file_exist(devpath)) {
	plat->spi_bus[index].bus_id = deviceindex;
	ret = MRAA_SUCCESS;
    } else {
	syslog(LOG_NOTICE, "spi: Device not initialized");
    }
    return ret;
}

/* NOT DONE / TESTED YET */
mraa_result_t
mraa_sam5d_i2c_init_pre(unsigned int bus)
{
    mraa_result_t ret = MRAA_ERROR_NO_RESOURCES;
    char devpath[MAX_SIZE];

    sprintf(devpath, "/dev/i2c-%u", plat->i2c_bus[bus].bus_id);
    if (!mraa_file_exist(devpath)) {
	syslog(LOG_INFO, "i2c: %s doesn't exist ", devpath);
	syslog(LOG_ERR, "i2c: Device not initialized");

	return ret;
    }
    return MRAA_SUCCESS;
}

/* NOT DONE / TESTED YET */
mraa_pwm_context
mraa_sam5d_pwm_init_replace(int pin)
{
    char devpath[MAX_SIZE];

    if (plat == NULL) {
	syslog(LOG_ERR, "pwm: Platform Not Initialised");
	return NULL;
    }
    if (plat->pins[pin].capabilities.pwm != 1) {
	syslog(LOG_ERR, "pwm: pin not capable of pwm");
	return NULL;
    }
    if (!mraa_file_exist(SYSFS_CLASS_PWM "pwmchip0")) {
	syslog(LOG_ERR, "pwm: pwmchip0 not found");
	return NULL;
    }

    sprintf(devpath, SYSFS_CLASS_PWM "pwm%u", plat->pins[pin].pwm.pinmap);
    if (!mraa_file_exist(devpath)) {
	FILE *fh;
	fh = fopen(SYSFS_CLASS_PWM "export", "w");
	if (fh == NULL) {
		syslog(LOG_ERR, "pwm: Failed to open /sys/class/pwm/export for writing, check access "
				"rights for user");
		return NULL;
	}
	if (fprintf(fh, "%d", plat->pins[pin].pwm.pinmap) < 0) {
		syslog(LOG_ERR, "pwm: Failed to write to sysfs-pwm-export");
	}
	fclose(fh);
    }

    if (mraa_file_exist(devpath)) {
	mraa_pwm_context dev = (mraa_pwm_context) calloc(1, sizeof(struct _pwm));
	if (dev == NULL)
		return NULL;
	dev->duty_fp = -1;
	dev->chipid = -1;
	dev->pin = plat->pins[pin].pwm.pinmap;
	dev->period = -1;
	return dev;
    } else
	syslog(LOG_ERR, "pwm: pin not initialized");
    return NULL;
}


mraa_board_t*
mraa_sama5()
{
    unsigned int uart0_enabled = 0;
    unsigned int uart2_enabled = 0;
    unsigned int uart3_enabled = 0;
    unsigned int uart4_enabled = 0;

    if (mraa_file_exist("/sys/class/tty/ttyS1"))
	uart0_enabled = 1;
    else
	uart2_enabled = 0;

    if (mraa_file_exist("/sys/class/tty/ttyS3"))
	uart3_enabled = 1;

    if (mraa_file_exist("/sys/class/tty/ttyS4"))
	uart4_enabled = 1;

    mraa_board_t *b = (mraa_board_t *) calloc(1, sizeof(mraa_board_t));
    if (b == NULL)
	return NULL;
    b->platform_name = PLATFORM_NAME_SAMA5;
    b->phy_pin_count = MRAA_SAMA5D27_PINCOUNT;

    if (b->platform_name == NULL) {
	goto error;
    }

    b->aio_count = 7;
    b->adc_raw = 12;
    b->adc_supported = 12;

    b->pwm_default_period = 500;
    b->pwm_max_period = 2147483;
    b->pwm_min_period = 1;

    b->pins = (mraa_pininfo_t *) calloc(b->phy_pin_count, sizeof(mraa_pininfo_t));
    if (b->pins == NULL) {
	goto error;
    }

    b->adv_func = (mraa_adv_func_t *) calloc(1, sizeof(mraa_adv_func_t));
    if (b->adv_func == NULL) {
	free(b->pins);
	goto error;
    }

    b->adv_func->uart_init_pre = &mraa_sam5d_uart_init_pre;
    b->adv_func->spi_init_pre = &mraa_sam5d_spi_init_pre;
    b->adv_func->i2c_init_pre = &mraa_sam5d_i2c_init_pre;
    b->adv_func->pwm_init_replace = &mraa_sam5d_pwm_init_replace;

    strncpy(b->pins[1].name, "VCC3V3", MRAA_PIN_NAME_SIZE);
    b->pins[1].capabilities = (mraa_pincapabilities_t){ 1, 0, 0, 0, 0, 0, 0, 0 };

    strncpy(b->pins[2].name, "VCC5V", MRAA_PIN_NAME_SIZE);
    b->pins[2].capabilities = (mraa_pincapabilities_t){ 1, 0, 0, 0, 0, 0, 0, 0 };

    strncpy(b->pins[3].name, "VCC3V3", MRAA_PIN_NAME_SIZE);
    b->pins[3].capabilities = (mraa_pincapabilities_t){ 1, 0, 0, 0, 0, 0, 0, 0 };

    strncpy(b->pins[4].name, "VCC5V", MRAA_PIN_NAME_SIZE);
    b->pins[4].capabilities = (mraa_pincapabilities_t){ 1, 0, 0, 0, 0, 0, 0, 0 };

    strncpy(b->pins[5].name, "VCC3V3", MRAA_PIN_NAME_SIZE);
    b->pins[5].capabilities = (mraa_pincapabilities_t){ 1, 0, 0, 0, 0, 0, 0, 0 };

    strncpy(b->pins[6].name, "VCC5V", MRAA_PIN_NAME_SIZE);
    b->pins[6].capabilities = (mraa_pincapabilities_t){ 1, 0, 0, 0, 1, 0, 0, 0 };
    b->pins[6].spi.mux_total = 0;

    strncpy(b->pins[7].name, "CLK_AUDIO", MRAA_PIN_NAME_SIZE);
    b->pins[7].capabilities = (mraa_pincapabilities_t){ 1, 0, 0, 0, 0, 0, 0, 0 };

    strncpy(b->pins[8].name, "COMPP", MRAA_PIN_NAME_SIZE);
    b->pins[8].capabilities = (mraa_pincapabilities_t){ 1, 0, 0, 0, 0, 0, 0, 0 };

    strncpy(b->pins[9].name, "STROBE", MRAA_PIN_NAME_SIZE);
    b->pins[9].capabilities = (mraa_pincapabilities_t){ 1, 0, 0, 0, 0, 0, 0, 0 };

    strncpy(b->pins[10].name, "COMPN", MRAA_PIN_NAME_SIZE);
    b->pins[10].capabilities = (mraa_pincapabilities_t){ 1, 0, 0, 0, 0, 0, 0, 0 };

    strncpy(b->pins[11].name, "PIOBU6", MRAA_PIN_NAME_SIZE);
    b->pins[11].capabilities = (mraa_pincapabilities_t){ 1, 0, 0, 0, 0, 0, 0, 0 };

    strncpy(b->pins[12].name, "PD19", MRAA_PIN_NAME_SIZE);
    b->pins[12].capabilities = (mraa_pincapabilities_t){ 1, 1, 0, 0, 0, 0, 0, 0 };
    b->pins[12].gpio.pinmap = (3*32 + 19);
    b->pins[12].gpio.parent_id = 0;
    b->pins[12].gpio.mux_total = 0;
    b->pins[12].uart.mux_total = 0;

    strncpy(b->pins[13].name, "PD20", MRAA_PIN_NAME_SIZE);
    b->pins[13].capabilities = (mraa_pincapabilities_t){ 1, 1, 0, 0, 0, 0, 0, 0 };
    b->pins[13].gpio.pinmap = (3*32 + 20);
    b->pins[13].gpio.parent_id = 0;
    b->pins[13].gpio.mux_total = 0;
    b->pins[13].uart.mux_total = 0;

    strncpy(b->pins[14].name, "PD30", MRAA_PIN_NAME_SIZE);
    b->pins[14].capabilities = (mraa_pincapabilities_t){ 1, 1, 0, 0, 0, 0, 0, 0 };
    b->pins[14].gpio.pinmap = (3*32 + 30);
    b->pins[14].gpio.parent_id = 0;
    b->pins[14].gpio.mux_total = 0;
    b->pins[14].uart.mux_total = 0;

    strncpy(b->pins[15].name, "PD27", MRAA_PIN_NAME_SIZE);
    b->pins[15].capabilities = (mraa_pincapabilities_t){ 1, 1, 0, 0, 0, 0, 0, 0 };
    b->pins[15].gpio.pinmap = (3*32 + 27);
    b->pins[15].gpio.parent_id = 0;
    b->pins[15].gpio.mux_total = 0;
    b->pins[15].uart.mux_total = 0;

    strncpy(b->pins[16].name, "PD28", MRAA_PIN_NAME_SIZE);
    b->pins[16].capabilities = (mraa_pincapabilities_t){ 1, 1, 0, 0, 0, 0, 0, 0 };
    b->pins[16].gpio.pinmap = (3*32 + 28);
    b->pins[16].gpio.parent_id = 0;
    b->pins[16].gpio.mux_total = 0;
    b->pins[16].uart.mux_total = 0;

    strncpy(b->pins[18].name, "PIOBU3", MRAA_PIN_NAME_SIZE);
    b->pins[18].capabilities = (mraa_pincapabilities_t){ 1, 0, 0, 0, 0, 0, 0, 0 };

    strncpy(b->pins[19].name, "PIOBU2", MRAA_PIN_NAME_SIZE);
    b->pins[19].capabilities = (mraa_pincapabilities_t){ 1, 0, 0, 0, 0, 0, 0, 0 };

    strncpy(b->pins[20].name, "PD26", MRAA_PIN_NAME_SIZE);
    b->pins[20].capabilities = (mraa_pincapabilities_t){ 1, 1, 0, 0, 0, 0, 0, 0 };
    b->pins[20].gpio.pinmap = (3*32 + 26);
    b->pins[20].gpio.parent_id = 0;
    b->pins[20].gpio.mux_total = 0;
    b->pins[20].uart.mux_total = 0;

    strncpy(b->pins[21].name, "PIOBU1", MRAA_PIN_NAME_SIZE);
    b->pins[21].capabilities = (mraa_pincapabilities_t){ 1, 0, 0, 0, 0, 0, 0, 0 };

    strncpy(b->pins[22].name, "RXD", MRAA_PIN_NAME_SIZE);
    b->pins[22].capabilities = (mraa_pincapabilities_t){ 1, 0, 0, 0, 0, 0, 0, 0 };

    strncpy(b->pins[23].name, "PD08", MRAA_PIN_NAME_SIZE);
    b->pins[23].capabilities = (mraa_pincapabilities_t){ 1, 1, 0, 0, 0, 0, 0, 0 };
    b->pins[23].gpio.pinmap = (3*32 + 8);
    b->pins[23].gpio.parent_id = 0;
    b->pins[23].gpio.mux_total = 0;
    b->pins[23].uart.mux_total = 0;

    strncpy(b->pins[24].name, "PD04", MRAA_PIN_NAME_SIZE);
    b->pins[24].capabilities = (mraa_pincapabilities_t){ 1, 1, 0, 0, 0, 0, 0, 0 };
    b->pins[24].gpio.pinmap = (3*32 + 4);
    b->pins[24].gpio.parent_id = 0;
    b->pins[24].gpio.mux_total = 0;
    b->pins[24].uart.mux_total = 0;

    strncpy(b->pins[25].name, "PD07", MRAA_PIN_NAME_SIZE);
    b->pins[25].capabilities = (mraa_pincapabilities_t){ 1, 1, 0, 0, 0, 0, 0, 0 };
    b->pins[25].gpio.pinmap = (3*32 + 7);
    b->pins[25].gpio.parent_id = 0;
    b->pins[25].gpio.mux_total = 0;
    b->pins[25].uart.mux_total = 0;

    strncpy(b->pins[26].name, "PD06", MRAA_PIN_NAME_SIZE);
    b->pins[26].capabilities = (mraa_pincapabilities_t){ 1, 1, 0, 0, 0, 0, 0, 0 };
    b->pins[26].gpio.pinmap = (3*32 + 6);
    b->pins[26].gpio.parent_id = 0;
    b->pins[26].gpio.mux_total = 0;
    b->pins[26].uart.mux_total = 0;

    strncpy(b->pins[27].name, "GND", MRAA_PIN_NAME_SIZE);
    b->pins[27].capabilities = (mraa_pincapabilities_t){ 1, 0, 0, 0, 0, 0, 0, 0 };

    strncpy(b->pins[28].name, "GND", MRAA_PIN_NAME_SIZE);
    b->pins[28].capabilities = (mraa_pincapabilities_t){ 1, 0, 0, 0, 0, 0, 0, 0 };

    strncpy(b->pins[29].name, "GND", MRAA_PIN_NAME_SIZE);
    b->pins[29].capabilities = (mraa_pincapabilities_t){ 1, 0, 0, 0, 0, 0, 0, 0 };

    strncpy(b->pins[30].name, "GND", MRAA_PIN_NAME_SIZE);
    b->pins[30].capabilities = (mraa_pincapabilities_t){ 1, 0, 0, 0, 0, 0, 0, 0 };

    strncpy(b->pins[31].name, "PD22", MRAA_PIN_NAME_SIZE);
    b->pins[31].capabilities = (mraa_pincapabilities_t){ 1, 1, 0, 0, 0, 1, 0, 0 };
    b->pins[31].i2c.mux_total = 0;
    b->pins[31].gpio.pinmap = (3*32 + 22);
    b->pins[31].gpio.parent_id = 0;
    b->pins[31].gpio.mux_total = 0;
    b->pins[31].uart.mux_total = 0;

    strncpy(b->pins[32].name, "PD21", MRAA_PIN_NAME_SIZE);
    b->pins[32].capabilities = (mraa_pincapabilities_t){ 1, 1, 0, 0, 0, 1, 0, 0 };
    b->pins[32].i2c.mux_total = 0;
    b->pins[32].gpio.pinmap = (3*32 + 21);
    b->pins[32].gpio.parent_id = 0;
    b->pins[32].gpio.mux_total = 0;
    b->pins[32].uart.mux_total = 0;

    strncpy(b->pins[33].name, "DATA", MRAA_PIN_NAME_SIZE);
    b->pins[33].capabilities = (mraa_pincapabilities_t){ 1, 0, 0, 0, 0, 0, 0, 0 };

    strncpy(b->pins[34].name, "PIOBU7", MRAA_PIN_NAME_SIZE);
    b->pins[34].capabilities = (mraa_pincapabilities_t){ 1, 0, 0, 0, 0, 0, 0, 0 };

    strncpy(b->pins[35].name, "PC12", MRAA_PIN_NAME_SIZE);
    b->pins[35].capabilities = (mraa_pincapabilities_t){ 1, 1, 0, 0, 0, 0, 0, 0 };
    b->pins[35].gpio.pinmap = (2*32 + 12);
    b->pins[35].gpio.parent_id = 0;
    b->pins[35].gpio.mux_total = 0;
    b->pins[35].uart.mux_total = 0;

    strncpy(b->pins[36].name, "PC25", MRAA_PIN_NAME_SIZE);
    b->pins[36].capabilities = (mraa_pincapabilities_t){ 1, 1, 0, 0, 0, 0, 0, 0 };
    b->pins[36].gpio.pinmap = (2*32 + 25);
    b->pins[36].gpio.parent_id = 0;
    b->pins[36].gpio.mux_total = 0;
    b->pins[36].uart.mux_total = 0;

    strncpy(b->pins[37].name, "PC23", MRAA_PIN_NAME_SIZE);
    b->pins[37].capabilities = (mraa_pincapabilities_t){ 1, 1, 0, 0, 0, 0, 0, 0 };
    b->pins[37].gpio.pinmap = (2*32 + 23);
    b->pins[37].gpio.parent_id = 0;
    b->pins[37].gpio.mux_total = 0;

    strncpy(b->pins[38].name, "PC18", MRAA_PIN_NAME_SIZE);
    b->pins[38].capabilities = (mraa_pincapabilities_t){ 1, 1, 0, 0, 0, 0, 0, 0 };
    b->pins[38].gpio.pinmap = (2*32 + 8);
    b->pins[38].gpio.parent_id = 0;
    b->pins[38].gpio.mux_total = 0;

    strncpy(b->pins[39].name, "PA13", MRAA_PIN_NAME_SIZE);
    b->pins[39].capabilities = (mraa_pincapabilities_t){ 1, 1, 0, 0, 0, 0, 0, 0 };
    b->pins[39].gpio.pinmap = (13);
    b->pins[39].gpio.parent_id = 0;
    b->pins[39].gpio.mux_total = 0;

    strncpy(b->pins[40].name, "PA12", MRAA_PIN_NAME_SIZE);
    b->pins[40].capabilities = (mraa_pincapabilities_t){ 1, 1, 0, 0, 0, 0, 0, 0 };
    b->pins[40].gpio.pinmap = (12);
    b->pins[40].gpio.parent_id = 0;
    b->pins[40].gpio.mux_total = 0;

    strncpy(b->pins[41].name, "PA31", MRAA_PIN_NAME_SIZE);
    b->pins[41].capabilities = (mraa_pincapabilities_t){ 1, 1, 0, 0, 0, 0, 0, 0 };
    b->pins[41].gpio.pinmap = (31);
    b->pins[41].gpio.parent_id = 0;
    b->pins[41].gpio.mux_total = 0;

    strncpy(b->pins[42].name, "SHDN", MRAA_PIN_NAME_SIZE);
    b->pins[42].capabilities = (mraa_pincapabilities_t){ 1, 0, 0, 0, 0, 0, 0, 0 };

    strncpy(b->pins[43].name, "PA29", MRAA_PIN_NAME_SIZE);
    b->pins[43].capabilities = (mraa_pincapabilities_t){ 1, 1, 0, 0, 0, 0, 0, 0 };
    b->pins[43].gpio.pinmap = (29);
    b->pins[43].gpio.parent_id = 0;
    b->pins[43].gpio.mux_total = 0;

    strncpy(b->pins[44].name, "PA14", MRAA_PIN_NAME_SIZE);
    b->pins[44].capabilities = (mraa_pincapabilities_t){ 1, 1, 0, 0, 0, 0, 0, 0 };
    b->pins[44].gpio.pinmap = (14);
    b->pins[44].gpio.parent_id = 0;
    b->pins[44].gpio.mux_total = 0;

    strncpy(b->pins[45].name, "PC31", MRAA_PIN_NAME_SIZE);
    b->pins[45].capabilities = (mraa_pincapabilities_t){ 1, 1, 0, 0, 0, 0, 0, 0 };
    b->pins[45].gpio.pinmap = (2*32 + 31);
    b->pins[45].gpio.parent_id = 0;
    b->pins[45].gpio.mux_total = 0;

    strncpy(b->pins[46].name, "PC16", MRAA_PIN_NAME_SIZE);
    b->pins[46].capabilities = (mraa_pincapabilities_t){ 1, 1, 0, 0, 0, 0, 0, 0 };
    b->pins[46].gpio.pinmap = (2*32 + 16);
    b->pins[46].gpio.parent_id = 0;
    b->pins[46].gpio.mux_total = 0;

    strncpy(b->pins[47].name, "PB09", MRAA_PIN_NAME_SIZE);
    b->pins[47].capabilities = (mraa_pincapabilities_t){ 1, 1, 0, 0, 0, 0, 0, 0 };
    b->pins[47].gpio.pinmap = (1*32 + 9);
    b->pins[47].gpio.parent_id = 0;
    b->pins[47].gpio.mux_total = 0;

    strncpy(b->pins[48].name, "PB07", MRAA_PIN_NAME_SIZE);
    b->pins[48].capabilities = (mraa_pincapabilities_t){ 1, 1, 0, 0, 0, 0, 0, 0 };
    b->pins[48].gpio.pinmap = (1*32 + 7);
    b->pins[48].gpio.parent_id = 0;
    b->pins[48].gpio.mux_total = 0;

    strncpy(b->pins[49].name, "GND", MRAA_PIN_NAME_SIZE);
    b->pins[49].capabilities = (mraa_pincapabilities_t){ 1, 0, 0, 0, 0, 0, 0, 0 };

    strncpy(b->pins[50].name, "GND", MRAA_PIN_NAME_SIZE);
    b->pins[50].capabilities = (mraa_pincapabilities_t){ 1, 0, 0, 0, 0, 0, 0, 0 };

    strncpy(b->pins[51].name, "PB05", MRAA_PIN_NAME_SIZE);
    b->pins[51].capabilities = (mraa_pincapabilities_t){ 1, 1, 0, 0, 0, 0, 0, 0 };
    b->pins[51].gpio.pinmap = (1*32 + 5);
    b->pins[51].gpio.parent_id = 0;
    b->pins[51].gpio.mux_total = 0;

    strncpy(b->pins[52].name, "PB10", MRAA_PIN_NAME_SIZE);
    b->pins[52].capabilities = (mraa_pincapabilities_t){ 1, 1, 0, 0, 0, 0, 0, 0 };
    b->pins[52].gpio.pinmap = (1*32 + 10);
    b->pins[52].gpio.parent_id = 0;
    b->pins[52].gpio.mux_total = 0;

    strncpy(b->pins[53].name, "PB08", MRAA_PIN_NAME_SIZE);
    b->pins[53].capabilities = (mraa_pincapabilities_t){ 1, 1, 0, 0, 0, 0, 0, 0 };
    b->pins[53].gpio.pinmap = (1*32 + 8);
    b->pins[53].gpio.parent_id = 0;
    b->pins[53].gpio.mux_total = 0;

    strncpy(b->pins[54].name, "VLED+", MRAA_PIN_NAME_SIZE);
    b->pins[54].capabilities = (mraa_pincapabilities_t){ 1, 0, 0, 0, 0, 0, 0, 0 };

    strncpy(b->pins[55].name, "nRST", MRAA_PIN_NAME_SIZE);
    b->pins[55].capabilities = (mraa_pincapabilities_t){ 1, 0, 0, 0, 0, 0, 0, 0 };

    strncpy(b->pins[56].name, "VLED+", MRAA_PIN_NAME_SIZE);
    b->pins[56].capabilities = (mraa_pincapabilities_t){ 1, 0, 0, 0, 0, 0, 0, 0 };

    strncpy(b->pins[57].name, "GND", MRAA_PIN_NAME_SIZE);
    b->pins[57].capabilities = (mraa_pincapabilities_t){ 1, 0, 0, 0, 0, 0, 0, 0 };

    strncpy(b->pins[58].name, "GND", MRAA_PIN_NAME_SIZE);
    b->pins[58].capabilities = (mraa_pincapabilities_t){ 1, 0, 0, 0, 0, 0, 0, 0 };

    strncpy(b->pins[59].name, "GND", MRAA_PIN_NAME_SIZE);
    b->pins[59].capabilities = (mraa_pincapabilities_t){ 1, 0, 0, 0, 0, 0, 0, 0 };

    strncpy(b->pins[60].name, "GND", MRAA_PIN_NAME_SIZE);
    b->pins[60].capabilities = (mraa_pincapabilities_t){ 1, 0, 0, 0, 0, 0, 0, 0 };

    strncpy(b->pins[61].name, "PC13", MRAA_PIN_NAME_SIZE);
    b->pins[61].capabilities = (mraa_pincapabilities_t){ 1, 1, 0, 0, 0, 0, 0, 0 };
    b->pins[61].gpio.pinmap = (2*32 + 13);
    b->pins[61].gpio.parent_id = 0;
    b->pins[61].gpio.mux_total = 0;

    strncpy(b->pins[62].name, "PC17", MRAA_PIN_NAME_SIZE);
    b->pins[62].capabilities = (mraa_pincapabilities_t){ 1, 1, 0, 0, 0, 0, 0, 0 };
    b->pins[62].gpio.pinmap = (2*32 + 17);
    b->pins[62].gpio.parent_id = 0;
    b->pins[62].gpio.mux_total = 0;

    strncpy(b->pins[63].name, "PC19", MRAA_PIN_NAME_SIZE);
    b->pins[63].capabilities = (mraa_pincapabilities_t){ 1, 1, 0, 0, 0, 0, 0, 0 };
    b->pins[63].gpio.pinmap = (2*32 + 19);
    b->pins[63].gpio.parent_id = 0;
    b->pins[63].gpio.mux_total = 0;

    strncpy(b->pins[64].name, "PD0_NPCS1_mbus1", MRAA_PIN_NAME_SIZE);
    b->pins[64].capabilities = (mraa_pincapabilities_t){ 1, 0, 0, 0, 1, 0, 0, 0 };
    b->pins[64].spi.mux_total = 0;

    strncpy(b->pins[65].name, "PC30_SPCK_mbus1", MRAA_PIN_NAME_SIZE);
    b->pins[65].capabilities = (mraa_pincapabilities_t){ 1, 0, 0, 0, 1, 0, 0, 0 };
    b->pins[65].spi.mux_total = 0;

    strncpy(b->pins[66].name, "PC29_MISO_mbus1", MRAA_PIN_NAME_SIZE);
    b->pins[66].capabilities = (mraa_pincapabilities_t){ 1, 0, 0, 0, 1, 0, 0, 0 };
    b->pins[66].spi.mux_total = 0;

    strncpy(b->pins[67].name, "PC28_MOSI_mbus1", MRAA_PIN_NAME_SIZE);
    b->pins[67].capabilities = (mraa_pincapabilities_t){ 1, 0, 0, 0, 1, 0, 0, 0 };
    b->pins[67].spi.mux_total = 0;

    strncpy(b->pins[68].name, "PB4_UTXD4", MRAA_PIN_NAME_SIZE);
    b->pins[68].capabilities = (mraa_pincapabilities_t){ 1, 0, 0, 0, 0, 0, 0, 1 };
    b->pins[68].uart.mux_total = 0;

    strncpy(b->pins[69].name, "PB3_URXD4", MRAA_PIN_NAME_SIZE);
    b->pins[69].capabilities = (mraa_pincapabilities_t){ 1, 0, 0, 0, 0, 0, 0, 1 };
    b->pins[69].uart.mux_total = 0;

    strncpy(b->pins[70].name, "PB27_UTXD0", MRAA_PIN_NAME_SIZE);
    b->pins[70].capabilities = (mraa_pincapabilities_t){ 1, 0, 0, 0, 0, 0, 0, 1 };
    b->pins[70].uart.mux_total = 0;

    strncpy(b->pins[71].name, "PB26_URXD0", MRAA_PIN_NAME_SIZE);
    b->pins[71].capabilities = (mraa_pincapabilities_t){ 1, 0, 0, 0, 0, 0, 0, 1 };
    b->pins[71].uart.mux_total = 0;

    strncpy(b->pins[72].name, "PWM1_OUT", MRAA_PIN_NAME_SIZE);
    b->pins[72].capabilities = (mraa_pincapabilities_t){ 1, 0, 1, 0, 0, 0, 0, 0 };
    b->pins[72].pwm.pinmap = 1;
    b->pins[72].pwm.mux_total = 0;

    strncpy(b->pins[73].name, "PD25_AN_mBUS1", MRAA_PIN_NAME_SIZE);
    b->pins[73].capabilities = (mraa_pincapabilities_t){ 1, 0, 0, 0, 0, 0, 1, 0 };
    b->pins[73].aio.pinmap = 6;

    strncpy(b->pins[74].name,"PB00_INT_mBUS1", MRAA_PIN_NAME_SIZE);
    b->pins[74].capabilities = (mraa_pincapabilities_t){ 1, 1, 0, 0, 0, 0, 0, 0 };
    b->pins[74].gpio.pinmap = (1*32 + 0);
    b->pins[74].gpio.parent_id = 0;
    b->pins[74].gpio.mux_total = 0;
    b->pins[74].uart.mux_total = 0;

    strncpy(b->pins[75].name,"PB2_RST_mBUS1", MRAA_PIN_NAME_SIZE);
    b->pins[75].capabilities = (mraa_pincapabilities_t){ 1, 1, 0, 0, 0, 0, 0, 0 };
    b->pins[75].gpio.pinmap = (1*32 + 2);
    b->pins[75].gpio.parent_id = 0;
    b->pins[75].gpio.mux_total = 0;
    b->pins[75].uart.mux_total = 0;

    strncpy(b->pins[76].name, "PB11_URXD3", MRAA_PIN_NAME_SIZE);
    b->pins[76].capabilities = (mraa_pincapabilities_t){ 1, 0, 0, 0, 0, 0, 0, 1 };
    b->pins[76].uart.mux_total = 0;

    strncpy(b->pins[77].name, "PB12_UTXD3", MRAA_PIN_NAME_SIZE);
    b->pins[77].capabilities = (mraa_pincapabilities_t){ 1, 0, 0, 0, 0, 0, 0, 1 };
    b->pins[77].uart.mux_total = 0;

    b->aio_non_seq = 1;
    b->aio_dev[6].pin = 73;

    b->i2c_bus_count = 3;
    b->def_i2c_bus = 2;
    b->i2c_bus[2].bus_id = 2;
    b->i2c_bus[2].sda = 32;
    b->i2c_bus[2].scl = 31;

    b->spi_bus_count = 4;
    b->def_spi_bus = 0;
    b->spi_bus[3].bus_id = 3;
    b->spi_bus[3].slave_s = 0;
    b->spi_bus[3].cs = 64;
    b->spi_bus[3].mosi = 67;
    b->spi_bus[3].miso = 66;
    b->spi_bus[3].sclk = 65;

    b->uart_dev_count = 5;
    b->def_uart_dev = 0;
   /******UART0 *******/
    b->uart_dev[0].rx = 71 ;
    b->uart_dev[0].tx = 70;
     /*****UART4****/
    b->uart_dev[4].rx = 69;
    b->uart_dev[4].tx = 68;
     /*****UART3****/
    b->uart_dev[3].rx = 76;
    b->uart_dev[3].tx = 77;

    b->gpio_count = 0;
    int i;
    for (i = 0; i < b->phy_pin_count; i++)
	if (b->pins[i].capabilities.gpio)
		b->gpio_count++;
    return b;
error:
    syslog(LOG_CRIT, "phyboard: failed to initialize");
    free(b);

    return NULL;
};
