/*
 *  Copyright (C) 2005 SAN People
 *  Copyright (C) 2008 Atmel
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/types.h>
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <linux/spi/at73c213.h>
#include <linux/gpio_keys.h>
#include <linux/input.h>
#include <linux/clk.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/fixed.h>
#include <linux/regulator/consumer.h>
#include <linux/mtd/physmap.h>
#include <linux/mtd/partitions.h>
#include <linux/platform_data/at91_adc.h>
#include <linux/spi/spi.h>
#include <linux/spi/flash.h>

#include <mach/hardware.h>
#include <asm/setup.h>
#include <asm/mach-types.h>
#include <asm/irq.h>
#include <asm/system_info.h>

#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/irq.h>

#include <mach/at91sam9_smc.h>
#include <mach/system_rev.h>
#include <mach/at91sam9260_matrix.h>
#include <mach/at91_matrix.h>

#include "at91_aic.h"
#include "board.h"
#include "sam9_smc.h"
#include "generic.h"
#include "gpio.h"

/* Thunderbolt GPIO Outputs */
#define POWER_LED		AT91_PIN_PB25
#define ACTIVITY_LED		AT91_PIN_PB22
#define SD_LED			AT91_PIN_PB30
#define SPARE_LED		AT91_PIN_PB11
#define STATUS_LED		AT91_PIN_PC8
#define IN_USE_LED		AT91_PIN_PB23

#define SD_NRST			AT91_PIN_PC10
#define USBH_PWR_EN		AT91_PIN_PC11
#define RADIO_PWR_EN		AT91_PIN_PB27
#define FLASH_NWP		AT91_PIN_PC6
#define BOOTFLASH_NWP		AT91_PIN_PA4

#define SPI0_MISO		AT91_PIN_PA0
#define SPI0_MOSI		AT91_PIN_PA1
#define SPI0_SPCK		AT91_PIN_PA2
#define SPI0_NPCS0		AT91_PIN_PA3

/* Thunderbolt GPIO Inputs */
#define USBD_5V_SENSE		AT91_PIN_PC12
#define USBH_PWR_FAULT		AT91_PIN_PC13

#define SD_NCD			AT91_PIN_PB29
#define SD_NWP			AT91_PIN_PB31
#define ETH_PHY_IRQ		AT91_PIN_PA22

static void __init thunderbolt_init_early(void)
{
	/* Initialize processor: 18.432 MHz crystal */
	//at91_initialize(18432000);
    at91_initialize(0);
}

/*
 * USB Host port
 */
static struct at91_usbh_data __initdata thunderbolt_usbh_data = {
	.ports		= 1,
	.vbus_pin	= { USBH_PWR_EN },
};

/*
 * USB Device port
 */
static struct at91_udc_data __initdata thunderbolt_udc_data = {
	.vbus_pin	= USBD_5V_SENSE,
	.pullup_pin	= 0,		/* pull-up driven by UDC */
};

/*
 * MACB Ethernet device
 */
static struct macb_platform_data __initdata thunderbolt_macb_data = {
	//.phy_irq_pin	= ETH_PHY_IRQ,
	//.is_rmii	= 1,
};

static void __init thunderbolt_add_device_macb(void)
{
	at91_add_device_eth(&thunderbolt_macb_data);
}

/*
 * NOR flash
 */
static struct mtd_partition thunderbolt_nor_partitions[] = {
	{
		.name   = "environment",
		.offset = 0,
		.size   = SZ_128K,
	},
	{
		.name	= "u-boot",
		.offset	= MTDPART_OFS_NXTBLK,
		.size	= SZ_256K,
	},
	{
		.name	= "kernel",
		.offset	= MTDPART_OFS_NXTBLK,
		.size	= SZ_2M,
	},
	{
		.name	= "root",
		.offset	= MTDPART_OFS_NXTBLK,
		.size	= 24 * SZ_1M,
	},
	{
		.name	= "data",
		.offset	= MTDPART_OFS_NXTBLK,
		.size	= MTDPART_SIZ_FULL,
	},
};

static struct physmap_flash_data thunderbolt_nor_data = {
	.width		= 2,
	.parts		= thunderbolt_nor_partitions,
	.nr_parts	= ARRAY_SIZE(thunderbolt_nor_partitions),
};

#define NOR_BASE	AT91_CHIPSELECT_0
#define NOR_SIZE	SZ_32M

static struct resource nor_flash_resources[] = {
	{
		.start	= NOR_BASE,
		.end	= NOR_BASE + NOR_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	}
};

static struct platform_device thunderbolt_nor_flash = {
	.name		= "physmap-flash",
	.id		= 0,
	.dev		= {
				.platform_data	= &thunderbolt_nor_data,
	},
	.resource	= nor_flash_resources,
	.num_resources	= ARRAY_SIZE(nor_flash_resources),
};

static struct sam9_smc_config __initdata thunderbolt_nor_smc_config = {
	.ncs_read_setup		= 0,
	.nrd_setup		= 0,
	.ncs_write_setup	= 0,
	.nwe_setup		= 0,

	.ncs_read_pulse		= 14,
	.nrd_pulse		= 13,
	.ncs_write_pulse	= 7,
	.nwe_pulse		= 7,

	.read_cycle		= 17,
	.write_cycle		= 10,

	.mode			= AT91_SMC_READMODE | AT91_SMC_WRITEMODE | AT91_SMC_EXNWMODE_DISABLE | AT91_SMC_BAT_SELECT | AT91_SMC_DBW_16,
	.tdf_cycles		= 1,
};

static __init void thunderbolt_add_device_nor(void)
{
	unsigned long csa;

	csa = at91_matrix_read(AT91_MATRIX_EBICSA);
	at91_matrix_write(AT91_MATRIX_EBICSA, csa | AT91_MATRIX_VDDIOMSEL_3_3V);

	/* configure chip-select 0 (NOR) */
	sam9_smc_configure(0, 0, &thunderbolt_nor_smc_config);

	platform_device_register(&thunderbolt_nor_flash);
}

/*
 * SPI flash
 */
static struct mtd_partition thunderbolt_spi_flash_partitions[] = {
	{
		.name   = "bootstrap",
		.offset = 0,
		.size   = 32 * SZ_1K,
	},
	{
		.name	= "unused",
		.offset	= MTDPART_OFS_NXTBLK,
		.size	= MTDPART_SIZ_FULL,
	},
};

static struct flash_platform_data thunderbolt_spi_flash_data = {
	.name		= "spi-flash",
	.parts		= thunderbolt_spi_flash_partitions,
	.nr_parts	= ARRAY_SIZE(thunderbolt_spi_flash_partitions),
};

static struct spi_board_info thunderbolt_spi_devices[] = {
	{	/* SPI flash */
		.modalias	= "m25p80",	// driver name
		.platform_data	= &thunderbolt_spi_flash_data,
		.chip_select	= 0,
		.max_speed_hz	= 10 * 1000000,
		.bus_num	= 0,
	},
};

/*
 * MCI (SD/MMC) on Slot A, 4-wire, with det_pin and wp_pin
 */
static struct mci_platform_data __initdata thunderbolt_mmc_data = {
	.slot[0] = {
		.bus_width	= 4,            /* Supports 4-wire */
		.detect_pin	= SD_NCD,		/* Card Detect pin */
    	.wp_pin		= SD_NWP,		/* Write Protect pin */
	},

};

static void __init thunderbolt_add_device_mmc(void)
{
	at91_add_device_mci(0, &thunderbolt_mmc_data);
}

/*
 * I2C devices populated on the board
 * 	(i2c hardware configuration is in at91sam9260_devices.c)
 */
static struct i2c_board_info __initdata thunderbolt_i2c_devices[] = {
	/* PCF8563 RTC w/ 7-bit address of 0x51 */
	{ I2C_BOARD_INFO("pcf8563", 0x51) },
	/* TMP275 temp sensor w/ address of 0x49 */
	{ I2C_BOARD_INFO("tmp275", 0x49) },
};

#if defined(CONFIG_W1_MASTER_GPIO)
static struct w1_gpio_platform_data w1_pdata = {
	.pin		= ONEWIRE_PIN,
	.is_open_drain	= 1,
};

static struct platform_device thunderbolt_w1_device = {
	.name			= "w1-gpio",
	.id			= -1,
	.dev.platform_data	= &w1_pdata,
};

void __init at91_add_device_w1(void)
{
	at91_set_GPIO_periph(w1_pdata.pin, 1);
	at91_set_multi_drive(w1_pdata.pin, 1);

	platform_device_register(&thunderbolt_w1_device);
}
#endif

static struct gpio_led thunderbolt_leds[] = {
	{	/* Power LED (Front, Top Left) */
		.name			= "power",
		.gpio			= POWER_LED,
		.active_low		= 1,
		.default_trigger	= "default-on",
	},
	{	/* Activity LED (Front, Bottom Left) */
		.name			= "activity",
		.gpio			= ACTIVITY_LED,
		.active_low		= 1,
		.default_trigger	= "tag-activity",
	},
	{	/* Spare LED (Front, Top Right) */
		.name			= "spare",
		.gpio			= SPARE_LED,
		.active_low		= 1,
		.default_trigger	= "none",
	},
	{	/* SD LED (Front, Bottom Right) */
		.name			= "sd",
		.gpio			= SD_LED,
		.active_low		= 1,
		.default_trigger	= "none",
	},
	{	/* IN USE LED (Rear, Top Right) */
		.name			= "in-use",
		.gpio			= IN_USE_LED,
		.active_low		= 1,
		.default_trigger	= "none",
	},
	{	/* STATUS LED (Rear, Bottom Right) */
		.name			= "status",
		.gpio			= STATUS_LED,
		.active_low		= 1,
		.default_trigger	= "default-on",
	},
};

static struct gpio_led econobox_leds[] = {
	{	/* Power LED (Rear, Top Right) */
		.name			= "power",
		.gpio			= AT91_PIN_PB23,
		.active_low		= 1,
		.default_trigger	= "default-on",
	},
	{	/* Activity LED (Rear, Bottom Right) */
		.name			= "activity",
		.gpio			= AT91_PIN_PC8,
		.active_low		= 1,
		.default_trigger	= "tag-activity",
	},
};

/*
 * GPIO Buttons
 */
static void __init thunderbolt_add_device_buttons(void) {}

/*
 * Analog-to-Digital Converter
 */

/* The LTC1407 ADC seems to always start out in the "sleep" state
 * so we need to bit-bang the wakeup sequence
 */
static void __init thunderbolt_adc_wake(void)
{
	at91_set_gpio_output(AT91_PIN_PB16, 0);		// SCK output
	at91_set_gpio_output(AT91_PIN_PB17, 0);		// CONV output
	
	at91_set_gpio_value(AT91_PIN_PB16, 1);		// SCK high
	at91_set_gpio_value(AT91_PIN_PB16, 0);		// SCK low

	at91_set_gpio_value(AT91_PIN_PB17, 1);		// CONV high
	at91_set_gpio_value(AT91_PIN_PB17, 0);		// CONV low

	at91_set_gpio_value(AT91_PIN_PB17, 1);		// CONV high
	at91_set_gpio_value(AT91_PIN_PB17, 0);		// CONV low

	at91_set_gpio_value(AT91_PIN_PB16, 1);		// SCK high
	at91_set_gpio_value(AT91_PIN_PB16, 0);		// SCK low
}

static void __init thunderbolt_board_init(void)
{
	/* Serial */
	/* DGBU on ttyS0. (Rx & Tx only) */
	at91_register_uart(0, 0, 0);
	at91_add_device_serial();

	/* USB Host */
	at91_add_device_usbh(&thunderbolt_usbh_data);
	/* USB Device */
	at91_add_device_udc(&thunderbolt_udc_data);
	/* NOR Flash */
	thunderbolt_add_device_nor();
	/* SPI */
	at91_add_device_spi(thunderbolt_spi_devices, ARRAY_SIZE(thunderbolt_spi_devices));
	/* Ethernet */
    thunderbolt_add_device_macb();
	/* MMC */
	thunderbolt_add_device_mmc();
	/* Ensure the ADC is awake before we enable the SSC */
	thunderbolt_adc_wake();
	/* SSC */
	at91_add_device_ssc(AT91SAM9260_ID_SSC, 
		ATMEL_SSC_TK | ATMEL_SSC_TF | ATMEL_SSC_RD);

	/* For Econobox reader (depopulated Thunderbolt) remap the existing
	 * LEDs and don't include any I2C devices */
	if (system_rev == 1) {
		/* LEDs */
		at91_gpio_leds(econobox_leds, ARRAY_SIZE(econobox_leds));
	} else {
		/* I2C */
		at91_add_device_i2c(thunderbolt_i2c_devices, ARRAY_SIZE(thunderbolt_i2c_devices));
		/* LEDs */
		at91_gpio_leds(thunderbolt_leds, ARRAY_SIZE(thunderbolt_leds));
	}
	/* Push Buttons */
	thunderbolt_add_device_buttons();
#if defined(CONFIG_W1_MASTER_GPIO)
	/* 1-Wire Bus for temp sensor */
	at91_add_device_w1();
#endif

}

MACHINE_START(RFCTHUNDERBOLT, "RF Code Thunderbolt")
	/* Maintainer: RF Code */
//	.phys_io	= AT91_BASE_SYS,
//	.io_pg_offst	= (AT91_VA_BASE_SYS >> 18) & 0xfffc,
	//.boot_params	= AT91_SDRAM_BASE + 0x100,
	.init_time	= at91sam926x_pit_init,
	//.timer		= &at91sam926x_timer,
	//.map_io		= thunderbolt_map_io,
	.map_io		= at91_map_io,
	//.init_irq	= thunderbolt_init_irq,
	.init_early	= thunderbolt_init_early,
	.init_irq	= at91_init_irq_default,
	.init_machine	= thunderbolt_board_init,
    .handle_irq = at91_aic_handle_irq,
MACHINE_END

