/*
 * drivers/misc/rfc_ltc1407.h
 *
 * Include file for the RF Code LTC1407 driver.
 */

#ifndef _RFC_LTC1407_H
#define _RFC_LTC1407_H

#include <linux/ioctl.h>

#define RFC_LTC1407_BITS_PER_SAMPLE	(14)

/* ioctl()'s for the char device interface */

struct rfc_ltc1407_raw_samples {
	unsigned short samples[4096][2];	/* Channel B is 0, A is 1 */
};

/* Very short blink of the TAG activity LED to indicate tag read */
#define TAG_LED_READ		_IO( 't', 0xA1 )
/* Turn on TAG activity LED for 500ms to indicate reader idle */
#define TAG_LED_IDLE		_IO( 't', 0xA2 )
/* Select sample-add (join) mode (0=off (default), 1=on) */
#define JOINMODE		_IOW( 't', 0xA3, int )
/* Set calibration on channel A */
#define	CALIBRATE_CH_A	_IOW('t', 0xA4, unsigned short *)
/* Set calibration on channel B */
#define	CALIBRATE_CH_B	_IOW('t', 0xA5, unsigned short *)
/* Read raw sample data */
#define READ_RAW_SAMPLES	_IOW('t',0xA6, struct rfc_ltc1407_raw_samples *)
/* Select sample cooking mode (0=classic (max of 4), 1=max of 4 avgs of sample with prev) */
#define COOKINGMODE		_IOW( 't', 0xA7, int )

/* Defined cooking modes */
#define COOKINGMODE_MAXOF4		(0)
#define COOKINGMODE_MAXOFFLAT4	(1)
#define COOKINGMODE_MAXOFAVG4	(2)
#define COOKINGMODE_MAXOFMFLAT4	(3)
#define COOKINGMODE_MAXOF2AVG4	(4)
#define COOKINGMODE_CHEBYSHEV_LP_64K (5)
#define COOKINGMODE_BESSEL_LP_96K (6)
#define MAX_COOKINGMODE         (6)

/* Exported functions */
#ifdef __KERNEL__

#endif /* __KERNEL___ */

#endif /* _RFC_LTC1407_H */
