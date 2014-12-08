/*
 * rfc_ltc1407  --  Device instance for LTC1407A on RF Code Thunderbolt board.
 *
 *  Copyright (C) 2009 RF Code
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

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/timer.h>
#include <linux/interrupt.h>
#include <linux/dma-mapping.h>
#include <linux/cdev.h>
#include <linux/leds.h>
#include <linux/slab.h>
#include <linux/atmel-ssc.h>
#include <mach/hardware.h>
//#include <mach/gpio.h>
#include <asm/system_info.h>
#include <sound/soc.h>	// needed for atmel_ssc_dai.h
#include "../../sound/soc/atmel/atmel_ssc_dai.h"
#include "rfc_ltc1407.h"

/* How many channels of data are in each SSC frame */
#define NUM_CHANNELS		2
/* How many bytes are consumed per sample (includes both channels) */
#define BYTES_PER_SAMPLE	(2 * NUM_CHANNELS)
/* How many samples do we want to store in each DMA buffer
 * (this must be a multiple of 4 for current sample reduction scheme) */
#define SAMPLES_PER_BUFFER	(4 * 1024)
/* How many bytes of memory are consumed for each DMA buffer */
#define DMA_BUFFER_BYTES_EACH	(SAMPLES_PER_BUFFER * BYTES_PER_SAMPLE)
/* How many 32-bit words of memory are consumed for each DMA buffer */
#define DMA_BUFFER_WORDS_EACH	(DMA_BUFFER_BYTES_EACH / 4)
/* How many DMA buffers do we want to rotate through */
#define NUM_DMA_BUFFERS		32
/* Length of calibration buffer, in unsigned shorts */
#define CALIBRATION_LENGTH	(2<<RFC_LTC1407_BITS_PER_SAMPLE)

/* How many bytes of memory are consumed for each buffer after it's cooked */
#define COOKED_BUFFER_BYTES_EACH	(DMA_BUFFER_BYTES_EACH / 4)

/* GPIO LED Definitions (for debug purposes) */
#define POWERLED		AT91_PIN_PB25
#define ACTIVITYLED		AT91_PIN_PB22
#define USBDLED			AT91_PIN_PC8
#define USBHLED			AT91_PIN_PB23

struct rfc_adc
{
	spinlock_t		lock;
	struct device		*adcdev;
	struct cdev		chrdev;
	void			*rx_dma_mapped;
	dma_addr_t		rx_dma;
	int			next_writebuf_index;
	int			active_writebuf_index;
	wait_queue_head_t	wait;
	int			readbuf_index;
	int			read_offset;
	int			overrun;
	unsigned char	cookedbuffer[COOKED_BUFFER_BYTES_EACH];	// Cooked samples go here
	unsigned short	*calibrate[2];
	void	(*cook_func)(unsigned short *src, unsigned short *dst);
	unsigned short	cook_vals[4];	/* State variables for cook function */
};
static struct rfc_adc adc;

static dev_t adc_devno;
static int srate_fast = 1;
static int join_mode = 0;
static int cook_mode = COOKINGMODE_MAXOFAVG4;

static struct class *tagadc_class;

static struct timer_list tag_led_timer;
static struct led_trigger *tag_led_trigger;
static int tag_led_is_on = 0;

static irqreturn_t adc_interrupt(int irq, void *privdata)
{
	struct ssc_device *ssc = privdata;
	u32 ssc_sr;

	/* Read the SSC status register; keep only enabled interrupt bits */
	ssc_sr = (unsigned long)ssc_readl(ssc->regs, SR)
			& (unsigned long)ssc_readl(ssc->regs, IMR);
	
	/* Another PDC buffer was filled? */
	if (ssc_sr & SSC_BIT(SR_ENDRX))
	{
		/* Update active_writebuf_index since the PDC has
		 * started using the next one */
		adc.active_writebuf_index = adc.next_writebuf_index;
		
		/* If the "read" index is still on the now active buffer
		 * there is a possibility of overrun */
		if (adc.active_writebuf_index == adc.readbuf_index)
			adc.overrun = 1;
		
		/* Advance buffer index to determine the next one to use */
		if (++adc.next_writebuf_index >= NUM_DMA_BUFFERS)
			adc.next_writebuf_index=0;
		/* Set the PDC "next" pointer and counter register */
		ssc_writel(ssc->regs, PDC_RNPR, (unsigned int)adc.rx_dma + 
			adc.next_writebuf_index * DMA_BUFFER_BYTES_EACH);
		ssc_writel(ssc->regs, PDC_RNCR, SAMPLES_PER_BUFFER);
		
		/* Let folks know there is more data available to read */
		wake_up_interruptible(&adc.wait);
	}
	
	/*if (ssc_sr & SSC_BIT(SR_RXRDY))
	{
		u32 rxdata = ssc_readl(ssc->regs, RHR);
	}*/

	return IRQ_HANDLED;
}

/* "Classic" cook - take 4 samples, return maximum */
static void classic_cook(unsigned short *src, unsigned short *dst)
{
	unsigned short j, val1, val2;
	unsigned short max1, max2;

	/* First is always the max so far */
	max2 = *(src++) & 0x3FFF;
	max1 = *(src++) & 0x3FFF;

	for(j = 0; j < 3; j++) {
		/* little endian, ch2 is before ch1 */
		val2 = *(src++) & 0x3FFF;
		val1 = *(src++) & 0x3FFF;
		if (val2 > max2) max2 = val2;
		if (val1 > max1) max1 = val1;
	}
	dst[0] = adc.calibrate[0][max1];
	dst[1] = adc.calibrate[1][max2];
}

static void classic_cook_1ch(unsigned short *src, unsigned short *dst)
{
	unsigned short j, val;
	unsigned short max;

	/* First is always the max so far */
	max = *(src++) & 0x3FFF;
	src++;	/* Skip unused channel */

	for(j = 0; j < 3; j++) {
		val = *(src++) & 0x3FFF;
		src++;	/* Skip unused channel */
		if (val > max) max = val;
	}
	/* Report value on first channel only */
	dst[0] = adc.calibrate[0][max];
	dst[1] = 0;
}

/* "Classic" joined cook - take 4 samples, return maximum */
static void classic_joincook(unsigned short *src, unsigned short *dst)
{
	unsigned short j, val;
	unsigned short max;

	/* First is always the max so far */
	max = *(src++) & 0x3FFF;
	max += *(src++) & 0x3FFF;

	for(j = 0; j < 3; j++) {
		/* little endian, ch2 is before ch1 */
		val = *(src++) & 0x3FFF;
		val += *(src++) & 0x3FFF;
		if (val > max) max = val;
	}
	max >>= 1;

	dst[0] = dst[1] = (adc.calibrate[0][max] + adc.calibrate[1][max]) >> 1;
}

/* Smoothed cook - take 4 samples, make each average of itself with previous sample, return maximum */
static void smoothmaxof4_cook(unsigned short *src, unsigned short *dst)
{
	unsigned short j, val1, val2;
	unsigned short max1, max2;
	unsigned short *pvals = adc.cook_vals;

	/* First is always the max so far */
	val2 = *(src++) & 0x3FFF;
	val1 = *(src++) & 0x3FFF;
	max2 = pvals[1] + val2;
	max1 = pvals[0] + val1;
	pvals[0] = val1;
	pvals[1] = val2;

	for(j = 0; j < 3; j++) {
		/* little endian, ch2 is before ch1 */
		val2 = *(src++) & 0x3FFF;
		val1 = *(src++) & 0x3FFF;
		if ((pvals[1] + val2) > max2) max2 = pvals[1] + val2;
		if ((pvals[0] + val1) > max1) max1 = pvals[0] + val1;
		pvals[1] = val2;
		pvals[0] = val1;
	}
	dst[0] = adc.calibrate[0][max1/2];
	dst[1] = adc.calibrate[1][max2/2];
}

static void smoothmaxof4_cook_1ch(unsigned short *src, unsigned short *dst)
{
	unsigned short j, val;
	unsigned short max;
	unsigned short *pvals = adc.cook_vals;

	/* First is always the max so far */
	val = *(src++) & 0x3FFF;
	src++;	/* Skip unused channel */
	max = pvals[1] + val;
	pvals[1] = val;

	for(j = 0; j < 3; j++) {
		val = *(src++) & 0x3FFF;
		src++;	/* Skip unused channel */
		if ((pvals[1] + val) > max) max = pvals[1] + val;
		pvals[1] = val;
	}
	/* Report value on first channel only */
	dst[0] = adc.calibrate[0][max/2];
	dst[1] = 0;
}

/* Smoothed joincook - take 4 samples, make each average of itself with previous sample, return maximum */
static void smoothmaxof4_joincook(unsigned short *src, unsigned short *dst)
{
	unsigned short j, val1, val2, val;
	unsigned short max;
	unsigned short *pvals = adc.cook_vals;

	/* First is always the max so far */
	val2 = *(src++) & 0x3FFF;
	val1 = *(src++) & 0x3FFF;
	max = pvals[1] + val2 + pvals[0] + val1;
	pvals[0] = val1;
	pvals[1] = val2;

	for(j = 0; j < 3; j++) {
		/* little endian, ch2 is before ch1 */
		val2 = *(src++) & 0x3FFF;
		val1 = *(src++) & 0x3FFF;
		val = pvals[1] + val2 + pvals[0] + val1;
		if(val > max) max = val;
		pvals[1] = val2;
		pvals[0] = val1;
	}
	max >>= 2;
	dst[0] = dst[1] = (adc.calibrate[0][max] + adc.calibrate[1][max]) >> 1;
}

/* Flatten cook - take 4 samples, average each with neighbor with closest value, return maximum */
static void flattenmaxof4_cook(unsigned short *src, unsigned short *dst)
{
	unsigned short j, val;
	unsigned short max[2] = { 0, 0 };
	unsigned short *pvals = adc.cook_vals;
	unsigned short diff1, diff2;
	unsigned short sval;
	int idx_prev = 2, idx_prev2 = 0;

	for(j = 0; j < 8; j++) {
		val = *(src++) & 0x3FFF;
		/* Difference between sample and prev, versus prev and one before */
		if(val > pvals[idx_prev])	
			diff1 = val - pvals[idx_prev];
		else
			diff1 = pvals[idx_prev] - val;
		if(pvals[idx_prev] > pvals[idx_prev2])
			diff2 = pvals[idx_prev] - pvals[idx_prev2];
		else
			diff2 = pvals[idx_prev2] - pvals[idx_prev];
		if(diff1 > diff2) {	/* If bigger, smooth with other sample */
			sval = (pvals[idx_prev] + pvals[idx_prev2]) >> 1;
		}
		else {
			sval = (pvals[idx_prev] + val) >> 1;
		}
		pvals[idx_prev2] = val;	/* Update second previous with our value */
		if(sval > max[j&1]) max[j&1] = sval;	/* Accumulate maximum smoothed */
		
		idx_prev = (idx_prev+1) & 3;
		idx_prev2 = (idx_prev2+1) & 3;
	}
	dst[0] = adc.calibrate[0][max[1]];
	dst[1] = adc.calibrate[1][max[0]];
}

static void flattenmaxof4_cook_1ch(unsigned short *src, unsigned short *dst)
{
	unsigned short j, val;
	unsigned short max = 0;
	unsigned short *pvals = adc.cook_vals;
	unsigned short diff1, diff2;
	unsigned short sval;
	int idx_prev = 2, idx_prev2 = 0;

	/* Only process data on the single channel */
	for(j = 0; j < 8; j+=2) {
		val = *(src++) & 0x3FFF;
		/* Difference between sample and prev, versus prev and one before */
		if(val > pvals[idx_prev])	
			diff1 = val - pvals[idx_prev];
		else
			diff1 = pvals[idx_prev] - val;
		if(pvals[idx_prev] > pvals[idx_prev2])
			diff2 = pvals[idx_prev] - pvals[idx_prev2];
		else
			diff2 = pvals[idx_prev2] - pvals[idx_prev];
		if(diff1 > diff2) {	/* If bigger, smooth with other sample */
			sval = (pvals[idx_prev] + pvals[idx_prev2]) >> 1;
		}
		else {
			sval = (pvals[idx_prev] + val) >> 1;
		}
		pvals[idx_prev2] = val;	/* Update second previous with our value */
		if(sval > max) max = sval;	/* Accumulate maximum smoothed */
		
		idx_prev = (idx_prev+2) & 3;
		idx_prev2 = (idx_prev2+2) & 3;
	}
	/* Report value on first channel only */
	dst[0] = adc.calibrate[0][max];
	dst[1] = 0;
}

/* Flatten cook - take 4 samples, min each with neighbor with closest value, return maximum */
static void flatten2maxof4_cook(unsigned short *src, unsigned short *dst)
{
	unsigned short j, val;
	unsigned short max[2] = { 0, 0 };
	unsigned short *pvals = adc.cook_vals;
	unsigned short diff1, diff2;
	int idx_prev = 2, idx_prev2 = 0;

	for(j = 0; j < 8; j++) {
		unsigned short v1, v2;
		unsigned short sval;

		val = *(src++) & 0x3FFF;
		/* Difference between sample and prev, versus prev and one before */
		if(val > pvals[idx_prev]) {
			diff1 = val - pvals[idx_prev];
			v1 = pvals[idx_prev];
		}
		else {
			diff1 = pvals[idx_prev] - val;
			v1 = val;
		}
		if(pvals[idx_prev] > pvals[idx_prev2]) {
			diff2 = pvals[idx_prev] - pvals[idx_prev2];
			v2 = pvals[idx_prev2];
		}
		else {
			diff2 = pvals[idx_prev2] - pvals[idx_prev];
			v2 = pvals[idx_prev];
		}
		if(diff1 > diff2) {	/* If bigger, smooth with other sample */
			sval = v2;
		}
		else {
			sval = v1;
		}
		pvals[idx_prev2] = val;	/* Update second previous with our value */
		if(sval > max[j&1]) max[j&1] = sval;	/* Accumulate maximum smoothed */
		
		idx_prev = (idx_prev+1) & 3;
		idx_prev2 = (idx_prev2+1) & 3;
	}
	dst[0] = adc.calibrate[0][max[1]];
	dst[1] = adc.calibrate[1][max[0]];
}

static void flatten2maxof4_cook_1ch(unsigned short *src, unsigned short *dst)
{
	unsigned short j, val;
	unsigned short max = 0;
	unsigned short *pvals = adc.cook_vals;
	unsigned short diff1, diff2;
	int idx_prev = 2, idx_prev2 = 0;

	/* Only process data on the single channel */
	for(j = 0; j < 8; j+=2) {
		unsigned short v1, v2;
		unsigned short sval;

		val = *(src++) & 0x3FFF;
		/* Difference between sample and prev, versus prev and one before */
		if(val > pvals[idx_prev]) {
			diff1 = val - pvals[idx_prev];
			v1 = pvals[idx_prev];
		}
		else {
			diff1 = pvals[idx_prev] - val;
			v1 = val;
		}
		if(pvals[idx_prev] > pvals[idx_prev2]) {
			diff2 = pvals[idx_prev] - pvals[idx_prev2];
			v2 = pvals[idx_prev2];
		}
		else {
			diff2 = pvals[idx_prev2] - pvals[idx_prev];
			v2 = pvals[idx_prev];
		}
		if(diff1 > diff2) {	/* If bigger, smooth with other sample */
			sval = v2;
		}
		else {
			sval = v1;
		}
		pvals[idx_prev2] = val;	/* Update second previous with our value */
		if(sval > max) max = sval;	/* Accumulate maximum smoothed */
		
		idx_prev = (idx_prev+2) & 3;
		idx_prev2 = (idx_prev2+2) & 3;
	}
	/* Report value on first channel only */
	dst[0] = adc.calibrate[0][max];
	dst[1] = 0;
}

/* extra smooth cook - take 4 samples, average each with neighbors, return maximum */
static void xsmoothmaxof4_cook(unsigned short *src, unsigned short *dst)
{
	unsigned short j, val;
	unsigned short max[2] = { 0, 0 };
	unsigned short *pvals = adc.cook_vals;
	unsigned short sval;
	int idx_prev = 2, idx_prev2 = 0;

	for(j = 0; j < 8; j++) {
		val = *(src++) & 0x3FFF;
		/* Difference between sample and prev, versus prev and one before */
		sval = (val + (pvals[idx_prev] << 1) + pvals[idx_prev2]) >> 2;
		pvals[idx_prev2] = val;	/* Update second previous with our value */
		if(sval > max[j&1]) max[j&1] = sval;	/* Accumulate maximum smoothed */
		
		idx_prev = (idx_prev+1) & 3;
		idx_prev2 = (idx_prev2+1) & 3;
	}
	dst[0] = adc.calibrate[0][max[1]];
	dst[1] = adc.calibrate[1][max[0]];
}

static void xsmoothmaxof4_cook_1ch(unsigned short *src, unsigned short *dst)
{
	unsigned short j, val;
	unsigned short max = 0;
	unsigned short *pvals = adc.cook_vals;
	unsigned short sval;
	int idx_prev = 2, idx_prev2 = 0;

	/* Only process data on the single channel */
	for(j = 0; j < 8; j+=2) {
		val = *(src++) & 0x3FFF;
		/* Difference between sample and prev, versus prev and one before */
		sval = (val + (pvals[idx_prev] << 1) + pvals[idx_prev2]) >> 2;
		pvals[idx_prev2] = val;	/* Update second previous with our value */
		if(sval > max) max = sval;	/* Accumulate maximum smoothed */
		
		idx_prev = (idx_prev+2) & 3;
		idx_prev2 = (idx_prev2+2) & 3;
	}
	/* Report value on first channel only */
	dst[0] = adc.calibrate[0][max];
	dst[1] = 0;
}

/* Smooth using 4th order Chebyshev lowpass filter with 64k corner */
static void chebyshev_lp_64k_cook(unsigned short *src, unsigned short *dst)
{
    int j;
    static int xv[2][8], yv[2][8];  /* Scale by 256 */
    static int off = 0;
    int off1, off2, off3, off4;
    int max = 0, max2 = 0;

	for(j = 0; j < 4; j++) {
        off = (off + 1) & 0x7;
        off1 = (off+1) & 0x7;
        off2 = (off1+1) & 0x7;
        off3 = (off2+1) & 0x7;
        off4 = (off3+1) & 0x7;
		/* little endian, ch2 is before ch1 */
        xv[1][off4] = (*(src++) & 0x3FFF) * 21; /* GAIN = 12.1440777 - scale by 256 */
        xv[0][off4] = (*(src++) & 0x3FFF) * 21;
        yv[0][off4] = (xv[0][off] + xv[0][off4]) + 4 * (xv[0][off1] + xv[0][off3]) + 6 * xv[0][off2]
            + (((-59 * yv[0][off]) + (140 * yv[0][off1]) + (-404 * yv[0][off2]) + (161 * yv[0][off3])) >> 9);
        yv[1][off4] = (xv[1][off] + xv[1][off4]) + 4 * (xv[1][off1] + xv[1][off3]) + 6 * xv[1][off2]
            + (((-59 * yv[1][off]) + (140 * yv[1][off1]) + (-404 * yv[1][off2]) + (161 * yv[1][off3])) >> 9);
        if(yv[0][off4] > max)
            max = yv[0][off4];
        if(yv[1][off4] > max2)
            max2 = yv[1][off4];
	}
	dst[0] = adc.calibrate[0][(max >> 8) & 0x3FFF];
	dst[1] = adc.calibrate[1][(max2 >> 8) & 0x3FFF];
}

/* Smooth using 4th order Chebyshev lowpass filter with 64k corner */
static void chebyshev_lp_64k_cook_1ch(unsigned short *src, unsigned short *dst)
{
    int j;
    static int xv[8], yv[8];  /* Scale by 256 */
    static int off = 0;
    int off1, off2, off3, off4;
    int max = 0;

	for(j = 0; j < 4; j++) {
        off = (off + 1) & 0x7;
        off1 = (off+1) & 0x7;
        off2 = (off1+1) & 0x7;
        off3 = (off2+1) & 0x7;
        off4 = (off3+1) & 0x7;
		/* little endian, ch2 is before ch1 */
        xv[off4] = (*(src++) & 0x3FFF) * 21; /* GAIN = 12.1440777 - scale by 256 */
        src++;
        yv[off4] = (xv[off] + xv[off4]) + 4 * (xv[off1] + xv[off3]) + 6 * xv[off2]
            + (((-59 * yv[off]) + (140 * yv[off1]) + (-404 * yv[off2]) + (161 * yv[off3])) >> 9);
        if(yv[off4] > max)
            max = yv[off4];
	}
	dst[0] = adc.calibrate[1][(max >> 8) & 0x3FFF];
	dst[1] = 0;
}

/* Smooth using 3rd order Bessel lowpass filter with 96k corner */
static void bessel_lp_96k_cook(unsigned short *src, unsigned short *dst)
{
    int j;
    static int xv[2][8], yv[2][8];  /* Scale by 256 */
    static int off = 0;
    int off1, off2, off3;
    int max = 0, max2 = 0;

	for(j = 0; j < 4; j++) {
        off = (off + 1) & 0x7;
        off1 = (off+1) & 0x7;
        off2 = (off1+1) & 0x7;
        off3 = (off2+1) & 0x7;
		/* little endian, ch2 is before ch1 */
        xv[1][off3] = (*(src++) & 0x3FFF) * 130; /* GAIN = 1.964404452 - scale by 256 */
        xv[0][off3] = (*(src++) & 0x3FFF) * 130;
        yv[0][off3] = (xv[0][off] + xv[0][off3]) + 3 * (xv[0][off1] + xv[0][off2])
            + ((-120 * yv[0][off]) >> 9) + ((-557 * yv[0][off1]) >> 9) + ((-896 * yv[0][off2]) >> 9);
        yv[1][off3] = (xv[1][off] + xv[1][off3]) + 3 * (xv[1][off1] + xv[1][off2])
            + ((-120 * yv[1][off]) >> 9) + ((-557 * yv[1][off1]) >> 9) + ((-896 * yv[1][off2]) >> 9);
        if(yv[0][off3] > max)
            max = yv[0][off3];
        if(yv[1][off3] > max2)
            max2 = yv[1][off3];
	}
	dst[0] = adc.calibrate[0][(max >> 8) & 0x3FFF];
	dst[1] = adc.calibrate[1][(max2 >> 8) & 0x3FFF];
}

static void bessel_lp_96k_cook_1ch(unsigned short *src, unsigned short *dst)
{
    int j;
    static int xv[8], yv[8];  /* Scale by 256 */
    static int off = 0;
    int off1, off2, off3;
    int max = 0;

	for(j = 0; j < 4; j++) {
        off = (off + 1) & 0x7;
        off1 = (off+1) & 0x7;
        off2 = (off1+1) & 0x7;
        off3 = (off2+1) & 0x7;
		/* little endian, ch2 is before ch1 */
        xv[off3] = (*(src++) & 0x3FFF) * 130; /* GAIN = 1.964404452 - scale by 256 */
        src++;
        yv[off3] = (xv[off] + xv[off3]) + 3 * (xv[off1] + xv[off2])
            + ((-120 * yv[off]) >> 9) + ((-557 * yv[off1]) >> 9) + ((-896 * yv[off2]) >> 9);
        if(yv[off3] > max)
            max = yv[off3];
	}
	dst[0] = adc.calibrate[1][(max >> 8) & 0x3FFF];
	dst[1] = 0;
}

/* Set the cook mode function, based on cook_mode and join_mode */
static void set_cook_func(void)
{
	/* If this is a board type with only 1 channel, use single-channel
	 * functions and ignore join_mode setting */
	if(system_rev == 1) {
		if(cook_mode == COOKINGMODE_MAXOF4) {
			adc.cook_func = classic_cook_1ch;
		}
		else if(cook_mode == COOKINGMODE_MAXOFFLAT4) {
			adc.cook_func = flattenmaxof4_cook_1ch;
		}
		else if(cook_mode == COOKINGMODE_MAXOFMFLAT4) {
			adc.cook_func = flatten2maxof4_cook_1ch;
		}
		else if(cook_mode == COOKINGMODE_MAXOF2AVG4) {
			adc.cook_func = xsmoothmaxof4_cook_1ch;
		}
        else if(cook_mode == COOKINGMODE_CHEBYSHEV_LP_64K) {
            adc.cook_func = chebyshev_lp_64k_cook_1ch;
        }
        else if(cook_mode == COOKINGMODE_BESSEL_LP_96K) {
            adc.cook_func = bessel_lp_96k_cook_1ch;
        }
		else {
			adc.cook_func = smoothmaxof4_cook_1ch;
		}
	}
	else if(join_mode == 0) {
		if(cook_mode == COOKINGMODE_MAXOF4) {
			adc.cook_func = classic_cook;
		}
		else if(cook_mode == COOKINGMODE_MAXOFFLAT4) {
			adc.cook_func = flattenmaxof4_cook;
		}
		else if(cook_mode == COOKINGMODE_MAXOFMFLAT4) {
			adc.cook_func = flatten2maxof4_cook;
		}
		else if(cook_mode == COOKINGMODE_MAXOF2AVG4) {
			adc.cook_func = xsmoothmaxof4_cook;
		}
        else if(cook_mode == COOKINGMODE_CHEBYSHEV_LP_64K) {
            adc.cook_func = chebyshev_lp_64k_cook;
        }
        else if(cook_mode == COOKINGMODE_BESSEL_LP_96K) {
            adc.cook_func = bessel_lp_96k_cook;
        }
		else {
			adc.cook_func = smoothmaxof4_cook;
		}
	}
	else {
		if(cook_mode == COOKINGMODE_MAXOF4) {
			adc.cook_func = classic_joincook;
		}
		else {
			adc.cook_func = smoothmaxof4_joincook;
		}
	}
}

static void devadc_buffer_cook(unsigned char *ptr, unsigned char *cooked)
{
	int i;
	unsigned short *src = (unsigned short *)ptr;
	unsigned short *dst = (unsigned short *)cooked;
	void	(*cook_func)(unsigned short *src, unsigned short *dst) = adc.cook_func;

	/* Do 4-to-1 reduction in samples, keep max value
	 * Note: DMA_BUFFER_WORDS_EACH must be a multiple of 4 */
	for (i=0; i<DMA_BUFFER_WORDS_EACH; i+=4)
	{
		cook_func(src, dst);	
		src += 8;	/* Consumes 4 samples from each channel */
		dst += 2;	/* Generates 1 sample for reach channel */
	}
	//printk("devadc_buffer_cook: cooked %d words @ %p\n", i, ptr);
}

static int devadc_open(struct inode *inode, struct file *fd)
{
	unsigned long		flags;
	//struct rfc_adc	*adc;

	//lock_kernel();
	//adc = container_of(inode->i_cdev, struct rfc_adc, chrdev);

	if (!adc.adcdev)
	{
		return -ENODEV;
	}
	//printk("devadc_open\n");

	spin_lock_irqsave(&adc.lock, flags);
	adc.overrun = 0;
	adc.readbuf_index = -1;
	spin_unlock_irqrestore(&adc.lock, flags);
	return 0;
}

static int devadc_close(struct inode *inode, struct file *fd)
{
	//printk("devadc_close\n");
	return 0;
}

static ssize_t devadc_read(struct file *fd, char __user *buf, 
		size_t len, loff_t *f_pos)
{
	//struct rfc_adc		*dev = fd->private_data;
	unsigned long	flags;
	size_t		bytes_copied = 0;
	int		bytes_remaining, copysize;
	unsigned char	*ptr;

	/* Ensure len is a multiple of 4 (2 channels * 2 bytes) */
	len &= ~0x3;
	
	if (len == 0) return -EINVAL;

	//printk("devadc_read: trying to read %d bytes\n", (int)len);

	spin_lock_irqsave(&adc.lock, flags);

	/* If the reader didn't keep up with the data, let them know */
	if (adc.overrun)
	{
		spin_unlock_irqrestore(&adc.lock, flags);
		printk(KERN_ERR "%s: devadc_read overrun\n", __FILE__);
		adc.overrun = 0;
		adc.readbuf_index = -1;
		return -EOVERFLOW;
	}
	//at91_set_gpio_value(USBDLED, 0);	// LED on
	
	/* If the file was just opened or overrun, initialize the read index 
	 * and offset to start with the most recently filled buffer */
	if (adc.readbuf_index < 0)
	{
		/* If the active "write" index is non-zero, just
		 * subtract one; else we need to wrap around */
		adc.readbuf_index = adc.active_writebuf_index ? 
			adc.active_writebuf_index - 1 : NUM_DMA_BUFFERS - 1;
		adc.read_offset = 0;
		//printk("devadc_read: resetting to index %d\n", adc.readbuf_index);
	}

	/* They must have room for 4 bytes (2 channels x 2 bytes) */
	while (len >= 4)
	{
		/* If the active write index hasn't moved past our read index
		 * there is no more new data */
		while (adc.active_writebuf_index == adc.readbuf_index)
		{
			/* Reenable interrupts before returning or sleeping */
			spin_unlock_irqrestore(&adc.lock, flags);
			
			//printk("devadc_read: getting sleepy, bytes_copied=%d\n", bytes_copied);
			if (fd->f_flags & (O_NONBLOCK|O_NDELAY))
			{
				//printk("devadc_read: nonblock so returning, bytes_copied=%d\n", bytes_copied);
				if (bytes_copied)
					return bytes_copied;
				return -EAGAIN;
			}
			/* Sleep until data is available */
			else if (wait_event_interruptible(adc.wait,
					adc.active_writebuf_index != adc.readbuf_index))
			{
				return -ERESTARTSYS;
			}
			/* Re-acquire the lock and retest before continuing */
			spin_lock_irqsave(&adc.lock, flags);
		}
		spin_unlock_irqrestore(&adc.lock, flags);
		
		/* Point to the next byte of data that we need to transfer */
		ptr = adc.cookedbuffer + adc.read_offset;
		//printk("devadc_read: pointing to %p\n", ptr);
	
		/* If we are starting into a new buffer, we need to cook it */
		if (adc.read_offset == 0)
		{
			/* Perform sample reduction */
			devadc_buffer_cook(adc.rx_dma_mapped + 
				adc.readbuf_index * DMA_BUFFER_BYTES_EACH, adc.cookedbuffer);
		}
		/* How many cooked bytes remain in this buffer? */
		bytes_remaining = COOKED_BUFFER_BYTES_EACH - adc.read_offset;
		
		/* We will copy the smaller of bytes_remaining or len */
		if (bytes_remaining > len)
			copysize = len;
		else copysize = bytes_remaining;

		//printk("devadc_read: copysize = %d\n", copysize);
		
		/* Copy the bytes to userspace */
		if (copy_to_user(buf, ptr, copysize))
		{
			/* Not all bytes could be copied */
			printk(KERN_ERR "%s: devadc_read: copy_to_user fault!\n", __FILE__);
			return -EFAULT;
		}
		/* Adjust all the counters and pointers */
		len -= copysize;
		bytes_copied += copysize;
		ptr += copysize;
		buf += copysize;
		adc.read_offset += copysize;

		/* Re-acquire the lock and retest before continuing */
		spin_lock_irqsave(&adc.lock, flags);
		
		/* If we finished off the cooked bytes in that buffer */
		if (adc.read_offset >= COOKED_BUFFER_BYTES_EACH)
		{
			/* Advance to the next buffer and reset the offset */
			if (++adc.readbuf_index >= NUM_DMA_BUFFERS)
				adc.readbuf_index=0;
			adc.read_offset = 0;
			//printk("devadc_read: advance to buffer %d\n", adc.readbuf_index);
		}
	}

	if (adc.overrun) printk(KERN_ERR "%s: overrun during read\n", __FILE__);
	
	spin_unlock_irqrestore(&adc.lock, flags);
	//printk("devadc_read: normal exit, bytes_copied = %d\n", bytes_copied);
	//at91_set_gpio_value(USBDLED, 1);	// LED off
	if (bytes_copied)
		return bytes_copied;
	else return -EAGAIN;
}

static unsigned int devadc_poll(struct file *fd, poll_table *wait)
{
	//struct devadc_dev	*dev = fd->private_data;
	unsigned long		flags;
	int			status = 0;

	/* Add our wait queue to wake them up when status changes */
	poll_wait(fd, &adc.wait, wait);

	/* Is there any data available to read? */
	spin_lock_irqsave(&adc.lock, flags);
	if (adc.active_writebuf_index != adc.readbuf_index)
	{
		status |= POLLIN | POLLRDNORM;
	}
	spin_unlock_irqrestore(&adc.lock, flags);

	return status;
}

static long devadc_ioctl(struct file *f, unsigned int cmd, unsigned long arg)
{
	int __user *p = (int __user *)arg;
	int val;

	switch (cmd) {
	case TAG_LED_READ:
		/* TAG_LED_READ generates a short blink indicating one
		 * or more tags were read */
		if (!tag_led_is_on) {
			//printk(KERN_ERR "%s: Tag Activity LED on\n", __FILE__);
			/* Turn on the LED(s) assigned to tag-activity */
			led_trigger_event((struct led_trigger *)tag_led_timer.data,
				LED_FULL);
			/* And queue the timer so we turn it off afer a short delay */
			mod_timer(&tag_led_timer, jiffies + msecs_to_jiffies(11));
			tag_led_is_on = 1;
		}
		return 0;
	case TAG_LED_IDLE:
		/* TAG_LED_IDLE turns on the tag activity LED for 500ms and is
		 * used to indicate reader idle status (called once/second) */
		if (!tag_led_is_on) {
			/* Turn on the LED(s) assigned to tag-activity */
			led_trigger_event((struct led_trigger *)tag_led_timer.data,
				LED_FULL);
			/* And queue the timer so we turn it off afer a 500ms delay */
			mod_timer(&tag_led_timer, jiffies + msecs_to_jiffies(500));
			tag_led_is_on = 1;
		}
		return 0;
	case JOINMODE:
		if (get_user(val, p)) {
			return -EFAULT;
		}
		if ((val < 0) || (val > 1)) {
			return -EINVAL;
		}
		join_mode = val;
		set_cook_func();	/* Update cook handler */
		return 0;
	case CALIBRATE_CH_A:
		if (get_user(val, p)) {
			return -EFAULT;
		}
		if(copy_from_user(adc.calibrate[0], (void *)val, sizeof(unsigned short) * CALIBRATION_LENGTH)) {
			return -EFAULT;
		}
		return 0;
	case CALIBRATE_CH_B:
		if (get_user(val, p)) {
			return -EFAULT;
		}
		if(copy_from_user(adc.calibrate[1], (void *)val, sizeof(unsigned short) * CALIBRATION_LENGTH)) {
			return -EFAULT;
		}
		return 0;
	case READ_RAW_SAMPLES:
		if (get_user(val, p)) {
			return -EFAULT;
		}
		if (copy_to_user((void __user *)val, adc.rx_dma_mapped + 
					adc.readbuf_index * DMA_BUFFER_BYTES_EACH,  DMA_BUFFER_BYTES_EACH)) {
			return -EFAULT;
		}
		return 0;
	case COOKINGMODE:
		if (get_user(val, p)) {
			return -EFAULT;
		}
		if ((val < 0) || (val > MAX_COOKINGMODE)) {
			return -EINVAL;
		}
		cook_mode = val;
		set_cook_func();	/* Update cook handler */
		return 0;
	default:
		break;
	}
	return -EINVAL;
}

static void tag_led_off(unsigned long data)
{
	led_trigger_event((struct led_trigger *)data, LED_OFF);
	tag_led_is_on = 0;
}

static struct file_operations devadc_io_operations = {
	.owner =	THIS_MODULE,
	.open =		devadc_open,
	.read =		devadc_read,
	//.write =	devadc_write,
	//.fsync =	devadc_fsync,
	.poll =		devadc_poll,
	.unlocked_ioctl = devadc_ioctl,
	.release =	devadc_close
};

/* Sysfs interface to show our (dynamic) device major number
 */
static ssize_t show_major(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t ret;

	sprintf(buf, "%u\n", MAJOR(adc_devno));
	ret = strlen(buf) + 1;
	return ret;
}
static DEVICE_ATTR(major, 0444, show_major, NULL);

/* Sysfs interface to show/set the srate_fast value (debug)
 */
static ssize_t show_rate (struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t ret;

	sprintf(buf, "%u\n", srate_fast);
	ret = strlen(buf) + 1;
	return ret;
}

static ssize_t store_rate (struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct ssc_device *ssc;
	
	if (size) {
		ssc = (struct ssc_device *)dev_get_drvdata(adc.adcdev);
		if (buf[0] == '0')
		{
			srate_fast = 0;
			ssc_writel(ssc->regs, CMR, 20);	/* MCK / (2 x 20) = 3.2768 MHz (slow) */
		}
		else
		{
			srate_fast = 1;
			ssc_writel(ssc->regs, CMR, 5);	/* MCK / (2 x 5) = 13.1072 MHz (fast) */
		}
		return size;
	}

	return -EINVAL;
}
static DEVICE_ATTR(rate, 0644, show_rate, store_rate);

static ssize_t adc_stop (struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct ssc_device *ssc;

	//if (*after && isspace(*after))
	//	count++;

	if (size) {
		ssc = (struct ssc_device *)dev_get_drvdata(adc.adcdev);
		
		if (buf[0] == '!') {
			ssc_writel(ssc->regs, IDR, 0xFFFFFFFF);
			ssc_writel(ssc->regs, CR, SSC_BIT(CR_SWRST));
		}
		
		return size;
	}
	return -EINVAL;
}
static DEVICE_ATTR(stop, 0222, NULL, adc_stop);

/* Driver init function. Get everything initialized, registered,
 * and running.
 */
static int __init rfc_ltc1407_init(void)
{
	struct ssc_device *ssc = NULL;
	int i, j;
	int ret;

	/* Request SSC device */
	ssc = ssc_request(0);
	if (IS_ERR(ssc)) {
		return PTR_ERR(ssc);
	}

	/* Request a device number for our character device interface */
	ret = alloc_chrdev_region(&adc_devno, 0, 1, "tag adc");
	if (ret) {
		printk(KERN_ERR "%s: alloc_chrdev_region returned %d\n", __FILE__, ret);
		return -ENODEV;
	}
	
	/* Create a class of devices for the adc to belong to */
	tagadc_class = class_create(THIS_MODULE, "adc");
	if (!tagadc_class)
	{
		printk(KERN_ERR "%s: class_create failed\n", __FILE__);
		return -ENODEV;
	}
		
	/* Create the device in sysfs and the device node */
	/* Switch from device_create_drvdata to device_create (newer kernel) */
	adc.adcdev = device_create(tagadc_class, &ssc->pdev->dev,
			adc_devno, ssc, "%s", "tagadc");
	if (!adc.adcdev)
	{
		printk(KERN_ERR "%s: device_create_drvdata failed\n", __FILE__);
		return -ENODEV;
	}
	
	/* Create sysfs property file(s) */
	ret = device_create_file(adc.adcdev , &dev_attr_major);
	if (ret) printk(KERN_ERR "%s: device_create_file (major) returned %d\n", __FILE__, ret);
	ret = device_create_file(adc.adcdev , &dev_attr_rate);
	if (ret) printk(KERN_ERR "%s: device_create_file (rate) returned %d\n", __FILE__, ret);
	ret = device_create_file(adc.adcdev , &dev_attr_stop);
	if (ret) printk(KERN_ERR "%s: device_create_file (stop) returned %d\n", __FILE__, ret);

	/* Allocate a block of memory for our DMA transfers */
	adc.rx_dma_mapped = dma_alloc_coherent(&ssc->pdev->dev,
			NUM_DMA_BUFFERS * DMA_BUFFER_BYTES_EACH,
			&adc.rx_dma, GFP_KERNEL);
	if (adc.rx_dma_mapped == NULL)
	{
		printk(KERN_ERR "%s: dma_alloc_coherent failed\n", __FILE__);
		return -ENOMEM;
	}
	//else printk ("rx_dma_mapped=%p, rx_dma=%p\n", adc.rx_dma_mapped, (void *)adc.rx_dma);
	/* Allocate and default calibration tables */
	for(i = 0; i < 2; i++) {
		adc.calibrate[i] = kmalloc(sizeof(unsigned short) * CALIBRATION_LENGTH, GFP_KERNEL);
		if(adc.calibrate[i] == NULL) {
			printk(KERN_ERR "%s: kmalloc of calibrate buffer failed\n", __FILE__);
			return -ENOMEM;
		}
		for(j = 0; j < CALIBRATION_LENGTH; j++) {
			adc.calibrate[i][j] = 0x00FF - (j >> 6);
		}
	}
	set_cook_func();	/* Update cook handler */

	spin_lock_init(&adc.lock);
	init_waitqueue_head(&adc.wait);

	/* Set up the SSC registers for proper operation with our device */
	ssc_writel(ssc->regs, CR, SSC_BIT(CR_SWRST));

	ssc_writel(ssc->regs, PDC_RPR, 0);
	ssc_writel(ssc->regs, PDC_RCR, 0);
	ssc_writel(ssc->regs, PDC_RNPR, 0);
	ssc_writel(ssc->regs, PDC_RNCR, 0);

	ssc_writel(ssc->regs, PDC_TPR, 0);
	ssc_writel(ssc->regs, PDC_TCR, 0);
	ssc_writel(ssc->regs, PDC_TNPR, 0);
	ssc_writel(ssc->regs, PDC_TNCR, 0);

	/* 131.072 MHz  MCK	    (based off of 18.432MHz XTAL)
	 *  13.1072 MHz Bitclk	    (MCK/10)
	 * 262.144 kHz  Sample rate (Bitclk / 50 clocks per sample) */

	/* set SSC clock mode register */
	if (srate_fast)
		ssc_writel(ssc->regs, CMR, 5);	/* MCK / (2 x 5) = 13.1072 MHz (fast) */
	else
		ssc_writel(ssc->regs, CMR, 20);	/* MCK / (2 x 20) = 3.2768 MHz (slow) */

	/* set transmit clock mode and format
	 * We don't use the transmitter but we need it to generate TF and TK */
	ssc_writel(ssc->regs, TCMR, 
			SSC_BF(TCMR_PERIOD, 24) |		// 2*(24+1) = 50 bit period
			SSC_BF(TCMR_STTDLY, 0) |		// 0 bit start delay
			SSC_BF(TCMR_START, SSC_START_RISING_RF) | // start = rising TF
			SSC_BF(TCMR_CKO, SSC_CKO_CONTINUOUS) |	// continuous clock output
			SSC_BF(TCMR_CKS, SSC_CKS_DIV));		// use divided clock
			
	ssc_writel(ssc->regs, TFMR, 
			SSC_BF(TFMR_FSOS, SSC_FSOS_POSITIVE) |	// positive TF pulse output
			SSC_BF(TFMR_FSLEN, 0));			// TF length is 1 bit
			
	/* set receive clock mode and format */
	ssc_writel(ssc->regs, RCMR, 
			SSC_BF(RCMR_STTDLY, 0) |		// 0 bit start delay
			SSC_BF(RCMR_START, SSC_START_TX_RX) |	// start = tx start
			SSC_BF(RCMR_CKI, SSC_CKI_FALLING) |	// sample on falling edge of CLK
			SSC_BF(RCMR_CKO, SSC_CKO_NONE) |	// no RK output
			SSC_BF(RCMR_CKS, SSC_CKS_CLOCK));	// use TK for clock
			
	ssc_writel(ssc->regs, RFMR,
			SSC_BF(RFMR_FSOS, SSC_FSOS_NONE) |	// no RF pulse output
			SSC_BF(RFMR_DATNB, 0) |			// 1 channel (treat as single 32bit word)
			SSC_BIT(RFMR_MSBF) |			// MSB first
			SSC_BF(RFMR_DATLEN, 31));		// 32 bits per transfer

	/* Enable our ISR for the SSC interrupt */
	ret = request_irq(ssc->irq, adc_interrupt, 0,
			"tag_adc", ssc);
	if (ret < 0) {
		printk(KERN_WARNING "%s: request_irq failure\n", __FILE__);
	}
	
	/* Set up the PDC buffer pointer and count registers */
	ssc_writel(ssc->regs, PDC_RPR, (unsigned int)adc.rx_dma);
	ssc_writel(ssc->regs, PDC_RCR, SAMPLES_PER_BUFFER);
	
	if (++adc.next_writebuf_index >= NUM_DMA_BUFFERS) adc.next_writebuf_index=0;
	ssc_writel(ssc->regs, PDC_RNPR, (unsigned int)adc.rx_dma + 
		adc.next_writebuf_index * DMA_BUFFER_BYTES_EACH);
	ssc_writel(ssc->regs, PDC_RNCR, SAMPLES_PER_BUFFER);
	
	/* Enable the desired SSC interrupt(s) and enable desired systems */
	ssc_writel(ssc->regs, IER, SSC_BIT(IER_ENDRX));
	//ssc_writel(ssc->regs, IER, SSC_BIT(IER_RXRDY));
	ssc_writel(ssc->regs, CR, SSC_BIT(CR_TXEN) | SSC_BIT(CR_RXEN));
	ssc_writel(ssc->regs, PDC_PTCR, SSC_BIT(PDC_PTCR_RXTEN));

	/* Now that everything is prepared, register our character device interface */
	cdev_init(&adc.chrdev, &devadc_io_operations);
	adc.chrdev.owner = THIS_MODULE;
	ret = cdev_add(&adc.chrdev, adc_devno, 1);
	if (ret) {
		printk(KERN_WARNING "%s: Failed to register char device\n", __FILE__);
	}

	/* Register an LED trigger for tag activity */
	led_trigger_register_simple("tag-activity", &tag_led_trigger);
	
	/* Initialize kernel timer for tag activity LED */
	init_timer(&tag_led_timer);
	tag_led_timer.function = tag_led_off;
	tag_led_timer.data = (unsigned long)tag_led_trigger;

	return ret;
}

static void __exit rfc_ltc1407_exit(void)
{
	int i;
	if (adc.adcdev)
	{
		struct ssc_device *ssc;
		ssc = (struct ssc_device *)dev_get_drvdata(adc.adcdev);
		
		if (ssc)
		{
			ssc_writel(ssc->regs, IDR, 0xFFFFFFFF);
			ssc_writel(ssc->regs, CR, SSC_BIT(CR_SWRST));
			free_irq(ssc->irq, ssc);
			
			if (adc.rx_dma_mapped)
			{
				dma_free_coherent(&ssc->pdev->dev,
					NUM_DMA_BUFFERS * DMA_BUFFER_BYTES_EACH,
					adc.rx_dma_mapped, adc.rx_dma);
				adc.rx_dma_mapped = NULL;
			}
			ssc_free(ssc);
		}
		for(i = 0; i < 2; i++) {
			if(adc.calibrate[i]) {
				kfree(adc.calibrate[i]);
				adc.calibrate[i] = NULL;
			}
		}
		
		/* Remove our sysfs property file(s) */
		device_remove_file(adc.adcdev, &dev_attr_major);
		device_remove_file(adc.adcdev, &dev_attr_rate);
		device_remove_file(adc.adcdev, &dev_attr_stop);
		
		/* Remove Character Device */
		cdev_del(&adc.chrdev);

		/* Remove other sysfs files */
		device_destroy(tagadc_class, adc_devno);
	}
	
	class_destroy(tagadc_class);
	
	unregister_chrdev_region(adc_devno, 1);
	
	del_timer_sync(&tag_led_timer);
}

module_init(rfc_ltc1407_init);
module_exit(rfc_ltc1407_exit);

/* Module information */
MODULE_AUTHOR("Ryan Joy");
MODULE_DESCRIPTION("Thunderbolt LTC1407A Driver");
MODULE_LICENSE("GPL");
