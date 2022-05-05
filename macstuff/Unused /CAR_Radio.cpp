//
//  CAR_Radio.cpp
//  rtl
//
//  Created by Vincent Moscaritolo on 4/18/22.
//

/*
 * rtl-sdr, turns your Realtek RTL2832 based DVB dongle into a SDR receiver
 * Copyright (C) 2012 by Steve Markgraf <steve@steve-m.de>
 * Copyright (C) 2012 by Hoernchen <la@tfc-server.de>
 * Copyright (C) 2012 by Kyle Keen <keenerd@gmail.com>
 * Copyright (C) 2013 by Elias Oenal <EliasOenal@gmail.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */


/*
 * written because people could not do real time
 * FM demod on Atom hardware with GNU radio
 * based on rtl_sdr.c and rtl_tcp.c
 *
 * lots of locks, but that is okay
 * (no many-to-many locks)
 *
 * todo:
 *       sanity checks
 *       scale squelch to other input parameters
 *       test all the demodulations
 *       pad output on hop
 *       frequency ranges could be stored better
 *       scaled AM demod amplification
 *       auto-hop after time limit
 *       peak detector to tune onto stronger signals
 *       fifo for active hop frequency
 *       clips
 *       noise squelch
 *       merge stereo patch
 *       merge soft agc patch
 *       merge udp patch
 *       testmode to detect overruns
 *       watchdog to reset bad dongle
 *       fix oversampling
 */


#include <errno.h>
#include <signal.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <pthread.h>
#include <unistd.h>

#include "CAR_Radio.hpp"
#include "rtl-sdr.h"

 
// MARK: -   convenience


static double atofs(char *s)
/* standard suffixes */
{
	char last;
	int len;
	double suff = 1.0;
	len = (int) strlen(s);
	last = s[len-1];
	s[len-1] = '\0';
	switch (last) {
		case 'g':
		case 'G':
			suff *= 1e3;
			/* fall-through */
		case 'm':
		case 'M':
			suff *= 1e3;
			/* fall-through */
		case 'k':
		case 'K':
			suff *= 1e3;
			suff *= atof(s);
			s[len-1] = last;
			return suff;
	}
	s[len-1] = last;
	return atof(s);
}

static int verbose_device_search(char *s)
{
	int i, device_count, device, offset;
	char *s2;
	char vendor[256], product[256], serial[256];
	device_count = rtlsdr_get_device_count();
	if (!device_count) {
		fprintf(stderr, "No supported devices found.\n");
		return -1;
	}
	fprintf(stderr, "Found %d device(s):\n", device_count);
	for (i = 0; i < device_count; i++) {
		rtlsdr_get_device_usb_strings(i, vendor, product, serial);
		fprintf(stderr, "  %d:  %s, %s, SN: %s\n", i, vendor, product, serial);
	}
	fprintf(stderr, "\n");
	/* does string look like raw id number */
	device = (int)strtol(s, &s2, 0);
	if (s2[0] == '\0' && device >= 0 && device < device_count) {
		fprintf(stderr, "Using device %d: %s\n",
			device, rtlsdr_get_device_name((uint32_t)device));
		return device;
	}
	/* does string exact match a serial */
	for (i = 0; i < device_count; i++) {
		rtlsdr_get_device_usb_strings(i, vendor, product, serial);
		if (strcmp(s, serial) != 0) {
			continue;}
		device = i;
		fprintf(stderr, "Using device %d: %s\n",
			device, rtlsdr_get_device_name((uint32_t)device));
		return device;
	}
	/* does string prefix match a serial */
	for (i = 0; i < device_count; i++) {
		rtlsdr_get_device_usb_strings(i, vendor, product, serial);
		if (strncmp(s, serial, strlen(s)) != 0) {
			continue;}
		device = i;
		fprintf(stderr, "Using device %d: %s\n",
			device, rtlsdr_get_device_name((uint32_t)device));
		return device;
	}
	/* does string suffix match a serial */
	for (i = 0; i < device_count; i++) {
		rtlsdr_get_device_usb_strings(i, vendor, product, serial);
		offset = (int) strlen(serial) - strlen(s);
		if (offset < 0) {
			continue;}
		if (strncmp(s, serial+offset, strlen(s)) != 0) {
			continue;}
		device = i;
		fprintf(stderr, "Using device %d: %s\n",
			device, rtlsdr_get_device_name((uint32_t)device));
		return device;
	}
	fprintf(stderr, "No matching devices found.\n");
	return -1;
}


int nearest_gain(rtlsdr_dev_t *dev, int target_gain)
{
	int i, r, err1, err2, count, nearest;
	int* gains;
	r = rtlsdr_set_tuner_gain_mode(dev, 1);
	if (r < 0) {
		fprintf(stderr, "WARNING: Failed to enable manual gain.\n");
		return r;
	}
	count = rtlsdr_get_tuner_gains(dev, NULL);
	if (count <= 0) {
		return 0;
	}
	gains = (int*) malloc(sizeof(int) * count);
	count = rtlsdr_get_tuner_gains(dev, gains);
	nearest = gains[0];
	for (i=0; i<count; i++) {
		err1 = abs(target_gain - nearest);
		err2 = abs(target_gain - gains[i]);
		if (err2 < err1) {
			nearest = gains[i];
		}
	}
	free(gains);
	return nearest;
}

int verbose_set_frequency(rtlsdr_dev_t *dev, uint32_t frequency)
{
	int r;
	r = rtlsdr_set_center_freq(dev, frequency);
	if (r < 0) {
		fprintf(stderr, "WARNING: Failed to set center freq.\n");
	} else {
//		fprintf(stderr, "Tuned to %u Hz.\n", frequency);
	}
	return r;
}

int verbose_set_sample_rate(rtlsdr_dev_t *dev, uint32_t samp_rate)
{
	int r;
	r = rtlsdr_set_sample_rate(dev, samp_rate);
	if (r < 0) {
		fprintf(stderr, "WARNING: Failed to set sample rate.\n");
	} else {
		fprintf(stderr, "Sampling at %u S/s.\n", samp_rate);
	}
	return r;
}

int verbose_direct_sampling(rtlsdr_dev_t *dev, int on)
{
	int r;
	r = rtlsdr_set_direct_sampling(dev, on);
	if (r != 0) {
		fprintf(stderr, "WARNING: Failed to set direct sampling mode.\n");
		return r;
	}
	if (on == 0) {
		fprintf(stderr, "Direct sampling mode disabled.\n");}
	if (on == 1) {
		fprintf(stderr, "Enabled direct sampling mode, input 1/I.\n");}
	if (on == 2) {
		fprintf(stderr, "Enabled direct sampling mode, input 2/Q.\n");}
	return r;
}

int verbose_offset_tuning(rtlsdr_dev_t *dev)
{
	int r;
	r = rtlsdr_set_offset_tuning(dev, 1);
	if (r != 0) {
		fprintf(stderr, "WARNING: Failed to set offset tuning.\n");
	} else {
		fprintf(stderr, "Offset tuning mode enabled.\n");
	}
	return r;
}

static int verbose_auto_gain(rtlsdr_dev_t *dev)
{
	int r;
	r = rtlsdr_set_tuner_gain_mode(dev, 0);
	if (r != 0) {
		fprintf(stderr, "WARNING: Failed to set tuner gain.\n");
	} else {
		fprintf(stderr, "Tuner gain set to automatic.\n");
	}
	return r;
}

static int verbose_gain_set(rtlsdr_dev_t *dev, int gain)
{
	int r;
	r = rtlsdr_set_tuner_gain_mode(dev, 1);
	if (r < 0) {
		fprintf(stderr, "WARNING: Failed to enable manual gain.\n");
		return r;
	}
	r = rtlsdr_set_tuner_gain(dev, gain);
	if (r != 0) {
		fprintf(stderr, "WARNING: Failed to set tuner gain.\n");
	} else {
		fprintf(stderr, "Tuner gain set to %0.2f dB.\n", gain/10.0);
	}
	return r;
}

static int verbose_ppm_set(rtlsdr_dev_t *dev, int ppm_error)
{
	int r;
	if (ppm_error == 0) {
		return 0;}
	r = rtlsdr_set_freq_correction(dev, ppm_error);
	if (r < 0) {
		fprintf(stderr, "WARNING: Failed to set ppm error.\n");
	} else {
		fprintf(stderr, "Tuner error set to %i ppm.\n", ppm_error);
	}
	return r;
}

static int verbose_reset_buffer(rtlsdr_dev_t *dev)
{
	int r;
	r = rtlsdr_reset_buffer(dev);
	if (r < 0) {
		fprintf(stderr, "WARNING: Failed to reset buffers.\n");}
	return r;
}


// MARK: -   rtl-fm stuff
 
static volatile int do_exit = 0;
static int lcm_post[17] = {1,1,1,3,1,5,3,7,1,9,5,11,3,13,7,15,1};
static int ACTUAL_BUF_LENGTH;

static int *atan_lut = NULL;
static int atan_lut_size = 131072; /* 512 KB */
static int atan_lut_coef = 8;

/* more cond dumbness */
#define safe_cond_signal(n, m) pthread_mutex_lock(m); pthread_cond_signal(n); pthread_mutex_unlock(m)
#define safe_cond_wait(n, m) pthread_mutex_lock(m); pthread_cond_wait(n, m); pthread_mutex_unlock(m)

/* {length, coef, coef, coef}  and scaled by 2^15
	for now, only length 9, optimal way to get +85% bandwidth */
#define CIC_TABLE_MAX 10
int cic_9_tables[][10] = {
	{0,},
	{9, -156,  -97, 2798, -15489, 61019, -15489, 2798,  -97, -156},
	{9, -128, -568, 5593, -24125, 74126, -24125, 5593, -568, -128},
	{9, -129, -639, 6187, -26281, 77511, -26281, 6187, -639, -129},
	{9, -122, -612, 6082, -26353, 77818, -26353, 6082, -612, -122},
	{9, -120, -602, 6015, -26269, 77757, -26269, 6015, -602, -120},
	{9, -120, -582, 5951, -26128, 77542, -26128, 5951, -582, -120},
	{9, -119, -580, 5931, -26094, 77505, -26094, 5931, -580, -119},
	{9, -119, -578, 5921, -26077, 77484, -26077, 5921, -578, -119},
	{9, -119, -577, 5917, -26067, 77473, -26067, 5917, -577, -119},
	{9, -199, -362, 5303, -25505, 77489, -25505, 5303, -362, -199},
};

#if defined(_MSC_VER) && (_MSC_VER < 1800)
static double log2(double n)
{
	return log(n) / log(2.0);
}
#endif

static void rotate_90(unsigned char *buf, uint32_t len)
/* 90 rotation is 1+0j, 0+1j, -1+0j, 0-1j
	or [0, 1, -3, 2, -4, -5, 7, -6] */
{
	uint32_t i;
	unsigned char tmp;
	for (i=0; i<len; i+=8) {
		/* uint8_t negation = 255 - x */
		tmp = 255 - buf[i+3];
		buf[i+3] = buf[i+2];
		buf[i+2] = tmp;

		buf[i+4] = 255 - buf[i+4];
		buf[i+5] = 255 - buf[i+5];

		tmp = 255 - buf[i+6];
		buf[i+6] = buf[i+7];
		buf[i+7] = tmp;
	}
}

static void low_pass(demod_state_t *d)
/* simple square window FIR */
{
	int i=0, i2=0;
	while (i < d->lp_len) {
		d->now_r += d->lowpassed[i];
		d->now_j += d->lowpassed[i+1];
		i += 2;
		d->prev_index++;
		if (d->prev_index < d->downsample) {
			continue;
		}
		d->lowpassed[i2]   = d->now_r; // * d->output_scale;
		d->lowpassed[i2+1] = d->now_j; // * d->output_scale;
		d->prev_index = 0;
		d->now_r = 0;
		d->now_j = 0;
		i2 += 2;
	}
	d->lp_len = i2;
}

static int low_pass_simple(int16_t *signal2, int len, int step)
// no wrap around, length must be multiple of step
{
	int i, i2, sum;
	for(i=0; i < len; i+=step) {
		sum = 0;
		for(i2=0; i2<step; i2++) {
			sum += (int)signal2[i + i2];
		}
		//signal2[i/step] = (int16_t)(sum / step);
		signal2[i/step] = (int16_t)(sum);
	}
	signal2[i/step + 1] = signal2[i/step];
	return len / step;
}

static void low_pass_real(demod_state_t *s)
/* simple square window FIR */
// add support for upsampling?
{
	int i=0, i2=0;
	int fast = (int)s->rate_out;
	int slow = s->rate_out2;
	while (i < s->result_len) {
		s->now_lpr += s->result[i];
		i++;
		s->prev_lpr_index += slow;
		if (s->prev_lpr_index < fast) {
			continue;
		}
		s->result[i2] = (int16_t)(s->now_lpr / (fast/slow));
		s->prev_lpr_index -= fast;
		s->now_lpr = 0;
		i2 += 1;
	}
	s->result_len = i2;
}

static void fifth_order(int16_t *data, int length, int16_t *hist)
/* for half of interleaved data */
{
	int i;
	int16_t a, b, c, d, e, f;
	a = hist[1];
	b = hist[2];
	c = hist[3];
	d = hist[4];
	e = hist[5];
	f = data[0];
	/* a downsample should improve resolution, so don't fully shift */
	data[0] = (a + (b+e)*5 + (c+d)*10 + f) >> 4;
	for (i=4; i<length; i+=4) {
		a = c;
		b = d;
		c = e;
		d = f;
		e = data[i-2];
		f = data[i];
		data[i/2] = (a + (b+e)*5 + (c+d)*10 + f) >> 4;
	}
	/* archive */
	hist[0] = a;
	hist[1] = b;
	hist[2] = c;
	hist[3] = d;
	hist[4] = e;
	hist[5] = f;
}

static void generic_fir(int16_t *data, int length, int *fir, int16_t *hist)
/* Okay, not at all generic.  Assumes length 9, fix that eventually. */
{
	int d, temp, sum;
	for (d=0; d<length; d+=2) {
		temp = data[d];
		sum = 0;
		sum += (hist[0] + hist[8]) * fir[1];
		sum += (hist[1] + hist[7]) * fir[2];
		sum += (hist[2] + hist[6]) * fir[3];
		sum += (hist[3] + hist[5]) * fir[4];
		sum +=            hist[4]  * fir[5];
		data[d] = sum >> 15 ;
		hist[0] = hist[1];
		hist[1] = hist[2];
		hist[2] = hist[3];
		hist[3] = hist[4];
		hist[4] = hist[5];
		hist[5] = hist[6];
		hist[6] = hist[7];
		hist[7] = hist[8];
		hist[8] = temp;
	}
}
/* define our own complex math ops
	because ARMv5 has no hardware float */

static void multiply(int ar, int aj, int br, int bj, int *cr, int *cj)
{
	*cr = ar*br - aj*bj;
	*cj = aj*br + ar*bj;
}

static int polar_discriminant(int ar, int aj, int br, int bj)
{
	int cr, cj;
	double angle;
	multiply(ar, aj, br, -bj, &cr, &cj);
	angle = atan2((double)cj, (double)cr);
	return (int)(angle / 3.14159 * (1<<14));
}

static int fast_atan2(int y, int x)
/* pre scaled for int16 */
{
	int yabs, angle;
	int pi4=(1<<12), pi34=3*(1<<12);  // note pi = 1<<14
	if (x==0 && y==0) {
		return 0;
	}
	yabs = y;
	if (yabs < 0) {
		yabs = -yabs;
	}
	if (x >= 0) {
		angle = pi4  - pi4 * (x-yabs) / (x+yabs);
	} else {
		angle = pi34 - pi4 * (x+yabs) / (yabs-x);
	}
	if (y < 0) {
		return -angle;
	}
	return angle;
}

static int polar_disc_fast(int ar, int aj, int br, int bj)
{
	int cr, cj;
	multiply(ar, aj, br, -bj, &cr, &cj);
	return fast_atan2(cj, cr);
}

static int atan_lut_init(void)
{
	int i = 0;

	atan_lut = (int*) malloc(atan_lut_size * sizeof(int));

	for (i = 0; i < atan_lut_size; i++) {
		atan_lut[i] = (int) (atan((double) i / (1<<atan_lut_coef)) / 3.14159 * (1<<14));
	}

	return 0;
}

int polar_disc_lut(int ar, int aj, int br, int bj)
{
	int cr, cj, x, x_abs;

	multiply(ar, aj, br, -bj, &cr, &cj);

	/* special cases */
	if (cr == 0 || cj == 0) {
		if (cr == 0 && cj == 0)
			{return 0;}
		if (cr == 0 && cj > 0)
			{return 1 << 13;}
		if (cr == 0 && cj < 0)
			{return -(1 << 13);}
		if (cj == 0 && cr > 0)
			{return 0;}
		if (cj == 0 && cr < 0)
			{return 1 << 14;}
	}

	/* real range -32768 - 32768 use 64x range -> absolute maximum: 2097152 */
	x = (cj << atan_lut_coef) / cr;
	x_abs = abs(x);

	if (x_abs >= atan_lut_size) {
		/* we can use linear range, but it is not necessary */
		return (cj > 0) ? 1<<13 : -(1<<13);
	}

	if (x > 0) {
		return (cj > 0) ? atan_lut[x] : atan_lut[x] - (1<<14);
	} else {
		return (cj > 0) ? (1<<14) - atan_lut[-x] : -atan_lut[-x];
	}

	return 0;
}

static void fm_demod(demod_state_t *fm)
{
	int i, pcm;
	int16_t *lp = fm->lowpassed;
	pcm = polar_discriminant(lp[0], lp[1],
		fm->pre_r, fm->pre_j);
	fm->result[0] = (int16_t)pcm;
	for (i = 2; i < (fm->lp_len-1); i += 2) {
		switch (fm->custom_atan) {
		case 0:
			pcm = polar_discriminant(lp[i], lp[i+1],
				lp[i-2], lp[i-1]);
			break;
		case 1:
			pcm = polar_disc_fast(lp[i], lp[i+1],
				lp[i-2], lp[i-1]);
			break;
		case 2:
			pcm = polar_disc_lut(lp[i], lp[i+1],
				lp[i-2], lp[i-1]);
			break;
		}
		fm->result[i/2] = (int16_t)pcm;
	}
	fm->pre_r = lp[fm->lp_len - 2];
	fm->pre_j = lp[fm->lp_len - 1];
	fm->result_len = fm->lp_len/2;
}

static void am_demod(demod_state_t *fm)
// todo, fix this extreme laziness
{
	int i, pcm;
	int16_t *lp = fm->lowpassed;
	int16_t *r  = fm->result;
	for (i = 0; i < fm->lp_len; i += 2) {
		// hypot uses floats but won't overflow
		//r[i/2] = (int16_t)hypot(lp[i], lp[i+1]);
		pcm = lp[i] * lp[i];
		pcm += lp[i+1] * lp[i+1];
		r[i/2] = (int16_t)sqrt(pcm) * fm->output_scale;
	}
	fm->result_len = fm->lp_len/2;
	// lowpass? (3khz)  highpass?  (dc)
}

static void usb_demod(demod_state_t *fm)
{
	int i, pcm;
	int16_t *lp = fm->lowpassed;
	int16_t *r  = fm->result;
	for (i = 0; i < fm->lp_len; i += 2) {
		pcm = lp[i] + lp[i+1];
		r[i/2] = (int16_t)pcm * fm->output_scale;
	}
	fm->result_len = fm->lp_len/2;
}

static void lsb_demod(demod_state_t *fm)
{
	int i, pcm;
	int16_t *lp = fm->lowpassed;
	int16_t *r  = fm->result;
	for (i = 0; i < fm->lp_len; i += 2) {
		pcm = lp[i] - lp[i+1];
		r[i/2] = (int16_t)pcm * fm->output_scale;
	}
	fm->result_len = fm->lp_len/2;
}

static void raw_demod(demod_state_t *fm)
{
	int i;
	for (i = 0; i < fm->lp_len; i++) {
		fm->result[i] = (int16_t)fm->lowpassed[i];
	}
	fm->result_len = fm->lp_len;
}
// MARK: -   Threads

static void optimal_settings(sdr_state_t *s, int freq, int rate)
{
	// giant ball of hacks
	// seems unable to do a single pass, 2:1
	int capture_freq, capture_rate;
	struct dongle_state *d = &s->dongle;
	struct demod_state *dm = &s->demod;
	struct controller_state *cs = &s->controller;
	dm->downsample = (1000000 / dm->rate_in) + 1;
	if (dm->downsample_passes) {
		dm->downsample_passes = (int)log2(dm->downsample) + 1;
		dm->downsample = 1 << dm->downsample_passes;
	}
	capture_freq = freq;
	capture_rate = dm->downsample * dm->rate_in;
	if (!d->offset_tuning) {
		capture_freq = freq + capture_rate/4;}
	capture_freq += cs->edge * dm->rate_in / 2;
	dm->output_scale = (1<<15) / (128 * dm->downsample);
	if (dm->output_scale < 1) {
		dm->output_scale = 1;}
	if (dm->mode_demod == &fm_demod) {
		dm->output_scale = 1;}
	d->freq = (uint32_t)capture_freq;
	d->rate = (uint32_t)capture_rate;
}

static void *controller_thread_fn(void *arg)
{
	// thoughts for multiple dongles
	// might be no good using a controller thread if retune/rate blocks
	int i;
	sdr_state_t *s = (sdr_state_t*)  arg;

	if (s->controller.wb_mode) {
		for (i=0; i < s->controller.freq_len; i++) {
			s->controller.freqs[i] += 16000;}
	}

//	 set up primary channel
	optimal_settings(s, s->controller.freqs[0], s->demod.rate_in);
	if (s->dongle.direct_sampling) {
		verbose_direct_sampling(s->dongle.dev, s->dongle.direct_sampling);}
	if (s->dongle.offset_tuning) {
		verbose_offset_tuning(s->dongle.dev);}

//Set the frequency
	verbose_set_frequency(s->dongle.dev, s->dongle.freq);
	fprintf(stderr, "Oversampling input by: %ix.\n", s->demod.downsample);
	fprintf(stderr, "Oversampling output by: %ix.\n", s->demod.post_downsample);
	fprintf(stderr, "Buffer size: %0.2fms\n",
		1000 * 0.5 * (float)ACTUAL_BUF_LENGTH / (float)s->dongle.rate);

//Set the sample rate
	verbose_set_sample_rate(s->dongle.dev, s->dongle.rate);
	fprintf(stderr, "Output at %u Hz.\n", s->demod.rate_in/s->demod.post_downsample);

	while (!do_exit) {
		safe_cond_wait(&s->controller.hop, &s->controller.hop_m);
		if (s->controller.freq_len <= 1) {
			continue;}
		// hacky hopping
		s->controller.freq_now = (s->controller.freq_now + 1) % s->controller.freq_len;
		optimal_settings(s, s->controller.freqs[s->controller.freq_now], s->demod.rate_in);
		rtlsdr_set_center_freq(s->dongle.dev, s->dongle.freq);
		s->dongle.mute = BUFFER_DUMP;
	}
	return 0;
}
 
static void *output_thread_fn(void *arg)
{
	sdr_state_t *s = (sdr_state_t*)  arg;

	while (!do_exit) {
		// use timedwait and pad out under runs
		safe_cond_wait(&s->output.ready, &s->output.ready_m);
		pthread_rwlock_rdlock(&s->output.rw);
		fwrite(s->output.result, 2, s->output.result_len, s->output.file);
		pthread_rwlock_unlock(&s->output.rw);
	}
	return 0;
}


static void rtlsdr_callback(unsigned char *buf, uint32_t len, void *ctx)
{
	int i;
	dongle_state_t *s = (dongle_state_t*) ctx;
	struct demod_state *d = s->demod_target;

	if (do_exit) {
		return;}
	if (!ctx) {
		return;}
	if (s->mute) {
		for (i=0; i<s->mute; i++) {
			buf[i] = 127;}
		s->mute = 0;
	}
	if (!s->offset_tuning) {
		rotate_90(buf, len);}
	for (i=0; i<(int)len; i++) {
		s->buf16[i] = (int16_t)buf[i] - 127;}
	pthread_rwlock_wrlock(&d->rw);
	memcpy(d->lowpassed, s->buf16, 2*len);
	d->lp_len = len;
	pthread_rwlock_unlock(&d->rw);
	safe_cond_signal(&d->ready, &d->ready_m);
}


void deemph_filter(struct demod_state *fm)
{
	static int avg;  // cheating...
	int i, d;
	// de-emph IIR
	// avg = avg * (1 - alpha) + sample * alpha;
	for (i = 0; i < fm->result_len; i++) {
		d = fm->result[i] - avg;
		if (d > 0) {
			avg += (d + fm->deemph_a/2) / fm->deemph_a;
		} else {
			avg += (d - fm->deemph_a/2) / fm->deemph_a;
		}
		fm->result[i] = (int16_t)avg;
	}
}

void dc_block_filter(struct demod_state *fm)
{
	int i, avg;
	int64_t sum = 0;
	for (i=0; i < fm->result_len; i++) {
		sum += fm->result[i];
	}
	avg = sum / fm->result_len;
	avg = (avg + fm->dc_avg * 9) / 10;
	for (i=0; i < fm->result_len; i++) {
		fm->result[i] -= avg;
	}
	fm->dc_avg = avg;
}

int mad(int16_t *samples, int len, int step)
/* mean average deviation */
{
	int i=0, sum=0, ave=0;
	if (len == 0)
		{return 0;}
	for (i=0; i<len; i+=step) {
		sum += samples[i];
	}
	ave = sum / (len * step);
	sum = 0;
	for (i=0; i<len; i+=step) {
		sum += abs(samples[i] - ave);
	}
	return sum / (len / step);
}

int rms(int16_t *samples, int len, int step)
/* largely lifted from rtl_power */
{
	int i;
	long p, t, s;
	double dc, err;

	p = t = 0L;
	for (i=0; i<len; i+=step) {
		s = (long)samples[i];
		t += s;
		p += s * s;
	}
	/* correct for dc offset in squares */
	dc = (double)(t*step) / (double)len;
	err = t * 2 * dc - dc * dc * len;

	return (int)sqrt((p-err) / len);
}

void arbitrary_upsample(int16_t *buf1, int16_t *buf2, int len1, int len2)
/* linear interpolation, len1 < len2 */
{
	int i = 1;
	int j = 0;
	int tick = 0;
	double frac;  // use integers...
	while (j < len2) {
		frac = (double)tick / (double)len2;
		buf2[j] = (int16_t)(buf1[i-1]*(1-frac) + buf1[i]*frac);
		j++;
		tick += len1;
		if (tick > len2) {
			tick -= len2;
			i++;
		}
		if (i >= len1) {
			i = len1 - 1;
			tick = len2;
		}
	}
}

void arbitrary_downsample(int16_t *buf1, int16_t *buf2, int len1, int len2)
/* fractional boxcar lowpass, len1 > len2 */
{
	int i = 1;
	int j = 0;
	int tick = 0;
	double remainder = 0;
	double frac;  // use integers...
	buf2[0] = 0;
	while (j < len2) {
		frac = 1.0;
		if ((tick + len2) > len1) {
			frac = (double)(len1 - tick) / (double)len2;}
		buf2[j] += (int16_t)((double)buf1[i] * frac + remainder);
		remainder = (double)buf1[i] * (1.0-frac);
		tick += len2;
		i++;
		if (tick > len1) {
			j++;
			buf2[j] = 0;
			tick -= len1;
		}
		if (i >= len1) {
			i = len1 - 1;
			tick = len1;
		}
	}
	for (j=0; j<len2; j++) {
		buf2[j] = buf2[j] * len2 / len1;}
}

void arbitrary_resample(int16_t *buf1, int16_t *buf2, int len1, int len2)
/* up to you to calculate lengths and make sure it does not go OOB
 * okay for buffers to overlap, if you are downsampling */
{
	if (len1 < len2) {
		arbitrary_upsample(buf1, buf2, len1, len2);
	} else {
		arbitrary_downsample(buf1, buf2, len1, len2);
	}
}
void full_demod(demod_state_t *d)
{
	int i, ds_p;
	int sr = 0;
	ds_p = d->downsample_passes;
	if (ds_p) {
		for (i=0; i < ds_p; i++) {
			fifth_order(d->lowpassed,   (d->lp_len >> i), d->lp_i_hist[i]);
			fifth_order(d->lowpassed+1, (d->lp_len >> i) - 1, d->lp_q_hist[i]);
		}
		d->lp_len = d->lp_len >> ds_p;
		/* droop compensation */
		if (d->comp_fir_size == 9 && ds_p <= CIC_TABLE_MAX) {
			generic_fir(d->lowpassed, d->lp_len,
				cic_9_tables[ds_p], d->droop_i_hist);
			generic_fir(d->lowpassed+1, d->lp_len-1,
				cic_9_tables[ds_p], d->droop_q_hist);
		}
	} else {
		low_pass(d);
	}
	/* power squelch */
	if (d->squelch_level) {
		sr = rms(d->lowpassed, d->lp_len, 1);
		if (sr < d->squelch_level) {
			d->squelch_hits++;
			for (i=0; i<d->lp_len; i++) {
				d->lowpassed[i] = 0;
			}
		} else {
			d->squelch_hits = 0;}
	}
	d->mode_demod(d);  /* lowpassed -> result */
	if (d->mode_demod == &raw_demod) {
		return;
	}
	/* todo, fm noise squelch */
	// use nicer filter here too?
	if (d->post_downsample > 1) {
		d->result_len = low_pass_simple(d->result, d->result_len, d->post_downsample);}
	if (d->deemph) {
		deemph_filter(d);}
	if (d->dc_block) {
		dc_block_filter(d);}
	if (d->rate_out2 > 0) {
		low_pass_real(d);
		//arbitrary_resample(d->result, d->result, d->result_len, d->result_len * d->rate_out2 / d->rate_out);
	}
}
static void *dongle_thread_fn(void *arg)
{
	sdr_state_t *s = (sdr_state_t*)  arg;

	rtlsdr_read_async(s->dongle.dev, rtlsdr_callback, s, 0, s->dongle.buf_len);
	return 0;
}



static void *demod_thread_fn(void *arg)
{
	sdr_state_t *s = (sdr_state_t*)  arg;

	demod_state_t *d = &s->demod;
	struct output_state *o = d->output_target;
	while (!do_exit) {
		safe_cond_wait(&d->ready, &d->ready_m);
		pthread_rwlock_wrlock(&d->rw);
		full_demod(d);
		pthread_rwlock_unlock(&d->rw);
		if (d->exit_flag) {
			do_exit = 1;
		}
		if (d->squelch_level && d->squelch_hits > d->conseq_squelch) {
			d->squelch_hits = d->conseq_squelch + 1;  /* hair trigger */
			safe_cond_signal(&s->controller.hop, &s->controller.hop_m);
			continue;
		}
		pthread_rwlock_wrlock(&o->rw);
		memcpy(o->result, d->result, 2*d->result_len);
		o->result_len = d->result_len;
		pthread_rwlock_unlock(&o->rw);
		safe_cond_signal(&o->ready, &o->ready_m);
	}
	return 0;
}

// MARK: -   CAR_Radio setup


void CAR_Radio::dongle_init(dongle_state_t *s)
{
	s->rate = DEFAULT_SAMPLE_RATE;
	s->gain = AUTO_GAIN; // tenths of a dB
	s->mute = 0;
	s->direct_sampling = 0;
	s->offset_tuning = 0;
	s->demod_target = &_state.demod;
}


void CAR_Radio::demod_init(demod_state_t *s)
{
	s->rate_in = DEFAULT_SAMPLE_RATE;
	s->rate_out = DEFAULT_SAMPLE_RATE;
	s->squelch_level = 0;
	s->conseq_squelch = 10;
	s->terminate_on_squelch = 0;
	s->squelch_hits = 11;
	s->downsample_passes = 0;
	s->comp_fir_size = 0;
	s->prev_index = 0;
	s->post_downsample = 1;  // once this works, default = 4
	s->custom_atan = 0;
	s->deemph = 0;
	s->rate_out2 = -1;  // flag for disabled
	s->mode_demod = &fm_demod;
	s->pre_j = s->pre_r = s->now_r = s->now_j = 0;
	s->prev_lpr_index = 0;
	s->deemph_a = 0;
	s->now_lpr = 0;
	s->dc_block = 0;
	s->dc_avg = 0;
	pthread_rwlock_init(&s->rw, NULL);
	pthread_cond_init(&s->ready, NULL);
	pthread_mutex_init(&s->ready_m, NULL);
	s->output_target = &_state.output;
}

void CAR_Radio::demod_cleanup(demod_state_t *s)
{
	pthread_rwlock_destroy(&s->rw);
	pthread_cond_destroy(&s->ready);
	pthread_mutex_destroy(&s->ready_m);
}

void CAR_Radio::output_init(output_state_t *s)
{
	s->rate = DEFAULT_SAMPLE_RATE;
	pthread_rwlock_init(&s->rw, NULL);
	pthread_cond_init(&s->ready, NULL);
	pthread_mutex_init(&s->ready_m, NULL);
}

void CAR_Radio::output_cleanup(output_state_t *s)
{
	pthread_rwlock_destroy(&s->rw);
	pthread_cond_destroy(&s->ready);
	pthread_mutex_destroy(&s->ready_m);
}

void CAR_Radio::controller_init(controller_state_t *s)
{
	s->freqs[0] = 100000000;
	s->freq_len = 0;
	s->edge = 0;
	s->wb_mode = 0;
	pthread_cond_init(&s->hop, NULL);
	pthread_mutex_init(&s->hop_m, NULL);
}

void CAR_Radio::controller_cleanup(controller_state_t *s)
{
	pthread_cond_destroy(&s->hop);
	pthread_mutex_destroy(&s->hop_m);
}


// MARK: -   CAR_Radio

CAR_Radio::CAR_Radio(){
_isSetup = false;
	
	dongle_init(&_state.dongle);
	demod_init(&_state.demod);
 	output_init(&_state.output);
 	controller_init(&_state.controller);
  }


CAR_Radio::~CAR_Radio(){
	stop();
	
	//dongle_cleanup(&dongle);
	demod_cleanup(&_state.demod);
	output_cleanup(&_state.output);
	controller_cleanup(&_state.controller);
	
	if (_state.output.file != stdout) {
		fclose(_state.output.file);}
	
	if(_isSetup){
		rtlsdr_close(_state.dongle.dev);
	}

}


bool CAR_Radio::begin(){
		
	int enable_biastee = 0;

	/* quadruple sample_rate to limit to Δθ to ±π/2 */
	_state.demod.rate_in *= _state.demod.post_downsample;
	
	if (!_state.output.rate) {
		_state.output.rate = _state.demod.rate_out;}
	
	if (_state.controller.freq_len == 0)
		return false;
	
	if (_state.controller.freq_len >= FREQUENCIES_LIMIT)
		return false;
	
	if (_state.controller.freq_len > 1 && _state.demod.squelch_level == 0)
		return false;

	// find dongle
	char str[] = "0";
	_state.dongle.dev_index = verbose_device_search(str);

	if (_state.dongle.dev_index < 0)
		return false;

	ACTUAL_BUF_LENGTH = lcm_post[_state.demod.post_downsample] * DEFAULT_BUF_LENGTH;

	if( rtlsdr_open(&_state.dongle.dev, (uint32_t)_state.dongle.dev_index) < 0)
		return false;

	if (_state.demod.deemph) {
		_state.demod.deemph_a = (int)round(1.0/((1.0-exp(-1.0/(_state.demod.rate_out * 75e-6)))));
	}

	/* Set the tuner gain */
	if (_state.dongle.gain == AUTO_GAIN) {
		verbose_auto_gain(_state.dongle.dev);
	} else {
		_state.dongle.gain = nearest_gain(_state.dongle.dev, _state.dongle.gain);
		verbose_gain_set(_state.dongle.dev, _state.dongle.gain);
	}

	rtlsdr_set_bias_tee(_state.dongle.dev, enable_biastee);
	if (enable_biastee)
		fprintf(stderr, "activated bias-T on GPIO PIN 0\n");

	verbose_ppm_set(_state.dongle.dev, _state.dongle.ppm_error);

	
	_state.output.file = stdout;
	
	/* Reset endpoint before we start reading from it (mandatory) */
	if(rtlsdr_reset_buffer(_state.dongle.dev) < 0)
		return false;

	
 	pthread_create(&_state.controller.thread, NULL, controller_thread_fn, (void *)(&_state));
 	usleep(100000);
 	pthread_create(&_state.output.thread, NULL, output_thread_fn, (void *)(&_state));
	pthread_create(&_state.demod.thread, NULL, demod_thread_fn, (void *)(&_state));
	pthread_create(&_state.dongle.thread, NULL, dongle_thread_fn, (void *)(&_state));



	_isSetup = true;
	
	return true;
	
}
  
void CAR_Radio::stop(){
	
	if(_isSetup){
		rtlsdr_close(_state.dongle.dev);

	}

	_isSetup = false;

}

bool  CAR_Radio::setFrequency(uint32_t freq){
	_state.controller.freqs[0] =freq;
	_state.controller.freq_len=1;
	verbose_set_frequency(_state.dongle.dev, _state.dongle.freq);
	return true;
}


bool  CAR_Radio::setFrequency(string freq){
	
 
	_state.controller.freqs[0] = (uint32_t) atofs( (char*)freq.c_str());
	_state.controller.freq_len=1;
	
	
	optimal_settings(&_state, _state.controller.freqs[0], _state.demod.rate_in);
	if (_state.dongle.direct_sampling) {
		verbose_direct_sampling(_state.dongle.dev, _state.dongle.direct_sampling);}
	if (_state.dongle.offset_tuning) {
		verbose_offset_tuning(_state.dongle.dev);}

	verbose_set_frequency(_state.dongle.dev, _state.dongle.freq);

	return true;

}

bool CAR_Radio::setMode(mode_t mode){
	
 
	switch(mode) {
			
		case 	MODE_FM:
			_state.demod.mode_demod = &fm_demod;
			break;
			
		case 	MODE_AM:
			_state.demod.mode_demod = &am_demod;
			break;
			
		case 	MODE_USB:
			_state.demod.mode_demod = &usb_demod;
			break;

		case 	MODE_LSB:
			_state.demod.mode_demod = &lsb_demod;
			break;
	
		case MODE_RAW:
			_state.demod.mode_demod = &raw_demod;
			break;

		case 	MODE_WBFM:
			_state.controller.wb_mode = 1;
			_state.demod.mode_demod = &fm_demod;
			_state.demod.rate_in = 170000;
			_state.demod.rate_out = 170000;
			_state.demod.rate_out2 = 32000;
			_state.output.rate = _state.demod.rate_out2;
			_state.demod.custom_atan = 1;
			//demod.post_downsample = 4;
			_state.demod.deemph = 1;
			_state.demod.squelch_level = 0;
	}
	return true;

}
