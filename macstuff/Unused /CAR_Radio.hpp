//
//  CAR_Radio.hpp
//  rtl
//
//  Created by Vincent Moscaritolo on 4/18/22.
//

#pragma once
#include <stdio.h>
#include <thread>			//Needed for std::thread
#include "rtl-sdr.h"


using namespace std;

#define DEFAULT_SAMPLE_RATE		24000
#define DEFAULT_BUF_LENGTH		(1 * 16384)
#define MAXIMUM_OVERSAMPLE		16
#define MAXIMUM_BUF_LENGTH		(MAXIMUM_OVERSAMPLE * DEFAULT_BUF_LENGTH)
#define AUTO_GAIN					-100
#define BUFFER_DUMP				4096
#define FREQUENCIES_LIMIT		1000

typedef struct demod_state demod_state_t;
typedef struct dongle_state dongle_state_t;
typedef struct output_state output_state_t;
typedef struct controller_state controller_state_t;
typedef struct sdr_state sdr_state_t;

typedef struct dongle_state {
 int      exit_flag;
 pthread_t thread;
 rtlsdr_dev_t *dev;
 int      dev_index;
 uint32_t freq;
 uint32_t rate;
 int      gain;
 uint16_t buf16[MAXIMUM_BUF_LENGTH];
 uint32_t buf_len;
 int      ppm_error;
 int      offset_tuning;
 int      direct_sampling;
 int      mute;
	demod_state_t *demod_target;
} dongle_state_t;

typedef struct output_state {
 int      exit_flag;
 pthread_t thread;
 FILE     *file;
 char     *filename;
 int16_t  result[MAXIMUM_BUF_LENGTH];
 int      result_len;
 int      rate;
 pthread_rwlock_t rw;
 pthread_cond_t ready;
 pthread_mutex_t ready_m;
} output_state_t;


typedef struct demod_state{
	
	int      exit_flag;
	pthread_t thread;
	int16_t  lowpassed[MAXIMUM_BUF_LENGTH];
	int      lp_len;
	int16_t  lp_i_hist[10][6];
	int16_t  lp_q_hist[10][6];
	int16_t  result[MAXIMUM_BUF_LENGTH];
	int16_t  droop_i_hist[9];
	int16_t  droop_q_hist[9];
	int      result_len;
	int      rate_in;
	int      rate_out;
	int      rate_out2;
	int      now_r, now_j;
	int      pre_r, pre_j;
	int      prev_index;
	int      downsample;    /* min 1, max 256 */
	int      post_downsample;
	int      output_scale;
	int      squelch_level, conseq_squelch, squelch_hits, terminate_on_squelch;
	int      downsample_passes;
	int      comp_fir_size;
	int      custom_atan;
	int      deemph, deemph_a;
	int      now_lpr;
	int      prev_lpr_index;
	int      dc_block, dc_avg;
	void     (*mode_demod)(struct demod_state*);
	pthread_rwlock_t rw;
	pthread_cond_t ready;
	pthread_mutex_t ready_m;
	struct output_state *output_target;
} demod_state_t;

typedef struct controller_state{

 int      exit_flag;
 pthread_t thread;
 uint32_t freqs[FREQUENCIES_LIMIT];
 int      freq_len;
 int      freq_now;
 int      edge;
 int      wb_mode;
 pthread_cond_t hop;
 pthread_mutex_t hop_m;
} controller_state_t;

typedef  struct  sdr_state {
	dongle_state_t 		dongle;
	demod_state_t 			demod;
	output_state_t 		output;
	controller_state_t 	controller;
} sdr_state_t;


class CAR_Radio
{
	
public:
	
	typedef enum {
		MODE_FM,
		MODE_AM,
		MODE_USB,
		MODE_LSB,
		MODE_WBFM,
		MODE_RAW
	}mode_t;
	
	CAR_Radio();
	~CAR_Radio();
	
	bool setFrequency(uint32_t);
	bool setFrequency(string);
	bool setMode(mode_t);
	bool begin();
	void stop();
	
private:
	
	void dongle_init( dongle_state_t *s);
	void demod_init( demod_state_t *s);
	void demod_cleanup( demod_state_t *s);
	void output_init( output_state_t *s);
	void output_cleanup( output_state_t *s);
	void controller_init( controller_state_t *s);
	void controller_cleanup( controller_state_t *s);
	
	bool		_isSetup;
	
	sdr_state_t _state;

};
