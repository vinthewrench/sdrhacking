//
//  main1.cpp
//  rtl
//
//  Created by Vincent Moscaritolo on 4/19/22.
//

 

#include <cstdio>
#include <stdlib.h>   // exit()
#include <unistd.h>
#include <mutex>
#include <condition_variable>

#include <atomic>
#include <csignal>
#include <queue>
#include <thread>
#include <string.h>
#include <sys/time.h>
#include <limits.h>
#include <stdexcept>
#include "CommonDefs.hpp"

#include "RtlSdr.hpp"
#include "AudioOutput.hpp"
#include "FmDecode.hpp"

/** Flag is set on SIGINT / SIGTERM. */
static atomic_bool stop_flag(false);

/** Handle Ctrl-C and SIGTERM. */
static void handle_sigterm(int sig)
{
	 stop_flag.store(true);

	 string msg = "\nGot signal ";
	 msg += strsignal(sig);
	 msg += ", stopping ...\n";

	 const char *s = msg.c_str();
	 write(STDERR_FILENO, s, strlen(s));
}
// MARK: -   DataBuffer


/** Buffer to move sample data between threads. */
template <class Element>
class DataBuffer
{
public:
	 /** Constructor. */
	 DataBuffer()
		  : m_qlen(0)
		  , m_end_marked(false)
	 { }

	 /** Add samples to the queue. */
	 void push(vector<Element>&& samples)
	 {
		  if (!samples.empty()) {
				unique_lock<mutex> lock(m_mutex);
				m_qlen += samples.size();
				m_queue.push(move(samples));
				lock.unlock();
				m_cond.notify_all();
		  }
	 }

	 /** Mark the end of the data stream. */
	 void push_end()
	 {
		  unique_lock<mutex> lock(m_mutex);
		  m_end_marked = true;
		  lock.unlock();
		  m_cond.notify_all();
	 }

	 /** Return number of samples in queue. */
	 size_t queued_samples()
	 {
		  unique_lock<mutex> lock(m_mutex);
		  return m_qlen;
	 }

	 /**
	  * If the queue is non-empty, remove a block from the queue and
	  * return the samples. If the end marker has been reached, return
	  * an empty vector. If the queue is empty, wait until more data is pushed
	  * or until the end marker is pushed.
	  */
	 vector<Element> pull()
	 {
		  vector<Element> ret;
		  unique_lock<mutex> lock(m_mutex);
		  while (m_queue.empty() && !m_end_marked)
				m_cond.wait(lock);
		  if (!m_queue.empty()) {
				m_qlen -= m_queue.front().size();
				swap(ret, m_queue.front());
				m_queue.pop();
		  }
		  return ret;
	 }

	 /** Return true if the end has been reached at the Pull side. */
	 bool pull_end_reached()
	 {
		  unique_lock<mutex> lock(m_mutex);
		  return m_qlen == 0 && m_end_marked;
	 }

	 /** Wait until the buffer contains minfill samples or an end marker. */
	 void wait_buffer_fill(size_t minfill)
	 {
		  unique_lock<mutex> lock(m_mutex);
		  while (m_qlen < minfill && !m_end_marked)
				m_cond.wait(lock);
	 }

private:
	 size_t              m_qlen;
	 bool                m_end_marked;
	 queue<vector<Element>> m_queue;
	 mutex               m_mutex;
	 condition_variable  m_cond;
};

// MARK: -   helpers
/* Return Unix time stamp in seconds. */
double get_time()
{
	 struct timeval tv;
	 gettimeofday(&tv, NULL);
	 return tv.tv_sec + 1.0e-6 * tv.tv_usec;
}


bool parse_int(const char *s, int& v, bool allow_unit=false)
{
	 char *endp;
	 long t = strtol(s, &endp, 10);
	 if (endp == s)
		  return false;
	 if ( allow_unit && *endp == 'k' &&
			t > INT_MIN / 1000 && t < INT_MAX / 1000 ) {
		  t *= 1000;
		  endp++;
	 }
	 if (*endp != '\0' || t < INT_MIN || t > INT_MAX)
		  return false;
	 v = (int)t;
	 return true;
}


bool parse_dbl(const char *s, double& v)
{
	 char *endp;
	 v = strtod(s, &endp);
	 if (endp == s)
		  return false;
	 if (*endp == 'k') {
		  v *= 1.0e3;
		  endp++;
	 } else if (*endp == 'M') {
		  v *= 1.0e6;
		  endp++;
	 } else if (*endp == 'G') {
		  v *= 1.0e9;
		  endp++;
	 }
	 return (*endp == '\0');
}


/** Simple linear gain adjustment. */
void adjust_gain(SampleVector& samples, double gain)
{
	 for (size_t i = 0, n = samples.size(); i < n; i++) {
		  samples[i] *= gain;
	 }
}


/**
 * Read data from source device and put it in a buffer.
 *
 * This code runs in a separate thread.
 * The RTL-SDR library is not capable of buffering large amounts of data.
 * Running this in a background thread ensures that the time between calls
 * to RtlSdrSource::get_samples() is very short.
 */
void read_source_data(RtlSdr *rtlsdr, DataBuffer<IQSample> *buf)
{
	 IQSampleVector iqsamples;

	 while (!stop_flag.load()) {

		  if (!rtlsdr->getSamples(iqsamples)) {
				fprintf(stderr, "ERROR: getSamples\n");
				exit(1);
		  }

		  buf->push(move(iqsamples));
	 }

	 buf->push_end();
}

// MARK: -   output thread

/**
 * Get data from output buffer and write to output stream.
 *
 * This code runs in a separate thread.
 */
void write_output_data(AudioOutput *output, DataBuffer<Sample> *buf,
							  unsigned int buf_minfill)
{
	 while (!stop_flag.load()) {

		  if (buf->queued_samples() == 0) {
				// The buffer is empty. Perhaps the output stream is consuming
				// samples faster than we can produce them. Wait until the buffer
				// is back at its nominal level to make sure this does not happen
				// too often.
				buf->wait_buffer_fill(buf_minfill);
		  }

		  if (buf->pull_end_reached()) {
				// Reached end of stream.
				break;
		  }

		  // Get samples from buffer and write to output.
		  SampleVector samples = buf->pull();
		  output->write(samples);
		  if (!(*output)) {
				fprintf(stderr, "ERROR: AudioOutput: %s\n", output->error().c_str());
		  }
	 }
}

// MARK: -   helpers

int main(int argc, const char * argv[]) {
	
	try {
		
		RtlSdr sdr;
		int  errnum = 0;
		
		double  freq    = -1;
		double  ifrate  = 1.0e6;
		int     lnagain = INT_MIN;
		bool    agcmode = false;
		int     pcmrate = 48000;
		bool    stereo  = true;
		double  bufsecs = -1;
		
		enum OutputMode { MODE_RAW, MODE_WAV, MODE_ALSA };
		OutputMode outmode = MODE_ALSA;
		string  filename;
		string  alsadev("default");
		
		
		auto devices = RtlSdr::get_devices();
		if( devices.size() == 0)
			throw Exception("No RTL devices found ");
		
		for( auto dev :devices){
			fprintf(stderr, "%2d \"%s\", \"%s\", \"%s\", \"%s\" \n",
					  dev.index,
					  dev.name.c_str(),
					  dev.vendor.c_str(),
					  dev.product.c_str(),
					  dev.serial.c_str()
					  );
		}
		
		// Catch Ctrl-C and SIGTERM
		struct sigaction sigact;
		sigact.sa_handler = handle_sigterm;
		sigemptyset(&sigact.sa_mask);
		sigact.sa_flags = SA_RESETHAND;
		if (sigaction(SIGINT, &sigact, NULL) < 0) {
			fprintf(stderr, "WARNING: can not install SIGINT handler (%s)\n",
					  strerror(errno));
		}
		if (sigaction(SIGTERM, &sigact, NULL) < 0) {
			fprintf(stderr, "WARNING: can not install SIGTERM handler (%s)\n",
					  strerror(errno));
		}
		
		
		if (!parse_dbl("106.7M", freq) || freq <= 0)
			throw Exception("Bad frequency ");
		
		// Intentionally tune at a higher frequency to avoid DC offset.
		double tuner_freq = freq + 0.25 * ifrate;
		
		if (!sdr.start(0,  &errnum))
			throw Exception("SDR Start failed ", errnum);
		
		// Check LNA gain.
		if (lnagain != INT_MIN) {
			vector<int> gains = sdr.getTunerGains();
			if (find(gains.begin(), gains.end(), lnagain) == gains.end()) {
				if (lnagain != INT_MIN + 1)
					fprintf(stderr, "ERROR: LNA gain %.1f dB not supported by tuner\n", lnagain * 0.1);
				fprintf(stderr, "Supported LNA gains: ");
				for (int g: gains)
					fprintf(stderr, " %.1f dB ", 0.1 * g);
				fprintf(stderr, "\n");
				exit(1);
			}
		}
		
		// Configure RTL-SDR device and start streaming.
		if (!sdr.configure(ifrate, tuner_freq, lnagain,
								 RtlSdr::default_blockLength, agcmode))
			throw Exception("SDR configure Failed ");
		
		tuner_freq = sdr.getFrequency();
		fprintf(stderr, "device tuned for:  %.6f MHz\n", tuner_freq * 1.0e-6);
		
		if (lnagain == INT_MIN)
			fprintf(stderr, "LNA gain:          auto\n");
		else
			fprintf(stderr, "LNA gain:          %.1f dB\n",
					  0.1 * sdr.getTunerGain());
		
		ifrate = sdr.getSampleRate();
		fprintf(stderr, "IF sample rate:    %.0f Hz\n", ifrate);
		
		fprintf(stderr, "RTL AGC mode:      %s\n",
				  agcmode ? "enabled" : "disabled");
		
		// Create source data queue.
		DataBuffer<IQSample> source_buffer;
		
		// Start reading from device in separate thread.
		thread source_thread(read_source_data, &sdr, &source_buffer);
		
		// The baseband signal is empty above 100 kHz, so we can
		// downsample to ~ 200 kS/s without loss of information.
		// This will speed up later processing stages.
		unsigned int downsample = max(1, int(ifrate / 215.0e3));
		fprintf(stderr, "baseband downsampling factor %u\n", downsample);
		
		// Prevent aliasing at very low output sample rates.
		double bandwidth_pcm = min(FmDecoder::default_bandwidth_pcm,
											0.45 * pcmrate);
		fprintf(stderr, "audio sample rate: %u Hz\n", pcmrate);
		fprintf(stderr, "audio bandwidth:   %.3f kHz\n", bandwidth_pcm * 1.0e-3);
		
		// Prepare decoder.
		FmDecoder fm(ifrate,                            // sample_rate_if
						 freq - tuner_freq,                 // tuning_offset
						 pcmrate,                           // sample_rate_pcm
						 stereo,                            // stereo
						 FmDecoder::default_deemphasis,     // deemphasis,
						 FmDecoder::default_bandwidth_if,   // bandwidth_if
						 FmDecoder::default_freq_dev,       // freq_dev
						 bandwidth_pcm,                     // bandwidth_pcm
						 downsample);                       // downsample
		
		// Calculate number of samples in audio buffer.
		unsigned int outputbuf_samples = 0;
		if (bufsecs < 0 &&
			 (outmode == MODE_ALSA || (outmode == MODE_RAW && filename == "-"))) {
			// Set default buffer to 1 second for interactive output streams.
			outputbuf_samples = pcmrate;
		} else if (bufsecs > 0) {
			// Calculate nr of samples for configured buffer length.
			outputbuf_samples = (unsigned int)(bufsecs * pcmrate);
		}
		if (outputbuf_samples > 0) {
			fprintf(stderr, "output buffer:     %.1f seconds\n",
					  outputbuf_samples / double(pcmrate));
		}
		
		//		// Open PPS file.
		//		if (!ppsfilename.empty()) {
		//			 if (ppsfilename == "-") {
		//				  fprintf(stderr, "writing pulse-per-second markers to stdout\n");
		//				  ppsfile = stdout;
		//			 } else {
		//				  fprintf(stderr, "writing pulse-per-second markers to '%s'\n",
		//							 ppsfilename.c_str());
		//				  ppsfile = fopen(ppsfilename.c_str(), "w");
		//				  if (ppsfile == NULL) {
		//						fprintf(stderr, "ERROR: can not open '%s' (%s)\n",
		//								  ppsfilename.c_str(), strerror(errno));
		//						exit(1);
		//				  }
		//			 }
		//			 fprintf(ppsfile, "#pps_index sample_index   unix_time\n");
		//			 fflush(ppsfile);
		//		}
		
		// Prepare output writer.
		unique_ptr<AudioOutput> audio_output;
		switch (outmode) {
			case MODE_RAW:
				fprintf(stderr, "writing raw 16-bit audio samples to '%s'\n",
						  filename.c_str());
				audio_output.reset(new RawAudioOutput(filename));
				break;
			case MODE_WAV:
				fprintf(stderr, "writing audio samples to '%s'\n",
						  filename.c_str());
				audio_output.reset(new WavAudioOutput(filename, pcmrate, stereo));
				break;
			case MODE_ALSA:
				fprintf(stderr, "playing audio to ALSA device '%s'\n",
						  alsadev.c_str());
				audio_output.reset(new AlsaAudioOutput(alsadev, pcmrate, stereo));
				break;
		}
		
		if (!(*audio_output)) {
			fprintf(stderr, "ERROR: AudioOutput: %s\n",
					  audio_output->error().c_str());
			exit(1);
		}
		
		
		// If buffering enabled, start background output thread.
		DataBuffer<Sample> output_buffer;
		thread output_thread;
		if (outputbuf_samples > 0) {
			unsigned int nchannel = stereo ? 2 : 1;
			output_thread = thread(write_output_data,
										  audio_output.get(),
										  &output_buffer,
										  outputbuf_samples * nchannel);
		}
		
		SampleVector audiosamples;
		bool inbuf_length_warning = false;
		double audio_level = 0;
		bool got_stereo = false;
		
	//	double block_time = get_time();
		
		// Main loop.
		for (unsigned int block = 0; !stop_flag.load(); block++) {

			 // Check for overflow of source buffer.
			 if (!inbuf_length_warning &&
				  source_buffer.queued_samples() > 10 * ifrate) {
				  fprintf(stderr,
							 "\nWARNING: Input buffer is growing (system too slow)\n");
				  inbuf_length_warning = true;
			 }

			 // Pull next block from source buffer.
			 IQSampleVector iqsamples = source_buffer.pull();
			 if (iqsamples.empty())
				  break;
//
//			 double prev_block_time = block_time;
//			 block_time = get_time();

			 // Decode FM signal.
			 fm.process(iqsamples, audiosamples);

			 // Measure audio level.
			 double audio_mean, audio_rms;
			 samples_mean_rms(audiosamples, audio_mean, audio_rms);
			 audio_level = 0.95 * audio_level + 0.05 * audio_rms;

			 // Set nominal audio volume.
			 adjust_gain(audiosamples, 0.5);

			 // Show statistics.
			 fprintf(stderr,
						"\rblk=%6d  freq=%8.4fMHz  IF=%+5.1fdB  BB=%+5.1fdB  audio=%+5.1fdB ",
						block,
						(tuner_freq + fm.get_tuning_offset()) * 1.0e-6,
						20*log10(fm.get_if_level()),
						20*log10(fm.get_baseband_level()) + 3.01,
						20*log10(audio_level) + 3.01);
			 if (outputbuf_samples > 0) {
				  unsigned int nchannel = stereo ? 2 : 1;
				  size_t buflen = output_buffer.queued_samples();
				  fprintf(stderr,
							 " buf=%.1fs ",
							 buflen / nchannel / double(pcmrate));
			 }
			 fflush(stderr);

			 // Show stereo status.
			 if (fm.stereo_detected() != got_stereo) {
				  got_stereo = fm.stereo_detected();
				  if (got_stereo)
						fprintf(stderr, "\ngot stereo signal (pilot level = %f)\n",
								  fm.get_pilot_level());
				  else
						fprintf(stderr, "\nlost stereo signal\n");
			 }

//			 // Write PPS markers.
//			 if (ppsfile != NULL) {
//				  for (const PilotPhaseLock::PpsEvent& ev : fm.get_pps_events()) {
//						double ts = prev_block_time;
//						ts += ev.block_position * (block_time - prev_block_time);
//						fprintf(ppsfile, "%8s %14s %18.6f\n",
//								  to_string(ev.pps_index).c_str(),
//								  to_string(ev.sample_index).c_str(),
//								  ts);
//						fflush(ppsfile);
//				  }
//			 }

			 // Throw away first block. It is noisy because IF filters
			 // are still starting up.
			 if (block > 0) {

				  // Write samples to output.
				  if (outputbuf_samples > 0) {
						// Buffered write.
						output_buffer.push(move(audiosamples));
				  } else {
						// Direct write.
						audio_output->write(audiosamples);
				  }
			 }

		}
		
		fprintf(stderr, "\n");

		// Join background threads.
		source_thread.join();
		if (outputbuf_samples > 0) {
			 output_buffer.push_end();
			 output_thread.join();
		}

		
		sdr.stop();
		
	}
	catch ( const Exception& e)  {
		fprintf(stderr," \tError %d %s\n\n", e.getErrorNumber(), e.what());
		
		return -1;
	}
	catch (std::invalid_argument& e)
	{
		fprintf(stderr, "EXCEPTION: %s ",e.what() );
		return -1;
	}
	
	return EXIT_SUCCESS;
	
}
