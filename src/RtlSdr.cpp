//
//  RtlSdr.cpp
//  rtl
//
//  Created by Vincent Moscaritolo on 4/19/22.
//
#include <climits>
#include <cstring>
 
#include "RtlSdr.hpp"


// MARK: -   RtlSdr

RtlSdr::RtlSdr(){
	_isSetup = false;
	_dev = NULL;
	_blockLength = default_blockLength;
	
}

RtlSdr::~RtlSdr(){
	stop();
}
 
bool RtlSdr::start(uint32_t dev_index, int *errorOut){
	
	bool success = false;
	int r;
	
	r = rtlsdr_open(&_dev, dev_index);
	if( r < 0){
		if(errorOut) *errorOut = r;
	}
	else {
		_isSetup = true;
		success = true;
	}
	return success;
}

void RtlSdr::stop(){
	if(_isSetup){
		rtlsdr_close(_dev);
	}

	_isSetup = false;
}


//bool RtlSdr::setFrequency(double freq){
//	
//	
//	return true;
//
//}
 

// Return current sample frequency in Hz.
uint32_t RtlSdr::getSampleRate()
{
	 return rtlsdr_get_sample_rate(_dev);
}


// Return current center frequency in Hz.
uint32_t RtlSdr::getFrequency()
{
	 return rtlsdr_get_center_freq(_dev);
}


// Return current tuner gain in units of 0.1 dB.
int RtlSdr::getTunerGain()
{
	 return rtlsdr_get_tuner_gain(_dev);
}


// Return a list of supported tuner gain settings in units of 0.1 dB.
vector<int> RtlSdr::getTunerGains()
{
	 int num_gains = rtlsdr_get_tuner_gains(_dev, NULL);
	 if (num_gains <= 0)
		  return vector<int>();

	 vector<int> gains(num_gains);
	 if (rtlsdr_get_tuner_gains(_dev, gains.data()) != num_gains)
		  return vector<int>();

	 return gains;
}


// Fetch a bunch of samples from the device.
bool RtlSdr::getSamples(IQSampleVector& samples)
{
	 int r, n_read;

	 if (!_isSetup ||  !_dev)
		  return false;

	 vector<uint8_t> buf(2 * _blockLength);

	 r = rtlsdr_read_sync(_dev, buf.data(), 2 * _blockLength, &n_read);
	 if (r < 0) {
//		  m_error = "rtlsdr_read_sync failed";
		  return false;
	 }

	 if (n_read != 2 * _blockLength) {
//		  m_error = "short read, samples lost";
		  return false;
	 }

	 samples.resize(_blockLength);
	 for (int i = 0; i < _blockLength; i++) {
		  int32_t re = buf[2*i];
		  int32_t im = buf[2*i+1];
		  samples[i] = IQSample( (re - 128) / IQSample::value_type(128),
										 (im - 128) / IQSample::value_type(128) );
	 }

	 return true;
}


bool RtlSdr::configure(uint32_t sample_rate,
							  uint32_t frequency,
							  int tuner_gain,
							  int block_length,
							  bool agcmode){
	int r;
	
	
	if (!_isSetup ||  !_dev)
		return false;
	
	r = rtlsdr_set_sample_rate(_dev, sample_rate);
	if (r < 0) {
		throw Exception("rtlsdr_set_sample_rate failed ");
		return false;
	}
	
	r = rtlsdr_set_center_freq(_dev, frequency);
	if (r < 0) {
		throw Exception("rtlsdr_set_center_freq failed ");
		return false;
	}
	
	if (tuner_gain == INT_MIN) {
		r = rtlsdr_set_tuner_gain_mode(_dev, 0);
		if (r < 0) {
			throw Exception("rtlsdr_set_tuner_gain_mode could not set automatic gain");
			return false;
		}
	} else {
		r = rtlsdr_set_tuner_gain_mode(_dev, 1);
		if (r < 0) {
			throw Exception("rtlsdr_set_tuner_gain_mode could not set manual gain");
			return false;
		}
		
		r = rtlsdr_set_tuner_gain(_dev, tuner_gain);
		if (r < 0) {
			throw Exception("rtlsdr_set_tuner_gain failed");
			return false;
		}
	}
	
	// set RTL AGC mode
	r = rtlsdr_set_agc_mode(_dev, int(agcmode));
	if (r < 0) {
		throw Exception("rtlsdr_set_agc_mode failed");
		return false;
	}
	
	// set block length
	_blockLength = (block_length < 4096) ? 4096 :
	(block_length > 1024 * 1024) ? 1024 * 1024 :
	block_length;
	_blockLength -= _blockLength % 4096;
	
	// reset buffer to start streaming
	if (rtlsdr_reset_buffer(_dev) < 0) {
		throw Exception("rtlsdr_reset_buffer failed");
		return false;
	}
	
	return true;
}



// MARK: -   Static functions

		/** Return a list of supported devices. */
vector<RtlSdr::device_info_t> RtlSdr::get_devices(){
	
	vector<device_info_t> result;

	int device_count = rtlsdr_get_device_count();
	if (device_count <= 0)
		 return result;

	result.reserve(device_count);
	for (int i = 0; i < device_count; i++) {
		
		device_info_t info;
		
		char vendor[256], product[256], serial[256];
		if( rtlsdr_get_device_usb_strings(i, vendor, product, serial) == 0){
			auto name =  rtlsdr_get_device_name(i);
			info.index = i;
			info.name = string(name);
			info.vendor = string(vendor);
			info.product = string(product);
			info.serial = string(serial);
			result.push_back(info);
		}
	}
	return result;
}

