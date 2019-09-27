// One-shot frequency counter also tests high frequency sampling
//
// When you press the MODE button it samples A0 to see if there's a dominant frequency.
// Sampling is at 200 kHz so you can find frequencies up to a little less than 100 kHz.
//
// Repository: https://github.com/rickkas7/ADCDMAGen3_RK
// License: MIT (free for use in open or closed source products)

#include "Particle.h"

#include "ADCDMAGen3_RK.h"

#include "FftComplex.h" // https://www.nayuki.io/page/free-small-fft-in-multiple-languages
#include <cmath> // std::abs

SYSTEM_THREAD(ENABLED);

SerialLogHandler logHandler;

const size_t SAMPLE_FREQ = 200000; // Hz

const size_t SAMPLES_IN_BUFFER = 1024;

static nrf_saadc_value_t buffer[SAMPLES_IN_BUFFER];

static nrf_saadc_value_t *bufferReady = 0;

std::vector< std::complex<double> > complexSamples;

ADCDMAGen3 adc;
bool startSampling = false;

// Forward declarations
void buttonHandler(system_event_t event, int data);


void setup() {
	// Register handler to handle clicking on the SETUP button
	System.on(button_click, buttonHandler);

	// Optional, just for testing so I can see the logs below
	// waitFor(Serial.isConnected, 10000);

	// Pre-allocate an array of complex<double> samples in a vector. We reuse this for FFT.
	complexSamples.resize(SAMPLES_IN_BUFFER);

	adc.withBufferCallback([](nrf_saadc_value_t *buf, size_t size) {
		// This gets executed after each sample buffer has been read.
		// Note: This is executed in interrupt context, so beware of what you do here!

		// We just remember the buffer and handle it from loop
		bufferReady = buf;
	});

	// In order to sample at 200 kHz you need to reduce the acqTime to 3 microseconds!

	adc .withSampleFreqHz(SAMPLE_FREQ)
		.withSingleBuffer(SAMPLES_IN_BUFFER, buffer)
		.withOneShotMode()
		.withResolution(NRF_SAADC_RESOLUTION_12BIT)
		.withAcqTime(NRF_SAADC_ACQTIME_3US)
		.withSamplePin(A0);

}

void loop() {

	if (bufferReady) {
		int16_t *src = (int16_t *)bufferReady;
		bufferReady = 0;

		Log.info("buffer ready");

		// We completely free the ADC between uses; you can leave it activated and
		// just use start() and stop() to turn on and off sampling
		adc.uninit();

		// The buffer contains 16-bit samples even when sampling at 8 bits!
		for(size_t ii = 0; ii < SAMPLES_IN_BUFFER; ii++) {
			// We are using 12-bit sampling so values are 0-4095 (inclusive).

			// Create real part from -1.0 to +1.0 instead
			complexSamples[ii] = (double)(src[ii] - 2048) / 2048;
		}

		// Run the FFT on the samples
		Fft::transformRadix2(complexSamples);

		double largestValue = 0.0;
		int largestFreq = -1;

		// Results in the frequency domain are from index 0 to index (SAMPLES_IN_BUFFER / 2)
		// where index 0 is the DC component and the maximum indexex is the highest
		// frequency, which is 1/2 the sampling frequency of 8kHz
		// (The top half of the buffer is the negative frequencies, which we ignore.)
		// Start at 1 to ignore the DC component
		for(size_t ii = 1; ii < SAMPLES_IN_BUFFER / 2; ii++) {
			int freq = ii * SAMPLE_FREQ / SAMPLES_IN_BUFFER;
			double value = std::abs(complexSamples[ii].real());
			if (value > 100.0 && value > largestValue) {
				largestValue = value;
				largestFreq = freq;
			}
		}
		if (largestFreq > 0) {
			Log.info("freq=%d", largestFreq);
		}
		else {
			Log.info("no notable frequency");
		}
	}

	if (startSampling) {
		startSampling = false;

		Log.info("start sampling");

		// You can either init() once, then use start() and stop() only, or uninit() when not actively using the ADC.
		adc.init();
		adc.start();
	}

}


// button handler for the SETUP button, used to toggle recording on and off
void buttonHandler(system_event_t event, int data) {
	// Start sampling from the loop thread
	startSampling = true;
}






