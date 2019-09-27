// Shows how to sample two channels using ADC DMA
//
// Samples A0 and A1, and prints which one has a DTMF signal on it (and which key is pressed)
//
// Repository: https://github.com/rickkas7/ADCDMAGen3_RK
// License: MIT (free for use in open or closed source products)

#include "Particle.h"

#include "ADCDMAGen3_RK.h"

#include <math.h>

SYSTEM_THREAD(ENABLED);

SerialLogHandler logHandler;

const size_t SAMPLE_FREQ = 8000; // Hz

const size_t SAMPLES_IN_BUFFER = 1024;

static nrf_saadc_value_t buffer0[SAMPLES_IN_BUFFER];
static nrf_saadc_value_t buffer1[SAMPLES_IN_BUFFER];

static nrf_saadc_value_t *bufferReady = 0;

const int dtmfLowFreq[4] = { 697, 770, 852, 941 };
const int dtmfHighFreq[4] = { 1209, 1336, 1477, 1633 };

char lastKey0 = 0;
char lastKey1 = 0;

// First index: low freq 0 - 3
// Second index: high freq 0 - 3
const char dtmfKey[4][4] = {
		{ '1', '2', '3', 'A' },
		{ '4', '5', '6', 'B' },
		{ '7', '8', '9', 'C' },
		{ '*', '0', '#', 'D' }
};

// Divide by 2 because two channels. There are two buffers here because we're sampling two
// pins, not because of double buffering.
float floatSamplesA0[SAMPLES_IN_BUFFER / 2];
float floatSamplesA1[SAMPLES_IN_BUFFER / 2];

ADCDMAGen3 adc;

// Forward declaration
char findPressedKey(float *samples, size_t numSamples);
float goertzelMagSquared(int numSamples, int targetFreq, int sampleFreq, const float *data);


void setup() {
	// Optional, just for testing so I can see the logs below
	// waitFor(Serial.isConnected, 10000);

	adc.withBufferCallback([](nrf_saadc_value_t *buf, size_t size) {
		// This gets executed after each sample buffer has been read.
		// Note: This is executed in interrupt context, so beware of what you do here!

		// We just remember the buffer and handle it from loop
		bufferReady = buf;
	});

	ret_code_t err = adc
		.withSampleFreqHz(SAMPLE_FREQ)
		.withDoubleBuffer(SAMPLES_IN_BUFFER, buffer0, buffer1)
		.withSampleMultiplePins({ A0, A1 })
		.init();

	Log.info("adc.init %lu", err);

	adc.start();


}

void loop() {

	if (bufferReady) {
		int16_t *src = (int16_t *)bufferReady;
		bufferReady = 0;

		// The buffer contains 16-bit samples even when sampling at 8 bits!
		size_t index0 = 0, index1 = 0;
		for(size_t ii = 0; ii < SAMPLES_IN_BUFFER; ) {
			// We are using 12-bit sampling so values are 0-4095 (inclusive).

			// Map to values from -1.0 to +1.0 as a float instead
			floatSamplesA0[index0++] = (float)(src[ii++] - 2048) / 2048;
			floatSamplesA1[index1++] = (float)(src[ii++] - 2048) / 2048;
		}

		char key = findPressedKey(floatSamplesA0, sizeof(floatSamplesA0)/sizeof(floatSamplesA0[0]));
		if (key != 0 && key != lastKey0) {
			Log.info("A0 pressed: %c", key);
		}
		lastKey0 = key;

		key = findPressedKey(floatSamplesA1, sizeof(floatSamplesA1)/sizeof(floatSamplesA1[0]));
		if (key != 0 && key != lastKey1) {
			Log.info("A1 pressed: %c", key);
		}
		lastKey1 = key;

	}

}


char findPressedKey(float *samples, size_t numSamples) {
	// Run the Goertzel algorithm to detect the DTMF frequencies
	// (see note below about this algorithm)
	int lowFreq = -1;
	for(size_t ii = 0; ii < sizeof(dtmfLowFreq) / sizeof(dtmfLowFreq[0]); ii++) {
		int freq = dtmfLowFreq[ii];
		float magSquared = goertzelMagSquared(numSamples, freq, SAMPLE_FREQ, samples);
		if (magSquared > 100.0) {
			lowFreq = (int)ii;
			break;
		}
	}

	int highFreq = -1;
	for(size_t ii = 0; ii < sizeof(dtmfHighFreq) / sizeof(dtmfHighFreq[0]); ii++) {
		int freq = dtmfHighFreq[ii];
		float magSquared = goertzelMagSquared(numSamples, freq, SAMPLE_FREQ, samples);
		if (magSquared > 100.0) {
			highFreq = (int)ii;
			break;
		}
	}

	if (lowFreq >= 0 && highFreq >= 0) {
		return dtmfKey[lowFreq][highFreq];
	}
	else {
		return 0;
	}
}


// One alternative is to use a FFT which will find the magnitude and phase of all
// frequencies (separated into buckets). The other is to use the Goertzel algorithm
// which finds the magnitude of a specific frequency. This is ideal when doing
// DTMF decoding because we have a maximum of 8 frequencies to check, and is more
// efficient in this case because we don't need the magnitude of the frequencies
// we don't care about, and also don't need the phase.
//
// https://en.wikipedia.org/wiki/Goertzel_algorithm
// Modified version of:
// https://stackoverflow.com/questions/11579367/implementation-of-goertzel-algorithm-in-c
float goertzelMagSquared(int numSamples, int targetFreq, int sampleFreq, const float *data) {
	float   omega,sine,cosine,coeff,q0,q1,q2,magnitudeSquared,real,imag;

	int k = (int) (0.5 + (((float)numSamples * targetFreq) / sampleFreq));
	omega = (2.0 * M_PI * k) / (float)numSamples;
	sine = sin(omega);
	cosine = cos(omega);
	coeff = 2.0 * cosine;
	q0=0;
	q1=0;
	q2=0;

	for(int ii=0; ii < numSamples; ii++) {
		q2 = q1;
		q1 = q0;
		q0 = coeff * q1 - q2 + data[ii];
	}

	real = (q0 - q1 * cosine);
	imag = (-q1 * sine);

	magnitudeSquared = real*real + imag*imag;
	return magnitudeSquared;
}



