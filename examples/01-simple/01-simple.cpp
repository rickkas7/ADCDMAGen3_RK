// Simple ADC DMA example

// Repository: https://github.com/rickkas7/ADCDMAGen3_RK
// License: MIT (free for use in open or closed source products)

#include "Particle.h"

#include "ADCDMAGen3_RK.h"

// Works threaded or not
SYSTEM_THREAD(ENABLED);

// Print debug message to USB serial
SerialLogHandler logHandler;

const size_t SAMPLE_FREQ = 16000; // Hz

const size_t SAMPLES_IN_BUFFER = 1024;

// This is where the samples are stored
static nrf_saadc_value_t buffer0[SAMPLES_IN_BUFFER];
static nrf_saadc_value_t buffer1[SAMPLES_IN_BUFFER];

static nrf_saadc_value_t *bufferReady = 0;


ADCDMAGen3 adc;

void myBufferCallback(nrf_saadc_value_t *buf, size_t size);


void setup() {
	// Optional, just for testing so I can see the logs below
	// waitFor(Serial.isConnected, 10000);

	ret_code_t err = adc
		.withSampleFreqHz(SAMPLE_FREQ)
		.withDoubleBuffer(SAMPLES_IN_BUFFER, buffer0, buffer1)
		.withSamplePin(A0)
		.withBufferCallback(myBufferCallback)
		.init();

	Log.info("adc.init %lu", err);

	adc.start();
}

void loop() {

	if (bufferReady) {
		int16_t *samples = (int16_t *)bufferReady;
		bufferReady = 0;

		// Do something with the samples here
		// For the default 12-bit sampling, each index in samples will be 0 - 4095 (inclusive)

	}
}

void myBufferCallback(nrf_saadc_value_t *buf, size_t size) {
	// This gets executed after each sample buffer has been read.
	// Note: This is executed in interrupt context, so beware of what you do here!

	// We just remember the buffer and handle it from loop
	bufferReady = buf;
}








