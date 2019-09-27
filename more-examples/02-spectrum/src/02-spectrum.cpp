#include "Particle.h"

#include "ADCDMAGen3_RK.h"


#include "FftComplex.h" // https://www.nayuki.io/page/free-small-fft-in-multiple-languages
#include <cmath> // std::abs


#include "Adafruit_SSD1306_RK.h"

// The Adafruit library defines abs as a macro. Undo that so std::abs works right.
#undef abs

SYSTEM_THREAD(ENABLED);

SerialLogHandler logHandler;

const unsigned long UPDATE_PERIOD_MS = 20;

const size_t SAMPLE_FREQ = 32000; // Hz

const size_t SAMPLES_IN_BUFFER = 512;

static nrf_saadc_value_t buffer[SAMPLES_IN_BUFFER * 2];

static nrf_saadc_value_t *bufferReady = 0;

std::vector< std::complex<double> > complexSamples;

const unsigned long UPDATE_PERIOD = 20;

unsigned long lastUpdate;

ADCDMAGen3 adc;


// SSD1306 SPI Display Settings
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

#define OLED_DC     A4
#define OLED_CS     A3
#define OLED_RESET  A5

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT,
		&SPI, OLED_DC, OLED_RESET, OLED_CS);


void setup() {
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

	ret_code_t err = adc
		.withSampleFreqHz(SAMPLE_FREQ)
		.withDoubleBufferSplit(SAMPLES_IN_BUFFER * 2, buffer)
		.withResolution(NRF_SAADC_RESOLUTION_12BIT)
		.withAcqTime(NRF_SAADC_ACQTIME_3US)
		.withSamplePin(A0)
		.init();

	Log.info("adc.init %lu", err);

	adc.start();

	display.begin(SSD1306_SWITCHCAPVCC);
    display.clearDisplay();
    display.display();

}

void loop() {
	if (millis() - lastUpdate >= UPDATE_PERIOD) {
		lastUpdate = millis();

		if (bufferReady) {
			int16_t *src = (int16_t *)bufferReady;
			bufferReady = 0;

			for(size_t ii = 0; ii < SAMPLES_IN_BUFFER; ii++) {
				// We are using 12-bit sampling so values are 0-4095 (inclusive).

				// Create real part from -1.0 to +1.0 instead
				complexSamples[ii] = (double)(src[ii] - 2048) / 2048;
			}

			// Run the FFT on the samples
			Fft::transformRadix2(complexSamples);

		    display.clearDisplay();

		    // Results in the frequency domain are from index 0 to index (SAMPLES_IN_BUFFER / 2)
			// where index 0 is the DC component and the maximum indexex is the highest
			// frequency, which is 1/2 the sampling frequency of 8kHz
			// (The top half of the buffer is the negative frequencies, which we ignore.)
			// Start at 1 to ignore the DC component
			for(size_t ii = 1; ii < 128; ii++) {
				// int freq = ii * SAMPLE_FREQ / SAMPLES_IN_BUFFER;
				double value = std::abs(complexSamples[ii].real());

				int height = (int)value * 64 / 5;
				if (height > 64) {
					height = 64;
				}

				if (height > 1) {
					display.drawLine(ii, 64, ii, 64 - height, 1);
				}
			}

		    display.display();
		}
	}

}







