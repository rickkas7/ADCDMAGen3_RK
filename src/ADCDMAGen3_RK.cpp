//
// Repository: https://github.com/rickkas7/ADCDMAGen3_RK
// License: MIT (free for use in open or closed source products)

#include "ADCDMAGen3_RK.h"

ADCDMAGen3 *ADCDMAGen3::instance = 0;

static Logger log("app.adc");

// This should match what's defined in adc_hal.cpp
static const nrfx_saadc_config_t defaultSaadcConfig = {
    .resolution         = NRF_SAADC_RESOLUTION_12BIT,
    .oversample         = NRF_SAADC_OVERSAMPLE_DISABLED,
    .interrupt_priority = NRFX_SAADC_CONFIG_IRQ_PRIORITY
};


ADCDMAGen3::ADCDMAGen3() {
	instance = this;

	// These are the default settings we use (some may be different than NRFX_SAADC_DEFAULT_CONFIG)
	saadcConfig.low_power_mode = false;
	saadcConfig.resolution = NRF_SAADC_RESOLUTION_12BIT;
	saadcConfig.oversample = NRF_SAADC_OVERSAMPLE_DISABLED;
	saadcConfig.interrupt_priority = APP_IRQ_PRIORITY_LOW;

	// These are also initialized by NRFX_SAADC_DEFAULT_CHANNEL_CONFIG_SE but some of these values
	// are different
	channelConfig.reference = NRF_SAADC_REFERENCE_VDD4;
	channelConfig.gain = NRF_SAADC_GAIN1_4;
	channelConfig.acq_time = NRF_SAADC_ACQTIME_10US;
    channelConfig.mode = NRF_SAADC_MODE_SINGLE_ENDED;
	channelConfig.pin_n = NRF_SAADC_INPUT_DISABLED;
	channelConfig.resistor_p = NRF_SAADC_RESISTOR_DISABLED;
	channelConfig.resistor_n = NRF_SAADC_RESISTOR_DISABLED;
	channelConfig.burst = NRF_SAADC_BURST_DISABLED;

	// Override the default setting to 32 bits
    timerConfig.bit_width = NRF_TIMER_BIT_WIDTH_32;

}

ADCDMAGen3::~ADCDMAGen3() {
	freeBufferPoolIfNecessary();
}



ret_code_t ADCDMAGen3::init() {
	ret_code_t err;

	if (pinList.empty()) {
		log.error("no pins set");
		return 1;
	}

	// Undo any previous initialization if necessary
	uninit();

	if (hardwareTimer.p_reg == NRF_TIMER4) {
		// Make sure NFC is off, because we're stealing the timer from it when using Timer4
		NFC.off();
	}

	// IMPORTANT: You must detach the IRQ handler from the system firmware and re-attach it to
	// user firmware, or this won't work right.
	attachInterruptDirect(SAADC_IRQn, nrfx_saadc_irq_handler, false);

	// This calls attachInterruptDirect for the selected timer
	attachTimerInterrupt();

	setupState = SetupState::ATTACH_INTERRUPT;

	// If buffers were not passed in, allocate them
	for(size_t ii = 0; ii < poolSize; ii++) {
		if (!bufferPool[ii]) {
			bufferPool[ii] = new nrf_saadc_value_t[bufferSize];
		}
	}

	// Initialize SAADC
	err = nrfx_saadc_init(&saadcConfig, adcCallbackStatic);
	if (err) {
		log.error("nrfx_saadc_init err=%lu", err);
		return err;
	}
	setupState = SetupState::SAADC_INIT;

	// Configure SAADC channel

	// Add channels
	for(auto it = pinList.begin(); it != pinList.end(); it++) {
		pin_t particlePin = *it;
		nrf_saadc_input_t ain = particlePinToAIN(particlePin);

		channelConfig.pin_p = ain;
		err = nrfx_saadc_channel_init(ain - NRF_SAADC_INPUT_AIN0, &channelConfig);
		if (err) {
			log.error("nrfx_saadc_channel_init err=%lu pin=%d", err, particlePin);
			return err;
		}
	}

	// Add buffers pools
	for(size_t ii = 0; ii < poolSize; ii++) {
		// Where to store the data
		err = nrfx_saadc_buffer_convert(bufferPool[ii], bufferSize);
		if (err) {
			log.error("nrfx_saadc_buffer_convert err=%lu ii=%d", err, ii);
			return err;
		}
	}

	// Initialize the timer
    err = nrfx_timer_init(&hardwareTimer, &timerConfig, timerCallbackStatic);
	if (err) {
		log.error("nrfx_timer_init err=%lu", err);
		return err;
	}
	setupState = SetupState::TIMER_INIT;


	// ticksPerSecond is typically 16000000 (16 MHz)
	// 1 tick = 0.0625 microseconds = 62.5 nanoseconds
    uint32_t ticksPerSecond = nrfx_timer_ms_to_ticks(&hardwareTimer, 1000);

	uint32_t ticks = ticksPerSecond / sampleFreqHz;

	// timerCcChannel default to NRF_TIMER_CC_CHANNEL0
	// timerShortMask default to NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK
    nrfx_timer_extended_compare(&hardwareTimer, timerCcChannel, ticks, timerShortMask, false);

    uint32_t timerCompareEventAddr = nrfx_timer_compare_event_address_get(&hardwareTimer, NRF_TIMER_CC_CHANNEL0);
    uint32_t saadcSampleTaskAddr   = nrfx_saadc_sample_task_get();

    // setup ppi channel so that timer compare event is triggering sample task in SAADC
    err = nrfx_ppi_channel_alloc(&ppiChannel);
	if (err) {
		log.error("nrfx_ppi_channel_alloc err=%lu", err);
		return err;
	}
	setupState = SetupState::PPI_ALLOC;

    err = nrfx_ppi_channel_assign(ppiChannel, timerCompareEventAddr, saadcSampleTaskAddr);
	if (err) {
		log.error("nrfx_ppi_channel_assign err=%lu", err);
		return err;
	}

	err = nrfx_ppi_channel_enable(ppiChannel);
	if (err) {
		log.error("nrfx_ppi_channel_enable err=%lu", err);
		return err;
	}
	setupState = SetupState::COMPLETE;

	return 0;
}

void ADCDMAGen3::uninit() {

	if (setupState >= SetupState::PPI_ALLOC) {
		// nrfx_ppi_channel_free will disable the channel if necessary, no need to do it here
		ret_code_t err = nrfx_ppi_channel_free(ppiChannel);
		if (err) {
			log.error("nrfx_ppi_channel_free err=%lu", err);
		}
	}

	if (setupState >= SetupState::TIMER_INIT) {
		// nrfx_timer_uninit will disable the timer if necessary, no need to do it here
	    nrfx_timer_uninit(&hardwareTimer);
	}

	if (setupState >= SetupState::SAADC_INIT) {
	    nrfx_saadc_uninit();
	}

	if (setupState >= SetupState::ATTACH_INTERRUPT) {
		detachTimerInterrupt();
		detachInterrupt(SAADC_IRQn);
	}

	setupState = SetupState::NOT_INITIALIZED;
}

void ADCDMAGen3::restoreDefaults() {
	nrfx_saadc_init(&defaultSaadcConfig, adcEmptyCallbackStatic);
	setupState = SetupState::SAADC_INIT;
}

ADCDMAGen3 &ADCDMAGen3::withSingleBuffer(size_t size) {
	freeBufferPoolIfNecessary();

	bufferSize = size;
	poolSize = 1;
	freeBufferPool = true;

	return *this;
}

ADCDMAGen3 &ADCDMAGen3::withSingleBuffer(size_t size, nrf_saadc_value_t *buf) {
	freeBufferPoolIfNecessary();

	bufferSize = size;
	poolSize = 1;
	bufferPool[0] = buf;
	freeBufferPool = false;

	return *this;
}

ADCDMAGen3 &ADCDMAGen3::withDoubleBuffer(size_t size) {
	freeBufferPoolIfNecessary();

	bufferSize = size;
	poolSize = 2;
	freeBufferPool = true;

	return *this;

}


ADCDMAGen3 &ADCDMAGen3::withDoubleBuffer(size_t size, nrf_saadc_value_t *buf0, nrf_saadc_value_t *buf1) {
	freeBufferPoolIfNecessary();

	bufferSize = size;
	poolSize = 2;
	bufferPool[0] = buf0;
	bufferPool[1] = buf1;
	freeBufferPool = false;

	return *this;
}


ADCDMAGen3 &ADCDMAGen3::withDoubleBufferSplit(size_t size, nrf_saadc_value_t *buf) {
	freeBufferPoolIfNecessary();

	bufferSize = size / 2;
	poolSize = 2;
	bufferPool[0] = buf;
	bufferPool[1] = &buf[bufferSize];
	freeBufferPool = false;

	return *this;
}



// Select the input pin for the channel.
// Particle  ADC   nRF    pin_p                 notes
// Pin       Chan  Pin
//           0     P0.02  NRF_SAADC_INPUT_AIN0  A7 on SoMs
// A0        1     P0.03  NRF_SAADC_INPUT_AIN1
// A1        2     P0.04  NRF_SAADC_INPUT_AIN2
//           3     P0.05  NRF_SAADC_INPUT_AIN3  A6 on SoMs, BAT on Argon and Xenon
// A2        4     P0.28  NRF_SAADC_INPUT_AIN4
// A3        5     P0.29  NRF_SAADC_INPUT_AIN5
// A4        6     P0.30  NRF_SAADC_INPUT_AIN6
// A5        7     P0.31  NRF_SAADC_INPUT_AIN7
nrf_saadc_input_t ADCDMAGen3::particlePinToAIN(pin_t pin) const {
	switch(pin) {
	case A0: return NRF_SAADC_INPUT_AIN1;
	case A1: return NRF_SAADC_INPUT_AIN2;
	case A2: return NRF_SAADC_INPUT_AIN4;
	case A3: return NRF_SAADC_INPUT_AIN5;
	case A4: return NRF_SAADC_INPUT_AIN6;
	case A5: return NRF_SAADC_INPUT_AIN7;
#ifdef A6
	case A6: return NRF_SAADC_INPUT_AIN3;
#endif
#ifdef A7
	case A7: return NRF_SAADC_INPUT_AIN0;
#endif
	default: return NRF_SAADC_INPUT_AIN0;
	}
}

void ADCDMAGen3::attachTimerInterrupt() {
	if (hardwareTimer.p_reg == NRF_TIMER1) {
		attachInterruptDirect(TIMER1_IRQn, nrfx_timer_1_irq_handler, false);
	}
	else
	if (hardwareTimer.p_reg == NRF_TIMER2) {
		attachInterruptDirect(TIMER2_IRQn, nrfx_timer_2_irq_handler, false);
	}
	else
	if (hardwareTimer.p_reg == NRF_TIMER3) {
		attachInterruptDirect(TIMER3_IRQn, nrfx_timer_3_irq_handler, false);
	}
	else
	if (hardwareTimer.p_reg == NRF_TIMER4) {
		attachInterruptDirect(TIMER4_IRQn, nrfx_timer_4_irq_handler, false);
	}
}

void ADCDMAGen3::detachTimerInterrupt() {
	if (hardwareTimer.p_reg == NRF_TIMER1) {
		detachInterrupt(TIMER1_IRQn);
	}
	else
	if (hardwareTimer.p_reg == NRF_TIMER2) {
		detachInterrupt(TIMER2_IRQn);
	}
	else
	if (hardwareTimer.p_reg == NRF_TIMER3) {
		detachInterrupt(TIMER3_IRQn);
	}
	else
	if (hardwareTimer.p_reg == NRF_TIMER4) {
		detachInterrupt(TIMER4_IRQn);
	}
}


void ADCDMAGen3::freeBufferPoolIfNecessary() {
	if (freeBufferPool) {
		for(size_t ii = 0; ii < poolSize; ii++) {
			delete[] bufferPool[ii];
			bufferPool[ii] = 0;
		}

	}
}

// [static]
void ADCDMAGen3::timerCallbackStatic(nrf_timer_event_t event_type, void* p_context) {
	// We don't do anything here, because the timer directly triggers the ADC sample
	// via PPI (peripheral-to-peripheral interconnect)
}


void ADCDMAGen3::adcCallback(nrfx_saadc_evt_t const *event) {
    if (event->type == NRFX_SAADC_EVT_DONE) {
        // Let loop handle this buffer
    	if (bufferCallback) {
    		bufferCallback(event->data.done.p_buffer, bufferSize);
    	}

		// Fetch this buffer again after the other one fills up
		nrfx_saadc_buffer_convert(event->data.done.p_buffer, bufferSize);

        if (!continuousMode) {
        	// Stop timer after the buffer is filled
        	nrfx_timer_disable(&hardwareTimer);
        }
    }
}

// [static]
void ADCDMAGen3::adcCallbackStatic(nrfx_saadc_evt_t const *event) {
    instance->adcCallback(event);
}

// [static]
void ADCDMAGen3::adcEmptyCallbackStatic(nrfx_saadc_evt_t const *event) {
	// Do nothing
}




