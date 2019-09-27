#ifndef __ADCDMAGEN3_RK_H
#define __ADCDMAGEN3_RK_H

#include "Particle.h"

// Particle.h defines macro CHG for the charge pin on some devices, which conflicts with nrfx_ppi.h.
#undef CHG

#include "nrfx_ppi.h"
#include "nrfx_saadc.h"
#include "nrfx_timer.h"

#include <vector>

/**
 * @brief Analog to digital conversion using DMA for Particle Gen 3 devices (Argon, Boron, Xenon)
 *
 * Note: You can only instantiate one of these per application since there can only be one
 * ADC DMA in use at a time (though you can sample multiple channels).
 *
 * Configuration is done using the withXXX() methods and can be chained, fluent-style.
 *
 * After setting it up, use the init() method to initialize the ADC DMA.
 */
class ADCDMAGen3 {
public:
	ADCDMAGen3();
	virtual ~ADCDMAGen3();

	/**
	 * @brief Initialize the ADC
	 *
	 * You typically use the withXXX() methods to configure the settings, then when you
	 * are done, call init().
	 *
	 * You can change settings and call init() again if desired.
	 *
	 * This call does not start sampling, use start() to do that.
	 */
	ret_code_t init();

	/**
	 * @brief Start sampling
	 *
	 * The default is to sample continuously (or until stop() is called).
	 *
	 * If you use withOneShotMode(), then the first buffer is filled and the sampling is stopped
	 * automatically and you should not call stop().
	 */
	void start() { nrfx_timer_enable(&hardwareTimer); };

	/**
	 * @brief Stop sampling
	 */
	void stop() { nrfx_timer_disable(&hardwareTimer); };

	/**
	 * @brief Uninitialize the ADC and free its resources
	 *
	 * You do not need to stop() before uninit(). Once you uninit() you can switch back to using
	 * analogRead() however you should also call restoreDefault() to make sure the settings used
	 * by analogRead() are restored properly.
	 */
	void uninit();

	/**
	 * @brief Restore the bit size, oversampling, and IRQ settings used by analogRead().
	 */
	void restoreDefaults();

	/**
	 * @brief Set a single buffer of the specified size, allocated on the heap during init()
	 *
	 * @param size size in samples, not bytes. Each sample is 16-bits (2 bytes).
	 *
	 * Single buffer should only be used in with withOneShotMode(). If you are continuously
	 * sampling you should use double buffer mode so writes can continue in the other buffer
	 * while you are handling the full buffer.
	 */
	ADCDMAGen3 &withSingleBuffer(size_t size);

	/**
	 * @brief Set a single buffer of the specified size and pointer to buffer
	 *
	 * @param size size in samples, not bytes. Each sample is 16-bits (2 bytes).
	 *
	 * @param buf Pointer to a buffer of samples. Note that each sample a nrf_saadc_value_t
	 * (16 bits, 2 bytes). You must not deallocate this buffer. It's typically allocated
	 * as a global variable.
	 *
	 * Single buffer should only be used in with withOneShotMode(). If you are continuously
	 * sampling you should use double buffer mode so writes can continue in the other buffer
	 * while you are handling the full buffer.
	 */
	ADCDMAGen3 &withSingleBuffer(size_t size, nrf_saadc_value_t *buf);


	/**
	 * @brief Set double buffers, each one of the specified size, allocated on the heap during init()
	 *
	 * @param size size in samples, not bytes. Each sample is 16-bits (2 bytes). And since it's
	 * a double buffer, 4 * size bytes will be allocated on the heap.
	 *
	 * Double buffers should be used with continuous mode is enabled (the default).
	 */
	ADCDMAGen3 &withDoubleBuffer(size_t size);

	/**
	 * @brief Set double buffers, each one of the specified size
	 *
	 * @param size size in samples, not bytes. Each sample is 16-bits (2 bytes).
	 *
	 * @param buf0 Pointer to a buffer of samples. Note that each sample a nrf_saadc_value_t
	 * (16 bits, 2 bytes). You must not deallocate this buffer. It's typically allocated
	 * as a global variable.
 	 *
	 * @param buf1 Pointer to a buffer of samples. Note that each sample a nrf_saadc_value_t
	 * (16 bits, 2 bytes). You must not deallocate this buffer. It's typically allocated
	 * as a global variable.
	 *
	 * Double buffers should be used with continuous mode is enabled (the default).
	 */
	ADCDMAGen3 &withDoubleBuffer(size_t size, nrf_saadc_value_t *buf0, nrf_saadc_value_t *buf1);


	/**
	 * @brief Set double buffers, created by taking a larger buffer and splitting it into two halves.
	 *
	 * @param size size in samples, not bytes, of buf Each sample is 16-bits (2 bytes). The sample
	 * size will be half that, because the buffer is split into two half-buffers.
	 *
	 * @param buf Pointer to a buffer of samples. Note that each sample a nrf_saadc_value_t
	 * (16 bits, 2 bytes). You must not deallocate this buffer. It's typically allocated
	 * as a global variable.
	 *
	 * Double buffers should be used with continuous mode is enabled (the default).
	 */
	ADCDMAGen3 &withDoubleBufferSplit(size_t size, nrf_saadc_value_t *buf);

	/**
	 * @brief Sets the sampling frequency
	 *
	 * @param sampleFreqHz The sampling frequency in Hertz or samples per second. Default is 16000.
	 *
	 * Maximum is around 200000 however make sure you also reduce the acqTime to NRF_SAADC_ACQTIME_3US when
	 * using the maximum sampling frequency.
	 */
	ADCDMAGen3 &withSampleFreqHz(size_t sampleFreqHz) { this->sampleFreqHz = sampleFreqHz; return *this; };

	/**
	 * @brief Gets the sampling frequency in Hertz (samples per second).
	 */
	size_t getSampleFreqHz() const { return sampleFreqHz; };

	/**
	 * @brief Sets the hardware timer to use if using automatic acquisition
	 *
	 * - 0: Softdevice - do not use
 	 * - 1: Radio - probably best to not use, might work if not using BLE or mesh
 	 * - 2: Usart (Serial1) Can use if not using Serial1.
 	 * - 3: Usart (Serial2 on Xenon) Cannot be used on Argon or Boron because it's required by NCP.
 	 * - 4: NFC - recommended (unless you need NFC). This is the default.
	 *
	 * Using a hardware timer is necessary to control the frequency of sampling. It's possible
	 * to manually trigger a single sample, but that tends to not be very useful and isn't supported
	 * by this library.
	 */
	ADCDMAGen3 &withHardwareTimer(nrfx_timer_t hardwareTimer) { this->hardwareTimer = hardwareTimer; return *this; };


	/**
	 * @brief Sets the ADC resolution
	 *
	 * - NRF_SAADC_RESOLUTION_8BIT
	 * - NRF_SAADC_RESOLUTION_10BIT
	 * - NRF_SAADC_RESOLUTION_12BIT (default)
	 * - NRF_SAADC_RESOLUTION_14BIT
	 *
	 * Note that the values stored in the buffer are always 16-bit values (even if the resolution is 8 bit).
	 */
	ADCDMAGen3 &withResolution(nrf_saadc_resolution_t resolution) { saadcConfig.resolution = resolution; return *this; };

	/**
	 * @brief Gets the ADC resolution
	 *
	 * See withResolution() for an explanation of return values.
	 */
	nrf_saadc_resolution_t getResolution() const { return saadcConfig.resolution; };

	/**
	 * @brief Sets the oversampling mode and burst mode
	 *
	 * @param oversample The oversampling value. The default is disabled.
	 *
	 * - NRF_SAADC_OVERSAMPLE_DISABLED (default)
	 * - NRF_SAADC_OVERSAMPLE_2X
	 * - NRF_SAADC_OVERSAMPLE_4X
	 * - NRF_SAADC_OVERSAMPLE_8X
	 * - NRF_SAADC_OVERSAMPLE_16X
	 * - NRF_SAADC_OVERSAMPLE_32X
	 * - NRF_SAADC_OVERSAMPLE_64X
	 * - NRF_SAADC_OVERSAMPLE_128X
	 * - NRF_SAADC_OVERSAMPLE_256X
	 *
	 * @param burst Enable burst mode. Default is disabled.
	 *
	 * - NRF_SAADC_BURST_DISABLED (default)
	 * - NRF_SAADC_BURST_ENABLED
	 *
	 * When enabling oversampling, multiple samples are taken and averaged to reduce noise. For example, if
	 * NRF_SAADC_OVERSAMPLE_4X is set, then 4 samples are taken and averaged.
	 *
	 * If burst mode is enabled, then those 4 samples are taken as quickly as possible.
	 *
	 * If burst mode is disabled, then those 4 samples are taken one on each trigger, resulting in a buffer
	 * taking 4 times longer to fill up, for example.
	 */
	ADCDMAGen3 &withOversample(nrf_saadc_oversample_t oversample, nrf_saadc_burst_t burst) { saadcConfig.oversample = oversample; channelConfig.burst = burst; return *this; };

	/**
	 * @brief Get the oversample setting
	 *
	 * See withOversample() for values
	 */
	nrf_saadc_oversample_t getOversample() const { return saadcConfig.oversample; };

	/**
	 * @brief Get the burstMode setting (true = burst mode enabled, false = disabled)
	 *
	 * See withOversample() for for information.
	 */
	nrf_saadc_burst_t getBurstMode() const { return channelConfig.burst; };

	/**
	 * @brief Sets the interrupt priority for ADC interrupts
	 *
	 * - APP_IRQ_PRIORITY_HIGHEST
	 * - APP_IRQ_PRIORITY_HIGH
	 * - APP_IRQ_PRIORITY_MID
	 * - APP_IRQ_PRIORITY_LOW (default)
	 * - APP_IRQ_PRIORITY_LOWEST
	 *
	 */
	ADCDMAGen3 &withInterruptPriority(uint8_t interruptPriority) { saadcConfig.interrupt_priority = interruptPriority; return *this; };

	/**
	 * @brief Gets the interrupt priority
	 *
	 * See withInterruptPriority() for values
	 */
	uint8_t getInterruptPriority() const { return saadcConfig.interrupt_priority; };

	/**
	 * @brief Set ADC reference and gain
	 *
	 * @param reference the voltage reference
	 *
	 * - NRF_SAADC_REFERENCE_INTERNAL - 0.6V
	 * - NRF_SAADC_REFERENCE_VDD4 - VDD/4, typically 3.3V, so 0.9V. This is the default and what is used by analogRead()
	 *
	 * @param gain the gain on the input signal
	 *
	 * - NRF_SAADC_GAIN1_6 - 1/6 gain, commonly used with the internal reference to allow for 0 - 3.6V)
	 * - NRF_SAADC_GAIN1_5 - 1/5 gain
	 * - NRF_SAADC_GAIN1_4 - 1/4 gain, commonly used with VDD4, so the input is from 0 to VDD (default)
	 * - NRF_SAADC_GAIN1_3 - 1/3 gain
	 * - NRF_SAADC_GAIN1_2 - 1/2 gain
	 * - NRF_SAADC_GAIN1 - no gain
	 * - NRF_SAADC_GAIN2 - 2x gain
	 * - NRF_SAADC_GAIN4 - 4x gain
	 *
	 * The default setting is NRF_SAADC_REFERENCE_VDD4, NRF_SAADC_GAIN1_4.
	 * Another common setting is NRF_SAADC_REFERENCE_INTERNAL, NRF_SAADC_GAIN1_6.
	 */
	ADCDMAGen3 &withReferenceGain(nrf_saadc_reference_t reference, nrf_saadc_gain_t gain) { channelConfig.reference = reference; channelConfig.gain = gain; return *this; };

	/**
	 * @brief Gets the voltage reference
	 *
	 * See withReferenceGain() for values.
	 */
	nrf_saadc_reference_t getReference() const { return channelConfig.reference; };

	/**
	 * @brief Get the gain
	 *
	 * See withReferenceGain() for values.
	 */
	nrf_saadc_gain_t getGain() const { return channelConfig.gain; };

	/**
	 * @brief Get the acquisition time
	 *
	 * @param acqTime The acquisition time value
	 *
	 * - NRF_SAADC_ACQTIME_3US - 3 microseconds
	 * - NRF_SAADC_ACQTIME_5US - 5 microseconds
	 * - NRF_SAADC_ACQTIME_10US - 10 microseconds (default)
	 * - NRF_SAADC_ACQTIME_15US - 15 microseconds
	 * - NRF_SAADC_ACQTIME_20US - 20 microseconds
	 * - NRF_SAADC_ACQTIME_40US - 40 microseconds
	 *
	 * Longer acquisition times allows for higher resistance inputs, but lower the maximum sampling frequency
	 */
	ADCDMAGen3 &withAcqTime(nrf_saadc_acqtime_t acqTime) { channelConfig.acq_time = acqTime; return *this; };

	/**
	 * @brief Get the acquisition time setting
	 *
	 * See withAcqTime() for values.
	 */
	nrf_saadc_acqtime_t getAcqTime() const { return channelConfig.acq_time; };


	/**
	 * @brief Sample a single pin
	 *
	 * @param pin The pin. Typically A0 - A5 on Gen 3, except for the B Series SoM which also supports A6 and A7.
	 *
	 * Calling this more than once replaces the previously sampled pin.
	 * If withSampleMultiplePins was previously called, calling this will sample only this pin.
	 *
	 * To sample multiple pins sequentially, use withSampleMultiplePins();
	 */
	ADCDMAGen3 &withSamplePin(pin_t pin) { pinList.clear(); pinList.push_back(pin); return *this; };

	/**
	 * @brief Sample multiple pins
	 *
	 * @param pins List of pins
	 *
	 * To use this, pass the list of pins within {}. For example:
	 *
	 * ```
	 * adc.withSampleMultiplePins( { A0, A1, A2, A3 } );
	 * ```
	 *
	 * Calling this replaces the previous list of pins to sample.
	 * If withSamplePin() was previously called, that pin will no longer be sampled (unless it's in this list of pins).
	 *
	 * To sample a single pin you can pass one pin in the {} or use withSamplePin().
	 */
	ADCDMAGen3 &withSampleMultiplePins(std::initializer_list<pin_t> pins) { this->pinList = pins; return *this; };


	/**
	 * @brief Set the timer CC channel. The default is NRF_TIMER_CC_CHANNEL0.
	 *
	 * You probably won't need to change this value. If you do, set it before calling init().
	 */
	ADCDMAGen3 &withTimerCcChannel(nrf_timer_cc_channel_t timerCcChannel) { this->timerCcChannel = timerCcChannel; return *this; };

	/**
	 * @brief Gets the timer CC channel
	 */
	nrf_timer_cc_channel_t getTimerCcChannel() const { return timerCcChannel; };

	/**
	 * @brief Set the timer short mask. The default is NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK.
	 *
	 * You probably won't need to change this value. If you do, set it before calling init().
	 */
	ADCDMAGen3 &withTimerShortMask(nrf_timer_short_mask_t timerShortMask) { this->timerShortMask = timerShortMask; return *this; };

	/**
	 * @brief Get the timer short mask
	 */
	nrf_timer_short_mask_t getTimerShortMask() const { return timerShortMask; };

	/**
	 * @brief Enable continuous mode. This is the default.
	 *
	 * In continuous mode, the ADC is sampled continuously until stop() is called. Each full buffer
	 * triggers the bufferCallback.
	 *
	 * The opposite of continuous mode is one shot mode, enabled using withOneShotMode().
	 */
	ADCDMAGen3 &withContinuousMode(bool continuousMode = true) { this->continuousMode = continuousMode; return *this; };

	/**
	 * @brief Enable one-shot mode (the opposite of continuous mode)
	 *
	 * In one-shot mode, the buffer is filled once then sampling stops. The ADC is left configured.
	 * You can resume sampling again using start().
	 */
	ADCDMAGen3 &withOneShotMode() { this->continuousMode = false; return *this; };

	/**
	 * @brief Sets the buffer callback function
	 *
	 * @param bufferCallback The function to call when a buffer is filled
	 *
	 * The bufferCallback function must have this prototype:
	 *
	 *   void bufferCallback(nrf_saadc_value_t *buf, size_t size);
	 *
	 * The buf is not used after this call completes, so you can modify it in place if desired for efficiency, but you
	 * must not rely on the contents after you return from the callback because it will be reused and filled with new
	 * data.
	 *
	 * Note that the buffer callback is called from an interrupt context so you must take care in what
	 * functions you call from it. In particular:
	 *
	 * - No allocating of memory
	 * - No long-running code
	 * - No blocking functions
	 * - Avoid all Particle.* functions like Particle.publish
	 * - Beware of using things like Serial.print that are not thread-safe
	 */
	ADCDMAGen3 &withBufferCallback(std::function<void(nrf_saadc_value_t *buf, size_t size)> bufferCallback) { this->bufferCallback = bufferCallback; return *this; };




	/**
	 * @brief Convert a Particle Pin A0 - A7 into the underlying ADC channel
	 *
	 * Particle  ADC   nRF    pin_p                 notes
	 * Pin       Chan  Pin
	 *           0     P0.02  NRF_SAADC_INPUT_AIN0  A7 on SoMs
	 * A0        1     P0.03  NRF_SAADC_INPUT_AIN1
	 * A1        2     P0.04  NRF_SAADC_INPUT_AIN2
	 *           3     P0.05  NRF_SAADC_INPUT_AIN3  A6 on SoMs, BAT on Argon and Xenon
	 * A2        4     P0.28  NRF_SAADC_INPUT_AIN4
	 * A3        5     P0.29  NRF_SAADC_INPUT_AIN5
	 * A4        6     P0.30  NRF_SAADC_INPUT_AIN6
	 * A5        7     P0.31  NRF_SAADC_INPUT_AIN7
	 */
	nrf_saadc_input_t particlePinToAIN(pin_t pin) const;

	/**
	 * @brief Attaches the timer interrupt to user code for the specified hardwareTimer
	 *
	 * Calls attachInterruptDirect() based on hardwareTimer.p_reg
	 */
	void attachTimerInterrupt();

	/**
	 * @brief Detaches the timer interrupt for the specified hardwareTimer
	 *
	 * Calls detachInterrupt() based on hardwareTimer.p_reg
	 */
	void detachTimerInterrupt();

	/**
	 * @brief Frees allocated buffer pools (if they were allocated and not passed in)
	 */
	void freeBufferPoolIfNecessary();

protected:
	/**
	 * @brief ADC callback function
	 *
	 * This is called from adcCallbackStatic when an ADC event occurs. It calls bufferCallback
	 * if defined on NRFX_SAADC_EVT_DONE.
	 */
	void adcCallback(nrfx_saadc_evt_t const *event);

	/**
	 * @brief Timer callback function
	 *
	 * This is the callback function passed to nrfx_timer_init. It doesn't currently
	 * do anything useful as the timer event is handled by PPI not code.
	 */
	static void timerCallbackStatic(nrf_timer_event_t eventType, void* context);

	/**
	 * @brief ADC callback function
	 *
	 * This is the callback function passed to nrfx_saadc_init. It calls addCallback (not static)
	 * using the instance static member to find the class instance (there can only be one).
	 */
	static void adcCallbackStatic(nrfx_saadc_evt_t const *event);

	/**
	 * @brief ADC callback function used by restoreDefaults
	 */
	static void adcEmptyCallbackStatic(nrfx_saadc_evt_t const *event);

	/**
	 * @brief Sample frequency in Hertz (samples per second)
	 *
	 * Set using withSampleFreqHz(). You must set this before calling init().
	 */
	size_t sampleFreqHz = 16000;

	/**
	 * @brief The hardware timer to use
	 *
	 * Set using withHardwareTimer(). You must set this before calling init().
	 */
	nrfx_timer_t hardwareTimer = NRFX_TIMER_INSTANCE(4);

	/**
	 * @brief SAADC configuration
	 *
	 * Note: we override settings in NRFX_SAADC_DEFAULT_CONFIG in the constructor for this
	 * class. Use with withXXX() methods to change fields before calling init().
	 */
	nrfx_saadc_config_t saadcConfig = NRFX_SAADC_DEFAULT_CONFIG;

	/**
	 * @brief SAADC channel configuration
	 *
	 * Note: we override settings in NRFX_SAADC_DEFAULT_CHANNEL_CONFIG_SE in the constructor for this
	 * class. Use with withXXX() methods to change fields before calling init(). The pin is updated
	 * before use (multiple times if using multi-pin mode).
	 *
	 * You must set this before calling init().
	 */
	nrf_saadc_channel_config_t channelConfig = NRFX_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN1);

	/**
	 * @brief Timer configuration
	 *
	 * You normally don't need to override these settings. You must set this before calling init().
	 */
	nrfx_timer_config_t timerConfig = NRFX_TIMER_DEFAULT_CONFIG;

	/**
	 * @brief Timer CC channel
	 *
	 * You normally don't need to override this setting, but if you do, use
	 * withTimerCcChannel() and withTimerShortMask().
	 *
	 * Default value is NRF_TIMER_CC_CHANNEL0. You must set this before calling init().
	 */
    nrf_timer_cc_channel_t timerCcChannel = NRF_TIMER_CC_CHANNEL0;

	/**
	 * @brief Timer Short Mask
	 *
	 * You normally don't need to override this setting, but if you do, use
	 * withTimerCcChannel() and withTimerShortMask().
	 *
	 * Default value is NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK. You must set this before calling init().
	 */
	nrf_timer_short_mask_t timerShortMask = NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK;

	/**
	 * @brief List of pins to sample
	 *
	 * Set using withSamplePin() or withSampleMultiplePins(). You must set this before calling init().
	 */
	std::vector<pin_t> pinList;

	/**
	 * @brief Buffer size.
	 *
	 * Set using withSingleBuffer() or withDoubleBuffer().  You must set this before calling init().
	 */
	size_t bufferSize = 512;

	/**
	 * @brief Number of buffers (1 or 2)
	 *
	 * Set using withSingleBuffer() or withDoubleBuffer().  You must set this before calling init().
	 */
	size_t poolSize = 2;

	/**
	 * @brief Pointers to the buffers. The number of non-zero entries depends on poolSize.
	 *
	 * Set using withSingleBuffer() or withDoubleBuffer().  You must set this before calling init().
	 */
	nrf_saadc_value_t *bufferPool[2] = { 0, 0 };

	/**
	 * @brief Whether to delete[] the buffers when calling freeBufferPoolIfNecessary()
	 *
	 * Set using withSingleBuffer() or withDoubleBuffer(). You should not modify this member.
	 */
	bool freeBufferPool = true;

	/**
	 * @brief State constants for how much of the initialization completed.
	 */
	enum class SetupState {
		NOT_INITIALIZED,
		ATTACH_INTERRUPT,
		SAADC_INIT,
		TIMER_INIT,
		PPI_ALLOC,
		COMPLETE
	};

	/**
	 * @brief State constants for how far the initialization completed.
	 *
	 * Set during init() and used during uninit().
	 */
	SetupState setupState = SetupState::NOT_INITIALIZED;

	/**
	 * @brief The PPI channel to use
	 *
	 * DeviceOS uses PPI channel
	 */
	nrf_ppi_channel_t  ppiChannel = NRF_PPI_CHANNEL0;

	/**
	 * @brief Whether the sampling is one-shot or continuous
	 *
	 * The default is continuous. You should always use double buffers when using continuous mode.
	 * Use withOneShotMode() to set this to false.
	 */
	bool continuousMode = true;

	/**
	 * @param Callback function that is called when a buffer is filled
	 *
	 * Note that this callback is called in an interrupt context so you need to be careful
	 * about what you do in it. Set it using withBufferCallback().
	 */
	std::function<void(nrf_saadc_value_t *buf, size_t size)> bufferCallback = 0;

	/**
	 * @param Global instance of this class
	 *
	 * Since there is only one SAADC engine on the nRF52, you can only have one instance
	 * of this class. It is stored in this variable. It's used from the static callbacks
	 * to find the singleton instance of this class.
	 */
	static ADCDMAGen3 *instance;

};

#endif /* __ADCDMAGEN3_RK_H */
