#include <config/stm32plus.h>
#include <config/gpio.h>
#include <config/timing.h>
#include <config/string.h>
#include <config/usart.h>
#include <timing/MicrosecondDelay.h>
#include "config/spi.h"

#include <stdlib.h>


using namespace stm32plus;

/**
 * Measures a period of the mirror rotation. Timer1 is used for capturing,
 * PA08 has to be connected to the pulse source
 */
class PeriodCounter {
public:
    PeriodCounter() {
        // ToDo: setup base timer clock in such a way that integral prescaler
        // is used
        input_timer.setPrescaler(input_timer.getClock() / 10000000 - 1);
        input_timer.TimerInterruptEventSender.insertSubscriber(
            TimerInterruptEventSourceSlot::bind(this, &PeriodCounter::on_timer_interrupt));
        input_timer.enableInterrupts(TIM_IT_CC1);
        input_timer.enablePeripheral();
    }

    /**
     * Waits for pulse on input.
     */
    void wait_for_pulse() {
        while (!triggered);
        triggered = false;
    }

    /**
     * Computes the length of the pulse in timer base unit
     */
    uint16_t get_pulse() const {
        return captures[1] - captures[0];
    }

    void on_timer_interrupt(TimerEventType tet, uint8_t timer_number) {
        if (timer_number == 1 && tet == TimerEventType::EVENT_COMPARE1) {
            captures[0] = captures[1];
            captures[1] = input_timer.getCapture();
            triggered = true;
        }
    }
private:
    typedef Timer1<
        Timer1InternalClockFeature,         // we'll need this for the frequency calculation
        TimerChannel1Feature<               // we're going to use channel 3
          TimerChannelICRisingEdgeFeature,  // rising edge trigger
          TimerChannelICDirectTiFeature,    // direct connection
          TimerChannelICPreScaler1Feature,  // prescaler of 1
          TimerChannelICFilterFeature<0>    // no filter
        >,
        Timer1InterruptFeature,           // we want to use interrupts
        Timer1GpioFeature<                // we want to read something from GPIO
          TIMER_REMAP_NONE,               // the GPIO input will not be remapped
          TIM1_CH1_IN                     // we will read channel 3 from GPIO PB0
        >
      > MyInputTimer;

    MyInputTimer input_timer;
    volatile uint16_t captures[2];
    volatile uint16_t period;
    volatile bool triggered;
};

/**
 * This class represents a single bit in word
 */
template <class BASE>
class Bit {
public:
    Bit(BASE& word, uint8_t position) : word(word), mask(1 << position) {}

    Bit& operator=(bool state) {
        if (state)
            word |= mask;
        else
            word &= ~mask;
    }

    operator bool() const {
        return word & mask;
    }
private:
    BASE& word;
    BASE  mask;
};

/**
 * Represents bitmask of given size and enables single bit access
 */
template <uint32_t BITS, class Base = uint32_t>
class Bitmask {
public:
    Bitmask() : data() { }

    /**
     * Gets single bit, if the position is outside of bounds, result is false
     */
    bool get(uint32_t position) const {
        div_t result = div(position, sizeof(Base) * 8);
        if (result.quot > SIZE)
            return false;
        return data[result.quot] & (1 << result.rem);
    }

    /**
     * Sets a single bit, if the position is outside of bounds, bitmask is not
     * modified
     */
    void set(uint32_t position, bool value = true) {
        div_t result = div(position, sizeof(Base) * 8);
        if (result.quot > SIZE)
            return;
        if (value)
            data[result.quot] |= 1 << result.rem;
        else
            data[result.quot] &= ~(1 << result.rem);
    }

    /**
     * Sets each Base element to given value
     */
    void set_to_base_value(Base value) {
        memset(data, value, SIZE);
    }

    /**
     * Sets each byte to given value
     */
    void set_to_byte_value(uint8_t value) {
        Base res = 0;
        for (uint8_t i = 0; i != sizeof(Base); i++) {
            res |= res << (i * 8);
        }
        set_to_base_value(res);
    }

    /**
     * Returns raw value of the bitmask
     */
    void* get_raw() {
        return data;
    }

    Bit<Base> operator[](uint32_t position) {
        div_t result = div(position, sizeof(Base) * 8);
        if (result.quot > SIZE)
            return Bit<Base>(garbage, 0);
        return Bit<Base>(data[result.quot], result.rem);
    }
private:
    static const uint32_t SIZE = ((BITS + sizeof(Base) - 1) / sizeof(Base)) * sizeof(Base);
    Base data[SIZE];
    Base garbage; // Garbage for unauthorized accesses via []
};

/**
 * This class contains double buffered pattern. The original pattern is preserved
 * and during sending another re-calibrated pattern is calculated.
 * Spi1 is used for sending, so PA07 is used as output for laser driving
 */
template <uint32_t PATTERN_SIZE, uint32_t BUFFER_SIZE>
class Pattern {
public:
private:
    Bitmask<PATTERN_SIZE> base_pattern;
    Bitmask<BUFFER_SIZE>  recalibrated_patterns[2];
};

GpioC<DefaultDigitalOutputFeature<13>, DefaultDigitalOutputFeature<14> > pc;
const int STEP = 13;
const int DIR = 14;

using Spi1Config = Spi1<>;

const int BUFF_SIZE = 250;
uint8_t buffer[BUFF_SIZE];

int main()
{
    Nvic::initialise();
    MillisecondTimer::initialise();

    Usart1<> usart1(115200);
    UsartPollingOutputStream outputStream(usart1);

    PeriodCounter counter;

    Spi1<>::Parameters spi1_conf;
    spi1_conf.spi_baudRatePrescaler = SPI_BaudRatePrescaler_16;
    spi1_conf.spi_direction = SPI_Direction_1Line_Tx;

    unsigned char byte = 0xFF;
    const int parts = 50;
    for (int i = 0; i != parts; i++) {
        byte = ~byte;
        for (int j = 0; j != BUFF_SIZE / parts; j++) {
            buffer[i * BUFF_SIZE / parts + j] = byte;
        }
    }
    
    for (int i = 0; i != 12; i++)
        buffer[i] = 0xff;
    
    int count = 0;
    bool state = false;
    pc[DIR].reset();
	for (;;) {
    	GpioA<DefaultDigitalOutputFeature<7>> pa;
    	pa[7].set();
    	counter.wait_for_pulse();
    	//Delay::delay(20);
    	/*for (int i = 0; i != 30; i++) {
        	pa[7].reset();
        	Delay::delay(delay);
        	pa[7].set();
        	Delay::delay(delay);
    	}*/
    	Spi1<> spi(spi1_conf);
    	spi.send(buffer, BUFF_SIZE);
		/*pc[STEP].set();
		MillisecondTimer::delay(1);
        pc[STEP].reset();
		MillisecondTimer::delay(1);
    	count++;
    	if (count == 16 * 200) {
        	count = 0;
        	if (state)
            	pc[DIR].reset();
        	else
            	pc[DIR].set();
        	state = !state;
    	}
    	char buf[15];
    	StringUtil::modp_uitoa10(period, buf);
    	outputStream << buf << " us\n";*/
	}
}
