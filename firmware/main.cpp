#include <config/stm32plus.h>
#include <config/gpio.h>
#include <config/timing.h>
#include <config/string.h>
#include <config/usart.h>
#include <timing/MicrosecondDelay.h>
#include "config/spi.h"

#include <stdlib.h>

#include "libs/fatfs/ff.h"

using namespace stm32plus;

// bitband type
typedef volatile uint32_t * bitband_t;

// addresses for bit banding
#define BITBAND_SRAM_REF               		(0x20000000)
#define BITBAND_SRAM_BASE              		(0x22000000)
#define BITBAND_PERIPH_REF               	(0x40000000)
#define BITBAND_PERIPH_BASE              	(0x42000000)

#define BITBAND_SRAM(address, bit)     ((bitband_t)(BITBAND_SRAM_BASE +   \
		(((uint32_t)address) - BITBAND_SRAM_REF) * 32 + (bit) * 4))

#define BITBAND_PERIPH(address, bit)   ((bitband_t)(BITBAND_PERIPH_BASE + \
		(((uint32_t)address) - BITBAND_PERIPH_REF) * 32 + (bit) * 4))

/**
 * Measures a period of the mirror rotation. Timer1 is used for capturing,
 * PA08 has to be connected to the pulse source
 */
class PeriodCounter {
public:
    PeriodCounter() {
        input_timer.setPrescaler(16); // The same prescaler which is used for SPI
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
     * Clears pulse flag
     */
    void clear_pulse_flag() {
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
class Bit {
public:
    Bit(void* word, uint8_t position) : bit(BITBAND_SRAM(word, position)) {}

    Bit& operator=(bool state) {
        *bit = state;
        return *this;
    }

    operator bool() const {
        return *bit;
    }

    /**
     * Moves to the next bit
     */
    void next() {
        bit++;
    }

    /**
     * Moves to the previous bit
     */
    void previous() {
        bit--;
    }
    
    /**
     * Moves by n bits
     */
    template <class T>
    void skip(T inc) {
        bit += inc;
    }
private:
    bitband_t bit;
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
        return *(BITBAND_SRAM(&data[result.quot], result.rem));
    }

    /**
     * Sets a single bit, if the position is outside of bounds, bitmask is not
     * modified
     */
    void set(uint32_t position, bool value = true) {
        div_t result = div(position, sizeof(Base) * 8);
        if (result.quot > SIZE)
            return;
        *(BITBAND_SRAM(&data[result.quot], result.rem)) = value;
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
    uint8_t* raw() {
        return (uint8_t*)data;
    }

    /**
     * Returns a raw bit-pointer to array
     */
    bitband_t bits() {
        return *BITBAND_SRAM(data, 0);
    }

    /**
     * Returns first bit
     */
    Bit front() {
        return Bit(data, 0);
    }

    /**
     * Returns number of bytes of raw data
     */
    constexpr uint32_t raw_size() const {
        return SIZE * sizeof(Base);
    }

    /**
     * Returns size in bytes of Base type
     */
    constexpr uint8_t base_size() const {
        return sizeof(Base);
    }

    /**
     * Returns number of bits in the bitmask
     */
    constexpr uint32_t size() const {
        return BITS;
    }

    Bit operator[](uint32_t position) {
        div_t result = div(position, sizeof(Base) * 8);
        if (result.quot > SIZE)
            return Bit(&garbage, 0);
        return Bit(&data[result.quot], result.rem);
    }
private:
    static constexpr uint32_t SIZE = (BITS + sizeof(Base) * 8 - 1) / (sizeof(Base) * 8);
    Base data[SIZE];
    Base garbage; // Garbage for unauthorized accesses via []
};

/**
 * This class contains double buffered pattern. The original pattern is preserved
 * and during sending another re-calibrated pattern is calculated.
 * Spi1 is used for sending, so PA07 is used as output for laser driving
 */
template <uint32_t PATTERN_SIZE, uint32_t BUFFER_SIZE, class PATTERN_BASE = uint32_t>
class Pattern {
public:
    Pattern(uint32_t pattern_width, uint32_t pattern_start)
        : index(0), pattern_start(pattern_start), pattern_width(pattern_width)
    {
        spi_params.spi_baudRatePrescaler = SPI_BaudRatePrescaler_16;
        spi_params.spi_direction = SPI_Direction_1Line_Tx;
        spi_params.spi_firstBit = SPI_FirstBit_LSB;
    }

    void set_pattern_width(uint32_t width) {
        pattern_width = width;
    }

    void set_pattern_start(uint32_t start) {
        pattern_start = start;
    }

    Bitmask<PATTERN_SIZE, PATTERN_BASE>& get_patern() {
        return base_pattern;
    }

    void draw_pattern(uint16_t last_period) {
        // Init SPI with DMA
        Spi1<> spi(spi_params);
        Spi1TxDmaChannel<
            SpiDmaWriterFeature<
                Spi1PeripheralTraits, DMA_Priority_High> > dmaSender;

        dmaSender.beginWrite(recalibrated_patterns[index].raw(), pattern_sizes[index]);

        // Prepare new image
        /*if(last_period < 6000)
            recalibrate_pattern(last_period);*/

        dmaSender.waitUntilComplete();
        spi.waitForIdle();

        // Put pin high
        //GpioA<DefaultDigitalOutputFeature<7>> pa;
        //pa[7].set();
    }

    void recalibrate_pattern(uint16_t period) {
        if (index++ == 1)
            index = 0;

        // Calculate width of the pattern in bytes (round it up)
        pattern_sizes[index] = ((((pattern_start + pattern_width) * period) / PERCENT) + 7) / 8;

        auto& pat = recalibrated_patterns[index];

        // Prepare header
        const uint32_t header_bits = period * pattern_start / PERCENT;
        const div_t result = div(header_bits, 8 * sizeof(uint32_t));
        uint32_t* raw_header = (uint32_t*)pat.raw();
        // Set the whole part
        memset(raw_header, 0x0, result.quot * sizeof(uint32_t));
        // Set the partial part
        Bit bit(raw_header + result.quot, 0);
        for (uint8_t i = 0; i != result.rem; i++) {
            bit = false;
            bit.next();
        }

        // Recalibrate the pattern
        const uint32_t pattern_bits = period * pattern_width / PERCENT;
        const uint32_t target = base_pattern.size() << 16;
        const uint32_t increment = (base_pattern.size() << 16) / pattern_bits;
        Bit source(base_pattern.raw(), 0);
        uint16_t last = 0;
        for (uint32_t index = 0; index <= target; index += increment) {
            uint16_t pos = index >> 16;
            source.skip(pos - last);
            last = pos;
            bit = (bool)source;
            bit.next();
        }
    }
private:
    Bitmask<PATTERN_SIZE, PATTERN_BASE> base_pattern;
    Bitmask<BUFFER_SIZE, PATTERN_BASE>  recalibrated_patterns[2];
    uint32_t pattern_sizes[2];
    uint8_t index;

    uint32_t pattern_start; // Start position in percents of one reflection
    uint32_t pattern_width; // Width in percents of one reflection; 100% = 100 000
    
    Spi1<>::Parameters spi_params;
    
    static constexpr uint32_t PERCENT = 100000;
};

GpioC<DefaultDigitalOutputFeature<13>, DefaultDigitalOutputFeature<14> > pc;
const int STEP_PIN = 13;
const int DIR_PIN = 14;

const int PATTERN_SIZE = 3000;

PeriodCounter counter;
Pattern<PATTERN_SIZE, 4000> pattern(60000, 6000);

extern "C" {
    void disk_timerproc(void);
}

void delay_callback(TimerEventType, unsigned char) {
    disk_timerproc();
}

int main()
{
    Nvic::initialise();
    MillisecondTimer::initialise();

    Usart1<> usart1(115200);
    UsartPollingOutputStream outputStream(usart1);
    
    Timer2<
        Timer2InternalClockFeature,       // the timer clock source is APB2 (APB on the F0)
        Timer2InterruptFeature            // gain access to interrupt functionality
      > timer;
    
    timer.TimerInterruptEventSender.insertSubscriber(
          TimerInterruptEventSourceSlot::bind(delay_callback)
        );
    timer.setTimeBaseByFrequency(10000, 1000, TIM_CounterMode_CenterAligned3);
    timer.clearPendingInterruptsFlag(TIM_IT_Update);
    timer.enableInterrupts(TIM_IT_Update);
    timer.enablePeripheral();
    
    while (true) {
        FATFS FatFs;
        FIL fil;
        uint32_t total, free;
        
        outputStream << "Trying to open drive\n";
        if (f_mount(0, &FatFs) == FR_OK) {
            outputStream << "Device opened!\n";
            if (f_open(&fil, "test.txt", FA_OPEN_ALWAYS | FA_READ) == FR_OK) {
                outputStream << "File opened!\n";
                char buffer[128];
                f_gets(buffer, 128, &fil);
                outputStream << buffer;
            }
            else {
                outputStream  << "Cannot open file!\n";
            }
            f_close(&fil);
        }
        else {
            outputStream << "Cannot open drive!\n";
        }
        f_mount(0, 0);
        while (true);
    }

    const unsigned int PARTS = 100;
    bool state = true;
    auto p = pattern.get_patern().front();
    for (unsigned int i = 0, index = 0; i != PARTS; i++) {
        state = !state;
        for (unsigned int j = 0; j != pattern.get_patern().size() / PARTS; j++, index++, p.next()) {
            p = state;
        }
    }
    
    //auto pp = pattern.get_patern().raw();
    /*for (int i = 0; i != 128; i++)
        p[i] = true;*/
    
    /*unsigned int word = 0xFFFFFFFF;
    auto pp = pattern.get_patern().raw();
    for (int i = 0, index = 0; i != PARTS; i++) {
        word = ~word;
        for (int j = 0; j != 3000 / PARTS; j++, index++) {
           *(BITBAND_SRAM(pp + index / 8, index % 8)) = word;
        }
    }*/
    
    /*for (int i = 0; i != p.size(); i++) {
            if (p.get(i))
                outputStream << "1";
            else
                outputStream << "0";
    }
    outputStream << "\n";*/
    pattern.recalibrate_pattern(3240);
    pattern.recalibrate_pattern(3240);
    
    pc[DIR_PIN].reset();
	for (;;) {
    	GpioA<DefaultDigitalOutputFeature<7>> pa;
    	pa[7].reset();
        //counter.wait_for_pulse();
    	uint16_t pulse = counter.get_pulse();
    	pattern.draw_pattern(pulse);
    	counter.clear_pulse_flag();
	}
}
