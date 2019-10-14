// This provides the functionality for Apollo3 devices.

#if defined(AM_PART_APOLLO3)

#include <ap3_types.h>
#include <am_hal_gpio.h>

#include "Adafruit_NeoPixel.h"

// The timing method used to control the NeoPixels
//#define PIN_METHOD_CTIMER_PWM
#define PIN_METHOD_FLASH_DELAY
//#define PIN_METHOD_FAST_GPIO

#if defined(PIN_METHOD_CTIMER_PWM)
#include <am_mcu_apollo.h>
#include <am_util.h>
#include <am_hal_clkgen.h>

// Note - Keep these PWM_CLK* defines in sync
#define PWM_CLK                     AM_HAL_CTIMER_HFRC_12MHZ
#define PWM_CLK_HZ                  12000000

extern "C" void process_next_pixel(void);
void apollo3ConfigureTimer(ap3_gpio_pad_t pad);

// Number of the timer to use for LED control
static uint32_t timerNumber;
// Segment of the timer to use for LED control
static uint32_t timerSegment;
// Timer interrupt to use for LED control
static uint32_t interrupt;

// Number of clock cycles for the total (high + low) LED control period
static uint32_t periodCycles;
// Number of high clock cycles to represent a 0
static uint32_t hiCycles0;
// Number of high clock cycles to represent a 1
static uint32_t hiCycles1;
#endif // PIN_METHOD_CTIMER_PWM

#if defined(PIN_METHOD_FLASH_DELAY)
#include <am_hal_flash.h>
#define PWM_CLK_HZ    48000000
#endif // PIN_METHOD_FLASH_DELAY

static uint8_t *ptr, *end, p, bitMask;

/*!
  @brief   Unset the NeoPixel output pad number.
  @param   pad  Apollo3 pad number
*/
void Adafruit_NeoPixel::apollo3UnsetPad(ap3_gpio_pad_t pad) {
#if defined(PIN_METHOD_CTIMER_PWM)
  // Disable interrupts for the Timer we are using on this board.
  am_hal_ctimer_int_disable(interrupt);
  NVIC_DisableIRQ(CTIMER_IRQn);
#elif defined(PIN_METHOD_FLASH_DELAY) || defined(PIN_METHOD_FAST_GPIO)
  // Unconfigure the pad for Fast GPIO.
  am_hal_gpio_fastgpio_disable(pad);
#endif
}

/*!
  @brief   Set the NeoPixel output pad number.
  @param   pad  Apollo3 pad number
*/
void Adafruit_NeoPixel::apollo3SetPad(ap3_gpio_pad_t pad) {
#if defined(PIN_METHOD_CTIMER_PWM)
  apollo3ConfigureTimer(pad);
#elif defined(PIN_METHOD_FLASH_DELAY) || defined(PIN_METHOD_FAST_GPIO)
  // Configure the pad to be used for Fast GPIO.
  am_hal_gpio_fastgpio_disable(pad);
  am_hal_gpio_fastgpio_clr(pad);

  am_hal_gpio_fast_pinconfig((uint64_t)0x1 << pad,
                             g_AM_HAL_GPIO_OUTPUT, 0);
  // uint32_t ui32Ret = am_hal_gpio_fast_pinconfig((uint64_t)0x1 << pad,
  //                                               g_AM_HAL_GPIO_OUTPUT, 0);
  // if (ui32Ret) {
  //   am_util_stdio_printf(
  //     "Error returned from am_hal_gpio_fast_pinconfig() = .\n", ui32Ret);
  // }
#endif
}

/*!
  @brief   Transmit pixel data in RAM to NeoPixels.
  @note    The current design is a quick hack and should be replaced with
           a more robust timing mechanism.
*/
void Adafruit_NeoPixel::apollo3Show(
  ap3_gpio_pad_t pad, uint8_t *pixels, uint32_t numBytes, boolean is800KHz) {

  ::ptr     =  pixels;
  ::end     =  ::ptr + numBytes;
  ::p       = *::ptr++;
  ::bitMask =  0x80;

#if defined(PIN_METHOD_CTIMER_PWM)
  // current system clock frequency in Hz
#ifdef NEO_KHZ400 // 800 KHz check needed only if 400 KHz support enabled
  if(is800KHz) {
#endif // NEO_KHZ400
    ::periodCycles = (uint32_t)(PWM_CLK_HZ * 0.00000125); // 1.25 uS period
    ::hiCycles0 = (uint32_t)round(PWM_CLK_HZ * 0.0000004); // 0 = 0.4 uS hi
    ::hiCycles1 = (uint32_t)round(PWM_CLK_HZ * 0.0000008); // 1 = 0.8 uS hi
#ifdef NEO_KHZ400
  } else { // 400 KHz bitstream
    ::periodCycles = (uint32_t)(PWM_CLK_HZ * 0.00000250); // 2.5 uS period
    ::hiCycles0 = (uint32_t)round(PWM_CLK_HZ * 0.0000008); // 0 = 0.8 uS hi
    ::hiCycles1 = (uint32_t)round(PWM_CLK_HZ * 0.0000016); // 1 = 1.6 uS hi
  }
#endif // NEO_KHZ400
  process_next_pixel();
// PIN_METHOD_CTIMER_PWM

#elif defined(PIN_METHOD_FLASH_DELAY)
  uint32_t delayCycles;
  // disable interrupts
  am_hal_interrupt_master_disable();
#ifdef NEO_KHZ400 // 800 KHz check needed only if 400 KHz support enabled
  if(is800KHz) {
#endif
    delayCycles = (uint32_t)round((PWM_CLK_HZ * 0.0000004) / 3); // 0 = 0.4 uS hi;
#ifdef NEO_KHZ400
  } else { // 400 KHz bitstream
    // NOTE - This timing may need to be tweaked
    delayCycles = (uint32_t)round((PWM_CLK_HZ * 0.0000008) / 3); // 0 = 0.8 uS hi;
  }
#endif // NEO_KHZ400
  for(;;) {
    am_hal_gpio_fastgpio_set(pad);
    if(::p & ::bitMask) {
      am_hal_flash_delay(delayCycles);
      am_hal_gpio_fastgpio_clr(pad);
    } else {
      am_hal_gpio_fastgpio_clr(pad);
      am_hal_flash_delay(delayCycles);
    }
    if (!(::bitMask >>= 1)) {
      if(::ptr >= ::end) break;
      ::p       = *::ptr++;
      ::bitMask = 0x80;
    }
  }
  // re-enable interrupts
  am_hal_interrupt_master_enable();
// PIN_METHOD_FLASH_DELAY

#elif defined(PIN_METHOD_FAST_GPIO)
  // Note - The timings used below are based on the Arduino Zero,
  //   Gemma/Trinket M0 code.

  // disable interrupts
  am_hal_interrupt_master_disable();

#ifdef NEO_KHZ400 // 800 KHz check needed only if 400 KHz support enabled
  if(is800KHz) {
#endif
    for(;;) {
      am_hal_gpio_fastgpio_set(pad);
      //asm("nop; nop; nop; nop; nop; nop; nop; nop;");
      asm("nop; nop; nop; nop; nop; nop; nop; nop; nop; nop; nop; nop;");
      if(::p & ::bitMask) {
        asm("nop; nop; nop; nop; nop; nop; nop; nop;"
            "nop; nop; nop; nop; nop; nop; nop; nop;"
            "nop; nop; nop; nop;");
        am_hal_gpio_fastgpio_clr(pad);
      } else {
        am_hal_gpio_fastgpio_clr(pad);
        asm("nop; nop; nop; nop; nop; nop; nop; nop;"
            "nop; nop; nop; nop; nop; nop; nop; nop;"
            "nop; nop; nop; nop;");
      }
      if(::bitMask >>= 1) {
	asm("nop; nop; nop; nop; nop; nop; nop; nop; nop;");
      } else {
        if(::ptr >= ::end) break;
        ::p       = *::ptr++;
        ::bitMask = 0x80;
      }
    }
#ifdef NEO_KHZ400
  } else { // 400 KHz bitstream
    // NOTE - These timings may need to be tweaked
    for(;;) {
      am_hal_gpio_fastgpio_set(pad);
      //asm("nop; nop; nop; nop; nop; nop; nop; nop; nop; nop; nop;");
      asm("nop; nop; nop; nop; nop; nop; nop; nop; nop; nop; nop; nop; nop; nop; nop;");
      if(::p & ::bitMask) {
        asm("nop; nop; nop; nop; nop; nop; nop; nop;"
            "nop; nop; nop; nop; nop; nop; nop; nop;"
            "nop; nop; nop; nop; nop; nop; nop; nop;"
            "nop; nop; nop;");
        am_hal_gpio_fastgpio_clr(pad);
      } else {
        am_hal_gpio_fastgpio_clr(pad);
        asm("nop; nop; nop; nop; nop; nop; nop; nop;"
            "nop; nop; nop; nop; nop; nop; nop; nop;"
            "nop; nop; nop; nop; nop; nop; nop; nop;"
            "nop; nop; nop;");
      }
      asm("nop; nop; nop; nop; nop; nop; nop; nop;"
          "nop; nop; nop; nop; nop; nop; nop; nop;"
          "nop; nop; nop; nop; nop; nop; nop; nop;"
          "nop; nop; nop; nop; nop; nop; nop; nop;");
      if(::bitMask >>= 1) {
        asm("nop; nop; nop; nop; nop; nop; nop;");
      } else {
        if(::ptr >= ::end) break;
        ::p       = *::ptr++;
        ::bitMask = 0x80;
      }
    }
  }
#endif // NEO_KHZ400
  // re-enable interrupts
  am_hal_interrupt_master_enable();
// PIN_METHOD_FAST_GPIO

#endif // PIN_METHOD_CTIMER_PWM
}

#if defined(PIN_METHOD_CTIMER_PWM)
/*!
  @brief Set the ctimer variables properly for the specified pad
  @param pad that will be used for the LED signal
  @return None.
*/
void apollo3PadToTimer(ap3_gpio_pad_t pad) {
  // Pad (FNCSEL) ctimer output signal Output Selection (REG_CTIMER_INCFG)
  //                2      3      4      5      6      7
  // PAD4 (6)  CT17 A4OUT2 B7OUT  A4OUT  A1OUT2 A6OUT2 A7OUT2
  // PAD5 (7)  CT8  A2OUT  A3OUT2 A4OUT2 B6OUT  A6OUT2 A7OUT2
  // PAD6 (5)  CT10 B2OUT  B3OUT2 B4OUT2 A6OUT  A6OUT2 A7OUT2
  // PAD7 (7)  CT19 B4OUT2 A2OUT  B4OUT  B1OUT2 A6OUT2 A7OUT2
  // PAD11 (2) CT31 B7OUT2 A6OUT  B7OUT  B3OUT2 A6OUT2 A7OUT2
  // PAD12 (2) CT0  A0OUT  B2OUT2 A5OUT2 A6OUT  A6OUT2 A7OUT2
  // PAD13 (2) CT2  B0OUT  B1OUT2 B6OUT2 A7OUT  A6OUT2 A7OUT2
  // PAD18 (2) CT4  A1OUT  A2OUT2 A5OUT2 B5OUT  A6OUT2 A7OUT2
  // PAD19 (2) CT6  B1OUT  A1OUT  B5OUT2 B7OUT  A6OUT2 A7OUT2
  // PAD22 (2) CT12 A3OUT  B1OUT  B0OUT2 B6OUT2 A6OUT2 A7OUT2
  // PAD23 (2) CT14 B3OUT  B1OUT  B7OUT2 A7OUT  A6OUT2 A7OUT2
  // PAD24 (5) CT21 A5OUT2 A1OUT  B5OUT  A0OUT2 A6OUT2 A7OUT2
  // PAD25 (2) CT1  A0OUT2 A0OUT  A5OUT  B7OUT2 A6OUT2 A7OUT2
  // PAD26 (2) CT3  B0OUT2 B0OUT  A1OUT  A6OUT  A6OUT2 A7OUT2
  // PAD27 (2) CT5  A1OUT2 A1OUT  B6OUT  A7OUT  A6OUT2 A7OUT2
  // PAD28 (2) CT7  B1OUT2 B1OUT  B5OUT  A7OUT  A6OUT2 A7OUT2
  // PAD29 (2) CT9  A2OUT2 A2OUT  A4OUT  B0OUT  A6OUT2 A7OUT2
  // PAD30 (2) CT11 B2OUT2 B2OUT  B4OUT  B5OUT2 A6OUT2 A7OUT2
  // PAD31 (2) CT13 A3OUT2 A3OUT  A6OUT  B4OUT2 A6OUT2 A7OUT2
  // PAD32 (2) CT15 B3OUT2 B3OUT  A7OUT  A4OUT2 A6OUT2 A7OUT2
  // PAD33 (6) CT23 B5OUT2 A7OUT  A5OUT  B0OUT2 A6OUT2 A7OUT2
  // PAD35 (5) CT27 B6OUT2 A1OUT  B6OUT  B2OUT2 A6OUT2 A7OUT2
  // PAD37 (7) CT29 B5OUT2 A1OUT  A7OUT  A3OUT2 A6OUT2 A7OUT2
  // PAD39 (2) CT25 B4OUT2 B2OUT  A6OUT  A2OUT2 A6OUT2 A7OUT2
  // PAD42 (2) CT16 A4OUT  A0OUT  A0OUT2 B3OUT2 A6OUT2 A7OUT2
  // PAD43 (2) CT18 B4OUT  B0OUT  A0OUT  A3OUT2 A6OUT2 A7OUT2
  // PAD44 (2) CT20 A5OUT  A1OUT  A1OUT2 B2OUT2 A6OUT2 A7OUT2
  // PAD45 (2) CT22 B5OUT  B1OUT  A6OUT  A2OUT2 A6OUT2 A7OUT2
  // PAD46 (2) CT24 A6OUT  A2OUT  A1OUT  B1OUT2 A6OUT2 A7OUT2
  // PAD47 (2) CT26 B6OUT  B2OUT  A5OUT  A1OUT2 A6OUT2 A7OUT2
  // PAD48 (2) CT28 A7OUTB A3OUT  A5OUT2 B0OUT2 A6OUT2 A7OUT2
  // PAD49 (2) CT30 B7OUT  B3OUT  A4OUT2 A0OUT2 A6OUT2 A7OUT2

  // TODO: Come up with a better way to set the
  //   timer number, segment, and interrupt

  switch(pad) {
  case(4): {
    // PAD4 (6) CT17  A4OUT2 B7OUT  A4OUT  A1OUT2 A6OUT2 A7OUT2
    timerNumber = 4;
    timerSegment = AM_HAL_CTIMER_TIMERA;
    interrupt = AM_HAL_CTIMER_INT_TIMERA4C0;
  }break;
  case(7): {
    // PAD7 (7) CT19  B4OUT2 A2OUT  B4OUT  B1OUT2 A6OUT2 A7OUT2
    timerNumber = 2;
    timerSegment = AM_HAL_CTIMER_TIMERA;
    interrupt = AM_HAL_CTIMER_INT_TIMERA2C0;
  }break;
  default: {
    // unable to configure timer for the specified pad
    return;
  }
  }
}

/*!
  @brief Configure the ctimer for the specified pad
  @param pad that will be used for the LED signal
  @return None.
*/
void apollo3ConfigureTimer(ap3_gpio_pad_t pad) {
  // Set the timer variables
  apollo3PadToTimer(pad);

  // Set the clock frequency.
  am_hal_clkgen_control(AM_HAL_CLKGEN_CONTROL_SYSCLK_MAX, 0);

  // Configure the output pin.
  am_hal_ctimer_output_config(timerNumber,
			      timerSegment,
			      pad,
			      AM_HAL_CTIMER_OUTPUT_NORMAL,
			      AM_HAL_GPIO_PIN_DRIVESTRENGTH_2MA);

  // Configure a timer to drive the LED.
  am_hal_ctimer_config_single(timerNumber,
			      timerSegment,
			      (AM_HAL_CTIMER_FN_PWM_ONCE    |
			       PWM_CLK                      |
			       AM_HAL_CTIMER_INT_ENABLE) );

  // Enable interrupts for the Timer we are using on this board.
  am_hal_ctimer_int_enable(interrupt);
  NVIC_EnableIRQ(CTIMER_IRQn);
  am_hal_interrupt_master_enable();
}

/*!
  @brief Process the pixels to be "shown".
  @note This function sets up a PWM timer to output a pixel. When the
        interrupt is triggered, the ISR will call this function again to
        output the next pixel. When there are no more pixels to process,
        this function returns without scheduling another timer.
  @return None.
*/
extern "C"
void process_next_pixel(void)
{
  // Break the cycle by returning if there are no more pixels to process.
  if(ptr > end) return;

  // Set up initial timer periods.
  am_hal_ctimer_period_set(timerNumber, timerSegment, periodCycles,
			   (p & bitMask) ? hiCycles1 : hiCycles0);

  // Reset and start the timer.
  am_hal_ctimer_clear(timerNumber, timerSegment);
  am_hal_ctimer_start(timerNumber, timerSegment);

  // Move to the next pixel for after the interrupt fires.
  if (!(bitMask >>= 1)) {
    p       = *ptr++;
    bitMask = 0x80;
  }
}

/*!
  @brief Timer Interrupt Serive Routine (ISR)
*/
extern "C" void
am_ctimer_isr(void)
{
  // Clear the interrupt that got us here.
  am_hal_ctimer_int_clear(interrupt);

  // Continue processing pixels.
  process_next_pixel();
}
#endif // PIN_METHOD_CTIMER_PWM

#endif // AM_PART_APOLLO3
