
#include "Stm32Gpio.hxx"
#include "utils/GpioInitializer.hxx"
#include "BlinkerGPIO.hxx"
#include "DummyGPIO.hxx"
#include "PWM.hxx"

// Blue and Gold LEDs
GPIO_PIN(LED_Blue, LedPin, B, 1);
GPIO_PIN(LED_Gold, LedPin, B, 0);
// Switch LEDs
GPIO_PIN(LED_A0, LedPin, A, 0);
GPIO_PIN(LED_A1, LedPin, A, 1);
GPIO_PIN(LED_A2, LedPin, A, 2);
GPIO_PIN(LED_A3, LedPin, A, 3);
GPIO_PIN(LED_A4, LedPin, A, 4);
GPIO_PIN(LED_A5, LedPin, A, 5);
GPIO_PIN(LED_A6, LedPin, A, 6);
GPIO_PIN(LED_A7, LedPin, A, 7);

// Blue and Gold Buttons
GPIO_PIN(BLUE_Btn, GpioInputPU, B, 8);
GPIO_PIN(GOLD_Btn, GpioInputPU, B, 9);

// Chip Select line for SPI1 IF we had an expansion port, we don't so not actually used. PB2 unconnected
GPIO_PIN(EXT_CS, GpioOutputSafeLow, B, 2);
// Latch line assigned to expansion port, we don't, so not used. PB12 unconnected
GPIO_PIN(EXT_LAT, GpioOutputSafeLow, B, 12);
// Latch line on the onboard outputs
GPIO_PIN(OUT_LAT, GpioOutputSafeLow, B, 4);
// Latch line on the onboard inputs
GPIO_PIN(INP_LAT, GpioOutputSafeLow, B, 15);

typedef GpioInitializer<BLUE_Btn_Pin, GOLD_Btn_Pin, //
	OUT_LAT_Pin, INP_LAT_Pin, LED_Blue_Pin, //
	LED_Gold_Pin, LED_A0_Pin, LED_A1_Pin,
	LED_A2_Pin, LED_A3_Pin, LED_A4_Pin,
	LED_A5_Pin, LED_A6_Pin, LED_A7_Pin,
	EXT_CS_Pin, EXT_LAT_Pin, DummyPin>
    GpioInit;

typedef LED_Blue_Pin BLINKER_RAW_Pin;
typedef BLINKER_Pin LED_Blue;

