
#include "Stm32Gpio.hxx"
#include "utils/GpioInitializer.hxx"
#include "BlinkerGPIO.hxx"
#include "DummyGPIO.hxx"
#include "PWM.hxx"

// Blue and Gold LEDs
GPIO_PIN(LED_Blue, LedPin, B, 1);
GPIO_PIN(LED_Gold, LedPin, B, 0);

// Blue and Gold Buttons
GPIO_PIN(BLUE_Btn, GpioInputPU, B, 8);
GPIO_PIN(GOLD_Btn, GpioInputPU, B, 9);

// Bell and Relay
GPIO_PIN(CTC_Bell, GpioOutputSafeHigh, A, 1);
GPIO_PIN(CTC_Relay, GpioOutputSafeHigh, A, 2);

// Chip Select line for SPI1 connected Flash
GPIO_PIN(FLASH_CS, GpioOutputSafeLow, A, 4);
// Chip Select line for SPI1 connected FRAM
GPIO_PIN(FRAM_CS, GpioOutputSafeLow, A, 3);
// Latch line on the onboard outputs
GPIO_PIN(OUT_LAT, GpioOutputSafeLow, B, 4);
// Latch line on the onboard inputs
GPIO_PIN(INP_LAT, GpioOutputSafeLow, B, 15);

typedef GpioInitializer<BLUE_Btn_Pin, GOLD_Btn_Pin, //
	CTC_Bell_Pin, CTC_Relay_Pin, OUT_LAT_Pin, //
	INP_LAT_Pin, LED_Blue_Pin, LED_Gold_Pin, //
	FLASH_CS_Pin, FRAM_CS_Pin, DummyPin>
    GpioInit;

typedef LED_Blue_Pin BLINKER_RAW_Pin;
typedef BLINKER_Pin LED_Blue;

