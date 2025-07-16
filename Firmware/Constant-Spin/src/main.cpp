#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <Wire.h>

// Neopixel configs
#define NUM_LEDS 32
#define LED_DATA_PIN PA7
#define LED_BRIGHTNESS 255
#define LED_ENABLE PA6

// Use adafruit_neopixel to handle the WS2812B RGB LEDs onboard.
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUM_LEDS, LED_DATA_PIN, NEO_GRB + NEO_KHZ800);

// Cyan, Turquoise, Blue, Violet. Pulled from FastLED. I think the violet is wrong? Will have to crosscheck.
// TODO: Cross-check LED Colors here.
uint32_t colorPalette[] = {65535, 4251856, 255, 15631086};

// Prototypes
void SystemClock_Config(void);
void SystemPower_Config(void);


/// @brief Fade all LEDs over the input duration.
/// @param duration Input Duration.
void fadeAllPixels(int duration) {
  // Set brightness each loop?
  // want 60FPS
  // duration in milliseconds
  // need a step every 17ms.
  int numSteps = duration / 17;
  int brightness = pixels.getBrightness();
  int brightnessStep = brightness / numSteps;

  // Perform the brightness step operation
  for (int i = 0; i < numSteps; i++) {
    pixels.setBrightness(brightnessStep);
    brightness -= brightnessStep;
    if (brightness < 0) {
      brightness = 0;
    }
    pixels.show();
    HAL_Delay(17);
  }

  // Clean up. Set strip to black and restore brightness.
  pixels.clear();
  pixels.setBrightness(LED_BRIGHTNESS);
}

/// @brief Setup runs from the beginning every time we wake up. Need to check the wakeup source when woken from shutdown.  
/// @brief INIT IMU to 12.5HZ low power. Init BMS to 250mA charge current.  
/// @brief Begin Lightning animation and read acceleration data into a moving average.  
/// @brief Compute standard deviation of the moving average. When it falls off, we know motion has stopped.  
/// @brief Compute the roll based off gravity and the reference vectors.    
/// @brief Pulse the appropriate color for the roll.  
/// @brief Re-configure IMU and wakeup pins.  
/// @brief Shutdown.  
void setup() {
  
  HAL_Init();

  SystemClock_Config();

  SystemPower_Config();

  // LED stuff. Active Low.
  pinMode(LED_ENABLE, OUTPUT);
  digitalWrite(LED_ENABLE, LOW);
  pixels.begin();
  pixels.clear();

}


/// @brief Required for the arduino framework. Code never reaches this point.
void loop() {
  // start of the loop, show the color.
  pixels.setPixelColor(random(NUM_LEDS), colorPalette[random(0, sizeof(colorPalette))]);
  pixels.setPixelColor(random(NUM_LEDS), colorPalette[random(0, sizeof(colorPalette))]);
  pixels.setPixelColor(random(NUM_LEDS), colorPalette[random(0, sizeof(colorPalette))]);
  pixels.setBrightness(LED_BRIGHTNESS);
  pixels.show();

  // Fade out the LED, then start the next loop.
  int fadeDuration = random(40, 100); // Random fade duration between 80ms and 150ms
  fadeAllPixels(fadeDuration);
}

/// @brief SystemClock Config ripped from a sample st project. Seems to work.  
/// @brief https://github.com/STMicroelectronics/STM32CubeL4/blob/d3b7dcfb72c6c6a046862654ef6e23cbed9935a8/Projects/NUCLEO-L412KB/Examples/PWR/PWR_SHUTDOWN/Src/main.c#L152
void SystemClock_Config(void) {
  // TODO: Double check that this can't be made more efficient. APPNOTE on this.
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};

  /* MSI is enabled after System reset, activate PLL with MSI as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    /* Initialization Error */
    // TODO: Find a way to reset the chip if this occurs? Due to the structure of this, 
    // this will PERMANANTLY lock up the die if this happens.
    while(1);
  }

  
  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;  
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;  
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    /* Initialization Error */
    // TODO: Find a way to reset the chip if this occurs? Due to the structure of this, 
    // this will PERMANANTLY lock up the die if this happens.
    while(1);
  }
}


void SystemPower_Config(void) {
  /* Enable Power Control clock */
  __HAL_RCC_PWR_CLK_ENABLE();
  
  /* Enable writing TAMP back-up registers */
  HAL_PWR_EnableBkUpAccess();

}
