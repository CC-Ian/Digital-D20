#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <Wire.h>

// LIS2DW12 register addresses
#define LIS2DW12_I2C_ADDRESS 0x19

#define LIS2DW12_REG_CTRL1 0x20
#define LIS2DW12_REG_CTRL4_INT1_PAD_CTRL 0x23
#define LIS2DW12_REG_CTRL7 0x3F

#define LIS2DW12_REG_WAKE_UP_THS 0x34
#define LIS2DW12_REG_WAKE_UP_DUR 0x35

// Neopixel configs
#define NUM_LEDS 16
#define LED_DATA_PIN PA4
#define LED_BRIGHTNESS 128


// IMU Structure
struct Acceleration {
  int16_t x;
  int16_t y;
  int16_t z;
};

Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUM_LEDS, LED_DATA_PIN, NEO_GRB + NEO_KHZ800);

// Cyan, Turquoise, Blue, Violet. Pulled from FastLED.
uint32_t colorPalette[] = {65535, 4251856, 255, 15631086};

// Prototypes
void SystemClock_Config(void);
void SystemPower_Config(void);

/*
* Setup runs from the beginning every time we wake up. Need to check the wakeup source when woken from shutdown.
* INIT IMU to 12.5HZ low power
* Begin Lightning animation
* Reading from IMU to ensure there's movement. May need to chain these? display a diode, then read IMU. Need averaging.
* At some point, motion stops.
* Read gravity on IMU
* determine number face up
* pluse colour
* Re-configure IMU for motion detection
* shutdown
*/

// Accelerometer related functions.
// Function to calculate the mean of an array of values
int calculateMean(int16_t data[], int size) {
  int sum = 0;
  for (int i = 0; i < size; i++) {
    sum += data[i];
  }
  return sum / size;
}

// Function to calculate the standard deviation of an array of values
int calculateStandardDeviation(int16_t data[], int size, int mean) {
  int variance = 0;
  for (int i = 0; i < size; i++) {
    variance += (data[i] - mean) * (data[i] - mean);
  }
  variance /= size;
  return sqrt(variance);
}

Acceleration ReadAccelerometer() {
  Acceleration accel;

  Wire.beginTransmission(0x19); // initiate transmission with LIS2DW12
  Wire.write(0x28); // select register 0x0F to read from
  Wire.endTransmission(false); // false to not release the bus

  // request 6 bytes of data (X, Y, Z for acceleration)
  Wire.requestFrom(0x19, 6); 
  if (Wire.available() >= 6) {
    // combine the two bytes for X value
    accel.x = Wire.read() | (Wire.read() << 8); 
    // do the same for the Y and Z values
    accel.y = Wire.read() | (Wire.read() << 8);
    accel.z = Wire.read() | (Wire.read() << 8);
  }
  return accel;
}


// LED related functions.
// Function to fade out all LEDs over a specified duration
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

void setup() {
  
  HAL_Init();

  SystemClock_Config();

  SystemPower_Config();

  pinMode(LED_BUILTIN, OUTPUT);

  HAL_Delay(100);

  // Initialize IMU in wake mode. Higher current draw, for more readings per second.
  Wire.setSCL(PB6); // 
  Wire.setSDA(PB7); //
  Wire.begin();

  // Ctrl 1 to 0x10. 1.6Hz, Low Power.
  Wire.beginTransmission(LIS2DW12_I2C_ADDRESS);
  Wire.write(LIS2DW12_REG_CTRL1);
  Wire.write(0x10);
  Wire.endTransmission();
  HAL_Delay(10);

  // ctrl 7 to 0x20. Enable Interrupts.
  Wire.beginTransmission(LIS2DW12_I2C_ADDRESS);
  Wire.write(LIS2DW12_REG_CTRL7);
  Wire.write(0x20);
  Wire.endTransmission();

  // Set wakeup duration to zero? (One sample above threshold)
  Wire.beginTransmission(LIS2DW12_I2C_ADDRESS);
  Wire.write(LIS2DW12_REG_WAKE_UP_DUR);
  Wire.write(0x00);
  Wire.endTransmission();

  // A high wakeup threshold so it only kicks on when it's being rolled?
  Wire.beginTransmission(LIS2DW12_I2C_ADDRESS);
  Wire.write(LIS2DW12_REG_WAKE_UP_THS);
  Wire.write(0x08);
  Wire.endTransmission();

  // Set Interrupt1 to pulse wakeup signal.
  Wire.beginTransmission(LIS2DW12_I2C_ADDRESS);
  Wire.write(LIS2DW12_REG_CTRL4_INT1_PAD_CTRL);
  Wire.write(0x20);
  Wire.endTransmission();

  // If returning from shutdown
  if (READ_REG(TAMP->BKP31R) == 1) {
    WRITE_REG(TAMP->BKP31R, 0x0);

    digitalWrite(LED_BUILTIN, 1);

    // Ctrl 1 to 0x64. 100Hz, Low Power. Expects ~5uA.
    // Kick on IMU in higher ORD for wake mode.
    Wire.beginTransmission(LIS2DW12_I2C_ADDRESS);
    Wire.write(LIS2DW12_REG_CTRL1);
    Wire.write(0b01010000); // ODR 100Hz, LP Mode 12/14, LP Mode 1
    Wire.endTransmission();

    delay(25);

    // Init Arrays for mean and standard deviation.
    int16_t x_vals[5];
    int16_t y_vals[5];
    int16_t z_vals[5];

    // Fill with 10 initial measurements
    for (int i = 0; i < 5; ++i) {
      Acceleration reading = ReadAccelerometer();
      x_vals[i] = reading.x;
      y_vals[i] = reading.y;
      z_vals[i] = reading.z;
      delay(25); // Adjust delay as necessary to suit your needs
    }

    // Calculate mean and standard deviation
    int meanX = calculateMean(x_vals, 5);
    int meanY = calculateMean(y_vals, 5);
    int meanZ = calculateMean(z_vals, 5);

    int stdDevX = calculateStandardDeviation(x_vals, 5, meanX);
    int stdDevY = calculateStandardDeviation(y_vals, 5, meanY);
    int stdDevZ = calculateStandardDeviation(z_vals, 5, meanZ);

    int stDevMean = (stdDevX + stdDevY + stdDevZ) / 3;

    pixels.begin();
    pixels.clear();

    uint8_t IMUIndex = 0;
    while (stDevMean > 200) {

      // start of the loop, show the color.
      pixels.setPixelColor(random(NUM_LEDS), colorPalette[random(0, sizeof(colorPalette))]);
      pixels.setPixelColor(random(NUM_LEDS), colorPalette[random(0, sizeof(colorPalette))]);
      pixels.setPixelColor(random(NUM_LEDS), colorPalette[random(0, sizeof(colorPalette))]);
      pixels.setBrightness(LED_BRIGHTNESS);
      pixels.show();

      // Acquire the next data point.
      Acceleration reading = ReadAccelerometer();
      x_vals[IMUIndex] = reading.x;
      y_vals[IMUIndex] = reading.y;
      z_vals[IMUIndex] = reading.z;

      // Calculate mean and standard deviation
      meanX = calculateMean(x_vals, 5);
      meanY = calculateMean(y_vals, 5);
      meanZ = calculateMean(z_vals, 5);

      stdDevX = calculateStandardDeviation(x_vals, 5, meanX);
      stdDevY = calculateStandardDeviation(y_vals, 5, meanY);
      stdDevZ = calculateStandardDeviation(z_vals, 5, meanZ);

      // Re-compute mean standard deviation. Expected to fall to around 50 when the die has come to a stop.
      stDevMean = (stdDevX + stdDevY + stdDevZ) / 3;

      // Fade out the LED, then start the next loop.
      int fadeDuration = random(40, 100); // Random fade duration between 80ms and 150ms
      fadeAllPixels(fadeDuration);

      IMUIndex = (IMUIndex + 1) % 10;
    }

    HAL_Delay(100);

    for (int i = 0; i <= 25; i++) {
      int brightness = map(i, 0, 25, 0, LED_BRIGHTNESS);
      pixels.fill(pixels.Color(255, 0, 0), 0, NUM_LEDS);
      pixels.setBrightness(brightness);
      pixels.show();
      HAL_Delay(17);
    }
  
    // Hold
    HAL_Delay(1000);
  
    for (int i = 0; i <= 25; i++) {
      int brightness = map(i, 0, 25, LED_BRIGHTNESS, 0);
      pixels.fill(pixels.Color(255, 0, 0), 0, NUM_LEDS);
      pixels.setBrightness(brightness);
      pixels.show();
      HAL_Delay(17);
    }

    pixels.clear();
    pixels.setBrightness(0);


    // Cleanup section.

    // Ctrl 1 to 0x10. 1.6Hz, Low Power.
    Wire.beginTransmission(LIS2DW12_I2C_ADDRESS);
    Wire.write(LIS2DW12_REG_CTRL1);
    Wire.write(0b00010000); // ODR 12.5Hz, LP Mode 12/14, LP Mode 1
    Wire.endTransmission();
    Wire.end();

    delay(50);

  }

  HAL_Delay(1000);

  // Everything past this point is used for shutdown mode config

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  __HAL_RCC_GPIOA_CLK_ENABLE();
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    
  // HAL_PWREx_EnableGPIOPullDown(PWR_GPIO_A, PWR_GPIO_BIT_0);
  // HAL_PWREx_EnablePullUpPullDownConfig();

  /* Disable used wakeup source: PWR_WAKEUP_PIN1 */
  HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN4);

  /* Clear all related wakeup flags */
  __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
    
  /* Enable wakeup pin WKUP1 */
  HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN4_HIGH);

  /* Set TAMP back-up register TAMP_BKP31R to indicate
       later on that system has entered shutdown mode  */
  WRITE_REG( TAMP->BKP31R, 0x1 );

  /* Enter the Shutdown mode */
  HAL_PWREx_EnterSHUTDOWNMode();

}

void loop() {
  // code should never reach this point.
}

void SystemClock_Config(void) {
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
    while(1);
  }
}

void SystemPower_Config(void) {
  /* Enable Power Control clock */
  __HAL_RCC_PWR_CLK_ENABLE();
  
  /* Enable writing TAMP back-up registers */
  HAL_PWR_EnableBkUpAccess();

}
