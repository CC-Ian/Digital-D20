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

#define NUM_READINGS 6

// BQ21080 BMS Registers
#define BMS_ADDRESS 0x6A
#define BMS_REG_ICHG_CTRL 0x04

// Neopixel configs
#define NUM_LEDS 16
#define LED_DATA_PIN PA4
#define LED_BRIGHTNESS 128
#define LED_ENABLE PA6


// IMU Structure
struct Acceleration {
  int16_t x;
  int16_t y;
  int16_t z;
};


/// @brief IMU Reference Vectors. 0th index corresponds to the 1 face. It's inverse, the 20.
int32_t referenceVectors[][3] = {
  {112, -5392, 14652},    // 1-20
  {9696, 8276, -9236},    // 2-19
  {-5960, -14796, -356},  // 3-18
  {-6512, 14676, 408},    // 4-17
  {9376, 9048, 9792},     // 5-16
  {-15384, 212, -5200},   // 6-15
  {9308, -9392, 8884},    // 7-14
  {-464, -6476, -14828},  // 8-13
  {-15200, 1672, 5480},   // 9-12
  {8676, -10664, -9188}   // 10-11
};


Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUM_LEDS, LED_DATA_PIN, NEO_GRB + NEO_KHZ800);

// Cyan, Turquoise, Blue, Violet. Pulled from FastLED. I think the violet is wrong? Will have to crosscheck.
// TODO: Cross-check LED Colors here.
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


/// @brief Calculate the mean of an input array.
/// @param data The array to compute the mean of.
/// @return The Mean.
int calculateMean(int16_t data[]) {
  int64_t sum = 0;

  for (int i = 0; i < NUM_READINGS; i++) {
    sum += data[i];
  }

  return sum / NUM_READINGS;
}

/// @brief Computes the standard deviation of an array of data.
/// @param data The Array to compute the standard deviation of. (1D)
/// @return Standard Deviation as an integer.
int calculateStandardDeviation(int16_t data[]) {
  // Initial Declarations. Need Size and Mean to find the stdev.
  int variance = 0;
  int mean = calculateMean(data);

  // Loop through each element of the array and add (value - mean)^2
  // I think this is more efficient than using actual square functions or the carrot.
  // It also might just get compiled out.
  for (int i = 0; i < NUM_READINGS; i++) {
    variance += (data[i] - mean) * (data[i] - mean);
  }

  // Return the stdev.
  return sqrt(variance / NUM_READINGS);
}

/// @brief Performs the dot product on two input vectors (3D).  
/// @brief https://en.wikipedia.org/wiki/Dot_product
/// @param A First Vector (3D).
/// @param B Second Vector (3D).
/// @return The Dot product of the two vectors
int64_t getDotProduct(int32_t A[3], int32_t B[3]) {
  return (A[0] * B[0]) + (A[1] * B[1]) + (A[2] * B[2]);
}

/// @brief Calculates the length of an input vector
/// @brief https://en.wikipedia.org/wiki/Magnitude_(mathematics)#Euclidean_vector_space
/// @param A Input Vector.
/// @return Magnitude of the input vector A.
int64_t getLength(int32_t A[3]) {
  return sqrt((A[0] * A[0]) + (A[1] * A[1]) + (A[2] * A[2]));
}

/// @brief Reads the acceleration data registers of the IMU.
/// @return Structure of X, Y, Z data.
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



/// @brief Compute the rolled value using an input vector.
/// @param measurement The input vector to compare against the references.
/// @return The number rolled [1-20] or 0 if no matching vector was found.
int computeRoll(Acceleration measurement) {
  // Create a vector from the input structure.
  int32_t measureVector[3];
  measureVector[0] = measurement.x;
  measureVector[1] = measurement.y;
  measureVector[2] = measurement.z;

  // Length of the input vector. No need to re-compute every loop.
  int64_t measureLength = getLength(measureVector); 

  for (int i = 0; i < sizeof(referenceVectors) / sizeof(referenceVectors[0]); i++) {
    int64_t referenceLength = getLength(referenceVectors[i]);
    int64_t dotProduct = getDotProduct(measureVector, referenceVectors[i]);
    float cosine = float(dotProduct) / float(referenceLength * measureLength);

    // Accept close enough values as our number. This does have some deadzone, but that is preferable to noting the wrong roll.
    // Magic number. I found that the values intersect somewhere around 0.95 I think. Don't want to deal with intersecting values, so we get this instead.
    if (cosine >= 0.975) {
      return (1 + i);     // makes 1 to 10.
    } else if (cosine <= -0.975) {
      return (20 - i);    // makes 20 to 11
    }
  }

  // Invalid roll. Die cocked or non-level surface.
  return 0;
}




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

/// @brief Initialize the IMU to low power, low ODR mode, and enable wakeup interrupt.
void initIMU() {
  // TODO: May wish to replace wakeup with tap interrupt. Was more responsive during testing.
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
}

/// @brief Init BMS to 250mA charge current.  
/// @brief https://www.ti.com/lit/ds/symlink/bq21080.pdf?ts=1712321276802&ref_url=https%253A%252F%252Fwww.ti.com%252Fproduct%252FBQ21080#%5B%7B%22num%22%3A437%2C%22gen%22%3A0%7D%2C%7B%22name%22%3A%22XYZ%22%7D%2Cnull%2C125.775%2Cnull%5D
void initBMS() {
  Wire.beginTransmission(BMS_ADDRESS);
  Wire.write(BMS_REG_ICHG_CTRL);
  Wire.write(0x30); // Enable charging, charge current 210mA.
  Wire.endTransmission();
}


/// @brief Main loop. Enters this point on startup. Or when woken from shutdown.
void setup() {
  
  HAL_Init();

  SystemClock_Config();

  SystemPower_Config();

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(LED_ENABLE, OUTPUT);
  digitalWrite(LED_ENABLE, HIGH);

  HAL_Delay(100);

  // Initialize IMU in wake mode. Higher current draw, for more readings per second.
  Wire.setSCL(PB6); // Set Clock pin.
  Wire.setSDA(PB7); // Set Data Pin.
  Wire.begin();

  initIMU();
  initBMS();
  

  // TODO: Need a way to hold the chip in wake mode during charge. This facilitates the ability to actually speak with the IC through GDB and an stlink.

  // If returning from shutdown:
  if (READ_REG(TAMP->BKP31R) == 1) {
    WRITE_REG(TAMP->BKP31R, 0x0);

    // Enable the LEDs
    digitalWrite(LED_ENABLE, LOW);

    // Ctrl 1 to 0x64. 100Hz, Low Power. Expects ~5uA.
    // Kick on IMU in higher ORD for wake mode.
    Wire.beginTransmission(LIS2DW12_I2C_ADDRESS);
    Wire.write(LIS2DW12_REG_CTRL1);
    Wire.write(0b01010000); // ODR 100Hz, LP Mode 12/14, LP Mode 1
    Wire.endTransmission();

    HAL_Delay(5);

    // Init Arrays for mean and standard deviation.
    int16_t x_vals[NUM_READINGS];
    int16_t y_vals[NUM_READINGS];
    int16_t z_vals[NUM_READINGS];

    // Fill with 10 initial measurements.
    for (int i = 0; i < NUM_READINGS; ++i) {
      Acceleration reading = ReadAccelerometer();
      x_vals[i] = reading.x;
      y_vals[i] = reading.y;
      z_vals[i] = reading.z;
      HAL_Delay(25); // Adjust delay as necessary to suit your needs
    }

    // Begin LEDs now to allow some settle time while we do math.
    pixels.begin();
    pixels.clear();

    // Calculate mean and standard deviation
    int stdDevX = calculateStandardDeviation(x_vals);
    int stdDevY = calculateStandardDeviation(y_vals);
    int stdDevZ = calculateStandardDeviation(z_vals);

    int stDevMean = (stdDevX + stdDevY + stdDevZ) / 3;

    // Monitor standard deviation of the IMU axes to continue the LED animation.
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
      stdDevX = calculateStandardDeviation(x_vals);
      stdDevY = calculateStandardDeviation(y_vals);
      stdDevZ = calculateStandardDeviation(z_vals);

      // Re-compute mean standard deviation. Expected to fall to around 50 when the die has come to a stop.
      stDevMean = (stdDevX + stdDevY + stdDevZ) / 3;

      // Fade out the LED, then start the next loop.
      int fadeDuration = random(40, 100); // Random fade duration between 80ms and 150ms
      fadeAllPixels(fadeDuration);

      IMUIndex = (IMUIndex + 1) % 10;
    }
    
    // Die has come to a stop. Take 10 measurements and average for final value.
    for (int i = 0; i < NUM_READINGS; ++i) {
      Acceleration reading = ReadAccelerometer();
      x_vals[i] = reading.x;
      y_vals[i] = reading.y;
      z_vals[i] = reading.z;
      HAL_Delay(10); // Adjust delay as necessary to suit your needs
    }

    // Compute roll and color.
    int roll = computeRoll({calculateMean(x_vals), calculateMean(y_vals), calculateMean(z_vals)}) - 1;
    if (roll > -1) {
      // If the roll is valid, compute the color to display, then pulse it in and out.
      uint8_t red = (255 - round(13.42 * roll));
      uint8_t green = round(13.42 * roll);
      uint32_t rollColor = pixels.Color(red, green, 0);

      for (int i = 0; i <= 25; i++) {
        int brightness = map(i, 0, 25, 0, LED_BRIGHTNESS);
        pixels.fill(rollColor, 0, NUM_LEDS);
        pixels.setBrightness(brightness);
        pixels.show();
        HAL_Delay(17);
      }
    
      // Hold
      HAL_Delay(1000);
    
      for (int i = 0; i <= 25; i++) {
        int brightness = map(i, 0, 25, LED_BRIGHTNESS, 0);
        pixels.fill(rollColor, 0, NUM_LEDS);
        pixels.setBrightness(brightness);
        pixels.show();
        HAL_Delay(17);
      }
    } else {
      for (int iter = 0; iter < 3; iter++) {
        for (int i = 0; i <= 25; i++) {
          int brightness = map(i, 0, 25, 0, LED_BRIGHTNESS);
          pixels.fill(65535, 0, NUM_LEDS);
          pixels.setBrightness(brightness);
          pixels.show();
          HAL_Delay(17);
        }
      
        // Hold
        HAL_Delay(250);
      
        for (int i = 0; i <= 25; i++) {
          int brightness = map(i, 0, 25, LED_BRIGHTNESS, 0);
          pixels.fill(65535, 0, NUM_LEDS);
          pixels.setBrightness(brightness);
          pixels.show();
          HAL_Delay(17);
        }
      }
    }

    HAL_Delay(2);
    pixels.clear();
    pixels.setBrightness(0);

    HAL_Delay(50);


    // Cleanup section.

    // Ctrl 1 to 0x10. 1.6Hz, Low Power.
    Wire.beginTransmission(LIS2DW12_I2C_ADDRESS);
    Wire.write(LIS2DW12_REG_CTRL1);
    Wire.write(0b00010000); // ODR 12.5Hz, LP Mode 12/14, LP Mode 1
    Wire.endTransmission();
    Wire.end();

    HAL_Delay(50);

  }

  pinMode(LED_ENABLE, INPUT_FLOATING);

  HAL_Delay(1000);

  // Everything past this point is used for shutdown mode config
  // TODO: Make this into it's own function. Self contained so it can be called.
  // Do I need the ability to selectively enable/disable wakup pins? Maybe disable the IMU during charge? Is it even worth it for 150nA?

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  __HAL_RCC_GPIOA_CLK_ENABLE();
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    
  // HAL_PWREx_EnableGPIOPullDown(PWR_GPIO_A, PWR_GPIO_BIT_0);
  // HAL_PWREx_EnablePullUpPullDownConfig();

  /* Disable used wakeup source: PWR_WAKEUP_PIN1 */
  HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN1);

  /* Clear all related wakeup flags */
  __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
    
  /* Enable wakeup pin WKUP1 */
  HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN1_HIGH);

  /* Set TAMP back-up register TAMP_BKP31R to indicate
       later on that system has entered shutdown mode  */
  WRITE_REG( TAMP->BKP31R, 0x1 );

  /* Enter the Shutdown mode */
  HAL_PWREx_EnterSHUTDOWNMode();

}


/// @brief Required for the arduino framework. Code never reaches this point.
void loop() {
  // code should never reach this point.
  // All actual math and operations are performed within the setup function, as the chip always
  // re-enters at setup when it's using shutdown mode.
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
