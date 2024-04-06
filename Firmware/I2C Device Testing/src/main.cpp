#include <Arduino.h>
#include <Wire.h>

// LIS2DW12 register addresses
#define LIS2DW12_I2C_ADDRESS 0x19
#define LIS2DW12_REG_CTRL1 0x20
#define LIS2DW12_REG_CTRL4_INT1_PAD_CTRL 0x23
#define LIS2DW12_REG_CTRL7 0x3F
#define LIS2DW12_REG_WAKE_UP_THS 0x34
#define LIS2DW12_REG_WAKE_UP_DUR 0x35

// BQ21080 BMS Registers
#define BMS_ADDRESS 0x6A
#define BMS_REG_ICHG_CTRL 0x04

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



/// @brief Calculate the mean of an input array.
/// @param data The array to compute the mean of.
/// @return The Mean.
int calculateMean(int16_t data[]) {
  int size = sizeof(data) - 1;
  int64_t sum = 0;

  for (int i = 0; i < size; i++) {
    sum += data[i];
  }

  return sum / size;
}

/// @brief Computes the standard deviation of an array of data.
/// @param data The Array to compute the standard deviation of. (1D)
/// @return Standard Deviation as an integer.
int calculateStandardDeviation(int16_t data[]) {
  // Initial Declarations. Need Size and Mean to find the stdev.
  int variance = 0;
  int size = sizeof(data) - 1;
  int mean = calculateMean(data);

  // Loop through each element of the array and add (value - mean)^2
  // I think this is more efficient than using actual square functions or the carrot.
  // It also might just get compiled out.
  for (int i = 0; i < size; i++) {
    variance += (data[i] - mean) * (data[i] - mean);
  }
    Serial.println(size);


  // Return the stdev.
  return sqrt(variance / size);
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


/// @brief Initialize the IMU to low power, low ODR mode, and enable wakeup interrupt.
void initIMU() {
  // TODO: May wish to replace wakeup with tap interrupt. Was more responsive during testing.
  // Ctrl 1 to 0x10. 1.6Hz, Low Power.
  Wire.beginTransmission(LIS2DW12_I2C_ADDRESS);
  Wire.write(LIS2DW12_REG_CTRL1);
  Wire.write(0x10);
  Wire.endTransmission();
  delay(10);

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

template <typename T, size_t N>
size_t lengthOf(T (&array)[N]) {
    return N;
}


void setup() {
  Wire.begin();
  Serial.begin(9600);
  Serial.println("Begin");

  initIMU();
  Serial.println("IMU INIT");
  initBMS();
  Serial.println("BMS INIT");


  // Ctrl 1 to 0x64. 100Hz, Low Power. Expects ~5uA.
  // Kick on IMU in higher ORD for wake mode.
  Wire.beginTransmission(LIS2DW12_I2C_ADDRESS);
  Wire.write(LIS2DW12_REG_CTRL1);
  Wire.write(0b01010000); // ODR 100Hz, LP Mode 12/14, LP Mode 1
  Wire.endTransmission();
  Serial.println("IMU 100Hz ODR");


  delay(25);

  // Init Arrays for mean and standard deviation.
  int16_t x_vals[10];
  int16_t y_vals[10];
  int16_t z_vals[10];

  Serial.println(lengthOf(x_vals));

  // Fill with 10 initial measurements.
  for (int i = 0; i < lengthOf(x_vals); ++i) {
    Acceleration reading = ReadAccelerometer();
    x_vals[i] = reading.x;
    y_vals[i] = reading.y;
    z_vals[i] = reading.z;
    delay(25); // Adjust delay as necessary to suit your needs
  }

  Serial.print("X vals size: ");
  int num_elements = sizeof(x_vals) / sizeof(x_vals[0]);
  Serial.println(num_elements);
  for (int i = 0; i < num_elements; i++) {
    Serial.println(x_vals[i]);
  }
  Serial.println("done");


  // Calculate mean and standard deviation
  int stdDevX = calculateStandardDeviation(x_vals);
  int stdDevY = calculateStandardDeviation(y_vals);
  int stdDevZ = calculateStandardDeviation(z_vals);

  int stDevMean = (stdDevX + stdDevY + stdDevZ) / 3;

    // Monitor standard deviation of the IMU axes to continue the LED animation.
  uint8_t IMUIndex = 0;
  while (stDevMean > 200) {
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

    IMUIndex = (IMUIndex + 1) % 10;
    delay(random(40, 100));
  }


}


void loop() {}