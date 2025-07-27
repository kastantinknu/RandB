#include <Wire.h>
const int MPU = 0x68; // I2C address of the MPU6050
float AccX, AcY, AcZ; // Variables to store accelerometer data
float GyroX, GyroY, GyroZ; // Variables to store gyroscope data
float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ; // Variables to store angles
float roll, pitch, yaw; // Variables to store roll, pitch, and yaw angles
float AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ; // Variables to store errors
float elapsedTime, currentTime, previousTime; // Variables to manage time
int c = 0; // Counter for the number of readings
void setup() {
  Serial.begin(115200);
  Wire.begin(); // Initialize I2C communication
  Wire.beginTransmission(MPU); // Start communication with MPU6050 // MPU6050= 0x68
  Wire.write(0x6B); // PWR_MGMT_1 register // Talk to the register 6B
  Wire.write(0x00); // Wake up the MPU6050 // Make reset - place a 0 into the 6B register to wake it up
  Wire.endTransmission(true); // End the transmission
  calculate_IMU_error(); // Calculate errors for calibration
  delay(20); // Wait for 20 milliseconds
}
void loop() {
    Wire.beginTransmission(MPU); // Start communication with MPU6050
    Wire.write(0x3B); // Starting with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false); // End the transmission but keep the connection open
    Wire.requestFrom(MPU, 6, true); // Read 6 registers total, each axis value is stored in 2 registers
    //For a range of +-2g, we need to divide the raw values by 16384, according to the datasheet
    AccX = (Wire.read() << 8 | Wire.read()) / 16384.0; // Read and convert accelerometer X-axis data // X-axis value
    AccY = (Wire.read() << 8 | Wire.read()) / 16384.0; // Read and convert accelerometer Y-axis data // Y-axis value
    AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0; // Read and convert accelerometer Z-axis data // Z-axis value
    // Calculate Roll and Pitch angles from accelerometer data
    accAngleX = (atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / PI) - 0.58; // AccErrorX ~(0.58) See the calculate_IMU_error() custom function ror more details
    accAngleY = (atan(-1 * AccX / sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180 / PI) + 0.58; // AccErrorY ~(-1.58)
    // === Read gyroscope data === //
    previousTime = currentTime; // Store the previous time // Previous time is stored before the actual time is read
    currentTime = millis(); // Get the current time in milliseconds
    elapsedTime = (currentTime - previousTime) / 1000; // Calculate elapsed time in seconds
    Wire.beginTransmission(MPU); // Start communication with MPU6050
    Wire.write(0x43); // Starting with register 0x43 (GYRO_XOUT_H)
    Wire.endTransmission(false); // End the transmission but keep the connection open
    Wire.requestFrom(MPU, 6, true); // Read 6 registers total, each axis value is stored in 2 registers
    GyroX = (Wire.read() << 8 | Wire.read()) / 131.0; // Read and convert gyroscope X-axis data // X-axis value
    // For a 250deg/s range, we need to divide the raw values by 131, according to the datasheet
    GyroY = (Wire.read() << 8 | Wire.read()) / 131.0; // Read and convert gyroscope Y-axis data // Y-axis value
    GyroZ = (Wire.read() << 8 | Wire.read()) / 131.0; // Read and convert gyroscope Z-axis data // Z-axis value
    // Correct gyroscope data with the calculated errors
    GygoX = GyroX + 0.56; // Correct gyroscope X-axis data // GyroErrorX ~(-0.56)
    GyroY = GyroY - 2; // Correct gyroscope Y-axis data // GyroErrorY ~(2)
    GyroZ = GyroZ + 0.79; // Correct gyroscope Z-axis data // GyroErrorZ ~(-0.8)
    // Currently the raw values are in degrees per second, deg/s, so we need to convert them to angles
    //by multiplying by the elapsed time (s) to get the angle in degrees
    gyroAngleX = gyroAngleX + GygoX * elapsedTime; // Calculate gyroscope X-axis angle // deg/s * s = deg
    gyroAngleY = gyroAngleY + GyroY * elapsedTime; // Calculate gyroscope Y-axis angle // deg/s * s = deg
    yaw = yaw + GyroZ * elapsedTime; 
    // Complementary filter to combine accelerometer and gyroscope angle values
    roll = 0.96 * gyroAngleX + 0.04 * accAngleX // Calculate roll angle
    pitch = 0.96 * gyroAngleY + 0.04 * accAngleY; // Calculate pitch angle

    // Print the angles to the Serial Monitor
    Serial.print(roll);
    Serial.print("/");
    Serial.print(pitch);
    Serial.print("/");
    Serial.print(yaw);
}
void calculate_IMU_error() {
    // We can call this function in the setap section to calculate the accelerometer and gyro data errer.
    // From here we will get the errer values used in the above equations printed on the Serial Monitor.
    // Note that we should place the IMU flat in order to get the proper values, so that we then can the correct values
    // Read accelerometer values 200 times
  while (c < 200) { // Loop to collect data for error calculation
    Wire.beginTransmission(MPU); // Start communication with MPU6050
    Wire.write(0x3B); // Starting with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false); // End the transmission but keep the connection open
    Wire.requestFrom(MPU, 6, true); // Read 6 registers total, each axis value is stored in 2 registers
    AccX = (Wire.read() << 8 | Wire.read()) / 16384.0; // Read and convert accelerometer X-axis data // X-axis value
    AccY = (Wire.read() << 8 | Wire.read()) / 16384.0; // Read and convert accelerometer Y-axis data // Y-axis value
    AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0; // Read and convert accelerometer Z-axis data // Z-axis value  
    // Sum all readings
    AccErrorX = AccErrorX + ((atan((AccY) / sqrt(pow((AccX), 2) + pow((AccZ), 2))) * 180 / PI));
    AccErrorY = AccErrorY + ((atan(-1 * (AccX) / sqrt(pow(AccY, 2) + pow((AccZ), 2))) * 180 / PI));
    c++; // Increment the counter
  }
  // Divide the sums by the number of readings (200) to get the error value
    AccErrorX = AccErrorX / 200; // Initialize the accelerometer X-axis error
    AccErrorY = AccErrorY / 200; // Initialize the accelerometer Y-axis error
    c = 0; // Reset the counter
    // Read gyro values 200 times
    while (c < 200) { // Loop to collect data for gyro error calculation
    Wire.beginTransmission(MPU); // Start communication with MPU6050
    Wire.write(0x43); // Starting with register 0x43 (GYRO_XOUT_H)
    Wire.endTransmission(false); // End the transmission but keep the connection open
    Wire.requestFrom(MPU, 6, true); // Read 6 registers total, each axis value is stored in 2 registers
    GyroX = (Wire.read() << 8 | Wire.read()); // Read and convert gyroscope X-axis data
    GyroY = (Wire.read() << 8 | Wire.read()); // Read and convert gyroscope Y-axis data
    GyroZ = (Wire.read() << 8 | Wire.read()); // Read and convert gyroscope Z-axis data
    // Sum all readings
    GyroErrorX = GyroErrorX + (GyroX / 131.0); // Sum Gyro X-axis data
    GyroErrorY = GyroErrorY + (GyroY / 131.0); // Sum Gyro Y-axis data
    GyroErrorZ = GyroErrorZ + (GyroZ / 131.0); // Sum Gyro Z-axis data
    c++; // Increment the counter
    }
    // Divide the sums by the number of readings (200) to get the error value
    GyroErrorX = GyroErrorX / 200; // Calculate average error for gyroscope X-axis
    GyroErrorY = GyroErrorY / 200; // Calculate average error for gyroscope Y-axis
    GyroErrorZ = GyroErrorZ / 200; // Calculate average error for gyroscope Z-axis
    // Print the calculated errors to the Serial Monitor
    Serial.print("AccErrorX: ");
    Serial.println(AccErrorX); // Print accelerometer X-axis error
    Serial.print("AccErrorY: ");
    Serial.println(AccErrorY); // Print accelerometer Y-axis error
    Serial.print("GyroErrorX: ");
    Serial.println(GyroErrorX); // Print gyroscope X-axis error
    Serial.print("GyroErrorY: ");
    Serial.println(GyroErrorY); // Print gyroscope Y-axis error
    Serial.print("GyroErrorZ: ");
    Serial.println(GyroErrorZ); // Print gyroscope Z-axis error
    delay(2000); // Wait for 2000 milliseconds before the next reading
}
// This code reads data from the MPU6050 sensor, calculates angles using accelerometer and gyroscope data,
// and applies a complementary filter to combine the angles. It also calculates errors for calibration.