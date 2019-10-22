#include <Wire.h>
#define HAND 4    // Sets Digital 4 pin as hand sensor
#define FOREARM 3 // Sets Digital 3 pin as forearm sensor
#define BACK 2    // Sets Digital 2 pin as back sensor

const int MPU = 0x68; // MPU6050 I2C addresses

struct SensorDataStructure {
  float AccX;  // Accelerometer x-axis value for MPU6050
  float AccY;  // Accelerometer y-axis value for MPU6050
  float AccZ;  // Accelerometer z-axis value for MPU6050
  float GyroX; // Gyrometer x-axis value for MPU6050
  float GyroY; // Gyrometer y-axis value for MPU6050
  float GyroZ; // Gyrometer z-axis value for MPU6050
} HandSensorData, ForearmSensorData, BackSensorData, SensorData;

void setup() {
  pinMode(HAND, OUTPUT);        // Sets hand digital pin as output pin
  pinMode(FOREARM, OUTPUT);     // Sets forearm digital pin as output pin
  pinMode(BACK, OUTPUT);        // Sets back digital pin as output pin
  
  Wire.begin();                 // Initiates I2C communication
  Wire.beginTransmission(MPU);  // Begins communication with the MPU
  Wire.write(0x6B);             // Access the power management register
  Wire.write(0x00);             // Wakes up the MPU
  Wire.endTransmission(true);   // Communication done
  Serial.begin(9600);           // Initialize serial port baud rate to 9600 
}

void ReadMPUValues() {
  Wire.beginTransmission(MPU);      // Begins communication with the MPU
  Wire.write(0x3B);                 // Register 0x3B upper 8 bits of x-axis acceleration data
  Wire.endTransmission(false);      // End communication
  Wire.requestFrom(MPU, 14, true);  // Request 14 registers

  SensorData.AccX  = Wire.read() << 8 | Wire.read(); // Reads in raw x-axis acceleration data
  SensorData.AccY  = Wire.read() << 8 | Wire.read(); // Reads in raw y-axis acceleration data
  SensorData.AccZ  = Wire.read() << 8 | Wire.read(); // Reads in raw z-axis acceleration data
  Wire.read(); Wire.read();                          // Reads in raw temperature data
  SensorData.GyroX = Wire.read() << 8 | Wire.read(); // Reads in raw x-axis gyroscope data
  SensorData.GyroY = Wire.read() << 8 | Wire.read(); // Reads in raw y-axis gyroscope data
  SensorData.GyroZ = Wire.read() << 8 | Wire.read(); // Reads in raw z-axis gyroscope data
}

void PrintMPUValues(SensorDataStructure SDS) {
  Serial.print("Accelerometer Values: [x = "); Serial.print(SDS.AccX);
  Serial.print(", y = "); Serial.print( SDS.AccY);
  Serial.print(", z = "); Serial.print(SDS.AccZ); Serial.println("]"); 
  Serial.print("Gyrorometer Values:   [x = "); Serial.print(SDS.GyroX);
  Serial.print(", y = "); Serial.print(SDS.GyroY);
  Serial.print(", z = "); Serial.print(SDS.GyroZ); Serial.println("]");
  Serial.println();
}

void CallibrateMPUValues() {
  SensorData.AccX = SensorData.AccX/16384.0;
  SensorData.AccY = SensorData.AccY/16384.0;
  SensorData.AccZ = SensorData.AccZ/16384.0;
  SensorData.GyroX = SensorData.GyroX/131.0;
  SensorData.GyroY = SensorData.GyroY/131.0;
  SensorData.GyroZ = SensorData.GyroZ/131.0;
}

void UpdateMPUSensorData() {
  CallibrateMPUValues();
  if (!digitalRead(HAND)) {
    HandSensorData = SensorData;
    Serial.println("Hand MPU6050 Readings");
  } else if (!digitalRead(FOREARM)) {
    ForearmSensorData = SensorData;
    Serial.println("Forearm MPU6050 Readings");
  } else if (!digitalRead(BACK)) {
    BackSensorData = SensorData;
    Serial.println("Back MPU6050 Readings");
  }
}

void DeactivateSensors() {
  digitalWrite(HAND, HIGH);     // Deactivates hand sensor
  digitalWrite(FOREARM, HIGH);  // Deactivates forearm sensor
  digitalWrite(BACK, HIGH);     // Deactivates back sensor
}

void ExecuteSensor(int value, SensorDataStructure sds) {
  DeactivateSensors();
  digitalWrite(value, LOW);  // Activates sensor
  delay(100);
  ReadMPUValues();
  UpdateMPUSensorData();
  PrintMPUValues(sds);
}

void loop() {
  ExecuteSensor(HAND, HandSensorData);
  ExecuteSensor(FOREARM, ForearmSensorData);
  ExecuteSensor(BACK, BackSensorData);
}
