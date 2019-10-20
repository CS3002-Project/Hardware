#include <Wire.h>
#define HAND 4        // Sets Digital 4 pin as hand sensor
#define FOREARM 3     // Sets Digital 3 pin as forearm sensor
#define BACK 2        // Sets Digital 2 pin as back sensor
#define RS 0.1        // Shunt resistor value
#define RL 10000      // Load resistor value
#define REFVOLTAGE 5  // Reference Voltage for Analog Read
#define RINA169 1000  // Resistor Multiplier due to Internal of Sensor

const int MPU = 0x68;       // MPU6050 I2C addresses
const int CurrentPort = A0; // Input pin for measuring Vout
const int VoltagePort = A1; // Input pin for measuring voltage divider

float rawCurrentReading;    // Variable to store value from analog read
float scaledCurrentReading; // Variable to store the scaled value from the analog value
float voltageReading, voltage, seconds;

float current;

unsigned long energy, prevTime, currTime;

int counter = 0;

struct SensorDataStructure {
  float AccX;  // Accelerometer x-axis value for MPU6050
  float AccY;  // Accelerometer y-axis value for MPU6050
  float AccZ;  // Accelerometer z-axis value for MPU6050
  float GyroX; // Gyrometer x-axis value for MPU6050
  float GyroY; // Gyrometer y-axis value for MPU6050
  float GyroZ; // Gyrometer z-axis value for MPU6050
} HandSensorData, ForearmSensorData, BackSensorData, SensorData;

struct PowerDataStructure {
  float Current;
  float Voltage;
  float Energy;
  float Power;
} PowerData;

void setup() {
  pinMode(HAND, OUTPUT);        // Sets hand digital pin as output pin
  pinMode(FOREARM, OUTPUT);     // Sets forearm digital pin as output pin
  pinMode(BACK, OUTPUT);        // Sets back digital pin as output pin

  Wire.begin();                 // Initiates I2C communication
  Wire.beginTransmission(MPU);  // Begins communication with the MPU
  Wire.write(0x6B);             // Access the power management register
  Wire.write(0x00);             // Wakes up the MPU
  Wire.endTransmission(true);   // Communication done

  Wire.beginTransmission(MPU);  // Begins communication with the MPU
  Wire.write(0x1C);             // Access Accelerometer Scale Register
  Wire.write(0x00);             // Set Accelerometer Scale
  Wire.endTransmission(true);   // Communication done

  Wire.beginTransmission(MPU);  // Begins communication with the MPU
  Wire.write(0x1B);             // Access Gyroscope Scale Register
  Wire.write(0x00);             // Set Gyroscope Range
  Wire.endTransmission(true);   // Communication done

  Serial.begin(115200);         // Initialize serial port baud rate to 115200

  Serial.println("Begin Dancing");
  delay(1000);
  Serial.println("Now");
}

void ReadMPUValues() {
  Wire.beginTransmission(MPU);      // Begins communication with the MPU
  Wire.write(0x3B);                 // Register 0x3B upper 8 bits of x-axis acceleration data
  Wire.endTransmission(false);      // End communication
  Wire.requestFrom(MPU, 12, true);  // Request 12 registers

  SensorData.AccX  = Wire.read() << 8 | Wire.read(); // Reads in raw x-axis acceleration data
  SensorData.AccY  = Wire.read() << 8 | Wire.read(); // Reads in raw y-axis acceleration data
  SensorData.AccZ  = Wire.read() << 8 | Wire.read(); // Reads in raw z-axis acceleration data
  Wire.read(); Wire.read();                          // Reads in raw temperature data
  SensorData.GyroX = Wire.read() << 8 | Wire.read(); // Reads in raw x-axis gyroscope data
  SensorData.GyroY = Wire.read() << 8 | Wire.read(); // Reads in raw y-axis gyroscope data
  SensorData.GyroZ = Wire.read() << 8 | Wire.read(); // Reads in raw z-axis gyroscope data
}

void CallibrateMPUValues() {
  SensorData.AccX = SensorData.AccX / 16384.0;
  SensorData.AccY = SensorData.AccY / 16384.0;
  SensorData.AccZ = SensorData.AccZ / 16384.0;
  SensorData.GyroX = SensorData.GyroX / 131.0;
  SensorData.GyroY = SensorData.GyroY / 131.0;
  SensorData.GyroZ = SensorData.GyroZ / 131.0;
}

void UpdateMPUSensorData() {
  CallibrateMPUValues();
  if (!digitalRead(HAND)) {
    HandSensorData = SensorData;
    //Serial.print("Hand:");
  } else if (!digitalRead(FOREARM)) {
    ForearmSensorData = SensorData;
    //Serial.print("Forearm:");
  } else if (!digitalRead(BACK)) {
    BackSensorData = SensorData;
    //Serial.print("Back:");
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
  ReadMPUValues();
  UpdateMPUSensorData();
  //PrintFormattedMPUValues(sds);
  PrintMPUValues(sds);
}

void ExecuteAllSensors() {
  ExecuteSensor(HAND, HandSensorData);
  ExecuteSensor(FOREARM, ForearmSensorData);
  ExecuteSensor(BACK, BackSensorData);
}

void PrintFormattedMPUValues(SensorDataStructure SDS) {
  Serial.print("Accelerometer Values: [x = "); Serial.print(SDS.AccX);
  Serial.print(", y = "); Serial.print( SDS.AccY);
  Serial.print(", z = "); Serial.print(SDS.AccZ); Serial.println("]");
  Serial.print("Gyrorometer Values:   [x = "); Serial.print(SDS.GyroX);
  Serial.print(", y = "); Serial.print(SDS.GyroY);
  Serial.print(", z = "); Serial.print(SDS.GyroZ); Serial.println("]");
  Serial.println();
}

void PrintMPUValues(SensorDataStructure SDS) {
  Serial.print(" "); Serial.print(SDS.AccX); Serial.print(" "); Serial.print(SDS.AccY); Serial.print(" "); Serial.print(SDS.AccZ);
  Serial.print(" "); Serial.print(SDS.GyroX); Serial.print(" "); Serial.print(SDS.GyroY); Serial.print(" "); Serial.print(SDS.GyroZ);
  Serial.print(" ");
}

void ReadCurrent() {
  rawCurrentReading = analogRead(CurrentPort);   // Read sensor value from INA169
  scaledCurrentReading = (rawCurrentReading * REFVOLTAGE) / 1023; // Scale the value to supply voltage that is 5V
  PowerData.Current = (scaledCurrentReading) / (RS * 10); // Is = (Vout x 1k) / (RS x RL)
}

void ReadVoltage() {
  voltageReading = analogRead(VoltagePort);
  PowerData.Voltage = (voltageReading * 5.0 * 2) / 1023;
}

void ReadPower() {
  PowerData.Power = PowerData.Voltage * scaledCurrentReading;
}

void ReadEnergy() {
  currTime = millis();
  PowerData.Energy += (PowerData.Current * PowerData.Voltage *  (currTime - prevTime) / 1000) / 3600;
  prevTime = currTime;
}

void CalculatePowerData() {
  ReadCurrent();
  ReadVoltage();
  ReadPower();
  ReadEnergy();
  //PrintPowerValues(PowerData);
}

void PrintPowerValues(PowerDataStructure PDS) {
  Serial.print("Current: "); Serial.print(PDS.Current, 9); Serial.print(" A, ");
  Serial.print("Voltage: "); Serial.print(PDS.Voltage, 3); Serial.print(" V, ");
  Serial.print("Energy: "); Serial.print(PDS.Energy, 3); Serial.print(" Wh, ");
  Serial.print("Power: "); Serial.print(PDS.Power, 3); Serial.println(" W");
}

void PrintCounterAndTime() {
  Serial.print(counter++);
  Serial.print(" ");
  seconds = currTime / 1000.0;
  Serial.print(seconds, 3);
}

void loop() {
  PrintCounterAndTime();
  CalculatePowerData();
  ExecuteAllSensors();
  Serial.println();
  delay(4);
}
