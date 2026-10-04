#include <Wire.h>
#include <Servo.h>

const int X_SERVO_PIN = 9;
const int Y_SERVO_PIN = 10;
const int MPU_ADDR = 0x68;

Servo xServo;
Servo yServo;

const int WINDOW_SIZE = 128;
int16_t sampleBuffer[WINDOW_SIZE] = {0};
int bufferIndex = 0;
bool bufferFull = false;

void setup() {
  Serial.begin(115200);
  while (!Serial) { ; } // Wait for serial monitor window to open
  
  Serial.println("Initializing system...");
  
  // Initializing I2C communication
  Wire.begin();
  
  Wire.beginTransmission(MPU_ADDR);
  int error = Wire.endTransmission(); 
  
  if (error == 0) {
    Serial.println("Success: MPU6050 physically connected!");
  } else {
    Serial.print("Error: Failed to connect. I2C Error Code: ");
    Serial.println(error);
  }

  Wire.beginTransmission(MPU_ADDR); 
  Wire.write(0x6B); // Target PWR_MGMT_1 register
  Wire.write(0);    // Set sleep bit to 0 (wake up)
  Wire.endTransmission(true); 

  xServo.write(90);
  yServo.write(90);
  xServo.attach(X_SERVO_PIN);
  yServo.attach(Y_SERVO_PIN);

  for (int i = 0; i < WINDOW_SIZE; i++) {
    sampleBuffer[i] = 0;
  }
}

void loop() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false); // Keep connection going repeatedly
  Wire.requestFrom(MPU_ADDR, 6, true); // requesting 6 bytes from accel

  // Combining two 8-bit bytes together to make a 16-bit number
  int16_t rawX = (Wire.read() << 8) | Wire.read();
  int16_t rawY = (Wire.read() << 8) | Wire.read(); 
  int16_t rawZ = (Wire.read() << 8) | Wire.read(); 

  // Converts analogue to digital mapping an accelerometer range
  int xAngle = map(rawX, -17000, 17000, 0, 180); 
  int yAngle = map(rawY, -17000, 17000, 0, 180);

  // Clamps values between 0-180
  xAngle = constrain(xAngle, 0, 180); 
  yAngle = constrain(yAngle, 0, 180);

  // Move the servos
  xServo.write(xAngle);
  yServo.write(yAngle);

  // Update rolling buffer
  sampleBuffer[bufferIndex] = rawX;
  bufferIndex++;
  if (bufferIndex >= WINDOW_SIZE) {
    bufferIndex = 0;
    bufferFull = true;
  }

  Calculations();
  delay(20);
}

void Calculations() { 
  int activeCount = bufferFull ? WINDOW_SIZE : bufferIndex;
  if (activeCount == 0) return;

  long sum = 0;
  for (int i = 0; i < activeCount; i++) {
    sum += sampleBuffer[i];
  }
  double mean = (double)sum / activeCount;
  
  double variance = 0;
  for (int i = 0; i < activeCount; i++) {
    double diff = sampleBuffer[i] - mean;
    variance += diff * diff;
  }
  double stdev = sqrt(variance / activeCount);

  Serial.print("Mean: ");
  Serial.print(mean);
  Serial.print(" | Standard Deviation: ");
  Serial.println(stdev);
}
