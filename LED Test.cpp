#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

//setup LED Pins
int LED_Back_Left = 9;
int LED_Front_Left = 10;
int LED_Front_Right = 11;
int LED_Back_Right = 12;

//declare acceleration and rotation variables 
int16_t ax, ay, az;
int16_t gx, gy, gz;

float Calibration_gx, Calibration_gy, Calibration_gz;
int CalibrationNumber;
//value for when light turns on
int max_top = 0;

int max_bottom = -1 * max_top;


  

void setup() {
  Serial.begin(9600);

  Wire.begin();
  mpu.initialize();

  // Verify connection
  if (mpu.testConnection()) {
    Serial.println("MPU-6050 connection successful");
  } else {
    Serial.println("MPU-6050 connection failed");
    while (1);
  }
  //perform calibration measurements
  for(CalibrationNumber = 0; CalibrationNumber < 2000; CalibrationNumber++){
    mpu.getRotation(&gx, &gy, &gz);
    Calibration_gx += gx;
    Calibration_gy += gy;
    Calibration_gz += gz;
    delay(1);
  }
  //calculate calibration values
  Calibration_gx /= 2000;
  Calibration_gy /= 2000;
  Calibration_gz /= 2000;
  Serial.println("Calibration Succesful...");
  }
  
void loop() {
  // Read accelerometer data
  mpu.getAcceleration(&ax, &ay, &az);
  
  // Read gyroscope data
  mpu.getRotation(&gx, &gy, &gz);
  //Change to calibrated data
  gx -= Calibration_gx;
  gy -= Calibration_gy;
  gz -= Calibration_gz;

  // Display accelerometer and gyroscope data on the serial monitor
  Serial.print(" | Accel X: "); Serial.print(ax);
  Serial.print(" | Accel Y: "); Serial.print(ay);
  Serial.print(" | Accel Z: "); Serial.print(az);
  Serial.print(" | Gyro X: "); Serial.print(gx);
  Serial.print(" | Gyro Y: "); Serial.print(gy);
  Serial.print(" | Gyro Z: "); Serial.println(gz);

   //rotation greater than set value
  if (gy > max_top){
    //turn on pins 11 and 12
    digitalWrite(LED_Front_Left, LOW);
    digitalWrite(LED_Back_Left, LOW);
    digitalWrite(LED_Front_Right, HIGH);
    digitalWrite(LED_Back_Right, HIGH);
  }
  if (gy < max_bottom){
    //turn on pins 9 and 10
    digitalWrite(LED_Front_Left, HIGH);
    digitalWrite(LED_Back_Left, HIGH);
    digitalWrite(LED_Front_Right, LOW);
    digitalWrite(LED_Back_Right, LOW);
  }
  if (gx > max_top){
    //turn on pins 9 and 12
    digitalWrite(LED_Front_Left, LOW);
    digitalWrite(LED_Back_Left, HIGH);
    digitalWrite(LED_Front_Right, LOW);
    digitalWrite(LED_Back_Right, HIGH);
  }
  if (gx < max_bottom){
    //turn on pins 100 and 11
    digitalWrite(LED_Front_Left, HIGH);
    digitalWrite(LED_Back_Left, LOW);
    digitalWrite(LED_Front_Right, HIGH);
    digitalWrite(LED_Back_Right, LOW);
  }
  delay(500); // Adjust the delay as needed
} 
