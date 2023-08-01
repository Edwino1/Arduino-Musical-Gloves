#include<Wire.h>
const int MPU = 0x68; // MPU6050 I2C address
float AccX, AccY, AccZ;
float GyroX, GyroY, GyroZ;
float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ;
float roll, pitch, yaw;
float AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ;
float elapsedTime, currentTime, previousTime;
int c = 0;
const int buzzer = 3;
const int flexPin1 = A0;
const int flexPin2 = A1;
const int flexPin3 = A2;
const int flexPin4 = A3;
const int flexPin5 = A6;

int value1;
int value2;
int value3;
int value4;
int value5;


void setup() {
  // put your setup code here, to run once:

Serial.begin(9600);
  Wire.begin();                      // Initialize comunication
  Wire.beginTransmission(MPU);       // Start communication with MPU6050 // MPU=0x68
  Wire.write(0x6B);                  // Talk to the register 6B
  Wire.write(0x00);                  // Make reset - place a 0 into the 6B register
  Wire.endTransmission(true);        //end the transmission
    
  calculate_IMU_error();
  delay(1000);
  
pinMode(flexPin1,INPUT);
pinMode(flexPin2,INPUT);
pinMode(flexPin3,INPUT);
pinMode(flexPin4, INPUT);
pinMode(flexPin5, INPUT);
pinMode(buzzer, OUTPUT);

 }


void loop() { 
  // put your main code here, to run repeatedly:
  // === Read acceleromter data === //
  Wire.beginTransmission(MPU);
  Wire.write(0x3B); // Start with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); // Read 6 registers total, each axis value is stored in 2 registers
  //For a range of +-2g, we need to divide the raw values by 16384, according to the datasheet
  AccX = (Wire.read() << 8 | Wire.read()) / 16384.0; // X-axis value
  AccY = (Wire.read() << 8 | Wire.read()) / 16384.0; // Y-axis value
  AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0; // Z-axis value
  // Calculating Roll and Pitch from the accelerometer data
  accAngleX = (atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / PI) - 1.31; // AccErrorX ~(1.01) See the calculate_IMU_error()custom function for more details
  accAngleY = (atan(-1 * AccX / sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180 / PI) + 4.30; // AccErrorY ~(-2.70)

  previousTime = currentTime;        // Previous time is stored before the actual time read
  currentTime = millis();            // Current time actual time read
  elapsedTime = (currentTime - previousTime) / 1000; // Divide by 1000 to get seconds
  Wire.beginTransmission(MPU);
  Wire.write(0x43); // Gyro data first register address 0x43
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); // Read 4 registers total, each axis value is stored in 2 registers
  GyroX = (Wire.read() << 8 | Wire.read()) / 131.0; // For a 250deg/s range we have to divide first the raw value by 131.0, according to the datasheet
  GyroY = (Wire.read() << 8 | Wire.read()) / 131.0;
  GyroZ = (Wire.read() << 8 | Wire.read()) / 131.0;
  // Correct the outputs with the calculated error values
  GyroX = GyroX - 1.77; // GyroErrorX ~(-4.79)
  GyroY = GyroY - 1.31; // GyroErrorY ~(-0.84)
  GyroZ = GyroZ + 0.46; // GyroErrorZ ~ (0.21)
  // Currently the raw values are in degrees per seconds, deg/s, so we need to multiply by sendonds (s) to get the angle in degrees
  gyroAngleX = gyroAngleX + GyroX * elapsedTime; // deg/s * s = deg
  gyroAngleY = gyroAngleY + GyroY * elapsedTime;
  yaw =  yaw + GyroZ * elapsedTime;
  // Complementary filter - combine acceleromter and gyro angle values
  roll = 0.96 * gyroAngleX + 0.04 * accAngleX;
  pitch = 0.96 * gyroAngleY + 0.04 * accAngleY;
 
    // Print the values on the serial monitor
  Serial.print("Roll: ");  
  Serial.print(roll);
  Serial.print("  ");
  Serial.print("Pitch: ");  
  Serial.print(pitch);
  Serial.print("  ");
  Serial.print("Yaw: ");  
  Serial.println(yaw);
  delay(50);

value1 = analogRead(flexPin1);
Serial.print("Flexpin1: ");
Serial.println(value1);
value2 = analogRead(flexPin2);
Serial.print("Flexpin2: ");
Serial.println(value2);
value3 = analogRead(flexPin3);
Serial.print("Flexpin3: ");
Serial.println(value3);
value4 = analogRead(flexPin4);
Serial.print("Flexpin4: ");
Serial.println(value4);
value5 = analogRead(flexPin5);
Serial.print("Flexpin5: ");
Serial.println(value5);
//delay(1000);

if (pitch > 50)
{
  Bass_mode();
  Serial.println("Bass");
}
else
{
  Piano_mode();
  Serial.println("Piano");
}

}


void calculate_IMU_error() {
  // We can call this funtion in the setup section to calculate the accelerometer and gyro data error. From here we will get the error values used in the above equations printed on the Serial Monitor.
  // Note that we should place the IMU flat in order to get the proper values, so that we then can the correct values
  // Read accelerometer values 200 times
  while (c < 400) {
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    AccX = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    AccY = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    // Sum all readings
    AccErrorX = AccErrorX + ((atan((AccY) / sqrt(pow((AccX), 2) + pow((AccZ), 2))) * 180 / PI));
    AccErrorY = AccErrorY + ((atan(-1 * (AccX) / sqrt(pow((AccY), 2) + pow((AccZ), 2))) * 180 / PI));
    c++;
  }
    //Divide the sum by 400 to get the error value
  AccErrorX = AccErrorX / 400;
  AccErrorY = AccErrorY / 200;
  c = 0;
  // Read gyro values 400 times
  while (c < 400) {
    Wire.beginTransmission(MPU);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    GyroX = Wire.read() << 8 | Wire.read();
    GyroY = Wire.read() << 8 | Wire.read();
    GyroZ = Wire.read() << 8 | Wire.read();
    // Sum all readings
    GyroErrorX = GyroErrorX + (GyroX / 131.0);
    GyroErrorY = GyroErrorY + (GyroY / 131.0);
    GyroErrorZ = GyroErrorZ + (GyroZ / 131.0);
    c++;
  }

   //Divide the sum by 400 to get the error value
  GyroErrorX = GyroErrorX / 400;
  GyroErrorY = GyroErrorY / 400;
  GyroErrorZ = GyroErrorZ / 400;
  // Print the error values on the Serial Monitor
  Serial.print("AccErrorX: ");
  Serial.println(AccErrorX);
  Serial.print("AccErrorY: ");
  Serial.println(AccErrorY);
  Serial.print("GyroErrorX: ");
  Serial.println(GyroErrorX);
  Serial.print("GyroErrorY: ");
  Serial.println(GyroErrorY);
  Serial.print("GyroErrorZ: ");
  Serial.println(GyroErrorZ);

}

void Piano_mode(){
  if (value1 > 800)
{
  tone(buzzer, 256); //C
}
else if (value2 > 800)
{
  tone(buzzer, 293); //D
}
else if (value3 > 800)
{
  tone(buzzer, 329); //E
}
else if (value4 > 800)
{
  tone(buzzer, 349); //F
}
else if (value5 > 800)
{
  tone(buzzer, 392); //G
}
else if (value4 <= 800)
{
  noTone(buzzer);
}
}

void Bass_mode(){
   if (value1 > 800)
{
  tone(buzzer, 82); //E
}
else if (value2 > 800)
{
  tone(buzzer, 110); //A
}
else if (value3 > 800)
{
  tone(buzzer, 147); //D
}
else if (value4 > 800)
{
  tone(buzzer, 196); //G
}
else if (value5 > 800)
{
  tone(buzzer, 245); //B
}
else if (value1 <= 800)
{
  noTone(buzzer);
}

}
