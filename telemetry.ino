
#include <SparkFunMPU9250-DMP.h>
#include "SparkFunBME280.h"
#include <Wire.h> //allows you to communicate with I2C devices

#define serial SerialUSB

MPU9250_DMP imu;
BME280 mySensor;


void setup() 
{
  Serial.begin(115200);
  Wire.begin();

 if (mySensor.beginI2C() == false) //Begin communication over I2C
  {
    Serial.println("The sensor did not respond. Please check wiring.");
    while(1); //Freeze
  }
  if (imu.begin() != INV_SUCCESS)
  {
    while (1)
    {
      serial.println("Unable to communicate with MPU-9250");
      serial.println("Check connections, and try again.");
      serial.println();
      delay(5000);
    }
  }
  imu.setSensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);
  // Gyro options are +/- 250, 500, 1000, or 2000 dps
  imu.setGyroFSR(2000); // Set gyro to 2000 dps
  imu.setAccelFSR(16); // Set accel to full range

  imu.setLPF(5); // Set LPF corner frequency to 5Hz

  imu.setSampleRate(10); // Set sample rate to 10Hz
}

void loop() 
{
  if ( imu.dataReady() )
  {
    imu.update(UPDATE_ACCEL | UPDATE_GYRO | UPDATE_COMPASS);
    printIMUData();
  }
}

void printIMUData(void)
{  
  float accelX = imu.calcAccel(imu.ax);
  float accelY = imu.calcAccel(imu.ay);
  float accelZ = imu.calcAccel(imu.az);
  float gyroX = imu.calcGyro(imu.gx);
  float gyroY = imu.calcGyro(imu.gy);
  float gyroZ = imu.calcGyro(imu.gz);
  float magX = imu.calcMag(imu.mx);
  float magY = imu.calcMag(imu.my);
  float magZ = imu.calcMag(imu.mz);
  float humidity = mySensor.readFloatHumidity();
  float pressure = mySensor.readFloatPressure();
  float altitude = mySensor.readFloatAltitudeFeet();
  float temp = mySensor.readTempF();
  
  serial.print(accelX);
  serial.print(",");
  serial.print(accelY);
  serial.print(",");
  serial.print(accelZ);
  serial.print(",");
  serial.print(gyroX);
  serial.print(",");
  serial.print(gyroY);
  serial.print(",");
  serial.print(gyroZ);
  serial.print(",");
  serial.print(humidity);
  serial.print(","); 
  serial.print(pressure);
  serial.print(","); 
  serial.print(altitude);
  serial.print(","); 
  serial.println(temp); 


  delay(500);
}

