#include <SparkFunMPU9250-DMP.h>
#include "SparkFunBME280.h"
#include <Wire.h> //allows you to communicate with I2C devices

#define serial SerialUSB
#include <Servo.h> // servo

// creates internal clock
class Time {
    int count;
  public:
    int seconds;
    const int delay = 100;  // must be a number < 100
    Time();             // constuctor
    void printSecs();
    void incrementCount() {count+=1;} // increments count
};

Time::Time() {
  count = 0;
  seconds = 0;
}

void Time::printSecs() {
  if (count == 10) {
    seconds++;
    serial.println(seconds);
    count = 0;
   }
}

int count = 0;
int seconds = 0;
double maxAlt = 0;
double minAlt = 10000;

MPU9250_DMP imu;
BME280 mySensor;
Servo myservo;
Time mainClock;

const int servo_pin = 11;
bool deployed = false;
const int start_angle = 0;

void setup() 
{
  
  myservo.attach(servo_pin);
  myservo.write(start_angle);
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

void loop() {
  double alt;
  double accelz;
  double accely;
  double gyroz;
  
  
  if ( imu.dataReady() ) {
    imu.update(UPDATE_ACCEL | UPDATE_GYRO | UPDATE_COMPASS);
    printIMUData(alt);
//    serial.println(alt);
//    serial.print("\t");
//    serial.println(accelz);
////    serial.print("\t");
////    serial.print(accely);
////    serial.print("\t");
//    serial.println(gyroz);
//    smallestAlt(alt, minAlt); 
//    largestAlt(alt, maxAlt);

//    
//    if (alt < 2000) {
//        for(int angle = 0; angle<=300; angle+=5) {   // command to move from 180 degrees to 0 degrees                             
//          myservo.write(angle);                      //command to rotate the servo to the specified angle
//          delay(50);
//          deployed = true;               
//        }
//        // stop 
//        if (deployed == true) {
//          delay(15000);
//          myservo.write(start_angle);
//          stop();
//        }
//    }
  }
}

void stop()
{
 while(1);
}


void printIMUData(double& alt)
{  
  mainClock.incrementCount();
  mainClock.printSecs();
  
//  serial.print(mainClock.seconds);
  
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
//  alt = altitude;
//  accelz = accelZ;
//  gyroz = gyroZ;
//  accely = accelY;
  
  
//  serial.print(accelX);
//  serial.print(",");
//  serial.print(accelY);
//  serial.print(",");
//  serial.print(accelZ);
//  serial.print(",");
//  serial.print(gyroX);
//  serial.print(",");
//  serial.print(gyroY);
//  serial.print(",");
//  serial.print(gyroZ);
//  serial.print(",");
//  serial.print(humidity);
//  serial.print(","); 
//  serial.print(pressure);
//  serial.print(","); 
//  serial.print(altitude);
//  serial.print(","); 
//  serial.println(temp); 

//  if (seconds == 10) {
//    serial.print("smallest altitude: ");
//    serial.println(minAlt);
//  }
  
  delay(mainClock.delay);    // uses the dalay preset in the time
}

void altChange() {
  
}

// calculates the smallest altitude
void smallestAlt(double alt, double& maxAlt) {
    if(alt<minAlt) {
    minAlt = alt;
   }
}

void largestAlt(double alt, double& maxAlt) {
   if(alt<maxAlt) {
    maxAlt = alt;
   }
//   serial.print(alt);
//   serial.print("\t");
//   serial.println(maxAlt);
}





