#include <SparkFunMPU9250-DMP.h>
#include "SparkFunBME280.h"
#include <Wire.h> //allows you to communicate with I2C devices

#define serial SerialUSB
#include <Servo.h> // servo

// creates internal clock
class Time {
  public:
    int count;
    int seconds;
    const int delay = 100;  // must be a number < 100
    Time();
    void printSecs();
    void incrementSecs();
};

Time::Time() {
  count = 0;
  seconds = 1;
}

void Time::incrementSecs() {
  count+=1;
  if (count == 10) {
    seconds++;
    serial.println(seconds);            // prints out secs
    count = 0;
   }
}

int callTime = 0;                       // stores time when alt is sampled
float initAlt;                          // stores init alt sample of interval
float finalAlt;                         // stores final alt sample of interval

bool initCall = false;                  // stores if alt is initially sampled
bool enable = false;                    // enables rocket to inflate payload (will only occur if Δalt > certain const)
                                        
bool inflated = false;                  // stores inflation status of payload
                                          
MPU9250_DMP imu;                        // creates imu sensor object
BME280 mySensor;                        // creates altimeter sensor object
Servo myservo;                          // creates servo object
Time mainClock;                         // creates main clock/time that rocket runs on
    
const int servo_pin = 11;               // output to the servo
const int start_angle = 0;
const int sampleTime = 1;

// configuring the sensor and pins
void setup() {
  myservo.attach(servo_pin);
  myservo.write(start_angle);
  Serial.begin(115200);
  Wire.begin();

  
 if (mySensor.beginI2C() == false) {    // Begin communication over I2C 
    Serial.println("The sensor did not respond. Please check wiring.");
    while(1); //Freeze
  }
  if (imu.begin() != INV_SUCCESS) {
    while (1) {
      serial.println("Unable to communicate with MPU-9250");
      serial.println("Check connections, and try again.");
      serial.println();
      delay(5000);
    }
  }
  
  imu.setSensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);
  // Gyro options are +/- 250, 500, 1000, or 2000 dps
  imu.setGyroFSR(2000);   // Set gyro to 2000 dps
  imu.setAccelFSR(16);    // Set accel to full range

  imu.setLPF(5);          // Set LPF corner frequency to 5Hz
  imu.setSampleRate(10);  // Set sample rate to 10Hz
}

// ----------------------------------------------------------- END OF SETUP -----------------------------------------------------------

// ----------------------------------------------------------- START OF LOOP ----------------------------------------------------------

void loop() {
  double alt;
  double accelz;
  double accely;
  double gyroz;
  
  if ( imu.dataReady() ) {
    imu.update(UPDATE_ACCEL | UPDATE_GYRO | UPDATE_COMPASS);
    printIMUData(alt);
  }

  if (mainClock.seconds % sampleTime == 0 && initCall == false) {
    callTime = mainClock.seconds;
    initAlt = mySensor.readFloatAltitudeFeet();           // samples alt, storing it as initial alt
    initCall = true;                                      // declaring that initial height has been sampled so it doesnt repeat
  }

  if (mainClock.seconds == (callTime + sampleTime) && initCall == true) {
    finalAlt = mySensor.readFloatAltitudeFeet();          // samples alt, storing it as final alt
    serial.print("Change in altitude: ");
    altChange(initAlt, finalAlt);

    if (altChange(initAlt, finalAlt) > 50) {                  // verifies that the rocket has taken off (has been in the air)
      enable = true;
    }
   
    if (altChange(initAlt, finalAlt) < 2 && enable) {         // open valve
      for(int angle = 0; angle <= 350; angle += 5) {                 
          myservo.write(angle);                     
          delay(50);
          inflated = true;               
        }
        delay(7500);                                          // wait 7.5s
        myservo.write(start_angle);                           // close Valve
        stop();                                               // end 
    }
    
    initCall = false;
   }
    
}

// ----------------------------------------------------------- END OF LOOP ----------------------------------------------------------

// ----------------------------------------------------------- FUNCTION DEFs --------------------------------------------------------

// initializes main rocket clock, samples and prints sensor data, every DELAY amount
void printIMUData(double& alt) {  
  mainClock.incrementSecs();
  
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
  
  delay(mainClock.delay);    // uses the dalay preset in the time
}

// calculates and returns difference between initial and final altitudes
float altChange(float initAlt, float finalAlt) {
    serial.println(abs(finalAlt-initAlt));
    return abs(finalAlt-initAlt);
}

// acts as a permanent end to the program
// only call when program needs to be ended entirely
void stop()
{
 while(1);                  // calls an infinite loop
}

// ----------------------------------------------------------- END FUNCTION DEFs -----------------------------------------------------

