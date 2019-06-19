// ------------------------------------------------------
// PAYLOAD INFLATION (RED SENSOR)
// UPLOAD TO RED SENSOR
// ------------------------------------------------------

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
bool firstEnable = false;
bool secondEnable = false;
bool mainEnable = false;                    // enables rocket to inflate payload (will only occur if Δalt > certain const)
                                        
bool inflated = false;                  // stores inflation status of payload
                                          
MPU9250_DMP imu;                        // creates imu sensor object
BME280 mySensor;                        // creates altimeter sensor object
Servo myservo;                          // creates servo object
Time mainClock;                         // creates main clock/time that rocket runs on

const int lockInput = 10;               // input to second armed mechanism
const int servo_pin = 11;               // output to the servo
const int buzzer = 12;                  // output to buzzer
const int start_angle = 0;
const int sampleTime = 1;

//const int test_led = 12;

bool firstRun = true;
bool lockEnable = false;
bool prevInput;                         // stores previous input of the armed (true for HIGH false for LOW)

long startTime;
long highTime = 0; // counts how much time the input is HIGH consecutively
long prevTime = 0;

const int finalInfaltionAngle = 45;     // 45 degrees
const int delayBeforeClosing = 2300;    // wait 2.3 seconds before closing
const int enable_velocity = 100;        // 100 ft/s



double initHeight;

// configuring the sensor and pins
void setup() {

  Serial.begin(115200);
  
  myservo.attach(servo_pin);
  myservo.write(start_angle);
  
  Wire.begin();
  digitalWrite(lockInput, LOW);
  pinMode(lockInput, INPUT);

  digitalWrite(buzzer, LOW);
  pinMode(buzzer, OUTPUT);
  
   
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

  
    startTime = millis();   // samples start time
  
}

// ----------------------------------------------------------- END OF SETUP -----------------------------------------------------------

// ----------------------------------------------------------- START OF LOOP ----------------------------------------------------------

void loop() {
//  digitalWrite(buzzer, LOW);
  
  if (lockEnable){

    digitalWrite(buzzer, LOW);
    
   // ------------------------------------------------- MAIN CODE -------------------------------------------------
    if (firstRun) {
    // samples initial height 5 times
    for (int i = 0; i < 5; i++){
         initHeight = mySensor.readFloatAltitudeFeet();
    }
    
    firstRun = false;   // prevent it from running again
  }

//  serial.print("initial Altitude"); serial.println(initHeight);
  double alt;
  double accelz;
  double accely;
  double gyroz;
  
  if ( imu.dataReady() ) {
    imu.update(UPDATE_ACCEL | UPDATE_GYRO | UPDATE_COMPASS);
  }

  if (mainClock.seconds % sampleTime == 0 && initCall == false) {
    callTime = mainClock.seconds;
    initAlt = mySensor.readFloatAltitudeFeet();           // samples alt, storing it as initial alt
//    serial.print("INITIAL height = "); serial.println(initAlt);
    initCall = true;                                      // declaring that initial height has been sampled so it doesnt repeat
  }

  if (mainClock.seconds == (callTime + sampleTime) && initCall == true) {
    finalAlt = mySensor.readFloatAltitudeFeet();          // samples alt, storing it as final alt
//    serial.print("FINAL height = "); serial.println(finalAlt);
//    serial.print("Change in altitude: "); serial.println(altChange(initAlt, finalAlt));

    serial.print("Absolute altitude: "); serial.println(finalAlt - initHeight);

    if (altChange(initAlt, finalAlt) >= enable_velocity) {
      firstEnable = true;  
    }

    if ((finalAlt - initHeight) >= 3000) {
      secondEnable = true;
    }
    
    if (firstEnable && secondEnable) { // we have taken off and traveled at least 3000 ft
      mainEnable = true;
    digitalWrite(buzzer, HIGH);
    delay(100);
    digitalWrite(buzzer, LOW);
    delay(100);
    digitalWrite(buzzer, HIGH);
    delay(100);
    }
    
    int totalAngle = 0;
    if (altChange(initAlt, finalAlt) < 2 && mainEnable && (finalAlt < (initHeight + 300))) {         // open valve
      for(int angle = 0; angle <= finalInfaltionAngle; angle += 5) {                 
          myservo.write(angle);                     
          delay(50);
          totalAngle += 5;
          inflated = true;               
        }
        
        delay(delayBeforeClosing);    // wait 2.3 seconds

       // close valve
       for (int angle = totalAngle; angle >= 0; angle -= 5) {
          myservo.write(angle);
          delay(50);
       }
        serial.println("------------------ THE VALVE IS CLOSED--------------------------------");                                       
        stop();                                               // end 
    }
    
    initCall = false;
   }

   
   mainClock.incrementSecs();
   delay(mainClock.delay);    // uses the dalay preset in the time
    
  }

  else {
//    serial.println("inside the else");
      if (digitalRead(lockInput) == LOW) {
          prevInput = false;  // the previous input was LOW
        } else {
          prevInput = true;   // the previous input was HIGH  
        }
        if (prevInput) {
          
          highTime  += (millis() - prevTime);
//          highTime +=1;
//          serial.print("high time:  ");serial.println(highTime);
         } else {
          highTime = 0; // reset it back to 0; 
//          serial.println("Disabled back to 0");
         }

         prevTime = millis();

         if (highTime >= 2000) { // if we've been high for 10 seconds, SOUND IT!
//           serial.println("ENABLE SOUND and lockEnable");
           digitalWrite(buzzer, HIGH);
           delay(5000);
           lockEnable = true;
         }      
   }
}

// ----------------------------------------------------------- END OF LOOP ----------------------------------------------------------

// ----------------------------------------------------------- FUNCTION DEFs --------------------------------------------------------

// initializes main rocket clock, samples and prints sensor data, every DELAY amount

// calculates and returns difference between initial and final altitudes
float altChange(float initAlt, float finalAlt) {
    return abs(finalAlt-initAlt);
}

// acts as a permanent end to the program
// only call when program needs to be ended entirely
void stop()
{
 while(1);                  // calls an infinite loop
}


// ----------------------------------------------------------- END FUNCTION DEFs -----------------------------------------------------

