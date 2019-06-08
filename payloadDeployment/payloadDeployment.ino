#include <Wire.h>
#include <Adafruit_MPL3115A2.h>


// Connect  3.3 to 3vo
//          gnd to gnd
//          SCL to SCL
//          SDA to SDA


Adafruit_MPL3115A2 baro = Adafruit_MPL3115A2();

// global vars ---------------------------------------------------
float initH = 0;
float maxH = 0;
bool firstRun = true;
const int minDeploymentH = 850;                                   // feet
const int maxDeploymentH = 1000;                                  // feet maybe change this to 1100
float prevAlt = -1;
float currAlt;
bool noseConeDeployed = true;
const int enableLED = 6;
const int led = 5;
const int redundantLED = 4;
int seconds = 0;
bool initCall = false;
const int sampleTime = 1;                                         // performing redundant check algo every second
int callTime = 0;


float initAlt;
float finalAlt;
bool enable = false;

// global vars ---------------------------------------------------


class Time {
  public:
    int count;
    int seconds;
    const int delay = 250;
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
  if (count == 2) {
    seconds++;
//    Serial.println(seconds);            // prints out secs
    count = 0;
   }
}

Time clock;

float mToF(float h);
void getInitialAlt(float& initAlt);
void getMax(float alt, float& max);
bool inDeployRange(float currAlt, float maxDeploymentH, float minDeploymentH);
bool decreasingH(float currH, float prevH);
float altChange(float initAlt, float finalAlt);

void setup() {
  Serial.begin(9600);
  pinMode(led, OUTPUT);
  pinMode(enableLED, OUTPUT);
  pinMode(redundantLED, OUTPUT);
}
 
void loop() {
  if (! baro.begin()) {                           // not an option
    Serial.println("Couldnt find sensor");
    return;
  } 
  if (firstRun) {                                    // will only run 1 time
    getInitialAlt(initH);  
    firstRun = false;
    prevAlt = -1;
  } else {
    prevAlt = currAlt;  
  }
  
  currAlt = mToF(baro.getAltitude()) - initH;

  getMax(currAlt, maxH);

//  ------------------------------------------------------------------ MAIN SYSTEM ---------------------------------------------------------------------
  if ( decreasingH(currAlt, prevAlt) && inDeployRange(currAlt, maxDeploymentH, minDeploymentH) && currAlt < maxH) {
    Serial.println("------------- DEPLOYED (via main system) ----------------");
    digitalWrite(led, HIGH);    // REPLACE THIS WITH TRANSISTOR CODE
    delay(5000);                // send current for 5 seconds
    while(1);                   // BOOM we good
  }

// ------------------------------------------------------------ END MAIN SYSTEM ---------------------------------------------------------------------



//  ------------------------------------------------------------------ REDUNDANT SYSTEM ---------------------------------------------------------------------
  if (clock.seconds % sampleTime == 0 && initCall == false) {
    callTime = clock.seconds;
    initAlt = currAlt;
    initCall = true;                                      // declaring that initial height has been sampled so it doesnt repeat
    Serial.print("Initial alt sampled - ");Serial.println(initAlt);
    }


  if (clock.seconds == (callTime + sampleTime) && initCall == true) {
    finalAlt = currAlt;
    Serial.print("Final alt sampled - "); Serial.println(finalAlt);
    
    
    Serial.print("Change in altitude: "); Serial.println(altChange(initAlt, finalAlt));
    
    
    if (altChange(initAlt, finalAlt) > 50) {  // verifies that the rocket has TAKEN OFF (has been in the air)
      enable = true;
//      digitalWrite(enableLED, HIGH);   
//      Serial.println("Enabled!!!");
    }

    if ( currAlt < minDeploymentH && currAlt < maxH && enable && decreasingH(currAlt, prevAlt)) {
      digitalWrite(redundantLED, HIGH); // REPLACE THIS WITH TRANSISTOR SIGNAL
      delay(5000);                                                                            // for 5 seconds
      Serial.println("------------- DEPLOYED (via redundant system) ----------------");
      while(1);                                
    }
    initCall = false;
  }
    delay(250);
    clock.incrementSecs();                     // checks and increments count, increments seconds after 4 incremenets of count
}

//  ---------------------------------------------------------- END REDUNDANT SYSTEM ---------------------------------------------------------------------
void getInitialAlt(float& initAlt) {
  float A;
  for (int i = 0; i < 5; i++) {                // sample the altitude 5 times to confirm it's stable
    A = baro.getAltitude();
  }
  initAlt = mToF(A);                           // return the altitude in feet
}

float mToF(float h){          
  return (h * 3.281);
}

void getMax(float alt, float& maximumH){
  if (alt > maximumH) maximumH = alt;
}

bool inDeployRange(float currAlt, float maxDeploymentH, float minDeploymentH){
  if (currAlt <= maxDeploymentH && currAlt >= minDeploymentH) {
    return true;  
  }
  return false;
}

bool decreasingH(float currH, float prevH) {
  if (currH < prevH) return true;
  else return false;
}

// calculates and returns difference between initial and final altitudes
float altChange(float initAlt, float finalAlt) {
    return abs(finalAlt-initAlt);
}

