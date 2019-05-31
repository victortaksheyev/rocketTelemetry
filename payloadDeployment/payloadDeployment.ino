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
const int minDeploymentH = 800;                                   // feet
const int maxDeploymentH = 1200;                                  // feet
float prevAlt = -1;
float currAlt;
bool noseConeDeployed = false;
// global vars ---------------------------------------------------


float mToF(float h);
void getInitialAlt(float& initAlt);
void getMax(float alt, float& max);
bool inDeployRange(float currAlt, float maxDeploymentH, float minDeploymentH);
bool decreasingH(float currH, float prevH);

void setup() {
  Serial.begin(9600);
  
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

//  Serial.print("initial altitude "); Serial.print(initH);
  
  currAlt = mToF(baro.getAltitude()) - initH;
  Serial.print("previous height: "); Serial.print(prevAlt);Serial.println(" ft");
  Serial.print("current height: ");Serial.print(currAlt); Serial.println(" ft");
  if (decreasingH(currAlt, prevAlt)) {
    Serial.println("Decreasing");
  }
  getMax(currAlt, maxH);
//  Serial.print("max H: "); Serial.println(maxH); 

//  if (inDeployRange(currAlt, maxDeploymentH, minDeploymentH)) Serial.println("True");
//  else Serial.println("False");
 
  // we can deploy if the following is true: 
  // we are between minDeployment and maxDeployment alt
  //                the nosecone is off the rocket
  //                the altitude is smaller than the maxAlt
  //                the current altitude is smaller than the previous altitude

  if (noseConeDeployed && decreasingH(currAlt, prevAlt) && inDeployRange(currAlt, maxDeploymentH, minDeploymentH) && currAlt < maxH) {
    Serial.println("******************DEPLOY******************");
    while(1); 
  }
 
  delay(250);
  
  
}



void getInitialAlt(float& initAlt) {
  float A;
  for (int i = 0; i < 5; i++) {               // sample the altitude 5 times to confirm it's stable
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

