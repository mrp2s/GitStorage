
unsigned int counter = 0;
unsigned int counter2 = 0;
float lastTemp = 0;
float lastAlt = 0;

float getAltitude(){
    float altitude1 = (pow((initPressure/barometer.getPressure()),(1/5.257))-1)*((lastTemp)+273.15)/0.0065;
    return altitude1+altitudeOffset;
}

void setInitPressure(){
  float totalValue = 0;
  float readings = 0;  
  
  while(readings<20){
     readings++;
     totalValue+=barometer.getPressure();
     delay(10);
  }
  lastTemp = barometer.getTemperature()/100.0f;
  initPressure = (totalValue/readings)-2000;  // -2000 because the formula works better a small distance above the reference pressure
  altitudeOffset = -getAltitude();
}


void setupBarometer(){
    barometer.begin();
    setInitPressure();
    stableAlt = getAltitude();
}

void updateBarometer(){
  stableAlt+=(lastAlt-stableAlt)/8.0f;
  currentPositionGrid[2] = stableAlt;

  //speeding up
  counter++;
  counter2++;
  if(counter > 12){
    counter = 0;
    lastTemp = barometer.getTemperature()/100.0f; 
  }
  if(counter2 > 3){
    counter2 = 0;
    lastAlt = getAltitude();  
  }
}


