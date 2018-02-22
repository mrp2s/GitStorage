bool blinkState = false;
float hertzCount = 0;
long secondCount = 0;

//#define POSITION_OUTPUT
//#define ROTATION_OUTPUT
//#define PRINT
//#difine OUTPUT_HERTZ

void debug(){
  //blink to indicate activity
   blinkState = !blinkState;
   digitalWrite(13, blinkState);


  //Print update frequency in hertz
  #ifdef OUTPUT_HERTZ
     hertzCount+=1;
     if(millis()-secondCount>1000){
        float secondsPassed = float(millis()-secondCount);
        secondsPassed/=1000.0f;
        Serial.println(hertzCount/secondsPassed);
        hertzCount = 0;
        secondCount = millis();
     }
  #endif


   //Print information:
    //Position
    #ifdef POSITION_OUTPUT
      Serial.print("Position: ");Serial.print("\t");
      Serial.print(currentPositionGrid[0]);Serial.print("\t");
      Serial.print(currentPositionGrid[1]);Serial.print("\t");
      Serial.print(currentPositionGrid[2]);Serial.print("\t");
    #endif

    #ifdef ROTATION_OUTPUT
    //Orientation
      Serial.print("Rotation: ");Serial.print("\t");
      Serial.print(eulerRotation.x*cvt);Serial.print("\t");
      Serial.print(eulerRotation.y*cvt);Serial.print("\t");
      Serial.println(eulerRotation.z*cvt);
    #endif

    #ifdef PRINT
    Serial.print("Channels: ");
    for(int i=0; i<CHANNELS; i++){
      Serial.print(percentValues[i]);Serial.print("\t");
    }
    Serial.println("");
    delay(20);
  #endif
}
