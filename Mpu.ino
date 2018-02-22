unsigned int magCounter = 0;

bool lastWasSpike = false;    //remove data spikes
bool lastlastWasSpike = false;
float rotationPitch = 0;
float rotationRoll = 0;
float rotationYaw = 0;

void initMpu(){ 
    pinMode(42,OUTPUT);
    digitalWrite(42,HIGH);
  
    uint8_t devStatus; 
    devStatus = mpu.dmpInitialize();

    mpu.setXGyroOffset(150);
    mpu.setYGyroOffset(-32);
    mpu.setZGyroOffset(-6);
    mpu.setXAccelOffset(-3000);
    mpu.setYAccelOffset(+63);
    mpu.setZAccelOffset(990); 

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        dmpReady = true;

        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

     
   int16_t a[3] = {0,0,0};
   int16_t g[3] = {0,0,0};
   int16_t m[3] = {0,0,0};
   initializer.getMotion9(&a[0], &a[1], &a[2], &g[0], &g[1], &g[2], &m[0], &m[1], &m[2]);
   delay(1000);
}


void updateMagnetometer(){
  magCounter++;
  if(magCounter>3){
    magCounter=0;
    mag.getHeading(&raw[0], &raw[1], &raw[2]);
  }
  for(int i=0; i<3; i++){
    float targetValue = (raw[i]+magOff[i])*magScale[i];
    correctedMag[i]+= (targetValue - correctedMag[i])/10.0f;
  }
}



void updateMagnetometerValues(){
    for(int i=0; i<3; i++){
      if(calibrateNow){
          if(raw[i]>magMax[i]){
            magMax[i] = raw[i];
          }
          if(raw[i]<magMin[i]){
            magMin[i] = raw[i];
          }
      }
      magOff[i] = -(magMax[i]+magMin[i])/2.0f;
      magScale[i] = 100.0f/(magMax[i]-magMin[i]);
    }
}



void getMagnetometerAngle(){
  magneticNorth[0] = atan2(correctedMag[0],correctedMag[1]);
  magneticNorth[1] = atan2(correctedMag[2],sqrt( sq(correctedMag[0]) + sq(correctedMag[1])));
  //magneticNorth[1] = atan2(correctedMag[2],correctedMag[0]);
}


void readDMP(){
// --------------- Reading DMP data ------------------
    if (!dmpReady) return;
    fifoCount = mpu.getFIFOCount();
    if (fifoCount == 1024) {
        mpu.resetFIFO();        
        Serial.println(F("FIFO has been overflown!"));
    } else if(fifoCount >= packetSize){
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        int16_t rotX, rotY, rotZ; 
        mpu.getRotation(&rotX, &rotY, &rotZ);
        rotationYaw = ((float)rotZ)/131.0f;
        rotationPitch = ((float)rotY)/131.0f;
        rotationRoll = ((float)rotX)/131.0f;
        mpu.resetFIFO();
   }

   updateMagnetometer();
   getMagnetometerAngle();
   north = toEuler(addToQuaternion(q, magneticNorth[0], magneticNorth[1], 0));
}


void convergeToNorth(){
  //---------------- Absolute Orientation ----------------------
   if(millis()<7000){
      northOffset = -north.x+magneticDeclination;
   }else{
      float targetOffset = -north.x+magneticDeclination;
      northOffset+=(targetOffset - northOffset)/700.0f;
   }
   Quaternion correctedRotation = getQuaternion(northOffset,0,0).getProduct(q); // Apply Offset to Dmp Rotation
   VectorFloat lastEuler = eulerRotation;
   eulerRotation = toEuler(correctedRotation);
   
   float maxAllowance = 0.380;  // 20 LSB/DEG/SEC
   if(millis()<7000){
    maxAllowance = 100;
   }
   
   if(abs(eulerRotation.x-lastEuler.x)>maxAllowance || abs(eulerRotation.y-lastEuler.y)>maxAllowance || abs(eulerRotation.z-lastEuler.z)>maxAllowance){
      Serial.println("it happened...");
      if(!lastWasSpike && !lastlastWasSpike){
        eulerRotation = lastEuler;
      }
      lastlastWasSpike = lastWasSpike;
      lastWasSpike=true;
   }else{
      lastlastWasSpike = lastWasSpike;
      lastWasSpike=false;
   }
}


void calibrateMagnetometer(){
    for(int i=0; i<3; i++){
      magMax[i] = -100000000;
      magMin[i] = 100000000;
    }
    //record max/min values for 30 seconds
     while(millis()<30000){
      Serial.print("Time to finish: ");Serial.println(floor((30000.0-millis())/1000.0));
      updateMagnetometer();
      updateMagnetometerValues();
     }
     //Print values
     Serial.println(F("max Values ")); Serial.println(magMax[0]); Serial.println(magMax[1]); Serial.println(magMax[2]);
     Serial.println(F("min Values ")); Serial.println(magMin[0]); Serial.println(magMin[1]); Serial.println(magMin[2]);
     
     delay(5000);
    updateMagnetometerValues();
}




