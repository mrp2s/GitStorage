#include <Servo.h>
#include <helper_3dmath.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <I2Cdev.h>
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// ----- BASIC Plane essentials -----
Servo ailRight;
Servo ailLeft;
Servo throttle;
int rotationRangeRight[] = {40,140};
int rotationRangeLeft[] = {40,140};

float rollOutput = 0;
float pitchOutput = 0; 
float trimLeft = 0;
float trimRight = 0;
float pD=0;
float rD=0;

float joyX = 0;
float joyY = 0;
float pot1 = 0;
float joyXSensitivity = 0.1;
float joyYSensitivity = 0.1;
unsigned long timestampWhenLastConnected = 0;
unsigned long timestampDataProcessed = 0;

// ----- MPU6050 essentials -----
MPU6050 mpu;
#define INTERRUPT_PIN 7
bool dmpReady = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];
bool lastWasAWhileLoop=false;  

// ----- Rotation information and Plane stability -----
Quaternion currentRotation;
bool first = true;
Quaternion targetRotation;
VectorFloat targetEuler(0,0,0);

//              pitch | roll
float pGain[2] = {0.0, 0.0};
float iGain[2] = {0.0, 0.0};
float dGain[2] = {90.0, 90.0};

float error[2] = {0.0, 0.0};   // pitch and roll
float lastError[2] = {0.0, 0.0}; // last pitch and roll error
float delErr[2] = {0.0, 0.0};
float accError[2] = {0.0, 0.0};
float resetCounter = 0;

float lastDeltaError[2] = {0.0, 0.0}; // to remove random spikes
float lastlastDeltaError[2] = {0.0, 0.0};


// ----- Interrupt -----
volatile bool mpuInterrupt = false;
void dmpDataReady(){
  mpuInterrupt = true;
}


// --------------------------   PID   PID   PID   -----------------------------------
void pid(){ 
  float pP = pGain[0] * error[0];
  float rP = pGain[1] * error[1];

  accError[0]+=iGain[0]*error[0];
  accError[1]+=iGain[1]*error[1];

  float anomalyMax = 50;
  float anomalyMin = -50;

  float deltaPitch = (dGain[0] * (error[0] - lastError[0]));
  float deltaRoll = (dGain[1] * (error[1] - lastError[1]));
  if(deltaPitch<anomalyMax && deltaPitch>anomalyMin){  
  }else{
    //Serial.println("Anomaly");
  }
  if(deltaRoll<anomalyMax && deltaRoll>anomalyMin){  
  }else{
    //Serial.println("Anomaly");
  }
  pD = deltaPitch;
  rD = deltaRoll;
  
  pitchOutput = pP + accError[0] + pD;
  rollOutput = rP + accError[1] + rD;

  //max min values
  if(pitchOutput>100){pitchOutput=100;}
  if(rollOutput>100){rollOutput=100;}
  if(pitchOutput<-100){pitchOutput=-100;}
  if(rollOutput<-100){rollOutput=-100;}
  
  left(pitchOutput+rollOutput);
  right(pitchOutput-rollOutput);
}

// --------------------------------- Plane -------------------------------
void motor(int percentage){
  int val = map(percentage,0,100,65,165);
  throttle.write(val);
}

void control(){
  float pitch = joyY*joyYSensitivity;
  float yaw = joyX*joyXSensitivity;
  targetEuler.x+=yaw;
  targetEuler.y+=pitch;
  targetRotation = getQuaternion(targetEuler.x, targetEuler.y, targetEuler.z);
}

void left(float percentage){
  ailLeft.write(map(percentage,-100,100,15,165)+trimLeft);
}

void right(float percentage){
  ailRight.write(map(percentage,-100,100,165,15)+trimRight);
}

void connectionLost(){
  
}
// ------------- Quaternion ----------------------
VectorFloat toEuler(Quaternion q){
        double yaw;
        double pitch;
        double roll;
       
        double sinr = 2.0 * (q.w * q.x + q.y * q.z);
        double cosr = 1.0 - 2.0 * (q.x * q.x + q.y * q.y);
        roll = atan2(sinr, cosr);
      
        double sinp = 2.0 * (q.w * q.y - q.z * q.x);
        if(fabs(sinp)>=1){
          if(sinp>=0){
            pitch = PI/2;
          }else if (sinp<0){
            pitch = -PI/2;
          }
        }else{
          pitch = asin(sinp);
        }
      
        double siny = 2.0 * (q.w * q.z + q.x * q.y);
        double cosy = 1.0 - 2.0 * (sq(q.y) + sq(q.z));
        yaw = atan2(siny,cosy);
      
        return VectorFloat(degrees(yaw), degrees(pitch), degrees(roll));
}



Quaternion getDifference(Quaternion frameOfRefrence, Quaternion target){
        Quaternion diff = frameOfRefrence.getConjugate().getProduct(target);
        return diff;
}



Quaternion addToQuaternion(Quaternion q, float yaw, float pitch, float roll){ 
        Quaternion extraRotation = getQuaternion(yaw, pitch, roll);
        Quaternion result = q.getProduct(extraRotation);
        return result;
}



Quaternion getQuaternion(float yaw, float pitch, float roll){
        Quaternion newQuaternion;
        double rad = PI/180;
        yaw*=rad;
        pitch*=rad;
        roll*=rad;
        
        double cy = cos(yaw * 0.5);
        double sy = sin(yaw * 0.5);
        double cr = cos(roll * 0.5);
        double sr = sin(roll * 0.5);
        double cp = cos(pitch * 0.5);
        double sp = sin(pitch * 0.5);
      
        
        newQuaternion.w = cr * cy * cp + sr * sy * sp;
        newQuaternion.z = cr * sy * cp - sr * cy * sp;
        newQuaternion.y = cr * cy * sp + sr * sy * cp;
        newQuaternion.x = sr * cy * cp - cr * sy * sp;
        return newQuaternion;
}


void setup() {
        // ------ Plane Setup ---------
        Serial1.begin(9600);
        Serial1.setTimeout(5);
        ailRight.attach(10);
        ailLeft.attach(5);
        throttle.attach(9);
        motor(0);
        
        //Setup MPU
        #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
              Wire.begin();
              Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
        #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
              Fastwire::setup(400, true);
        #endif
        Serial.begin(115200);
      
        Serial.println(F("Initializing i2c devices..."));
        mpu.initialize();
        pinMode(INTERRUPT_PIN, INPUT);
      
        Serial.println(F("Testing device connections..."));
        Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
      
        Serial.println(F("Initializing DMP..."));
         devStatus = mpu.dmpInitialize();
      
        mpu.setXGyroOffset(-37);
        mpu.setYGyroOffset(-7);
        mpu.setXGyroOffset(4);
      
        mpu.setXAccelOffset(1337);
        mpu.setYAccelOffset(-2364);
        mpu.setZAccelOffset(2290);
        
        if(devStatus == 0){
          Serial.println(F("Enabling DMP..."));
          mpu.setDMPEnabled(true);
      
          Serial.println(F("Enabling Interrupt detection"));
          attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN),dmpDataReady, RISING);
          mpuIntStatus = mpu.getIntStatus();
      
          Serial.println(F("DMP ready! Waiting for first interrupt..."));
          dmpReady = true;
      
          packetSize = mpu.dmpGetFIFOPacketSize();
          Serial.println(packetSize);
        } else{
          Serial.print(F("DMP initialization failed code: "));
          Serial.print(devStatus);
          Serial.print(F("\n"));
        }
}

void loop() {
        if(!dmpReady) return;
        
        // ---------- Transmitter communication -----------
        /*
        if(Serial1.available() > 0){  
          char selector = Serial1.read();
    
          if(selector == 'y'){
            joyY = map(Serial1.parseFloat(),10,90,-10,10);
            if(joyY>10){joyY=10;}
            if(joyY<-10){joyY=-10;}
            timestampWhenLastConnected=millis();
          }
    
          if(selector == 'x'){
            joyX = map(Serial1.parseFloat(),10,90,-10,10);
            if(joyX>10){joyX=10;}
            if(joyX<-10){joyX=-10;}
            timestampWhenLastConnected=millis();
          }
    
          if(selector == 't'){
            pot1 = map(Serial1.parseFloat(),10,90,0,100);
            timestampWhenLastConnected=millis();
          }
    
          if(selector == 'o'){
            timestampWhenLastConnected=millis();
          }
        }*/

        if(millis()-timestampWhenLastConnected>1000){
          connectionLost();
        }else{
          if(millis()-timestampDataProcessed>=20){
            control();
            timestampDataProcessed=millis();
          }
        }

        // --------- Gyro Processing ---------
        fifoCount = mpu.getFIFOCount();
        if(mpuInterrupt || fifoCount>=packetSize){
          mpuInterrupt = false;
          mpuIntStatus = mpu.getIntStatus();
         
          
          if((mpuIntStatus & 0x10) || fifoCount==1024){
            mpu.resetFIFO();
            Serial.println(F("FIFO OVERFLOW!"));
          } else if(mpuIntStatus & 0x02){
            if(fifoCount < packetSize){
              fifoCount = mpu.getFIFOCount();
              Serial.println(F("while loop!"));
              lastWasAWhileLoop=true;
              return;
            }
        
            mpu.getFIFOBytes(fifoBuffer, packetSize);
            fifoCount-=packetSize;
            mpu.dmpGetQuaternion(&currentRotation, fifoBuffer);
            
            if(first){
              first = false;
              targetEuler.x = toEuler(currentRotation).x;
            }
            
            if(lastWasAWhileLoop){
              lastWasAWhileLoop=false;
              return;
            }
            //Calculate Errors and do PID
            VectorFloat currentVector = toEuler(currentRotation);
            VectorFloat targetVector = toEuler(targetRotation);

            float yawDifference = toEuler(getDifference(getQuaternion(currentVector.x,0,0),getQuaternion(targetVector.x,0,0))).x;
            error[0] = targetVector.y - currentVector.y;
            error[1] = (map(yawDifference,-180,180,-70,70))-currentVector.z;
            float deltaError[] = {error[0]-lastError[0], error[1]-lastError[1]};
            
            float alt1ChangeDeltaError[2] = {deltaError[0]-lastDeltaError[0], deltaError[1]-lastDeltaError[1]};
            float alt2ChangeDeltaError[2] = {(deltaError[0]-lastlastDeltaError[0])/2.0, (deltaError[1]-lastlastDeltaError[1])/2.0};
            float alt3ChangeDeltaError[2] = {lastDeltaError[0]-lastlastDeltaError[0], lastDeltaError[1]-lastlastDeltaError[1]};

            
            for(int i=0; i<2; i++){
                float v[3] = {alt1ChangeDeltaError[i], alt2ChangeDeltaError[i], alt3ChangeDeltaError[i]};
                float smallestAbs = min(min(abs(v[0]),abs(v[1])),abs(v[2]));
                for(int j=0; j<3; j++){
                  if(abs(v[j]) == smallestAbs){
                    delErr[i]+=v[j];
                    break;
                  }
                }
            }     

            resetCounter+=1;
            if(resetCounter>10){
              if(abs(delErr[0]-deltaError[0])>0.9 || abs(delErr[1]-deltaError[1])>0.9){
                
              }else{
                resetCounter=0;
                delErr[0]=deltaError[0];
                delErr[1]=deltaError[1];
              }
            }
            
            pid();
            /*Serial.print(pitchOutput);
            Serial.print("\t");
            Serial.print(rollOutput);
            Serial.print("\t");
            Serial.print(error[0]);
            Serial.print("\t");
            Serial.print(error[1]);
            
            Serial.print("\t\t");*/
            Serial.print(delErr[1]*10);
            //Serial.print("\t");
            //Serial.print(deltaError[1]*10);
            Serial.print("\t");
            Serial.print(0);
            Serial.print("\t");
            Serial.print(0);

            
            //Serial.print(changeDeltaError[0]*100);
            //Serial.print("\t");
            
            //Serial.print(fifoCount);
            
            Serial.println();
           lastError[0] = error[0];
           lastError[1] = error[1];
           
           lastlastDeltaError[0] = lastDeltaError[0];
           lastlastDeltaError[1] = lastDeltaError[1];
           lastDeltaError[0] = deltaError[0];
           lastDeltaError[1] = deltaError[1];
          }
        }
}





