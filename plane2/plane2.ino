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

// ----- Rotation information and Plane stability -----
Quaternion currentRotation;
boolean first = true;
Quaternion targetRotation;
VectorFloat targetEuler(0,20,0);

//              pitch | roll
float pGain[2] = {0.0, 0.0};
float iGain[2] = {0.0, 0.0};
float dGain[2] = {0.0, 0.0};

float error[2] = {0.0, 0.0};   // pitch and roll
float lastError[2] = {0.0, 0.0}; // last pitch and roll error
float accError[2] = {0.0, 0.0};





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

  float pD = dGain[0] * (error[0] - lastError[0]);
  float rD = dGain[1] * (error[1] - lastError[1]);

  pitchOutput = pP + accError[0] + pD;
  rollOutput = rP + accError[1] + rD;
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
        } else{
          Serial.print(F("DMP initialization failed code: "));
          Serial.print(devStatus);
          Serial.print(F("\n"));
        }
}

void loop() {
        if(!dmpReady) return;
        
        // ---------- Transmitter communication -----------
        if(Serial1.available() > 0){  
          char selector = Serial1.read();
    
          if(selector == 'y'){
            joyY = map(Serial1.parseFloat(),10,90,-10,10);
            timestampWhenLastConnected=millis();
          }
    
          if(selector == 'x'){
            joyX = map(Serial1.parseFloat(),10,90,-10,10);
            timestampWhenLastConnected=millis();
          }
    
          if(selector == 't'){
            pot1 = map(Serial1.parseFloat(),10,90,0,100);
            timestampWhenLastConnected=millis();
          }
    
          if(selector == 'o'){
            timestampWhenLastConnected=millis();
          }
        }

        if(millis()-timestampWhenLastConnected>1000){
          connectionLost();
        }else{
          if(millis()-timestampDataProcessed>=20){
            control();
            timestampDataProcessed=millis();
          }
        }

        // --------- Gyro Processing ---------
        if(mpuInterrupt || fifoCount>=packetSize){
          mpuInterrupt = false;
          mpuIntStatus = mpu.getIntStatus();
        
          fifoCount = mpu.getFIFOCount();
          
          if((mpuIntStatus & 0x10) || fifoCount==1024){
            mpu.resetFIFO();
            Serial.println(F("FIFO OVERFLOW!"));
          } else if(mpuIntStatus & 0x02){
            while(fifoCount < packetSize){
              fifoCount = mpu.getFIFOCount();
              Serial.println("while loop!");
            }
        
            mpu.getFIFOBytes(fifoBuffer, packetSize);
            fifoCount-=packetSize;
            mpu.dmpGetQuaternion(&currentRotation, fifoBuffer);
            
            if(first){
              first = false;
              targetEuler.x = toEuler(currentRotation).x;
            }
            
            //Calculate Errors and do PID
            float yawDifference = toEuler(getDifference(getQuaternion(toEuler(currentRotation).x,0,0),getQuaternion(toEuler(targetRotation).x,0,0))).x;
            error[0] = toEuler(targetRotation).y - toEuler(currentRotation).y;
            error[1] = (map(yawDifference,-180,180,-70,70))-toEuler(currentRotation).z;
            
            pid();
            
            lastError[0] = error[0];
            lastError[1] = error[1];

            VectorFloat tr = toEuler(targetRotation);
            Serial.print("TargetRotation\t");
            Serial.print(yawDifference);
            Serial.print("\t");
            Serial.print(error[0]);
            Serial.print("\t");
            Serial.println(error[1]);

            // --- TODO --- 
            
      
            
          }
        }
}





