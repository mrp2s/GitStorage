#include <helper_3dmath.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <I2Cdev.h>


#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 mpu;

#define INTERRUPT_PIN 7
#define LED_PIN 13
bool blinkState = false;

bool dmpReady = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];

Quaternion currentRotation;
Quaternion targetRotation;
VectorInt16 aa;
VectorInt16 aaReal;
VectorInt16 aaWorld;
VectorFloat gravity;
float euler[3];
float ypr[3];
float ypr2target[3];


// ---- Interrupt ---
volatile bool mpuInterrupt = false;
void dmpDataReady(){
  mpuInterrupt = true;
}

VectorFloat toEuler(Quaternion q){
  double yaw;
  double pitch;
  double roll;
 
  double sinr = +2.0 * (q.w * q.x + q.y * q.z);
  double cosr = +1.0 - 2.0 * (q.x * q.x + q.y * q.y);
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

void setTargetRotation(float yaw, float pitch, float roll){
  targetRotation = getQuaternion(yaw, pitch, roll);
}

void addToTargetRotation(float yaw, float pitch, float roll){
  targetRotation = addToQuaternion(targetRotation, yaw, pitch, roll);
}




void setup() {
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

  Serial.println(F("\n send any character to start..."));
  while(Serial.available() && Serial.read());
  while(!Serial.available());
  while(Serial.available() && Serial.read());

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

  pinMode(LED_PIN, OUTPUT);
}

void loop() {
  if(!dmpReady) return;

  while(!mpuInterrupt && fifoCount<packetSize){}

  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  fifoCount = mpu.getFIFOCount();

  if((mpuIntStatus & 0x10) || fifoCount==1024){
    mpu.resetFIFO();
    Serial.println(F("FIFO OVERFLOW!"));
  } else if(mpuIntStatus & 0x02){
    while(fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    mpu.getFIFOBytes(fifoBuffer, packetSize);
    fifoCount-=packetSize;

    mpu.dmpGetQuaternion(&currentRotation, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &currentRotation);
    mpu.dmpGetYawPitchRoll(ypr, &currentRotation, &gravity);
    
    /*Serial.print("ypr\t");
    Serial.print(ypr[0] * 180/M_PI);
    Serial.print("\t");
    Serial.print(ypr[1] * 180/M_PI);
    Serial.print("\t");
    Serial.println(ypr[2] * 180/M_PI);*/

    //ypr
    Quaternion target = getQuaternion(0,0,0);
    Quaternion angle2target = getDifference(currentRotation,target);
    //Quaternion q = getQuaternion(90,180,-76.56);
    VectorFloat E = toEuler(angle2target);


    Serial.print("euler\t");
    Serial.print(E.x);
    Serial.print("\t");
    Serial.print(E.y);
    Serial.print("\t");
    Serial.println(E.z);
    
    /*
    Serial.print("target\t");
    Serial.print(q.w);
    Serial.print("\t");
    Serial.print(q.x);
    Serial.print("\t");
    Serial.print(q.y);
    Serial.print("\t");
    Serial.println(q.z);
    
    Serial.print("quat\t");
    Serial.print(currentRotation.w);
    Serial.print("\t");
    Serial.print(currentRotation.x);
    Serial.print("\t");
    Serial.print(currentRotation.y);
    Serial.print("\t");
    Serial.println(currentRotation.z);*/
    
    

    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
  }
}





