#include <MPU9150.h>
#include <AK8975.h>
#include <NMEAGPS.h>
#include <GPSport.h>
#include <MS5611.h>

#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include <Wire.h>
#endif


MPU6050 mpu(0x68);
// 8, 9, 10, 12

// ------------------- PLANE REMOTE CONTROL ------------
#define CHANNELS 8
#define INT_PIN 2
#define DEADZONE 15
float percentValues[CHANNELS];

// ------------------- Altimeter ------------------
MS5611 barometer = MS5611();
float initPressure = 0;
float initAltitude = 0;
float stableAlt = 0;
float altitudeOffset = 0;

// -------------------- Compass -------------------- 
AK8975 mag;
MPU9150 initializer;
float correctedMag[3] = {0,0,0};
int16_t raw[3] = {0,0,0};
float magOff[3] = {0,0,0};
float magScale[3] = {100,100,100};
float magMax[3] = {83,73,-12};
float magMin[3] = {-39,-53,-143};
float magneticNorth[2] = {0,0};
float northOffset = 0;

float magneticDeclination = 6.0; // Declination in Stockholm, Sweden.
boolean calibrateNow = false;         //Set true for calibration
/*
max Values 
83.00
73.00 
-12.00
min Values 
-43.00
-53.00
-143.00


max Values 
113.00
76.00
-13.00
min Values 
-42.00
-97.00
-158.00

 */

 // ----------------------- GPS ------------------------
NMEAGPS  gps; // This parses the GPS characters
gps_fix  fix; // This holds on to the latest values
double startingPosition[2] = {0,0};
double currentPosition[2] = {0,0};
float currentPositionGrid[3] = {0,0,0};
double latestLat[10];
double latestLon[10];
boolean startingPositionSet = false;


// -------------------- Debugging -------------------
bool dmpReady = false;

// ----------------------- DMP ----------------------
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer


// -------------------- Orientation -----------------
Quaternion q;           // [w, x, y, z]         dmp Rotation
VectorFloat eulerRotation;
VectorFloat north;

float cvt = 180.0/PI;


// ================================================================
// ===                         SETUP                            ===
// ================================================================

void setup() {
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000);
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    Serial.begin(115200);
    Serial3.begin(115200);
    delay(300);

    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    mag.initialize();
    initializer.initialize();

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
    Serial.println(mag.testConnection() ? "AK8975 connection successful" : "AK8975 connection failed");
    Serial.println(initializer.testConnection() ? "AK8975 connection successful" : "AK8975 connection failed");
    
    Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    while (Serial.available() && Serial.read()); // empty buffer

    Serial.println(F("Initializing DMP..."));
    initMpu();

    pinMode(13, OUTPUT);

   if(calibrateNow){
    calibrateMagnetometer();
   }
   updateMagnetometerValues();

   setupBarometer();
   setupPPM();
}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
   readDMP();
   convergeToNorth();
   handleGPS();
   updateBarometer();
   handlePPM();

   if(percentValues[7]>90){ //gear switch is off --> manual override
      setPitch(map(percentValues[1],-100,100,30,-30));
      setRoll(map(percentValues[0],-100,100,45,-45));
   }else{
      flyToAngle(0, (percentValues[5]/4.0f-15.0f)/cvt, 0);
   }
  
   debug();
}
