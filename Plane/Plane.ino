#include <Servo.h>

Servo ailRight;
Servo ailLeft;

Servo throttle;

float pitch=0;
float roll=0;
float gas=0;

float leftAngle=90;
float rightAngle=90;

int MaxAngle=140;
int MinAngle=40;

long timeLog=0;

void setup() {

  //Serial.begin(57600);
  //Serial.setTimeout(5);

  Serial1.begin(9600);
  Serial1.setTimeout(5);
  
  ailRight.attach(2);
  ailLeft.attach(3);
  throttle.attach(7);
  motor(0);
  timeLog=millis();
}



void loop() {
  
  if(Serial1.available() > 0){    
    char selector = Serial1.read();
    
    if(selector == 'y'){
    pitch = map(Serial1.parseFloat(),10,90,MinAngle,MaxAngle);
    }
    
    if(selector == 'x'){
    roll = map(Serial1.parseFloat(),10,90,-50,50);
    }
    
    if(selector == 't'){
    gas = map(Serial1.parseFloat(),10,90,0,100);
    }
    
    if(selector == 'o'){
      timeLog=millis();
    }
  }
  
  if((millis()-timeLog)>2000){
    ailLeft.write(135);
    ailRight.write(45);
    motor(0);
  }else{
    leftAngle=pitch+roll;
    rightAngle=map(pitch,MinAngle,MaxAngle,MaxAngle,MinAngle)+roll;
    checkAngles();
    ailLeft.write(leftAngle+25);
    ailRight.write(rightAngle-25);
    motor(gas);
  }
}

void checkAngles(){
  if(leftAngle>MaxAngle){
    leftAngle=MaxAngle;
  }
  if(leftAngle<MinAngle){
    leftAngle=MinAngle;
  }
  
  if(rightAngle>MaxAngle){
    rightAngle=MaxAngle;
  }
  if(rightAngle<MinAngle){
    rightAngle=MinAngle;
  }
}

void motor(int percentage){
  int val = map(percentage,0,100,65,165);
  throttle.write(val);
}
