int joyX;
int joyY;
int throttle;

int joyXPin = 0;
int joyYPin = 2;
int throttlePin = 1;

void setup(){
  Serial.begin(9600);
  Serial1.begin(9600);
}

void loop(){
  joyX = int(map(analogRead(joyXPin),0,1023,10,90));
  joyY = int(map(analogRead(joyYPin),0,1023,10,90));
  throttle = int(map(analogRead(throttlePin),0,1023,10,90));
  
  Serial.println(analogRead(throttlePin));
  
  String xData=" x";
  xData+=joyX;
  Serial1.print(xData);
  delay(50);
  
  String yData=" y";
  yData+=joyY;
  Serial1.print(yData);
  delay(50);
  
  String tData=" t";
  tData+=throttle;
  Serial1.print(tData);
  delay(50);
}
