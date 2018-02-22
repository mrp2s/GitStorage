float pGain[3] = {0,160,170};  // 170, 165
float dGain[3] = {0,9 ,2};   // 5 2
float iGain[3] = {0,0.0,0.0}; //0.2

float acc[3] = {0,0,0};
float maxI = 20;
unsigned long timer = 0;

void setPitch(float value){
  Serial3.print('p');Serial3.println(value);
}

void setRoll(float value){
  Serial3.print('r');Serial3.println(value);
}

void setYaw(float value){
  Serial3.print('y');Serial3.println(value);
}

void setFlaps(float value){
  Serial3.print('f');Serial3.println(value);
}




void PID(float yawError, float pitchError, float rollError){
  //yaw
  float yawOutput = -rotationYaw*dGain[0];
  
  //pitch [1]
  acc[1]+=iGain[1]*pitchError;
  acc[1] = constrain(acc[1], -maxI, maxI);
  float pitchOutput = pitchError*pGain[1] - rotationPitch*dGain[1] + acc[1];
  

  //roll [2]
  acc[2]+=iGain[2]*rollError;
  acc[2] = constrain(acc[2], -maxI, maxI);
  float rollOutput = rollError*pGain[2] - rotationRoll*dGain[2] + acc[2];
  

  // updateServos
  setPitch(constrain(pitchOutput, -90, 90));
  setRoll(constrain(rollOutput, -90, 90));
  setYaw(constrain(yawOutput, -90, 90));
}


void flyToAngle(float targetYaw, float targetPitch, float targetRoll){
  float pitchError = cos(eulerRotation.z) * (targetPitch-eulerRotation.y);
  float rollError = targetRoll - eulerRotation.z;
  if(abs(rollError)>180){
    rollError = rollError - 360;
  }
  PID(0, pitchError, rollError);
}
