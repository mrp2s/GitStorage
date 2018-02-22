volatile uint16_t rawValues[CHANNELS];

void setupPPM(){
  resetChannels();
  attachInterrupt(digitalPinToInterrupt(INT_PIN), ppm, RISING);
}


void resetChannels(){
  for(int i=0; i<CHANNELS; i++){
    rawValues[i] = 1500;
    percentValues[i] = 0;
  }
}


void handlePPM(){ 
  for(int i=0; i<CHANNELS; i++){
    percentValues[i] = map(rawValues[i], 1068.0f, 1920.0f, -100.0f, 100.0f); //map input --> 0 to 100 %
  }
}


void ppm(){
  uint16_t now, diff;
  static uint16_t lastMicros = 0;
  static uint8_t currentChannel = 0;

  now = micros();
  sei();
  diff = now - lastMicros;
  lastMicros = now;
  if(diff>3000){
    currentChannel = 0;
  }else{
    if(diff>900 && diff<2500 && currentChannel<CHANNELS){
      rawValues[currentChannel] = diff;
    }
    currentChannel++;
  }
}
