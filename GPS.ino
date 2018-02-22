const unsigned char ubxRate10Hz[] PROGMEM =
  { 0x06,0x08,0x06,0x00,100,0x00,0x01,0x00,0x01,0x00 };
const unsigned char ubxDisableGGA[] PROGMEM =
  { 0x06,0x01,0x08,0x00,0xF0,0x00,0x00,0x00,0x00,0x00,0x00,0x01 };
const unsigned char ubxDisableGSA[] PROGMEM =
  { 0x06,0x01,0x08,0x00,0xF0,0x02,0x00,0x00,0x00,0x00,0x00,0x01 };
const unsigned char ubxDisableGSV[] PROGMEM =
  { 0x06,0x01,0x08,0x00,0xF0,0x03,0x00,0x00,0x00,0x00,0x00,0x01 };
const unsigned char ubxDisableRMC[] PROGMEM =
  { 0x06,0x01,0x08,0x00,0xF0,0x04,0x00,0x00,0x00,0x00,0x00,0x01 };
const unsigned char ubxDisableVTG[] PROGMEM =
  { 0x06,0x01,0x08,0x00,0xF0,0x05,0x00,0x00,0x00,0x00,0x00,0x01 };
const unsigned char ubxDisableZDA[] PROGMEM =
  { 0x06,0x01,0x08,0x00,0xF0,0x08,0x00,0x00,0x00,0x00,0x00,0x01 };

uint8_t LastSentenceInInterval = 0xFF; // storage for the run-time selection
const uint32_t COMMAND_DELAY = 250;

void setupGPS(){
  onlyGLL();
  gpsPort.begin(9600);
  sendUBX( ubxRate10Hz, sizeof(ubxRate10Hz) );
}


void onlyGLL(){
  sendUBX( ubxDisableRMC, sizeof(ubxDisableRMC) );
    delay( COMMAND_DELAY );
  //sendUBX( ubxDisableGLL, sizeof(ubxDisableGLL) );
  sendUBX( ubxDisableGSV, sizeof(ubxDisableGSV) );
    delay( COMMAND_DELAY );
  sendUBX( ubxDisableGSA, sizeof(ubxDisableGSA) );
    delay( COMMAND_DELAY );
  sendUBX( ubxDisableGGA, sizeof(ubxDisableGGA) );
    delay( COMMAND_DELAY );
  sendUBX( ubxDisableVTG, sizeof(ubxDisableVTG) );
    delay( COMMAND_DELAY );
  sendUBX( ubxDisableZDA, sizeof(ubxDisableZDA) );
  LastSentenceInInterval = NMEAGPS::NMEA_GLL;
}


void sendUBX( const unsigned char *progmemBytes, size_t len ){
  gpsPort.write( 0xB5 ); // SYNC1
  gpsPort.write( 0x62 ); // SYNC2

  uint8_t a = 0, b = 0;
  while (len-- > 0) {
    uint8_t c = pgm_read_byte( progmemBytes++ );
    a += c;
    b += a;
    gpsPort.write( c );
  }

  gpsPort.write( a ); // CHECKSUM A
  gpsPort.write( b ); // CHECKSUM B

}


void handleGPS(){
    //set the starting position
    if(!startingPositionSet && latestLat[9]!=0 && millis()>9000){
      startingPositionSet = true;
      float total[2];
      for(int i=0; i<10; i++){
        total[0]+=latestLat[i];
        total[1]+=latestLon[i];
      }
      startingPosition[0] = total[0]/10.0f;
      startingPosition[1] = total[1]/10.0f;
      Serial.print("\nStartingPosition set, ");Serial.print(startingPosition[0],6);Serial.print(", ");Serial.println(startingPosition[1],6);
    }

    // -- read the gps position --
    while (gps.available(gpsPort)) {
        float timeLog = millis();
        fix = gps.read();
    
        //Serial.print( F("Location: ") );
        if (fix.valid.location) {
            //DEBUG_PORT.print( currentPosition[0], 6 );
            //DEBUG_PORT.print( ", " );
            //DEBUG_PORT.println( currentPosition[1], 6 );
            //Serial.print( F("Starting Location: ") );
            //DEBUG_PORT.print( startingPosition[0], 6 );
            //DEBUG_PORT.print( ", " );
            //DEBUG_PORT.println( startingPosition[1], 6 );
            insertToStorage(fix.latitude(), fix.longitude());
            updateGrid(currentPosition[0], currentPosition[1], startingPosition[0], startingPosition[1]);
        }
        
        //Serial.print(",    Milliseconds spent: ");Serial.println(millis()-timeLog);
    }
}


void insertToStorage(float lat, float lon){
  // -------- store the last second of gps values in -----------
  for(int i=8; i>=0; i--){
    latestLat[i+1] = latestLat[i];
    latestLon[i+1] = latestLon[i];
  }
  latestLat[0] = lat;
  latestLon[0] = lon;
 
  // ---------- calculate current position using a weighted avrage ----------------- 
  currentPosition[0] = 0;
  currentPosition[1] = 0;
  
  double w[] = {4.0,3.0,2.0,1.0};
  double totalWeights = 0;
  for(int i=0; i<4; i++){
    totalWeights+=w[i];
    currentPosition[0]+=w[i]*latestLat[i];
    currentPosition[1]+=w[i]*latestLon[i];
  }
  currentPosition[0]/=totalWeights;
  currentPosition[1]/=totalWeights;
}



float updateGrid(float cPosLat, float cPosLon, float sPosLat, float sPosLon){
  double latMid = (cPosLat+sPosLat)/2.0;
  
  double meterPerDegLat = 111319.444444;
  double meterPerDegLon = 111130.555556 * cos(latMid*PI/180.0);
  
  double deltaLat = cPosLat - sPosLat;
  double deltaLon = cPosLon - sPosLon;

  currentPositionGrid[1] = deltaLat * meterPerDegLat;
  currentPositionGrid[0] = deltaLon * meterPerDegLon;
}





