ADC_Channel* getCalibrationChannel(int num) {
  switch (num) {
    case 0 ... (NUM_ADC_SCAN_CHANNELS-1):
      return &(ChanArray[num]);
      break;
    case NUM_ADC_SCAN_CHANNELS:
      return &EpeeADC;
      break;
    case NUM_ADC_SCAN_CHANNELS+1:
      return &FoilADC;
      break;
    case NUM_ADC_SCAN_CHANNELS+2:
      return &WeaponAC;
      break;
  }
  //It should never get here, but just in case
  return &(ChanArray[0]);
}

void writeCalibrationData() {
  int loc;
  int p = 0;
  uint8_t buf[100];
  //String content = "";
  ADC_Channel *CalChannel;
  
  if (!isInitialized) {
    InternalFS.begin();
    isInitialized = true;
  }
  if ( file.open(FILENAME, FILE_O_WRITE) )
  {
    file.seek(0);
    p = 0;
    buf[p++] = NUM_CAL_CHANNELS;
    Serial.println("Write Calibration Data");
    for (int k = 0; k < NUM_CAL_CHANNELS; k++) {
      CalChannel=getCalibrationChannel(k);
      buf[p++] = highByte(CalChannel->getTrim());
      buf[p++] = lowByte(CalChannel->getTrim());
    }
    int fsiz = 1 + (NUM_CAL_CHANNELS * 2);
    int actual=file.write(buf, fsiz);
    file.close();
    if (actual!=fsiz) {
      Serial.print("Error writing calibration Data, tried ");Serial.print(fsiz);Serial.print(" but got ");Serial.println(actual);
      tft.print("Error Writing File "); tft.println(actual);
    }
    else Serial.print("Done writing Calibration Data ");Serial.println(fsiz);
  } else
  {
    Serial.println("Failed to open Calibration File");
    tft.println("Error Opening File");
  }
}



void loadCalibrationData() {
  byte calData;
  int loc = 0;
  bool calValid = false;
  int p = 0;
  uint8_t buf[100];
  char txtBuffer[50];
  ADC_Channel *CalChannel;
  
  if (!isInitialized) {
    InternalFS.begin();
    isInitialized = true;
  }
  if (file.open(FILENAME, FILE_O_READ)) {
    int cal;
    int l = file.read(buf, 100);
    p = 0;
    Serial.print("Read Calibration, got "); Serial.print(buf[p]); Serial.println(" channels");
    int n = buf[p++];
    for (int k = 0; k < NUM_CAL_CHANNELS; k++) {
      CalChannel=getCalibrationChannel(k);
      //if (k<n) cal = buf[p++] | (buf[p++] >> 8); 
      if (k<n) cal = (buf[p++] << 8) | (buf[p++]); 
      else {
        Serial.println("Fewer calibrations in file than in code");
        cal=0;
      }
      if (cal >= calibrationErrorValue) { 
        cal = 0;
        Serial.println("Error: invalid calibration value read");
      }
      CalChannel->setTrim(cal);
      sprintf(txtBuffer,"Cal Offset %s = %u",CalChannel->ch_label,cal);
      Serial.println(txtBuffer);
    }
    file.close();
  } else Serial.println("Can't open Calibration Data File");
}
