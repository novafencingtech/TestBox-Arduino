void setEepromLocation() {
  int k=0;
    
  while (k<EEPROM.length()) {
    if (EEPROM.read(k)==calibrationValid){
      eepromAddr=k;
      break;
    }
    k+=eepromLocationStep;
  }
  if (k>=(EEPROM.length()-eepromLocationStep)) {
    eepromAddr=0;
  } 
}

void writeCalibrationData() {
  int loc;

  if ((EEPROM.read(eepromAddr)==calibrationValid)) {
    EEPROM.update(eepromAddr,calibrationInvalid); //Mark old calibration as invalid
    eepromAddr+=eepromLocationStep; //Move to next block of memory
    if (eepromAddr>=(EEPROM.length()-eepromLocationStep)) {
      eepromAddr=0;
    }
  } else {
    setEepromLocation();
  }

  loc=eepromAddr;
  EEPROM.update(loc,calibrationValid);
  loc++;
  for (int k=0;k<NUM_ADC_SCAN_CHANNELS; k++) {
    EEPROM.update(loc, ChanArray[k].getTrim());
    loc++;
  }
}

void loadCalibrationData() {
  byte calData;
  int loc=0;
  bool calValid=false;

  //If we don't have valid calibration re-check the location
  if (!(EEPROM.read(eepromAddr)==calibrationValid)) {
    setEepromLocation();
  }
  
  loc=eepromAddr;

  calValid=(EEPROM.read(loc)==calibrationValid); 

  loc++; //Move to next location
  for (int k=0;k<NUM_ADC_SCAN_CHANNELS; k++) {
    if (calValid) {
      calData=EEPROM.read(loc);
      if (calData>calibrationErrorValue) {
        calData=0;
      }
    } else {
      calData=0;
    }    
    ChanArray[k].setTrim(calData);
    loc++;    
  }  
}
