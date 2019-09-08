#include <Adafruit_LittleFS.h>
#include <InternalFileSystem.h>

using namespace Adafruit_LittleFS_Namespace;

#define FILENAME    "/calibration.txt"

File file(InternalFS);
int p = 0;
uint8_t buf[100];
bool isInitialized = false;

void setEepromLocation() {
  int k = 0;
  /*

    while (k<EEPROM.length()) {
    if (EEPROM.read(k)==calibrationValid){
      eepromAddr=k;
      break;
    }
    k+=eepromLocationStep;
    }
    if (k>=(EEPROM.length()-eepromLocationStep)) {
    eepromAddr=0;
    } */
}


void writeCalibrationData() {
  int loc;
  String content = "";
  if (!isInitialized) {
    InternalFS.begin();
    isInitialized = true;
  }
  if ( file.open(FILENAME, FILE_O_WRITE) )
  {
    p = 0;
    buf[p++] = NUM_ADC_SCAN_CHANNELS;
    Serial.println("Write Calibration Data");
    for (int k = 0; k < NUM_ADC_SCAN_CHANNELS; k++) {
      buf[p++] = lowByte(ChanArray[k].getTrim());
      buf[p++] = highByte(ChanArray[k].getTrim());
    }
    file.write(buf, 1 + (NUM_ADC_SCAN_CHANNELS * 2));
    file.close();
  } else
  {
    Serial.println("Failed to open Calibration File");
  }

  Serial.println("Done writing Calibration Data");
}



void loadCalibrationData() {
  byte calData;
  int loc = 0;
  bool calValid = false;
  if (!isInitialized) {
    InternalFS.begin();
    isInitialized = true;
  }
  if (file.open(FILENAME, FILE_O_READ)) {
    uint16_t cal;
    int l = file.read(buf, 100);
    p = 0;
    Serial.print("Read Calibration, got "); Serial.print(buf[p]); Serial.println(" channels");
    int n = buf[p++];
    for (int k = 0; k < NUM_ADC_SCAN_CHANNELS; k++) {
      if (k<n) cal = buf[p++] | (buf[p++] >> 8); 
      else {
        Serial.println("Fewer calibrations in file than in code");
        cal=0;
      }
      if (cal >= calibrationErrorValue) cal = 0;
      ChanArray[k].setTrim(cal);
    }
    file.close();
  } else Serial.println("Can't open Calibration Data File");

  /*
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
      loc+=eepromStorageSize;
    }  */
}
