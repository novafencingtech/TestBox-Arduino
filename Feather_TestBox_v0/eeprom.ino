#include <Adafruit_LittleFS.h>
#include <InternalFileSystem.h>

using namespace Adafruit_LittleFS_Namespace;

#define FILENAME    "/calibration.txt"

File file(InternalFS);
bool isInitialized = false;



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
    p = 0;
    buf[p++] = NUM_CAL_CHANNELS;
    Serial.println("Write Calibration Data");
    for (int k = 0; k < NUM_CAL_CHANNELS; k++) {
      CalChannel=getCalibrationChannel(k);
      buf[p++] = lowByte(CalChannel->getTrim());
      buf[p++] = highByte(CalChannel->getTrim());
    }
    file.write(buf, 1 + (NUM_CAL_CHANNELS * 2));
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
  int p = 0;
  uint8_t buf[100];
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
      if (k<n) cal = buf[p++] | (buf[p++] >> 8); 
      else {
        Serial.println("Fewer calibrations in file than in code");
        cal=0;
      }
      if (cal >= calibrationErrorValue) cal = 0;
      CalChannel->setTrim(cal);
    }
    file.close();
  } else Serial.println("Can't open Calibration Data File");
}
