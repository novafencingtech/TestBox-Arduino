#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <string.h>
#include <Preferences.h>

#include <SoftwareSerial.h>
#include <FA05_BLE_Library.h>
//#include "FA05_BLE_Library.h"

const byte rxPin = 21;
const byte txPin = -1;

const byte BUTTON_PIN = 33;
const byte LED_PIN = 2;

const byte PACKET_START_BYTE = 0xFF;
const int PACKET_SIZE = 10;

const uint8_t fa05HeaderPkt = 0;
const uint8_t fa05ScoreRightPkt = 1;
const uint8_t fa05ScoreLeftPkt = 2;
const uint8_t fa05TimeSecPkt = 3;
const uint8_t fa05TimeMinPkt = 4;
const uint8_t fa05LightPkt = 5;
const uint8_t fa05PriorityPkt = 6;
const uint8_t fa05FlashPkt = 7;
const uint8_t fa05CardPkt = 8;
const uint8_t fa05ChkSumPkt = 9;

uint8_t buffer1[2 * PACKET_SIZE];
uint8_t lastPacket[PACKET_SIZE];
uint8_t currentState[PACKET_SIZE];
uint32_t packetCount = 0; //Data should be a 30 data frames/sec, so 10sec heart beat
char strBuffer[128] = ""; //Generic buffer for parsing string data

// set up a new serial object
SoftwareSerial fa05SerialIn(rxPin, txPin);

Preferences kvStore;

const char fa05DeviceMfr[8] = "GSAllen";

char txDeviceName[8] = "NoVA";

BLEServer* pServer = NULL;
//Service & Characteristics to support pairing & Discovery
BLEService *fa05DiscoveryService = NULL;
uint8_t fa05DiscoveryUUID128[ESP_UUID_LEN_128];

uint8_t bleAdvertisementRaw[31];

//BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
//BLEAdvertisementData *advertisementData=NULL;
BLECharacteristic *serverAddrChar = NULL;
BLECharacteristic *serverConnectTimeRemainingChar = NULL;
//Service & Characteristics for the remote relay
BLEService *fa05BLEService = NULL;
uint8_t fa05ServiceUUID128[ESP_UUID_LEN_128];
BLECharacteristic* rawDataBLEChar = NULL;
BLECharacteristic* lightsBLEChar = NULL;
BLECharacteristic* matchBLEChar = NULL;
BLECharacteristic* heartbeatBLEChar = NULL;
bool deviceConnected = false;
bool pairingEnabled = false;
int connectedCount = 0;
//bool oldDeviceConnected = false;

bool startAdvertising=false;


esp_ble_adv_data_t advertisementData = {
  .set_scan_rsp = false,
  .include_name = true,
  .include_txpower = false,
  .min_interval = 0x320,
  .max_interval = 0x640,
  .appearance = 0x00,
  .manufacturer_len = 0,
  .p_manufacturer_data = (uint8_t *) fa05DeviceMfr,
  .service_data_len = 0,
  .p_service_data = NULL,
  .service_uuid_len = ESP_UUID_LEN_128,
  .p_service_uuid = NULL,
  .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT)
};

static esp_ble_adv_params_t advParams;
/*={
  .adv_int_min=0x20,
  .adv_int_max=0x40,
  .adv_type=ADV_TYPE_IND,
  .own_addr_type=BLE_ADDR_TYPE_PUBLIC,
  //.peer_addr=0x0,
  //.peer_addr_type=BLE_ADDR_TYPE_PUBLIC,
  .channel_map=ADV_CHNL_ALL,
  .adv_filter_policy=ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
  };*/

char connectedDevices[8][40]; //List of connected devices (up to 8)

const int SCORE_INTERVAL = 1000; //Update the score at least this often
const int LED_FAST_BLINK = 100; //Update the score at least this often
const int LED_SLOW_BLINK = 500; //Update the score at least this often

const int LIGHTS_INTERVAL = 200; //Update the score at least this often, even if no new data (limits dropped packets)
const int HEART_BEAT_INTERVAL = 5 * 1000;
const uint32_t SCORE_IDLE_TIMEOUT = 1 * 60 * 1000;
const uint32_t FA05_OFF_TIMEOUT = 5 * 1000;
const uint32_t FA05_IDLE_TIMEOUT = 3 * 60 * 1000;

const unsigned long tShortPress = 100; //ms-Short button press
const unsigned long tLongPress = 1000; //ms - Long button press
const unsigned long tDoubleClick = 1000; //ms -- for two clicks

char serviceUUID[37] = BASE_FA05_UUID;
const uint8_t MAX_NUM_STRIPS=25;
uint8_t activeStrip = 00;
bool stripChanged=false;

machineStatus_BLE statusBLE;
lightData_BLE lightsBLE;
matchData_BLE scoreBLE;

unsigned long fa05LastPacket = 0;
unsigned long fa05LastActive = 0;
bool fa05SendingData = false;
uint32_t tLastScoreChange = 0;
unsigned long tPairingActive = 0;

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) { 
      connectedCount++;
      deviceConnected = true;
      //BLEDevice::startAdvertising();
      startAdvertising=true;
      //pServer->updateConnParam(
      Serial.println("Device connected");
    };

    void onDisconnect(BLEServer* pServer) {
      //deviceConnected = BLEDevice::connected();
      connectedCount--;
      startAdvertising=true;
      deviceConnected = (connectedCount > 0);
      Serial.println("Device disconnected");
      if (connectedCount < 0) {
        Serial.println("Disconnect with no devices connected");
      }
      if (!deviceConnected) {
        Serial.println("No devices connected");
      }
      
    }
};

void createBLECharacteristics() {
  BLEDescriptor *tempDescriptor;
  BLEUUID txtDescriptor = BLEUUID((uint16_t)0x2901);
  BLE2902 *clConfig;



  // Create a BLE Characteristic
  rawDataBLEChar = fa05BLEService->createCharacteristic(
                     RAW_DATA_UUID,
                     BLECharacteristic::PROPERTY_READ   |
                     BLECharacteristic::PROPERTY_NOTIFY
                     //BLECharacteristic::PROPERTY_INDICATE
                   );

  // https://www.bluetooth.com/specifications/gatt/viewer?attributeXmlFile=org.bluetooth.descriptor.gatt.client_characteristic_configuration.xml
  // Create a BLE Descriptor
  rawDataBLEChar->addDescriptor(new BLE2902());
  tempDescriptor = new BLEDescriptor(txtDescriptor);
  tempDescriptor->setValue("Raw FA-01/-05 Serial");
  rawDataBLEChar->addDescriptor(tempDescriptor);

  // Create a BLE Characteristic
  lightsBLEChar = fa05BLEService->createCharacteristic(
                    LIGHT_STATE_UUID,
                    BLECharacteristic::PROPERTY_READ   |
                    BLECharacteristic::PROPERTY_NOTIFY 
                    //BLECharacteristic::PROPERTY_INDICATE
                  );

  // https://www.bluetooth.com/specifications/gatt/viewer?attributeXmlFile=org.bluetooth.descriptor.gatt.client_characteristic_configuration.xml
  // Create a BLE Descriptor
  clConfig = new BLE2902();
  clConfig->setNotifications(true);
  lightsBLEChar->addDescriptor(clConfig);
  /*tempDescriptor= new BLEDescriptor(txtDescriptor);
    tempDescriptor->setValue("Indicator light status");
    lightsBLEChar->addDescriptor(tempDescriptor);*/

  // Create a BLE Characteristic
  matchBLEChar = fa05BLEService->createCharacteristic(
                   MATCH_UUID,
                   BLECharacteristic::PROPERTY_READ   |
                   BLECharacteristic::PROPERTY_NOTIFY
                   //BLECharacteristic::PROPERTY_INDICATE
                 );

  // https://www.bluetooth.com/specifications/gatt/viewer?attributeXmlFile=org.bluetooth.descriptor.gatt.client_characteristic_configuration.xml
  // Create a BLE Descriptor
  clConfig = new BLE2902();
  clConfig->setNotifications(true);
  matchBLEChar->addDescriptor(clConfig);
  /*tempDescriptor= new BLEDescriptor(BLEUUID((uint16_t)0x2901));
  tempDescriptor->setValue("Score data");
  matchBLEChar->addDescriptor(tempDescriptor);*/

  // Create a BLE Characteristic
  heartbeatBLEChar = fa05BLEService->createCharacteristic(
                       HEARTBEAT_UUID,
                       BLECharacteristic::PROPERTY_READ   |
                       BLECharacteristic::PROPERTY_NOTIFY 
                       //BLECharacteristic::PROPERTY_INDICATE
                     );

  // https://www.bluetooth.com/specifications/gatt/viewer?attributeXmlFile=org.bluetooth.descriptor.gatt.client_characteristic_configuration.xml
  // Create a BLE Descriptor
  clConfig = new BLE2902();
  clConfig->setNotifications(true);
  heartbeatBLEChar->addDescriptor(clConfig);
  tempDescriptor = new BLEDescriptor(BLEUUID((uint16_t)0x2901));
  tempDescriptor->setValue("FA-05 Status");
  heartbeatBLEChar->addDescriptor(tempDescriptor);
}

void setAdvertisingParams() {
  advParams.adv_int_min = 0x160;
  advParams.adv_int_max = 0x320;
  advParams.adv_type = ADV_TYPE_IND;
  advParams.own_addr_type = BLE_ADDR_TYPE_PUBLIC;
  //.peer_addr=0x0,
  //.peer_addr_type=BLE_ADDR_TYPE_PUBLIC,
  advParams.channel_map = ADV_CHNL_ALL;
  advParams.adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY;
}

void setServiceUUID() {
  char str[18] = "";
  std::string addr;
  //BLEAddress localBLE;

  memcpy(fa05DiscoveryUUID128, BLEUUID(FA05_DISCOVERY_UUID).getNative()->uuid.uuid128, ESP_UUID_LEN_128);

  BLEAddress localBLE = BLEDevice::getAddress();
  addr = localBLE.toString();
  addr.copy(str, 8, 9); //Copy 8 characters starting at pos 9
  //Serial.println(addr.c_str());
  //Serial.println(str);
  /*value = (~number) << 8;
    //Serial.println(value,HEX);
    //value = (value + number) ^ (UUID_seed);
    Serial.println(value, HEX);
    sprintf(str, "%4x", value);
    Serial.println(str);

    serviceUUID[4] = str[0];
    serviceUUID[5] = str[1];
    serviceUUID[6] = str[2];
    serviceUUID[7] = str[3];*/

  //Append the last three octets to generate a device unique UUID for the data service.
  // This allows controlled discovery and pairing
  strncat(serviceUUID, str, 2);
  strncat(serviceUUID, &(str[3]), 2);
  strncat(serviceUUID, &(str[6]), 2);
  //serviceUUID+=str[9];
  //Serial.println(serviceUUID);

  //fa05DiscoveryUUID128=BLEUUID()
  memcpy(fa05ServiceUUID128, BLEUUID(serviceUUID).getNative()->uuid.uuid128, 16);
}

void advertisingCallback(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
  switch (event) {
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT :
      //esp_ble_gap_start_advertising(&advParams);
      Serial.println("Advertising configured: ");
      Serial.println(BLEUUID(advertisementData.p_service_uuid, ESP_UUID_LEN_128, false).toString().c_str());
      break;
    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
      Serial.println("Advertising started");
      break;
  }
}

void advertiseDiscoveryMode() {
  //Turn off advertising, if active
  if (esp_ble_gap_stop_advertising() !=ESP_OK) {
    //Serial.println("Error stopping advertising");    
  }

  esp_ble_gap_set_device_name(txDeviceName);
  setAdvertisingParams();
  advParams.adv_int_min = 0x40;
  advParams.adv_int_max = 0xA0;
  advertisementData.p_service_uuid = fa05DiscoveryUUID128;  
  if (esp_ble_gap_config_adv_data(&advertisementData) == ESP_OK) {
    Serial.print("Discovery mode: ");
    //Serial.println(BLEUUID(fa05DiscoveryUUID128).toString());    
    delay(200);
    esp_ble_gap_start_advertising(&advParams);
    startAdvertising=false;
  } else {
    //Serial.println("Adv config failed");
  }
}

void advertiseServiceMode() {
  esp_ble_gap_stop_advertising();

  esp_ble_gap_set_device_name(txDeviceName);
  setAdvertisingParams();
  advertisementData.p_service_uuid = fa05ServiceUUID128;  
  advParams.adv_int_min = 0x320;
  advParams.adv_int_max = 0x640;
  //advertisementData.min_interval = 0x320; //500ms
  //advertisementData.max_interval = 0x640; //1000ms

  if (esp_ble_gap_config_adv_data(&advertisementData) == ESP_OK) {
    Serial.println("Service mode");
    delay(200);
    //delay(100);
    esp_ble_gap_start_advertising(&advParams);
    startAdvertising=false;
  } else {
    //Serial.println("Adv config failed");
  }
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);
  /*for (int k = 0; k < 10; k++) {
    digitalWrite(LED_PIN, LOW);
    delay(LED_FAST_BLINK);
    digitalWrite(LED_PIN, HIGH);
    delay(LED_FAST_BLINK);
  }
  digitalWrite(LED_PIN, LOW);
  delay(1000);
  digitalWrite(LED_PIN, HIGH);
  delay(1000);
  digitalWrite(LED_PIN, LOW);*/

  kvStore.begin("FA05-Tx");
  activeStrip=kvStore.getUChar("StripNum");
  sprintf(txDeviceName,"NoVA%u",activeStrip);

  // Create the BLE Device
  BLEDevice::init(txDeviceName);
  esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_DEFAULT,ESP_PWR_LVL_P9);
  esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_ADV,ESP_PWR_LVL_P9);
  esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_SCAN,ESP_PWR_LVL_P9);
  
  setServiceUUID();

  // Create the BLE Server
  pServer = BLEDevice::createServer();

  //BLEDevice::setPower(ESP_PWR_LVL_P3);
  pServer->setCallbacks(new MyServerCallbacks());
  esp_ble_gap_register_callback(advertisingCallback);

  // Create the BLE Discovery Service
  fa05DiscoveryService = pServer->createService(FA05_DISCOVERY_UUID );
  serverAddrChar = fa05DiscoveryService->createCharacteristic(
                     FA05_SERVICE_CHAR_UUID,
                     BLECharacteristic::PROPERTY_READ
                   );
  serverAddrChar->setValue(serviceUUID);
  serverConnectTimeRemainingChar = fa05DiscoveryService->createCharacteristic(
                                     FA05_DISCOVERY_TIME_UUID,
                                     BLECharacteristic::PROPERTY_READ
                                   );
  //serverConnectTimeRemainingChar->setValue(discoveryTimeRemaining);
  //serverConnectTimeRemainingChar->setValue(&(45));
  fa05DiscoveryService->start();

  fa05BLEService = pServer->createService(serviceUUID);
  createBLECharacteristics();

  // Start the service
  fa05BLEService->start();

  fa05SerialIn.begin(2400);

  // Start advertising
  //advertiseDiscoveryMode();
  startAdvertising=true;
  //setPairing(true);
  //Serial.println("Waiting a client connection to notify...");
}

void updateBLEData() {
  unsigned long tNow = millis();
  static unsigned long tHeartBeat = 0;
  static unsigned long tLights = 0;
  static unsigned long tScore = 0;
  bool scoreUpdate = false;
  bool lightUpdate = false;
  bool rawUpdate=false;
  static unsigned long tic, toc;

  for (int k = 0; k < PACKET_SIZE; k++) {
    if (currentState[k] != lastPacket[k]) {
      //packetChange = true;
      fa05LastActive = tNow;
      if (statusBLE.isActive==false) {
        statusBLE.isActive=true;
        updateHeartBeat();
      }
      switch (k) {
        case fa05LightPkt:
          lightUpdate = true;
          break;
        case fa05ScoreRightPkt:
        case fa05ScoreLeftPkt:
        case fa05TimeSecPkt:
        case fa05TimeMinPkt:
        case fa05PriorityPkt:
        case fa05FlashPkt:
        case fa05CardPkt:
          tLastScoreChange = millis();
          scoreUpdate = true;          
          break;
        case fa05ChkSumPkt:
          if (fa05SendingData) {
            rawUpdate=true;
            //sendRawData();
          }
          break;
      }
      /*Serial.println("Change");
        Serial.print("Old Byte #"); Serial.print(k); Serial.print("\t"); Serial.print(lastPacket[k], HEX); Serial.print("\t"); Serial.println(lastPacket[k], BIN);
        Serial.print("New Byte #"); Serial.print(k); Serial.print("\t"); Serial.print(newPacket[k], HEX); Serial.print("\t"); Serial.println(newPacket[k], BIN);
        Serial.print("Check Sum:"); Serial.print("\t"); Serial.print(chkSum, HEX); Serial.print("\t"); Serial.print(chkSum, BIN);
      */
    }
  }
  if ((tNow - tScore) > SCORE_INTERVAL) { //Force score updates
    scoreUpdate = true;
  }
  if ((scoreUpdate) && fa05SendingData) {
    updateScore();    
    updateScoreBLEService();
    
    tScore = millis();
  }

  if ((tNow - tLights) > LIGHTS_INTERVAL) {
    lightUpdate = true;
  }
  if ((lightUpdate) && fa05SendingData) {
    updateLightStatus();
    tLights = millis();
  }
  if (rawUpdate) {
    sendRawData();
  }  
  if ((tNow - tHeartBeat) > HEART_BEAT_INTERVAL) {
    //tic=millis();
    updateHeartBeat();
    sendRawData();
    tHeartBeat = millis();
    //uint8_t oldStrip=kvStore.getUChar("StripNum");
    if (stripChanged) {
      fa05SerialIn.enableRx(false);
      kvStore.putUChar("StripNum",activeStrip);
      stripChanged=false;
      fa05SerialIn.enableRx(true);
    }
    //toc=millis();
    //Serial.print("BLE Update time = "); Serial.print(toc-tic); Serial.println(" ms");
  }
}

void updateLightStatus() {
  lightsBLE.offTargetL = bitRead(currentState[fa05LightPkt], 0);
  lightsBLE.offTargetR = bitRead(currentState[fa05LightPkt], 1);
  lightsBLE.touchL = bitRead(currentState[fa05LightPkt], 2);
  lightsBLE.touchR = bitRead(currentState[fa05LightPkt], 3);
  lightsBLE.gndFaultR = bitRead(currentState[fa05LightPkt], 4);
  lightsBLE.gndFaultL = bitRead(currentState[fa05LightPkt], 5);
  lightsBLE.lightsOnly = ((millis() - tLastScoreChange) > SCORE_IDLE_TIMEOUT) ? 1 : 0;
  lightsBLEChar->setValue( &(lightsBLE.value), sizeof(lightData_BLE));
  //pCharacteristic->notify();
  //lightsBLEChar->indicate();
  lightsBLEChar->notify();
  //Serial.print("Updating light status: "); Serial.println(lightsBLE.value, HEX);
}

static uint8_t hexToUint8(uint8_t hexValue) {
  char buf[4];
  uint8_t decValue=0;

  //Serial.print("Hex value: "); Serial.println(hexValue,HEX);
  sprintf(buf,"%x",hexValue);
  //Serial.print("String value: "); Serial.println(buf);  
  sscanf(buf,"%u",&decValue);
  //Serial.print("Dec value: "); Serial.println(decValue);
  return decValue;
}

void updateScore() {
  static uint8_t oldPacket=0;
  const uint8_t id=fa05FlashPkt;
  char buf[64];
  #define getBit(val, bit) (val & (bit<<1))==0 ? 0 : 1
  
  scoreBLE.scoreL = hexToUint8(currentState[fa05ScoreLeftPkt]);
  scoreBLE.scoreR = hexToUint8(currentState[fa05ScoreRightPkt]);
  scoreBLE.timeRemainMin = hexToUint8(currentState[fa05TimeMinPkt]);
  scoreBLE.timeRemainSec = hexToUint8(currentState[fa05TimeSecPkt]);
  //Serial.println(currentState[fa05TimeSecPkt]);
  scoreBLE.matchCount = hexToUint8(currentState[fa05PriorityPkt] & (0x3)); //Only the first two bits are the match count
  scoreBLE.redR = bitRead(currentState[fa05CardPkt], 0);
  scoreBLE.redL = bitRead(currentState[fa05CardPkt], 1);
  scoreBLE.yellowR = bitRead(currentState[fa05CardPkt], 2);
  scoreBLE.yellowL = bitRead(currentState[fa05CardPkt], 3);
  scoreBLE.clockRunning = bitRead(currentState[fa05FlashPkt], 0);
  //scoreBLE.clockRunning = getBit(currentState[fa05FlashPkt], 0);
  //scoreBLE.clockRunning = 1;
  scoreBLE.clockIdle = ((millis() - tLastScoreChange) > SCORE_IDLE_TIMEOUT) ? 1 : 0;
  scoreBLE.priorityR = bitRead(currentState[fa05PriorityPkt], 2);
  scoreBLE.priorityL = bitRead(currentState[fa05PriorityPkt], 3);
  //scoreBLE.rawBytes[5]=0xFF;

  /*if (currentState[id]!=oldPacket) {
    Serial.print("Packet changed:  old = "); Serial.print(oldPacket,BIN); Serial.print(" new = "); 
    Serial.println(currentState[id],BIN);
    for (uint8_t k = 0; k<8; k++) {
      sprintf(buf,"Bit #%u = %u",k,bitRead(currentState[id],k));
      Serial.println(buf);
    }
    oldPacket=currentState[id];

    Serial.print("matchBLE = ");
    for (uint8_t k = 0; k<matchDataBLEPacketSize+2; k++) {
      Serial.print(scoreBLE.rawBytes[k],HEX);Serial.print("-");
    }
    Serial.println("");
  }*/  
  
}

// Sends the scoring inf
void updateScoreBLEService() {
  matchBLEChar->setValue( scoreBLE.rawBytes,matchDataBLEPacketSize);
  matchBLEChar->notify();
  //Serial.println("Updating time & score");
}

void sendRawData() {
  rawDataBLEChar->setValue(currentState, PACKET_SIZE);
  rawDataBLEChar->notify();
  //Serial.println("Sending raw data ");
}

void checkErrors() {
  static unsigned long tError=0;
  unsigned long tNow;
  //lastError=
/*
  if (lastErorr!=statusBLE.errCode) {
    tError=tNow;
  } else {
    if (tNow-tError>fa05_ERROR_TIME) {
      statusBLE.errCode=fa05err_NoError;
    }
  }*/
}

void updateHeartBeat() {
  //static unsigned long lastUpdate=0;
  
  statusBLE.upTimeSec = millis() / 1000UL;
  statusBLE.isActive = ((millis() - fa05LastActive) <= FA05_IDLE_TIMEOUT) ? 1 : 0;
  statusBLE.stripNum = activeStrip;
  heartbeatBLEChar->setValue( statusBLE.rawBytes, heartBeatPacketSize);
  heartbeatBLEChar->notify();
  //lastUpdate=millis();
  //Serial.print("Strip #"); Serial.println(activeStrip);
  //Serial.println("Heart-beat sent");
}

void checkForDataPacket() {
  bool packetDone = false;
  bool packetChange = false;
  static uint8_t indx = 0;
  static bool startFound = false;
  //static uint8_t packetCount = 0;
  uint8_t incData, chkSum;

  if (fa05SerialIn.overflow()) {
    Serial.println("Serial port overflow detected");
  }

  if (fa05SerialIn.available() > 0) {
    if (startFound == false) {
      if (fa05SerialIn.peek() == PACKET_START_BYTE) {
        //Serial.println("Start found");
        indx = 0;
        buffer1[indx] = fa05SerialIn.read();
        indx++;
        startFound = true;
        packetDone = false;
        //chkSum = buffer1[indx];
      } else {
        incData = fa05SerialIn.read();
      }
    }
    if (startFound == true) {
      while (fa05SerialIn.available() > 0) {
        if (fa05SerialIn.peek() == PACKET_START_BYTE) {
          //Serial.println("Packet done");
          //chkSum=chkSum-buffer1[indx-1];
          packetDone = true;
          startFound = false;
          /*for (int k = 0; k < PACKET_SIZE; k++) {
            Serial.print("Byte #"); Serial.print(k); Serial.print("\t"); Serial.print(buffer1[k], HEX); Serial.print("\t"); Serial.println(buffer1[k], BIN);
            }*/
          break;
        }
        incData = fa05SerialIn.read();
        //chkSum = chkSum + incData;
        buffer1[indx] = incData;
        indx++;
      }
    }
  }

  if (packetDone) {
    chkSum = 0;
    for (int k = 0; k < PACKET_SIZE - 1; k++) {
      chkSum += buffer1[k];
    }
    if (chkSum == buffer1[PACKET_SIZE - 1]) {
      for (int k = 0; k < PACKET_SIZE; k++) {
        currentState[k] = buffer1[k];
      }
      //packetChange = false;
      fa05SendingData = true;
      fa05LastPacket = millis();
      statusBLE.packetCount++;
      //statusBLE.errCode = fa05err_NoError;
      updateBLEData();
      for (int k = 0; k < PACKET_SIZE; k++) {
        lastPacket[k] = currentState[k];
      }
      packetDone = false;
    } else { //check sum failed, assume that we're lost in mid-stream
      packetDone = false;
      statusBLE.errCount++;
      statusBLE.errCode = fa05err_BadPacket;
      /*Serial.println("Bad check sum");
        Serial.print("ChkSum expt #"); Serial.print("\t"); Serial.print(chkSum, HEX); Serial.print("\t"); Serial.println(chkSum, BIN);
        Serial.print("ChkSum act #"); Serial.print("\t"); Serial.print(newPacket[PACKET_SIZE - 1], HEX); Serial.print("\t"); Serial.println(newPacket[PACKET_SIZE - 1], BIN);
        startFound = false;
        //endFound = false;
        for (int k = 0; k < PACKET_SIZE; k++) {
        Serial.print("Byte #"); Serial.print(k); Serial.print("\t"); Serial.print(newPacket[k], HEX); Serial.print("\t"); Serial.println(newPacket[k], BIN);
        }*/
    } //end ChkSum loop
  }

  if ((millis() - fa05LastPacket) > FA05_OFF_TIMEOUT) {
    fa05SendingData = false;
    statusBLE.errCode = fa05err_NoData;
    statusBLE.isActive = false;
  }
}

void setPairing(bool active) {
  if (active) {
    advertiseDiscoveryMode();
    tPairingActive = millis();
    pairingEnabled = true;
  } else {
    advertiseServiceMode();
  }
  pairingEnabled = active;
}

void checkButtonState() {
  const unsigned long LOCK_OUT=1000; //Don't allow button pushes faster than X. 
  const int DEBOUNCE_TIME=20;
  static bool lastState=HIGH;
  static bool newState=HIGH;
  bool valid=false;
  static unsigned long tLastPress = 0;
  static unsigned long tPressed = 0;
  static unsigned long tSwitch = 0;
  static unsigned long tLockout = 0;
  static unsigned long tLow=0;
  static unsigned long tChange=0;
  unsigned long tNow = millis();

  int pinState=digitalRead(BUTTON_PIN);

  if (newState!=pinState) {
    if ((tNow-tChange)>DEBOUNCE_TIME) {
      valid=true;
      newState=pinState;      
    } else {    
      tChange=tNow;    
      return;
    }
  } 
  
  if ((newState != lastState) && (valid)) {
    if ((tNow-tLastPress)>LOCK_OUT) {
      
    }
    if (newState == HIGH) { //Button release
      if ((tNow - tSwitch) > tShortPress) {
        tLastPress = tNow;
        activeStrip++;
        if (activeStrip > MAX_NUM_STRIPS) {
          activeStrip=0;
        }
        sprintf(txDeviceName,"NoVA%u",activeStrip);
        stripChanged=true;
        //
        updateHeartBeat();        
        //Serial.println("Short press");
        //Short button press
      }
    }
    tSwitch = tNow;
    lastState = newState;
  }

  if ((newState == LOW) && ((tNow - tSwitch) > tLongPress)) {
    //Long button push action
    digitalWrite(LED_PIN, HIGH);
    while (digitalRead(BUTTON_PIN) == LOW) {
      delay(50); //Loop until released
    }
    tLastPress = millis();
    lastState = HIGH;
    tSwitch = millis();
    setPairing(true);
    //delay(200);
    //Serial.println("Long press");
    //tPairingActive=millis();
  }
}


void loop() {
  //uint8_t lights = LIGHT_STATUS_NULL | (random(0, 2) << R_TOUCH_BIT);
  // put your main code here, to run repeatedly:
  // notify changed value
  static unsigned long tLED = 0;
  static unsigned long tic,toc=0;
  

  tic=millis();
  checkForDataPacket();

  //delay(30000);
  checkButtonState();
  checkErrors();

  updateBLEData();

  if (startAdvertising) {
    if (pairingEnabled) {
      advertiseDiscoveryMode();
    } else {
      advertiseServiceMode();
    }
  }

  if (pairingEnabled) {
    if ((millis() - tPairingActive) > FA05_PAIRING_TIMEOUT) {
      setPairing(false);
    }
    if ((millis() - tLED) > LED_FAST_BLINK) {
      digitalWrite(LED_PIN, !digitalRead(LED_PIN));
      tLED=millis();
    }
  } else {
    digitalWrite(LED_PIN, deviceConnected);
  }
  toc=millis();
  //Serial.print("Loop time = "); Serial.print(toc-tic); Serial.println(" ms");
}
