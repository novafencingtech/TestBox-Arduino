



#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <string.h>

#include <SoftwareSerial.h>
#include <FA05_BLE_Library.h>
//#include "FA05_BLE_Library.h"

const byte rxPin = 22;
const byte txPin = 8;

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
char strBuffer[128]=""; //Generic buffer for parsing string data

// set up a new serial object
SoftwareSerial fa05SerialIn(rxPin, txPin); //Invert the input to correct for the optoisolator in the scoring machine

const char fa05DeviceMfr[8]="NOVA FC";

BLEServer* pServer = NULL;
//Service & Characteristics to support pairing & Discovery
BLEService *fa05DiscoveryService=NULL;
uint8_t fa05DiscoveryUUID128[16];

uint8_t bleAdvertisementRaw[31];

//BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
//BLEAdvertisementData *advertisementData=NULL;
BLECharacteristic *serverAddrChar = NULL;
BLECharacteristic *serverConnectTimeRemainingChar = NULL;
//Service & Characteristics for the remote relay
BLEService *fa05BLEService = NULL;
uint8_t fa05ServiceUUID128[16];
BLECharacteristic* rawDataBLEChar = NULL;
BLECharacteristic* lightsBLEChar = NULL;
BLECharacteristic* matchBLEChar = NULL;
BLECharacteristic* heartbeatBLEChar = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;


esp_ble_adv_data_t advertisementData={
  .set_scan_rsp=false,
.include_name=false,
.include_txpower=false,
.min_interval=0x20,
.max_interval=0x40,
.appearance=0x00,
.manufacturer_len=0,
.p_manufacturer_data=(uint8_t *) fa05DeviceMfr, 
.service_data_len=0,
.p_service_data=NULL,
.service_uuid_len=16,
.p_service_uuid=NULL,
.flag=(ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT)};

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

const int HEART_BEAT_INTERVAL = 5 * 1000;
const uint32_t SCORE_IDLE_TIMEOUT = 10*60*1000; 
const uint32_t FA05_OFF_TIMEOUT = 5*1000; 

char serviceUUID[37] = BASE_FA05_UUID;
//int activeStrip = 00;

machineStatus_BLE statusBLE;
lightData_BLE lightsBLE;
matchData_BLE scoreBLE;

uint32_t fa05LastActive=0;
bool fa05SendingData=false;
uint32_t tLastScoreChange=0;
int discoveryTimeRemaining = 0;

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
      BLEDevice::startAdvertising();
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
    }
};

void createBLECharacteristics() {
  BLEDescriptor *tempDescriptor;
  BLEUUID txtDescriptor=BLEUUID((uint16_t)0x2901);
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
  tempDescriptor= new BLEDescriptor(txtDescriptor);
  tempDescriptor->setValue("Raw FA-01/-05 Serial Data");
  rawDataBLEChar->addDescriptor(tempDescriptor);

  // Create a BLE Characteristic
  lightsBLEChar = fa05BLEService->createCharacteristic(
                    LIGHT_STATE_UUID,
                    BLECharacteristic::PROPERTY_READ   |
                    //BLECharacteristic::PROPERTY_NOTIFY |
                    BLECharacteristic::PROPERTY_INDICATE
                  );

  // https://www.bluetooth.com/specifications/gatt/viewer?attributeXmlFile=org.bluetooth.descriptor.gatt.client_characteristic_configuration.xml
  // Create a BLE Descriptor
  clConfig=new BLE2902();
  clConfig->setIndications(true);
  lightsBLEChar->addDescriptor(clConfig);
  /*tempDescriptor= new BLEDescriptor(txtDescriptor);
  tempDescriptor->setValue("Indicator light status");
  lightsBLEChar->addDescriptor(tempDescriptor);*/

  // Create a BLE Characteristic
  matchBLEChar = fa05BLEService->createCharacteristic(
                   MATCH_UUID,
                   BLECharacteristic::PROPERTY_READ   |
                   //BLECharacteristic::PROPERTY_NOTIFY |
                   BLECharacteristic::PROPERTY_INDICATE
                 );

  // https://www.bluetooth.com/specifications/gatt/viewer?attributeXmlFile=org.bluetooth.descriptor.gatt.client_characteristic_configuration.xml
  // Create a BLE Descriptor
  clConfig=new BLE2902();
  clConfig->setIndications(true);
  matchBLEChar->addDescriptor(clConfig);  
  /*tempDescriptor= new BLEDescriptor(BLEUUID((uint16_t)0x2901));
  tempDescriptor->setValue("Match score & time data");
  matchBLEChar->addDescriptor(tempDescriptor);*/

  // Create a BLE Characteristic
  heartbeatBLEChar = fa05BLEService->createCharacteristic(
                       HEARTBEAT_UUID,
                       BLECharacteristic::PROPERTY_READ   |
                       //BLECharacteristic::PROPERTY_NOTIFY |
                       BLECharacteristic::PROPERTY_INDICATE
                     );

  // https://www.bluetooth.com/specifications/gatt/viewer?attributeXmlFile=org.bluetooth.descriptor.gatt.client_characteristic_configuration.xml
  // Create a BLE Descriptor
  clConfig=new BLE2902();
  clConfig->setIndications(true);
  heartbeatBLEChar->addDescriptor(clConfig);  
  tempDescriptor= new BLEDescriptor(BLEUUID((uint16_t)0x2901));
  tempDescriptor->setValue("FA-05 Status Information");
  heartbeatBLEChar->addDescriptor(tempDescriptor);
}

void setAdvertisingParams() {
  advParams.adv_int_min=0x20;
  advParams.adv_int_max=0x40;
  advParams.adv_type=ADV_TYPE_IND;
  advParams.own_addr_type=BLE_ADDR_TYPE_PUBLIC;
  //.peer_addr=0x0,
  //.peer_addr_type=BLE_ADDR_TYPE_PUBLIC,
  advParams.channel_map=ADV_CHNL_ALL;
  advParams.adv_filter_policy=ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY;
}

void setServiceUUID() {
  char str[18] = "";
  std::string addr;
  //BLEAddress localBLE;

  memcpy(fa05DiscoveryUUID128,BLEUUID(FA05_DISCOVERY_UUID).getNative()->uuid.uuid128,16);
  
  BLEAddress localBLE=BLEDevice::getAddress();
  addr=localBLE.toString();
  addr.copy(str,8,9); //Copy 8 characters starting at pos 9
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
  strncat(serviceUUID,str,2);
  strncat(serviceUUID,&(str[3]),2);
  strncat(serviceUUID,&(str[6]),2);
  //serviceUUID+=str[9];
  //Serial.println(serviceUUID);

  //fa05DiscoveryUUID128=BLEUUID()
  memcpy(fa05ServiceUUID128,BLEUUID(serviceUUID).getNative()->uuid.uuid128,16);
}

void advertisingCallback(esp_gap_ble_cb_event_t event,esp_ble_gap_cb_param_t *param){
  switch (event) {
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT :
      esp_ble_gap_start_advertising(&advParams);
      break;
    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
      Serial.println("Advertising started");
      break;
  }
}

void advertiseDiscoveryMode() {
  //Turn off advertising, if active
  esp_ble_gap_stop_advertising();

  setAdvertisingParams();
  advertisementData.p_service_uuid=fa05DiscoveryUUID128;  
  if (esp_ble_gap_config_adv_data(&advertisementData)==ESP_OK) {    
    
    //esp_ble_gap_start_advertising(&advParams);    
  } else {
    Serial.println("Adv config failed");
  }
}

void advertiseServiceMode() {
  esp_ble_gap_stop_advertising();

  setAdvertisingParams();
  advertisementData.p_service_uuid=fa05ServiceUUID128;
  
  if (esp_ble_gap_config_adv_data(&advertisementData)==ESP_OK) {    
    //delay(100);
    //esp_ble_gap_start_advertising(&advParams);    
  } else {
    Serial.println("Adv config failed");
  }
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  // Create the BLE Device
  BLEDevice::init("ESP32");
  setServiceUUID();

  // Create the BLE Server
  pServer = BLEDevice::createServer();
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
  serverConnectTimeRemainingChar->setValue(discoveryTimeRemaining);
  fa05DiscoveryService->start();
  
  fa05BLEService = pServer->createService(serviceUUID);
  createBLECharacteristics();

  // Start the service
  fa05BLEService->start();

  fa05SerialIn.begin(2400);

  // Start advertising
  advertiseDiscoveryMode();
  
  //BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  //advertisementData=new BLEAdvertisementData();
  //advertisementData->setServiceData(BLEUUID(FA05_DISCOVERY_UUID),std::string(serviceUUID));
  //pAdvertising->setScanResponseData(*advertisementData);
  //pAdvertising->setAdvertisementData(*advertisementData);
  //Serial.println("Data packet =");
  //Serial.println(advertisementData->getPayload().c_str());
  
  //pAdvertising->addServiceUUID(FA05_DISCOVERY_UUID);
  //BLEAdvertising *sAdvertising = BLEDevice::getAdvertising();
  //sAdvertising->addServiceUUID(serviceUUID);
  //pAdvertising->setScanResponse(false);
  //pAdvertising->setMinPreferred(0x0);  // set value to 0x00 to not advertise this parameter
  //pAdvertising->addData("54");
  //BLEDevice::startAdvertising();
  Serial.println("Waiting a client connection to notify...");
  
}

void updateBLEData() {
  unsigned long tNow = millis();
  static unsigned long tHeartBeat;
  bool scoreUpdate = false;

  for (int k = 0; k < PACKET_SIZE; k++) {
    if (currentState[k] != lastPacket[k]) {
      //packetChange = true;
      switch (k) {
        case fa05LightPkt:
          updateLightStatus();
          break;
        case fa05ScoreRightPkt:
        case fa05ScoreLeftPkt:
        case fa05TimeSecPkt:
        case fa05TimeMinPkt:
        case fa05PriorityPkt:
        case fa05FlashPkt:
        case fa05CardPkt:
          tLastScoreChange=millis();
          scoreUpdate = true;
          updateScore();
          break;
        case fa05ChkSumPkt:
          sendRawData();
          break;
      }
      /*Serial.println("Change");
        Serial.print("Old Byte #"); Serial.print(k); Serial.print("\t"); Serial.print(lastPacket[k], HEX); Serial.print("\t"); Serial.println(lastPacket[k], BIN);
        Serial.print("New Byte #"); Serial.print(k); Serial.print("\t"); Serial.print(newPacket[k], HEX); Serial.print("\t"); Serial.println(newPacket[k], BIN);
        Serial.print("Check Sum:"); Serial.print("\t"); Serial.print(chkSum, HEX); Serial.print("\t"); Serial.print(chkSum, BIN);
      */
    }
  }

  if (scoreUpdate) {
    updateScoreBLEService();
  }

  if ((tNow - tHeartBeat) > HEART_BEAT_INTERVAL) {
    updateHeartBeat();
    tHeartBeat = millis();
  }
}

void updateLightStatus() {
  lightsBLE.offTargetL = bitRead(currentState[fa05LightPkt], 0);
  lightsBLE.offTargetR = bitRead(currentState[fa05LightPkt], 1);
  lightsBLE.touchL = bitRead(currentState[fa05LightPkt], 2);
  lightsBLE.touchR = bitRead(currentState[fa05LightPkt], 3);
  lightsBLE.gndFaultR = bitRead(currentState[fa05LightPkt], 4);
  lightsBLE.gndFaultL = bitRead(currentState[fa05LightPkt], 5);
  lightsBLE.lightsOnly = ((millis()-tLastScoreChange)>SCORE_IDLE_TIMEOUT) ? 1 : 0;
  lightsBLEChar->setValue( &(lightsBLE.value), sizeof(lightData_BLE));
  //pCharacteristic->notify();
  lightsBLEChar->indicate();
  Serial.print("Updating light status: "); Serial.println(lightsBLE.value, HEX);
}

void updateScore() {
  scoreBLE.scoreL = currentState[fa05ScoreLeftPkt];
  scoreBLE.scoreR = currentState[fa05ScoreRightPkt];
  scoreBLE.timeRemainMin = currentState[fa05TimeMinPkt];
  scoreBLE.timeRemainSec = currentState[fa05TimeSecPkt];
  scoreBLE.matchCount = currentState[fa05PriorityPkt] & (0x3); //Only the first two bits are the match count
  scoreBLE.redR = bitRead(currentState[fa05CardPkt], 0);
  scoreBLE.redL = bitRead(currentState[fa05CardPkt], 1);
  scoreBLE.yellowR = bitRead(currentState[fa05CardPkt], 2);
  scoreBLE.yellowL = bitRead(currentState[fa05CardPkt], 3);
  scoreBLE.clockRunning = bitRead(currentState[fa05FlashPkt], 0);
  scoreBLE.clockIdle = ((millis()-tLastScoreChange)>SCORE_IDLE_TIMEOUT) ? 1 : 0;
  scoreBLE.priorityR = bitRead(currentState[fa05PriorityPkt], 2);
  scoreBLE.priorityL = bitRead(currentState[fa05PriorityPkt], 3);
}

// Sends the scoring inf
void updateScoreBLEService() {
  matchBLEChar->setValue( scoreBLE.rawBytes,matchDataBLEPacketSize);
  matchBLEChar->indicate();
  Serial.println("Updating time & score");
}

void sendRawData() {
  rawDataBLEChar->setValue(currentState, PACKET_SIZE);
  rawDataBLEChar->indicate();
  Serial.println("Sending raw data ");
}

void updateHeartBeat() {
  statusBLE.upTimeSec=millis()/1000;
  statusBLE.isActive=fa05SendingData;
  heartbeatBLEChar->setValue( statusBLE.rawBytes, heartBeatPacketSize);
  heartbeatBLEChar->indicate();
}

void checkForDataPacket() {
  bool packetDone = false;
  bool packetChange = false;
  static uint8_t indx = 0;
  static bool startFound = false;
  //static uint8_t packetCount = 0;
  uint8_t incData, chkSum;

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
      fa05SendingData=true;
      fa05LastActive=millis();
      statusBLE.packetCount++;
      statusBLE.errCode=fa05err_NoError;
      updateBLEData();
      for (int k = 0; k < PACKET_SIZE; k++) {
          lastPacket[k] = currentState[k];
      }      
      packetDone = false;
    } else { //check sum failed, assume that we're lost in mid-stream
      packetDone = false;
      statusBLE.errCount++;
      statusBLE.errCode=fa05err_BadPacket;
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
  
  if ((millis()-fa05LastActive)>FA05_OFF_TIMEOUT) {
    fa05SendingData=false;
    //statusBLE.errCode=fa05err_NoData;
    //statusBLE.isActive=false;
  }
}

void loop() {
  //uint8_t lights = LIGHT_STATUS_NULL | (random(0, 2) << R_TOUCH_BIT);
  // put your main code here, to run repeatedly:
  // notify changed value

  checkForDataPacket();

  delay(30000);
  advertiseServiceMode();
  
  //BLEAdvertising *sAdvertising = BLEDevice::getAdvertising();
  //sAdvertising->stop();
  //advertisementData->setCompleteServices(serviceUUID);
  //advertisementData->setCompleteServices(BLEUUID(serviceUUID));
  //sAdvertising->setAdvertisementData(*advertisementData);
  //sAdvertising->setScanResponseData(*advertisementData);
  //sAdvertising->setScanResponse(true);
  //sAdvertising->start();
  
  if (deviceConnected) {
    updateBLEData();
    //checkForDataPacket();
  }
  // disconnecting
  if (!deviceConnected && oldDeviceConnected) {
    delay(500); // give the bluetooth stack the chance to get things ready
    //pServer->startAdvertising(); // restart advertising
    Serial.println("start advertising");
    oldDeviceConnected = deviceConnected;
  }
  // connecting
  if (deviceConnected && !oldDeviceConnected) {
    // do stuff here on connecting
    oldDeviceConnected = deviceConnected;
  }

}
