

/**
   A BLE client example that is rich in capabilities.
   There is a lot new capabilities implemented.
   author unknown
   updated by chegewara
*/

#include <BLEDevice.h>
//#include <BLEScan.h>
//#include <EEPROM.h>
#include <Preferences.h>
#include <FA05_BLE_Library.h>

#define GPIOPINOUT ESP32_FORUM_PINOUT
#include <MatrixHardware_ESP32_V0.h>
//#define ESP32_I2S_CLOCK_SPEED (10000000UL)
#define ESP32_I2S_CLOCK_SPEED (100000UL)
//#define ESP32_I2S_CLOCK_SPEED (50000UL)
#define R1_PIN  GPIO_NUM_13

#include <SmartMatrix3.h>

// ========================== CONFIG START ===================================================

/// SmartMatrix Defines
#define COLOR_DEPTH 24                  // known working: 24, 48 - If the sketch uses type `rgb24` directly, COLOR_DEPTH must be 24
const uint8_t kMatrixWidth = 64;        // known working: 32, 64, 96, 128
const uint8_t kMatrixHeight = 32;       // known working: 16, 32, 48, 64
const uint8_t kRefreshDepth = 36;       // known working: 24, 36, 48
const uint8_t kDmaBufferRows = 4;       // known working: 2-4, use 2 to save memory, more to keep from dropping frames and automatically lowering refresh rate
const uint8_t kPanelType = SMARTMATRIX_HUB75_32ROW_MOD16SCAN;   // use SMARTMATRIX_HUB75_16ROW_MOD8SCAN for common 16x32 panels
const uint8_t kMatrixOptions = (SMARTMATRIX_OPTIONS_NONE);      // see http://docs.pixelmatix.com/SmartMatrix for options
const uint8_t kIndexedLayerOptions = (SM_INDEXED_OPTIONS_NONE);
const uint8_t kBackgroundLayerOptions = (SM_BACKGROUND_OPTIONS_NONE);

SMARTMATRIX_ALLOCATE_BUFFERS(matrix, kMatrixWidth, kMatrixHeight, kRefreshDepth, kDmaBufferRows, kPanelType, kMatrixOptions);
SMARTMATRIX_ALLOCATE_BACKGROUND_LAYER(backgroundLayer, kMatrixWidth, kMatrixHeight, COLOR_DEPTH, kBackgroundLayerOptions);
//SMARTMATRIX_ALLOCATE_INDEXED_LAYER(indexedLayer, kMatrixWidth, kMatrixHeight, COLOR_DEPTH, kIndexedLayerOptions);

const int defaultBrightness = (100 * 255) / 100;  // full (100%) brightness
const int dimBrightness = (20 * 255) / 100;  // dim (20%) brightness
// ========================== CONFIG END ======================================================

const SM_RGB LED_BLACK = {0x0, 0x0, 0x0};
const SM_RGB LED_WHITE_LOW = {50, 50, 50};
const SM_RGB LED_WHITE_MED = {210, 210, 210};
const SM_RGB LED_WHITE_HIGH = {0xff, 0xff, 0xff};

const SM_RGB LED_RED_LOW = {50, 0x0, 0x0};
const SM_RGB LED_RED_MED = {170, 0x0, 0x0};
const SM_RGB LED_RED_HIGH = {0xff, 0x0, 0x0};

const SM_RGB LED_GREEN_LOW = {0x0, 50, 0x0};
const SM_RGB LED_GREEN_MED = {0x0, 150, 0x0};
const SM_RGB LED_GREEN_HIGH = {0x0, 0xff, 0x0};

const SM_RGB LED_BLUE_LOW = {0x0, 0, 50};
const SM_RGB LED_BLUE_MED = {0x0, 0, 150};
const SM_RGB LED_BLUE_HIGH = {0x0, 0x0, 0xff};

const SM_RGB LED_PURPLE_HIGH = {0xff, 0, 0xff};

const SM_RGB LED_YELLOW_MED = {160, 160, 0};
const SM_RGB LED_YELLOW_HIGH = {0xff, 0xff, 0};

const SM_RGB LED_ORANGE_HIGH = {0xff, 150, 0};

//ESP_GATTC_NOTIFY_EVT

#define UUID_STR_LEN 37

//const unsigned long BLE_SEARCH_TIMEOUT=45*1000; //Time

// The remote service we wish to connect to.
//static BLEUUID serviceUUID("4fafc201-1fb5-459e-8fcc-c5c9c331914b");
// The characteristic of the remote service we are interested in.
//static BLEUUID    charUUID("beb5483e-36e1-4688-b7f5-ea07361b26a8");

//static boolean doConnect = false;
//static boolean connected = false;
//static boolean doScan = false;

boolean txPaired = false;
boolean txFound = false;
boolean isConnected = false;
boolean pairingEnabled = false;

const unsigned long tSleepInterval = 60 * 1000; //Wakeup ever 60s to scan for devices if not connected.
const unsigned long idleTimeOut = 15 * 60 * 1000; //Switch to idle mode if inactive for 15min

unsigned long tLastSeen = 0; //timer for the last time a heart-beat packet was recieved
unsigned long tLastActive = 0; //timer for the last time the scoring machine changed state
unsigned long tPairingActive = 0; //
unsigned long tLastScan = 0;

// Sleep = Paired but not connected to Tx module, Idle = connected waiting for scoring machine state change,
// Active = Running full repeat,  LightsOnly = only showing hit indications
// Unpaired = Not paired with the BLE server
//
enum RX_STATE {Sleep, Idle, Active, LightsOnly, Unpaired};

RX_STATE rxModuleStatus = Sleep;

enum BLE_ERROR_CODE {BLE_SUCCESS = 0, DeviceNotFound, ServiceNotFound, CharNotFound, RegFailure, ConnectFail};

static BLERemoteCharacteristic* fa05TxDiscChar;
static BLERemoteCharacteristic* fa05TxTimeChar;
static BLERemoteCharacteristic* heartBeatChar;
static BLERemoteCharacteristic* rawDataChar;
static BLERemoteCharacteristic* lightsChar;
static BLERemoteCharacteristic* scoreChar;

static BLEAdvertisedDevice* searchDevice;
static BLEAdvertisedDevice* fa05Tx;

byte statusTextLocX = 25;
byte statusTextLocY = 31-5;

char fa05ServiceUUID[UUID_STR_LEN];
//char[UUID_STR_LEN] fa05ServiceUUID="";
BLEUUID discoveryUUID{FA05_DISCOVERY_UUID};
BLEUUID fa05TxService;

machineStatus_BLE fa05Status;
//,.upTimeSec=0,.isActive=0,.errCode=fa05err_NoError,.errCount=0};
lightData_BLE fa05Lights;
matchData_BLE fa05Score;

Preferences kvStore;

void updateLights();
void updateScore();
void updateCards();
void runDemo();
BLE_ERROR_CODE connectToFA05Service(BLEClient *pClient);

static void heartbeatCallback(
  BLERemoteCharacteristic* pBLERemoteCharacteristic,
  uint8_t* pData,
  size_t packet_size,
  bool isNotify) {
  if (pBLERemoteCharacteristic->getUUID().equals(BLEUUID(HEARTBEAT_UUID))) {
    memcpy(fa05Status.rawBytes, pData, packet_size);
    //updateHeartBeat();
  }
}

static void updateHeartBeat() {
  tLastSeen = millis();
  if (fa05Status.isActive == false) {
    setRxState(Idle);
  }
}

static void lightsCallback(
  BLERemoteCharacteristic* pBLERemoteCharacteristic,
  uint8_t* pData,
  size_t packet_size,
  bool isNotify) {
  if (pBLERemoteCharacteristic->getUUID().equals(BLEUUID(LIGHT_STATE_UUID ))) {
    memcpy(&(fa05Lights.value), pData, packet_size);
    updateLights();
  }
}

void setRxState(RX_STATE newState) {
  switch (newState) {
    case (Idle):
      break;
  }
  rxModuleStatus = newState;
}

static void scoreCallback(
  BLERemoteCharacteristic* pBLERemoteCharacteristic,
  uint8_t* pData,
  size_t packet_size,
  bool isNotify) {
  if (pBLERemoteCharacteristic->getUUID().equals(BLEUUID(MATCH_UUID)))
  {
    memcpy(fa05Score.rawBytes, pData, packet_size);
    updateScore();
  }
}



class MyClientCallback : public BLEClientCallbacks {
    void onConnect(BLEClient* pclient) {
      if (txPaired) {

      } else {
        pairService(pclient);
      }
    }

    void onDisconnect(BLEClient* pclient) {
      //connected = false;
      txFound = false;
      isConnected = false;
      Serial.println("onDisconnect");
    }
};

void pairService(BLEClient*  pClient) {
  Serial.println(" - Created client");

  BLERemoteService* pRemoteService = pClient->getService(discoveryUUID);
  if (pRemoteService == nullptr) {
    Serial.print("Failed to find our service UUID: ");
    Serial.println(discoveryUUID.toString().c_str());
    pClient->disconnect();
    return;
  }
  Serial.println(" - Found our service");

  // Obtain a reference to the characteristic in the service of the remote BLE server.
  BLERemoteCharacteristic* pRemoteCharacteristic = pRemoteService->getCharacteristic(FA05_SERVICE_CHAR_UUID);
  if (pRemoteCharacteristic == nullptr) {
    Serial.print("Failed to find our characteristic UUID: ");
    Serial.println(FA05_SERVICE_CHAR_UUID);
    pClient->disconnect();
    return;
  }
  Serial.println(" - Found our characteristic");

  // Read the value of the characteristic.
  if (pRemoteCharacteristic->canRead()) {
    std::string value = pRemoteCharacteristic->readValue();
    fa05TxService = BLEUUID(value);

    if (connectToFA05Service(pClient) == BLE_SUCCESS) {
      txPaired = true;
      txFound = true;
      isConnected = true;
      kvStore.putString("lastConnectedUUID", fa05ServiceUUID);
      //kvStore.remove("lastConnectedUUID");
    };
    Serial.print("The characteristic value was: ");
    Serial.println(value.c_str());
  }

}

BLE_ERROR_CODE connectToFA05Service(BLEClient *pClient) {
  BLE_ERROR_CODE errVal = BLE_SUCCESS;
  BLERemoteService* pRemoteService = pClient->getService(fa05TxService);

  if (pRemoteService == nullptr) {
    return (ServiceNotFound);
  }

  BLERemoteCharacteristic* pRemoteCharacteristic = pRemoteService->getCharacteristic(HEARTBEAT_UUID);
  if (pRemoteCharacteristic->canNotify()) {
    pRemoteCharacteristic->registerForNotify(heartbeatCallback);
  }

  pRemoteCharacteristic = pRemoteService->getCharacteristic(MATCH_UUID);
  if (pRemoteCharacteristic->canNotify()) {
    pRemoteCharacteristic->registerForNotify(scoreCallback);
  }

  pRemoteCharacteristic = pRemoteService->getCharacteristic(LIGHT_STATE_UUID);
  if (pRemoteCharacteristic->canNotify()) {
    pRemoteCharacteristic->registerForNotify(lightsCallback);
  }
  return (BLE_SUCCESS);
}

bool connectToServer() {
  Serial.print("Forming a connection to ");
  Serial.println(searchDevice->getAddress().toString().c_str());

  BLEClient*  pClient  = BLEDevice::createClient();
  Serial.println(" - Created client");

  pClient->setClientCallbacks(new MyClientCallback());

  // Connect to the remove BLE Server.
  pClient->connect(searchDevice);  // if you pass BLEAdvertisedDevice instead of address, it will be recognized type of peer device address (public or private)
  Serial.println(" - Connected to server");

  // Obtain a reference to the service we are after in the remote BLE server.
  if ((txPaired == false) && pairingEnabled) {
    pairService(pClient);
  }

  connectToFA05Service(pClient);

}
/**
   Scan for BLE servers and find the first one that advertises the service we are looking for.
*/
class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
    /**
        Called for each advertising BLE server.
    */
    void onResult(BLEAdvertisedDevice advertisedDevice) {
      Serial.print("BLE Advertised Device found: ");
      Serial.println(advertisedDevice.toString().c_str());

      // We have found a device, let us now see if it contains the service we are looking for.
      if (txPaired) {
        if (advertisedDevice.haveServiceUUID() && advertisedDevice.isAdvertisingService(fa05TxService)) {
          BLEDevice::getScan()->stop();
          fa05Tx = new BLEAdvertisedDevice(advertisedDevice);
          txFound = true;
          //doConnect = true;
          //doScan = true;
        }
      } else {
        if (pairingEnabled) {
          if (advertisedDevice.haveServiceUUID() && advertisedDevice.isAdvertisingService(BLEUUID(FA05_DISCOVERY_UUID))) {
            BLEDevice::getScan()->stop();
            searchDevice = new BLEAdvertisedDevice(advertisedDevice);
            txFound = true;
          } // Found our server
        } // onResult
      }
    }
}; // MyAdvertisedDeviceCallbacks

void scanForDevices() {
  BLEScanResults bleDeviceList;
  // Retrieve a Scanner and set the callback we want to use to be informed when we
  // have detected a new device.  Specify that we want active scanning and start the
  // scan to run for 5 seconds.
  BLEScan* pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setInterval(1349);
  pBLEScan->setWindow(449);
  pBLEScan->setActiveScan(true);
  bleDeviceList = pBLEScan->start(5, false);
  tLastScan = millis();
}


void changeSystemState(RX_STATE newState){
  
}



//{Sleep, Idle, Active, LightsOnly, Unpaired};
void updateSystemState() {
  unsigned long tNow = millis();

  //checkButtonStatus();

  switch (rxModuleStatus) {
    case (Unpaired):
      if ((pairingEnabled) && ((tNow - tPairingActive) > FA05_PAIRING_TIMEOUT)) {
        pairingEnabled = false;
        setStatusText(" N/C ", LED_RED_LOW);
      }
      if (pairingEnabled) {
        setStatusText("Pair", LED_BLUE_LOW);
      }
    case (Sleep):
      if ((tNow - tLastScan) > tSleepInterval) {
        scanForDevices();
        tLastScan = millis();
      }
      //backgroundLayer.setFont(font3x5);
      //backgroundLayer.drawString(28,0,LED_BLUE_LOW,"Sleep");
      break;
    case (Idle):
      if (isConnected == false) {
        changeSystemState(Sleep);
      }
      if ((tNow - tLastActive) < idleTimeOut) {
        changeSystemState(Active);
      }
      break;
    case (Active):
      if (fa05Lights.lightsOnly) {
        changeSystemState(LightsOnly);
      }
    case (LightsOnly):
      if (fa05Lights.lightsOnly == false) {
        changeSystemState(Active);
      }
      if (isConnected == false) {
        changeSystemState(Sleep);
      }
      if ((tNow - tLastActive) > idleTimeOut) {
        changeSystemState(Idle);
      }
  }
}

void initBLEStructs() {
  //fa05Lights.value = 0;
  //fa05Score.rawBytes{0, 0, 0, 1, 0, 0};

}


void setup() {
  Serial.begin(115200);
  Serial.println("Starting Arduino BLE Client application...");

  kvStore.begin("FA05-Rx");
  BLEDevice::init("");

  initBLEStructs();

  // setup matrix
  matrix.addLayer(&backgroundLayer);
  matrix.begin();

  backgroundLayer.fillScreen(LED_BLACK);

  if (kvStore.getString("lastConnectedUUID", fa05ServiceUUID, UUID_STR_LEN) == 0) {
    Serial.println("No devices saved");
    fa05ServiceUUID[0] = '\0';
    txPaired = false;
    txFound = false;
    isConnected = false;
    pairingEnabled = false;
  } else {
    kvStore.getString("lastConnectedUUID", fa05ServiceUUID, UUID_STR_LEN);
    fa05TxService = BLEUUID(fa05ServiceUUID);
    txPaired = true;
    txFound = false;
    isConnected = false;
    pairingEnabled = true;
  }

  //scanForDiscoveryServices();
} // End of setup.


// This is the Arduino main loop function.
void loop() {

  // If the flag "doConnect" is true then we have scanned for and found the desired
  // BLE Server with which we wish to connect.  Now we connect to it.  Once we are
  // connected we set the connected flag to be true.
  if (isConnected == false) {
    if (txFound == true) {
      //connectToServer();
    } else {
      //checkForScanning();
    }
  }

  //checkButtonState();
  runDemo();
  updateSystemState();
} // End of loop
