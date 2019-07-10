//#include <EEPROM.h>
#include <Arduino.h>
#include <SPI.h>
#include <nrf52.h>
#include <nrf_saadc.h>
#include <avr/dtostrf.h>
//#include <nrf_adc.h>
//#include <sdk_config.h>

#include "Channel.h"
#include "string.h"
//#include "BluetoothSerial.h"

#define NUM_ADC_SCAN_CHANNELS 9 //9 combinations 
#define OHM_FIELD_WIDTH 4 //Width of the text display for ohms
#define SERIAL_BUFFER_SIZE 128 //Length of serial buffer 
#define ADC_BUFFER_SIZE 128                                 

const uint8_t ADC_UNIT=0;
nrf_saadc_channel_config_t ADC_CONFIG = {.resistor_p = NRF_SAADC_RESISTOR_DISABLED,
                                         .resistor_n = NRF_SAADC_RESISTOR_PULLDOWN,
                                         .gain = NRF_SAADC_GAIN1_4,
                                         .reference = NRF_SAADC_REFERENCE_INTERNAL,
                                         .acq_time = NRF_SAADC_ACQTIME_40US,
                                         .mode = NRF_SAADC_MODE_SINGLE_ENDED,
                                         .burst = NRF_SAADC_BURST_ENABLED,
                                         .pin_p = NRF_SAADC_INPUT_AIN0,
                                         .pin_n = NRF_SAADC_INPUT_DISABLED
                                        };
nrf_saadc_value_t ADC_Buffer1[ADC_BUFFER_SIZE];
nrf_saadc_value_t ADC_Buffer2[ADC_BUFFER_SIZE];

// Change the calibration valid flag when changing the format of the calibration data
const byte calibrationValid = 0xA0; //Flags the last valid calibration
const byte calibrationInvalid = 0xEE; //Marks a calibration as invalid and moves to the next location (used for wear leveling)
int eepromLocationStep  = 1+sizeof(int)*(NUM_ADC_SCAN_CHANNELS+2); //Step size in bytes between eeprom flags;
const int calibrationErrorValue = 1000; //If calibration value exceeds, generate an error
const byte calibrationRetries = 3; // Exit after this many retries

// Pile of various parameters and constants for the box to use
const int maxADCthreshold = 4000; //Used for switching between high/low gain
//const int minADCthreshold = 20; //Used for switching between high/low gain
const long powerOffTimeOut = 180000; //Time before switching to idle mode for scanning (in ms);
const int idleDisconnectTime = 5000; //Time before switching to idle mode for scanning (in ms);
const int weaponStateHoldTime = 250; //ms - How long the light remains lit after a weapon-press
const int weaponFoilDebounce = 15; //ms - How long the light remains lit after a weapon-press
const int weaponEpeeDebounce = 3; //ms - How long the light remains lit after a weapon-press
const int t_Error_Display = 2000; //ms - How long to display error/debug messages;
const int tLCDRefresh = 400; //ms - How often to refresh the lcd display
const int tLEDRefresh = 50; //ms - How often to refresh the lcd display
//const long tLEDResync = 10000; //ms -- Completely reset the LED display
const long tBatteryInterval = 30000; //ms - Check battery every 30s
//const long tLCDIdleOff = 30000; //ms - Turn LCD off if idle for more than 1 min
const int tSerialRefresh = 200; //ms - How often to send data over the serial port
const int tPowerOffPress = 1500; //ms - How long to hold the button down before it's considered a long press
const int tModeSwitchLockOut = 300; //ms - Used to prevent accidental double mode switches
const int tEnterCalibrationMode = 4000; //ms - How long to hold before entering calibration mode
const int tIdleLEDBlink = 750; //ms

const float HIGH_RESISTANCE_THRESHOLD = 5.0;
const int CABLE_DISCONNECT_THRESHOLD = 4090;
const float OPEN_CIRCUIT_VALUE = 99.0;

//MUX Settings if needed
const SPISettings OLED_SPI_SETTINGS(2000000, MSBFIRST, SPI_MODE0);
const uint8_t MUX_LATCH = 16;
const uint8_t MUX_CLK = 15;
const uint8_t MUX_DATA = 7;

//MUX is MSBFIRST, bit 0=NC, bits 1-3=source, bits 4-6=sink, bit 7=weapon
const byte MUX_DISABLED=0x0;
const byte MUX_CABLE_AA = B00100010;
const byte MUX_CABLE_AB = B01000010;
const byte MUX_CABLE_AC = B10000010;
const byte MUX_CABLE_BA = B00100100;
const byte MUX_CABLE_BB = B01000100;
const byte MUX_CABLE_BC = B10000100;
const byte MUX_CABLE_CA = B00101000;
const byte MUX_CABLE_CB = B01001000;
const byte MUX_CABLE_CC = B10001000;
const byte MUX_WEAPON_MODE = B00011010; //Source=A & C, Sink=B, bit 7=Link

//Bit definitions for the status word
const byte BITAA = 0;
const byte BITAB = 1;
const byte BITAC = 2;
const byte BITBA = 3;
const byte BITBB = 4;
const byte BITBC = 5;
const byte BITCA = 6;
const byte BITCB = 7;
const byte BITCC = 8;

//Pin defintions, as needed
const byte POWER_CONTROL = 11;
const byte DIAG_PORT = 25;
const byte BUTTON_PIN = 27;
const byte LED1_PIN = 17;
const byte LED2_PIN = 19;

// Basic variables
volatile long numSamples = 0;
volatile float EffSampleRate;
float batteryVoltage = 3.7;

//Some timing stuff if needed
volatile long timing_seg = 0;
volatile long tic = 0;
volatile long toc = 0;
volatile long tLastActive = 0; //ms - Time that an event was last detected

// Store the box state
volatile char BoxState = 'w'; //i=Idle; c=Cable; w=Weapon; r=WeaponResistance; s=sleep;

// ADC timer settings;


//Serial buffer string
char outputString[SERIAL_BUFFER_SIZE];
volatile long tIdle = 0;
//BluetoothSerial SerialBT;

#define EEPROM_SIZE 128 //Size of the simulated eeprom in bytes
int eepromAddr = 0; //Used to store the current address used by the eeprom
const int eepromStorageSize = sizeof(int); //Size of stored values in bytes

//ADC parameters
ADC_Channel ChanArray[NUM_ADC_SCAN_CHANNELS] {0, 3, 3, 4, 1, 4, 5, 5, 2}; //AA, AB, AC, BA, BB, BC, CA, CB, CC, Foil, Epee
const byte ChannelScanOrder[NUM_ADC_SCAN_CHANNELS]={1,2,3,4,5,6,7,8,0}; //Array showing the *Next channel, so Ch0 -> Ch1, Ch8->Ch0
ADC_Channel FoilADC(5);
ADC_Channel EpeeADC(3);
volatile ADC_Channel* ActiveCh;

//This struct is probably not needed for ESP32
struct testbox_line {
  byte directionBit;
  byte stateBit;
  byte digitalInMask;
  ADC_Channel* analogIn;
  byte pin_change;
};

testbox_line bananaA;
testbox_line bananaB;
testbox_line bananaC;

//Struct defining the cable/lame properties
struct CableData {
  uint16_t statusByte = 0;
  int line_AA = 4095;
  int line_AB = 4095;
  int line_AC = 4095;
  int line_BA = 4095;
  int line_BB = 4095;
  int line_BC = 4095;
  int line_CA = 4095;
  int line_CB = 4095;
  int line_CC = 4095;
  bool update_flag = false;
  bool error_msg = false;
  bool cableDC = true; //Cable disconnected flag
  bool lameMode = false;  //Lame mode flag
  float ohm_AA = 0;
  float ohm_BB = 0;
  float ohm_CC = 0;
  float ohm_AAMax = 0;
  float ohm_BBMax = 0;
  float ohm_CCMax = 0;
  long tLastConnect = 0;
};

CableData cableState;

struct weapon_test {
  bool epeeOn = false;
  long tEpeeTrigger = 0;
  long tEpeeInterOn = 0;
  bool foilOn = false;
  long tFoilTrigger = 0;
  long tFoilInterOn = 0;
  bool lineAC = false;
  //byte epeeInterruptBit = PCINT4;
  //byte foilInterruptBit = PCINT6;
  long tLightChange = 1000; //ms -- time for the intermittent LED to be on
  byte update_flag = false;
  float ohm_Foil = 0;
  float ohm_Epee = 0;
  float ohm_FoilMax = 0;
  float ohm_EpeeMax = 0;
  long tLastConnect = 0;
  bool cableDC = false;
};

volatile weapon_test weaponState;

//Used for ADC sampling
//Used for ADC sampling
#ifdef __cplusplus
extern "C" {
#endif
void SAADC_IRQHandler(void) {
  //digitalWrite(DIAG_PORT,HIGH);
  long tempVal=0;
  long t_now = millis();
  int ADCValue;  

  tempVal=0;

  //digitalWrite(DIAG_PORT,HIGH);

  if (nrf_saadc_event_check(NRF_SAADC_EVENT_RESULTDONE)) {
    nrf_saadc_event_clear(NRF_SAADC_EVENT_RESULTDONE);
    ADCValue = *(nrf_saadc_buffer_pointer_get());
  }
  
  if ( (ActiveCh->ch_label[0] == 'A') || (ActiveCh->ch_label[0] == 'E') ) {
    numSamples++;  //Indicates a full scan was performed
    //Serial.println(ADCValue);
  }
  
  ActiveCh->lastValue=ADCValue;
  //Smoothing equivalent to ((lastValue*(2^n-1))+newValue)/2^n
  // This approximates a moving average, but uses only bit-shifts
  tempVal=(ActiveCh->averageVal<<(ActiveCh->NUM_AVE_POW2));
  tempVal=tempVal-ActiveCh->averageVal;
  tempVal+=ADCValue;
  ActiveCh->averageVal=(tempVal>>(ActiveCh->NUM_AVE_POW2));
  
  if (ADCValue > ActiveCh->decay_max) {
    ActiveCh->maxval = ADCValue;
    ActiveCh->decay_max = ADCValue;
    ActiveCh->t_max = t_now;
  }

  if (ADCValue < ActiveCh->decay_min) {
    ActiveCh->minval = ADCValue;
    ActiveCh->decay_min = ADCValue;
    ActiveCh->t_min = t_now;
  }

  //Load the next channel
  if (ActiveCh->ADC_Scan) {   
    ActiveCh = ActiveCh->nextChannel;
    // Switch the MUX to toggle the MOSFETs
    digitalWrite(MUX_LATCH,LOW); //equivalent to digitalWrite(4, LOW); Toggle the SPI
    shiftOut(MUX_DATA, MUX_CLK, MSBFIRST, ActiveCh->muxSetting);
    //shiftOut(MUX_DATA, MUX_CLK, MSBFIRST, MUX_CABLE_BB);
    digitalWrite(MUX_LATCH, HIGH); //equivalent to digitalWrite(4,HIGH);
    //nrf_saadc_channel_pos_input_set(ADC_UNIT,ActiveCh->AIn);
    NRF_SAADC->CH[ADC_UNIT].PSELP=ActiveCh->AIn;

    nrf_saadc_task_trigger(NRF_SAADC_TASK_START);
    
  }
    //Reset the buffer and trigger sampling  
  nrf_saadc_buffer_init(ADC_Buffer1, 1);
  nrf_saadc_task_trigger(NRF_SAADC_TASK_SAMPLE);

  //digitalWrite(DIAG_PORT,LOW);
}
#ifdef __cplusplus
}
#endif



void setup() {
  // put your setup code here, to run once:

  //Activate display
  delay(250); //Hold for half second to power on
  //Initialize the display
  //CreateDisplay();

  //Hold the power on
  pinMode(POWER_CONTROL,OUTPUT);
  digitalWrite(POWER_CONTROL,HIGH);

  // Enable timing diagnostics
  pinMode(DIAG_PORT,OUTPUT);
  digitalWrite(DIAG_PORT,LOW);
  pinMode(LED1_PIN,OUTPUT);
  digitalWrite(LED1_PIN,HIGH);
  pinMode(LED2_PIN,OUTPUT);
  digitalWrite(LED2_PIN,HIGH);

  //EEPROM.begin(EEPROM_SIZE);
  
  //Initialize the MUX if needed
  //Serial.println("Starting MUX");
  pinMode(MUX_LATCH,OUTPUT);
  pinMode(MUX_CLK,OUTPUT);
  pinMode(MUX_DATA,OUTPUT);

  digitalWrite(MUX_LATCH,LOW);
  digitalWrite(MUX_CLK,LOW);
  digitalWrite(MUX_DATA,LOW);
  
  //initialise vspi with default pins
  //SCLK = 18, MISO = 19, MOSI = 23, SS = 5
  digitalWrite(MUX_LATCH,LOW); //equivalent to digitalWrite(4, LOW); Toggle the SPI
  shiftOut(MUX_DATA, MUX_CLK, MSBFIRST, MUX_DISABLED);
  digitalWrite(MUX_LATCH, HIGH); //equivalent to digitalWrite(4,HIGH);
  //while (true) {}

  //Enable button interrupts
  //pinMode(BUTTON_PIN,INPUT);
  
  //
  //SerialBT.begin("ESP32test"); //Bluetooth device name
  Serial.begin(115200);
  

  //Initialize the various channel settings
  Serial.println("Initializing channels");
  
  InitializeChannels();
  InitializeADC();
    
  setBoxMode('c');  //Start the box
  Serial.println("Setup complete");
  
  //BlinkLEDThenPowerOff();
}



// Pin change interrupt handler.  Used for weapon test mode.
// New function needed for ESP32
/*
ISR(PCINT0_vect) {
  long t_now = millis();
  byte newPins = BANANA_DIG_IN;
  static long t_prev = 0;
  static byte prevPins = 0;
  byte changed = 0;
  byte state = 0;

  changed = (prevPins ^ newPins);

  if (changed & bananaA.digitalInMask) {
    tLastActive = t_now;
    state = (newPins & bananaA.digitalInMask) > 0;
    if ((t_now - t_prev) > weaponEpeeDebounce) {
      //Serial.print("Epee Trigger t="); Serial.println(t_now);
      t_prev = t_now;
      weaponState.update_flag = true;
      weaponState.tEpeeTrigger = t_now;
    }
  }
  //Serial.println(changed);
  state = (newPins & bananaC.digitalInMask) > 0;
  //Serial.println(state);
  if (changed & bananaC.digitalInMask) {
    tLastActive = t_now;
    if ((t_now - t_prev) > weaponFoilDebounce) {
      //Serial.println("Foil Trigger");Serial.println(t_now);
      t_prev = t_now;
      weaponState.update_flag = true;
      weaponState.tFoilTrigger = t_now;
    }
  }
  prevPins = newPins;
}*/

void loop() {
  // put your main code here, to run repeatedly:
  long t_now = millis();
  static long t_LCD_upd = 0;
  static long t_LED_upd = 0;
  static long t_LED_reset = 0;
  static long t_Serial_upd = 0;
  static long t_Battery_Check = 0;
  static bool force_update = false;
  static bool valueChanged = false;
  static bool idleLEDIsOn = false;
  static long tIdleLEDOn = 0;
  static bool bLCDOff=false;

  //bitWrite(PORTD,PORTD3,!bitRead(PORTD,PORTD3));

  //checkButtonState(); //Handle button pushes for mode states

  if (t_now - t_LCD_upd > tLCDRefresh) {
    force_update = true;
  }
  if (t_now - t_LED_upd > tLEDRefresh) {
    force_update = true;
  }

  switch (BoxState) {
    case 'c':
      if (force_update) {
        tic=micros();
        for (int k = 0; k < NUM_ADC_SCAN_CHANNELS; k++) {
          ChanArray[k].updateVals();
        }
        updateCableState();
        force_update = false;
        toc=micros();
        timing_seg=toc-tic;
      }
      break;
    case 'r':
      if (force_update) {
        FoilADC.updateVals();
        EpeeADC.updateVals();
      }
      updateWeaponResistance();
      break;
    case 'w':
      //updateWeaponState();
      break;
    case 's':
      break;
  }

  //Automatic power off while idle
  if ((t_now - tLastActive) > powerOffTimeOut) {
    //setPowerOff();
  }

  /*
  if ((t_now - tLastActive) > tLCDIdleOff) {
    if ((cableState.cableDC) || (BoxState == 'w')) {
      lcd.setBacklight(0); //Turn off the display
      bLCDOff=true;
      if ((t_now - tIdleLEDOn) > tIdleLEDBlink) {
        tIdleLEDOn=millis();
        if (!idleLEDIsOn) {
          LED_block1[0] = ledColorBlue;
          idleLEDIsOn=true;          
        } else {
          LED_block1[0] = CRGB::Black;
          idleLEDIsOn = false;
        }
        FastLED.show();
      }
    } 
  } else {
      if (bLCDOff) {
        lcd.setBacklight(1); //Turn backlight on
      }
      if (idleLEDIsOn) {
        LED_block1[0] = CRGB::Black;
        idleLEDIsOn = false;
      }
  }*/
  /*
  if (((t_now - t_Battery_Check) > tBatteryInterval) && ((t_now - tLastActive) > tBatteryInterval)) {
    CheckBatteryStatus();
    t_Battery_Check = millis();
    if ((batteryVoltage < 3.5) && (batteryVoltage > 2.0)) {
      lcd.setCursor(0, 0);
      lcd.print(F("  Low Battery  "));
      t_LCD_upd = millis() + 1000;
    }
  }*/

  if (t_now - t_Serial_upd > tSerialRefresh) {
    writeSerialOutput(BoxState);
    t_Serial_upd = millis();
    //Serial.print("Timing (us) = ");Serial.println(timing_seg);
  }

  if (t_now - t_LCD_upd > tLCDRefresh) {
    // Display function would go here
    digitalWrite(LED1_PIN,!digitalRead(LED1_PIN));
    //updateLCDDisplay(BoxState);
    t_LCD_upd = millis();
  }
}
