

#define ARM_ARCH_7EM //Target Cortex-M4F
#define ARM_MATH_CM4 //Required for arm_math library
#define __FPU_PRESENT 1

#define TTArmBoardRev 2 

// Used for defining board revision specific changes and features
#if (TTArmBoardRev>=2)
  #define TTArmMOSFETWeaponAC 1 //Present on rev2 boards, Rev 2 boards have a battery disconnect switch
#else
  #define TTArmMOSFETWeaponAC 0 
#endif

#include <arm_math.h>

#include <Arduino.h>
#include "boardSelection.h"
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1351.h>
#include <nrf_saadc.h>
#include <avr/dtostrf.h>
#include <Adafruit_LittleFS.h>
#include <InternalFileSystem.h>
#include <nrfx_gpiote.h>
//#include <string.h>

using namespace Adafruit_LittleFS_Namespace;

//#include <nrf_adc.h>
//#include <sdk_config.h>

#include "Channel.h"
#include "string.h"
#include "CaptureBuffer.h"
#include "oledGraphClass.h"
#include "oledDisplaySettings.h"

#define DISPLAY_SPLASH_IMAGE 1
#define FAST_LED_ACTIVE 1

#if FAST_LED_ACTIVE 
#define FASTLED_INTERRUPT_RETRY_COUNT 3
#include <FastLED.h>
//Required for FAST LED
#define LED_TYPE WS2812B
#define COLOR_ORDER GRB
//#define LED_DATA_PIN 25 //LEDs are on SDA/pin25
CRGB lameLED;
#endif



static const char VERSION_NUM[16] = "1.2-1.3"; //Version-Adafruit Feather board version
static const char BUILD_DATE[16] = "2024-03-25";

#ifdef DISPLAY_SPLASH_IMAGE
#include "splashScreenImage.c"
#else 
#define DISPLAY_SPLASH_IMAGE 0
#endif

#define NUM_ADC_SCAN_CHANNELS 9 //9 combinations 
#define OHM_FIELD_WIDTH 4 //Width of the text display for ohms
#define SERIAL_OUTPUT_BUFFER_SIZE 256 //Length of serial buffer
#define ADC_BUFFER_SIZE 8
#define OPEN_CIRCUIT 999.9

const uint8_t ADC_UNIT = 0;
nrf_saadc_channel_config_t ADC_CONFIG = {.resistor_p = NRF_SAADC_RESISTOR_DISABLED,
                                         .resistor_n = NRF_SAADC_RESISTOR_PULLDOWN,
                                         .gain = NRF_SAADC_GAIN1_4,
                                         .reference = NRF_SAADC_REFERENCE_INTERNAL,
                                         .acq_time = NRF_SAADC_ACQTIME_40US,
                                         .mode = NRF_SAADC_MODE_SINGLE_ENDED,
                                         .burst = NRF_SAADC_BURST_ENABLED
                                         //.pin_p = NRF_SAADC_INPUT_AIN0,
                                         //.pin_n = NRF_SAADC_INPUT_DISABLED
                                        };
nrf_saadc_channel_config_t FAST_ADC_CONFIG = {.resistor_p = NRF_SAADC_RESISTOR_DISABLED,
                                         .resistor_n = NRF_SAADC_RESISTOR_PULLDOWN,
                                         .gain = NRF_SAADC_GAIN1_4,
                                         .reference = NRF_SAADC_REFERENCE_INTERNAL,
                                         .acq_time = NRF_SAADC_ACQTIME_20US,
                                         .mode = NRF_SAADC_MODE_SINGLE_ENDED,
                                         .burst = NRF_SAADC_BURST_ENABLED
                                         //.pin_p = NRF_SAADC_INPUT_AIN0,
                                         //.pin_n = NRF_SAADC_INPUT_DISABLED
                                        };
nrf_saadc_value_t ADC_Buffer1[ADC_BUFFER_SIZE]; //Buffer for ADC sample reads

//Assumes 50Hz downsampled frequency, 2nd order Butterworth filter coefs
// http://www.micromodeler.com/dsp/
float LowPass8HzCoef[5] = {// Scaled for floating point
  0.1340603846876515, 0.268120769375303, 0.1340603846876515, 0.6190201888761353, -0.15526172762674134// b0, b1, b2, a1, a2
};
float LowPass5HzCoef[5] = {// Scaled for floating point
  0.06745527388907192, 0.13491054777814385, 0.06745527388907192, 1.142980502539901, -0.41280159809618855// b0, b1, b2, a1, a2
};
float LowPass3HzCoef[5] = {// Scaled for floating point
  0.02785976611713601, 0.05571953223427202, 0.02785976611713601, 1.4754804435926463, -0.5869195080611904// b0, b1, b2, a1, a2
};
float LowPass2HzCoef[5] = {// Scaled for floating point
    0.01335920002785649, 0.02671840005571298, 0.01335920002785649, 1.6474599810769768, -0.7008967811884027// b0, b1, b2, a1, a2
};
float LowPass1HzCoef[5] = {// Scaled for floating point
  0.02785976611713601, 0.05571953223427202, 0.02785976611713601, 1.4754804435926463, -0.5869195080611904// b0, b1, b2, a1, a2
};
float LowPass0p2HzCoef[5] = {// Scaled for floating point
  0.00015514842347569914, 0.0003102968469513983, 0.00015514842347569914, 1.9644605802052322, -0.9650811738991351// b0, b1, b2, a1, a2
};
float LameLPF1HzCoef[5] = {
  // Scaled for floating point
    0.1367287359973195, 0.1367287359973195, 0, 0.726542528005361, 0// b0, b1, b2, a1, a2
};
float LameLPF2HzCoef[5] = {
  // Scaled for floating point
  0.1367287359973195, 0.1367287359973195, 0, 0.726542528005361, 0// b0, b1, b2, a1, a2
};


// Change the calibration valid flag when changing the format of the calibration data
const byte calibrationValid = 0xA0; //Flags the last valid calibration
const byte calibrationInvalid = 0xEE; //Marks a calibration as invalid and moves to the next location (used for wear leveling)
//int eepromLocationStep  = 1 + sizeof(int) * (NUM_ADC_SCAN_CHANNELS + 2); //Step size in bytes between eeprom flags;
const int calibrationErrorValue = 1000; //If calibration value exceeds, generate an error
const byte calibrationRetries = 3; // Exit after this many retries

// Pile of various parameters and constants for the box to use
const int maxADCthreshold = 4000; //Used for switching between high/low gain
const int shortADCthreshold = 3000; //ADC values below this will show as a short
//const int minADCthreshold = 20; //Used for switching between high/low gain
constexpr long powerOffTimeOut = 15L * 60L * 1000L; //Time before switching off if inactive;
const long cableDisconnectTimeOut = 1000L; //Time out before flagging the cable as disconnected
const long weaponDisconnectTimeOut = 10000L; //Time out before flagging the cable as disconnected
const int weaponStateHoldTime = 500; //ms - How long the light remains lit after a weapon-press
const long weaponFoilDebounce = 15; //ms - How long the light remains lit after a weapon-press
const int weaponEpeeDebounce = 3; //ms - How long the light remains lit after a weapon-press
static constexpr int usFoilDebounce = weaponFoilDebounce * 1000; //Foil debounce in us
static constexpr int usEpeeDebounce = weaponEpeeDebounce * 1000; //Foil debounce in us
const int t_Error_Display = 2000; //ms - How long to display error/debug messages;
const int tLCDRefresh = 400; //ms - How often to refresh the lcd display
const int tLEDRefresh = 50; //ms - How often to refresh the lcd display
const int tOLEDRefresh = 20; //ms - How often to refresh the OLED display
//constexpr long tDimOLED = 15L * 1000L; //ms - How long before the OLED dims
constexpr long tOledOff = 5L*60L * 1000L; //ms - How long before the OLED turns off
//const long tLEDResync = 10000; //ms -- Completely reset the LED display
const long tBatteryInterval = 10000; //ms - Check battery every 10s
const long tIdleModeOn = 5000L; //ms - Switch to idle mode after 5s of in-activity.
//const uint8_t weaponIdleMultiplier = 4; //ms - Switch to idle mode after 20s of in-activity.
const long tIdleWakeUpInterval = 20; //ms - How often to check inputs for changes while idle
const int tSerialRefresh = 500; //ms - How often to send data over the serial port
const int tPowerOffPress = 1500; //ms - How long to hold the button down before it's considered a long press
const int tModeSwitchLockOut = 500; //ms - Used to prevent accidental double mode switches
const int tEnterCalibrationMode = 4000; //ms - How long to hold before entering calibration mode
const int tIdleLEDBlink = 750; //ms
const int wdtTimerResetInterval = 3; //s
const int tMaxHold = 1000; //ms -- Duration for a min/max hold value
const long dispCaptureHoldTime = 500; //ms -- Minimum Duration that a hit capture displays for
const long tLameWaitTime = 1000; //ms -- Duration before switch to lame mode from cable mode.

const float HIGH_RESISTANCE_THRESHOLD = 5.0;
const int CABLE_DISCONNECT_THRESHOLD = 4000;
const float OPEN_CIRCUIT_VALUE = 999.9;
const float MAX_LAME_RESISTANCE = 40.0f;

//MUX Settings if needed
const SPISettings OLED_SPI_SETTINGS(2000000, MSBFIRST, SPI_MODE0);
//const uint8_t MUX_LATCH = 16;
//const uint8_t MUX_CLK = 15;
//const uint8_t MUX_DATA = 7;

//MUX is MSBFIRST, bit 0=NC, bits 1-3=source,bit 4=weaponB, bits 5-7=sink, 
//For rev 2 board MUX is MSBFIRST, bit 0=WeaponC GND, bits 1-3=source A/B/C,bit 4=WeaponB GND, bits 5-7=Cable A/B/C
const byte MUX_DISABLED = 0x0;
const byte MUX_CABLE_AA = B00100010;
const byte MUX_CABLE_AB = B01000010;
const byte MUX_CABLE_AC = B10000010;
const byte MUX_CABLE_BA = B00100100;
const byte MUX_CABLE_BB = B01000100;
const byte MUX_CABLE_BC = B10000100;
const byte MUX_CABLE_CA = B00101000;
const byte MUX_CABLE_CB = B01001000;
const byte MUX_CABLE_CC = B10001000;
const byte MUX_WEAPON_MODE = B00011010; //Source=A & C, Sink=B
const byte MUX_WEAPON_AB = B00010010;
const byte MUX_WEAPON_CB = B00011000;
#if TTArmMOSFETWeaponAC
  const byte MUX_WEAPON_AC = B00000011; //Only A is source, C relies on pull-down resistor
#else
  const byte MUX_WEAPON_AC = B00000011; //Only A is source, C relies on pull-down resistor
#endif

nrfx_gpiote_in_config_t weaponPinConfig = {
  .sense = NRF_GPIOTE_POLARITY_TOGGLE,
  .pull = NRF_GPIO_PIN_NOPULL,
  .is_watcher = false,
  .hi_accuracy = true,
  .skip_gpio_setup = false
};

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
//Defined in "boardSelection.h"
//const byte POWER_CONTROL = 11;
//const byte DIAG_PIN = 25;
//const byte BUTTON_PIN = 27;
//const byte LED1_PIN = 17;
//const byte LED2_PIN = 19;
//const byte FASTLED_PIN 

// Basic variables
volatile long numSamples = 0;
volatile float EffSampleRate;
float batteryVoltage = 3.7;

//Some timing stuff if needed
volatile long timing_seg = 0;
volatile long tic = 0;
volatile long toc = 0;
volatile long tLastActive = 0; //ms - Time that an event was last detected

bool wdtOverride = false;

// Store the box state
enum TestBoxModes {
  CABLE,
  WPN_TEST,
  WPN_GRAPH,
  HIT_CAPTURE,
  PROBE,
  BOX_IDLE,
  BOX_OFF
};
volatile TestBoxModes BoxState = BOX_IDLE; //i=Idle; c=Cable; w=Weapon; r=WeaponResistance; s=sleep;

enum batteryDisplayModes {
  NONE,
  PERCENT,
  VOLTAGE
};
batteryDisplayModes batteryDisplayType = PERCENT;

// Option 1: use any pins but a little slower
//Adafruit_SSD1351 tft = Adafruit_SSD1351(SCREEN_WIDTH, SCREEN_HEIGHT, CS_PIN, DC_PIN, MOSI_PIN, SCLK_PIN, RST_PIN);

// Option 2: must use the hardware SPI pins
// (for UNO thats sclk = 13 and sid = 11) and pin 10 must be
// an output. This is much faster - also required if you want
// to use the microSD card (see the image drawing example)
//SPIClass oledSPI = SPIClass(NRF_SPIM0, PIN_SPI_MISO, SCLK_PIN, MOSI_PIN);
//oledSPI->begin();
Adafruit_SSD1351 tft = Adafruit_SSD1351(SCREEN_WIDTH, SCREEN_HEIGHT, &SPI, CS_PIN, DC_PIN);
//Adafruit_SSD1351 tft = Adafruit_SSD1351(SCREEN_WIDTH, SCREEN_HEIGHT, &oledSPI, CS_PIN, DC_PIN);

oledColorList colorList;
oledGraph lameGraph;
oledReverseHBarGraph lameBar;
oledGraphLabel lameLabel;
oledGraph weaponGraph;
oledGraph captureGraph;

oledReverseHBarGraph lineABar;
oledGraphLabel lineALabel;
oledReverseHBarGraph lineBBar;
oledGraphLabel lineBLabel;
oledReverseHBarGraph lineCBar;
oledGraphLabel lineCLabel;

oledReverseHBarGraph prEpeeBar;
oledGraphLabel prEpeeLabel;
oledReverseHBarGraph prFoilBar;
oledGraphLabel prFoilLabel;
oledReverseHBarGraph prACBar;
oledGraphLabel prACLabel;
oledReverseHBarGraph prABar;
oledGraphLabel prALabel;
oledReverseHBarGraph prBBar;
oledGraphLabel prBLabel;
oledReverseHBarGraph prCBar;
oledGraphLabel prCLabel;



// ADC timer settings;


//Serial buffer string
char outputString[SERIAL_OUTPUT_BUFFER_SIZE];
volatile long tIdle = 0;
//BluetoothSerial SerialBT;


//File settingsFile(InternalFS);
//File calFile(InternalFS);
//#define CAL_FILENAME "/ADC_Calibration.txt"
//#define SETTINGS_FILENAME "/Settings.txt"


//ADC parameters
//ADC_Channel ChanArray[NUM_ADC_SCAN_CHANNELS] {0, 3, 3, 4, 1, 4, 5, 5, 2}; //AA, AB, AC, BA, BB, BC, CA, CB, CC
ADC_Channel ChanArray[NUM_ADC_SCAN_CHANNELS] {AINpinA_amp, AINpinA_raw, AINpinA_raw, AINpinB_raw, AINpinB_amp, AINpinB_raw, AINpinC_raw, AINpinC_raw, AINpinC_amp}; //AA, AB, AC, BA, BB, BC, CA, CB, CC
ADC_Channel ProbeArray[6] {AINpinA_amp, AINpinC_amp, AINpinA_amp, AINpinB_amp, AINpinA_amp, AINpinC_amp}; // Epee (AB), Foil (CB), WepGnd (AC), BPrA, APrA,  CPrA
//const byte ChannelScanOrder[NUM_ADC_SCAN_CHANNELS] = {1, 2, 4, 5, 3, 8, 7, 0, 6}; //Array showing the *Next channel, so Ch0 -> Ch1, Ch8->Ch0
const byte ChannelScanOrder[NUM_ADC_SCAN_CHANNELS] = {1, 2, 3, 4, 5, 6, 7, 8, 0}; //Array showing the *Next channel, so Ch0 -> Ch1, Ch8->Ch0
ADC_Channel FoilADC(AINpinC_amp);
ADC_Channel EpeeADC(AINpinA_amp);
ADC_Channel WeaponAC(AINpinC_raw);
ADC_Channel CableCheck[3] {AINpinA_amp,AINpinB_amp,AINpinC_amp};
//ADC_Channel BatteryMonitor(5);
ADC_Channel* ActiveCh;
static constexpr byte NUM_CAL_CHANNELS = NUM_ADC_SCAN_CHANNELS + 2;

//Buffers for high-speed capture
#define PRE_TRIGGER_SIZE 20
#define ADC_CAPTURE_LEN 128 //128-PreTrigger
//Declare arrays here so that we can use pointers internally to channels
int ADC_PreTrigEpee[2][PRE_TRIGGER_SIZE];
int ADC_PreTrigFoil[2][PRE_TRIGGER_SIZE];
int ADC_CaptureBuffer[ADC_CAPTURE_LEN]; //Buffer for ADC sample reads

//Struct defining the cable/lame properties
struct CableData {
  uint16_t statusByte = (1 << BITAA) || (1 < BITBB) || (1 < BITCC);
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
  bool maskMode = false;  //Mask clip flag
  float ohm_AA = OPEN_CIRCUIT_VALUE;
  float ohm_BB = OPEN_CIRCUIT_VALUE;
  float ohm_CC = OPEN_CIRCUIT_VALUE;
  float ohm_Lame = OPEN_CIRCUIT_VALUE;
  float ohm_AAMax = OPEN_CIRCUIT_VALUE;
  float ohm_BBMax = OPEN_CIRCUIT_VALUE;
  float ohm_CCMax = OPEN_CIRCUIT_VALUE;
  float ohm_LameMax = OPEN_CIRCUIT_VALUE;
  long tLastConnect = 0;
  float cableOhm[9];

  arm_biquad_casd_df1_inst_f32 LineALowPass;
  float LineALPFState[4];
  arm_biquad_casd_df1_inst_f32 LineBLowPass;
  float LineBLPFState[4];
  arm_biquad_casd_df1_inst_f32 LineCLowPass;
  float LineCLPFState[4];
  arm_biquad_casd_df1_inst_f32 LameLowPass;
  float LameLPFState[4];
};

struct IdleDataStruct {
  uint16_t cableStatus;
  bool foilActive = false;
  bool epeeActive = false;
};

CableData cableState;

struct weapon_test {
  bool epeeOn = false;
  bool epeeInterOn = false;
  long tEpeeTrigger = 0;
  long tEpeeInterOn = 0;
  bool foilOn = false;
  bool foilInterOn = false;
  long tFoilTrigger = 0;
  long tFoilInterOn = 0;
  bool lineAC = false;
  //byte epeeInterruptBit = PCINT4;
  //byte foilInterruptBit = PCINT6;
  long tLightChange = weaponStateHoldTime; //ms -- time for the intermittent LED to be on
  byte update_flag = false;
  float ohm_Foil = 0;
  int ohm10xFoil = 0;
  float ohm_Epee = 0;
  int ohm10xEpee = 0;
  float ohm_FoilMax = 0;
  float ohm_EpeeMax = 0;
  long tLastConnect = 0;
  bool cableDC = true;

  arm_biquad_casd_df1_inst_f32 EpeeLowPass;
  float EpeeLPFState[4];
  arm_biquad_casd_df1_inst_f32 FoilLowPass;
  float FoilLPFState[4];
};

volatile weapon_test weaponState;

struct probeDataStruct {
  float ohm_APr = OPEN_CIRCUIT_VALUE;
  float ohm_BPr = OPEN_CIRCUIT_VALUE;
  float ohm_CPr = OPEN_CIRCUIT_VALUE;
  float ohm_Foil = OPEN_CIRCUIT_VALUE;
  float ohm_Epee = OPEN_CIRCUIT_VALUE;
  float ohm_WpnAC = OPEN_CIRCUIT_VALUE; 

  arm_biquad_casd_df1_inst_f32 ProbeALPF;
  float ProbeALPFState[4];
  arm_biquad_casd_df1_inst_f32 ProbeBLPF;
  float ProbeBLPFState[4];
  arm_biquad_casd_df1_inst_f32 ProbeCLPF;
  float ProbeCLPFState[4];
  arm_biquad_casd_df1_inst_f32 FoilLPF;
  float FoilLPFState[4];
  arm_biquad_casd_df1_inst_f32 EpeeLPF;
  float EpeeLPFState[4];
  arm_biquad_casd_df1_inst_f32 WpnACLPF;
  float WpnACLPFState[4];
} probeData;

//bool BatteryCheck=false;

//Used for ADC sampling
#ifdef __cplusplus
extern "C" {
#endif
void SAADC_IRQHandler(void) {
  //digitalWrite(DIAG_PIN,HIGH);
  q31_t tempVal = 0;
  long t_now = millis();
  int ADCValue;
  float flVal = 0.0;
  static bool foilState=false;
  static bool epeeState=false;
  bool tempState=false;


  tempVal = 0;

  /*if (BatteryCheck) {
    Serial.println("We shouldn't be here.");
    return;
  }*/

  if (nrf_saadc_event_check(NRF_SAADC,NRF_SAADC_EVENT_RESULTDONE)) {
    nrf_saadc_event_clear(NRF_SAADC,NRF_SAADC_EVENT_RESULTDONE);
    ADCValue = *(nrf_saadc_buffer_pointer_get(NRF_SAADC));
  } else {
    //Serial.println("SAADC fault");
    return; //This should never happen
  }
  //digitalWrite(DIAG_PIN,HIGH);

  if ( (ActiveCh->ch_label[0] == 'A') || (ActiveCh->ch_label[0] == 'E') ) {
    numSamples++;  //Indicates a full scan was performed
    //Serial.println(ADCValue);
  }

  ActiveCh->lastValue = ADCValue;

  ActiveCh->sampleBuffer[ActiveCh->sampleCount] = (long) (ADCValue << 8); //Convert to Q1.31 format with a long cast and bit shift
  ActiveCh->sampleCount++;
  if (ActiveCh->sampleCount >= ActiveCh->FIR_BLOCK_SIZE) {
    //digitalWrite(DIAG_PIN,HIGH);
    arm_fir_decimate_fast_q31(&(ActiveCh->FIR_filter), ActiveCh->sampleBuffer, &(ActiveCh->filterValue), ActiveCh->FIR_BLOCK_SIZE);
    ActiveCh->filterValue = (ActiveCh->filterValue >> 8);
    ActiveCh->valueReady = true;
    ActiveCh->sampleCount = 0;
  }

  tempState=(ADCValue<maxADCthreshold);
  if (ActiveCh == &EpeeADC) {
    if (tempState!=epeeState) {
        ISR_EpeeHitDetect();
        epeeState=tempState;
    }
  }
  if (ActiveCh == &FoilADC) {
    if (tempState!=foilState) {
      ISR_FoilHitDetect();
      foilState=tempState;
    }
  }

  if ((t_now - ActiveCh->t_max) > tMaxHold) {
    ActiveCh->decay_max = 0;
  }

  if ((t_now - ActiveCh->t_min) > tMaxHold) {
    ActiveCh->decay_min = 4096;
  }

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

  if (ActiveCh->bufferEnabled) {
    ActiveCh->hsBuffer.AddSample(ActiveCh->lastValue);
  }

  //Load the next channel
  if (ActiveCh->ADC_Scan) {
    ActiveCh = ActiveCh->nextChannel;
    // Switch the MUX to toggle the MOSFETs
    digitalWrite(MUX_LATCH, LOW); //equivalent to digitalWrite(4, LOW); Toggle the SPI
    shiftOut(MUX_DATA, MUX_CLK, MSBFIRST, ActiveCh->muxSetting);
    //shiftOut(MUX_DATA, MUX_CLK, MSBFIRST, MUX_CABLE_AA);
    digitalWrite(MUX_LATCH, HIGH); //equivalent to digitalWrite(4,HIGH);
    //nrf_saadc_channel_pos_input_set(ADC_UNIT,ActiveCh->AIn);
    NRF_SAADC->CH[ADC_UNIT].PSELP = ActiveCh->AIn;

    nrf_saadc_task_trigger(NRF_SAADC,NRF_SAADC_TASK_START);
  }

  //Reset the buffer and trigger sampling
  nrf_saadc_buffer_init(NRF_SAADC,ADC_Buffer1, 1);
  nrf_saadc_task_trigger(NRF_SAADC,NRF_SAADC_TASK_SAMPLE);

  //digitalWrite(DIAG_PIN,LOW);
}
#ifdef __cplusplus
}
#endif

void wdtOverrideCallback(void *p_context) {
  //if (wdtOverride) {
    //Manually kick the watch dog
    //NRF_WDT->RR[0] = WDT_RR_RR_Reload; //Reload watchdog register 0
  //}
}

void setup() {
  // put your setup code here, to run once:
  pinMode(POWER_CONTROL, OUTPUT);
  digitalWrite(POWER_CONTROL, LOW);

  //Activate display
  delay(50); //Hold for half second to power on
  digitalWrite(POWER_CONTROL, HIGH);
  //Initialize the display
  InitializeDisplay();
  //delay(100);
  //Hold the power on
  delay(250);

  //Required to fix FPU prevent sleep bug
  //Likely no longer necessary with release 0.24
  //Enabling causes the code to hang
  //NVIC_SetPriority(FPU_IRQn, 100);
  //NVIC_EnableIRQ(FPU_IRQn);

  // Enable timing diagnostics
  pinMode(DIAG_PIN, OUTPUT);
  digitalWrite(DIAG_PIN, LOW);
  pinMode(LED1_PIN, OUTPUT);
  digitalWrite(LED1_PIN, LOW);
  pinMode(LED2_PIN, OUTPUT);
  digitalWrite(LED2_PIN, LOW);

  //EEPROM.begin(EEPROM_SIZE);

  //Initialize the MUX if needed
  //Serial.println("Starting MUX");
  pinMode(MUX_LATCH, OUTPUT);
  pinMode(MUX_CLK, OUTPUT);
  pinMode(MUX_DATA, OUTPUT);

  digitalWrite(MUX_LATCH, LOW);
  digitalWrite(MUX_CLK, LOW);
  digitalWrite(MUX_DATA, LOW);

  //initialise vspi with default pins
  //SCLK = 18, MISO = 19, MOSI = 23, SS = 5
  digitalWrite(MUX_LATCH, LOW); //equivalent to digitalWrite(4, LOW); Toggle the SPI
  shiftOut(MUX_DATA, MUX_CLK, MSBFIRST, MUX_DISABLED);
  digitalWrite(MUX_LATCH, HIGH); //equivalent to digitalWrite(4,HIGH);
  //while (true) {}

  //Enable button interrupts
  pinMode(BUTTON_PIN, INPUT);

  //Wait for boot to start Serial
  Serial.begin(115200);
  InternalFS.begin();

  //Initialize the various channel settings
  Serial.println("Initializing channels");
  //delay(500);
  InitializeChannels();
  InitializeADC(false);

  CheckBatteryStatus();
  //displayBatteryStatus();
  wdt_init();
  //app_timer_start(wdtOverrideTimer,6554,true);
  
  InitializeCableData();
  InitializeWeaponData();
  tLastActive = -1 * (tIdleModeOn + 1); //Put the box into idle mode initially
  //tLastActive= 0;

  //delay(1000);
  setBoxMode(BOX_IDLE);  //Start the box
  Serial.println("Setup complete");

  //BlinkLEDThenPowerOff();
  //delay(500);
  #if FAST_LED_ACTIVE
  lameLED=CRGB::Black;
  FastLED.show();
  #endif
  
}

/*
//Used to clear FPU interrupt so sleep works
#ifdef __cplusplus
extern "C" {
#endif
#define FPU_EXCEPTION_MASK 0x0000009F
void FPU_IRQHandler(void)
{
  uint32_t *fpscr = (uint32_t *)(FPU->FPCAR + 0x40);
  (void)__get_FPSCR();

  *fpscr = *fpscr & ~(FPU_EXCEPTION_MASK);

  //Serial.println("FPU event");
}
#ifdef __cplusplus
}
#endif
*/

// Pin change interrupt handler.  Used for weapon test mode.
// New function needed for ESP32

//void ISR_EpeeHitDetect(nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t action) {
void ISR_EpeeHitDetect() {
  long t_now = millis();
  static long t_prev = 0;
  //byte changed = 0;
  //byte state = nrf_gpio_pin_read(LineADetect);
  //byte pin = LineADetect;

  //tLastActive = t_now;
  //Serial.println("Epee ISR");
  if ((t_now - t_prev) > weaponEpeeDebounce) {
    //Serial.print("Epee Trigger t="); Serial.println(t_now);
    t_prev = t_now;
    weaponState.update_flag = true;
    weaponState.tEpeeTrigger = t_now;
  }
}

void ISR_FoilHitDetect() {
  long t_now = millis();
  static long t_prev = 0;
  //byte changed = 0;
  //byte state = nrf_gpio_pin_read(LineCDetect);

  //tLastActive = t_now;
  if ((t_now - t_prev) > weaponFoilDebounce) {
    //Serial.println("Foil Trigger"); Serial.println(t_now);
    t_prev = t_now;
    weaponState.update_flag = true;
    weaponState.tFoilTrigger = t_now;
  }
}

void wdt_init(void)
{
  NRF_WDT->CONFIG = (WDT_CONFIG_HALT_Pause << WDT_CONFIG_HALT_Pos) | ( WDT_CONFIG_SLEEP_Run << WDT_CONFIG_SLEEP_Pos);
  NRF_WDT->CRV = wdtTimerResetInterval*32768; // Watchdog timer reset interval
  NRF_WDT->RREN |= WDT_RREN_RR0_Msk; //Enable reload register 0
  NRF_WDT->TASKS_START = 1;
}



void loop() {
  // put your main code here, to run repeatedly:
  long t_now = millis();
  static long t_OLED_upd = 0;
  static long t_LED_reset = 0;
  static long t_Serial_upd = 0;
  static long t_Battery_Check = 0;
  static long t_idle_Check = 0;
  static bool force_update = false;
  static bool valueChanged = false;
  static bool bLCDOff = false;
  bool updateReady = false;

  //bitWrite(PORTD,PORTD3,!bitRead(PORTD,PORTD3));

  checkButtonState(); //Handle button pushes for mode states

  switch (BoxState) {
    case CABLE:
      updateReady = true;
      for (int k = 0; k < NUM_ADC_SCAN_CHANNELS; k++) {
        if (!ChanArray[k].valueReady) {
          updateReady = false;
        }
      }
      if (updateReady) {
        updateCableState();
        //toc = micros();
        //timing_seg = toc - tic;
        //updateOLED(BoxState);
        if ((cableState.cableDC) && ( (t_now - t_idle_Check) > tIdleWakeUpInterval) && ((t_now-tLastActive)>cableDisconnectTimeOut)) {
          //Serial.println("Checking idle connections");
          if (checkWeaponConnected()) {
            Serial.println("Weapon connected, switching mode");
            setBoxMode(WPN_GRAPH);
            return;
          }
          checkCableConnected();
          StartADC();
          t_idle_Check = millis();
          //updateIdleMode();
        }
      }
      break;
    case HIT_CAPTURE:
    case WPN_GRAPH:
      //FoilADC.updateVals();
      //EpeeADC.updateVals();
      updateWeaponResistance();
      updateWeaponState();
      if ((weaponState.cableDC) && ( (t_now - t_idle_Check) > tIdleWakeUpInterval)) {
        if (checkCableConnected()) {
          setBoxMode(CABLE);
          return;
        }
        checkWeaponConnected();
        StartADC();
        t_idle_Check = millis();
      }
      break;
    case WPN_TEST:
      updateWeaponStateDigital();
      break;
    case PROBE:
      updateProbe();
      break;
    case BOX_IDLE:
      delay(tIdleWakeUpInterval);
      updateIdleMode();
      //timing_seg=toc-tic;
      break;
  }

  //Automatic power off while idle
  if ((t_now - tLastActive) > powerOffTimeOut) {
    Serial.println("Idle Power Off");
    setPowerOff();
  }

  if ((t_now - tLastActive) > tIdleModeOn) {
    if ((BoxState == CABLE) && (cableState.cableDC)) {
      setBoxMode(BOX_IDLE);
    }
    if ((BoxState == WPN_GRAPH) && (weaponState.cableDC))  {
      if (!cableState.cableDC) {
        setBoxMode(BOX_IDLE);
      } else {
        setBoxMode(CABLE);
      }
    }
    if ((BoxState == WPN_TEST) && (!weaponState.foilOn) && (!weaponState.epeeOn)) {
      //setBoxMode(BOX_IDLE);
    }
    //Serial.println("Setting idle mode");
  } else {
    if (BoxState == BOX_IDLE) {
      if (!cableState.cableDC) {
        //Serial.println("Wake up!");
        setBoxMode(CABLE);
      } else {
        if (!weaponState.cableDC) {
          setBoxMode(WPN_GRAPH);
        }
      }
    }
  }

  if ((t_now - t_Battery_Check) > tBatteryInterval) {
    digitalWrite(LED1_PIN, HIGH);
    CheckBatteryStatus();
    t_Battery_Check = millis();
    displayBatteryStatus();
    digitalWrite(LED1_PIN, LOW);
  }

  if (t_now - t_Serial_upd > tSerialRefresh) {
    writeSerialOutput(BoxState);
    //Serial.print("tActive= ");Serial.println(tLastActive);
    t_Serial_upd = millis();
    //Serial.print("Timing (us) = ");Serial.println(timing_seg);
  }

  if (t_now - t_OLED_upd > tOLEDRefresh) {
    //digitalWrite(DIAG_PIN,HIGH);
    //if ( ((t_now - tLastActive) > tDimOLED) && (BoxState != 'i') ) {
    //dimOLEDDisplay();
    //updateOLED('d');
    //} else {
    updateOLED(BoxState);
    //}
    //digitalWrite(DIAG_PIN,LOW);
    t_OLED_upd = millis();
    NRF_WDT->RR[0] = WDT_RR_RR_Reload; //Reload watchdog register 0
    //Serial.print("Timing (us) = ");Serial.println(timing_seg);
  }
}
