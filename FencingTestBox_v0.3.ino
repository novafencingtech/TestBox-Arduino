#include <Wire.h>
#include <LiquidCrystal_PCF8574.h>
#include <EEPROM.h>
//#include <LowPower.h>
#include <SPI.h>
//#include <avr/interrupt.h>
//#include <avr/pgmspace.h>
#include "Channel.h"
#include "string.h"
#include "LED_Display.h"

#define NUM_ADC_SCAN_CHANNELS 6
#define PINx PINB
#define BANANA_DIG_IN PINB
#define LCD_TEXT_COLS 16
#define LCD_TEXT_ROWS 2
#define OHM_FIELD_WIDTH 4 //Width of the text display for ohms
#define SERIAL_BUFFER_SIZE 81 //Length of serial buffer 


// Constants
//const float maxLEDthreshold=255;
const CRGB ledColorGreen = CRGB(0, 5, 0);
const CRGB ledColorRed = CRGB(10, 0, 0);
const CRGB ledColorBlue = CRGB(0, 0, 7);
const CRGB ledColorWhite = CRGB(8, 8, 8);
const CRGB ledColorYellow = CRGB(5, 5, 0);
const CRGB ledColorOrange = CRGB(10, 3, 0);

// Change the calibration valid flag when changing the format of the calibration data
const byte calibrationValid = 0xFA; //Flags the last valid calibration
const byte calibrationInvalid = 0xEE; //Marks a calibration as invalid and moves to the next location (used for wear leveling)
int eepromLocationStep  = (1 + NUM_ADC_SCAN_CHANNELS+2); //Step size in bytes between eeprom flags;
const byte calibrationErrorValue = 200; //If calibration value exceeds, generate an error
const byte calibrationRetries = 3; // Exit after this many retries

const byte maxADCthreshold = 200; //Used for switching between high/low gain
const byte minADCthreshold = 20; //Used for switching between high/low gain
const long powerOffTimeOut = 180000; //Time before switching to idle mode for scanning (in ms);
const int idleDisconnectTime = 5000; //Time before switching to idle mode for scanning (in ms);
const int weaponStateHoldTime = 250; //ms - How long the light remains lit after a weapon-press
const int weaponFoilDebounce = 15; //ms - How long the light remains lit after a weapon-press
const int weaponEpeeDebounce = 3; //ms - How long the light remains lit after a weapon-press
const int t_Error_Display = 2000; //ms - How long to display error/debug messages;
const int tLCDRefresh = 400; //ms - How often to refresh the lcd display
const int tLEDRefresh = 50; //ms - How often to refresh the lcd display
const long tLEDResync = 10000; //ms -- Completely reset the LED display
const long tBatteryInterval = 30000; //ms - Check battery every 30s
const long tLCDIdleOff = 30000; //ms - Turn LCD off if idle for more than 1 min
const int tSerialRefresh = 200; //ms - How often to send data over the serial port
const int tPowerOffPress = 1500; //ms - How long to hold the button down before it's considered a long press
const int tModeSwitchLockOut = 300; //ms - Used to prevent accidental double mode switches
const int tEnterCalibrationMode = 4000; //ms - How long to hold before entering calibration mode
const int tIdleLEDBlink = 750; //ms

const byte ADC_STABLE_CYCLES = 3;

const float HIGH_RESISTANCE_THRESHOLD = 5.0;
const int CABLE_DISCONNECT_THRESHOLD = 1023;
const float OPEN_CIRCUIT_VALUE = 99.0;

const SPISettings MUX_SPI_SETTINGS(2000000, MSBFIRST, SPI_MODE0);
const byte MUX_LATCH = PORTD4;
const byte MUX_CABLE_AA = B00010010;
const byte MUX_CABLE_AB = B00100010;
const byte MUX_CABLE_AC = B01000010;
const byte MUX_CABLE_BB = B00100100;
const byte MUX_CABLE_BC = B01000100;
const byte MUX_CABLE_CC = B01001000;
const byte MUX_WEAPON_MODE = B00000000;

const byte BITAA = 0;
const byte BITAB = 1;
const byte BITAC = 2;
const byte BITBC = 3;
const byte BITBB = 4;
const byte BITCC = 5;

const byte POWER_CONTROL = PORTD7;
const byte DIAG_PORT = PORTD3;

// Basic variables
volatile long numSamples = 0;
volatile float EffSampleRate;
byte batteryCharge = 0;

volatile long timing_seg = 0;
volatile long tic = 0;
volatile long toc = 0;
volatile long tLastActive = 0; //ms - Time that an event was last detected

volatile char BoxState = 'w'; //i=Idle; c=Cable; w=Weapon; r=WeaponResistance; s=sleep;

char outputString[SERIAL_BUFFER_SIZE];
char lcdString1[LCD_TEXT_COLS + 1] = "";
char lcdString2[LCD_TEXT_COLS + 1] = "";
bool NewErrorFlag = false;
long t_error;
volatile long tIdle = 0;

float batteryVoltage = 3.7;

int eepromAddr = 0; //Used to store the current address used by the eeprom


//ADC_Channel ChanArray[NUM_ADC_SCAN_CHANNELS]{{7,11},11,11,12,{6,12},{5,10}}; //AA, AB, AC, BC, BB, CC
ADC_Channel ChanArray[NUM_ADC_SCAN_CHANNELS] {7, 11, 12, 11, 6, 5}; //AA, AB, BC, AC, BB, CC
//ADC_Channel FoilADC(6);
//ADC_Channel EpeeADC(6);
volatile ADC_Channel* ActiveCh;

//Required for FAST LED
#define LED_TYPE WS2812B
#define COLOR_ORDER GRB
#define DATA_PIN_LED_BLOCK1 5 //LEDs are on D5
//LED_Display(CRGB[] FastLEDArray, char numLEDs, char offsetLED, char displayMode)

//Choose the correct settings for the length of the LED bar graphs
#define LED6_GRAPH 0 
#define LED12_GRAPH 1 

#if (LED12_GRAPH)
  #define NUM_LEDS_BLOCK1 36
  CRGB LED_block1[NUM_LEDS_BLOCK1];
  LED_Display lineAGauge(LED_block1, 12, 0, 'g');
  LED_Display lineBGauge(LED_block1, 12, 12, 'g');
  LED_Display lineCGauge(LED_block1, 12, 24, 'g');
#endif
#if (LED6_GRAPH)
  #define NUM_LEDS_BLOCK1 18
  CRGB LED_block1[NUM_LEDS_BLOCK1];
  LED_Display lineAGauge(LED_block1, 6, 0, 'g');
  LED_Display lineBGauge(LED_block1, 6, 6, 'g');
  LED_Display lineCGauge(LED_block1, 6, 12, 'g');
#endif

// LCD display
LiquidCrystal_PCF8574 lcd(0x27);

struct testbox_line {
  byte directionBit;
  byte stateBit;
  byte digitalInMask;
  ADC_Channel* analogIn;
  byte pin_change;
};

struct CableData {
  byte statusByte = 0;
  int line_AB = 255;
  int line_AC = 255;
  int line_BC = 255;
  bool update_flag = false;
  bool error_msg = false;
  bool cableDC = false;
  bool lameMode = false;
  float ohm_AA = 0;
  float ohm_BB = 0;
  float ohm_CC = 0;
  float ohm_AAMax = 0;
  float ohm_BBMax = 0;
  float ohm_CCMax = 0;
  long tLastConnect = 0;
};

struct weapon_test {
  bool epeeOn = false;
  long tEpeeTrigger = 0;
  long tEpeeInterOn = 0;
  bool foilOn = false;
  long tFoilTrigger = 0;
  long tFoilInterOn = 0;
  bool lineAC = false;
  byte epeeInterruptBit = PCINT4;
  byte foilInterruptBit = PCINT6;
  long tLightChange = 1000; //ms -- time for the intermittent LED to be on
  byte update_flag = false;
  float ohm_Foil = 0;
  float ohm_Epee = 0;
  float ohm_FoilMax = 0;
  float ohm_EpeeMax = 0;
  long tLastConnect = 0;
  bool cableDC = false;
};

struct buttonEvent {
  byte pin_vec;
  long eventTime;
};

testbox_line bananaA;
testbox_line bananaB;
testbox_line bananaC;

CableData cableState;

volatile weapon_test weaponState;
volatile char newBoxState;

void setup() {
  // put your setup code here, to run once:

  //Activate display
  delay(250); //Hold for half second to power on
  CreateDisplay();

  bitSet(DDRD, POWER_CONTROL); //Set PORTD2 for output
  bitSet(PORTD, POWER_CONTROL); //Set power to high (keep the box on)!

  bitSet(DDRB, DDB0); //Set the SS pin as an output to avoid screw-ups
  bitSet(PORTB, PORTB0);  //Set to HIGH to turn off LED

  //Set PINF as all inputs and disable any pull-ups
  DDRF = 0;
  PORTF = 0;

  //Enable diagnostic light
  //bitSet(DDRB,RX_LED);
  //bitClear(PORTB,RX_LED);  //RX Light on

  // Enable timing diagnostics
  bitSet(DDRD, DIAG_PORT);
  bitClear(PORTD, DIAG_PORT);

  bitSet(MCUCR, PUD); //Disable all internal pull-ups (We don't need them, everything is externally low)

  // Initialize the MUX
  bitSet(DDRD, MUX_LATCH); //Set output on MUX_LATCH
  bitClear(PORTD, MUX_LATCH); //equivalent to digitalWrite(4, LOW); Toggle the SPI
  SPI.begin();
  bitSet(SPCR, MSTR);  //Set Master mode
  SPI.beginTransaction(MUX_SPI_SETTINGS);
  SPI.transfer(B00000000); //Set MUX to low
  SPI.endTransaction();
  bitSet(PORTD, MUX_LATCH); //equivalent to digitalWrite(4,HIGH);

  //Enable button interrupts
  bitClear(DDRE, DDE6); //Set PINE6 as input
  bitClear(PORTE, PORTE6); //Disable Pull-up resistor
  //EICRB |= (1 << ISC60) | (0 << ISC61); //set up INT6 for falling edge
  //EIMSK |= (1 << INT6);                     //enable INT6
  //sei();  //Enable ALL interrupts

  //delay(2000);
  Serial.begin(115200);
  if (Serial) {
    //Serial.write(F("Starting initialization..."));
    //Serial.println(eepromLocationStep);
  }

  analogReference(INTERNAL);

  InitializeChannels();
  InitializeLEDs();
  InitializeCableLEDs();
  InitializeADC();

  //CheckBatteryStatus();
  //displayBatteryStatus();
  //delay(1500);

  setBoxMode('c');
}

//Used for ADC processing.
ISR(ADC_vect)
{
  //static long t1 = micros();
  byte lowbits=(ADCL>>6); //right shift the lowest bits
  byte x = ADCH;  // read 8 bit value from ADC (left adjusted)
  static bool isStable = false;
  static byte stableCount = 0;
  long t_now = millis();
  long tempVal=0;

  stableCount++;
  if (stableCount < ADC_STABLE_CYCLES) {
    ADCSRA |= (1 << ADSC);
    return;
  } else {
    isStable = true;
  }

  if (ActiveCh->ch_label[0] == 'A') {
    numSamples++;
    //Serial.print(x,BIN);Serial.print(" ");Serial.println(lowbits,BIN);
  }

  //numSamples++;

  //Simulated ADC output values for testing.
  // Comment this switch statement out for real usage.

  /*
    switch (ActiveCh->ch_label) {
    case 'A':
      x = 50;
      break;
    case 'B':
      x = micros() & (B01001111); //Random number 0-8
      break;
    case 'C':
      //x=micros() & (B00001110); //
      x = millis() / 100;
      break;
    case '1':
      x = 250;
      break;
    case '2':
      x = 250;
      break;
    case '3':
      x = 255;
      break;
    }*/

  //ActiveCh->sampleArray[ActiveCh->sampleIndex] = x;
  ActiveCh->lastADCH=x;
  ActiveCh->lastADCL=lowbits;
  ActiveCh->highSum+=x;
  ActiveCh->lowSum+=lowbits;
  ActiveCh->sampleIndex++;
  if (ActiveCh->sampleIndex == ActiveCh->NUM_AVE) {
    ActiveCh->sampleIndex = 0; //Reset the index counter
    tempVal=(ActiveCh->highSum<<2)+(ActiveCh->lowSum);
    ActiveCh->averageVal=(tempVal>>(ActiveCh->NUM_AVE_POW2));
    ActiveCh->highSum=0;
    ActiveCh->lowSum=0;
  }

  if (x > ActiveCh->decay_max) {
    ActiveCh->maxval = x;
    ActiveCh->decay_max = x;
    ActiveCh->t_max = t_now;
  }

  if (x < ActiveCh->decay_min) {
    ActiveCh->minval = x;
    ActiveCh->decay_min = x;
    ActiveCh->t_min = t_now;
  }
  /*
    if (x > maxADCthreshold && ActiveCh->isLowRange()) {
    ActiveCh->setRangeHigh();
    ActiveCh->setADCChannelActive();
    isStable = false;
    }
    if (x < minADCthreshold && ActiveCh->isHighRange()) {
    ActiveCh->setRangeLow();
    ActiveCh->setADCChannelActive();
    isStable = false;
    }*/

  //Load the next channel
  if (ActiveCh->ADC_Scan && isStable) {
    ActiveCh = ActiveCh->nextChannel;

    // Switch the MUX to toggle the MOSFETs
    bitClear(PORTD, MUX_LATCH); //equivalent to digitalWrite(4, LOW); Toggle the SPI
    SPI.beginTransaction(MUX_SPI_SETTINGS);
    SPI.transfer(ActiveCh->muxSetting);
    SPI.endTransaction();
    bitSet(PORTD, MUX_LATCH);  //equivalent to digitalWrite(4,HIGH);

    ActiveCh->setADCChannelActive();
    isStable = false;
    stableCount = 0;
  }

  // Code block for stability
  //delayMicroseconds(100);
  //isStable=true;

  // Start the next conversion
  ADCSRA |= (1 << ADSC);
  //timing_seg=micros()-tic;
}

// Pin change interrupt handler.  Used for weapon test mode.
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
}

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

  checkButtonState(); //Handle button pushes for mode states

  if (t_now - t_LCD_upd > tLCDRefresh) {
    force_update = true;
  }
  if (t_now - t_LED_upd > tLEDRefresh) {
    force_update = true;
  }

  switch (BoxState) {
    case 'c':
      if (force_update) {
        for (int k = 0; k < NUM_ADC_SCAN_CHANNELS; k++) {
          ChanArray[k].updateVals();
        }
        updateCableState();
        force_update = false;
      }
      break;
    case 'r':
      if (force_update) {
        //FoilADC.updateVals();
        //EpeeADC.updateVals();
      }
      //updateWeaponResistance();
      break;
    case 'w':
      updateWeaponState();
      break;
    case 's':
      break;
  }

  /*
    if (t_now - t_LED_reset > tLEDResync) {
    for (int k=0; k<NUM_LEDS_BLOCK1; k++) {
      LED_block1[k]=CRGB::Black;
    }
    FastLED.show();
    t_LED_reset=millis();
    }*/

  //Automatic power off while idle
  if ((t_now - tLastActive) > powerOffTimeOut) {
    setPowerOff();
  }

  if (t_now - t_LED_upd > tLEDRefresh) {
    updateLEDDisplay(BoxState);
    FastLED.show();
    t_LED_upd = millis();
  }
  
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
  }

  if (((t_now - t_Battery_Check) > tBatteryInterval) && ((t_now - tLastActive) > tBatteryInterval)) {
    CheckBatteryStatus();
    t_Battery_Check = millis();
    if ((batteryVoltage < 3.5) && (batteryVoltage > 2.0)) {
      lcd.setCursor(0, 0);
      lcd.print(F("  Low Battery  "));
      t_LCD_upd = millis() + 1000;
    }
  }

  if (t_now - t_Serial_upd > tSerialRefresh) {
    if (Serial) {
      if (Serial.availableForWrite() == 64) { //Only write if the buffer is empty
        writeSerialOutput(BoxState);
        /*
          float dt = (float(millis() - t_Serial_upd) * 1.0e-3);
          EffSampleRate = float(numSamples) / dt;
          Serial.print(F("Eff. Sample Rate = ")); Serial.print(EffSampleRate);
          Serial.print(F(" Hz  Num samples = ")); Serial.print(numSamples); Serial.print(F(" Time = "));
          Serial.println(dt);*/
      }
    }
    t_Serial_upd = millis();
  }

  if (t_now - t_LCD_upd > tLCDRefresh) {
    bitWrite(PORTB, PORTB0, !bitRead(PORTB, PORTB0)); //Blink the LCD to indicate still alive
    updateLCDDisplay(BoxState);
    //Serial.println(lcdString1);
    //Serial.println(lcdString2);
    t_LCD_upd = millis();
  }
}
