
#define ARM_ARCH_7EM //Target Cortex-M4F
#define ARM_MATH_CM4 //Required for arm_math library
#define __FPU_PRESENT 1

#define TTArmBoardRev 1 

#include <Arduino.h>
#include "oledDisplaySettings.h"

#define NUM_ADC_SCAN_CHANNELS 9 //9 combinations 
#define OHM_FIELD_WIDTH 4 //Width of the text display for ohms
#define SERIAL_OUTPUT_BUFFER_SIZE 256 //Length of serial buffer
#define ADC_BUFFER_SIZE 8
#define OPEN_CIRCUIT 999.9

const uint32_t MUX_HOLD_TIME=10000; //ms

//Pin defintions, as needed
const byte POWER_CONTROL = 11;
const byte DIAG_PIN = 25;
const byte BUTTON_PIN = 27;
const byte LED1_PIN = 17;
const byte LED2_PIN = 19;
//const byte FASTLED_PIN

const SPISettings OLED_SPI_SETTINGS(2000000, MSBFIRST, SPI_MODE0);
const uint8_t MUX_LATCH = 16;
const uint8_t MUX_CLK = 15;
const uint8_t MUX_DATA = 7;

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

Adafruit_SSD1351 tft = Adafruit_SSD1351(SCREEN_WIDTH, SCREEN_HEIGHT, &SPI, CS_PIN, DC_PIN);
char outputString[SERIAL_OUTPUT_BUFFER_SIZE];
char displayBuf[SERIAL_OUTPUT_BUFFER_SIZE];

#define MUX_COUNT (12)
char muxLabels[MUX_COUNT][4]={"--","AA","AB","AC","BA","BB","BC","CA","CB","CC","Ep","Fl"};
const byte muxList[MUX_COUNT]={MUX_DISABLED,MUX_CABLE_AA,MUX_CABLE_AB,MUX_CABLE_AC,MUX_CABLE_BA,MUX_CABLE_BB,MUX_CABLE_BC,MUX_CABLE_CA,MUX_CABLE_CB,MUX_CABLE_CC,MUX_WEAPON_AB,MUX_WEAPON_CB};

#define ADC_COUNT (6)
const uint8_t adcList[ADC_COUNT]={PIN_A0,PIN_A1,PIN_A2,PIN_A3,PIN_A4,PIN_A5};
const char adcName[ADC_COUNT][8]={"Alow","Blow","Clow","Ahi","Bhi","Chi"};
uint32_t adcVal[ADC_COUNT];

const uint8_t AVE_COUNT=3; //2^3 averages

void setup() {
  // put your setup code here, to run once:

  initScreen();  
  pinMode(POWER_CONTROL, OUTPUT);
  digitalWrite(POWER_CONTROL, HIGH);
  Serial.begin(115200);
  delay(100);
  uint16_t val=selfTest();
  if (val) {
    Serial.print("Self-test failed!"); Serial.print(val,BIN);
  }
  pinMode(PIN_A0, INPUT_PULLDOWN);
  pinMode(PIN_A1, INPUT_PULLDOWN);
  pinMode(PIN_A2, INPUT_PULLDOWN);
  pinMode(PIN_A3, INPUT_PULLDOWN);
  pinMode(PIN_A4, INPUT_PULLDOWN);  
  pinMode(PIN_A5, INPUT_PULLDOWN);  

  initMUX();
  initADC();  
}

void initMUX() {
  pinMode(MUX_LATCH, OUTPUT);
  pinMode(MUX_CLK, OUTPUT);
  pinMode(MUX_DATA, OUTPUT);

  pinMode(PIN_LED1, OUTPUT);
  pinMode(PIN_LED2, OUTPUT);
  
  digitalWrite(MUX_LATCH, LOW);
  digitalWrite(MUX_CLK, LOW);
  digitalWrite(MUX_DATA, LOW);
  setMUX(0);
}

void initScreen() {
  char buf[12];
  uint8_t yOffset=15;
  uint8_t txtSpace=16;
  
  tft.begin();
  tft.setRotation(3);  //3 sets the display top to be aligned with the Feather uUSB.
  tft.fillRect(0, 0, 128, 128, BLACK);
  tft.setCursor(0, 0);
  tft.println("Debug mode");  
}

void setMUX(uint8_t num){
  if (num>=MUX_COUNT) {num=0;}
  digitalWrite(MUX_LATCH, LOW); //equivalent to digitalWrite(4, LOW); Toggle the SPI
  shiftOut(MUX_DATA, MUX_CLK, MSBFIRST, muxList[num]);
  digitalWrite(MUX_LATCH, HIGH);  
}

void initADC() {
  analogCalibrateOffset();
  analogReference(AR_INTERNAL_2_4);
  analogReadResolution(12);
  analogOversampling(32);
  analogSampleTime(20);
}

uint16_t selfTest() {
  static const int numPins=6;
  byte pinList[6]={PIN_A0,PIN_A1,PIN_A2,PIN_A3,PIN_A4,PIN_A5};  
  uint16_t adcResult=0;
  uint16_t res=0;

  analogCalibrateOffset();
  analogReference(AR_INTERNAL_2_4);
  analogReadResolution(12);
  analogOversampling(32);
  analogSampleTime(20);
  
  for (int k=0;k<numPins; k++) {
    pinMode(pinList[k], INPUT_PULLUP);
    delay(20);    
    res=analogRead(pinList[k]);
    if (res<=4090) {
      adcResult= adcResult & (0x1 << k);
    }    
    pinMode(pinList[k], INPUT_PULLDOWN);
    delay(20);    
    res=analogRead(pinList[k]);
    if (res>=10) {
      adcResult = adcResult & (0x1 << (k+8));
    }
    pinMode(pinList[k], INPUT);
  }
  return adcResult;
}


void loop() {
  static uint8_t muxSetting=0;
  static long tUpdate=-1*MUX_HOLD_TIME;
  static long tDisplay=-1*MUX_HOLD_TIME;
  long tic, toc;

  if ((millis()-tUpdate)>MUX_HOLD_TIME) {
    muxSetting=(muxSetting+1) % MUX_COUNT;//wrap the mux setting
    %muxSetting = 1;
    
    setMUX(muxSetting);
    delay(20);
    tft.setCursor(100,0);
    tft.setTextColor(GREEN,BLACK);
    tft.setTextSize(2);
    tft.print(muxLabels[muxSetting]);
    tUpdate=millis();
  }
  digitalWrite(PIN_LED2,HIGH);
  tic=micros();
  for (int k=0; k<ADC_COUNT; k++) {
    adcVal[k]=analogRead(adcList[k]);
  }
  toc=micros();
  digitalWrite(PIN_LED2,LOW);
  if ((millis()-tDisplay)<50) {
    return;  //Skip updating the display
  }
  Serial.print(millis());Serial.print(" , ");
  Serial.print(muxLabels[muxSetting]);Serial.print(" , ");
  tft.setTextSize(2);  
  uint8_t yOffset=20;
  uint8_t txtSpace=18;
  for (int k=0; k<ADC_COUNT; k++) {
    tft.setTextSize(2);
    tft.setCursor(0, yOffset+k*txtSpace);
    tft.setTextColor(WHITE,BLACK);
    snprintf(displayBuf,16,"%s=",adcName[k]);
    Serial.print(displayBuf); Serial.print(",\t");
    tft.print(displayBuf);
    if (adcVal[k]>200) {
      tft.setTextColor(RED,BLACK);
    } else {
      tft.setTextColor(GREEN,BLACK);
    }
    snprintf(displayBuf,16,"%4u",adcVal[k]);
    Serial.print(displayBuf); Serial.print(",\t");

    tft.println(displayBuf);
    tft.setTextSize(1);
    tft.setTextColor(WHITE,BLACK);
    tft.setCursor(100, 120);
    snprintf(displayBuf,16,"%u",toc-tic);
    tft.println(displayBuf);
    tDisplay=millis();
  }
  Serial.println();  
}
