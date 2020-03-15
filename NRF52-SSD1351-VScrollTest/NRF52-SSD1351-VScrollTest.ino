// Screen dimensions
#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT 128 // Change this to 96 for 1.27" OLED.

// You can use any (4 or) 5 pins
#define SCLK_PIN 12
#define MOSI_PIN 13
#define DC_PIN   20
#define CS_PIN   31
//#define RST_PIN  20


// Color definitions
#define  BLACK           0x0000
#define BLUE            0x001F
#define RED             0xF800
#define GREEN           0x07E0
#define CYAN            0x07FF
#define MAGENTA         0xF81F
#define YELLOW          0xFFE0  
#define WHITE           0xFFFF

#include <Arduino.h>
//#include <Serial.h>
#include <Adafruit_GFX.h>
#include "Adafruit_SSD1351.h"
#include <SPI.h>

// Option 1: use any pins but a little slower
Adafruit_SSD1351 tft = Adafruit_SSD1351(SCREEN_WIDTH, SCREEN_HEIGHT, CS_PIN, DC_PIN, MOSI_PIN, SCLK_PIN);
//Adafruit_SSD1351 tft_bottom = Adafruit_SSD1351(SCREEN_WIDTH, 128-40, CS_PIN, DC_PIN, MOSI_PIN, SCLK_PIN);  

// Option 2: must use the hardware SPI pins
// (for UNO thats sclk = 13 and sid = 11) and pin 10 must be
// an output. This is much faster - also required if you want
// to use the microSD card (see the image drawing example)
//Adafruit_SSD1351 tft = Adafruit_SSD1351(SCREEN_WIDTH, SCREEN_HEIGHT, &SPI, CS_PIN, DC_PIN);

void setup() {
  tft.begin();
  
  
  tft.setRotation(3);
  //tft.setAddrWindow(0,0,127,127);
  //tft.setDisplayStartLine(0x0);
  
  tft.setHorizontalScroll(false, 0xff, 0,128,0x01);
    

  tft.enableDisplay(false);
  tft.enableDisplay(true);
  tft.fillRect(0, 0, 128, 128, BLACK);
  //tft.drawLine(0,0,127, 127, WHITE);

  //tft.fillRect(0,0,127,127,BLACK);
  tft.setTextSize(2);
  tft.setTextColor(WHITE,BLACK);
  tft.setTextWrap(false);
  tft.setCursor(0,0);
  tft.print(" Static");

  tft.setCursor(0,85);
  tft.print("Scrolling");
  //tft.setHorizontalScroll(true, 0xff, 25,103,B01);

  //tft.setDisplayOffset(40);
  tft.setMuxRatio(127);
  

  //Serial.begin();
  //Serial.println("Display running");
  
}


void loop() {
  struct lineBuffer {
    uint8_t pix_start=0;
    uint8_t lineLength=0;
    uint16_t color=BLACK;
  };
  const int BUF_SIZE=100;
  static int lastVal=50;
  int newVal=50;
  unsigned long tic,toc;
  static uint8_t dispRow,zeroRow;
  

  static lineBuffer circBuffer[BUF_SIZE];
  static int bufIndx=0;
  
  // put your main code here, to run repeatedly:

  delay(100);

  tic=micros();
  tft.setCursor(0,0);

  tft.drawFastVLine(zeroRow,50,128-50,BLACK);
  
  newVal=lastVal+random(-10,10);
  if (newVal<50) {
    newVal=60;
  }
  if (newVal>127) {
    newVal=117;
  }
  dispRow=(zeroRow+100) & 0x7F; //127 for mod operator

  if (newVal>lastVal) {
    tft.drawFastVLine(dispRow,lastVal,newVal-lastVal,RED);  
  } else {
    tft.drawFastVLine(dispRow,lastVal,lastVal-newVal,GREEN); 
    if (lastVal==newVal) {
      tft.drawFastVLine(dispRow,lastVal,1,BLUE); 
    }
  }
  tft.fillRect((dispRow+1) & 0x7F,lastVal-2,5,5,BLACK);
  tft.fillRect((dispRow+2) & 0x7F,newVal-2,5,5,CYAN);

  lastVal=newVal;
  zeroRow=(zeroRow+1) & (B01111111);

  tft.setCursor(zeroRow+1,0);
  tft.print(" Static");
  tft.setDisplayStartLine(zeroRow);

  toc=micros();
  //tft.setCursor(zeroRow+75,0);
  //tft.print(dispRow);
}
