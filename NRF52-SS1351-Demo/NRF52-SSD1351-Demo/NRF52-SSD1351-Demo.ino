// Screen dimensions
#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT 128 // Change this to 96 for 1.27" OLED.

#if defined(ARDUINO_NRF52840_FEATHER)
  //nrf52840 definition
  #define SCLK_PIN (26)
  #define MOSI_PIN (25)
  #define DC_PIN   (2)
  //#define DC_PIN   (5)
  #define CS_PIN  (9)
  #define RST_PIN  (20)  
#else
  const byte LED1_PIN = 17;
  const byte LED2_PIN = 19;
  #define LED_DATA_PIN 25 //LEDs are on SDA/pin25

  //SPI pin definitions
  #define SCLK_PIN 12
  #define MOSI_PIN 13
  #define DC_PIN   20
  #define CS_PIN   31
  //#define RST_PIN  20
#endif




// Color definitions
#define  BLACK           0x0000
#define BLUE            0x001F
#define RED             0xF800
#define GREEN           0x07E0
#define CYAN            0x07FF
#define MAGENTA         0xF81F
#define YELLOW          0xFFE0  
#define WHITE           0xFFFF

//#include <nrf52840.h>
#include <Adafruit_SSD1351.h>
#include <Adafruit_GFX.h>
#include <SPI.h>
//#include "TTArmGraphic.c"
#include "splashScreenImage.c"
#include <Fonts/FreeSansBold24pt7b.h>

//Create class to allow using buffer with fixed stack memory
class GFXcanvas16stack : protected GFXcanvas16 {
public:
  // Constructor
  GFXcanvas16stack(uint16_t* memPointer) :  GFXcanvas16(1,1) {
    if (buffer) {
      free(buffer);  //Free the automatically created buffer
    }
    buffer=memPointer; //Reset the pointer.
  }
  ~GFXcanvas16stack() {};  //Do not try to free stack memory

private:
  // Private members specific to this subclass
};

//#include <nrf_nvmc.h>

//#ifdef _VARIANT_FEATHER52840_  
//#include <nrfx/mdk/nrf52.h>
//#include <nrf52840.h>
//uint32_t *NFCPINS=&(0x0000020C);
//#endif

// Option 1: use any pins but a little slower
//Adafruit_SSD1351 tft = Adafruit_SSD1351(SCREEN_WIDTH, SCREEN_HEIGHT, CS_PIN, DC_PIN, MOSI_PIN, SCLK_PIN);  

// Option 2: must use the hardware SPI pins
// (for UNO thats sclk = 13 and sid = 11) and pin 10 must be
// an output. This is much faster - also required if you want
// to use the microSD card (see the image drawing example)
Adafruit_SSD1351 tft = Adafruit_SSD1351(SCREEN_WIDTH, SCREEN_HEIGHT, &SPI, CS_PIN, DC_PIN);
//SPIClass oledSPI = SPIClass(NRF_SPIM2, PIN_SPI_MISO, SCLK_PIN, MOSI_PIN);
//Adafruit_SSD1351 tft = Adafruit_SSD1351(SCREEN_WIDTH, SCREEN_HEIGHT, &oledSPI, CS_PIN, DC_PIN);

// Pre-allocate buffer memory in global address space
//uint16_t gfxBuffer[SCREEN_WIDTH][SCREEN_HEIGHT];
//GFXcanvas16stack screenBuffer(&(gfxBuffer[0][0]));
const int GFX_BUFFER_HEIGHT=80;
GFXcanvas16 screenBuffer(SCREEN_WIDTH,GFX_BUFFER_HEIGHT);

void setup() {
  Serial.begin(115200);
  while ( !Serial ) {
    if (millis()>5000) {
      break;
    }
    delay(10);   // for nrf52840 with native usb
  }

  //pinMode(RST_PIN,OUTPUT);
  //digitalWrite(RST_PIN,LOW);
  //delay(200);
  //digitalWrite(RST_PIN,HIGH);

  pinMode(PIN_LED2,OUTPUT);
  digitalWrite(PIN_LED2,(NRF_UICR->NFCPINS & 0x0001));
  /*if (NRF_UICR->NFCPINS & 0x0001) {
    digitalWrite(PIN_LED1,HIGH);
    NRF_NVMC->CONFIG = 0x0001;
    while (!NRF_NVMC->READYNEXT) delay(50);
    NRF_UICR->NFCPINS = 0x0000;
    while (!NRF_NVMC->READYNEXT) delay(50);
    NRF_NVMC->CONFIG = 0x0000;
  }*/
  digitalWrite(PIN_LED1,LOW);
  digitalWrite(PIN_LED2,(NRF_UICR->NFCPINS & 0x0001));
  
  tft.begin();
  pinMode(DC_PIN,OUTPUT);
  digitalWrite(DC_PIN,HIGH);
  tft.fillRect(0, 0, 128, 128, BLACK);
  tft.drawLine(0,0,127, 127, WHITE);

  displaySplashScreen();
  delay(500);

  Serial.println(F("Benchmark                Time (microseconds)"));
  delay(10);
  Serial.print(F("Screen fill              "));
  Serial.println(testFillScreen());
  delay(500);

  Serial.print(F("Text                     "));
  Serial.println(testText());
  delay(3000);

  Serial.print(F("Lines                    "));
  Serial.println(testLines(CYAN));
  delay(500);

  Serial.print(F("Horiz/Vert Lines         "));
  Serial.println(testFastLines(RED, BLUE));
  delay(500);

  Serial.print(F("Rectangles (outline)     "));
  Serial.println(testRects(GREEN));
  delay(500);

  Serial.print(F("Rectangles (filled)      "));
  Serial.println(testFilledRects(YELLOW, MAGENTA));
  delay(500);

  Serial.print(F("Circles (filled)         "));
  Serial.println(testFilledCircles(10, MAGENTA));

  Serial.print(F("Circles (outline)        "));
  Serial.println(testCircles(10, WHITE));
  delay(500);

  Serial.print(F("Triangles (outline)      "));
  Serial.println(testTriangles());
  delay(500);

  Serial.print(F("Triangles (filled)       "));
  Serial.println(testFilledTriangles());
  delay(500);

  Serial.print(F("Rounded rects (outline)  "));
  Serial.println(testRoundRects());
  delay(500);

  Serial.print(F("Rounded rects (filled)   "));
  Serial.println(testFilledRoundRects());
  delay(500);

  testGfxBuffer();
  delay(1000);

  Serial.println(F("Done!"));

  
}

void displayRGB565Bitmap(uint16_t x, uint16_t y, uint16_t *pixels, uint16_t w, uint16_t h) {
  tft.startWrite();

  //  hWindow=(h>hMax) ? h : hMax
  //for (uint16_t k=0; k<h; k++) { 
  tft.setAddrWindow(x, y, w, h);
    //tft.setAddrWindow(x,y,w,32);
  tft.writePixels(pixels, w * h);

  tft.dmaWait();
  // Reset the address window to full screen;
  //tft.setAddrWindow(0,0,SCREEN_WIDTH,SCREEN_HEIGHT);
  tft.endWrite();
}

void displaySplashScreen() {
  int16_t xi, yj;
  uint16_t *pixelColor;  
  //tft.startWrite();
  //tft.setAddrWindow(0, 0, 16, 16);
  //tft.writePixels(&(pixels[k*w]), w * 1);
  //lameTxtCanvas.setTextSize(3);
  //lameTxtCanvas.setTextColor(colorList.cBLUE);
  //lameTxtCanvas.setCursor(0, LAME_DIGIT_HEIGHT - 3);
  //lameTxtCanvas.print("---");
  //displayRGB565Bitmap(0, 27, lameTxtCanvas.getBuffer(), 128, LAME_DIGIT_HEIGHT);
  //delay(2000);

  //tft.fillRect(0, 0, 128, 128, BLACK);
  //tft.fillRect(0, 0, 128, 128, colorList.cMAGENTA);
  //tft.dmaWait();
  tft.fillRect(0, 0, 128, 128, BLACK);
  //tft.fillRect(0, 0, 128, 128, colorList.cMAGENTA);
  tft.dmaWait();
  // Need to raster the buffer due to 832 memory limitations
  int drawRows=GFX_BUFFER_HEIGHT;
  for (int16_t k=0; k<splashImage.height; k+=GFX_BUFFER_HEIGHT) {
    if ((k+GFX_BUFFER_HEIGHT)>SCREEN_HEIGHT) {
      drawRows=(k+GFX_BUFFER_HEIGHT)-SCREEN_HEIGHT;      
    }
    screenBuffer.drawRGBBitmap(0,0,(uint16_t *)&(splashImage.pixel_data[2*k*splashImage.width]),splashImage.width, drawRows);
    //screenBuffer.drawRGBBitmap(0,0,(uint16_t *)&(splashImage.pixel_data[0]),splashImage.width, 64);
    displayRGB565Bitmap(0, k, screenBuffer.getBuffer(), splashImage.width, drawRows);
  }


/*
#if defined(ARDUINO_NRF52840_FEATHER)  
  //displayRGB565Bitmap(0, 30, (uint16_t *)&(splashImage.pixel_data[0]), splashImage.width, splashImage.height);
  screenBuffer.drawRGBBitmap(0,0,(uint16_t *)&(splashImage.pixel_data[0]),splashImage.width, splashImage.height);
  displayRGB565Bitmap(0, 0, screenBuffer.getBuffer(), splashImage.width, splashImage.height);
  //displayRGB565Bitmap(0, 30, (uint16_t *)&(splashImage.pixel_data[0]), splashImage.width, splashImage.height);
#else
  //tft.drawRGBBitmap(0, 0, (uint16_t*) & (splashImage.pixel_data[0]), splashImage.width, splashImage.height);
  displayRGB565Bitmap(0, 0, (uint16_t *)&(splashImage.pixel_data[0]), splashImage.width, splashImage.height);
#endif
*/
}

void loop() {
  // put your main code here, to run repeatedly:
  long drawTime=0;
  char buf[64];
  float tDraw=0;
  uint8_t tempSize=32;
  uint16_t red,green,blue;
  uint16_t indx=0;
  //uint16_t pixelData[2*32*32+1];

  tft.setRotation(3);
  tft.fillRect(0, 0, 128, 128, BLACK);
  delay(5);
  Serial.println("Drawing splash screen");
  /*for (int j=0; j<tempSize; j++) {    
    for (int k=0; k<tempSize; k++) {
      indx=2*(tempSize*j+k);
      pixelData[tempSize*j+k]=*( (uint16_t *) & (splashImage.pixel_data[2*(tempSize*j+k)]));
    }    
  }  */
  long tic=micros();
  //delay(5000);
  //tft.fillRect(0, 0, 128, 128, BLACK);  
  //tft.drawRGBBitmap(0,0, (uint16_t *) pixelData, tempSize,tempSize);
  //displayRGB565Bitmap(0, 0, (uint16_t *) splashImage.pixel_data, splashImage.width, splashImage.height);
  displaySplashScreen();
  //tft.drawRGBBitmap(0, 0, (uint16_t*) & (splashImage.pixel_data[0]), splashImage.width, splashImage.height);
  //Serial.print("Drawing image that is : ");Serial.print(splashImage.width);Serial.print("x");Serial.println(splashImage.height);
  //tft.drawRGBBitmap(0, 0, (uint16_t *) splashImage.pixel_data, splashImage.width, splashImage.height);
  //tft.dmaWait();
  //tft.drawRGBBitmap(0, 0, (uint16_t*) &(splashImage.pixel_data[32*128]), 64, 64);
  long toc=micros();
  tDraw=(toc-tic)/1e3; //Convert to millis
  tft.setCursor(0, 120);
  tft.setTextColor(WHITE);  tft.setTextSize(1);
  snprintf(buf, 64, "%d",(toc-tic) );
  tft.println(buf);
  Serial.println(buf);
  delay(5000);
  
  for(uint8_t rotation=0; rotation<4; rotation++) {
    tft.setRotation(rotation);
    testText();
    delay(1000);
  }
  
}

unsigned long testGfxBuffer() {
  float frameRate = 0;
  int frameRateInt = 0;

  unsigned long tStart = micros();
  unsigned long tNow = micros();
  screenBuffer.setFont(&FreeSansBold24pt7b);
  screenBuffer.setTextSize(2);
  for (int loopNum=1; loopNum<500; loopNum++) {
    tNow = micros();
    frameRate=float(loopNum)/float((tNow-tStart)*1e-6);
    frameRateInt=frameRate;
    screenBuffer.fillRect(0,0,SCREEN_WIDTH,SCREEN_HEIGHT,BLACK);
    screenBuffer.fillRect(0,0,16,16,CYAN);
    screenBuffer.fillRect(127-16,0,16,16,GREEN);
    screenBuffer.fillRect(0,127-16,16,16,MAGENTA);
    screenBuffer.fillRect(127-16,127-16,16,16,WHITE);
    //screenBuffer.setTextSize(6);
    screenBuffer.setTextColor(GREEN);
    screenBuffer.setCursor(10, 100);
    frameRateInt=frameRate;
    screenBuffer.print(frameRateInt);
    displayRGB565Bitmap(0, 0, screenBuffer.getBuffer(), 128, 128);
    }
  unsigned long tStop = micros();
  return (tStop-tStart);
}


unsigned long testFillScreen() {
  unsigned long start = micros();
  tft.fillScreen(BLACK);
  yield();
  tft.fillScreen(RED);
  yield();
  tft.fillScreen(GREEN);
  yield();
  tft.fillScreen(BLUE);
  yield();
  tft.fillScreen(BLACK);
  yield();
  return micros() - start;
}

unsigned long testText() {
  tft.fillScreen(BLACK);
  unsigned long start = micros();
  tft.setCursor(0, 0);
  tft.setTextColor(WHITE);  tft.setTextSize(1);
  tft.println("Hello World!");
  tft.setTextColor(YELLOW); tft.setTextSize(2);
  tft.println(1234.56);
  tft.setTextColor(RED);    tft.setTextSize(3);
  tft.println(0xDEADBEEF, HEX);
  tft.println();
  tft.setTextColor(GREEN);
  tft.setTextSize(5);
  tft.println("Groop");
  tft.setTextSize(2);
  tft.println("I implore thee,");
  tft.setTextSize(1);
  tft.println("my foonting turlingdromes.");
  tft.println("And hooptiously drangle me");
  tft.println("with crinkly bindlewurdles,");
  tft.println("Or I will rend thee");
  tft.println("in the gobberwarts");
  tft.println("with my blurglecruncheon,");
  tft.println("see if I don't!");
  return micros() - start;
}

unsigned long testLines(uint16_t color) {
  unsigned long start, t;
  int           x1, y1, x2, y2,
                w = tft.width(),
                h = tft.height();

  tft.fillScreen(BLACK);
  yield();
  
  x1 = y1 = 0;
  y2    = h - 1;
  start = micros();
  for(x2=0; x2<w; x2+=6) tft.drawLine(x1, y1, x2, y2, color);
  x2    = w - 1;
  for(y2=0; y2<h; y2+=6) tft.drawLine(x1, y1, x2, y2, color);
  t     = micros() - start; // fillScreen doesn't count against timing

  yield();
  tft.fillScreen(BLACK);
  yield();

  x1    = w - 1;
  y1    = 0;
  y2    = h - 1;
  start = micros();
  for(x2=0; x2<w; x2+=6) tft.drawLine(x1, y1, x2, y2, color);
  x2    = 0;
  for(y2=0; y2<h; y2+=6) tft.drawLine(x1, y1, x2, y2, color);
  t    += micros() - start;

  yield();
  tft.fillScreen(BLACK);
  yield();

  x1    = 0;
  y1    = h - 1;
  y2    = 0;
  start = micros();
  for(x2=0; x2<w; x2+=6) tft.drawLine(x1, y1, x2, y2, color);
  x2    = w - 1;
  for(y2=0; y2<h; y2+=6) tft.drawLine(x1, y1, x2, y2, color);
  t    += micros() - start;

  yield();
  tft.fillScreen(BLACK);
  yield();

  x1    = w - 1;
  y1    = h - 1;
  y2    = 0;
  start = micros();
  for(x2=0; x2<w; x2+=6) tft.drawLine(x1, y1, x2, y2, color);
  x2    = 0;
  for(y2=0; y2<h; y2+=6) tft.drawLine(x1, y1, x2, y2, color);

  yield();
  return micros() - start;
}

unsigned long testFastLines(uint16_t color1, uint16_t color2) {
  unsigned long start;
  int           x, y, w = tft.width(), h = tft.height();

  tft.fillScreen(BLACK);
  start = micros();
  for(y=0; y<h; y+=5) tft.drawFastHLine(0, y, w, color1);
  for(x=0; x<w; x+=5) tft.drawFastVLine(x, 0, h, color2);

  return micros() - start;
}

unsigned long testRects(uint16_t color) {
  unsigned long start;
  int           n, i, i2,
                cx = tft.width()  / 2,
                cy = tft.height() / 2;

  tft.fillScreen(BLACK);
  n     = min(tft.width(), tft.height());
  start = micros();
  for(i=2; i<n; i+=6) {
    i2 = i / 2;
    tft.drawRect(cx-i2, cy-i2, i, i, color);
  }

  return micros() - start;
}

unsigned long testFilledRects(uint16_t color1, uint16_t color2) {
  unsigned long start, t = 0;
  int           n, i, i2,
                cx = tft.width()  / 2 - 1,
                cy = tft.height() / 2 - 1;

  tft.fillScreen(BLACK);
  n = min(tft.width(), tft.height());
  for(i=n; i>0; i-=6) {
    i2    = i / 2;
    start = micros();
    tft.fillRect(cx-i2, cy-i2, i, i, color1);
    t    += micros() - start;
    // Outlines are not included in timing results
    tft.drawRect(cx-i2, cy-i2, i, i, color2);
    yield();
  }

  return t;
}

unsigned long testFilledCircles(uint8_t radius, uint16_t color) {
  unsigned long start;
  int x, y, w = tft.width(), h = tft.height(), r2 = radius * 2;

  tft.fillScreen(BLACK);
  start = micros();
  for(x=radius; x<w; x+=r2) {
    for(y=radius; y<h; y+=r2) {
      tft.fillCircle(x, y, radius, color);
    }
  }

  return micros() - start;
}

unsigned long testCircles(uint8_t radius, uint16_t color) {
  unsigned long start;
  int           x, y, r2 = radius * 2,
                w = tft.width()  + radius,
                h = tft.height() + radius;

  // Screen is not cleared for this one -- this is
  // intentional and does not affect the reported time.
  start = micros();
  for(x=0; x<w; x+=r2) {
    for(y=0; y<h; y+=r2) {
      tft.drawCircle(x, y, radius, color);
    }
  }

  return micros() - start;
}

unsigned long testTriangles() {
  unsigned long start;
  int           n, i, cx = tft.width()  / 2 - 1,
                      cy = tft.height() / 2 - 1;

  tft.fillScreen(BLACK);
  n     = min(cx, cy);
  start = micros();
  for(i=0; i<n; i+=5) {
    tft.drawTriangle(
      cx    , cy - i, // peak
      cx - i, cy + i, // bottom left
      cx + i, cy + i, // bottom right
      tft.color565(i, i, i));
  }

  return micros() - start;
}

unsigned long testFilledTriangles() {
  unsigned long start, t = 0;
  int           i, cx = tft.width()  / 2 - 1,
                   cy = tft.height() / 2 - 1;

  tft.fillScreen(BLACK);
  start = micros();
  for(i=min(cx,cy); i>10; i-=5) {
    start = micros();
    tft.fillTriangle(cx, cy - i, cx - i, cy + i, cx + i, cy + i,
      tft.color565(0, i*10, i*10));
    t += micros() - start;
    tft.drawTriangle(cx, cy - i, cx - i, cy + i, cx + i, cy + i,
      tft.color565(i*10, i*10, 0));
    yield();
  }

  return t;
}

unsigned long testRoundRects() {
  unsigned long start;
  int           w, i, i2,
                cx = tft.width()  / 2 - 1,
                cy = tft.height() / 2 - 1;

  tft.fillScreen(BLACK);
  w     = min(tft.width(), tft.height());
  start = micros();
  for(i=0; i<w; i+=6) {
    i2 = i / 2;
    tft.drawRoundRect(cx-i2, cy-i2, i, i, i/8, tft.color565(i, 0, 0));
  }

  return micros() - start;
}

unsigned long testFilledRoundRects() {
  unsigned long start;
  int           i, i2,
                cx = tft.width()  / 2 - 1,
                cy = tft.height() / 2 - 1;

  tft.fillScreen(BLACK);
  start = micros();
  for(i=min(tft.width(), tft.height()); i>20; i-=6) {
    i2 = i / 2;
    tft.fillRoundRect(cx-i2, cy-i2, i, i, i/8, tft.color565(0, i, 0));
    yield();
  }

  return micros() - start;
}
