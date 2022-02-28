// Screen dimensions
#define SCREEN_WIDTH  320
#define SCREEN_HEIGHT 240 // Change this to 96 for 1.27" OLED.

// You can use any (4 or) 5 pins
//#define SCLK_PIN 12
//#define MOSI_PIN 13
// Need to modify variant.cpp to add the pin numbers for additional ports
#define DC_PIN   22 //PA13
#define CS_PIN   9 //PA19
#define RST_PIN  21 //PA12

// Color definitions
#define BLACK           0x0000
#define BLUE            0x001F
#define RED             0xF800
#define GREEN           0x07E0
#define CYAN            0x07FF
#define MAGENTA         0xF81F
#define ORANGE          0xFD20
#define YELLOW          0xFFE0
#define WHITE           0xFFFF
#define GREY            0x79EF
#define DARKBLUE        0x0007

#define BARCOLOR  YELLOW
#define GRAPHCOLOR  RED

#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <Adafruit_SPITFT.h>
#include <SPI.h>
#include "oledGraphClass.h"


// Option 1: use any pins but a little slower
//Adafruit_SSD1351 tft = Adafruit_SSD1351(SCREEN_WIDTH, SCREEN_HEIGHT, CS_PIN, DC_PIN, MOSI_PIN, SCLK_PIN, RST_PIN);

// Option 2: must use the hardware SPI pins
// (for UNO thats sclk = 13 and sid = 11) and pin 10 must be
// an output. This is much faster - also required if you want
// to use the microSD card (see the image drawing example)
//Adafruit_SSD1351 tft = Adafruit_SSD1351(SCREEN_WIDTH, SCREEN_HEIGHT, &SPI, CS_PIN, DC_PIN);
Adafruit_ILI9341 tft = Adafruit_ILI9341(CS_PIN, DC_PIN, RST_PIN);
bool CheckCableStatusByte(uint16_t errorCheck) {
  return ((cableState.statusByte & errorCheck) == errorCheck);
}
static const float cableOrangeThresh = 15.0; //threshold from red to organge for cables in tenths of an ohm, 150=15.0 ohms
static const float cableGreenThresh = 5.0;   //threshaold from orange to green for cables in tenths of an ohm, 50=5.0 ohms
static const float weaponOrangeThresh = 5.0; //threshold from red to organge for cables in tenths of an ohm, 50=5.0 ohms
static const float weaponGreenThresh = 2.0;   //threshaold from orange to green for cables in tenths of an ohm, 20=2.0 ohms
int slowShift = 0;         //counts refreshs to avoid fast scale shifts
int oldScale = 200;        //max value of full graph for last refresh
//int oldVal = 0;            //prior value (global so we don't have to pass by ref everywhere)
long unsigned int startTime, endTime, totalTime, timeSamples;

static const int cableBarGraphStartX = 120;
static const int cableBarGraphWidth = 320 - cableBarGraphStartX - 1;


void tftDisplayMessage(char *msg) {
  tft.setTextSize(2);
  tft.setCursor(2, 2);
  tft.setTextColor(ORANGE, DARKBLUE);
  tft.print(msg);

}

void printVal(int x, int y, int valColor, char *lab, int val) {
  tft.setTextSize(2);
  tft.setCursor(y, x);
  tft.setTextColor(YELLOW, BLACK);
  tft.print(lab); tft.print("="); //display connection string
  tft.setTextColor(valColor, BLACK);  //set the text color to right color, black background
  //Print ohms in a 4 character space
  //"XXXX", "XXX ", "XX  " or "X.XX"
  if (val == 9999) {
    tft.print("OPEN");
    return;
  }
  if (val < 0) val = 0;
  tft.print(val / 10); //display whole ohms
  if (val > 10000) return;  //if the value is >1000 ohms, no space
  if (val > 1000) {  //f the value is >100 ohms
    tft.print(" ");  //black out on space after a 3 digit value
    return;
  }
  if (val > 320) { //if the 100>value>32
    tft.print("  ");  //black out two spaces after a 2 digit value
    return;
  }
  if (val < 100) { //if less than 10 ohms
    tft.print("."); //display the period
    tft.print( val % 10); //display tenths
    tft.print(" ");  //make it 4 chars
  }
  else tft.print("  ");  //value between 32 and 10 ohms
  return;
}
/*
  int ValColor(int val) {
  if (val > weaponOrangeThresh) return RED;
  if (val > weaponGreenThresh) return ORANGE;
  return GREEN;
  }*/

void displaySplashScreen() {
  tft.fillRect(0, 0, 320, 240, BLACK);
  tft.drawRGBBitmap(0, 0, (uint16_t*) & (splashImage.pixel_data[0]), splashImage.width, splashImage.height);
}

void InitializeDisplay()
{
  //Initialize FastLED
#if FAST_LED_ACTIVE
  FastLED.addLeds<LED_TYPE, LED_DATA_PIN, COLOR_ORDER>(&lameLED, 1);
  FastLED.setCorrection( TypicalLEDStrip );
  lameLED = CRGB(25, 10, 70);
  FastLED.show();
#endif

  tft.begin();
  tft.setRotation(3);  //3 sets the display top to be aligned with the Feather uUSB.
  //tft.fillRect(0, 0, 320, 240, BLACK);
  tft.fillScreen(BLACK);
  tft.setCursor(0, 0);
  if (DISPLAY_SPLASH_IMAGE) {
    displaySplashScreen();
  } else {
    tft.setTextColor(YELLOW, BLACK); tft.setTextSize(2);
    tft.println("Welcome to\n  TTtarm\n");
    tft.setTextColor(CYAN, BLACK); tft.setTextSize(2);
    tft.println("  G Allen \n     &\n  B Rosen");
  }
  tft.setCursor(65, 120);
  tft.setTextSize(1);
  tft.setTextColor(MAGENTA, BLACK);
  tft.print(BUILD_DATE);

  tft.setCursor(0, 120);
  tft.setTextSize(1);
  tft.setTextColor(CYAN, BLACK);
  tft.print(VERSION_NUM);

  //oledGraph(Adafruit_SSD1351 *tft,int X, int Y, int height, int width,float minValue, float maxValue);
  weaponGraph = oledGraph(&tft, 0, 50, 180, 320, 0.0f, 10.0f);
  captureGraph = oledGraph(&tft, 0, 50, 180, 320, 0.0f, 40.0f);
  lameGraph = oledGraph(&tft, 0, 100, 240 - 100 - 1, 320, 0.0f, 20.0f);
  barGraphLame = oledBarGraph(&tft, 0,  55,  20,  320,  0.0,  25.0);
  barGraphA = oledBarGraph(&tft, cableBarGraphStartX, 40, 25, cableBarGraphWidth,  0.0f,  24.0);
  barGraphA.setTitle("AA=");
  barGraphB = oledBarGraph(&tft, cableBarGraphStartX, 120, 25, cableBarGraphWidth, 0.0f,  24.0);
  barGraphB.setTitle("BB=");
  barGraphC = oledBarGraph(&tft, cableBarGraphStartX, 200, 25, cableBarGraphWidth, 0.0f,  24.0);
  barGraphC.setTitle("CC=");

  int bars = 5;
  float lameVals[5] {0.0f, 5.0f, 10.0f, 15.0f, 20.0f};
  int lameColors[5] {lameGraph.cGREEN, lameGraph.cYELLOW, lameGraph.cORANGE, lameGraph.cRED, lameGraph.cRED};
  lameGraph.setHorizontalBarValues(5, lameVals, lameColors);
  barGraphLame.setHorizontalBarValues(5, lameVals, lameColors);
  barGraphLame.setTitlePosition(160 - 90, 25);
  barGraphLame.setTitleSize(3);
  barGraphLame.setTitle("");
  //lameGraph.setHorizontalBarValues(5, [0.0f,5.0f,10.0f,15.0f,20.0f], [lameGraph.cGREEN,lameGraph.cYELLOW,lameGraph.cORANGE,lameGraph.cRED,lameGraph.cRED]);

  float wvals[4] {0.0f, 2.0f, 5.0f, 10.0f};
  int wcolors[4] {weaponGraph.cGREEN, weaponGraph.cORANGE, weaponGraph.cRED, weaponGraph.cRED};
  weaponGraph.setHorizontalBarValues(4, wvals, wcolors);

  float cvals[4] {0.0f, 10.0f, 20.0f, 40.0f};
  int ccolors[4] {captureGraph.cGREEN, captureGraph.cORANGE, captureGraph.cRED, captureGraph.cRED};
  captureGraph.setHorizontalBarValues(4, cvals, ccolors);

  float cableVals[5] {0.0f, 5.0f, 10.0f, 15.0f, 20.0f};
  int cableColors[5] {lameGraph.cGREEN, lameGraph.cYELLOW, lameGraph.cORANGE, lameGraph.cRED, lameGraph.cRED};
  barGraphA.setHorizontalBarValues(5, cableVals, cableColors);
  barGraphB.setHorizontalBarValues(5, cableVals, cableColors);
  barGraphC.setHorizontalBarValues(5, cableVals, cableColors);

  lameLED=CRGB::Black;
  FastLED.show();
}

void dimOLEDDisplay() {
  //static bool alreadyDimmed=false;
  //Fill the rest of the display with black leaving only the top bar.
  //tft.fillRect(0, 17, 128, 128 - 17, BLACK);
#if FAST_LED_ACTIVE
  lameLED = CRGB::Black;
  FastLED.show();
#endif
}

void displayBatteryStatus() {
  char voltString[5] = "\0";
  char percString[5] = "\0";
  int battPercent = 75;
  int rectColor = WHITE;
  int fillColor = WHITE;
  byte barW;
  bool battActive = false;
  static const byte BattH = 10;
  static const byte BattW = 20;
  //static const byte BattX0=113-BattW-1;
  static const int BattX0 = 320 - BattW - 4;
  static const int BattY0 = 3;

  if (batteryVoltage > 2.5) {
    battPercent = int( (batteryVoltage - 3.3) / (4.1 - 3.3) * 100);
    battActive = true;
  } else {
    return;
  }
  tft.setTextSize(1);

  if (!battActive) {
    tft.fillRect(BattX0 - 1, BattY0 - 1, 128 - BattX0 + 1, BattY0 + 1, BLACK);
    return;
  }

  if (battPercent <= 15 ) {
    fillColor = RED;
    rectColor = RED;
  }
  if (battPercent >= 15 ) {
    fillColor = YELLOW;
  }
  if (battPercent >= 50 ) {
    fillColor = GREEN;
  }
  barW = min(BattW, (int) battPercent / BattW);
  if (barW < 0) {
    barW = 0;
  }

  tft.drawRect(BattX0 - 1, BattY0 - 1, BattW + 2, BattH + 2, rectColor);
  tft.drawFastVLine(BattX0 - 2, BattY0 + 1, 8, rectColor);
  tft.fillRect(BattX0 + (BattW - barW), BattY0, barW, BattH, fillColor);
  //tft.setCursor(BattX0 + BattW + 3, BattY0);
  //tft.setTextSize(1);
  //tft.setTextColor(CYAN,BLACK);
  //tft.print(battPercent);
}


int floatTo10xInt(float g) {
  if (g < 0.0) g = 0.0;
  return ((int) (g * 10.0 + .5));
}

float getOhms(char *s) {
  //arduino does not support strings in switch statements
  /*switch (s[0]) {
    case 'A':
      return floatTo10xInt(cableState.ohm_AA);
      break;
    case 'B':
      return floatTo10xInt(cableState.ohm_BB);
    case 'C':
      return floatTo10xInt(cableState.ohm_CC);
    }*/
  //if (s == "AA") return cableState.ohm_AA;
  //if (s == "BB") return cableState.ohm_BB;
  //if (s == "CC") return cableState.ohm_CC;

  for (int k = 0; k < NUM_ADC_SCAN_CHANNELS; k++) {
    if ((s[0] == ChanArray[k].ch_label[1]) && (s[1] == ChanArray[k].ch_label[2]) ) {
      return cableState.cableOhm[k];
    }
  }
  /*if (s == "AA") return floatTo10xInt(cableState.ohm_AA);
    if (s == "AB") return floatTo10xInt(cableState.ohm_AA);
    if (s == "AC") return floatTo10xInt(cableState.ohm_AA);
    if (s == "BA") return floatTo10xInt(cableState.ohm_BB);
    if (s == "BB") return floatTo10xInt(cableState.ohm_BB);
    if (s == "BC") return floatTo10xInt(cableState.ohm_BB);
    if (s == "CA") return floatTo10xInt(cableState.ohm_CC);
    if (s == "CB") return floatTo10xInt(cableState.ohm_CC);
    if (s == "CC") return floatTo10xInt(cableState.ohm_CC);*/
}


void labelTitle(char *s, int color) {
  const int sizeX=118;
  static char *oldFault;
  static GFXcanvas1 canvas(sizeX, 20);
  static int prevColor=BLACK;

  if ((strcmp(oldFault,s)==0) && (color==prevColor)) return; //no change
  
  canvas.setFont(&FreeSansBold12pt7b);
  oldFault = s;
  //tft.setTextSize(2);
  canvas.fillRect(0, 0, sizeX, 20, BLACK); //clear label area
  canvas.setCursor(0, 18);
  canvas.setTextColor(color, BLACK);
  canvas.println(s);
  tft.drawBitmap(0, 5, canvas.getBuffer(), sizeX, 20, color, BLACK);
}

void updateOLED(TestBoxModes Mode) {
  static int i = 0, val, oldMode = 'z';
  //static bool oledEnabled = true;
  enum lastConnected { first, none, epee, foil, shorted};
  enum displayStates {disp_off, disp_idle, disp_cable, disp_clip, disp_lame, disp_wpnR, disp_wpnTest, disp_capture, disp_unk, disp_error};
  static lastConnected lastConnection;
  lastConnected newConnection;
  //static lastConnected lastConnectionSeen;
  static displayStates currentDisplayState = disp_unk;
  static displayStates oldDisplayState = disp_unk;
  static int foilIndicator, epeeIndicator, foilInterIndicator, epeeInterIndicator;
  static unsigned long sTime; //start time of last seen foil/epee connection
  static unsigned long dispHoldTime; //Don't allow display to switch mode if this is set
  static unsigned long lastCapture = 0;
  bool inter;
  static long tIdleLEDOn = 0;
  //static long tIdleLEDOn

  //Serial.println(Mode);

  if ((millis() - tLastActive) > tOledOff) {
    //tft.enableDisplay(false);
    //oledEnabled = false;
    currentDisplayState = disp_off;
    //return;
  } else {
    if (currentDisplayState == disp_off) {
      //tft.enableDisplay(true);
      currentDisplayState = disp_unk;
      //oledEnabled = true;
    }
  }

  //Check the display state
  if (currentDisplayState != disp_off) {
    switch (Mode) {
      case CABLE:
        currentDisplayState = disp_cable;
        if (cableState.lameMode) {
          currentDisplayState = disp_lame;
          break;
        }
        if (cableState.maskMode) {
          //currentDisplayState = disp_clip;
          currentDisplayState = disp_cable;
          break;
        }
        break;
      case WPN_GRAPH:
        currentDisplayState = disp_wpnR;
        break;
      case HIT_CAPTURE:
        currentDisplayState = disp_capture;
        break;
      case WPN_TEST:
        currentDisplayState = disp_wpnTest;
        break;
      case BOX_IDLE:
        currentDisplayState = disp_idle;
        break;
    }
  }

  //Change the display state
  if (oldDisplayState != currentDisplayState) {
    //Serial.println("Setting new mode");
    //tft.fillRect(0, 0, 128, 128, BLACK);
    tft.fillScreen(BLACK);
    tft.setCursor(2, 2);
#if FAST_LED_ACTIVE
    lameLED = CRGB::Black;
    FastLED.show();
#endif
    //tft.setTextColor(YELLOW, BLACK); tft.setTextSize(2);

    switch (currentDisplayState) {
      case disp_lame:
        createLameDisplay();
        break;
      case disp_cable:
        //tft.setTextColor(YELLOW, BLACK); tft.setTextSize(2);
        //tft.print("Cable");
        labelTitle("Cable", YELLOW);
        barGraphA.resetGraph();
        barGraphA.setTitle("AA=");
        barGraphB.resetGraph();
        barGraphB.setTitle("BB=");        
        barGraphC.resetGraph();
        barGraphC.setTitle("CC=");    

        break;
      case disp_capture:
        labelTitle("Capture", RED);
        captureGraph.resetGraph();
        lastConnection = first;
        break;
      case disp_wpnR:
        foilIndicator = epeeIndicator = foilInterIndicator = epeeInterIndicator = BLACK;
        lastConnection = first;
        createWeaponDisplay();
        break;
      case disp_wpnTest:
        foilIndicator = epeeIndicator = foilInterIndicator = epeeInterIndicator = BLACK;
        lastConnection = first;
        labelTitle("WpnTest", YELLOW);
        //tft.setTextSize(2);
        //tft.print("WpnTest");
        break;
      case disp_idle:
        tft.setCursor(2, 2);
        //tft.setTextSize(2);
        //tft.setTextColor(BLUE, BLACK);
        labelTitle(" Idle", BLUE);
        //tft.print("Idle");
        //tft.fillScreen(BLACK);
    }
    oldDisplayState = currentDisplayState;
    displayBatteryStatus();
  }

  switch (currentDisplayState) {
    case disp_cable:
    case disp_clip:
      startTime = micros();
      updateDisplayCableMode();
      endTime = micros();
      if (startTime < endTime) {
        totalTime += (endTime - startTime);
        timeSamples += 1ul;
      }
      break;

    case disp_lame:
      updateLameDisplay();
      break;

    case disp_capture: {
        updateWeaponIndicators();
        static bool armed = false;
        long captureDuration = 0;
        int trigIndx, lastIndx, trimVal;

        if (FoilADC.hsBuffer.CaptureDone()) {
          newConnection = foil;
          lastConnection = foil;
          sTime = millis();
          trigIndx = FoilADC.hsBuffer.getTriggerIndex();
          lastIndx = FoilADC.hsBuffer.getLastTriggerIndex();
          trimVal = FoilADC.getTrim();
          captureDuration = FoilADC.hsBuffer.getTriggerDuration();
          labelTitle("Foil hit", RED);
          //armed=false;
        }

        if (EpeeADC.hsBuffer.CaptureDone()) {
          newConnection = epee;
          lastConnection = epee;
          trigIndx = EpeeADC.hsBuffer.getTriggerIndex();
          lastIndx = EpeeADC.hsBuffer.getLastTriggerIndex();
          trimVal = EpeeADC.getTrim();
          captureDuration = EpeeADC.hsBuffer.getTriggerDuration();
          sTime = millis();
          labelTitle("Epee hit", RED);
          //armed=false;
        }

        if ( millis() > (dispCaptureHoldTime + lastCapture) ) {
          if (armed == false) {
            FoilADC.hsBuffer.ResetTrigger();
            EpeeADC.hsBuffer.ResetTrigger();
            //StartADC();
            labelTitle("Armed", GREEN);
            armed = true;
          }
          if (FoilADC.hsBuffer.CaptureDone() || EpeeADC.hsBuffer.CaptureDone())  {
            lastCapture = millis();
            armed = false;
            captureGraph.resetGraph();
            for (int k = 0; k < ADC_CAPTURE_LEN; k++) {
              captureGraph.updateGraph((ADC_CaptureBuffer[k] - trimVal)*FoilADC.LOW_GAIN);
            }
            tft.drawFastVLine(trigIndx, 28, 127, CYAN);
            tft.drawFastVLine(lastIndx, 28, 127, CYAN);
            tft.setTextSize(1);
            tft.setTextColor(CYAN);
            tft.setCursor(trigIndx + 20, 65);
            tft.print(captureDuration / 1000);
            if (lastIndx >= (ADC_CAPTURE_LEN - 1)) {
              tft.print("+");
            }
            tft.print(" ms");
            captureGraph.drawTextLabels();

            //weaponGraph.updateGraph(weaponState.ohm_Foil);
            //FoilADC.hsBuffer.ResetTrigger();
            //EpeeADC.hsBuffer.ResetTrigger();
            //labelTitle("Armed",GREEN);
          }
        }

        /*switch (lastConnection) {
          case foil:
            //weaponGraph.updateGraph(weaponState.ohm_Foil);
            printVal(0, 50, YELLOW, "", weaponState.ohm10xFoil);
            break;
          case epee:
            //weaponGraph.updateGraph(weaponState.ohm_Epee);
            printVal(0, 50, YELLOW, "", weaponState.ohm10xEpee);
            break;
          }*/
      }
      break;
    case disp_wpnR:
      updateWeaponIndicators();

      newConnection = lastConnection;
      if (weaponState.foilOn) {
        //lastConnection = foil;
        sTime = millis();
        newConnection = foil;
      }
      if (weaponState.epeeOn) {
        //lastConnection = epee;
        newConnection = epee;
        sTime = millis();
      }
      if ((lastConnection == shorted) && (sTime < dispHoldTime)) {
        newConnection = shorted;
      }

      if ((weaponState.foilOn) && (weaponState.epeeOn)) { //short
        newConnection = shorted;
        dispHoldTime = millis() + 5000ul;
        //lastConnection = shorted;
      }

      if ((millis() - sTime) > 5000ul) {
        newConnection = none;
      }

      if (newConnection != lastConnection)  {
        //tft.fillRect(0, 28, 128, 100, BLACK);
        //tft.setTextSize(2);
        //tft.setTextColor(GREEN, BLACK);
        //tft.setCursor(2, 2);
        if (newConnection == epee) {
          //tft.print("Epee ");
          labelTitle("Epee", GREEN);
          weaponGraph.resetGraph();
        }
        if (newConnection == foil) {
          //tft.print("Foil ");
          labelTitle("Foil", GREEN);
          weaponGraph.resetGraph();
        }
        if (newConnection == shorted) {
          labelTitle("Wep GND", RED);
          /*
            tft.fillRect(0, 27, 128, 100, BLACK);
            oldA = oldB = 320;
            //oldA=weaponState.ohm10xEpee;
            //oldB=weaponState.ohm10xFoil;
            barGraph(BBAR, 8, weaponState.ohm10xEpee, oldA, "AB");
            barGraph(CBAR, 8, weaponState.ohm10xFoil, oldB, "BC");*/
        }
        if (newConnection == none) {
          //tft.setTextColor(YELLOW, BLACK);
          weaponGraph.resetGraph();
          if (lastConnection == epee) {
            labelTitle("Epee Open", YELLOW);
            //tft.print("Epee Open");
          }
          if (lastConnection == foil) {
            labelTitle("Foil Open", YELLOW);
            //tft.print("Foil Open");
          }
        }
        lastConnection = newConnection;
      }

      switch (lastConnection) {
        case foil:
          weaponGraph.updateGraph(weaponState.ohm_Foil);
          printVal(0, 50, YELLOW, "", weaponState.ohm10xFoil);
          break;
        case epee:
          weaponGraph.updateGraph(weaponState.ohm_Epee);
          printVal(0, 50, YELLOW, "", weaponState.ohm10xEpee);
          break;
        case first:
        case none:
          break;
        case shorted:
          //barGraph(BBAR, 8, weaponState.ohm10xEpee, oldA, "AB");
          //oldA=weaponState.ohm10xEpee;
          //barGraph(CBAR, 8, weaponState.ohm10xFoil, oldB, "BC");
          //oldB=weaponState.ohm10xFoil;
          break;
      }
      break;
    case disp_wpnTest:
      updateWeaponTestLights();
      break;
    case disp_idle:
    case disp_off:
      //Serial.println("Updating display");
      if ((millis() - tIdleLEDOn) > tIdleLEDBlink) {
        digitalWrite(LED1_PIN, HIGH);
        delay(10);
        digitalWrite(LED1_PIN, LOW);
        tIdleLEDOn = millis();
      }
      break;
  }
}

void updateWeaponIndicators() {
  static int foilIndicator = BLACK;
  static int epeeIndicator = BLACK;
  static int foilInterIndicator = BLACK;
  static int epeeInterIndicator = BLACK;

  if (weaponState.foilOn) {
    if (foilIndicator != RED) {
      tft.fillRect(0, 19, 20, 7, RED);
      foilIndicator = RED;
    }
  }
  else if (foilIndicator != BLACK) {
    tft.fillRect(0, 19, 20, 7, BLACK);
    foilIndicator = BLACK;
  }
  if (weaponState.foilInterOn) {
    if (foilInterIndicator != YELLOW) {
      tft.fillRect(20, 19, 20, 7, YELLOW);
      foilInterIndicator = YELLOW;
    }
  }
  else if (foilInterIndicator != BLACK) {
    tft.fillRect(20, 19, 20, 7, BLACK);
    foilInterIndicator = BLACK;
  }
  if (weaponState.epeeOn) {
    if (epeeIndicator != GREEN) {
      tft.fillRect(70, 19, 20, 7, GREEN);
      epeeIndicator = GREEN;
    }
  }
  else if (epeeIndicator != BLACK) {
    tft.fillRect(70, 19, 20, 7, BLACK);
    epeeIndicator = BLACK;
  }
  if (weaponState.epeeInterOn) {
    if (epeeInterIndicator != YELLOW) {
      tft.fillRect(90, 19, 20, 7, YELLOW);
      epeeInterIndicator = YELLOW;
    }
  }
  else if (epeeInterIndicator != BLACK) {
    tft.fillRect(90, 19, 20, 7, BLACK);
    epeeInterIndicator = BLACK;
  }
}

void updateWeaponTestLights() {
  static int foilIndicator = BLACK;
  static int epeeIndicator = BLACK;
  static int foilInterIndicator = BLACK;
  static int epeeInterIndicator = BLACK;
  if (weaponState.foilOn) {
    if (foilIndicator != RED) {
      tft.fillRect(0, 28, 50, 50, RED);
      foilIndicator = RED;
    }
  }
  else if (foilIndicator != BLACK) {
    tft.fillRect(0, 28, 50, 50, BLACK);
    foilIndicator = BLACK;
  }
  if (weaponState.foilInterOn) {
    if (foilInterIndicator != YELLOW) {
      tft.fillRect(0, 80, 50, 40, YELLOW);
      foilInterIndicator = YELLOW;
    }
  }
  else if (foilInterIndicator != BLACK) {
    tft.fillRect(0, 80, 50, 40, BLACK);
    foilInterIndicator = BLACK;
  }
  if (weaponState.epeeOn) {
    if (epeeIndicator != GREEN) {
      tft.fillRect(70, 28, 128 - 50 , 50 , GREEN);
      epeeIndicator = GREEN;
    }
  }
  else if (epeeIndicator != BLACK) {
    tft.fillRect(70, 28, 128 - 50 , 50 , BLACK);
    epeeIndicator = BLACK;
  }
  if (weaponState.epeeInterOn) {
    if (epeeInterIndicator != YELLOW) {
      tft.fillRect(70, 80, 128 - 50, 40, YELLOW);
      epeeInterIndicator = YELLOW;
    }
  }
  else if (epeeInterIndicator != BLACK) {
    tft.fillRect(70, 80, 128 - 50, 40, BLACK);
    epeeInterIndicator = BLACK;
  }
}

void updateDisplayCableMode() {
  //static char displayMode = 'c';
  bool ABcross = CheckCableStatusByte( (1 << BITAB) );
  bool ACcross = CheckCableStatusByte( (1 << BITAC)  );
  bool BAcross = CheckCableStatusByte( (1 << BITBA)  );
  bool BCcross = CheckCableStatusByte((1 << BITBC));
  bool CAcross = CheckCableStatusByte((1 << BITCA));
  bool CBcross = CheckCableStatusByte((1 << BITCB));
  bool ABshort = (CheckCableStatusByte((1 << BITAB)) || CheckCableStatusByte((1 << BITBA))) && ((!CheckCableStatusByte((1 << BITAA))) || (!CheckCableStatusByte((1 << BITBB))));
  bool BCshort = (CheckCableStatusByte((1 << BITBC)) || CheckCableStatusByte((1 << BITCB))) && ((!CheckCableStatusByte((1 << BITBB))) || (!CheckCableStatusByte((1 << BITCC))));
  bool ACshort = (CheckCableStatusByte((1 << BITAC)) || CheckCableStatusByte((1 << BITCA))) && ((!CheckCableStatusByte((1 << BITAA))) || (!CheckCableStatusByte((1 << BITCC))));
  static char buf[15];

  //tft.setTextSize(2);

  if (ABshort) //AB short
    if (BCshort) {//if AB and BC then AC has to be shorted
      //A - B - C short
      labelTitle("Short ABC", RED);
      barGraphA.setTitle("AB=");
      barGraphA.updateGraph(getOhms("AB"));

      barGraphB.setTitle("BC=");
      barGraphB.updateGraph(getOhms("BC"));

      barGraphC.setTitle("AC=");
      barGraphC.updateGraph(getOhms("AC"));
    }
    else {
      //AB short, CC okay or open
      labelTitle("Short AB", RED);
      barGraphA.setTitle("AB=");
      barGraphA.updateGraph(getOhms("AB"));

      barGraphB.setTitle("BB=");
      barGraphB.updateGraph(cableState.ohm_BB);

      barGraphC.setTitle("CC=");
      barGraphC.updateGraph(cableState.ohm_CC);
    }
  else if (BCshort) {
    //BC and not AB (and therefore not AC), AA okay or open
    labelTitle("Short BC", RED);
    barGraphA.setTitle("AB=");
    barGraphA.updateGraph(getOhms("AB"));

    barGraphB.setTitle("BC=");
    barGraphB.updateGraph(getOhms("BC"));

    barGraphC.setTitle("CC=");
    barGraphC.updateGraph(cableState.ohm_CC);
  }
  else if (ACshort) {
    //AC and not AB, and therefore not BC, BB okay or open
    labelTitle("Short AC", RED);
    barGraphA.setTitle("AC=");
    barGraphA.updateGraph(getOhms("AC"));

    barGraphB.setTitle("BB=");
    barGraphB.updateGraph(cableState.ohm_BB);

    barGraphC.setTitle("CC=");
    barGraphC.updateGraph(cableState.ohm_CC);
  }
  else //no shorts
    if (ABcross)
      if (BAcross) {
        //AB cross, CC okay or open
        labelTitle("AB Cross", RED);
        barGraphA.setTitle("AB=");
        barGraphA.updateGraph(getOhms("AB"));

        barGraphB.setTitle("BA=");
        barGraphB.updateGraph(getOhms("BA"));

        barGraphC.setTitle("CC=");
        barGraphC.updateGraph(cableState.ohm_CC);
      }
      else if (CAcross) {
        //A->B C->A, could be B->C or open
        labelTitle("ABC Cross", RED);
        barGraphA.setTitle("AC=");
        barGraphA.updateGraph(getOhms("AC"));

        barGraphB.setTitle("BB=");
        barGraphB.updateGraph(cableState.ohm_BB);

        barGraphC.setTitle("CC=");
        barGraphC.updateGraph(cableState.ohm_CC);
      }
      else {
        //AB cross, don't know what happened to BA
        labelTitle("AB Cross", RED);
        barGraphA.setTitle("AB=");
        barGraphA.updateGraph(getOhms("AB"));

        barGraphB.setTitle("BB=");
        barGraphB.updateGraph(cableState.ohm_BB);

        barGraphC.setTitle("CC=");
        barGraphC.updateGraph(cableState.ohm_CC);
      }
    else if (BCcross) {
      //we could test CB, and know it was a real BC cross, but if that failed, something has to be open
      labelTitle("BC Cross", RED);
      barGraphA.setTitle("AA=");
      barGraphA.updateGraph(cableState.ohm_AA);

      barGraphB.setTitle("BC=");
      barGraphB.updateGraph(getOhms("BC"));

      barGraphC.setTitle("CB=");
      barGraphC.updateGraph(getOhms("CB"));

    }
    else if (ACcross)
      if (CAcross) {
        //AC cross, BB good or open
        labelTitle("AC Cross", RED);
        barGraphA.setTitle("AC=");
        barGraphA.updateGraph(getOhms("AC"));

        barGraphB.setTitle("BB=");
        barGraphB.updateGraph(cableState.ohm_BB);

        barGraphC.setTitle("CA=");
        barGraphC.updateGraph(getOhms("CA"));
      }
      else if (BAcross) {
        //A->C, B->A, C->B
        labelTitle("ACB Cross", RED);
        barGraphA.setTitle("AC=");
        barGraphA.updateGraph(getOhms("AC"));

        barGraphB.setTitle("BA=");
        barGraphB.updateGraph(getOhms("BA"));

        barGraphC.setTitle("CB=");
        barGraphC.updateGraph(getOhms("CB"));

      }
      else {
        //something is open
        labelTitle("AC Cross", RED);
        barGraphA.setTitle("AC=");
        barGraphA.updateGraph(getOhms("AC"));

        barGraphB.setTitle("BB=");
        barGraphB.updateGraph(cableState.ohm_BB);

        barGraphC.setTitle("CA=");
        barGraphC.updateGraph(getOhms("CA"));
      }

    else  //something is open
      if (BAcross) {
        //BA but not AB
        labelTitle("BA Cross", RED);
        barGraphA.setTitle("AB=");
        barGraphA.updateGraph(getOhms("AB"));

        barGraphB.setTitle("BA=");
        barGraphB.updateGraph(getOhms("BA"));

        barGraphC.setTitle("CC=");
        barGraphC.updateGraph(cableState.ohm_CC);
      }
      else if (CBcross) {
        //CB but not BC
        labelTitle("CB Cross", RED);
        barGraphA.setTitle("AA=");
        barGraphA.updateGraph(cableState.ohm_AA);

        barGraphB.setTitle("BC=");
        barGraphB.updateGraph(getOhms("BC"));

        barGraphC.setTitle("CB=");
        barGraphC.updateGraph(getOhms("CB"));

      }
      else if (CAcross) {
        //CA but not AC
        labelTitle("CA Cross", RED);
        barGraphA.setTitle("AC=");
        barGraphA.updateGraph(getOhms("AC"));

        barGraphB.setTitle("BB=");
        barGraphB.updateGraph(cableState.ohm_BB);

        barGraphC.setTitle("CA=");
        barGraphC.updateGraph(getOhms("CA"));

      }
      else {//no shorts/no crosses
        //labelTitle("Cable", GREEN);
        barGraphA.setTitle("AA=");
        barGraphA.updateGraph(cableState.ohm_AA);

        barGraphB.setTitle("BB=");
        barGraphB.updateGraph(cableState.ohm_BB);

        barGraphC.setTitle("CC=");
        barGraphC.updateGraph(cableState.ohm_CC);
         
        strcpy(buf,"Fault ");
        bool cableFault=false;
        
        if (CheckCableStatusByte((1<<BITAA))) {
          sprintf(buf,"%s%c",buf,'A');
          cableFault=true;
        }        
        if (CheckCableStatusByte((1<<BITBB))) {
          sprintf(buf,"%s%c",buf,'B');
          cableFault=true;
        }
        if (CheckCableStatusByte((1<<BITCC))) {
          sprintf(buf,"%s%c",buf,'C');
          cableFault=true;
        }
        if (cableFault) {
          //strcat("\0",buf);
          labelTitle(buf, RED);
        } else {
          labelTitle("Cable",GREEN);
        }
      }
}

void createWeaponDisplay() {
  //tft.setCursor(2, 2);
  //tft.setTextColor(YELLOW, BLACK); tft.setTextSize(2);
  //tft.print("Weapon");
  labelTitle("Weapon", YELLOW);
  weaponGraph.resetGraph();
}

// barGraph(X, H, oldVal, newVal, oldVal)
void createLameDisplay() {
  labelTitle("Lame", YELLOW);
  lameGraph.resetGraph();
  barGraphLame.resetGraph();
}

void updateLameDisplay() {
  static int oldVal = 9999;
  float lameVal;
  static byte i = 0;

  if (cableState.ohm_Lame<MAX_LAME_RESISTANCE) {
    lameVal=cableState.ohm_Lame;
  } else {
    lameVal=OPEN_CIRCUIT_VALUE;
  }

  if (lameVal <= 5.0) {
    labelTitle("Lame", GREEN);
  } else {
    labelTitle("Lame", RED);
  }
  lameGraph.updateGraph(lameVal);
  barGraphLame.updateGraph(lameVal);
#if FAST_LED_ACTIVE
  updateLED(lameVal);
#endif FAST_LED_ACTIVE
}

#if FAST_LED_ACTIVE
void updateLED(float value) {
  const float lameVals[5] {-1.0f, 5.0f, 10.0f, 15.0f, 20.0f};
  const CRGB lameColors[5] {CRGB::Green, CRGB::Yellow, CRGB::Orange, CRGB::OrangeRed, CRGB::DarkRed};
  static CRGB prevLEDColor = CRGB::Black;
  uint8_t R, G, B;

  R = G = B = 0;
  lameLED = CRGB::DarkRed;
  for (int k = 0; k < 5; k++) {
    if (value >= lameVals[k]) {
      lameLED = lameColors[k];
    } else {
      break;
    }
  }
  lameLED.nscale8_video(64); //50% brightness reduction
  if (lameLED != prevLEDColor) {
    prevLEDColor = lameLED;
    //lameLED=CRGB::Green;
    FastLED.show();
  }
}
#endif

void writeSerialOutput(TestBoxModes Mode) {
  // Use Generic format "WF01E00CA00.0B00.0C00.0AB etc
  // First letter: W/C indicates mode
  // F - Foil, On/Off, Intermittent light
  // E - Epee , On/Off, Intermittent light
  // C - Cable, 4-digit float
  const long tHeartBeatInterval = 5000; //Time in ms
  const byte tempBufferSize = 16;
  int bufferSize = SERIAL_BUFFER_SIZE;
  char tempString1[tempBufferSize];
  char tempString2[tempBufferSize];
  static long tHeartBeat = 0;

  //char printStringBuffer[SERIAL_BUFFER_SIZE];

  long t_now = millis();
  static long t_last_upd = 0;
  float EffSampleRate = 0;
  // return;

  tempString1[0] = '\0'; //Reset the temp String
  tempString2[0] = '\0'; //Reset the temp String
  //tempString3[0]='\0';  //Reset the temp String
  outputString[0] = '\0'; //Reset the output string

  if ((t_now - tHeartBeat) > tHeartBeatInterval) {
    float dt = (float(t_now - t_last_upd) * 1.0e-3);
    EffSampleRate = float(numSamples) / dt;

    dtostrf(EffSampleRate, 5, 0, tempString1); //Effective sample rate
    dtostrf(batteryVoltage, 5, 2, tempString2);
    snprintf(outputString, bufferSize, "t=%ld, N=%ld, %sHz, Bat=%sV\r\n", t_now, numSamples, tempString1, tempString2);
    Serial.write(outputString);
    tHeartBeat = millis();
    t_last_upd = millis();
    numSamples = 0;

    Serial.println("Sample buffer[0]:");
    for (int k=0;k<13; k++) {
      Serial.println( (ChanArray[0].sampleBuffer[k] >>8));
    }
  }

  outputString[0] = '\0'; //Reset the outputString
  switch (Mode) {
    case CABLE:
      //Print the status byte
      dtostrf(cableState.ohm_AA, 5, 2, tempString1);
      dtostrf(cableState.ohm_AAMax, 5, 2, tempString2);

      snprintf(outputString, bufferSize, "C1,%ld,%x,A,%s,%s,B,", t_now, cableState.statusByte, tempString1, tempString2);
      dtostrf(cableState.ohm_BB, 5, 2, tempString1);
      dtostrf(cableState.ohm_BBMax, 5, 2, tempString2);
      strncat(outputString, tempString1, bufferSize); strncat(outputString, ",", bufferSize); strncat(outputString, tempString2, bufferSize);
      dtostrf(cableState.ohm_CC, 5, 2, tempString1);
      dtostrf(cableState.ohm_CCMax, 5, 2, tempString2);
      strncat(outputString, ",C,", bufferSize); strncat(outputString, tempString1, bufferSize); strncat(outputString, ",", bufferSize); strncat(outputString, tempString2, bufferSize);
      strncat(outputString, "\r\n", bufferSize);
      if (Serial) {
        Serial.write(outputString);
        Serial.flush();
      }
      snprintf(outputString, bufferSize, "C2,%ld,%x", t_now, cableState.statusByte);
      snprintf(tempString1, tempBufferSize, ",AA,%d", cableState.line_AA);
      strncat(outputString, tempString1, bufferSize);
      snprintf(tempString1, tempBufferSize, ",AB,%d", cableState.line_AB);
      strncat(outputString, tempString1, bufferSize);
      snprintf(tempString1, tempBufferSize, ",AC,%d", cableState.line_AC);
      strncat(outputString, tempString1, bufferSize);
      snprintf(tempString1, tempBufferSize, ",BA,%d", cableState.line_BA);
      strncat(outputString, tempString1, bufferSize);
      snprintf(tempString1, tempBufferSize, ",BB,%d", cableState.line_BB);
      strncat(outputString, tempString1, bufferSize);
      snprintf(tempString1, tempBufferSize, ",BC,%d", cableState.line_BC);
      strncat(outputString, tempString1, bufferSize);
      snprintf(tempString1, tempBufferSize, ",CA,%d", cableState.line_CA);
      strncat(outputString, tempString1, bufferSize);
      snprintf(tempString1, tempBufferSize, ",CB,%d", cableState.line_CB);
      strncat(outputString, tempString1, bufferSize);
      snprintf(tempString1, tempBufferSize, ",CC,%d", cableState.line_CC);
      strncat(outputString, tempString1, bufferSize);
      if (timeSamples > 0ul) {
        snprintf(tempString1, tempBufferSize, ",DTime, %ul", (totalTime / timeSamples) / 1000ul);
        strncat(outputString, tempString1, bufferSize);
      }
      strncat(outputString, "\r\n", bufferSize);
      /*if (Serial.availableForWrite()>strlen(outputString) ){
         Serial.write(outputString);
        }*/
      /*
        dtostrf(cableState.ohm_AA,5,2,tempString1);
        dtostrf(cableState.ohm_BB,5,2,tempString2);
        dtostrf(cableState.ohm_CC,5,2,tempString3);
        //Serial.print(tempString1); Serial.print(tempString2); Serial.println(tempString3);
        snprintf(outputString,bufferSize,"Rave (A B C) =%u, %s, %s, %s\r\n\0",cableState.statusByte,tempString1,tempString2,tempString3);
        Serial.write(outputString);

        dtostrf(cableState.ohm_AAMax,5,2,tempString1);
        dtostrf(cableState.ohm_BBMax,5,2,tempString2);
        dtostrf(cableState.ohm_CCMax,5,2,tempString3);
        snprintf(outputString,bufferSize,"Rmax =, %s, %s, %s\r\n\0",tempString1,tempString2,tempString3);
        Serial.write(outputString);

        snprintf(outputString,bufferSize,"Xadc (AB AC BC)=, %d, %d, %d\r\n\0",cableState.line_AB,cableState.line_AC,cableState.line_BC);
        Serial.write(outputString);
      */
      break;
    case WPN_TEST:
      //Format is "W,time,E,Light On/Off,t last epee trigger, F, Foil light On/Off, t last foil trigger
      snprintf(outputString, bufferSize, "W,%ld,E,%u,%ld,F,%u,%ld\r\n", t_now, weaponState.epeeOn, weaponState.tEpeeTrigger, weaponState.foilOn, weaponState.tFoilTrigger);
      //Serial.write(outputString);
      break;
    case HIT_CAPTURE:
    case WPN_GRAPH:
      //Format is "R,time,Epee resistance, Foil resistance"
      snprintf(outputString, bufferSize, "R,%ld,E,", t_now);
      dtostrf(weaponState.ohm_Epee, 5, 2, tempString1);
      dtostrf(weaponState.ohm_Foil, 5, 2, tempString2);
      strncat(outputString, tempString1, bufferSize); strncat(outputString, ",F,", bufferSize); strncat(outputString, tempString2, bufferSize);
      strncat(outputString, "\r\n", bufferSize);
      break;
  }

  Serial.write(outputString); //Serial.println(totalTime / timeSamples);

  /*if (SerialBT.available()){
    SerialBT.print(outputString);
    }*/
}
