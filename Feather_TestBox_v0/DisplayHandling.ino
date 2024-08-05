bool CheckCableStatusByte(uint16_t errorCheck) {
  return ((cableState.statusByte & errorCheck) == errorCheck);
}
int cableOrangeThresh = 150;  //threshold from red to organge for cables in tenths of an ohm, 150=15.0 ohms
int cableGreenThresh = 50;    //threshaold from orange to green for cables in tenths of an ohm, 50=5.0 ohms
int weaponOrangeThresh = 50;  //threshold from red to organge for cables in tenths of an ohm, 50=5.0 ohms
int weaponGreenThresh = 20;   //threshaold from orange to green for cables in tenths of an ohm, 20=2.0 ohms
int slowShift = 0;            //counts refreshs to avoid fast scale shifts
int oldScale = 200;           //max value of full graph for last refresh
//int oldVal = 0;            //prior value (global so we don't have to pass by ref everywhere)
#define SLOWCOUNT 8  //how many refreshes before scale can change
long unsigned int startTime, endTime, totalTime, timeSamples;


int scaleWidth(int val) {  //tenths don't divide by 4 very well
  int sVal;                //so this expressly sets the break points
  sVal = (val / 10) * 4;
  switch (val % 10) {
    case 0:
    case 1: return sVal; break;  //x.0 or x.1 is x
    case 2:
    case 3: return (sVal + 1); break;  //x.2 or x.3 is x 1/4
    case 4:
    case 5:
    case 6: return (sVal + 2); break;  //x.4, x.5, x.6 is x 1/2
    case 7:
    case 8: return (sVal + 3); break;  //x7, x.8 is x 3/4
    case 9: return (sVal + 4); break;  //x.9 is x+1
  }

  //Should never get here
  return 0;
}

void tftDisplayMessage(const char *msg) {
  tft.setTextSize(2);
  tft.setCursor(2, 2);
  tft.setTextColor(ORANGE, DARKBLUE);
  tft.print(msg);
}

void printVal(int x, int y, int valColor, const char *lab, int val) {
  tft.setTextSize(2);
  tft.setCursor(y, x);
  tft.setTextColor(YELLOW, BLACK);
  tft.print(lab);
  tft.print("=");                     //display connection string
  tft.setTextColor(valColor, BLACK);  //set the text color to right color, black background
  //Print ohms in a 4 character space
  //"XXXX", "XXX ", "XX  " or "X.XX"
  if (val == 9999) {
    tft.print("OPEN");
    return;
  }
  if (val < 0) val = 0;
  tft.print(val / 10);      //display whole ohms
  if (val > 10000) return;  //if the value is >1000 ohms, no space
  if (val > 1000) {         //f the value is >100 ohms
    tft.print(" ");         //black out on space after a 3 digit value
    return;
  }
  if (val > 320) {    //if the 100>value>32
    tft.print("  ");  //black out two spaces after a 2 digit value
    return;
  }
  if (val < 100) {         //if less than 10 ohms
    tft.print(".");        //display the period
    tft.print(val % 10);   //display tenths
    tft.print(" ");        //make it 4 chars
  } else tft.print("  ");  //value between 32 and 10 ohms
  return;
}

// barGraph(X, H, oldVal, newVal, oldVal)
//  X = top lime
//  H = height of bar
//  oldVal = last value in tenths of an ohm
//  newVal = current value
// with 128 pixel wide screen, show 32 ohms * 4 pixels/ohm
void barGraph(int X, int H, int newVal, int &oldVal, const char *conn) {
  int difVal, rOld, rNew, bc = ORANGE;
  bool newColor = false;
  //Serial.print(conn); Serial.print("||New value = ");Serial.println(newVal);
  if (newVal > cableOrangeThresh) {                                                          //if the new value is greater than the orange threshold
    bc = RED;                                                                                //then its a red bar
    if (oldVal <= cableOrangeThresh) newColor = true;                                        //did we change color?
  } else if (newVal <= cableGreenThresh) {                                                   //if new value is less than the green threshold
    bc = GREEN;                                                                              //then its a green bar
    if (oldVal > cableGreenThresh) newColor = true;                                          //did we change color
  } else if ((oldVal < cableOrangeThresh) || (oldVal >= cableGreenThresh)) newColor = true;  //orange, but not if the prior value wasn't orange
  printVal(X, 0, bc, conn, newVal);
  if (newVal > 320) {
    tft.fillRect(0, X + 16, 128, H, BLACK);
    oldVal = newVal;
    return;  // no bar if beyond 32 ohms
  }
  rOld = 320 - oldVal;                                                                         //row of prior value
  rNew = 320 - newVal;                                                                         //row of new value
  difVal = rNew - rOld;                                                                        //length of bar (could be negative
  if (difVal > 0) {                                                                            //increase bar graph length
    if (newColor) tft.fillRect(0, X + 16, scaleWidth(rNew), H, bc);                            //if color change, draw complete bar in new color
    else tft.fillRect(scaleWidth(rOld), X + 16, scaleWidth(rNew) - scaleWidth(rOld), H, bc);   //just draw extension
  } else if (difVal < 0) {                                                                     //decrease bar graph length
    if (newColor) tft.fillRect(0, X + 16, scaleWidth(rNew), H, bc);                            //if color change, draw complete bar
    tft.fillRect(scaleWidth(rNew), X + 16, -(scaleWidth(rNew) - scaleWidth(rOld)), H, BLACK);  //black out right part of prior bar
  }
  oldVal = newVal;  //and we're done, save old value
}

int ValColor(int val) {
  if (val > weaponOrangeThresh) return RED;
  if (val > weaponGreenThresh) return ORANGE;
  return GREEN;
}
void drawVLine(int col, int startRow, int endRow, int color) {
  int sRow = min(max(27, startRow), 127);
  int eRow = min(max(27, endRow), 127);
  if (sRow == eRow) tft.drawPixel(col, sRow, color);
  else tft.drawFastVLine(col, sRow, eRow - sRow + 1, color);
  String s = "???";
  if (color == RED) s = "RED";
  if (color == ORANGE) s = "ORANGE";
  if (color == GREEN) s = "GREEN";
  if (color == BLACK) s = "BLACK";
  //  if (weaponState.ohm10xFoil > 18) { Serial.print(col);Serial.print("=");Serial.print(sRow);Serial.print(",");Serial.print(eRow);Serial.print(","); Serial.println(s);}
}

void displayRGB565Bitmap(int16_t x, int16_t y, uint16_t *pixels, uint16_t w, uint16_t h) {
  
  tft.startWrite();

  //  hWindow=(h>hMax) ? h : hMax
  //for (uint16_t k=0; k<h; k++) { 
  tft.setAddrWindow(x, y, w, h);
  //tft.setAddrWindow(x,y,w,32);
  tft.writePixels(&(pixels[0]), w * h);

  //tft.dmaWait();
  // Reset the address window to full screen;
  //tft.setAddrWindow(0,0,SCREEN_WIDTH,SCREEN_HEIGHT);
  tft.endWrite();
}


/*
void displayRGB565Bitmap(uint16_t x, uint16_t y, uint16_t *pixels, uint16_t w, uint16_t h) { 
  tft.startWrite();
  for (int16_t j = 0; j < h; j++) {    
    for (int16_t i = 0; i < w; i++) {
      tft.writePixel(x + i, y+j, pixels[j * w + i]);
    }
  }
  tft.dmaWait();
  tft.endWrite();
}*/


void displaySplashScreen() {
  int16_t xi, yj;
  //uint16_t *pixelColor;  
  //tft.startWrite();
  //tft.setAddrWindow(0, 0, 16, 16);
  //tft.writePixels(&(pixels[k*w]), w * 1);
  
  tft.fillRect(0, 0, 128, 128, BLACK);
  //tft.fillRect(0, 0, 128, 128, colorList.cMAGENTA);
//tft.dmaWait();
  #if defined(ARDUINO_NRF52840_FEATHER)
    gfxBuffer.drawRGBBitmap(0,0,(uint16_t *)&(splashImage.pixel_data[0]),splashImage.width, splashImage.height);
    displayRGB565Bitmap(0, 0, gfxBuffer.getBuffer(), splashImage.width, splashImage.height);
  #else
  
  displayRGB565Bitmap(0, 0, (uint16_t *)&(splashImage.pixel_data[0]), splashImage.width, splashImage.height);
  #endif
}

void InitializeDisplay() {
  //Initialize FastLED
#if FAST_LED_ACTIVE
  FastLED.addLeds<LED_TYPE, LED_DATA_PIN, COLOR_ORDER>(&lameLED, 1);
  FastLED.setCorrection(TypicalLEDStrip);
  lameLED = CRGB(25, 10, 70);
  FastLED.show();
#endif

//For Feather 52840, we need to ensure that the D2 pin is enabled.
// Check the NFCPINS status and change it, if needed.
#if defined(ARDUINO_NRF52840_FEATHER)
  if (NRF_UICR->NFCPINS & 0x0001) {
    digitalWrite(PIN_LED1, HIGH);
    NRF_NVMC->CONFIG = 0x0001;
    while (!NRF_NVMC->READYNEXT) delay(50);
    NRF_UICR->NFCPINS = 0x0000;
    while (!NRF_NVMC->READYNEXT) delay(50);
    NRF_NVMC->CONFIG = 0x0000;
    digitalWrite(PIN_LED1, LOW);
  }
#endif

  //oledSPI.begin();
  tft.begin();
  tft.setRotation(3);  //3 sets the display top to be aligned with the Feather uUSB.
  //tft.fillRect(0, 0, 128, 128, BLACK);
  //tft.setCursor(0, 0);

  if (DISPLAY_SPLASH_IMAGE) {
    displaySplashScreen();
  } else {
    tft.fillRect(0, 0, 128, 128, BLACK);
    tft.setCursor(0, 0);
    tft.setTextColor(YELLOW, BLACK);
    tft.setTextSize(2);
    tft.println("Welcome to\n  TTtarm\n");
    tft.setTextColor(CYAN, BLACK);
    tft.setTextSize(2);
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

  tft.setCursor(110, 8);
  tft.setTextSize(1);
  tft.setTextColor(0x57fb, BLACK);
  tft.print(MCU_ID_STRING);

  //oledGraph(Adafruit_SSD1351 *tft,int X, int Y, int height, int width,float minValue, float maxValue);
  weaponGraph = oledGraph(&tft, 0, 27, 100, 128, 0.0f, 10.0f);
  captureGraph = oledGraph(&tft, 0, 27, 100, 128, 0.0f, 40.0f);
  //lameGraph = oledGraph(&tft, 0, 40, 127 - 40, 128, 0.0f, 20.0f);

  lineALabel = oledGraphLabel(&tft, 0, ABAR);
  lineABar = oledReverseHBarGraph(&tft, 0, ABAR + 17, barHeight, 128, 0.0f, 25.0f);
  lineBLabel = oledGraphLabel(&tft, 0, BBAR);
  lineBBar = oledReverseHBarGraph(&tft, 0, BBAR + 17, barHeight, 128, 0.0f, 25.0f);
  lineCLabel = oledGraphLabel(&tft, 0, CBAR);
  lineCBar = oledReverseHBarGraph(&tft, 0, CBAR + 17, barHeight, 128, 0.0f, 25.0f);
  float cableLimits[5] = { 0.0f, 5.0f, 15.0f, 20.0f, 999.9f };
  int cableColors[5] = { colorList.cGREEN, colorList.cYELLOW, colorList.cORANGE, colorList.cLIGHTRED, colorList.cLIGHTRED };
  lineABar.setBarColors(5, cableLimits, cableColors);
  lineALabel.setColors(5, cableLimits, cableColors);
  lineBBar.setBarColors(5, cableLimits, cableColors);
  lineBLabel.setColors(5, cableLimits, cableColors);
  lineCBar.setBarColors(5, cableLimits, cableColors);
  lineCLabel.setColors(5, cableLimits, cableColors);

  initializeProbeScreen();

  //prEpeeBar = oledBarGraph(&tft,)
  //prFoilBar = oledBarGraph(&tft,)
  //prACBar = oledBarGraph(&tft,)
  //prABar = oledBarGraph(&tft,)
  //prBBar = oledBarGraph(&tft,)
  //prCBar = oledBarGraph()

  int bars = 5;
  float lameVals[5]{ 0.0f, 5.0f, 10.0f, 15.0f, 20.0f };
  int lameColors[5]{ colorList.cGREEN, colorList.cYELLOW, colorList.cORANGE, colorList.cLIGHTRED, colorList.cLIGHTRED };
  //lameGraph.setHorizontalBarValues(5, lameVals, lameColors);
  lameBar = oledReverseHBarGraph(&tft, 0, 107, 20, 128, 0.0f, 20.0f);
  lameBar.setBarColors(5, lameVals, lameColors);
  lameLabel = oledGraphLabel(&tft, 0, 0);
  lameLabel.setColors(5, lameVals, lameColors);

  float wvals[4]{ 0.0f, 2.0f, 5.0f, 10.0f };
  int wcolors[4]{ colorList.cGREEN, colorList.cORANGE, colorList.cLIGHTRED, colorList.cLIGHTRED };
  weaponGraph.setHorizontalBarValues(4, wvals, wcolors);

  float cvals[4]{ 0.0f, 10.0f, 20.0f, 40.0f };
  int ccolors[4]{ colorList.cGREEN, colorList.cORANGE, colorList.cLIGHTRED, colorList.cLIGHTRED };
  captureGraph.setHorizontalBarValues(4, cvals, ccolors);

  //lameLED=CRGB::Black;
  //FastLED.show();
}

void dimOLEDDisplay() {
  //static bool alreadyDimmed=false;
  //Fill the rest of the display with black leaving only the top bar.
  tft.fillRect(0, 17, 128, 128 - 17, BLACK);
#if FAST_LED_ACTIVE
  lameLED = CRGB::Black;
  FastLED.show();
#endif
}

void initializeProbeScreen() {
  float limits[3] = { 0.0f, 5.0f, 10.0f };
  int colors[3] = { colorList.cGREEN, colorList.cYELLOW, colorList.cLIGHTRED };
  int barSpace = 19;
  int barYStart = 19;
  int barLeftEdge = 97;
  int barHeight = 16;
  prEpeeBar = oledReverseHBarGraph(&tft, 0, barYStart + barSpace, barHeight, 128, 0.0f, 15.0f);
  prEpeeLabel = oledGraphLabel(&tft, 0, barYStart);
  prEpeeBar.setBarColors(3, limits, colors);
  prEpeeLabel.setColors(3, limits, colors);
  prFoilBar = oledReverseHBarGraph(&tft, barLeftEdge, barYStart + barSpace, barHeight, 128 - barLeftEdge, 0.0f, 20.0f);
  prFoilLabel = oledGraphLabel(&tft, 0, barYStart + barSpace);
  prFoilBar.setBarColors(3, limits, colors);
  prFoilLabel.setColors(3, limits, colors);
  prACBar = oledReverseHBarGraph(&tft, barLeftEdge, barYStart + barSpace * 2, barHeight, 128 - barLeftEdge, 0.0f, 15.0f);
  prACLabel = oledGraphLabel(&tft, 0, barYStart + barSpace * 2);
  prACBar.setBarColors(3, limits, colors);
  prACLabel.setColors(3, limits, colors);
  prABar = oledReverseHBarGraph(&tft, barLeftEdge, barYStart + barSpace * 3, barHeight, 128 - barLeftEdge, 0.0f, 15.0f);
  prALabel = oledGraphLabel(&tft, 0, barYStart + barSpace * 3);
  prABar.setBarColors(3, limits, colors);
  prALabel.setColors(3, limits, colors);
  prBBar = oledReverseHBarGraph(&tft, barLeftEdge, barYStart + barSpace * 4, barHeight, 128 - barLeftEdge, 0.0f, 15.0f);
  prBLabel = oledGraphLabel(&tft, 0, barYStart + barSpace * 4);
  prBBar.setBarColors(3, limits, colors);
  prBLabel.setColors(3, limits, colors);
  prCBar = oledReverseHBarGraph(&tft, barLeftEdge, barYStart + barSpace * 5, barHeight, 128 - barLeftEdge, 0.0f, 15.0f);
  prCLabel = oledGraphLabel(&tft, 0, barYStart + barSpace * 5);
  prCBar.setBarColors(3, limits, colors);
  prCLabel.setColors(3, limits, colors);
}

void displayBatteryStatus() {
  char voltString[5] = "\0";
  char percString[5] = "\0";
  int battPercent = 75;
  int rectColor = WHITE;
  int fillColor = WHITE;
  byte barW;
  bool battActive = false;
  static const byte BattH = 7;
  static const byte BattW = 10;
  //static const byte BattX0=113-BattW-1;
  static const byte BattX0 = 128 - BattW - 1;
  static const byte BattY0 = 1;

  if (batteryVoltage > 2.5) {
    battPercent = int((batteryVoltage - 3.3) / (4.1 - 3.3) * 100);
    battActive = true;
  } else {
    return;
  }
  tft.setTextSize(1);

  if (!battActive) {
    tft.fillRect(BattX0 - 1, BattY0 - 1, 128 - BattX0 + 1, BattY0 + 1, BLACK);
    return;
  }

  if (battPercent <= 15) {
    fillColor = RED;
    rectColor = RED;
  }
  if (battPercent >= 15) {
    fillColor = YELLOW;
  }
  if (battPercent >= 50) {
    fillColor = GREEN;
  }
  barW = min(BattW, (int)battPercent / BattW);
  if (barW < 0) {
    barW = 0;
  }

  tft.drawRect(BattX0 - 1, BattY0 - 1, BattW + 2, BattH + 2, rectColor);
  tft.drawFastVLine(BattX0 - 2, BattY0 + 1, 5, rectColor);
  tft.fillRect(BattX0 + (BattW - barW), BattY0, barW, BattH, fillColor);
  tft.setCursor(BattX0 + BattW + 3, BattY0);
  //tft.setTextSize(1);
  //tft.setTextColor(CYAN,BLACK);
  //tft.print(battPercent);
}
/*int floatTo10xInt(float g) {
  if (g < 0.0) g = 0.0;
  return ((int) (g * 10.0 + .5));
}*/
float gv(const char *s) {
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
  if (s == "AA") return cableState.ohm_AA;
  if (s == "BB") return cableState.ohm_BB;
  if (s == "CC") return cableState.ohm_CC;

  for (int k = 0; k < NUM_ADC_SCAN_CHANNELS; k++) {
    if ((s[0] == ChanArray[k].ch_label[1]) && (s[1] == ChanArray[k].ch_label[2])) {
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
  return -999;
}


void labelTitle(const char *s, int color) {
  const int LBL_MAX = 15;
  static char lblBuffer[LBL_MAX + 1];
  static int oldColor = BLACK;
  //if (s == oldFault) return; //no change
  if ((strcmp(lblBuffer, s) == 0) && (oldColor == color)) return;
  oldColor = color;
  if (strlen(s) <= LBL_MAX) {
    strcpy(lblBuffer, s);
  } else {
    strncpy(lblBuffer, s, LBL_MAX);
    lblBuffer[LBL_MAX] = '\0';
  }
  tft.setTextSize(2);
  tft.fillRect(0, 0, 115, 20, BLACK);  //clear label area
  tft.setCursor(2, 2);
  tft.setTextColor(color, BLACK);
  tft.println(lblBuffer);
}

static int oldA = 0, oldB = 0, oldC = 0;

void graph1(const char *s) {
  lineABar.updateGraph(gv(s));
  lineALabel.printLabel(s, gv(s));
  //barGraph(ABAR, 8, gv(s), oldA, s);
}
void graph2(const char *s) {
  lineBBar.updateGraph(gv(s));
  lineBLabel.printLabel(s, gv(s));
  //barGraph(BBAR, 8, gv(s), oldB, s);
}
void graph3(const char *s) {
  lineCBar.updateGraph(gv(s));
  lineCLabel.printLabel(s, gv(s));
  //barGraph(CBAR, 8, gv(s), oldC, s);
}

void updateOLED(TestBoxModes Mode) {
  static int i = 0, val, oldMode = 'z';
  //static bool oledEnabled = true;
  enum lastConnected { first,
                       none,
                       epee,
                       foil,
                       shorted };
  enum displayStates { disp_off,
                       disp_idle,
                       disp_cable,
                       disp_clip,
                       disp_lame,
                       disp_probe,
                       disp_wpnR,
                       disp_wpnTest,
                       disp_capture,
                       disp_unk,
                       disp_error };
  static lastConnected lastConnection;
  lastConnected newConnection;
  //static lastConnected lastConnectionSeen;
  static displayStates currentDisplayState = disp_unk;
  static displayStates oldDisplayState = disp_unk;
  static int foilIndicator, epeeIndicator, foilInterIndicator, epeeInterIndicator;
  static unsigned long sTime;                //start time of last seen foil/epee connection
  const unsigned long dispChangeHold = 500;  //Don't allow display to switch mode if this is set
  static long dispHoldTime = 500;            //Don't allow display to switch mode if this is set
  static unsigned long tDisplaySwitch = 0;   //Last time the display mode changed
  static unsigned long lastCapture = 0;
  bool inter;
  static long tIdleLEDOn = 0;
  //static long tIdleLEDOn

  //Serial.println(Mode);

  if ((millis() - tLastActive) > tOledOff) {
    tft.enableDisplay(false);
    //oledEnabled = false;
    currentDisplayState = disp_off;
    //return;
  } else {
    if (currentDisplayState == disp_off) {
      tft.enableDisplay(true);
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
      case PROBE:
        currentDisplayState = disp_probe;
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
  if ((oldDisplayState != currentDisplayState) && ((millis() - tDisplaySwitch) > dispChangeHold)) {
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
      case disp_probe:
        createProbeDisplay();
        break;
      case disp_cable:
        //tft.setTextColor(YELLOW, BLACK); tft.setTextSize(2);
        //tft.print("Cable");
        labelTitle("Cable", YELLOW);
        oldA = oldB = oldC = 320;
        totalTime = 0ul;
        timeSamples = 0ul;
        tft.drawFastVLine(127, 31, 96, CYAN);  //0.0f line
        tft.drawFastVLine(102, 31, 96, CYAN);  //5.0f line
        tft.drawFastVLine(77, 31, 96, CYAN);   //10.0f line
        tft.drawFastVLine(26, 31, 96, CYAN);   //20.f line
        //tft.fillRect(0, X + 16, 128, H, BLACK);
        //Height of bar is 8, height of textsize(2) is 14, 1 pix between text and bar, 1 pix between text/bar and line
        lineABar.resetGraph();
        lineBBar.resetGraph();
        lineCBar.resetGraph();
        //tft.fillRect(0, ABAR - 1, 128, 26, BLACK);
        //tft.fillRect(0, BBAR - 1, 128, 26, BLACK);
        //tft.fillRect(0, CBAR - 1, 128, 26, BLACK);
        tft.setTextSize(1);
        tft.setTextColor(WHITE);
        tft.setCursor(26 - 14, 120);
        tft.print("20");
        tft.setCursor(77 - 14, 120);
        tft.print("10");
        tft.setCursor(102 - 8, 120);
        tft.print("5");
        tft.setCursor(127 - 8, 120);
        tft.print("0");
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
        labelTitle("Idle", BLUE);
        //tft.print("Idle");
        //tft.fillScreen(BLACK);
    }
    oldDisplayState = currentDisplayState;
    displayBatteryStatus();
    tDisplaySwitch = millis();
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

    case disp_probe:
      updateProbeDisplay();
      break;

    case disp_capture:
      {
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

        if (millis() > (dispCaptureHoldTime + lastCapture)) {
          if (armed == false) {
            FoilADC.hsBuffer.ResetTrigger();
            EpeeADC.hsBuffer.ResetTrigger();
            //StartADC();
            labelTitle("Armed", GREEN);
            armed = true;
          }
          if (FoilADC.hsBuffer.CaptureDone() || EpeeADC.hsBuffer.CaptureDone()) {
            lastCapture = millis();
            armed = false;
            captureGraph.resetGraph();
            for (int k = 0; k < ADC_CAPTURE_LEN; k++) {
              captureGraph.updateGraph((ADC_CaptureBuffer[k] - trimVal) * FoilADC.LOW_GAIN);
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

      if ((weaponState.foilOn) && (weaponState.epeeOn)) {  //short
        newConnection = shorted;
        dispHoldTime = millis() + 5000ul;
        //lastConnection = shorted;
      }

      if ((millis() - sTime) > 5000ul) {
        newConnection = none;
      }

      if (newConnection != lastConnection) {
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
          tft.fillRect(0, 27, 128, 100, BLACK);
          oldA = oldB = 320;
          //oldA=weaponState.ohm10xEpee;
          //oldB=weaponState.ohm10xFoil;
          //barGraph(BBAR, 8, weaponState.ohm10xEpee, oldA, "AB");
          //barGraph(CBAR, 8, weaponState.ohm10xFoil, oldB, "BC");
          lineBLabel.printLabel("AB", weaponState.ohm_Epee, true, colorList.cMAGENTA);
          lineBBar.updateGraph(weaponState.ohm_Epee);
          lineCLabel.printLabel("BC", weaponState.ohm_Foil, true, colorList.cMAGENTA);
          lineCBar.updateGraph(weaponState.ohm_Foil);
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
          lineBLabel.printLabel("AB", weaponState.ohm_Epee, true, colorList.cMAGENTA);
          lineBBar.updateGraph(weaponState.ohm_Epee);
          lineCLabel.printLabel("BC", weaponState.ohm_Foil, true, colorList.cMAGENTA);
          lineCBar.updateGraph(weaponState.ohm_Foil);
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
        digitalWrite(LED2_PIN, HIGH);
        delay(10);
        digitalWrite(LED2_PIN, LOW);
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
  } else if (foilIndicator != BLACK) {
    tft.fillRect(0, 19, 20, 7, BLACK);
    foilIndicator = BLACK;
  }
  if (weaponState.foilInterOn) {
    if (foilInterIndicator != YELLOW) {
      tft.fillRect(20, 19, 20, 7, YELLOW);
      foilInterIndicator = YELLOW;
    }
  } else if (foilInterIndicator != BLACK) {
    tft.fillRect(20, 19, 20, 7, BLACK);
    foilInterIndicator = BLACK;
  }
  if (weaponState.epeeOn) {
    if (epeeIndicator != GREEN) {
      tft.fillRect(70, 19, 20, 7, GREEN);
      epeeIndicator = GREEN;
    }
  } else if (epeeIndicator != BLACK) {
    tft.fillRect(70, 19, 20, 7, BLACK);
    epeeIndicator = BLACK;
  }
  if (weaponState.epeeInterOn) {
    if (epeeInterIndicator != YELLOW) {
      tft.fillRect(90, 19, 20, 7, YELLOW);
      epeeInterIndicator = YELLOW;
    }
  } else if (epeeInterIndicator != BLACK) {
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
  } else if (foilIndicator != BLACK) {
    tft.fillRect(0, 28, 50, 50, BLACK);
    foilIndicator = BLACK;
  }
  if (weaponState.foilInterOn) {
    if (foilInterIndicator != YELLOW) {
      tft.fillRect(0, 80, 50, 40, YELLOW);
      foilInterIndicator = YELLOW;
    }
  } else if (foilInterIndicator != BLACK) {
    tft.fillRect(0, 80, 50, 40, BLACK);
    foilInterIndicator = BLACK;
  }
  if (weaponState.epeeOn) {
    if (epeeIndicator != GREEN) {
      tft.fillRect(70, 28, 128 - 50, 50, GREEN);
      epeeIndicator = GREEN;
    }
  } else if (epeeIndicator != BLACK) {
    tft.fillRect(70, 28, 128 - 50, 50, BLACK);
    epeeIndicator = BLACK;
  }
  if (weaponState.epeeInterOn) {
    if (epeeInterIndicator != YELLOW) {
      tft.fillRect(70, 80, 128 - 50, 40, YELLOW);
      epeeInterIndicator = YELLOW;
    }
  } else if (epeeInterIndicator != BLACK) {
    tft.fillRect(70, 80, 128 - 50, 40, BLACK);
    epeeInterIndicator = BLACK;
  }
}

void updateDisplayCableMode() {
  //static char displayMode = 'c';
  const uint16_t faultCheck = ~((1 << BITAA) | (1 << BITBB) | (1 << BITCC));
  bool ABcross = CheckCableStatusByte((1 << BITAB));
  bool ACcross = CheckCableStatusByte((1 << BITAC));
  bool BAcross = CheckCableStatusByte((1 << BITBA));
  bool BCcross = CheckCableStatusByte((1 << BITBC));
  bool CAcross = CheckCableStatusByte((1 << BITCA));
  bool CBcross = CheckCableStatusByte((1 << BITCB));
  bool ABshort = (CheckCableStatusByte((1 << BITAB)) || CheckCableStatusByte((1 << BITBA))) && ((!CheckCableStatusByte((1 << BITAA))) || (!CheckCableStatusByte((1 << BITBB))));
  bool BCshort = (CheckCableStatusByte((1 << BITBC)) || CheckCableStatusByte((1 << BITCB))) && ((!CheckCableStatusByte((1 << BITBB))) || (!CheckCableStatusByte((1 << BITCC))));
  bool ACshort = (CheckCableStatusByte((1 << BITAC)) || CheckCableStatusByte((1 << BITCA))) && ((!CheckCableStatusByte((1 << BITAA))) || (!CheckCableStatusByte((1 << BITCC))));

  //tft.setTextSize(2);

  if ((cableState.statusByte & faultCheck) == 0) {  //Only faults are high resistance
    if (cableState.statusByte == 0) {
      labelTitle("Cable OK", GREEN);
    } else {
      labelTitle("Cable", YELLOW);
    }
    graph1("AA");
    graph2("BB");
    graph3("CC");
    return;
  }

  if (ABshort) {    //AB short
    if (BCshort) {  //if AB and BC then AC has to be shorted
      //A - B - C short
      labelTitle("Short ABC", RED);
      graph1("AB");
      graph2("BC");
      graph3("AC");
    } else {
      //AB short, CC okay or open
      labelTitle("Short AB", RED);
      graph1("AB");
      graph2("BB");
      graph3("CC");
    }
  } else if (BCshort) {
    //BC and not AB (and therefore not AC), AA okay or open
    labelTitle("Short BC", RED);
    graph1("AA");
    graph2("BC");
    graph3("CC");
  } else if (ACshort) {
    //AC and not AB, and therefore not BC, BB okay or open
    labelTitle("Short AC", RED);
    graph1("AC");
    graph2("BB");
    graph3("CC");
  } else if (ABcross) {  //no shorts, check crosses
    if (BAcross) {
      //AB cross, CC okay or open
      labelTitle("AB Cross", RED);
      graph1("AB");
      graph2("BA");
      graph3("CC");
    } else if (CAcross) {
      //A->B C->A, could be B->C or open
      labelTitle("ABC Cross", RED);
      graph1("AB");
      graph2("BC");
      graph3("CA");
    } else {
      //AB cross, don't know what happened to BA
      labelTitle("AB Cross", RED);
      graph1("AB");
      graph2("BB");  //always going to be open
      graph3("CC");
    }
  } else if (BCcross) {
    //we could test CB, and know it was a real BC cross, but if that failed, something has to be open
    labelTitle("BC Cross", RED);
    graph1("AA");
    graph2("BC");
    graph3("CB");
  } else if (ACcross) {
    if (CAcross) {
      //AC cross, BB good or open
      labelTitle("AC Cross", RED);
      graph1("AC");
      graph2("BB");
      graph3("CA");
    } else if (BAcross) {
      //A->C, B->A, C->B
      labelTitle("ACB Cross", RED);
      graph1("AC");
      graph2("BA");
      graph3("CB");
    } else {
      //something is open
      labelTitle("AC Cross", RED);
      graph1("AC");
      graph2("BB");
      graph3("CA");
    }
  } else if (BAcross) {  //something is open
    //BA but not AB
    labelTitle("BA Cross", RED);
    graph1("AB");
    graph2("BA");
    graph3("CC");
  } else if (CBcross) {
    //CB but not BC
    labelTitle("CB Cross", RED);
    graph1("AA");
    graph2("BC");
    graph3("CB");
  } else if (CAcross) {
    //CA but not AC
    labelTitle("CA Cross", RED);
    graph1("AC");
    graph2("BB");
    graph3("CA");
  }
}

void createWeaponDisplay() {
  //tft.setCursor(2, 2);
  tft.setTextColor(colorList.cYELLOW, colorList.cBLACK);
  tft.setTextSize(2);
  //tft.print("Weapon");
  labelTitle("Weapon", colorList.cYELLOW);
  weaponGraph.resetGraph();
}

void createProbeDisplay() {
  tft.fillRect(0, 27, 128, 100, colorList.cBLACK);
  labelTitle("Probe", colorList.cMAGENTA);
  //prEpeeLabel.printLabel("Ep", OPEN_CIRCUIT_VALUE);
  //prEpeeBar.updateGraph(OPEN_CIRCUIT_VALUE);
  //prFoilLabel.printLabel("Fl", OPEN_CIRCUIT_VALUE);
  //prFoilBar.updateGraph(OPEN_CIRCUIT_VALUE);
  if (TTArmMOSFETWeaponAC) {
    //prACLabel.printLabel("GND", OPEN_CIRCUIT_VALUE);
    //prACBar.updateGraph(OPEN_CIRCUIT_VALUE);
  }
  prALabel.printLabel("PrA", OPEN_CIRCUIT_VALUE);
  prABar.updateGraph(OPEN_CIRCUIT_VALUE);
  prBLabel.printLabel("PrB", OPEN_CIRCUIT_VALUE);
  prBBar.updateGraph(OPEN_CIRCUIT_VALUE);
  prCLabel.printLabel("PrC", OPEN_CIRCUIT_VALUE);
  prCBar.updateGraph(OPEN_CIRCUIT_VALUE);
}

void updateProbeDisplay() {
  enum connectionType { none,
                        epee,
                        foil,
                        shorted };
  static connectionType lastConnected = none;
  connectionType newConnection = lastConnected;
  static long tLastChanged = 0;

  if (probeData.ohm_Epee < OPEN_CIRCUIT_VALUE) {
    newConnection = epee;
  }
  if (probeData.ohm_Foil < OPEN_CIRCUIT_VALUE) {
    newConnection = foil;
  }
  //If we were shorted, hold that configuration
  if (lastConnected == shorted && ((millis() - tLastChanged) < tIdleModeOn)) {
    newConnection = shorted;
  }
  if (probeData.ohm_Epee < OPEN_CIRCUIT_VALUE && probeData.ohm_Foil < OPEN_CIRCUIT_VALUE) {
    newConnection = shorted;
  }
  if (probeData.ohm_WpnAC < OPEN_CIRCUIT_VALUE) {
    newConnection = shorted;
  }
  if (newConnection != lastConnected) {
    prEpeeBar.resetGraph();
    lastConnected = newConnection;
    prEpeeLabel.clearLabel();
    prFoilLabel.clearLabel();
    prACLabel.clearLabel();
    tLastChanged = millis();
  }
  switch (lastConnected) {
    case epee:
      prEpeeLabel.printLabel("Epee", probeData.ohm_Epee);
      prEpeeBar.updateGraph(probeData.ohm_Epee);
      break;
    case foil:
      prEpeeLabel.printLabel("Foil", probeData.ohm_Foil);
      prEpeeBar.updateGraph(probeData.ohm_Foil);
      break;
    case shorted:
      prEpeeLabel.printLabel("Epee", probeData.ohm_Epee);
      prFoilLabel.printLabel("Foil", probeData.ohm_Foil);
      if (TTArmMOSFETWeaponAC) {
        prACLabel.printLabel("GND", probeData.ohm_WpnAC);
        //prACBar.updateGraph(probeData.ohm_WpnAC);
      }
  }
  prALabel.printLabel("PrA", probeData.ohm_APr);
  prABar.updateGraph(probeData.ohm_APr);
  prBLabel.printLabel("PrB", probeData.ohm_BPr);
  prBBar.updateGraph(probeData.ohm_BPr);
  prCLabel.printLabel("PrC", probeData.ohm_CPr);
  prCBar.updateGraph(probeData.ohm_CPr);
}
// barGraph(X, H, oldVal, newVal, oldVal)
void createLameDisplay() {
  labelTitle("", YELLOW);
  //lameGraph.resetGraph();
}

void updateLameDisplay() {
  float lameOhms;
  int txtColor = colorList.cBLACK;
  char msg[5];

  if (cableState.ohm_Lame < MAX_LAME_RESISTANCE) {
    lameOhms = cableState.ohm_Lame;
  } else {
    lameOhms = OPEN_CIRCUIT_VALUE;
  }

  lameBar.updateGraph(lameOhms);
  lameLabel.printLabel("Lame", lameOhms);
  txtColor = lameBar.getBarColor();

  gfxBuffer.setFont(&FreeSansBold24pt7b);
  gfxBuffer.setTextSize(2);

  gfxBuffer.setTextColor(txtColor);
  gfxBuffer.fillRect(0, 0, 128, LAME_DIGIT_HEIGHT, colorList.cBLACK);

  int intOhms = floor(lameOhms);
  int deciOhms = floor(10 * (lameOhms - intOhms));
  if (lameOhms < 10) {
    gfxBuffer.setCursor(0, LAME_DIGIT_HEIGHT - 3);
    gfxBuffer.print(intOhms);
    gfxBuffer.print(".");
    gfxBuffer.setCursor(72, LAME_DIGIT_HEIGHT - 3);
    gfxBuffer.print(deciOhms);
  } else if (lameOhms == OPEN_CIRCUIT_VALUE) {
    gfxBuffer.setTextColor(colorList.cBLUE);
    gfxBuffer.setCursor(32, LAME_DIGIT_HEIGHT - 3);
    gfxBuffer.print("--");
  } else {
    gfxBuffer.setCursor(12, LAME_DIGIT_HEIGHT - 3);
    gfxBuffer.print(intOhms);
  }

  //tft.drawBitmap(0, 32, lameTxtCanvas.getBuffer(), 128, 64, txtColor, colorList.cBLACK);
  displayRGB565Bitmap(0, 27, gfxBuffer.getBuffer(), 128, LAME_DIGIT_HEIGHT);

  tft.setTextColor(txtColor);

  //lameGraph.updateGraph(lameOhms);
#if FAST_LED_ACTIVE
  updateLED(lameOhms);
#endif
}

#if FAST_LED_ACTIVE
void updateLED(float value) {
  const float lameVals[5]{ -1.0f, 5.0f, 10.0f, 15.0f, 20.0f };
  const CRGB lameColors[5]{ CRGB::Green, CRGB::Yellow, CRGB::Orange, CRGB::OrangeRed, CRGB::DarkRed };
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
  lameLED.nscale8_video(64);  //50% brightness reduction
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
  const long tHeartBeatInterval = 5000;  //Time in ms
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

  tempString1[0] = '\0';  //Reset the temp String
  tempString2[0] = '\0';  //Reset the temp String
  //tempString3[0]='\0';  //Reset the temp String
  outputString[0] = '\0';  //Reset the output string

  if ((t_now - tHeartBeat) > tHeartBeatInterval) {
    float dt = (float(t_now - t_last_upd) * 1.0e-3);
    EffSampleRate = float(numSamples) / dt;

    dtostrf(EffSampleRate, 5, 0, tempString1);  //Effective sample rate
    dtostrf(batteryVoltage, 5, 2, tempString2);
    snprintf(outputString, bufferSize, "t=%ld, N=%ld, %sHz, Bat=%sV\r\n", t_now, numSamples, tempString1, tempString2);
    Serial.write(outputString);
    tHeartBeat = millis();
    t_last_upd = millis();
    numSamples = 0;
  }

  outputString[0] = '\0';  //Reset the outputString
  switch (Mode) {
    case CABLE:
      //Print the status byte
      dtostrf(cableState.ohm_AA, 5, 2, tempString1);
      dtostrf(cableState.ohm_AAMax, 5, 2, tempString2);

      snprintf(outputString, bufferSize, "C1,%ld,%x,A,%s,%s,B,", t_now, cableState.statusByte, tempString1, tempString2);
      dtostrf(cableState.ohm_BB, 5, 2, tempString1);
      dtostrf(cableState.ohm_BBMax, 5, 2, tempString2);
      strncat(outputString, tempString1, bufferSize);
      strncat(outputString, ",", bufferSize);
      strncat(outputString, tempString2, bufferSize);
      dtostrf(cableState.ohm_CC, 5, 2, tempString1);
      dtostrf(cableState.ohm_CCMax, 5, 2, tempString2);
      strncat(outputString, ",C,", bufferSize);
      strncat(outputString, tempString1, bufferSize);
      strncat(outputString, ",", bufferSize);
      strncat(outputString, tempString2, bufferSize);
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
      strncat(outputString, tempString1, bufferSize);
      strncat(outputString, ",F,", bufferSize);
      strncat(outputString, tempString2, bufferSize);
      strncat(outputString, "\r\n", bufferSize);
      break;
  }

  Serial.write(outputString);  //Serial.println(totalTime / timeSamples);

  /*if (SerialBT.available()){
    SerialBT.print(outputString);
    }*/
}
