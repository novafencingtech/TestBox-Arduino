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
#include <Adafruit_SSD1351.h>
#include <Adafruit_SPITFT.h>
#include <SPI.h>

// Option 1: use any pins but a little slower
//Adafruit_SSD1351 tft = Adafruit_SSD1351(SCREEN_WIDTH, SCREEN_HEIGHT, CS_PIN, DC_PIN, MOSI_PIN, SCLK_PIN, RST_PIN);

// Option 2: must use the hardware SPI pins
// (for UNO thats sclk = 13 and sid = 11) and pin 10 must be
// an output. This is much faster - also required if you want
// to use the microSD card (see the image drawing example)
Adafruit_SSD1351 tft = Adafruit_SSD1351(SCREEN_WIDTH, SCREEN_HEIGHT, &SPI, CS_PIN, DC_PIN);
bool CheckCableStatusByte(uint16_t errorCheck) {
  return ((cableState.statusByte & errorCheck) == errorCheck);
}
int cableOrangeThresh = 150; //threshold from red to organge for cables in tenths of an ohm, 150=15.0 ohms
int cableGreenThresh = 50;   //threshaold from orange to green for cables in tenths of an ohm, 50=5.0 ohms
int weaponOrangeThresh = 50; //threshold from red to organge for cables in tenths of an ohm, 50=5.0 ohms
int weaponGreenThresh = 20;   //threshaold from orange to green for cables in tenths of an ohm, 20=2.0 ohms
int slowShift = 0;         //counts refreshs to avoid fast scale shifts
int oldScale = 200;        //max value of full graph for last refresh
//int oldVal = 0;            //prior value (global so we don't have to pass by ref everywhere)
#define SLOWCOUNT 8          //how many refreshes before scale can change
long unsigned int startTime, endTime, totalTime, timeSamples;


int scaleWidth(int val) {  //tenths don't divide by 4 very well
  int sVal;                //so this expressly sets the break points
  sVal = (val / 10) * 4;
  switch (val % 10) {
    case 0:
    case 1: return sVal; break; //x.0 or x.1 is x
    case 2:
    case 3: return (sVal + 1); break;  //x.2 or x.3 is x 1/4
    case 4:
    case 5:
    case 6: return (sVal + 2); break;  //x.4, x.5, x.6 is x 1/2
    case 7:
    case 8: return (sVal + 3); break;  //x7, x.8 is x 3/4
    case 9: return (sVal + 4); break;  //x.9 is x+1
  }
}

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

// barGraph(X, H, oldVal, newVal, oldVal)
//  X = top lime
//  H = height of bar
//  oldVal = last value in tenths of an ohm
//  newVal = current value
// with 128 pixel wide screen, show 32 ohms * 4 pixels/ohm
void barGraph(int X, int H, int newVal, int &oldVal, char *conn) {
  int difVal, rOld, rNew, bc = ORANGE; bool newColor = false;
  //Serial.print(conn); Serial.print("||New value = ");Serial.println(newVal);
  if (newVal > cableOrangeThresh) {  //if the new value is greater than the orange threshold
    bc = RED; //then its a red bar
    if (oldVal <= cableOrangeThresh) newColor = true;  //did we change color?
  }
  else if (newVal <= cableGreenThresh) {  //if new value is less than the green threshold
    bc = GREEN;  //then its a green bar
    if (oldVal > cableGreenThresh) newColor = true;  //did we change color
  }
  else if ((oldVal < cableOrangeThresh) || (oldVal >= cableGreenThresh)) newColor = true;  //orange, but not if the prior value wasn't orange
  printVal(X, 0, bc, conn, newVal);
  if (newVal > 320) {
    tft.fillRect(0, X + 16, 128, H, BLACK);
    oldVal = newVal;
    return; // no bar if beyond 32 ohms
  }
  rOld = 320 - oldVal; //row of prior value
  rNew = 320 - newVal;  //row of new value
  difVal = rNew - rOld;  //length of bar (could be negative
  if (difVal > 0)  {//increase bar graph length
    if (newColor) tft.fillRect(0, X + 16, scaleWidth(rNew), H, bc); //if color change, draw complete bar in new color
    else tft.fillRect(scaleWidth(rOld), X + 16, scaleWidth(rNew) - scaleWidth(rOld), H, bc); //just draw extension
  }
  else if (difVal < 0) { //decrease bar graph length
    if (newColor) tft.fillRect(0, X + 16, scaleWidth(rNew), H, bc);  //if color change, draw complete bar
    tft.fillRect(scaleWidth(rNew), X + 16, -(scaleWidth(rNew) - scaleWidth(rOld)), H, BLACK); //black out right part of prior bar
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
/*
  void drawColumn(int col, int val) {
  //Assumes we have only one graph EVER
  static int oldVal = 0;
  static int oldRow = 0;
  //0-9.9 ohm, rows 27-127
  int valRow = 127 - min(val, 99); //convert ohms in tenths to row
  //int oldRow = 127 - min(oldVal, 99); //same with old val
  int topRow, botRow, orangeRow, greenRow;
  //sort ends so we're always drawing from top to bottom
  if (valRow > oldRow) {
    topRow = oldRow;
    botRow = valRow;
  } else {
    topRow = valRow;
    botRow = oldRow;
  }
  //  if (weaponState.ohm10xFoil) > 18) { Serial.print("vline ");Serial.print(floatTo10xInt(cableState.ohm_AA));Serial.print("=");Serial.print(topRow);Serial.print(",");Serial.println(botRow);}
  orangeRow = 127 - weaponOrangeThresh; //first line of orange
  greenRow = 127 - weaponGreenThresh; //first line of green
  tft.drawPixel(col, 27, WHITE); //draw top scale (9.9 ohms), could be erased later
  drawVLine(col, 28, topRow - 1, BLACK); //clear out above topRow
  if (topRow > 77) tft.drawPixel(col, 77, WHITE); //5 ohms = 50 rows, 127-50 = 77
  if (topRow > 107) tft.drawPixel(col, 107, WHITE); //2 ohms = 20 rows, 127-20 = 107
  if (topRow < orangeRow) { //some red
    drawVLine(col, topRow, min(botRow, orangeRow - 1), RED);
    if (botRow >= orangeRow) {
      if (botRow >= greenRow) { //extends to green
        drawVLine(col, orangeRow, greenRow - 1, ORANGE); //complete orange section
        drawVLine(col, greenRow, botRow, GREEN);
      } else { //ends in orange
        drawVLine(col, orangeRow, botRow, ORANGE);
      }
    }
  } else {  //no red
    if (topRow < greenRow) { //some orange
      drawVLine(col, topRow, min(botRow, greenRow - 1), ORANGE);
      if (botRow > greenRow) { //extends to green
        drawVLine(col, greenRow, botRow, GREEN);
      }
    } else { //green only
      drawVLine(col, topRow, botRow, GREEN);
    }
  }
  if (botRow < 127) { //fill black to bottom
    drawVLine(col, botRow + 1, 127, BLACK);
    if (botRow < 77) tft.drawPixel(col, 77, WHITE); //restore 5 ohm line
    if (botRow < 107) tft.drawPixel(col, 107, WHITE); //restore 2 ohm line
    tft.drawPixel(col, 127, WHITE); //restore 0 ohm line
  }
  if (col < 127) {
    int icol = col + 1; int irow = max(valRow - 1, 28);
    //    if (val>18) {Serial.print(icol);Serial.print("=");;Serial.print(irow);Serial.println(",CYAN");}
    tft.drawFastVLine(icol, irow, 2, CYAN);
  }  //show where we are as 2x1 cyan line at col+1
  oldRow = valRow;
  oldVal = val;
  }*/

//Draw a static line graph from an array
//  rData = array with resistance values in tenths of an ohm
//  oThresh = resistance of Orange threshold im tenths of an ohm
//  gThresh = resistance of Green threshold in tenths of an ohm
/* void LineGraph(int rData[127]) {
  int i, val, maxVal, vMax, vRatio, yVal, oldY, startColor, endColor, oY, gY;
  maxVal = 0;
  for (i = 0; i < 128; i++) if ((rData[i] > val) && (rData[i] != 990)) maxVal = rData[i]; //find biggest value
  // We reserve 28 rows for text, leaving 100 rows for graph
  // Graphs are scaled to the max resistance in the array
  //    Max R  Precision (1 pixel)
  //      10    .1 ohm
  //      20   .25 ohm
  //     100    1 ohm
  //     500    5 ohm
  startTime = micros();
  vMax = 100;  // assume 10 ohm scale
  if (maxVal > 100) vMax = 500;  //possible 20 ohm scale
  if (maxVal > 200) vMax = 1000;  //possible 100 scale
  if (maxVal > 1000) vMax = 5000;  //nope, it's 500 ohm scale
  if (vMax != oldScale) {  //don't change scale fast
    if (slowShift++ < SLOWCOUNT) {
      vMax = oldScale;  //use the old scale for a while
    }
    else {  //new scale
      oldScale = vMax;  //change scale
      slowShift = 0;  //restart slow filter
    }
  } else slowShift = 0;  //stay in your lane
  vRatio = vMax / 100;  //tenths of an ohm per row
  oY = 100 - (weaponOrangeThresh / vRatio);  //calculate row # of orange threshold
  gY = 100 - (weaponGreenThresh / vRatio);  // same for green threshold
  //oldVal = rData[0]; //force first column to be a point
  for (i = 0; i < 128; i++) {
    val = rData[i];
    if (val == 990) val = maxVal; //if not connected, use max value in array
    drawColumn(i, val);
    //oldVal = val;
  }
  endTime = micros();
  if (startTime < endTime) {
    totalTime = endTime - startTime;
    timeSamples = 1ul;
  };
  }
*/

void displaySplashScreen() {
  tft.fillRect(0, 0, 128, 128, BLACK);
  tft.drawRGBBitmap(0,0,(uint16_t*) &(splashImage.pixel_data[0]),splashImage.width,splashImage.height);
}

void InitializeDisplay()
{
  tft.begin();
  tft.setRotation(3);  //3 sets the display top to be aligned with the Feather uUSB.
  tft.fillRect(0, 0, 128, 128, BLACK);
  tft.setCursor(0, 0);
  if (DISPLAY_SPLASH_IMAGE) {
    displaySplashScreen();
  } else {
    tft.setTextColor(YELLOW, BLACK); tft.setTextSize(2);
    tft.println("Welcome to\n  TTtarm\n");
    tft.setTextColor(CYAN, BLACK); tft.setTextSize(2);
    tft.println("  G Allen \n     &\n  B Rosen");
  }
  
  //oledGraph(Adafruit_SSD1351 *tft,int X, int Y, int height, int width,float minValue, float maxValue);
  weaponGraph = oledGraph(&tft, 0, 27, 100, 128, 0.0f, 10.0f);
  captureGraph = oledGraph(&tft, 0, 27, 100, 128, 0.0f, 40.0f);
  lameGraph = oledGraph(&tft, 0, 40, 127 - 40, 128, 0.0f, 20.0f);

  int bars = 5;
  float vals[5] {0.0f, 5.0f, 10.0f, 15.0f, 20.0f};
  int colors[5] {lameGraph.cGREEN, lameGraph.cYELLOW, lameGraph.cORANGE, lameGraph.cRED, lameGraph.cRED};
  lameGraph.setHorizontalBarValues(5, vals, colors);
  //lameGraph.setHorizontalBarValues(5, [0.0f,5.0f,10.0f,15.0f,20.0f], [lameGraph.cGREEN,lameGraph.cYELLOW,lameGraph.cORANGE,lameGraph.cRED,lameGraph.cRED]);

  float wvals[4] {0.0f, 2.0f, 5.0f, 10.0f};
  int wcolors[4] {weaponGraph.cGREEN, weaponGraph.cORANGE, weaponGraph.cRED, weaponGraph.cRED};
  weaponGraph.setHorizontalBarValues(4, wvals, wcolors);

  float cvals[4] {0.0f, 10.0f, 20.0f, 40.0f};
  int ccolors[4] {captureGraph.cGREEN, captureGraph.cORANGE, captureGraph.cRED, captureGraph.cRED};
  captureGraph.setHorizontalBarValues(4, cvals, ccolors);
}

void dimOLEDDisplay() {
  //static bool alreadyDimmed=false;
  //Fill the rest of the display with black leaving only the top bar.
  tft.fillRect(0, 17, 128, 128 - 17, BLACK);
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
  tft.drawFastVLine(BattX0 - 2, BattY0 + 1, 5, rectColor);
  tft.fillRect(BattX0 + (BattW - barW), BattY0, barW, BattH, fillColor);
  tft.setCursor(BattX0 + BattW + 3, BattY0);
  //tft.setTextSize(1);
  //tft.setTextColor(CYAN,BLACK);
  //tft.print(battPercent);
}
int floatTo10xInt(float g) {
  if (g < 0.0) g = 0.0;
  return ((int) (g * 10.0 + .5));
}
int gv(char *s) {
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
  if (s == "AA") return floatTo10xInt(cableState.ohm_AA);
  if (s == "BB") return floatTo10xInt(cableState.ohm_BB);
  if (s == "CC") return floatTo10xInt(cableState.ohm_CC);

  for (int k = 0; k < NUM_ADC_SCAN_CHANNELS; k++) {
    if ((s[0] == ChanArray[k].ch_label[1]) && (s[1] == ChanArray[k].ch_label[2]) ) {
      return floatTo10xInt(cableState.cableOhm[k]);
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
  static char *oldFault;
  if (s == oldFault) return; //no change
  oldFault = s;
  tft.setTextSize(2);
  tft.fillRect(0, 0, 115, 20, BLACK); //clear label area
  tft.setCursor(2, 2);
  tft.setTextColor(color, BLACK);
  tft.println(s);
}

static int oldA = 0, oldB = 0, oldC = 0;
#define ABAR 25
#define BBAR 60
#define CBAR 95
void graph1(char *s) {
  barGraph(ABAR, 8, gv(s), oldA, s);
}
void graph2(char *s) {
  barGraph(BBAR, 8, gv(s), oldB, s);
}
void graph3(char *s) {
  barGraph(CBAR, 8, gv(s), oldC, s);
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
    //tft.setTextColor(YELLOW, BLACK); tft.setTextSize(2);

    switch (currentDisplayState) {
      case disp_lame:
        createLameDisplay();
        break;
      case disp_cable:
        //tft.setTextColor(YELLOW, BLACK); tft.setTextSize(2);
        //tft.print("Cable");
        labelTitle("Cable", YELLOW);
        oldA = oldB = oldC = 320;
        totalTime = 0ul; timeSamples = 0ul;
        tft.drawFastVLine(127, 31, 96, CYAN);
        tft.drawFastVLine(95, 31, 96, CYAN);
        tft.drawFastVLine(63, 31, 96, CYAN);
        tft.drawFastVLine(31, 31, 96, CYAN);
        //tft.fillRect(0, X + 16, 128, H, BLACK);
        //Height of bar is 8, height of textsize(2) is 14, 1 pix between text and bar, 1 pix between text/bar and line
        tft.fillRect(0, ABAR - 1, 128, 26, BLACK);
        tft.fillRect(0, BBAR - 1, 128, 26, BLACK);
        tft.fillRect(0, CBAR - 1, 128, 26, BLACK);
        tft.setTextSize(1);
        tft.setCursor(31 - 14, 120);
        tft.print("15");
        tft.setCursor(63 - 14, 120);
        tft.print("10");
        tft.setCursor(95 - 8, 120);
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
        long captureDuration=0;
        int trigIndx,lastIndx,trimVal;

        if (FoilADC.hsBuffer.CaptureDone()) {
          newConnection = foil;
          lastConnection = foil;
          sTime = millis();
          trigIndx=FoilADC.hsBuffer.getTriggerIndex();
          lastIndx=FoilADC.hsBuffer.getLastTriggerIndex();
          trimVal=FoilADC.getTrim();
          captureDuration=FoilADC.hsBuffer.getTriggerDuration();
          labelTitle("Foil hit", RED);
        }

        if (EpeeADC.hsBuffer.CaptureDone()) {
          newConnection = epee;
          lastConnection = epee;
          trigIndx=EpeeADC.hsBuffer.getTriggerIndex();
          lastIndx=EpeeADC.hsBuffer.getLastTriggerIndex();
          trimVal=EpeeADC.getTrim();
          captureDuration=EpeeADC.hsBuffer.getTriggerDuration();
          sTime = millis();
          labelTitle("Epee hit", RED);
        }

        if ( millis() > (dispCaptureHoldTime + lastCapture) ) {
          if (armed == false) {
            FoilADC.hsBuffer.ResetTrigger();
            EpeeADC.hsBuffer.ResetTrigger();
            labelTitle("Armed", GREEN);
            armed = true;
          }
          if (FoilADC.hsBuffer.CaptureDone() || EpeeADC.hsBuffer.CaptureDone())  {
            captureGraph.resetGraph();
            for (int k = 0; k < ADC_CAPTURE_LEN; k++) {
              captureGraph.updateGraph((ADC_CaptureBuffer[k]-trimVal)*FoilADC.LOW_GAIN);
            }            
            tft.drawFastVLine(trigIndx, 28, 127, CYAN);
            tft.drawFastVLine(lastIndx, 28, 127, CYAN);
            tft.setTextSize(1);
            tft.setTextColor(CYAN);
            tft.setCursor(trigIndx+20,65);            
            tft.print(captureDuration/1000);
            if (lastIndx>=(ADC_CAPTURE_LEN-1)) {
              tft.print("+");
            }
            tft.print(" ms");
            captureGraph.drawTextLabels();
            lastCapture = millis();
            armed = false;
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
      if ((lastConnection==shorted) && (sTime < dispHoldTime)) {
        newConnection=shorted;
      }
      
      if ((weaponState.foilOn) && (weaponState.epeeOn)) { //short
        newConnection = shorted;
        dispHoldTime=millis()+5000ul;
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
          tft.fillRect(0, 27, 128, 100, BLACK);
          oldA = oldB = 320;
          //oldA=weaponState.ohm10xEpee;
          //oldB=weaponState.ohm10xFoil;
          barGraph(BBAR, 8, weaponState.ohm10xEpee, oldA, "AB");
          barGraph(CBAR, 8, weaponState.ohm10xFoil, oldB, "BC");
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
          barGraph(BBAR, 8, weaponState.ohm10xEpee, oldA, "AB");
          //oldA=weaponState.ohm10xEpee;
          barGraph(CBAR, 8, weaponState.ohm10xFoil, oldB, "BC");
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

  tft.setTextSize(2);
  if (ABshort) //AB short
    if (BCshort) {//if AB and BC then AC has to be shorted
      //A - B - C short
      labelTitle("Short ABC", RED);
      graph1("AB");
      graph2("BC");
      graph3("AC");
    }
    else {
      //AB short, CC okay or open
      labelTitle("Short AB", RED);
      graph1("AB");
      graph2("BB");
      graph3("CC");
    }
  else if (BCshort) {
    //BC and not AB (and therefore not AC), AA okay or open
    labelTitle("Short BC", RED);
    graph1("AA");
    graph2("BC");
    graph3("CC");
  }
  else if (ACshort) {
    //AC and not AB, and therefore not BC, BB okay or open
    labelTitle("Short AC", RED);
    graph1("AC");
    graph2("BB");
    graph3("CC");
  }
  else //no shorts
    if (ABcross)
      if (BAcross) {
        //AB cross, CC okay or open
        labelTitle("AB Cross", RED);
        graph1("AB");
        graph2("BA");
        graph3("CC");
      }
      else if (CAcross) {
        //A->B C->A, could be B->C or open
        labelTitle("ABC Cross", RED);
        graph1("AB");
        graph2("BC");
        graph3("CA");
      }
      else {
        //AB cross, don't know what happened to BA
        labelTitle("AB Cross", RED);
        graph1("AB");
        graph2("BB"); //always going to be open
        graph3("CC");
      }
    else if (BCcross) {
      //we could test CB, and know it was a real BC cross, but if that failed, something has to be open
      labelTitle("BC Cross", RED);
      graph1("AA");
      graph2("BC");
      graph3("CB");
    }
    else if (ACcross)
      if (CAcross) {
        //AC cross, BB good or open
        labelTitle("AC Cross", RED);
        graph1("AC");
        graph2("BB");
        graph3("CA");
      }
      else if (BAcross) {
        //A->C, B->A, C->B
        labelTitle("ACB Cross", RED);
        graph1("AC");
        graph2("BA");
        graph3("CB");
      }
      else {
        //something is open
        labelTitle("AC Cross", RED);
        graph1("AC");
        graph2("BB");
        graph3("CA");
      }

    else  //something is open
      if (BAcross) {
        //BA but not AB
        labelTitle("BA Cross", RED);
        graph1("AB");
        graph2("BA");
        graph3("CC");
      }
      else if (CBcross) {
        //CB but not BC
        labelTitle("CB Cross", RED);
        graph1("AA");
        graph2("BC");
        graph3("CB");
      }
      else if (CAcross) {
        //CA but not AC
        labelTitle("CA Cross", RED);
        graph1("AC");
        graph2("BB");
        graph3("CA");
      }
      else {//no shorts/no crosses
        labelTitle("Cable", GREEN);
        graph1("AA");
        graph2("BB");
        graph3("CC");
      }
}

void createWeaponDisplay() {
  //tft.setCursor(2, 2);
  tft.setTextColor(YELLOW, BLACK); tft.setTextSize(2);
  //tft.print("Weapon");
  labelTitle("Weapon", YELLOW);
  weaponGraph.resetGraph();
}

// barGraph(X, H, oldVal, newVal, oldVal)
void createLameDisplay() {
  labelTitle("Lame", YELLOW);
  lameGraph.resetGraph();
}

void updateLameDisplay() {
  static int oldVal = 9999;
  int lameVal = floatTo10xInt(cableState.ohm_CC);
  static byte i = 0;

  //Serial.println(lameVal);
  barGraph(2, 18, lameVal, oldVal, "Lame");
  oldVal = lameVal;
  if (lameVal > 50) {
    labelTitle("Lame", GREEN);
  } else {
    labelTitle("Lame", RED);
  }
  lameGraph.updateGraph(cableState.ohm_CC);
}

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
