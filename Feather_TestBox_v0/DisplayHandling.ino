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
int oldVal = 0;            //prior value (global so we don't have to pass by ref everywhere)
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

void printVal(int x, int y, int valColor, char *lab, int val) {
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
void drawColumn(int col, int val) {
  //0-9.9 ohm, rows 27-127
  int valRow = 127 - min(val, 99); //convert ohms in tenths to row
  int oldRow = 127 - min(oldVal, 99); //same with old val
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
}

//Draw a static line graph from an array
//  rData = array with resistance values in tenths of an ohm
//  oThresh = resistance of Orange threshold im tenths of an ohm
//  gThresh = resistance of Green threshold in tenths of an ohm
void LineGraph(int rData[127]) {
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
  oldVal = rData[0]; //force first column to be a point
  for (i = 0; i < 128; i++) {
    val = rData[i];
    if (val == 990) val = maxVal; //if not connected, use max value in array
    drawColumn(i, val);
    oldVal = val;
  }
  endTime = micros();
  if (startTime < endTime) {
    totalTime = endTime - startTime;
    timeSamples = 1ul;
  };
}
void InitializeDisplay()
{
  tft.begin();
  tft.setRotation(3);  //3 sets the display top to be aligned with the Feather uUSB. 
  tft.fillRect(0, 0, 128, 128, BLACK);
  tft.setCursor(0, 0);
  tft.setTextColor(YELLOW, BLACK); tft.setTextSize(2);
  tft.println("Welcome to TTtarm");
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

  if (batteryVoltage > 3.4) {
    battPercent = int( (batteryVoltage - 3.4) / (4.2 - 3.4) * 100);
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

char *oldFault;
void labelFault(char *s) {
  if (oldFault == s) return; //no change
  oldFault = s;
  tft.fillRect(0, 0, 115, 20, BLACK); //clear label area
  tft.setCursor(2, 2);
  if (s == "Cable") { //no fault
    tft.setTextColor(GREEN, BLACK);
    tft.println("Cable");
    return;
  }
  tft.setTextColor(RED, BLACK);
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

void updateOLED(char Mode) {
  static int i = 0, val, oldMode = 'z';
  static bool oldFoil = false;
  static bool oldEpee = false;
  static bool oledEnabled = true;
  enum lastConnected { first, none, epee, foil, shorted};
  static lastConnected lastConnection;
  static int foilIndicator, epeeIndicator, foilInterIndicator, epeeInterIndicator;
  static unsigned long sTime; //start time of last seen foil/epee connection
  bool inter;
  bool ABcross = CheckCableStatusByte((1 << BITAB));
  bool ACcross = CheckCableStatusByte((1 << BITAC));
  bool BAcross = CheckCableStatusByte((1 << BITBA));
  bool BCcross = CheckCableStatusByte((1 << BITBC));
  bool CAcross = CheckCableStatusByte((1 << BITCA));
  bool CBcross = CheckCableStatusByte((1 << BITCB));
  bool ABshort = (CheckCableStatusByte((1 << BITAB)) || CheckCableStatusByte((1 << BITBA))) && ((!CheckCableStatusByte((1 << BITAA))) || (!CheckCableStatusByte((1 << BITBB))));
  bool BCshort = (CheckCableStatusByte((1 << BITBC)) || CheckCableStatusByte((1 << BITCB))) && ((!CheckCableStatusByte((1 << BITBB))) || (!CheckCableStatusByte((1 << BITCC))));
  bool ACshort = (CheckCableStatusByte((1 << BITAC)) || CheckCableStatusByte((1 << BITCA))) && ((!CheckCableStatusByte((1 << BITAA))) || (!CheckCableStatusByte((1 << BITCC))));

  //Serial.println(Mode);

  if (Mode != oldMode) {
    //Serial.println("Setting new mode");
    tft.enableDisplay(true);
    oldMode = Mode;
    if (Mode != 'd') {
      tft.fillRect(0, 0, 128, 128, BLACK);
      tft.setCursor(0, 0);
      tft.setTextColor(YELLOW, BLACK); tft.setTextSize(2);
    }


    switch (Mode) {
      case 'c':
        oldFault = "";
        tft.print("Cable");
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
      case 'r':
        foilIndicator = epeeIndicator = foilInterIndicator = epeeInterIndicator = BLACK;
        lastConnection = first;
        oldVal = 990;
        oldEpee = false;
        oldFoil = false;
        i = 0;
        tft.setTextSize(2);
        tft.print("Weapon");
        break;
      case 'w':
        foilIndicator = epeeIndicator = foilInterIndicator = epeeInterIndicator = BLACK;
        lastConnection = first;
        tft.setTextSize(2);
        tft.print("WpnTest");
        break;
      case 'i':
        tft.fillScreen(BLACK);
        tft.setTextSize(2);
        tft.setTextColor(BLUE, BLACK);
        tft.print("Idle");
        //tft.fillScreen(BLACK);
    }
    displayBatteryStatus();
  }

  if ((millis() - tLastActive) > tOledOff) {
    tft.enableDisplay(false);
    oledEnabled = false;
    return;
  } else {
    if (!oledEnabled) {
      tft.enableDisplay(true);
      oledEnabled = true;
    }
  }

  switch (Mode) {
    case 'c':
      startTime = micros();
      tft.setTextSize(2);
      if (ABshort) //AB short
        if (BCshort) {//if AB and BC then AC has to be shorted
          //A - B - C short
          labelFault("Short ABC");
          graph1("AB");
          graph2("BC");
          graph3("AC");
        }
        else {
          //AB short, CC okay or open
          labelFault("Short AB");
          graph1("AB");
          graph2("BB");
          graph3("CC");
        }
      else if (BCshort) {
        //BC and not AB (and therefore not AC), AA okay or open
        labelFault("Short BC");
        graph1("AA");
        graph2("BC");
        graph3("CC");
      }
      else if (ACshort) {
        //AC and not AB, and therefore not BC, BB okay or open
        labelFault("Short AC");
        graph1("AC");
        graph2("BB");
        graph3("CC");
      }
      else //no shorts
        if (ABcross)
          if (BAcross) {
            //AB cross, CC okay or open
            labelFault("AB Cross");
            graph1("AB");
            graph2("BA");
            graph3("CC");
          }
          else if (CAcross) {
            //A->B C->A, could be B->C or open
            labelFault("ABC Cross");
            graph1("AB");
            graph2("BC");
            graph3("CA");
          }
          else {
            //AB cross, don't know what happened to BA
            labelFault("AB Cross.");
            graph1("AB");
            graph2("BB"); //always going to be open
            graph3("CC");
          }
        else if (BCcross) {
          //we could test CB, and know it was a real BC cross, but if that failed, something has to be open
          labelFault("BC Cross");
          graph1("AA");
          graph2("BC");
          graph3("CB");
        }
        else if (ACcross)
          if (CAcross) {
            //AC cross, BB good or open
            labelFault("AC Cross");
            graph1("AC");
            graph2("BB");
            graph3("CA");
          }
          else if (BAcross) {
            //A->C, B->A, C->B
            labelFault("ACB Cross");
            graph1("AC");
            graph2("BA");
            graph3("CB");
          }
          else {
            //something is open
            labelFault("AC Cross.");
            graph1("AC");
            graph2("BB");
            graph3("CA");
          }

        else  //something is open
          if (BAcross) {
            //BA but not AB
            labelFault("BA Cross");
            graph1("AB");
            graph2("BA");
            graph3("CC");
          }
          else if (CBcross) {
            //CB but not BC
            labelFault("CB Cross");
            graph1("AA");
            graph2("BC");
            graph3("CB");
          }
          else if (CAcross) {
            //CA but not AC
            labelFault("CA Cross");
            graph1("AC");
            graph2("BB");
            graph3("CA");
          }
          else {//no shorts/no crosses
            labelFault("Cable");
            graph1("AA");
            graph2("BB");
            graph3("CC");
          }

      endTime = micros();
      if (startTime < endTime) {
        totalTime += (endTime - startTime);
        timeSamples += 1ul;
      }

      break;
    case 'r':
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
      tft.setTextSize(2);
      tft.setTextColor(YELLOW, BLACK);
      tft.setCursor(0, 0);
      if (weaponState.foilOn) {
        if (weaponState.epeeOn) {  //short
          if ((oldFoil != weaponState.foilOn) || (oldEpee != weaponState.epeeOn)) {
            tft.fillRect(0, 27, 128, 100, BLACK);
            tft.print("Weapon");
            tft.setTextColor(RED, BLACK);
            tft.setCursor(75, 0);
            tft.print("Gnd");
            oldA = oldB = 320;
            lastConnection = shorted;
          }
          barGraph(BBAR, 8, weaponState.ohm10xEpee, oldA, "AB");
          barGraph(CBAR, 8, weaponState.ohm10xFoil, oldB, "BC");
        }
        else {
          if (!oldFoil || lastConnection != foil)  {
            tft.fillRect(0, 28, 128, 100, BLACK);
            tft.setCursor(0, 0);
            tft.print("Foil     ");
            i = 0; oldVal = 0;
            lastConnection = foil;
            tft.drawFastHLine(0, 27, 128, WHITE);
            tft.drawFastHLine(0, 77, 128, WHITE);
            tft.drawFastHLine(0, 107, 128, WHITE);
            tft.drawFastHLine(0, 127, 128, WHITE);
          }
          val = weaponState.ohm10xFoil;
          drawColumn(i, val);
          printVal(0, 50, YELLOW, "", val);
          oldVal = val;
          if (++i >= 128) i = 0;
          sTime = millis(); //keep reseting every time we see it enabled
        }
      }
      else if (weaponState.epeeOn) {
        if (!oldEpee || lastConnection != epee) {
          tft.fillRect(0, 28, 128, 100, BLACK);
          tft.setCursor(0, 0);
          tft.print("Epee     ");
          i = 0; oldVal = 0;
          lastConnection = epee;
          tft.drawFastHLine(0, 27, 128, WHITE);
          tft.drawFastHLine(0, 77, 128, WHITE);
          tft.drawFastHLine(0, 107, 128, WHITE);
          tft.drawFastHLine(0, 127, 128, WHITE);
        }
        val = weaponState.ohm10xEpee;
        drawColumn(i, val);
        printVal(0, 50, YELLOW, "", val);
        oldVal = val;
        if (++i >= 128) i = 0;
        sTime = millis(); //keep reseting every time we see it enabled
      }
      else {  //no connect
        switch (lastConnection) {
          case foil:
            val = weaponState.ohm10xFoil;
            drawColumn(i, val);
            printVal(0, 50, YELLOW, "", val);
            oldVal = val;
            if (++i >= 128) i = 0;
            if ((millis() - sTime) > 5000ul) {
              lastConnection = none;
            }
            break;
          case epee:
            val = weaponState.ohm10xEpee;
            drawColumn(i, val);
            printVal(0, 50, YELLOW, "", val);
            oldVal = val;
            if (++i >= 128) i = 0;
            if ((millis() - sTime) > 5000ul) {
              lastConnection = none;
            }
            break;
          case first:
          case none:
          case shorted:
            if (oldFoil || oldEpee) {
              tft.fillRect(0, 28, 128, 100, BLACK);
              tft.setCursor(0, 0);
              tft.setTextColor(YELLOW, BLACK);
              if (lastConnection != first)
                tft.print(oldFoil ? "Epee Open" : "Foil Open");

              tft.drawFastHLine(0, 27, 128, WHITE);
              tft.drawFastHLine(0, 77, 128, WHITE);
              tft.drawFastHLine(0, 107, 128, WHITE);
              tft.drawFastHLine(0, 127, 128, WHITE);
            }
            break;
        }
      }
      oldFoil = weaponState.foilOn;
      oldEpee = weaponState.epeeOn;

      break;
    case 'w':
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
      break;
    case 'i':
      //tft.enableDisplay(false);
      break;
  }
}

void writeSerialOutput(char Mode) {
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
    snprintf(outputString, bufferSize, "t=%ld, N=%ld, %sHz, %c, Bat=%sV\r\n", t_now, numSamples, tempString1, BoxState, tempString2);
    Serial.write(outputString);
    tHeartBeat = millis();
    t_last_upd = millis();
    numSamples = 0;
  }

  outputString[0] = '\0'; //Reset the outputString
  switch (Mode) {
    case 'c':
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
    case 'w':
      //Format is "W,time,E,Light On/Off,t last epee trigger, F, Foil light On/Off, t last foil trigger
      snprintf(outputString, bufferSize, "W,%ld,E,%u,%ld,F,%u,%ld\r\n", t_now, weaponState.epeeOn, weaponState.tEpeeTrigger, weaponState.foilOn, weaponState.tFoilTrigger);
      //Serial.write(outputString);
      break;
    case 'r':
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
