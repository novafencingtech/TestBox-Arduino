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

void printVal(int x, int y, int valColor, String lab, int val) {
  tft.setCursor(y, x);
  tft.setTextColor(YELLOW, BLACK);
  tft.print(lab); tft.print("="); //display connection string
  tft.setTextColor(valColor, BLACK);  //set the text color to right color, black background
  //Print ohms in a 4 character space
  //"XXXX", "XXX ", "XX  " or "X.XX"
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
  }
  else tft.print("  ");  //value between 32 and 10 ohms
  return;
}

// barGraph(X, H, oldVal, newVal, oldVal, oThresh, gThresh)
//  X = top lime
//  H = height of bar
//  oldVal = last value in tenths of an ohm
//  newVal = current value
//  oThresh = threshold for Orange color
//  gThresh = threshold for Green color
// with 128 pixel wide screen, show 32 ohms * 4 pixels/ohm
void barGraph(int X, int H, int newVal, int &oldVal, int oThresh, int gThresh, String conn) {
  int difVal, rOld, rNew, bc = ORANGE; bool newColor = false;
  if (newVal > oThresh) {  //if the new value is greater than the orange threshold
    bc = RED; //then its a red bar
    if (oldVal <= oThresh) newColor = true;  //did we change color?
  }
  else if (newVal <= gThresh) {  //if new value is less than the green threshold
    bc = GREEN;  //then its a green bar
    if (oldVal > gThresh) newColor = true;  //did we change color
  }
  else if ((oldVal < oThresh) || (oldVal >= gThresh)) newColor = true;  //orange, but not if the prior value wasn't orange
  printVal(X, 0, bc, conn, newVal);
  if (newVal > 320) {
    tft.fillRect(0, X + 16, 128, H, BLACK);
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

int ValColor(int val, int oThresh, int gThresh) {
  if (val > oThresh) return RED;
  if (val > gThresh) return ORANGE;
  return GREEN;
}
//Line Graph helper.  Line graphs are vertical lines from the old value to the new value
//This draws a line in the right color(s)
//It assumes that the endpoints (start/end row values) are sorted
// col = column to draw
// topY = row of top of line
// bottomY = row of bottom of line
// oY = row where Orange starts
// gY = row where Green starts
void drawColumn(int col, int topY, int bottomY, int oY, int gY) {
  int startColor, endColor;
  if (topY > 27) tft.drawFastVLine(col, 27, topY - 1, BLACK); //clear above start of line
  startColor = RED;
  if (topY > oY) startColor = ORANGE;
  if (topY > gY) startColor = GREEN;
  endColor = RED;
  if (bottomY > oY) endColor = ORANGE;
  if (bottomY > gY) endColor = GREEN;
  if (startColor == endColor) {  //check if this is a zero length line
    if (topY == bottomY) tft.drawPixel(col, topY, startColor); //draw a pixel)

    else tft.drawFastVLine(col, topY, bottomY - topY, startColor);  //no, single color line
  }
  else if ((startColor == RED) && (endColor == GREEN)) { //do we pass through both thresholds?
    tft.drawFastVLine(col, topY, oY - topY, RED);  //draw red from current point to orange threshold
    tft.drawFastVLine(col, oY, gY - oY, ORANGE);  //draw orange from orange threshold to green threshold
    tft.drawFastVLine(col, gY, bottomY - gY, GREEN);  // draw green from green threshold to prior point
  }
  else {  //passes through one threshold
    if (startColor == RED) { //starts in the rea
      tft.drawFastVLine(col, topY, oY - topY, RED);  //draw red from current point to orange threshold
      tft.drawFastVLine(col, oY, bottomY - oY, ORANGE);  //draw orange from current orange threshold to prior point

    }
    else {  //starts in orange
      tft.drawFastVLine(col, topY, gY - topY, ORANGE);  //draw orange from start point to green threshold
      tft.drawFastVLine(col, gY, bottomY - gY, GREEN);  //draw green from green threshold to prior point

    }
  }
  if (bottomY < 127) tft.drawFastVLine(col, bottomY + 1, 126 - bottomY, BLACK);  //clear from prior point to bottom

}
//Helper for Line Graphs
//draws a column.
//  col = column where line is drawn
//  val = resistance of current value in tenths of an ohm
//  oldVal = resistance of prior value in tenths of an ohm
//  vMax = scale (value of top of graph -1)
//  vRatio = tenths of an ohm per pixel
//  oY = row value where line turns to orange
//  gY = rowe value where lime turns to green
int drawGraphColumn(int col, int val, int vMax, int vRatio, int oY, int oG) {
  int yVal, oldY;

  if (val >= vMax) val = vMax - 1;  //clamp to vMax-1
  yVal = 127 - (val / vRatio);  //Convert resistance to row #
  oldY = 127 - (oldVal / vRatio);  //same for prior row

  if (oldY > yVal) drawColumn(col, yVal, oldY, oY, oG);  // line goes down
  else drawColumn(col, oldY, yVal, oY, oG);  //line goes up

}
//Draw a static line graph from an array
//  rData = array with resistance values in tenths of an ohm
//  oThresh = resistance of Orange threshold im tenths of an ohm
//  gThresh = resistance of Green threshold in tenths of an ohm
void LineGraph(int rData[127], int oThresh, int gThresh) {
  int i, val, vMax, vRatio, yVal, oldY, startColor, endColor, oY, gY;
  val = 0;
  for (i = 0; i < 128; i++) if (rData[i] > val) val = rData[i]; //find biggest value
  // We reserve 28 rows for text, leaving 100 rows for graph
  // Graphs are scaled to the max resistance in the array
  //    Max R  Precision (1 pixel)
  //      20   .25 ohm
  //     100    1 ohm
  //     500    5 ohm
  startTime = micros();
  vMax = 200;  // assume 20 ohm scale
  if (val > 200) vMax = 1000;  //possible 100 scale
  if (val > 1000) vMax = 5000;  //nope, it's 1000 ohm scale
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
  oY = 127 - (oThresh / vRatio);  //calculate row # of orange threshold
  gY = 127 - (gThresh / vRatio);  // same for green threshold
  oldVal = rData[0]; //force first column to be a point
  for (i = 0; i < 128; i++) {
    val = rData[i];
    drawGraphColumn(i, val, vMax, vRatio, oY, gY);
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
  tft.fillRect(0, 0, 128, 128, BLACK);
  tft.setCursor(0, 0);
  tft.setTextColor(YELLOW, BLACK); tft.setTextSize(2);
  tft.println("Welcome to TTtarm");
}
/*
  void updateCableLCD() {
  static byte priorStatus;
  byte statusCheck=((1<<BITAA)|(1<<BITBB)|(1<<BITCC));
  char numString[OHM_FIELD_WIDTH+1]="";
  //int strLen;

  strcpy_P(lcdString2,PSTR(""));

  strcpy_P(numString,PSTR(" -- "));
  if (cableState.ohm_AA<OPEN_CIRCUIT_VALUE) {
    dtostrf(cableState.ohm_AA,OHM_FIELD_WIDTH,2,numString);
  }

  strncat(lcdString2,numString,OHM_FIELD_WIDTH);
  strncat_P(lcdString2,PSTR("  "),2);

  strcpy_P(numString,PSTR(" -- "));
  if (cableState.ohm_BB<OPEN_CIRCUIT_VALUE) {
    dtostrf(cableState.ohm_BB,OHM_FIELD_WIDTH,2,numString);
  }
  strncat(lcdString2,numString,OHM_FIELD_WIDTH);
  strncat_P(lcdString2,PSTR("  "),2);

  strcpy_P(numString,PSTR(" -- "));
  if (cableState.ohm_CC<OPEN_CIRCUIT_VALUE) {
    dtostrf(cableState.ohm_CC,OHM_FIELD_WIDTH,2,numString);
  }
  strncat(lcdString2,numString,OHM_FIELD_WIDTH);

  lcd.setCursor(0,1);
  lcd.print(lcdString2);
  //strcat(lcdString2,"\n");
  //Serial.write(lcdString2);

  if (cableState.statusByte==0) {
    lcd.setCursor(0,0);
    lcd.print(F("   Cable GOOD   "));
    return;
  }

  if (!(cableState.statusByte==priorStatus)) {
    priorStatus=cableState.statusByte;
    tLastActive=millis();
  }

  //Clear all bits other than the main lines, check if they're all disconnected
  if (cableState.cableDC) {
      lcd.setCursor(0,0);
      lcd.print(F("Cable disconnect"));
      return;
  }

  if (cableState.lameMode) {
    lcd.setCursor(0,0);
    if (!bitRead(cableState.statusByte,BITAA)) {
      lcd.print(F("    Lame Good   "));
    } else {
      lcd.print(F(" Bad spot/cable "));
    }
    return;
  }

  statusCheck=((1<<BITAA)|(1<<BITBB)|(1<<BITCC));
  if ((cableState.statusByte & statusCheck)>0) {
    strcpy_P(lcdString1,PSTR(" Line "));
    if bitRead(cableState.statusByte,BITAA) {
      strncat_P(lcdString1,PSTR("A"),1);
    }
    if bitRead(cableState.statusByte,BITBB) {
      strncat_P(lcdString1,PSTR("B"),1);
    }
    if bitRead(cableState.statusByte,BITCC) {
      strncat_P(lcdString1,PSTR("C"),1);
    }
    strncat_P(lcdString1,PSTR(" high"),8);
  }

  statusCheck=( (1<<BITAB) );
  if (CheckCableStatusByte(statusCheck)) {
    strncpy_P(lcdString1,PSTR("Line AB short"),LCD_TEXT_COLS);
  }
  statusCheck=( (1<<BITBC) );
  if (CheckCableStatusByte(statusCheck)) {
    strncpy_P(lcdString1,PSTR("Line BC short"),LCD_TEXT_COLS);
  }
  statusCheck=( (1<<BITAC) );
  if (CheckCableStatusByte(statusCheck)) {
    strncpy_P(lcdString1,PSTR("Line AC short"),LCD_TEXT_COLS);
  }
  statusCheck=( (1<<BITAA)|(1<<BITBB)|(1<<BITAB) );
  if (CheckCableStatusByte(statusCheck)) {
    strncpy_P(lcdString1,PSTR("Line AB cross"),LCD_TEXT_COLS);
  }
  statusCheck=( (1<<BITBB)|(1<<BITBC)|(1<<BITBC) );
  if (CheckCableStatusByte(statusCheck)) {
    strncpy_P(lcdString1,PSTR("Line BC cross"),LCD_TEXT_COLS);
  }
  statusCheck=( (1<<BITAA)|(1<<BITCC)|(1<<BITAC) );
  if (CheckCableStatusByte(statusCheck)) {
    strncpy_P(lcdString1,PSTR("Line AC cross"),LCD_TEXT_COLS);
  }
  statusCheck=((1<<BITAA)|(1<<BITBB)|(1<<BITCC)|(1<<BITAB)|(1<<BITBC));
    if (CheckCableStatusByte(statusCheck)) {
      strncpy(lcdString1,"BAD! A->B B->C  ",LCD_TEXT_COLS);
    }
  statusCheck=((1<<BITAA)|(1<<BITBB)|(1<<BITCC)|(1<<BITAC));
    if (CheckCableStatusByte(statusCheck)) {
      strncpy(lcdString1,"BAD! A->C  B->A",LCD_TEXT_COLS);
    }

  while (strlen(lcdString1)<LCD_TEXT_COLS) {
    strcat(lcdString1," ");
  }
  lcd.setCursor(0,0);
  lcd.print(lcdString1);
  }*/


void displayBatteryStatus() {
  char voltString[5] = "\0";
  char percString[5] = "\0";
  float battPercent = 0;

  if (batteryVoltage > 3.4) {
    battPercent = (batteryVoltage - 3.4) / (4.2 - 3.4) * 100;
  }
  /*
    lcd.clear();
    dtostrf(batteryVoltage,4,2,voltString);
    dtostrf(battPercent,3,0,percString);
    lcd.print(F("  Battery   "));
    lcd.setCursor(0,1);
    snprintf(lcdString2,16," %sV  %s%% ",voltString,percString);
    lcd.print(lcdString2);*/
}
int grahamToBrian(float g) {
  if (g < 0.0) g = 0.0;
  return ((int) (g* 10.0+.5));
}
int gv(String s) {
  //arduino does not support strings in switch statements
  if (s == "AA") return grahamToBrian(cableState.ohm_AA);
  if (s == "AB") return grahamToBrian(cableState.ohm_AA);
  if (s == "AC") return grahamToBrian(cableState.ohm_AA);
  if (s == "BA") return grahamToBrian(cableState.ohm_BB);
  if (s == "BB") return grahamToBrian(cableState.ohm_BB);
  if (s == "BC") return grahamToBrian(cableState.ohm_BB);
  if (s == "CA") return grahamToBrian(cableState.ohm_CC);
  if (s == "CB") return grahamToBrian(cableState.ohm_CC);
  if (s == "CC") return grahamToBrian(cableState.ohm_CC);
}
String oldFault;
void labelFault(String s) {
  if (oldFault==s) return; //no change
  oldFault=s;
  tft.fillRect(0,0, 115, 20, BLACK); //clear label area
  tft.setCursor(2,2);
  if (s=="Cable") { //no fault
    tft.setTextColor(GREEN, BLACK);
    tft.println("Cable");
    return;
  }
  tft.setTextColor(RED, ORANGE);
  tft.println(s);
}
static int oldA = 0, oldB = 0, oldC = 0;
void graph1(String s) {
  barGraph(30, 8, gv(s), oldA, 100, 50, s);
}
void graph2(String s) {
  barGraph(65, 8, gv(s), oldB, 100, 50, s);
}
void graph3(String s) {
  barGraph(100, 8,gv(s), oldC, 100, 50, s);
}

void updateOLED(char Mode) {
  static int i = 0, val, oldMode = 'z';
  bool ABcross = CheckCableStatusByte((1 << BITAB));
  bool ACcross = CheckCableStatusByte((1 << BITAC));
  bool BAcross = CheckCableStatusByte((1 << BITBA));
  bool BCcross = CheckCableStatusByte((1 << BITBC));
  bool CAcross = CheckCableStatusByte((1 << BITCA));
  bool CBcross = CheckCableStatusByte((1 << BITCB));
  bool ABshort = (CheckCableStatusByte((1 << BITAB)) || CheckCableStatusByte((1 << BITBA))) && ((!CheckCableStatusByte((1 << BITAA))) || (!CheckCableStatusByte((1 << BITBB))));
  bool BCshort = (CheckCableStatusByte((1 << BITBC)) || CheckCableStatusByte((1 << BITCB))) && ((!CheckCableStatusByte((1 << BITBB))) || (!CheckCableStatusByte((1 << BITCC))));
  bool ACshort = (CheckCableStatusByte((1 << BITAC)) || CheckCableStatusByte((1 << BITCA))) && ((!CheckCableStatusByte((1 << BITAA))) || (!CheckCableStatusByte((1 << BITCC))));

  tft.setCursor(0, 0);
  tft.setTextColor(YELLOW, BLACK); tft.setTextSize(2);
  if (Mode != oldMode) {
    oldMode = Mode;
    switch (Mode) {
      case 'c':
        tft.fillRect(0, 0, 128, 128, BLACK);
        oldFault="";
        tft.setTextColor(YELLOW, BLACK); tft.setTextSize(2); tft.print("Cable");
        oldA=oldB=oldC=320;
        totalTime=0ul; timeSamples=0ul;
        break;
      case 'w':
      case 'r':
        break;
    }
  }
  switch (Mode) {
    case 'c':
      startTime=micros();
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

      endTime=micros();
      if (startTime<endTime) { totalTime+=(endTime-startTime); timeSamples+=1ul;}

      break;
    case 'w':
    case 'r':
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
      if (timeSamples>0ul) {
        snprintf(tempString1, tempBufferSize, ",DTime, %ul", (totalTime/timeSamples)/1000ul);
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
      break;
  }

  Serial.write(outputString);Serial.println(totalTime/timeSamples);

  /*if (SerialBT.available()){
    SerialBT.print(outputString);
    }*/
}