#include "oledGraphClass.h"


//Sets the main buffer
void oledGraph::oledGraph(Adafruit_SSD1351 *tft,int X, int Y, int height, int width, float minValue, float maxValue){
  _locX=X;
  _locY=Y;
  _height=height;
  _width=width;
  _tft=tft;
}

void oledGraph::setVerticalLimits(int lower, int upper) {
  if ((upper-lower)<=1) return;
  _pixelScaleFactor=(upper-lower)/(height-1);
  _offset=lower;
  _limitMin=lower;
  _limitMax=upper;
}

void oledGraph::setHorizontalBarValues(int numBars, int[] values) {
  for (int k=0; k<numBars; k++) {
    _hBarY[k]=(values[k]-_offset)*_pixelScaleFactor;
    _hBarValues[k]=values[k];
  }
}

void oledGraph::resetGraph() {}

void oledGraph::drawColumn(int val){
  static int oldRow=0;  

  if (_col==_locX) {
    drawTextLabels();
  }
  //int valRow = 127 - min(val, 99); //convert ohms in tenths to row
  //int valRow = _locY + (_pixelScaleFactor*(val-_offset)); //convert ohms in tenths to row
  valRow=_locY+_height-(_pixelScaleFactor*(val-_offset));
  valRow=min(valRow,_locY);
  
  int topRow, botRow, orangeRow, greenRow;
  //sort ends so we're always drawing from top to bottom
  if (valRow > oldRow) {
    topRow = oldRow;
    botRow = valRow;
  } else {
    topRow = valRow;
    botRow = oldRow;
  }

  topRow=min(topRow,_locY);
  botRow=max(botRow,_maxY);
  _tft->drawFastVLine(_col, _locY, _maxY-_locY, BLACK); //clear out the column
  
  _tft->drawPixel(_col, _locY, WHITE); //draw top scale (9.9 ohms), could be erased later
  for (int k=0; k<_numActiveBars; k++) {
    _tft->drawPixel(_col, _hBarY[k], WHITE);
  }
  _tft->drawPixel(_col, _maxY, WHITE); //draw bottom scale (9.9 ohms), could be erased later

  int curRow=topRow;
  int endRow;
  for (int k=_numActiveBars-1; k>=0; k--) {
    if (topRow<_hBarY[k]) {
      endRow=min(_hBarY[k],botRow);
      _tft->drawFastVLine(_col,curRow,curRow-_hBarY[k],_hBarColors[k]);
      curRow=endRow;
      if (endRow>=botRow) break;      
    }
  }
  
  if (botRow < _maxY) { //fill black to bottom
    tft->drawFastVLine(col, botRow + 1, (_maxY-(botRow+1)), BLACK);
      for (int k=0; k<_numActiveBars; k++) {
        if (botRow<_hBarY[k]) {
          _tft->drawPixel(_col, _hBarY[k], WHITE);
        }
      }
    _tft->drawPixel(_col, _maxY, WHITE); //draw bottom scale (9.9 ohms), could be erased later    
  }
  
  if (_col < (_locX+_width) ) {
    int icol = _col + 1; int irow = max(valRow - 1, _locY);
    //    if (val>18) {Serial.print(icol);Serial.print("=");;Serial.print(irow);Serial.println(",CYAN");}
    _tft->drawFastVLine(icol, irow, 2, CYAN);
  }  //show where we are as 2x1 cyan line at col+1
  oldRow=valRow;
  oldVal=val;

  if (++_col>(_locX+_width)) {
    _col=_locX;
  }
}
