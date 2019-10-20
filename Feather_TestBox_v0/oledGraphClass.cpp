#include "oledGraphClass.h"


//Sets the main buffer
oledGraph::oledGraph(Adafruit_SSD1351 *tft,int X, int Y, int height, int width, float minValue, float maxValue){
  _locX=X;
  _locY=Y;
  _height=height;
  _width=width;
  _maxX=_locX+width;
  _maxY=_locY+height;
  _tft=tft;
  _pixelScaleFactor=(_height)/(maxValue-minValue);
  _offset=minValue;
  _limitMin=minValue;
  _limitMax=maxValue;
}

oledGraph::oledGraph() {}


void oledGraph::setVerticalLimits(float minValue, float maxValue) {
  if (maxValue>=minValue) return;
  _pixelScaleFactor=(_height)/(maxValue-minValue);
  _offset=minValue;
  _limitMin=minValue;
  _limitMax=maxValue;
}

void oledGraph::setHorizontalBarValues(int numBars, float values[], int colors[]) {
  
  for (int k=0; k<numBars; k++) {    
    _hBarY[k]=_maxY-max(int((values[k]-_offset)*_pixelScaleFactor+0.5f),0);
    if (_hBarY[k]<_locY) {_hBarY[k]=_locY;}
    if (_hBarY[k]>_maxY) {_hBarY[k]=_maxY;}
    _hBarValues[k]=values[k];
    _hBarColors[k]=colors[k];
    //Serial.print("Value = ");Serial.print("_hBarValues[k]");Serial.print(" Loc = ");Serial.println(_hBarY[k]);    
  }
  _numActiveBars=numBars;
}

void oledGraph::drawTextLabels() {
  _tft->setTextSize(1);
  _tft->setTextColor(cCYAN);
  for (int k=0; k<_numActiveBars-1; k++) {//top bar plotted differently
    _tft->setCursor(_maxX-12, _hBarY[k]-9);
    _tft->print(_hBarValues[k]);
  }
  
  _tft->setCursor(_maxX-12, _hBarY[_numActiveBars-1]+3);
  _tft->print(_hBarValues[_numActiveBars-1]);  
}

void oledGraph::drawHLines() {
  
  for (int k=0; k<_numActiveBars; k++) {
    _tft->drawFastHLine(_locX, _hBarY[k], _maxX-_locX-1,cWHITE);
    Serial.print("Value = ");Serial.print(_hBarValues[k]);Serial.print(" Loc = ");Serial.println(_hBarY[k]);    
    //Serial.print("Drawing Line @ ");Serial.println(_hBarY[k]);
  }
  //_tft->drawPixel(_col, _maxY, cWHITE); //draw bottom scale (9.9 ohms), could be erased later
}

void oledGraph::resetGraph() {
  drawHLines();
  drawTextLabels();
  }

void oledGraph::updateGraph(float newValue){
  if (_col==_locX) {
    drawHLines();
    drawTextLabels();
  }
  drawColumn(newValue);
  
  if (++_col>(_locX+_width)) {
    _col=_locX;
  }
}

void oledGraph::drawColumn(float val){
  static int oldRow=0;  
  int valRow;

  //int valRow = 127 - min(val, 99); //convert ohms in tenths to row
  //int valRow = _locY + (_pixelScaleFactor*(val-_offset)); //convert ohms in tenths to row
  valRow=_maxY-int(_pixelScaleFactor*(val-_offset)-0.5f);
  valRow=max(valRow,_locY);
  valRow=min(valRow,_maxY);
  
  int topRow, botRow;
  //sort ends so we're always drawing from bottom to top (large to small values)
  if (valRow > oldRow) {
    topRow = oldRow;
    botRow = valRow;
  } else {
    topRow = valRow;
    botRow = oldRow;
  }

  topRow=max(topRow,_locY);
  botRow=min(botRow,_maxY);
  _tft->drawFastVLine(_col, _locY, _maxY-_locY+1, cBLACK); //clear out the column
  
  //Re-draw the horizontal lines
  for (int k=0; k<_numActiveBars; k++) {
    _tft->drawPixel(_col, _hBarY[k], cWHITE);
  }  

  int curRow=topRow;
  int endRow;
  bool lineDrawn=false;
  for (int k=_numActiveBars-1; k>0; k--) {//Draw from top down
    if ( (curRow>=_hBarY[k]) && (curRow<=_hBarY[k-1]) )  { //Indexes down, so below this bar
      if (topRow==botRow) {
        _tft->drawPixel(_col,curRow,_hBarColors[k-1]);
        lineDrawn=true;
        break;
      }
      endRow=min(_hBarY[k-1],botRow);
      _tft->drawFastVLine(_col,curRow,endRow-curRow+1,_hBarColors[k-1]); //
      curRow=endRow;
      lineDrawn=true;
      //if (endRow>=botRow) break;      
    }
  }
  /*if (lineDrawn==false) {
    Serial.println("Error no lines drawn");
    Serial.print("valRow = "); Serial.println(valRow);
    Serial.print("oldRow = "); Serial.println(oldRow);
    Serial.print("topRow = "); Serial.println(topRow);
    Serial.print("botRow = "); Serial.println(botRow);
   for (int k=_numActiveBars-1; k>0; k--) {
       Serial.print("Value = ");Serial.print(_hBarValues[k]);Serial.print(" Loc = ");Serial.println(_hBarY[k]);    
   } //Serial.print("Drawing Line @ ");Serial.println(_hBarY[k]);
  }*/
  oldRow=valRow;
  
  /*if (botRow < _maxY) { //fill black to bottom
    _tft->drawFastVLine(_col, botRow + 1, (_maxY-(botRow+1)), cBLACK);
      for (int k=0; k<_numActiveBars; k++) {
        if (botRow<_hBarY[k]) {
          _tft->drawPixel(_col, _hBarY[k], cWHITE);
        }
      }
    _tft->drawPixel(_col, _maxY, cWHITE); //draw bottom scale (9.9 ohms), could be erased later    
  }*/
  
  if (_col < (_locX+_width) ) {
    int irow = max(valRow - 1, _locY);
    //    if (val>18) {Serial.print(icol);Serial.print("=");;Serial.print(irow);Serial.println(",CYAN");}
    _tft->drawFastVLine(_col+1, irow, 2, cCYAN);
    _tft->drawFastVLine(_col+2, irow, 2, cCYAN);
  }  //show where we are as 2x1 cyan line at col+1
  
}
