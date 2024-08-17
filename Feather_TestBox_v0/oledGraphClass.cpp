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
    //Serial.print("Value = ");Serial.print(_hBarValues[k]);Serial.print(" Loc = ");Serial.println(_hBarY[k]);    
    //Serial.print("Drawing Line @ ");Serial.println(_hBarY[k]);
  }
  //_tft->drawPixel(_col, _maxY, cWHITE); //draw bottom scale (9.9 ohms), could be erased later
}

void oledGraph::resetGraph() {
  _tft->fillRect(_locX,_locY,_width,_height,cBLACK);
  drawHLines();
  drawTextLabels();
  _col=_locX;
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

  if (_col==_locX) { //Starting over, no need to draw a line
    topRow=valRow;
    botRow=valRow;
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

/*
    oledBarGraph();
    oledBarGraph(Adafruit_SSD1351 *tft,int X, int Y, int height, int width); 
    oledBarGraph(Adafruit_SSD1351 *tft,int X, int Y, int height, int width,float minValue, float maxValue); 
    void setGraphLimits(int X, int Y, int H, int W);
    void setVerticalLimits(float minValue, float maxValue);
    void setHorizontalBarValues(int numBars, float values[], int colors[]);
    void updateGraph(float newValue);
    void resetGraph();
    //void drawTextLabels();
    void drawHLines();
    void drawTextLabels();

    private:
      void drawColumn(float val);
*/

oledReverseHBarGraph::oledReverseHBarGraph(Adafruit_SSD1351 *tft,int X, int Y, int height, int width,float minValue, float maxValue) {
  _locX=X;
  _locY=Y;
  _height=height;
  _width=width;
  _maxX=_locX+width;
  _maxY=_locY+height;
  _tft=tft;
  _pixelScaleFactor=(_width)/(maxValue-minValue);
  _offset=minValue;
  _limitMin=minValue;
  _limitMax=maxValue;
}

oledReverseHBarGraph::oledReverseHBarGraph() {

}

void oledReverseHBarGraph::setBarColors(int numBars, float values[], int colors[]) {
  for (int k=0; k<numBars; k++) {    
    _hBarX[k]=_locX+max(int((_limitMax-values[k])*_pixelScaleFactor+0.5f),0);
    if (_hBarX[k]<_locX) {_hBarX[k]=_locX;}
    if (_hBarX[k]>_maxX) {_hBarX[k]=_maxX;}
    _hBarValues[k]=values[k];
    _hBarColors[k]=colors[k];
    //Serial.print("Value = ");Serial.print("_hBarValues[k]");Serial.print(" Loc = ");Serial.println(_hBarY[k]);    
  }
  _numActiveBars=numBars;
}


void oledReverseHBarGraph::resetGraph() {
  _tft->fillRect(_locX, _locY, _width, _height, cBLACK);
  _barValue = _limitMax;
  _barEnd = _locX; 
}

int oledReverseHBarGraph::getLineColor(int col){
  int lineColor=cBLACK;
  for (int k=0; k<_numActiveBars; k++) {    
    if (col>_hBarX[k]) { 
      return lineColor;  //Next bar is to the left, return the prior color 
      //Serial.print("Color set = "); Serial.println(lineColor, HEX);
    } else {
      lineColor=_hBarColors[k];
    }
  }
  return _hBarColors[_numActiveBars-1]; //Return the last color bar if it's beyond it
}

int oledReverseHBarGraph::getBarEnd(float val){
  //Serial.print("Bar end = "); Serial.println(colValue);
  int colValue=_locX;
  if (val<_limitMax) {
    colValue=int((_limitMax-val)*_pixelScaleFactor+0.5f+_locX);
    colValue=min(colValue,_maxX);    
  }
  //Serial.print("Bar end = "); Serial.println(colValue);
  return colValue;
}

int oledReverseHBarGraph::getBarColor() {
  return _barColor;
}

void oledReverseHBarGraph::updateGraph(float newValue) {
  int difVal, newEnd, gColor;

  newEnd = getBarEnd(newValue);
  difVal = newEnd - _barEnd;
  if (difVal==0) return; // return if no change
  gColor = getLineColor(newEnd);
  

  //printVal(X, 0, bc, conn, newValue);
  if (newValue > _limitMax) {
    //gColor=_hBarColors[_numActiveBars-1];
    if (_barValue <= _limitMax) {
      //Serial.println("Graph Reset");      
      resetGraph(); //If we previously had a value, blank the graph
    }
    //_barColor=gColor;
    _barValue = newValue;
    _barEnd = _locX; 
    return; // no bar if beyond 32 ohms
  } 
  
  if (difVal < 0) {
    //Serial.print("New end = "); Serial.println(newEnd);
    _tft->fillRect(newEnd+1, _locY, _barEnd-newEnd, _height, cBLACK);  //Black out the prior bar     
  }
  if (gColor != _barColor) {
    _tft->fillRect(_locX, _locY, newEnd-_locX,_height,gColor);
    _barColor=gColor; 
  } else {
    if (difVal > 0)  {//increase bar graph length
      _tft->fillRect(_barEnd,_locY,difVal,_height,gColor);  
    }
  }
  _barColor=gColor;  
  _barEnd=newEnd;
  _barValue=newValue;  //and we're done, save old value
}




/* Function for adjusting the color in segments, 
 *  moving to archive
 *  

void oledReverseHBarGraph::updateGraph(float newValue) {
  int difVal, newEnd, gColor;

  //printVal(X, 0, bc, conn, newValue);
  if (newValue > _limitMax) {
    if (_barValue <= _limitMax) {
      //Serial.println("Graph Reset");
      resetGraph(); //If we previously had a value, blank the graph
    }
    _barValue = newValue;
    _barEnd = _locX; 
    return; // no bar if beyond 32 ohms
  } 

  newEnd = getBarEnd(newValue);
  difVal = newEnd - _barEnd;

  if (difVal < 0) {
    //Serial.print("New end = "); Serial.println(newEnd);
    _tft->fillRect(newEnd+1, _locY, _barEnd-newEnd, _height, cBLACK);  //Black out the prior bar     
  }  
  if (difVal > 0)  {//increase bar graph length
    //Serial.print("Bar increase = "); Serial.println(difVal);
    for (int dCol=_barEnd; dCol<=newEnd; dCol++) {
      gColor=getLineColor(dCol);
      _tft->drawFastVLine(dCol, _locY, _height, gColor);      
    }  
  } 
  _barEnd=newEnd;
  _barValue=newValue;  //and we're done, save old value
}
*/

oledGraphLabel::oledGraphLabel(Adafruit_SSD1351 *tft,uint16_t x, uint16_t y, int size, uint16_t color) {
  _tft=tft;
  _locX=x;
  _locY=y;
  _fontSize=size;
  _lblColor=color;
}
oledGraphLabel::oledGraphLabel() {

}

void oledGraphLabel::setFontSize(int size) {
  _fontSize=size;
}

void oledGraphLabel::setColors(int numBars, float values[], int colors[]){
  _numColors=numBars;
  for (int k=0;k<numBars;k++){
    _colorValues[k]=values[k];
    _colorList[k]=colors[k];
  }
  //memcpy(_colorValues,values,numBars*sizeof(values[0]));
  //memcpy(_valColors,colors,numBars*sizeof(colors[0]));  
}

void oledGraphLabel::clearLabel() {
  _tft->setTextSize(_fontSize);
  _tft->setCursor(_locX, _locY);  
  for (int k=0; k<10; k++) {
    _tft->print(" ");  
  }  
}


void oledGraphLabel::printLabel(const char *lab, float val, bool forceColor, uint16_t newColor) {
  uint16_t lblColor=cYELLOW;  
  char strBuf[5]="";

  _tft->setTextSize(_fontSize);
  _tft->setCursor(_locX, _locY);  
  
  if (val<0.0) {val =0.0f;}

  if (forceColor) {
    lblColor=newColor;
  } else {
    lblColor=cRED;
    for (int k=0; k<_numColors; k++) {
      if (val>=_colorValues[k]) {
        lblColor=_colorList[k];        
      } else {
        break;
      }
    }
  } 
  
  _tft->setTextColor(lblColor, cBLACK);
  _tft->print(lab); _tft->print("="); //display connection string
  //tft.setTextColor(valColor, BLACK);  //set the text color to right color, black background
  //Print ohms in a 4 character space
  //"XXXX", "XXX ", "XX  " or "X.XX"
  if (val > 500) {
    _tft->print("OPEN");
    //_openTxt=true;
    return;
  }

  if (val>10) {    
    dtostrf(val,-4,0,strBuf); //Don't display decimal points
  } else {
    dtostrf(val,-4,1,strBuf);
  }
  _tft->print(strBuf);     
  return;
}
