#include "oledGraphClass.h"

//Sets the main buffer
oledGraph::oledGraph(Adafruit_ILI9341 *tft, int X, int Y, int height, int width, float minValue, float maxValue) {
  _locX = X;
  _locY = Y;
  _height = height;
  _width = width;
  _maxX = _locX + width;
  _maxY = _locY + height;
  _tft = tft;
  _pixelScaleFactor = (_height) / (maxValue - minValue);
  _offset = minValue;
  _limitMin = minValue;
  _limitMax = maxValue;
}

oledGraph::oledGraph() {}


void oledGraph::setVerticalLimits(float minValue, float maxValue) {
  if (maxValue >= minValue) return;
  _pixelScaleFactor = (_height) / (maxValue - minValue);
  _offset = minValue;
  _limitMin = minValue;
  _limitMax = maxValue;
}

void oledGraph::setHorizontalBarValues(int numBars, float values[], int colors[]) {

  for (int k = 0; k < numBars; k++) {
    _hBarY[k] = _maxY - max(int((values[k] - _offset) * _pixelScaleFactor + 0.5f), 0);
    if (_hBarY[k] < _locY) {
      _hBarY[k] = _locY;
    }
    if (_hBarY[k] > _maxY) {
      _hBarY[k] = _maxY;
    }
    _hBarValues[k] = values[k];
    _hBarColors[k] = colors[k];
    //Serial.print("Value = ");Serial.print("_hBarValues[k]");Serial.print(" Loc = ");Serial.println(_hBarY[k]);
  }
  _numActiveBars = numBars;
}

void oledGraph::drawTextLabels() {
  _tft->setTextSize(2);
  _tft->setTextColor(cCYAN);
  for (int k = 0; k < _numActiveBars; k++) { //top bar plotted differently
    _tft->setCursor(_maxX - 25, _hBarY[k] - 15);
    _tft->print(_hBarValues[k]);
  }

  //_tft->setCursor(_maxX-25, _hBarY[_numActiveBars-1]+15);
  //_tft->print(_hBarValues[_numActiveBars-1]);
}

void oledGraph::drawHLines() {

  for (int k = 0; k < _numActiveBars; k++) {
    _tft->drawFastHLine(_locX, _hBarY[k], _maxX - _locX - 1, cWHITE);
    //Serial.print("Value = ");Serial.print(_hBarValues[k]);Serial.print(" Loc = ");Serial.println(_hBarY[k]);
    //Serial.print("Drawing Line @ ");Serial.println(_hBarY[k]);
  }
  //_tft->drawPixel(_col, _maxY, cWHITE); //draw bottom scale (9.9 ohms), could be erased later
}

void oledGraph::resetGraph() {
  _tft->fillRect(_locX, _locY, _width, _height, cBLACK);
  drawHLines();
  drawTextLabels();
  _col = _locX;
}

void oledGraph::updateGraph(float newValue) {
  if (_col == _locX) {
    drawHLines();
    drawTextLabels();
  }
  drawColumn(newValue);

  if (++_col > (_locX + _width)) {
    _col = _locX;
  }
}

void oledGraph::drawColumn(float val) {
  static int oldRow = 0;
  int valRow;

  //int valRow = 127 - min(val, 99); //convert ohms in tenths to row
  //int valRow = _locY + (_pixelScaleFactor*(val-_offset)); //convert ohms in tenths to row
  valRow = _maxY - int(_pixelScaleFactor * (val - _offset) - 0.5f);
  valRow = max(valRow, _locY);
  valRow = min(valRow, _maxY);

  int topRow, botRow;
  //sort ends so we're always drawing from bottom to top (large to small values)
  if (valRow > oldRow) {
    topRow = oldRow;
    botRow = valRow;
  } else {
    topRow = valRow;
    botRow = oldRow;
  }

  if (_col == _locX) { //Starting over, no need to draw a line
    topRow = valRow;
    botRow = valRow;
  }

  topRow = max(topRow, _locY);
  botRow = min(botRow, _maxY);
  _tft->drawFastVLine(_col, _locY, _maxY - _locY + 1, cBLACK); //clear out the column

  //Re-draw the horizontal lines
  for (int k = 0; k < _numActiveBars; k++) {
    _tft->drawPixel(_col, _hBarY[k], cWHITE);
  }

  int curRow = topRow;
  int endRow;
  bool lineDrawn = false;
  for (int k = _numActiveBars - 1; k > 0; k--) { //Draw from top down
    if ( (curRow >= _hBarY[k]) && (curRow <= _hBarY[k - 1]) )  { //Indexes down, so below this bar
      if (topRow == botRow) {
        _tft->drawPixel(_col, curRow, _hBarColors[k - 1]);
        _tft->drawPixel(_col, curRow + 1, _hBarColors[k - 1]);
        lineDrawn = true;
        break;
      }
      endRow = min(_hBarY[k - 1], botRow);
      _tft->drawFastVLine(_col, curRow, endRow - curRow + 1, _hBarColors[k - 1]); //
      curRow = endRow;
      lineDrawn = true;
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
  oldRow = valRow;

  /*if (botRow < _maxY) { //fill black to bottom
    _tft->drawFastVLine(_col, botRow + 1, (_maxY-(botRow+1)), cBLACK);
      for (int k=0; k<_numActiveBars; k++) {
        if (botRow<_hBarY[k]) {
          _tft->drawPixel(_col, _hBarY[k], cWHITE);
        }
      }
    _tft->drawPixel(_col, _maxY, cWHITE); //draw bottom scale (9.9 ohms), could be erased later
    }*/

  if (_col < (_locX + _width) ) {
    int irow = max(valRow - 1, _locY);
    //    if (val>18) {Serial.print(icol);Serial.print("=");;Serial.print(irow);Serial.println(",CYAN");}
    _tft->drawFastVLine(_col + 1, irow, 3, cCYAN);
    _tft->drawFastVLine(_col + 2, irow, 3, cCYAN);
  }  //show where we are as 2x1 cyan line at col+1

}

oledBarGraph::oledBarGraph() {

}


oledBarGraph::oledBarGraph(Adafruit_ILI9341 *tft, int X, int Y, int height, int width, float minValue, float maxValue)
{
  _tft = tft;
  _locX = X;
  _locY = Y;
  _prevEnd=_locX;
  _titleTextPosX = _locX;
  _titleTextPosY = _locY - 32;
  _valTextPosX = _locX + 80;
  _valTextPosY = _titleTextPosY;

  _height = min(height, _maxY - _locY);
  _width = min(width, _maxX - _locX);
  setGraphLimits(minValue, maxValue);
}
void oledBarGraph::setGraphLimits(float minValue, float maxValue) {
  _limitMin = minValue;
  _limitMin = maxValue;
  _pixelScaleFactor = (float)_width / (maxValue - minValue);
  _offset = minValue;
}

void oledBarGraph::setHorizontalBarValues(int numBars, float values[], int colors[]) {

  for (int k = 0; k < numBars; k++) {
    _hBarX[k] = _maxX - max(int((values[k] - _offset) * _pixelScaleFactor + 0.5f), 0);
    if (_hBarX[k] < _locX) {
      _hBarX[k] = _locX;
    }
    if (_hBarX[k] > _maxX) {
      _hBarX[k] = _maxX;
    }
    _hBarValues[k] = values[k];
    _hBarColors[k] = colors[k];
    //Serial.print("Value = ");Serial.print("_hBarValues[k]");Serial.print(" Loc = ");Serial.println(_hBarY[k]);
  }
  _numActiveBars = numBars;
}
void oledBarGraph::updateGraph(float newValue) {
  bool newColor = false;
  int barColor = 0;
  int newEnd = 0;
  //static int prevEnd = 0;
  char buf[64];

  for (int k = _numActiveBars - 1; k >= 0; k--) {
    barColor = _hBarColors[k];
    if (newValue > _hBarValues[k]) {
      break;
    }
  }

  //Check if we assigned a new Color
  newColor = !(barColor == _prevColor);

  newEnd = int((newValue + _offset) * _pixelScaleFactor + 0.5) + _locX;
  newEnd = min(newEnd, _width+_locX);
  newEnd = max(newEnd, _locX);

  //If too high display nothing
  /*
  if (newValue > _noDisplayLimit) {
    _tft->fillRect(_locX, _locY, _width, _height, cBLACK);
    _prevColor = cBLACK;
    newColor = false;
    _prevEnd = _locX;
    updateTextValue(newValue);
    return;
  }*/


  //If we change color, erase and redraw the bar
  if (newColor) {
    //_tft->fillRect(_locX, _locY, _width, _height, cBLACK);
    _tft->fillRect(_locX, _locY, newEnd-_locX, _height, barColor);
    _prevColor = barColor;
    //prevEnd = _locX;
  }
  //sprintf(buf, "new=%d, prev=%d",newEnd,_prevEnd);
  //Serial.println(buf);
  if (newEnd >= _prevEnd) {
    _tft->fillRect(_prevEnd, _locY, newEnd - _prevEnd, _height, barColor);
    //_tft->fillRect(0, _locY, newEnd, _height, barColor);
  } else {
    _tft->fillRect(newEnd, _locY, _prevEnd - newEnd, _height, cBLACK);
  }
  _prevEnd = newEnd;
  _prevColor = barColor;
  updateTextValue(newValue);

}

void oledBarGraph::resetGraph() {
  setTitle("");
  drawTickMarks();
  updateGraph(1.0e37f);

}
void oledBarGraph::drawTickMarks() {
  int tickLoc = 0;
  char buf[8];
  int tickValue = 0;

  _tft->setTextSize(1);
  _tft->setTextColor(cWHITE, cBLACK);
  _tft->setFont();

  for (int k = 0; k < _numActiveBars; k++) {
    tickLoc = int( (_hBarValues[k] + _offset) * _pixelScaleFactor + 0.5);
    _tft->drawFastVLine(tickLoc + _locX, _locY - 2, _height + 10, cCYAN);
    _tft->setCursor(tickLoc + 2 + _locX, _locY + _height + 2);

    tickValue = int(_hBarValues[k]);
    if (_hBarValues[k] != tickValue) {
      dtostrf(_hBarValues[k], 4, 1, buf);
    } else {
      sprintf(buf, "%d", tickValue);
    }
    _tft->print(buf);
  }
}

void oledBarGraph::setTitle(char *title) {
  //const int sizeX=74;
  //const int sizeY=26;
  static char *oldFault;
  //static GFXcanvas1 canvas(sizeX, sizeY);
  static int titleColor = cBLACK;
  //int16_t  x1, y1;
  //uint16_t w, h;
  //char buf[32];

  //canvas.setFont(&FreeSans18pt7b);
  //canvas.getTextBounds(title,0,sizeY-1,&x1,&y1,&w,&h);
  //_valTextPosX=_titleTextPosX+w+8;
  
  if ((strncmp(_title, title, TITLE_BUF_SIZE - 1) == 0)) {
    if (titleColor==_prevColor) {
      return;
    }
  }

  //canvas.getTextBounds(_title,0,sizeY-1,&x1,&y1,&w,&h);  
  //canvas.fillRect(x1,y1,w,h,cBLACK);

  _tft->setCursor(_titleTextPosX, _titleTextPosY);
  _tft->setTextColor(cBLACK,cBLACK);
  for (int k=0; k<strlen(_title); k++) {
    _tft->print(" ");  
  }
  
  //canvas.getTextBounds("OPEN",0,sizeY-1,&x1,&y1,&w,&h);
  //sprintf(buf,"w=%u,h=%u",w,h);
  //Serial.println(buf);
  
  snprintf(_title, TITLE_BUF_SIZE - 1, "%s", title); //Copy the new title to our buffer

  //canvas.fillRect(0,0,sizeX,20,cBLACK);//Assume 7 character title max 
  //canvas.setFont(&FreeSans18pt7b);
  if (_prevColor==cBLACK) {
    titleColor=cRED;
    _tft->setTextColor(cRED, cBLACK);
  } else {
    titleColor=_prevColor;
    _tft->setTextColor(_prevColor, cBLACK);
  }  
  //canvas.setCursor(0, sizeY-1);
  _tft->setTextSize(_txtSize);
  _tft->setCursor(_titleTextPosX, _titleTextPosY);
  _tft->print(_title);
  
  //_valTextPosX=_locX+strlen(_title)*4+2;

  //_tft->drawBitmap(_titleTextPosX, _titleTextPosY, canvas.getBuffer(), sizeX, sizeY, titleColor, cBLACK);
  titleColor = _prevColor;
}

void oledBarGraph::updateTextValue(float value) {
  //const int sizeX=80;
  //const int sizeY=32;
  //static GFXcanvas1 canvas(sizeX, sizeY);
  int val, dec;
  char buf[6];
    
  val = floor(value + 0.05); //Account for rounding 0.95 -> 1
  dec = floor((value - val) * 10 + 0.5);
  snprintf(buf, 5, "%2d.%d", val, dec);

  _tft->setFont();
  _tft->setTextSize(3);
  _tft->setCursor(_valTextPosX,_valTextPosY);
  //_tft->print("    ");

  //canvas.fillRect(0,0,66,24,cBLACK); //Clear ##.# size block
  //_tft->drawRect(_valTextPosX,_valTextPosY,(6)*4*_txtSize,8*_txtSize,cCYAN);//Assume 7 character title max
  //_tft->drawRect(_valTextPosX,_locY+_lblOffset-16,(5)*12,16,cCYAN);
  
  //canvassetFont();
  //_tft->setTextSize(_txtSize);

  //canvas.setCursor(0, sizeY-1);  
  _tft->setCursor(_valTextPosX,_valTextPosY);
  if (value > _noDisplayLimit) {
    _tft->setTextColor(cRED,cBLACK);    
    _tft->print("open");
    //_tft->drawBitmap(_valTextPosX, _valTextPosY, canvas.getBuffer(), sizeX, sizeY, cRED, cBLACK);
  } else {    
    _tft->setTextColor(_prevColor,cBLACK);    
    _tft->print(buf);
    //canvas.print(buf);
    //_tft->drawBitmap(_valTextPosX, _valTextPosY, canvas.getBuffer(), sizeX, sizeY, _prevColor, cBLACK);
  }
  
  
}

void oledBarGraph::setTitlePosition(int X, int Y) {
  _titleTextPosX = X;
  _titleTextPosY = Y;
  _valTextPosX=_titleTextPosX+50;
  _valTextPosY= Y;
  //setTitle(_title);
}

void oledBarGraph::setTitleSize(int txtSize) {
  _txtSize = txtSize;
}
