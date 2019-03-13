/*
  class LED_Display
  {

  public:
    LED_Display(CRGB LEDArr[],byte numLEDs, byte offsetLED, char displayMode);
    void setDisplayMode(char displayMode);
    void updateLED();
    void setValue(float value);
    void setColor(CRGB color);
    void setMaxValue(float maxValue);
    void setMaxColor(CRGB color);
    void setMaxEnable(bool enable);
    void setMaxBlink(bool enable);
    void setDisplayBlink(bool enable);
    void setScaleMax(float maxScaleValue);
    bool setThreshold(byte thresholdNum, float value, CRGB color);
    void displayOff();
    void displayTest();

  private:
    CRGB* _baseLEDArr;
    CRGB* _LEDArr;
    bool _maxEnabled=false;
    CRGB _maxColor=CRGB(0,0,30);
    char _numLEDs=1;
    char _offsetLED=0;
    char _displayMode;
    float _colorThresholds[maxThresholds];
    CRGB _colorThresholdValues[maxThresholds];
    float _currentValue=0;
    char _numThresholds=0;
    float _maxScaleValue=1.0;
    float _currentMaxValue=0.0;
    long _blinkRate=100;
    bool _blinkAll=false;
    bool _blinkMax=true;
    bool _blinkAllOff=false;
    bool _blinkMaxOff=false;
    byte _numThresholdsSet=0;
    byte _maxPixel=0;
    byte _maxThreshold=maxThresholds;
    float _stepValue=1.0;

    void updateMaxPixel();
    void setSingleLEDValue(float value);
    void setGraphLEDValue(float value);
    void setBarLEDValue(float value);
  };
*/

#include "LED_Display.h"

// FastLEDArray should be initialized prior to this function call
LED_Display::LED_Display(CRGB LEDArr[], byte numLEDs, byte offsetLED, char displayMode) {
  _numLEDs = numLEDs;
  _LEDArr = &(LEDArr[offsetLED]);
  _baseLEDArr = LEDArr;
  _offsetLED = offsetLED;
  _displayMode = displayMode;
  _numThresholdsSet = 1;
  _colorThresholds[0] = 1e10;
  //_LEDValueList={0.5,1.0,2.0,4.0,8.0,16.0};
  switch (displayMode) {
    case 'g':
      _maxEnabled = true;
      _blinkMax = false;
      break;
    case 'b':
      _maxEnabled = false;
      _blinkMax = false;
      break;
    case 's':
      _maxEnabled = false;
      _blinkMax = false;
      break;
  }
};

static byte LED_Display::_numLEDValues = NUM_LED_LIST_VALUES;
static float LED_Display::_LEDValueList[] = {1.0, 2.0, 4.0, 8.0, 16.0, 32.0};


void LED_Display::setDisplayMode(char displayMode) {
  _displayMode = displayMode;
};

void LED_Display::setMaxBlink(bool enable) {
  _blinkMax = enable;
}

void LED_Display::setDisplayBlink(bool enable) {
  _blinkAll = enable;
}
/*
  void LED_Display::setBlinkRate(long t_ms) {
  _blinkRate = t_ms;
  }*/

void LED_Display::setValueListMode(bool state) {
  _ValueListMode = state;
}

void LED_Display::setOffOn(bool state) {
  _Enabled = state;
  updateLED();
}

bool LED_Display::isOn() {
  return _Enabled;
}

void LED_Display::updateLED() {
  long t_now = millis();
  static long tMaxBlink;
  static long tDisplayBlink;
  static bool maxOff = false;
  static bool displayOff = false;

  if (!_Enabled) {
    displayOff = true;
  }
  if (_blinkAll) {
    if ((t_now - tDisplayBlink) > _blinkRate) {
      if (displayOff && _Enabled) {
        displayOff = false;
      } else {
        displayOff = true;
      }
      tDisplayBlink = t_now;
    }
  } else {
    if (_Enabled) {
      displayOff = false;
    }
  }
  if (displayOff) {
    for (byte k = 0; k < _numLEDs; k++) {
      _LEDArr[k] = CRGB::Black;
    }
  } else {
    if (_valueEnabled) {
      setValue(_currentValue);
      setMaxValue(_currentMaxValue);
      if ((_blinkMax) && ((t_now - tMaxBlink) > _blinkRate)) {
        if (maxOff) {
          //_LEDArr[_maxPixel] = _maxColor;
          maxOff = false;
        } else {
          _LEDArr[_maxPixel] = CRGB::Black;
          maxOff = true;
        }
        tMaxBlink = t_now;
      }
    } else {
      setColor(_currColor);
    }
  }
}


void LED_Display::setMaxEnable(bool enable) {
  _maxEnabled = enable;
}

void LED_Display::setValue(float value) {
  _valueEnabled = true;
  switch (_displayMode) {
    case 's':
      setSingleLEDValue(value);
      break;
    case 'b':
      setBarLEDValue(value);
      break;
    case 'g':
      setGraphLEDValue(value);
      break;
  }
};


void LED_Display::setColor(CRGB color) {
  _valueEnabled = false;
  for (byte k = 0; k < _numLEDs; k++) {
    _LEDArr[k] = color;
  }
  _currColor = color;
}
/*
  void LED_Display::setMaxColor(CRGB color) {
  _maxColor = color;
  }*/

void LED_Display::setMaxValue(float val) {
  if (val < 0) {
    val = 0.0;
  }
  _maxEnabled = true;
  _currentMaxValue = val;
  updateMaxPixel();
}

bool LED_Display::setThreshold(byte thresholdNum, float value, CRGB color) {
  if (thresholdNum >= _maxThreshold) {
    return false;
  }
  if (thresholdNum <= (_numThresholdsSet)) {
    _colorThresholds[thresholdNum] = value;
    _colorThresholdValues[thresholdNum] = color;
  }
  if (thresholdNum == _numThresholdsSet) {
    _numThresholdsSet++;
  }
  return true;
}

void LED_Display::clearThresholds() {
  _numThresholdsSet = 0;
}


void LED_Display::setScaleMax(float maxScaleValue) {
  _maxScaleValue = maxScaleValue;
  _stepValue = _maxScaleValue / _numLEDs;
}

void LED_Display::setSingleLEDValue(float value) {
  for (byte j = _numThresholdsSet - 1; j >= 0; j--) {
    if (_colorThresholds[j] > value)  {
      _LEDArr[0] = _colorThresholdValues[j];
    } else break;
  }
  _currentValue = value;
};

void LED_Display::setBarLEDValue(float value) {
  for (byte j = _numThresholdsSet - 1; j >= 0; j--) {
    if (_colorThresholds[j] > value)  {
      _LEDArr[0] = _colorThresholdValues[j];
    } else break;
  }
  for (byte k = 1; k < _numLEDs; k++) {
    _LEDArr[k] = _LEDArr[0];
  }
  _currentValue = value;
}

void LED_Display::setGraphLEDValue(float value) {
  if (value < 0) {
    value = 0.0;
  }
  int k_max = floor(value / _stepValue);
  float frac = (value / _stepValue - k_max);
  float LEDval = 0;
  int k = 0;

  if (_ValueListMode) {
    k_max = 0;
    for (k = 0; k < _numLEDValues; k++) {
      //Serial.println(k);
      if (value > _LEDValueList[k]) {
        k_max = k;
        //Serial.print("km = ");Serial.println(k);
      }
    }
  }

  if (k_max <= 0) {
    k_max = 0;
  }
  if (k_max >= _numLEDs) {
    k_max = _numLEDs - 1;
  }
  for (k = 0; k < _numLEDs; k++) {
    if (_ValueListMode) {
      if (k > (_numLEDValues - 1)) {
        LEDval = _LEDValueList[(_numLEDValues - 1)];
      } else {
        LEDval = _LEDValueList[k];
      }
    } else {
      LEDval = (k + 1) * _stepValue;
    }
    if (k > k_max) {
      _LEDArr[k] = CRGB::Black;
    } else {
      _LEDArr[k] = _colorThresholdValues[_numThresholdsSet - 1];
      for (byte j = _numThresholdsSet - 1; j >= 0; j--) {
        if (_colorThresholds[j] > LEDval)  {
          _LEDArr[k] = _colorThresholdValues[j];
        } else break;
      }
    }
  }
  //Serial.print(F("Base = "));Serial.print(_LEDArr[k_max].r);
  if ((frac < 0.5) && !(_ValueListMode)) {
    _LEDArr[k_max].r = _LEDArr[k_max].r >> 2;
    _LEDArr[k_max].g = _LEDArr[k_max].g >> 1;
    _LEDArr[k_max].b = _LEDArr[k_max].b >> 1;
  }
  if (k_max == 0) {
    _LEDArr[k_max] = _colorThresholdValues[0];
  }
  //Serial.print(F("Scaled = "));Serial.println(_LEDArr[k_max].r);
  _currentValue = value;
};

void LED_Display::updateMaxPixel() {
  int km = 0;
  _maxPixel = 0;
  //char buf[64];

  if (_maxEnabled) {
    //updateLED();
    //dtostrf(_currentMaxValue,4,2,buf);
    //Serial.println(buf);
    if (_ValueListMode) {
      if (_currentMaxValue > _LEDValueList[_numLEDValues - 1]) {
        _LEDArr[_numLEDValues - 1] = _maxColor;
        _maxPixel = _numLEDValues - 1;
        return;
      }
      for (int k = (_numLEDValues - 1); k >= 0; k--) {
        //Serial.println(k);
        if (_currentMaxValue > _LEDValueList[k]) {
          km = k + 1;
          break;
        }
      }
    } else {
      km = floor(_currentMaxValue / _stepValue);
    }

    _maxPixel = km;
    //Always show the max if it would be off-scale
    if (km > (_numLEDs - 1)) {
      km = _numLEDs - 1;
      _LEDArr[km] = _maxColor;
      _maxPixel = km;
    }
    //Only highlight the max when it's over a black pixel, otherwise assume not shown
    if ((_LEDArr[km].r == 0) && (_LEDArr[km].g == 0) && (_LEDArr[km].b == 0)) {
      _LEDArr[km] = _maxColor;
      _maxPixel = km;
    }

  }
}
