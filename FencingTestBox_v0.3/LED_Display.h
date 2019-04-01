//#define FASTLED_ALLOW_INTERRUPTS 0
#include <FastLED.h>


#define maxThresholds 5
#define NUM_LED_LIST_VALUES 6

class LED_Display
{

  public:
    LED_Display(CRGB LEDArr[],byte numLEDs, byte offsetLED, char displayMode);
    void setDisplayMode(char displayMode);
    void updateLED();
    void setValue(float value);
    void setColor(CRGB color);
    //void setMaxValue(float maxValue);
    //void setMaxColor(CRGB color);
    //void setMaxEnable(bool enable);
    //void setMaxBlink(bool enable);
    void setDisplayBlink(bool enable);
    //void setBlinkRate(long t_ms);
    void setScaleMax(float maxScaleValue);
    void setOffOn(bool on);
    bool isOn();
    bool setThreshold(byte thresholdNum, float value, CRGB color);
    void displayOff();
    void displayTest();
    void clearThresholds();
    void setValueListMode(bool on);
    //static const byte _numLEDValues=6;
    //static const float _LEDValueList[6]={0.5,1.0,2.0,4.0,8.0,16.0};
    //static const bool _ValueListMode=true;  

  
    
  private:
    CRGB* _baseLEDArr;
    CRGB* _LEDArr;
    CRGB _currColor;
    bool _valueEnabled;
    bool _maxEnabled=false;
    CRGB _maxColor=CRGB(20,0,20);
    bool _Enabled=true;
    char _numLEDs=1;
    char _offsetLED=0;
    char _displayMode;
    CRGB _colorThresholdValues[maxThresholds];
    float _colorThresholds[maxThresholds];
    float _currentValue=0;
    char _numThresholds=0;
    float _maxScaleValue=1.0;
    float _currentMaxValue=0.0;
    long _blinkRate=50;
    bool _blinkAll=false;
    bool _blinkMax=false;
    bool _blinkAllOff=false;
    bool _blinkMaxOff=false;
    byte _numThresholdsSet=0;
    byte _maxPixel=0;
    byte _maxThreshold=maxThresholds;
    float _stepValue=1.0;
    //float _logbase=2.0;
    //int _logoffset=-1;
    //byte _numLEDValues=6;
    bool _ValueListMode=true;
    static byte _numLEDValues;
    static float _LEDValueList[NUM_LED_LIST_VALUES];
    
    //void updateMaxPixel();
    void setSingleLEDValue(float value);
    void setGraphLEDValue(float value);
    void setBarLEDValue(float value);
};
