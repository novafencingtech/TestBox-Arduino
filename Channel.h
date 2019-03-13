/*
  ADC_Channel.h - Class for handling ADC channels quickly

*/
#ifndef Channel_h
#define Channel_h

#include "Arduino.h"

struct ADCMux {
  byte ADCNum = 0;
  bool MUX5setting = 0;
  byte channelMask = 0;
  float gain = 1.0; //Scale factor
};



class ADC_Channel
{
  public:
    //static const int NUM_EXP_VALS=7;
    //const float expDecayFcnArg[NUM_EXP_VALS]={0.1,0.2,0.5,1.0,1.5,2.5,4};
    //const float expDecayFcnVal[NUM_EXP_VALS]={0.9,0.828,0.607,0.368,0.223,0.082,0.018};
    static const int DECAY_TAU = 5; //Reduce max by 1 cnt every XX ms
    static const float HIGH_GAIN = 4.0;
    // Assuming 4.096Vref, 2kOhm resistor, G=48, 255 cnts/2.56V
    //static const float LOW_GAIN = 0.1021; //9.792 cnt/Ohm resistance
    static const float LOW_GAIN= 0.025525; //10-bit conversion factor
    static const byte NUM_AVE_POW2 = 5; //32 Samples
    static const byte NUM_AVE = (1 << NUM_AVE_POW2);
    static const byte MAX_ADC_VAL=150;
    //const byte DECAY_RATE=1; //Subtract everytime
    ADC_Channel(byte ADC_Num);
    ADC_Channel(byte lowADC_Num, byte highADC_Num);
    void updateVals();

    void setRangeHigh();
    void setRangeLow();
    bool isDualRange();
    bool isLowRange();
    bool isHighRange();
    void setADCChannelActive();
    void resetMinMaxValues();
    float getValue();
    float getDecayMaxValue();
    byte getDecayMaxRawValue();
    byte getDecayMinValue();
    int getRawValue();
    ADCMux getADCMuxSetting(byte ADC_Ch);

    void setTrim(byte val);
    byte getTrim();

    //volatile byte newval = 0;
    volatile char sampleIndex=0;
    volatile unsigned int highSum=0;
    volatile unsigned int lowSum=0;
    volatile byte lastADCH=0;
    volatile byte lastADCL=0;
    volatile byte maxval = 0;
    volatile byte minval = 255;
    volatile long averageVal=0;
    
    volatile long t_max = 0;
    volatile long t_min = 0;

    volatile byte decay_max = 0;
    volatile byte decay_min = 0;

    bool ADC_Scan = true;
    ADC_Channel* nextChannel;
    char ch_label[4]; //Use three characters to indicate channel indices.  ch_label[0] = 'A','B','C' on primary channels; ch_label[0]=' ' on secondary channels 
    byte muxSetting;

  private:
    //float _myExpDecayFcn(float val);
    ADCMux* _ActiveRange;
    ADCMux _highRangeADC;
    ADCMux _lowRangeADC;
    int _sampleAve = 0;
    bool _isHighRange = true;
    bool _dualRangeEnabled = false;
    byte _ADC_number = 0;
    byte _MUX_byte = B00000000;
    byte _ADCOffset = 0;
};

#endif
