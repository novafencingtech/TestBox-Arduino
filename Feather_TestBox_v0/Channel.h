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
    static const float HIGH_GAIN;
    static const int SAMPLE_BUFFER_SIZE = 128;
    // Assuming 4.096Vref, 2kOhm resistor, G=48, 255 cnts/2.56V
    static const float LOW_GAIN; //10-bit conversion factor
    static const byte NUM_AVE_POW2 = 5; //32 Samples
    static const byte NUM_AVE = (1 << NUM_AVE_POW2);
    static const byte ADC_BIT_DEPTH = 12;
    
    ADC_Channel(byte GPIO_Num);
    ADC_Channel(byte lowADC_Num, byte highADC_Num);
    void updateVals();

    void setRangeHigh();
    void setRangeLow();
    bool isDualRange();
    bool isLowRange();
    bool isHighRange();
    
    void resetMinMaxValues();
    void setDecayRate(float value);
    float getValue();
    float getDecayMaxValue();
    int getDecayMaxRawValue();
    int getDecayMinValue();
    int getRawValue();

    void setTrim(int val);
    int getTrim();
    volatile int sampleCount=0;
    volatile int lastValue=0;
    volatile int maxval = 0;
    volatile int minval = (1<<ADC_BIT_DEPTH)-1;
    volatile long averageVal=0;

    bool bufferActive=false;
    volatile int sampleIndex=0;
    volatile int *activeBuffer;
    
    volatile long t_max = 0;
    volatile long t_min = 0;

    volatile int decay_max = 0;
    volatile int decay_min = 0;

    bool ADC_Scan = true;
    ADC_Channel* nextChannel;
    uint16_t AIn=0;
    char ch_label[4]; //Use three characters to indicate channel indices.  ch_label[0] = 'A','B','C' on primary channels; ch_label[0]=' ' on secondary channels 
    byte muxSetting; //Used for setting the external MOSFET mux

  private:
    //float _myExpDecayFcn(float val);
    float _decayTau = 150.0; //Exponential decay constant in ms
    float _gain=LOW_GAIN;
    ADCMux* _ActiveRange;
    ADCMux _highRangeADC;
    ADCMux _lowRangeADC;
    bool _isHighRange = true;
    bool _dualRangeEnabled = false;
    int _ADCOffset = 0;
};

#endif
