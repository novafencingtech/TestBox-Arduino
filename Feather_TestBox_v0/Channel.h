/*
  ADC_Channel.h - Class for handling ADC channels quickly

*/
#ifndef Channel_h
#define Channel_h

#include "Arduino.h"

#ifndef arm_math_h
  #define ARM_MATH_CM4
  #include <arm_math.h>
#endif
//struct ADCMux {
//  byte ADCNum = 0;
//  bool MUX5setting = 0;
//  byte channelMask = 0;
//  float gain = 1.0; //Scale factor
//};

#define DECIMATION_FACTOR 13
#define NUM_DECIMATION_TAPS 13
#define STATE_LENGTH 25 //NumTaps+blockSize-1


class ADC_Channel
{
  public:
    static const float HIGH_GAIN;
    //static const int SAMPLE_BUFFER_SIZE = 32;
    //static const int NUM_BUFFERS=4;
    // Assuming 4.096Vref, 2kOhm resistor, G=48, 255 cnts/2.56V
    static const float LOW_GAIN; //10-bit conversion factor
    static const byte ADC_BIT_DEPTH = 12;
    static q31_t DECI_FILTER[NUM_DECIMATION_TAPS];
    static const byte FIR_BLOCK_SIZE=DECIMATION_FACTOR;
    //static const q31_t EXP_DECAY[16];
    static const float DECAY_TAU; //Exponential decay constant in ms

    
    ADC_Channel(byte AnalogIn);
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
    int sampleCount=0;
    int lastValue=0;
    volatile int maxval = 0;
    volatile int minval = (1<<ADC_BIT_DEPTH)-1;
    q31_t sampleBuffer[DECIMATION_FACTOR];    
    volatile bool valueReady=false;
    arm_fir_decimate_instance_q31 FIR_filter;
    q31_t FIRState[STATE_LENGTH];
    q31_t filterValue=0;

    bool captureEnabled=false;
    bool captureActive=false;
    bool captureDone=false;
    int bufferIndex=0;
    int *captureBuffer;
    int *trigBuffer[2];
    long triggerTime=0;
    
    
    
    volatile long t_max = 0;
    volatile long t_min = 0;

    volatile int decay_max = 0;
    volatile byte DecayIndex =0; 
    volatile int decay_min = 0;

    bool ADC_Scan = true;
    ADC_Channel* nextChannel;
    uint16_t AIn=0;
    char ch_label[4]; //Use three characters to indicate channel indices.  ch_label[0] = 'A','B','C' on primary channels; ch_label[0]=' ' on secondary channels 
    byte muxSetting; //Used for setting the external MOSFET mux
    
    //int ringBuffer[NUM_BUFFERS][SAMPLE_BUFFER_SIZE];
  private:
    //float _myExpDecayFcn(float val);
    float _gain=LOW_GAIN;
    byte _lowRangeAIn=0;
    byte _highRangeAIn=0;
    bool _isHighRange = true;
    bool _dualRangeEnabled = false;
    int _ADCOffset = 0;
};

#endif
