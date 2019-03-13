/*
class ADC_Channel
{
  struct Ardiuno_ADC {
      bool MUX5=false;
      float gain=1.0; //Scale factor for high gain mode
      byte ch_mask=0;
      
    }
  
  public:
    
    const float DECAY_TAU=500.0; //exponential decay constant in ms
    const float SCALE_FACTOR=1.0; //Conversion from ADC counts to real units
    ADC_Channel(byte ADC_Num);
    void updateVals();
    void updateDisp();
    bool getMUX5();
    void setScan(bool scan_enable);
    byte getMUX();
    void setMUX(byte MUXByteValue);
    void setRangeHigh();
    void setRangeLow();
    bool isDualRange();
      
    void setTrim(byte val);
        
    byte newval=0;
    byte maxval=0;
    byte minval=255;
    long t_max=0;
    long t_min=0;
    
    volatile bool update_max_flag=false;
    volatile bool update_min_flag=false;
        
    byte decay_max=0;
    byte decay_min=0;    
    
    bool ADC_Scan=true;
    ADC_Channel* nextChannel;
    Ardiuno_ADC* ActiveRange;
    char ch_label;
    byte muxSetting;

        
  private:
    Arduino_ADC HighRangeADC;
    Arduino_ADC LowRangeADC;
    bool _dualRangeEnabled=false;
    byte _ADC_number=0;
    byte _MUX_byte=B00000000;
};
*/
#include "Arduino.h"
#include "Channel.h"


ADC_Channel::ADC_Channel(byte ADC_Num) {
  _lowRangeADC=getADCMuxSetting(ADC_Num);
  _lowRangeADC.gain=LOW_GAIN;
  _highRangeADC=_lowRangeADC;
  _isHighRange=false;
  _dualRangeEnabled=false;  
  _ActiveRange=&(_lowRangeADC);
  
  nextChannel=this; //Point it back to itself for safety 
}

ADC_Channel::ADC_Channel(byte lowADC_Num,byte highADC_Num) {  
  _lowRangeADC=getADCMuxSetting(lowADC_Num);
  _lowRangeADC.gain=LOW_GAIN;
  
  _ActiveRange=&(_lowRangeADC);
  _highRangeADC=getADCMuxSetting(highADC_Num);
  _highRangeADC.gain=HIGH_GAIN;
  _dualRangeEnabled=true;
  _isHighRange=true;  

  nextChannel=this; //Point it back to itself for safety 
}

ADCMux ADC_Channel::getADCMuxSetting(byte ADC_Ch) {    
  ADCMux ADCMuxValue;

  ADCMuxValue.channelMask=ADC_Ch;
  ADCMuxValue.MUX5setting=0;
  ADCMuxValue.ADCNum=ADC_Ch;
  
  if (ADC_Ch>7) {
    ADCMuxValue.MUX5setting=1;
    ADCMuxValue.channelMask=ADC_Ch-8;
  }
  if (ADC_Ch>13 || (ADC_Ch==2 || ADC_Ch==3)) {
    ADCMuxValue.MUX5setting=0;
    ADCMuxValue.channelMask=B00001111; //if ADC Number out of range, set to GND or Temp(MUX5=true)
  }
  return ADCMuxValue;
}

void ADC_Channel::updateVals() {
  long t_now=millis();
  int val;
  char buf[32];


  //Calculate the box car average
  /*
  _sampleAve=0;
  for (int k=0;k<NUM_AVE;k++){
    _sampleAve+=sampleArray[k];
  }
  _sampleAve=(_sampleAve >> NUM_AVE_POW2);
  */
  _sampleAve=averageVal;

  val=(t_now-t_max)/DECAY_TAU;
  if (maxval>MAX_ADC_VAL) {
    maxval=MAX_ADC_VAL;
    t_max=t_now;
  }
  if (maxval>val) {
    decay_max=maxval-val;
  } else {
    decay_max=lastADCH;
    t_max=t_now;
    maxval=lastADCH;
  }
  if (decay_max<lastADCH) {
    decay_max=lastADCH;
    t_max=t_now;
    maxval=lastADCH;
  }
  
  /*if (ch_label[0]=='A') {
    snprintf(buf,32,"d=%u,m=%u,v=%u\r\n",decay_max,maxval,val);
    Serial.write(buf);
  }*/  
  val=(t_now-t_min)/DECAY_TAU;
  if ( (255-minval)>val) {
    decay_min=minval+val;
  } else {
    //minval=sampleArray[sampleIndex];
    minval=lastADCH;
    //decay_min=sampleArray[sampleIndex];
    t_min=t_now;
  }
  if (decay_min>lastADCH) {
    decay_min=lastADCH;
    t_min=millis();
    minval=lastADCH;
  }
}

void ADC_Channel::setTrim(byte val) {
  _ADCOffset=val;
}

byte ADC_Channel::getTrim() {
  return _ADCOffset;
}

float ADC_Channel::getValue() {
  return ( (float(_sampleAve)-_ADCOffset)*(_ActiveRange->gain));
  /*
  if (_sampleAve>_ADCOffset) {
    return (float(_sampleAve-_ADCOffset)*(_ActiveRange->gain));  
  } else {
    return (float(0.0));
  }*/
  
}

float ADC_Channel::getDecayMaxValue() {
  int temp=(decay_max<<2); //Need to left shift because max function is only the upper 8 bits
  if (temp>_ADCOffset) {
    return ( (float(temp-_ADCOffset))*(_ActiveRange->gain) );
  } else return 0.0;
}

byte ADC_Channel::getDecayMinValue() {
  return (decay_min);
}

int ADC_Channel::getRawValue() {
  return _sampleAve;
}

byte ADC_Channel::getDecayMaxRawValue() {
  return (decay_max);
}


void ADC_Channel::setRangeHigh() {
  if (_ActiveRange!=&(_highRangeADC)) {
    _ActiveRange=&(_highRangeADC);
    maxval=(_lowRangeADC.gain/_highRangeADC.gain)*(float)maxval;
    minval=(_lowRangeADC.gain/_highRangeADC.gain)*(float)minval;
    _isHighRange=true;
  }
}

void ADC_Channel::setRangeLow() {
  if (_ActiveRange!=&(_lowRangeADC)) {
    _ActiveRange=&(_lowRangeADC);
    maxval=(_highRangeADC.gain/_lowRangeADC.gain)*maxval;
    minval=(_highRangeADC.gain/_lowRangeADC.gain)*minval;
    _isHighRange=false;
  }
}

bool ADC_Channel::isHighRange(){
  if (!_dualRangeEnabled) {
    return false;
  }
  return _isHighRange;
}

bool ADC_Channel::isLowRange(){
  if (!_dualRangeEnabled) {
    return false;
  }
  return !_isHighRange;
}

void ADC_Channel::resetMinMaxValues(){
  maxval=0;
  decay_max=0;
  minval=255;
  decay_min=255;
}

void ADC_Channel::setADCChannelActive() {
  byte x;
  
  if (bitRead(ADCSRB,MUX5)!=_ActiveRange->MUX5setting) {
    bitWrite(ADCSRB,MUX5,_ActiveRange->MUX5setting);
  }
  x=(ADMUX & B11100000); //Clear the channel mask without changing other bits
  ADMUX= (x | _ActiveRange->channelMask); //Load the new channel mask
}
