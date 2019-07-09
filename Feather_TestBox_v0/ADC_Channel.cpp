/*
class ADC_Channel

*/
#include "Channel.h"

const float ADC_Channel::HIGH_GAIN= 4.0f;
const float ADC_Channel::LOW_GAIN= 0.003843f; // Assuming 2.50Vref, 2kOhm resistor, G=50, 4095 cnts/1.5V

ADC_Channel::ADC_Channel(byte AnalogIn) {
  AIn=AnalogIn+1; //+1 Required because 0=Not Connected
  _lowRangeADC.gain=LOW_GAIN;
  _highRangeADC=_lowRangeADC;
  _isHighRange=false;
  _dualRangeEnabled=false;  
  _ActiveRange=&(_lowRangeADC);
  
  nextChannel=this; //Point it back to itself for safety 
}

ADC_Channel::ADC_Channel(byte lowADC_Num,byte highADC_Num) {  
  _lowRangeADC.gain=LOW_GAIN;
  
  _ActiveRange=&(_lowRangeADC);
  _highRangeADC.gain=HIGH_GAIN;
  _dualRangeEnabled=true;
  _isHighRange=true;  

  nextChannel=this; //Point it back to itself for safety 
}

void ADC_Channel::updateVals() {
  long t_now=millis();
  int val;
  char buf[32];

  //Calculate the box car average
  //_sampleAve=averageVal;

  if (maxval>averageVal) {
    decay_max=int(float(maxval-averageVal)*exp(-1.0f*(t_now-t_max)/_decayTau))+averageVal;
    //decay_max=decay_max-(maxval-averageVal)*(t_now-t_max)/_decayTau;
    //decay_max--;
  }
  
  if (decay_max<=lastValue) {
    decay_max=lastValue;
    t_max=t_now;
    maxval=lastValue;
  } 

  if (minval<averageVal) {
    decay_min=int(float(minval-averageVal)*exp(-1.0f*(t_now-t_min)/_decayTau))+averageVal;
    //decay_min=decay_min+(averageVal-minval)*(t_now-t_min)/_decayTau;
    //decay_min++;
  }
  if (decay_min>=lastValue) {
    minval=lastValue;
    decay_min=lastValue;
    t_min=t_now;
  }
}

void ADC_Channel::setTrim(int val) {
  _ADCOffset=val;
}

int ADC_Channel::getTrim() {
  return _ADCOffset;
}

float ADC_Channel::getValue() {
  return ( (float(averageVal)-_ADCOffset)*(_ActiveRange->gain));  
}

float ADC_Channel::getDecayMaxValue() {
  if (decay_max>_ADCOffset) {
    return ( (float(decay_max-_ADCOffset))*(_ActiveRange->gain) );
  } else return 0.0;
}

int ADC_Channel::getDecayMinValue() {
  return (decay_min);
}

int ADC_Channel::getRawValue() {
  return averageVal;
}

int ADC_Channel::getDecayMaxRawValue() {
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
  minval=(1<<12)-1;
  decay_min=(1<<12)-1;
}
