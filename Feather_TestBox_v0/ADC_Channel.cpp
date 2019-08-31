/*
class ADC_Channel

*/
#include "Channel.h"

const float ADC_Channel::HIGH_GAIN= 0.234375f;  // Assuming 2.50Vref, 1kOhm resistor, G=1, 4095 cnts/2.4V G=1/4; 0.6Vref
const float ADC_Channel::LOW_GAIN= 0.011160714f; // Assuming 2.50Vref, 1kOhm resistor, G=21, 4095 cnts/2.4V G=1/4; 0.6Vref
q31_t ADC_Channel::DECI_FILTER[NUM_DECIMATION_TAPS]={21315851,55216453,109109079,178053560,248292458,301254934,320998979,
                              301254934,248292458,178053560,109109079,55216453,21315851};
//const q31_t ADC_Channel::EXP_DECAY[16]={26828,21965,17983,14724,12055,9870,8080,6616,5417,4435,3631,2973,2434,1993,1631,128};
//const q31_t ADC_Channel::EXP_DECAY[16]={1943123559,1758210904,1590895015,1439501338,1302514674,1178564014,1066408820,964926603,
 //                                         873101696,790015084,714835209,646809645,585257569,529562948,479168370,1024};
const float ADC_Channel::DECAY_TAU=200.0f;



ADC_Channel::ADC_Channel(byte AnalogIn) {
  AIn=AnalogIn+1; //+1 Required because 0=Not Connected
  _lowRangeAIn=AIn;
  _dualRangeEnabled=false;  

  arm_fir_decimate_init_q31(&(this->FIR_filter),NUM_DECIMATION_TAPS,FIR_BLOCK_SIZE,(this->DECI_FILTER),this->FIRState,FIR_BLOCK_SIZE);
    
  nextChannel=this; //Point it back to itself for safety 
}

ADC_Channel::ADC_Channel(byte lowADC_Num,byte highADC_Num) {  
  _lowRangeAIn=lowADC_Num+1; //+1 Required because 0=Not Connected
  _highRangeAIn=highADC_Num+1;
  
  AIn=_lowRangeAIn;
  _dualRangeEnabled=true;
  _isHighRange=false;

  arm_fir_decimate_init_q31(&(this->FIR_filter),NUM_DECIMATION_TAPS,FIR_BLOCK_SIZE,(this->DECI_FILTER),this->FIRState,FIR_BLOCK_SIZE);

  nextChannel=this; //Point it back to itself for safety 
}

void ADC_Channel::updateVals() {
  long t_now=millis();
  int val;
  //char buf[32];
  
  if (maxval>filterValue) {
    decay_max=int(float(maxval-filterValue)*exp((t_max-t_now)/DECAY_TAU))+filterValue;
    //decay_max=decay_max-(maxval-averageVal)*(t_now-t_max)/_decayTau;
    //decay_max--;
  }
  
  if (decay_max<=lastValue) {
    decay_max=lastValue;
    t_max=t_now;
    maxval=lastValue;
  } 

  if (minval<filterValue) {
    decay_min=int(float(minval-filterValue)*exp((t_min-t_now)/DECAY_TAU))+filterValue;
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
  return ( (float(filterValue)-_ADCOffset)*(_gain));  
}

float ADC_Channel::getDecayMaxValue() {
  if (decay_max>_ADCOffset) {
    return ( (float(decay_max-_ADCOffset))*(_gain) );
  } else return 0.0;
}

int ADC_Channel::getDecayMinValue() {
  return (decay_min);
}

int ADC_Channel::getRawValue() {
  return filterValue;
}

int ADC_Channel::getDecayMaxRawValue() {
  return (decay_max);
}


void ADC_Channel::setRangeHigh() {
  _gain=HIGH_GAIN;
}

void ADC_Channel::setRangeLow() {
  _gain=LOW_GAIN;
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
