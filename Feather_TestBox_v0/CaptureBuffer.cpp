
#include "CaptureBuffer.h"





//Sets the main buffer
void CaptureBuffer::SetBuffers(int *mainBuffer, int bufferSize, int *preTrigger1, int *preTrigger2, int preTriggerLen) {
  _capBufSize = bufferSize;
  _preTrigSize = preTriggerLen;
  _captureBuffer = mainBuffer;
  _trigBuffer[0] = preTrigger1;
  _trigBuffer[1] = preTrigger2;
}

int* CaptureBuffer::GetBuffer(int *samplesInBuffer) {
  *samplesInBuffer = _capBufSize;
  return _captureBuffer;
}

void CaptureBuffer::setTrigger(int value, bool TriggerHigh) {
  _trigValue = value;
  _trigHigh = TriggerHigh;
}
void CaptureBuffer::setTrigger(int value, bool TriggerHigh, long debounce) { //debounce in us
  _trigDebounce = debounce;
  setTrigger(value, TriggerHigh);
}

bool CaptureBuffer::CheckTrigger(int val) {
  if (_trigHigh) {return (val>_trigValue);}
  return (val<_trigValue);
}

bool CaptureBuffer::CheckTriggerLastSample() {
  if (_trigHigh) {return (_lastValue>_trigValue);}
  return (_lastValue<_trigValue);
}

long CaptureBuffer::getLastStateChange() {
  return _tStateChange;
}

long CaptureBuffer::getLastTriggerMs() {
  return _trigTime_ms;
}

void CaptureBuffer::AddSample(int value) {
  _lastValue=value;
  static bool oldTriggerState=false;
  bool triggerValue=CheckTrigger(value);

  if (triggerValue!=oldTriggerState) {
    _tStateChange=millis();
    oldTriggerState=triggerValue;
  }
  
  switch (triggerState) {
  case 'i': //Trigger initiated but not confirmed
  case 'w': // Waiting for trigger
    (_trigBuffer[_tBufNum])[_ptIndx] = value;
      _ptIndx++;
      if (_ptIndx == _preTrigSize) {
        _ptIndx = 0;
        _tBufNum = _tBufNum ^ 0x1; //XOR to toggle between buffer 0 and buffer 1
      }
      if (triggerValue) {
        if ( (triggerState == 'i') && ((micros() - _trigTime) > _trigDebounce) ) { // If trigger confirmed goto trigger mode
          _capIndx = _preTrigSize + _ptIndx;
          triggerState = 't';
        } else {
          triggerState = 'i';
          _trigTime = micros();
          _trigTime_ms = millis();
        }
      } else {
        //CheckTrigger == false
        triggerState = 'w'; //If trigger !valid set to wait state
      }
      break;
    case 't':
      _captureBuffer[_capIndx] = value;
      _capIndx++;
      if (_capIndx >= _capBufSize) { //Capture buffer is full, trigger complete
        triggerState = 'd';
        FinalizeBuffer();  //Finalize the capture buffer by adding the pre-triggervalues;
      }
      break;
    case 'd':
      break;
  }
}

void CaptureBuffer::FinalizeBuffer() {
  memcpy(_captureBuffer, _trigBuffer[(_tBufNum ^ 0x1)], _preTrigSize * sizeof(int)); //Copy the older pretrigger buffer first
  memcpy( &(_captureBuffer[_preTrigSize]), _trigBuffer[_tBufNum], _ptIndx * sizeof(int)); // Copy the partially filled trigger buffer
}

bool CaptureBuffer::CaptureDone() {
  return triggerState = 'd';
}

void CaptureBuffer::ResetTrigger() {
  _capIndx=0;
  triggerState = 'w';
}
