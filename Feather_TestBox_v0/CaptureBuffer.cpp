
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
  if (_trigHigh) {
    return (val > _trigValue);
  }
  return (val < _trigValue);
}

bool CaptureBuffer::CheckTriggerLastSample() {
  if (_trigHigh) {
    return (_lastValue > _trigValue);
  }
  return (_lastValue < _trigValue);
}

long CaptureBuffer::getLastStateChange() {
  return _tStateChange;
}

long CaptureBuffer::getLastTriggerMs() {
  return _trigTime_ms;
}

int CaptureBuffer::getTriggerIndex() {
  return _trigSampleIndex;
}

long CaptureBuffer::getCaptureTime() {
  return (_captureTime-_trigTime);
}

int CaptureBuffer::getLastTriggerIndex() {
  return _lastCapTriggerIndx;
}

long CaptureBuffer::getTriggerDuration() {
  return (_lastTrigTime-_trigTime);
}

void CaptureBuffer::AddSample(int value) {
  _lastValue = value;
  static bool oldTriggerState = false;
  bool triggerValue = CheckTrigger(value);
  static int _trigResetCount=0;
  static unsigned long trigResetTime=0;
  //static armCount=0;

  if (triggerValue != oldTriggerState) {
    _tStateChange = millis();
    oldTriggerState = triggerValue;
  }

  switch (triggerState) {
    case 'i': //Trigger initiated but not confirmed
    // After the debounce time any valid trigger will confirm the trigger.
    // If a second full debounce interval passes the trigger is discarded. 
      _captureBuffer[_capIndx] = value;
      _capIndx++;
      if ((micros() - _trigTime) > _trigDebounce)  { // If trigger confirmed goto trigger mode
        if (triggerValue) {
          triggerState = 't';
          _lastTrigTime=_trigValidTime=micros();
          //for (int k=_trigSampleIndex-2; k<_capIndx+2; k++) {
          //  Serial.print(_captureBuffer[k]);Serial.print(" ");
          //}
          //Serial.println("Trigger confirmed");
        } else {
          if (micros()>(_trigTime+_trigDebounce+_trigDebounce)) {//If we haven't confirmed a trigger after a full debounce interval reject  
            ResetTrigger(); //Trigger invalid reset and move on
            //Serial.println("Trigger invalid");
          }
        }
      }
      break;
    case 'w': // Waiting for trigger
      (_trigBuffer[_tBufNum])[_ptIndx] = value;
      _ptIndx++;
      if (_ptIndx == _preTrigSize) {
        _ptIndx = 0;
        _tBufNum = _tBufNum ^ 0x1; //XOR to toggle between buffer 0 and buffer 1
        //Serial.println("Next buffer");
      }
      if (triggerValue) {
        _capIndx = _preTrigSize + _ptIndx;
        _trigSampleIndex=_capIndx;
        triggerState = 'i';
        //Serial.println("Trigger detected");
        _lastTrigTime=_trigTime = micros();
        _trigTime_ms = millis();
        _trigResetCount=0;
      }
      break;
    case 't':    
      if (triggerValue) {
        _lastTrigTime=micros();
        _lastCapTriggerIndx=_capIndx;
      }
      _captureBuffer[_capIndx] = value;
      _capIndx++;
      if (_capIndx >= _capBufSize) { //Capture buffer is full, trigger complete
        triggerState = 'd';
        _captureTime = micros();
        FinalizeBuffer();  //Finalize the capture buffer by adding the pre-triggervalues;
        //Serial.println("Capture done");
      }
      break;
    case 'd':
      break;
    case 'r':
      if (triggerValue == false) {
        _armCount++;
        if (_armCount > rearmThresholdCount) { //Require 10 consecutive false triggers to re-arm
          triggerState = 'w'; //require trigger to false prior to arming
          //Serial.println("Armed");
        }
      } else {
        _armCount = 0;
      }
      break;
  }
}

void CaptureBuffer::FinalizeBuffer() {
  memcpy(_captureBuffer, _trigBuffer[(_tBufNum ^ 0x1)], _preTrigSize * sizeof(int)); //Copy the older pretrigger buffer first
  memcpy( &(_captureBuffer[_preTrigSize]), _trigBuffer[_tBufNum], _ptIndx * sizeof(int)); // Copy the partially filled trigger buffer
}

bool CaptureBuffer::CaptureDone() {
  return (triggerState == 'd');
}

void CaptureBuffer::ResetTrigger() {
  _capIndx = 0;
  _tBufNum = 0;
  _ptIndx = 0;
  _armCount = 0;
  _trigTime=0;
  triggerState = 'r';
  for (int k=0; k<_capBufSize; k++) {
    _captureBuffer[k]=9999;
  }
  Serial.println("Trigger reset");
}
