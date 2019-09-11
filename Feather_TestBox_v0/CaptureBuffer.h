
#ifndef CaptureBuffer_h
#define CaptureBuffer_h

#include "Arduino.h"


class CaptureBuffer
{
  public:
    int* GetBuffer(int *samplesInBuffer); 
    void AddSample(int value);
    void SetBuffers(int *mainBuffer, int bufferSize, int *preTrigger1, int *preTrigger2, int preTriggerLen);
    bool CaptureDone();
    void ResetTrigger();
    bool CheckTrigger(int val);
    void setTrigger(int value, bool TriggerHigh);
    void setTrigger(int value, bool TriggerHigh, long debounce); //debounce in us
    long getLastTriggerMs();
    

  private:
    void FinalizeBuffer();
    
    bool captureDone=false;
    char triggerState='w'; // 'w' -- wait, 't' -- triggered, 'd'-done, 'i' --initiated
    byte _tBufNum=0;
    int _ptIndx=0; //pre-trigger buffer index
    int _capIndx=0; //Capture buffer index
    int *_captureBuffer;
    int *_trigBuffer[2]; 
    
    int _preTrigSize;
    int _capBufSize;

    long _trigTime=0;  //Time in us
    long _trigTime_ms=0;
    int _trigValue;
    
    bool _trigHigh;
    long _trigDebounce=1500; //Trigger must remain below threshold continuously for this duration
    
};

#endif
