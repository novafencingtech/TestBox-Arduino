
#ifndef CaptureBuffer_h
#define CaptureBuffer_h

#include "Arduino.h"


class CaptureBuffer
{
  public:

    static const int rearmThresholdCount=10;
    static const int trigResetSampleThreshold=3;
    int* GetBuffer(int *samplesInBuffer); //Returns pointer to the buffer start, and the number of samples in the buffer
    void AddSample(int value); 
    void SetBuffers(int *mainBuffer, int bufferSize, int *preTrigger1, int *preTrigger2, int preTriggerLen);
    bool CaptureDone(); //Checks if a capture has completed. 
    void ResetTrigger();  //Resets and allows for a new capture to occur. 
    bool CheckTrigger(int val);
    bool CheckTriggerLastSample();
    void setTrigger(int value, bool TriggerHigh);
    void setTrigger(int value, bool TriggerHigh, long debounce); //debounce in us
    long getLastTriggerMs();
    long getLastStateChange();
    int getTriggerIndex();
    int getLastTriggerIndex();
    long getTriggerDuration(); //Time between initial trigger & last trigger in the buffer
    long getCaptureTime(); //Interval between trigger start and capture finish

  private:
    void FinalizeBuffer();
    
    bool captureDone=false;
    char triggerState='w'; // 'w' -- wait, 't' -- triggered, 'd'-done, 'i' --initiated, 'r' -- reset
    byte _tBufNum=0;
    int _ptIndx=0; //pre-trigger buffer index
    int _capIndx=0; //Capture buffer index
    int *_captureBuffer;
    int *_trigBuffer[2]; 
    
    int _preTrigSize;
    int _capBufSize;
    int _lastValue;

    int _trigSampleIndex=0;
    int _lastCapTriggerIndx=0;
    unsigned long _lastTrigTime=0;
    
    unsigned long _tStateChange=0;
    unsigned long _trigTime=0;  //Time in us
    unsigned long _trigTime_ms=0;
    unsigned long _trigValidTime=0;
    unsigned long _captureTime=0;
    int _trigValue;
    
    bool _trigHigh;
    long _trigDebounce=1500; //Trigger must remain below threshold continuously for this duration

    int _armCount=0;    
};

#endif
