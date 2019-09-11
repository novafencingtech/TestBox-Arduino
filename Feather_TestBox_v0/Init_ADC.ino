void InitializeADC() {
  // Configures the basic ADC, does not set ADMUX channels.
  
  nrf_saadc_enable();
  nrf_saadc_int_disable(NRF_SAADC_INT_ALL);
  //NVIC_SetPriority(ADC_IRQn, 6);
  
  //ADC Configuration code
  nrf_saadc_channel_init(ADC_UNIT, &ADC_CONFIG);
  // Set the resolution to 12-bit (0..4095)
  nrf_saadc_resolution_set(NRF_SAADC_RESOLUTION_12BIT);
  NRF_SAADC->SAMPLERATE = (SAADC_SAMPLERATE_MODE_Task << SAADC_SAMPLERATE_MODE_Pos);
  //Calibrate the ADC
  nrf_saadc_task_trigger(NRF_SAADC_TASK_CALIBRATEOFFSET);
  while (!nrf_saadc_event_check(NRF_SAADC_EVENT_CALIBRATEDONE)) {
    delay(10);
  } 
  nrf_saadc_event_clear(NRF_SAADC_EVENT_CALIBRATEDONE); 
  
  nrf_saadc_oversample_set(NRF_SAADC_OVERSAMPLE_4X);
  
  nrf_saadc_burst_set(ADC_UNIT,NRF_SAADC_BURST_ENABLED);
}

void StartADC() {
  nrf_saadc_int_disable(NRF_SAADC_INT_ALL);
  nrf_saadc_event_clear(NRF_SAADC_EVENT_RESULTDONE);
  nrf_saadc_int_enable(NRF_SAADC_INT_RESULTDONE);
  nrf_saadc_buffer_init(ADC_Buffer1, 1);
  NVIC_EnableIRQ(ADC_IRQn);
  nrf_saadc_task_trigger(NRF_SAADC_TASK_START);
  nrf_saadc_task_trigger(NRF_SAADC_TASK_SAMPLE);
  //Serial.println("ADC Started");  
}

void StopADC() {
  NVIC_DisableIRQ(ADC_IRQn);
  nrf_saadc_int_disable(NRF_SAADC_INT_ALL);
  nrf_saadc_task_trigger(NRF_SAADC_TASK_STOP);
}

void InitializeChannels() {

  snprintf(ChanArray[0].ch_label,4,"AAA");
  ChanArray[0].muxSetting = MUX_CABLE_AA;
  
  snprintf(ChanArray[1].ch_label,4," AB");
  ChanArray[1].muxSetting = MUX_CABLE_AB;
  ChanArray[1].setRangeHigh();
  
  snprintf(ChanArray[2].ch_label,4," AC");
  ChanArray[2].muxSetting = MUX_CABLE_AC;
  ChanArray[2].setRangeHigh();
  
  snprintf(ChanArray[3].ch_label,4," BA");
  ChanArray[3].muxSetting = MUX_CABLE_BA;
  ChanArray[3].setRangeHigh();
  
  snprintf(ChanArray[4].ch_label,4,"BBB");
  ChanArray[4].muxSetting = MUX_CABLE_BB;
  
  snprintf(ChanArray[5].ch_label,4," BC");
  ChanArray[5].muxSetting = MUX_CABLE_BC;
  ChanArray[5].setRangeHigh();
  
  snprintf(ChanArray[6].ch_label,4," CA");
  ChanArray[6].muxSetting = MUX_CABLE_CA;
  ChanArray[6].setRangeHigh();
  
  snprintf(ChanArray[7].ch_label,4," CB");
  ChanArray[7].muxSetting = MUX_CABLE_CB;
  ChanArray[7].setRangeHigh();
  
  snprintf(ChanArray[8].ch_label,4,"CCC");
  ChanArray[8].muxSetting = MUX_CABLE_CC;

  EpeeADC.muxSetting=MUX_WEAPON_AB;
  snprintf(EpeeADC.ch_label,4,"EAB");
  EpeeADC.nextChannel=&FoilADC;
  EpeeADC.hsBuffer.SetBuffers(ADC_CaptureBuffer,ADC_CAPTURE_LEN,&(ADC_PreTrigEpee[0][0]),&(ADC_PreTrigEpee[1][0]),PRE_TRIGGER_SIZE);
  EpeeADC.hsBuffer.setTrigger(maxADCthreshold,false); //If signal drops belows this value, AB is connected.  
  EpeeADC.bufferEnabled=true;
  
  FoilADC.muxSetting=MUX_WEAPON_CB;
  snprintf(FoilADC.ch_label,4,"FBC");
  FoilADC.nextChannel=&WeaponAC;
  FoilADC.hsBuffer.SetBuffers(ADC_CaptureBuffer,ADC_CAPTURE_LEN,&(ADC_PreTrigFoil[0][0]),&(ADC_PreTrigFoil[1][0]),PRE_TRIGGER_SIZE);
  FoilADC.hsBuffer.setTrigger(maxADCthreshold,true); //If signal goes above this the circuit is effectively open
  FoilADC.bufferEnabled=true;
  
  WeaponAC.muxSetting=MUX_WEAPON_AC;
  snprintf(WeaponAC.ch_label,4,"WAC");
  WeaponAC.nextChannel=&EpeeADC;
  WeaponAC.setRangeHigh();
  
  
  for (int k = 0; k < (NUM_ADC_SCAN_CHANNELS); k++) {
    ChanArray[k].nextChannel = &(ChanArray[ChannelScanOrder[k]]);
    ChanArray[k].ADC_Scan = true;
  }
  ActiveCh = ChanArray; //Pointer to the first channel
  /*  
  for (int k=0;k<NUM_ADC_SCAN_CHANNELS;k++) {
    Serial.print("Channel ");Serial.print(k);Serial.print("=");Serial.print((long)&(ChanArray[k]),HEX);
    Serial.print("\tGPIO =");Serial.print(ChanArray[k].AIn);
    Serial.print("\tNext=");Serial.println((long)ChanArray[k].nextChannel,HEX);
  }*/

  //Initialize the cable Low Pass Filters
    arm_biquad_cascade_df1_init_f32(&(cableState.LineALowPass),1,LowPass5HzCoef,cableState.LineALPFState);
    arm_biquad_cascade_df1_init_f32(&(cableState.LineBLowPass),1,LowPass5HzCoef,cableState.LineBLPFState);
    arm_biquad_cascade_df1_init_f32(&(cableState.LineCLowPass),1,LowPass5HzCoef,cableState.LineCLPFState);
}


void calibrateSystem() {
  bool cal_valid = false;
  unsigned int ave_data = 0;
  int adc_val;
  long tPress = 0;
  byte errorCount=0;
  char buf[32];
  char label[8];

  setCableTestMode();

  for (int k = 0; k < NUM_ADC_SCAN_CHANNELS; k++) {
    switch (ChanArray[k].ch_label[0]) {
      case 'A':
      case 'B':
      case 'C':
      case 'F':
      case 'E':        

        //Set the proper channel MUX
        digitalWrite(MUX_LATCH,LOW); //equivalent to digitalWrite(4, LOW); Toggle the SPI
        shiftOut(MUX_DATA, MUX_CLK, LSBFIRST, ChanArray[k].muxSetting);
        digitalWrite(MUX_LATCH,HIGH);  //equivalent to digitalWrite(4,HIGH);
        //ChanArray[k].setADCChannelActive();
        
        cal_valid = false;
        errorCount=0;
        label[0]=ChanArray[k].ch_label[0];
        label[1]='\0';
        switch (ChanArray[k].ch_label[0]) {
          case 'F': sprintf(label,"%s","Foil"); break;
          case 'E': sprintf(label,"%s","Epee"); break;
        }
        
        while (!cal_valid) {        
          sprintf(buf,"Short %s\n",label);  
          Serial.write(buf);
          
          while (digitalRead(BUTTON_PIN) == LOW) {}; //Wait until button pressed

          ave_data = 0;
          for (int j = 0; j < ADC_Channel::FIR_BLOCK_SIZE; j++) {
            ave_data+=analogRead(ChanArray[k].AIn);
          }
          adc_val = (ave_data/ADC_Channel::FIR_BLOCK_SIZE);

          snprintf(buf, 16, "Cal %c: %u", ChanArray[k].ch_label[0], adc_val);
          Serial.println(buf);
          if (adc_val < calibrationErrorValue) {
            errorCount=0;
            Serial.println("Cal Success");
            ChanArray[k].setTrim(adc_val);
            cal_valid = true;
            delay(1500);
          } else {
            errorCount++;
            Serial.println("Cal Error");
            delay(2000);
            cal_valid = false;
            if (errorCount>=calibrationRetries) {
              Serial.println("Cal Failed.  Exiting....");
              delay(2000);
              loadCalibrationData();
              return;
            }
          }
        }
        break;
      default:
        ChanArray[k].setTrim(0);
        break;
    }
  }
  Serial.println("Tap to save or hold to discard");

  while (digitalRead(BUTTON_PIN) == LOW) {}; //Wait until button pressed
  tPress = millis();
  while (digitalRead(BUTTON_PIN) == HIGH) {
    if ((millis() - tPress) > tPowerOffPress) {
      Serial.println("Discarding");
    }
    delay(100);
  }

  if ((millis() - tPress) < tPowerOffPress) {
    writeCalibrationData();
    Serial.println("Cal saved.");
    delay(1000);
  } else {
    loadCalibrationData();
    delay(500);
  }
  // Automatically go to cable test mode
  setCableTestMode();
}
