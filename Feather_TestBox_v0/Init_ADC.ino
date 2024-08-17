void InitializeADC(bool fastAcq) {
  // Configures the basic ADC, does not set ADMUX channels.

  nrf_saadc_enable(NRF_SAADC);
  nrf_saadc_int_disable(NRF_SAADC,NRF_SAADC_INT_ALL);
  //NVIC_SetPriority(ADC_IRQn, 6);

  //ADC Configuration code
  if (fastAcq) {
    nrf_saadc_channel_init(NRF_SAADC,ADC_UNIT, &FAST_ADC_CONFIG);
  } else {
    nrf_saadc_channel_init(NRF_SAADC,ADC_UNIT, &ADC_CONFIG);
  }
  // Set the resolution to 12-bit (0..4095)
  nrf_saadc_resolution_set(NRF_SAADC,NRF_SAADC_RESOLUTION_12BIT);
  NRF_SAADC->SAMPLERATE = (SAADC_SAMPLERATE_MODE_Task << SAADC_SAMPLERATE_MODE_Pos);
  //Calibrate the ADC
  nrf_saadc_task_trigger(NRF_SAADC,NRF_SAADC_TASK_CALIBRATEOFFSET);
  while (!nrf_saadc_event_check(NRF_SAADC,NRF_SAADC_EVENT_CALIBRATEDONE)) {
    delay(10);
  }
  nrf_saadc_event_clear(NRF_SAADC,NRF_SAADC_EVENT_CALIBRATEDONE);

  nrf_saadc_oversample_set(NRF_SAADC,NRF_SAADC_OVERSAMPLE_4X);

  nrf_saadc_burst_set(NRF_SAADC,ADC_UNIT, NRF_SAADC_BURST_ENABLED);
}

void StartADC() {
  nrf_saadc_int_disable(NRF_SAADC,NRF_SAADC_INT_ALL);
  nrf_saadc_event_clear(NRF_SAADC,NRF_SAADC_EVENT_RESULTDONE);
  nrf_saadc_int_enable(NRF_SAADC,NRF_SAADC_INT_RESULTDONE);
  NVIC_ClearPendingIRQ(ADC_IRQn);
  NVIC_EnableIRQ(ADC_IRQn);
  nrf_saadc_task_trigger(NRF_SAADC,NRF_SAADC_TASK_START);
  
  nrf_saadc_buffer_init(NRF_SAADC,ADC_Buffer1, 1);
  nrf_saadc_task_trigger(NRF_SAADC,NRF_SAADC_TASK_SAMPLE);
  //Serial.println("ADC Started");
}

void InitializeCableData() {
  ActiveCh = &(ChanArray[0]);
  //Serial.println(ActiveCh->ch_label);
  digitalWrite(MUX_LATCH, LOW); //equivalent to digitalWrite(4, LOW); Toggle the SPI
  shiftOut(MUX_DATA, MUX_CLK, MSBFIRST, ActiveCh->muxSetting);
  //shiftOut(MUX_DATA, MUX_CLK, MSBFIRST, MUX_CABLE_BB);
  digitalWrite(MUX_LATCH, HIGH); //equivalent to digitalWrite(4,HIGH);
  NRF_SAADC->CH[ADC_UNIT].PSELP = ActiveCh->AIn;

  StartADC();

  for (int m = 0; m < 20; m++) {
    for (int k = 0; k < NUM_ADC_SCAN_CHANNELS; k++) {
      while (!(ChanArray[k].valueReady)) {
      }
      ChanArray[k].valueReady = false;
    }
    updateCableState();
  }

  StopADC();

  cableState.cableDC = true;
  cableState.tLastConnect = -1 * (cableDisconnectTimeOut);
  Serial.println("Cable data initialized");
}

void InitializeWeaponData() {
  ActiveCh = &(FoilADC);
  //Serial.println(ActiveCh->ch_label);
  digitalWrite(MUX_LATCH, LOW); //equivalent to digitalWrite(4, LOW); Toggle the SPI
  shiftOut(MUX_DATA, MUX_CLK, MSBFIRST, ActiveCh->muxSetting);
  //shiftOut(MUX_DATA, MUX_CLK, MSBFIRST, MUX_CABLE_BB);
  digitalWrite(MUX_LATCH, HIGH); //equivalent to digitalWrite(4,HIGH);
  NRF_SAADC->CH[ADC_UNIT].PSELP = ActiveCh->AIn;

  StartADC();

  for (int m = 0; m < 20; m++) {
    while ( (!FoilADC.valueReady) && (!EpeeADC.valueReady) ) {
    }
    updateWeaponResistance();
    updateWeaponState();
    FoilADC.valueReady = false;
    EpeeADC.valueReady=false;
  }

  StopADC();

  weaponState.cableDC = true;
  weaponState.tLastConnect = -1 * (weaponDisconnectTimeOut);
  Serial.println("Weapon data initialized");
}

void StopADC() {
  NVIC_DisableIRQ(ADC_IRQn);
  NVIC_ClearPendingIRQ(ADC_IRQn);
  nrf_saadc_int_disable(NRF_SAADC,NRF_SAADC_INT_ALL);
  nrf_saadc_task_trigger(NRF_SAADC,NRF_SAADC_TASK_STOP);
}

void InitializeChannels() {

  snprintf(ChanArray[0].ch_label, 4, "AAA");
  ChanArray[0].muxSetting = MUX_CABLE_AA;

  snprintf(ChanArray[1].ch_label, 4, " AB");
  ChanArray[1].muxSetting = MUX_CABLE_AB;
  ChanArray[1].setRangeHigh();

  snprintf(ChanArray[2].ch_label, 4, " AC");
  ChanArray[2].muxSetting = MUX_CABLE_AC;
  ChanArray[2].setRangeHigh();

  snprintf(ChanArray[3].ch_label, 4, " BA");
  ChanArray[3].muxSetting = MUX_CABLE_BA;
  ChanArray[3].setRangeHigh();

  snprintf(ChanArray[4].ch_label, 4, "BBB");
  ChanArray[4].muxSetting = MUX_CABLE_BB;

  snprintf(ChanArray[5].ch_label, 4, " BC");
  ChanArray[5].muxSetting = MUX_CABLE_BC;
  ChanArray[5].setRangeHigh();

  snprintf(ChanArray[6].ch_label, 4, " CA");
  ChanArray[6].muxSetting = MUX_CABLE_CA;
  ChanArray[6].setRangeHigh();

  snprintf(ChanArray[7].ch_label, 4, " CB");
  ChanArray[7].muxSetting = MUX_CABLE_CB;
  ChanArray[7].setRangeHigh();

  snprintf(ChanArray[8].ch_label, 4, "CCC");
  ChanArray[8].muxSetting = MUX_CABLE_CC;

  EpeeADC.muxSetting = MUX_WEAPON_AB;
  snprintf(EpeeADC.ch_label, 4, "EAB");
  EpeeADC.nextChannel = &FoilADC;
  EpeeADC.hsBuffer.SetBuffers(ADC_CaptureBuffer, ADC_CAPTURE_LEN, &(ADC_PreTrigEpee[0][0]), &(ADC_PreTrigEpee[1][0]), PRE_TRIGGER_SIZE);
  EpeeADC.hsBuffer.setTrigger(maxADCthreshold, false); //If signal drops belows this value, AB is connected.
  EpeeADC.bufferEnabled = true;

  FoilADC.muxSetting = MUX_WEAPON_CB;
  snprintf(FoilADC.ch_label, 4, "FBC");
  FoilADC.nextChannel = &WeaponAC;
  FoilADC.hsBuffer.SetBuffers(ADC_CaptureBuffer, ADC_CAPTURE_LEN, &(ADC_PreTrigFoil[0][0]), &(ADC_PreTrigFoil[1][0]), PRE_TRIGGER_SIZE);
  FoilADC.hsBuffer.setTrigger(maxADCthreshold, false); //If signal goes above this the circuit is effectively open
  FoilADC.bufferEnabled = true;

  WeaponAC.muxSetting = MUX_WEAPON_AC;
  snprintf(WeaponAC.ch_label, 4, "WAC");
  WeaponAC.nextChannel = &(CableCheck[0]);
  WeaponAC.setRangeHigh();

  CableCheck[0].muxSetting = MUX_CABLE_AA;
  snprintf(CableCheck[0].ch_label, 4, "WAA");
  CableCheck[0].nextChannel = &(CableCheck[1]);

  CableCheck[1].muxSetting = MUX_CABLE_BB;
  snprintf(CableCheck[1].ch_label, 4, "WBB");
  CableCheck[1].nextChannel = &(CableCheck[2]);

  CableCheck[2].muxSetting = MUX_CABLE_CC;
  snprintf(CableCheck[2].ch_label, 4, "WCC");
  CableCheck[2].nextChannel = &EpeeADC;  

  loadCalibrationData();
  CableCheck[0].setTrim(ChanArray[0].getTrim()); //AA
  CableCheck[1].setTrim(ChanArray[4].getTrim()); //BB  
  CableCheck[2].setTrim(ChanArray[8].getTrim());  //CC
  /*
  Serial.print("Channel ");Serial.print(EpeeADC.ch_label);Serial.print("=");Serial.print((long)&(EpeeADC),HEX);Serial.print("\tNext=");Serial.println((long)EpeeADC.nextChannel,HEX);
  Serial.print("Channel ");Serial.print(FoilADC.ch_label);Serial.print("=");Serial.print((long)&(FoilADC),HEX);Serial.print("\tNext=");Serial.println((long)FoilADC.nextChannel,HEX);
  Serial.print("Channel ");Serial.print(WeaponAC.ch_label);Serial.print("=");Serial.print((long)&(WeaponAC),HEX);Serial.print("\tNext=");Serial.println((long)WeaponAC.nextChannel,HEX);
  for (int k=0;k<=2;k++) {
    Serial.print("Channel ");Serial.print(k);Serial.print("=");Serial.print((long)&(CableCheck[k]),HEX);
    //Serial.print("\tGPIO =");Serial.print(CableCheck[k].AIn);
    Serial.print("\tNext=");Serial.println((long)CableCheck[k].nextChannel,HEX);
  }*/
  
  initializeProbeChannels();

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
  arm_biquad_cascade_df1_init_f32(&(cableState.LineALowPass), 1, LowPass3HzCoef, cableState.LineALPFState);
  arm_biquad_cascade_df1_init_f32(&(cableState.LineBLowPass), 1, LowPass3HzCoef, cableState.LineBLPFState);
  arm_biquad_cascade_df1_init_f32(&(cableState.LineCLowPass), 1, LowPass3HzCoef, cableState.LineCLPFState);
  arm_biquad_cascade_df1_init_f32(&(cableState.LameLowPass), 1, LameLPF3HzCoef, cableState.LameLPFState);
}

void initializeProbeChannels() {
  //ADC_Channel ProbeArray[6] {0, 2, 5, 1, 0, 2}; // Epee (AB), Foil (CB), WepGnd (AC), BPrA, APrA,  CPrA
  snprintf(ProbeArray[0].ch_label, 4, "EAB");
  ProbeArray[0].muxSetting = MUX_WEAPON_AB;
  arm_biquad_cascade_df1_init_f32(&(probeData.EpeeLPF), 1, LowPass8HzCoef, probeData.EpeeLPFState);
  ProbeArray[0].setTrim(EpeeADC.getTrim());

  snprintf(ProbeArray[1].ch_label, 4, "FBC");
  ProbeArray[1].muxSetting = MUX_WEAPON_CB;
  ProbeArray[1].setTrim(FoilADC.getTrim());
  arm_biquad_cascade_df1_init_f32(&(probeData.FoilLPF), 1, LowPass8HzCoef, probeData.FoilLPFState);

  snprintf(ProbeArray[2].ch_label, 4, "WAC");
  ProbeArray[2].muxSetting = MUX_WEAPON_AC;
  ProbeArray[2].setTrim(WeaponAC.getTrim());
  arm_biquad_cascade_df1_init_f32(&(probeData.WpnACLPF), 1, LowPass8HzCoef, probeData.WpnACLPFState);

  snprintf(ProbeArray[3].ch_label, 4, "BPr");
  ProbeArray[3].muxSetting = MUX_CABLE_BA;
  ProbeArray[3].setTrim(ChanArray[4].getTrim());
  arm_biquad_cascade_df1_init_f32(&(probeData.ProbeBLPF), 1, LowPass5HzCoef, probeData.ProbeBLPFState);

  snprintf(ProbeArray[4].ch_label, 4, "APr");
  ProbeArray[4].muxSetting = MUX_CABLE_AA;
  ProbeArray[4].setTrim(ChanArray[0].getTrim());
  arm_biquad_cascade_df1_init_f32(&(probeData.ProbeALPF), 1, LowPass5HzCoef, probeData.ProbeALPFState);

  snprintf(ProbeArray[5].ch_label, 4, "CPr");
  ProbeArray[5].muxSetting = MUX_CABLE_CA;
  ProbeArray[5].setTrim(ChanArray[8].getTrim());
  arm_biquad_cascade_df1_init_f32(&(probeData.ProbeCLPF), 1, LowPass5HzCoef, probeData.ProbeCLPFState);

  for (int k = 0; k < (6); k++) {
    ProbeArray[k].nextChannel = &(ProbeArray[k+1]);
    ProbeArray[k].ADC_Scan = true;
  }
  ProbeArray[5].nextChannel = &(ProbeArray[0]);
}


void calibrateSystem() {
  bool cal_valid = false;
  long ave_data = 0;
  int adc_val;
  long tPress = 0;
  byte errorCount = 0;
  char buf[32];
  char label[8];
  int numCycles = 64;
  int sampleCount = 0;
  ADC_Channel *CalChan;
  nrf_saadc_value_t *adcRead;
  long tic, toc;

  setCableTestMode();
  StopADC();

  //nrf_saadc_buffer_init(ADC_Buffer1, ADC_BUFFER_SIZE); 
  
  for (int k = 0; k < NUM_CAL_CHANNELS; k++) {
    CalChan = getCalibrationChannel(k);

    tft.fillRect(0, 0, 128, 128, BLACK); //Clears the screen
    tft.setTextSize(2); //Medium size text

    nrf_saadc_task_trigger(NRF_SAADC,NRF_SAADC_TASK_STOP);
    //Set the proper channel MUX
    digitalWrite(MUX_LATCH, LOW); //equivalent to digitalWrite(4, LOW); Toggle the SPI
    shiftOut(MUX_DATA, MUX_CLK, MSBFIRST, CalChan->muxSetting);
    digitalWrite(MUX_LATCH, HIGH); //equivalent to digitalWrite(4,HIGH);

    cal_valid = false;
    errorCount = 0;
    label[0] = CalChan->ch_label[1];
    label[1] = '-';
    label[2] = CalChan->ch_label[2];
    label[3] = '\0';

    while (!cal_valid) {
      //sprintf(buf,"Connect %s\n Press button",label);
      //Serial.write(buf);
      NRF_WDT->RR[0] = WDT_RR_RR_Reload; //Reload watchdog register 0
      tft.fillScreen(BLACK);
      tft.setCursor(2, 2);
      tft.setTextColor(CYAN, BLACK);
      tft.println("  Connect ");
      tft.setTextColor(BLUE, BLACK);
      tft.print(k < (NUM_CAL_CHANNELS - 2) ? "Cable " : "Weapon ");
      tft.println(label);
      tft.setTextColor(CYAN, BLACK);
      tft.println("Press Btn");


      while (digitalRead(BUTTON_PIN) == LOW) {
        NRF_WDT->RR[0] = WDT_RR_RR_Reload; //Reload watchdog register 0
      }; //Wait until button pressed

      NRF_SAADC->CH[ADC_UNIT].PSELP = CalChan->AIn;
      nrf_saadc_task_trigger(NRF_SAADC,NRF_SAADC_TASK_START);
      ave_data = 0;
      sampleCount = 0;
      tic = micros();
      for (int j = 0; j < numCycles; j++) {
        nrf_saadc_buffer_init(NRF_SAADC,ADC_Buffer1, ADC_BUFFER_SIZE);
        nrf_saadc_event_clear(NRF_SAADC,NRF_SAADC_EVENT_RESULTDONE);
        nrf_saadc_event_clear(NRF_SAADC,NRF_SAADC_EVENT_END);
        nrf_saadc_task_trigger(NRF_SAADC,NRF_SAADC_TASK_START);
        while (!nrf_saadc_event_check(NRF_SAADC,NRF_SAADC_EVENT_END)) {
          nrf_saadc_task_trigger(NRF_SAADC,NRF_SAADC_TASK_SAMPLE);
          while (nrf_saadc_busy_check(NRF_SAADC)) {}
        }
        adcRead = nrf_saadc_buffer_pointer_get(NRF_SAADC);
        for (int m = 0; m < nrf_saadc_amount_get(NRF_SAADC); m++) {
          adc_val = adcRead[m];
          ave_data += adc_val;
          //Serial.println(m);
          sampleCount++;
        }
      }
      toc = micros();
      Serial.print("Samples= "); Serial.print(sampleCount); Serial.print(" | "); Serial.print(toc - tic); Serial.println("us");
      adc_val = (ave_data / sampleCount);
      NRF_WDT->RR[0] = WDT_RR_RR_Reload; //Reload watchdog register 0
      //snprintf(buf, 16, "Cal value: %u", adc_val);
      tft.print("Value=");
      //tft.println(adc_val);
      //Serial.println(buf);
      if (adc_val < calibrationErrorValue) {
        errorCount = 0;
        tft.setTextColor(GREEN, BLACK);
        tft.println(adc_val);
        tft.println("Success!");
        CalChan->setTrim(adc_val);
        cal_valid = true;
        delay(1000);
      } else {
        errorCount++;
        tft.setTextColor(RED, BLACK);
        tft.println(adc_val);
        tft.println("Error!");
        //Serial.println("Cal Error");
        delay(750);
        cal_valid = false;
        if (errorCount >= calibrationRetries) {
          //Serial.println("Cal Failed.  Exiting....");
          tft.println("Failed");
          tft.println("Exiting...");
          delay(1000);
          loadCalibrationData();
          return;
        }
      }
    }
  }
  tft.fillRect(0, 0, 128, 128, BLACK);
  tft.setCursor(2, 2);
  tft.println("Tap->save");
  tft.println("----------");
  tft.println("Hold->exit");
  tft.println("  w/o save\n");

  while (digitalRead(BUTTON_PIN) == LOW) {
    NRF_WDT->RR[0] = WDT_RR_RR_Reload; //Reload watchdog register 0
  }; //Wait until button pressed
  tPress = millis();
  while (digitalRead(BUTTON_PIN) == HIGH) {
    NRF_WDT->RR[0] = WDT_RR_RR_Reload; //Reload watchdog register 0
    if ((millis() - tPress) > tPowerOffPress) {
      tft.setTextColor(RED, BLACK);
      Serial.println("Discarding");
      tft.println("Discarding");
      delay(1000); //let user see message
    }
    delay(100);
  }

  if ((millis() - tPress) < tPowerOffPress) {
    NRF_WDT->RR[0] = WDT_RR_RR_Reload; //Reload watchdog register 0
    writeCalibrationData();
    tft.setTextColor(GREEN, BLACK);
    Serial.println("Saved");
    tft.println("Cal saved.");
    delay(1000);
  } else {
    NRF_WDT->RR[0] = WDT_RR_RR_Reload; //Reload watchdog register 0
    Serial.println("skipping");
    loadCalibrationData();
    delay(500);
  }
  // Automatically go to cable test mode
  setBoxMode(CABLE);
}
