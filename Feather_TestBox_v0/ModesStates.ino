void BlinkLEDThenPowerOff() {
  digitalWrite(LED2_PIN, LOW);
  for (int k = 0; k < 10; k++) {
    digitalWrite(LED2_PIN, HIGH);
    delay(100);
    digitalWrite(LED2_PIN, LOW);
    delay(100);
    digitalWrite(LED2_PIN, HIGH);
    delay(100);
    digitalWrite(LED2_PIN, LOW);
    delay(700);
  }
  digitalWrite(POWER_CONTROL, LOW);
}


void setWeaponTestMode() {
  StopADC();

  digitalWrite(MUX_LATCH, LOW); //equivalent to digitalWrite(4, LOW); Toggle the SPI
  shiftOut(MUX_DATA, MUX_CLK, MSBFIRST, MUX_WEAPON_MODE);
  digitalWrite(MUX_LATCH, HIGH); //equivalent to digitalWrite(4, HIGH); Toggle the SPI

  nrf_gpio_cfg(LineADetect, NRF_GPIO_PIN_DIR_INPUT, NRF_GPIO_PIN_INPUT_CONNECT, NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_S0S1, NRF_GPIO_PIN_SENSE_LOW);
  nrf_gpio_cfg(LineCDetect, NRF_GPIO_PIN_DIR_INPUT, NRF_GPIO_PIN_INPUT_CONNECT, NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_S0S1, NRF_GPIO_PIN_SENSE_HIGH);

  setWeaponInterrupts();

  //tLastActive = millis();

  BoxState = WPN_TEST;
  
}

void setWeaponResistanceMode(bool enableCapture) {
  detachInterrupt(LineADetect);
  detachInterrupt(LineCDetect);
  nrf_gpio_cfg(LineADetect, NRF_GPIO_PIN_DIR_INPUT, NRF_GPIO_PIN_INPUT_CONNECT, NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_S0S1, NRF_GPIO_PIN_SENSE_LOW);
  nrf_gpio_cfg(LineCDetect, NRF_GPIO_PIN_DIR_INPUT, NRF_GPIO_PIN_INPUT_CONNECT, NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_S0S1, NRF_GPIO_PIN_SENSE_HIGH);

  digitalWrite(MUX_LATCH, LOW); //equivalent to digitalWrite(4, LOW); Toggle the SPI
  shiftOut(MUX_DATA, MUX_CLK, MSBFIRST, (byte) 0x0);
  digitalWrite(MUX_LATCH, HIGH);

  StopADC();

  cableState.cableDC = true;
  
  InitializeADC(enableCapture);
  loadCalibrationData();
  ActiveCh = &(FoilADC);

  if (enableCapture) {
    //FoilADC.hsBuffer.SetBuffers(int *mainBuffer, int bufferSize, int *preTrigger1, int *preTrigger2, int preTriggerLen);
    FoilADC.hsBuffer.SetBuffers(ADC_CaptureBuffer,ADC_CAPTURE_LEN,&(ADC_PreTrigFoil[0][0]),&(ADC_PreTrigFoil[1][0]),PRE_TRIGGER_SIZE);
    //FoilADC.hsBuffer.setTrigger(int value, bool TriggerHigh, long debounce); //debounce in us
    FoilADC.hsBuffer.setTrigger(maxADCthreshold, true, usEpeeDebounce); //debounce in us
    FoilADC.bufferEnabled=true;
    FoilADC.hsBuffer.ResetTrigger();    

    EpeeADC.hsBuffer.SetBuffers(ADC_CaptureBuffer,ADC_CAPTURE_LEN,&(ADC_PreTrigEpee[0][0]),&(ADC_PreTrigEpee[1][0]),PRE_TRIGGER_SIZE);
    //FoilADC.hsBuffer.setTrigger(int value, bool TriggerHigh, long debounce); //debounce in us
    EpeeADC.hsBuffer.setTrigger(shortADCthreshold, false, usEpeeDebounce); //debounce in us
    EpeeADC.bufferEnabled=true;
    EpeeADC.hsBuffer.ResetTrigger();  

    BoxState = HIT_CAPTURE;
  } else {
    EpeeADC.bufferEnabled=false;
    EpeeADC.hsBuffer.ResetTrigger();
    FoilADC.bufferEnabled=false;
    FoilADC.hsBuffer.ResetTrigger();
    BoxState = WPN_GRAPH;
  }
  
  StartADC();
}

void setCableTestMode() {
  detachInterrupt(LineADetect);
  detachInterrupt(LineCDetect);
  nrf_gpio_cfg(LineADetect, NRF_GPIO_PIN_DIR_INPUT, NRF_GPIO_PIN_INPUT_CONNECT, NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_S0S1, NRF_GPIO_PIN_SENSE_LOW);
  nrf_gpio_cfg(LineCDetect, NRF_GPIO_PIN_DIR_INPUT, NRF_GPIO_PIN_INPUT_CONNECT, NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_S0S1, NRF_GPIO_PIN_SENSE_HIGH);

  digitalWrite(MUX_LATCH, LOW); //equivalent to digitalWrite(4, LOW); Toggle the SPI
  shiftOut(MUX_DATA, MUX_CLK, MSBFIRST, (byte) 0x0);
  digitalWrite(MUX_LATCH, HIGH);

  StopADC();

  // Re-initialize the ADC
  InitializeADC(false);
  loadCalibrationData();

  ActiveCh = &(ChanArray[0]);

  weaponState.cableDC = true;
  
  BoxState = CABLE;
  //tLastActive = millis();
  //updateCableState();
  StartADC();
}

void setProbeMode() {
  detachInterrupt(LineADetect);
  detachInterrupt(LineCDetect);
  nrf_gpio_cfg(LineADetect, NRF_GPIO_PIN_DIR_INPUT, NRF_GPIO_PIN_INPUT_CONNECT, NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_S0S1, NRF_GPIO_PIN_SENSE_LOW);
  nrf_gpio_cfg(LineCDetect, NRF_GPIO_PIN_DIR_INPUT, NRF_GPIO_PIN_INPUT_CONNECT, NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_S0S1, NRF_GPIO_PIN_SENSE_HIGH);

  digitalWrite(MUX_LATCH, LOW); //equivalent to digitalWrite(4, LOW); Toggle the SPI
  shiftOut(MUX_DATA, MUX_CLK, MSBFIRST, (byte) 0x0);
  digitalWrite(MUX_LATCH, HIGH);

  StopADC();

  // Re-initialize the ADC
  InitializeADC(false);
  loadCalibrationData();

  ActiveCh = &(ProbeArray[0]);

  weaponState.cableDC = true;
  
  BoxState = PROBE;
  StartADC();
}

void updateCableState() {
  uint16_t statusCheck;
  //static long tLastConnect = 0;
  //long tNow = millis();
  static uint16_t priorStatus = (1 << BITAA) || (1 << BITBB) || (1 << BITBB);
  float tempValue = 0;

  if (ChanArray[0].getRawValue() < CABLE_DISCONNECT_THRESHOLD) {
    tempValue = ChanArray[0].getValue();
    arm_biquad_cascade_df1_f32(&(cableState.LineALowPass), &tempValue, &(cableState.ohm_AA), 1);
    cableState.ohm_AAMax = ChanArray[0].getDecayMaxValue();
  }
  else {
    cableState.ohm_AA = OPEN_CIRCUIT_VALUE;
    cableState.ohm_AAMax = OPEN_CIRCUIT_VALUE;
  }

  if (ChanArray[4].getRawValue() < CABLE_DISCONNECT_THRESHOLD) {
    tempValue = ChanArray[4].getValue();
    arm_biquad_cascade_df1_f32(&(cableState.LineBLowPass), &tempValue, &(cableState.ohm_BB), 1);
    cableState.ohm_BBMax = ChanArray[4].getDecayMaxValue();
  }
  else {
    cableState.ohm_BB = OPEN_CIRCUIT_VALUE;
    cableState.ohm_BBMax = OPEN_CIRCUIT_VALUE;
  }

  if (ChanArray[8].getRawValue() < CABLE_DISCONNECT_THRESHOLD) {
    tempValue = ChanArray[8].getValue();
    arm_biquad_cascade_df1_f32(&(cableState.LineCLowPass), &tempValue, &(cableState.ohm_CC), 1);
    arm_biquad_cascade_df1_f32(&(cableState.LameLowPass), &tempValue, &(cableState.ohm_Lame), 1);
    cableState.ohm_CCMax = ChanArray[8].getDecayMaxValue();
    cableState.ohm_LameMax = ChanArray[8].getDecayMaxValue();
  }
  else {
    tempValue = ChanArray[8].getValue();
    cableState.ohm_CC = OPEN_CIRCUIT_VALUE;
    cableState.ohm_CCMax = OPEN_CIRCUIT_VALUE;    
    arm_biquad_cascade_df1_f32(&(cableState.LameLowPass), &tempValue, &(cableState.ohm_Lame), 1);
    //cableState.ohm_Lame = OPEN_CIRCUIT_VALUE;
    cableState.ohm_LameMax = OPEN_CIRCUIT_VALUE;
  }

  cableState.line_AA = ChanArray[0].getRawValue();
  cableState.line_AB = ChanArray[1].getRawValue();
  cableState.line_AC = ChanArray[2].getRawValue();
  cableState.line_BA = ChanArray[3].getRawValue();
  cableState.line_BB = ChanArray[4].getRawValue();
  cableState.line_BC = ChanArray[5].getRawValue();
  cableState.line_CA = ChanArray[6].getRawValue();
  cableState.line_CB = ChanArray[7].getRawValue();
  cableState.line_CC = ChanArray[8].getRawValue();

  for (int k = 0; k < NUM_ADC_SCAN_CHANNELS; k++) {
    if ( (ChanArray[k].getRawValue()>shortADCthreshold) && (ChanArray[k].isHighRange()) ) {
      cableState.cableOhm[k]=OPEN_CIRCUIT_VALUE;
    } else {
      cableState.cableOhm[k] = ChanArray[k].getValue();
    }
    ChanArray[k].valueReady = false;
  }

  bitWrite(cableState.statusByte, BITAA, cableState.ohm_AAMax > HIGH_RESISTANCE_THRESHOLD);
  bitWrite(cableState.statusByte, BITBB, cableState.ohm_BBMax > HIGH_RESISTANCE_THRESHOLD);
  bitWrite(cableState.statusByte, BITCC, cableState.ohm_CCMax > HIGH_RESISTANCE_THRESHOLD);

  bitWrite(cableState.statusByte, BITAB, (cableState.line_AB < shortADCthreshold));
  bitWrite(cableState.statusByte, BITAC, cableState.line_AC < shortADCthreshold);
  bitWrite(cableState.statusByte, BITBA, cableState.line_BA < shortADCthreshold);
  bitWrite(cableState.statusByte, BITBC, cableState.line_BC < shortADCthreshold);
  bitWrite(cableState.statusByte, BITCA, cableState.line_CA < shortADCthreshold);
  bitWrite(cableState.statusByte, BITCB, cableState.line_CB < shortADCthreshold);

  //Serial.println(cableState.line_AB);
  if (priorStatus != cableState.statusByte) {
    tLastActive = millis();
    priorStatus = cableState.statusByte;
  }

  if (cableState.cableDC) {
    //Serial.println("Disconnected");
    for (int k = 0; k < NUM_ADC_SCAN_CHANNELS; k++) {
      ChanArray[k].resetMinMaxValues();
    }
  }

  cableState.cableDC = false;

  //Check if only testing a lame (line C-C only)
  if (cableState.ohm_AA >= OPEN_CIRCUIT_VALUE &&
      cableState.ohm_BB >= OPEN_CIRCUIT_VALUE &&
      cableState.ohm_CC >= OPEN_CIRCUIT_VALUE &&
      !bitRead(cableState.statusByte, BITAB) &&
      !bitRead(cableState.statusByte, BITAC) &&
      !bitRead(cableState.statusByte, BITBC) )
  {
    if ((millis() - cableState.tLastConnect) > cableDisconnectTimeOut) {
      //Serial.println("Disconnected");
      cableState.cableDC = true;
      //cableState.lameMode = false;
      //cableState.maskMode = false;
      //cableState.lameMode = false;
    }
  } else {
    //Serial.println("Connected");
    cableState.tLastConnect = millis();
    cableState.cableDC = false;
  }

  //Check if only testing a mask cord (line A-A only)
  uint16_t statusMask = ~( (1<<BITAA) | (1<<BITBB) | (1<<BITCC));
  if  ((cableState.statusByte & statusMask)==0) {
    if ((cableState.ohm_AA >= OPEN_CIRCUIT_VALUE) && (cableState.ohm_BB >= OPEN_CIRCUIT_VALUE)) {     
      if ((cableState.ohm_CC < OPEN_CIRCUIT_VALUE) && ( (millis()-tLastActive)>tLameWaitTime) ) {
          cableState.lameMode = true;
          cableState.maskMode = false;
        }
    } else {
      cableState.lameMode = false;
    } 
  }
  
  statusCheck = ((1 << BITBB) | (1 << BITCC));
  if ( (cableState.statusByte & ~(1 << BITAA)) == statusCheck) {
    if ((cableState.ohm_AA < OPEN_CIRCUIT_VALUE) && ((cableState.ohm_BB >= OPEN_CIRCUIT_VALUE) && (cableState.ohm_CC >= OPEN_CIRCUIT_VALUE)) ) {
      cableState.lameMode = false;
      cableState.maskMode = true;
    }
  } else {
    cableState.maskMode = false;
  }

}



void setWeaponInterrupts() {
  attachInterrupt(LineADetect, &ISR_EpeeHitDetect, CHANGE);
  attachInterrupt(LineCDetect, &ISR_FoilHitDetect, CHANGE);
}


void setBoxMode(TestBoxModes newMode) {
  //Serial.print("Setting mode = "); Serial.println(mode);
  switch (newMode) {
    case CABLE:
      setCableTestMode();
      break;
    case WPN_TEST:
      setWeaponTestMode();
      break;
    case WPN_GRAPH:
      setWeaponResistanceMode(false);
      break;
    case BOX_IDLE:
      setIdleMode();
      break;
    case PROBE:
      setProbeMode();
      break;
    case HIT_CAPTURE:
      setWeaponResistanceMode(true);
      break;
  }
}

void setIdleMode() {
  updateIdleMode();
  BoxState = BOX_IDLE;
}

void updateIdleMode() {
  //Serial.println("Still idle");
  detachInterrupt(LineADetect);
  detachInterrupt(LineCDetect);
  //nrf_gpio_cfg(LineADetect, NRF_GPIO_PIN_DIR_INPUT, NRF_GPIO_PIN_INPUT_DISCONNECT, NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_S0S1, NRF_GPIO_PIN_SENSE_LOW);
  //nrf_gpio_cfg(LineCDetect, NRF_GPIO_PIN_DIR_INPUT, NRF_GPIO_PIN_INPUT_DISCONNECT, NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_S0S1, NRF_GPIO_PIN_SENSE_HIGH);
  
  if (checkCableConnected()) {
    setBoxMode(CABLE);
    return;
  }
  if (checkWeaponConnected()) {
    setBoxMode(WPN_GRAPH);
    return;
  }
    
}

bool checkCableConnected() {  
  digitalWrite(MUX_LATCH, LOW); //equivalent to digitalWrite(4, LOW); Toggle the SPI
  shiftOut(MUX_DATA, MUX_CLK, MSBFIRST, (byte) 0x0);
  digitalWrite(MUX_LATCH, HIGH); //equivalent to digitalWrite(4, HIGH); Toggle the SPI

  StopADC();
  for (int k = 0; k < NUM_ADC_SCAN_CHANNELS; k++) {
    ChanArray[k].valueReady = false;
    ChanArray[k].sampleCount = 0;
  }

  ActiveCh = &(ChanArray[0]);
  //Serial.println(ActiveCh->ch_label);
  digitalWrite(MUX_LATCH, LOW); //equivalent to digitalWrite(4, LOW); Toggle the SPI
  shiftOut(MUX_DATA, MUX_CLK, MSBFIRST, ActiveCh->muxSetting);
  //shiftOut(MUX_DATA, MUX_CLK, MSBFIRST, MUX_CABLE_BB);
  digitalWrite(MUX_LATCH, HIGH); //equivalent to digitalWrite(4,HIGH);
  //nrf_saadc_channel_pos_input_set(ADC_UNIT,ActiveCh->AIn);
  NRF_SAADC->CH[ADC_UNIT].PSELP = ActiveCh->AIn;

  delay(1);

  StartADC();

  for (int k = 0; k < NUM_ADC_SCAN_CHANNELS; k++) {
    while (!(ChanArray[k].valueReady)) {
    }
    ChanArray[k].valueReady = false;
  }
  updateCableState();
  StopADC();

  return (!cableState.cableDC);
}

bool checkWeaponConnected() {
  FoilADC.valueReady = false;
  EpeeADC.valueReady = false;
  
  StopADC();
  ActiveCh = &(FoilADC);
  digitalWrite(MUX_LATCH, LOW); //equivalent to digitalWrite(4, LOW); Toggle the SPI
  shiftOut(MUX_DATA, MUX_CLK, MSBFIRST, ActiveCh->muxSetting);
  //shiftOut(MUX_DATA, MUX_CLK, MSBFIRST, MUX_CABLE_BB);
  digitalWrite(MUX_LATCH, HIGH); //equivalent to digitalWrite(4,HIGH);
  //nrf_saadc_channel_pos_input_set(ADC_UNIT,ActiveCh->AIn);
  NRF_SAADC->CH[ADC_UNIT].PSELP = ActiveCh->AIn;

  StartADC();
  while ( (!FoilADC.valueReady) && (!EpeeADC.valueReady) ) {
  }
  StopADC();
  updateWeaponResistance();
  updateWeaponState();

  //while (nrf_saadc_busy_check(NRF_SAADC)) {};

  digitalWrite(MUX_LATCH, LOW); //equivalent to digitalWrite(4, LOW); Toggle the SPI
  shiftOut(MUX_DATA, MUX_CLK, MSBFIRST, (byte) 0x0);
  digitalWrite(MUX_LATCH, HIGH); //equivalent to digitalWrite(4, HIGH); Toggle the SPI

  return (!weaponState.cableDC);
}

void updateWeaponResistance() {
  long t_now = millis();

  if (FoilADC.getRawValue() < CABLE_DISCONNECT_THRESHOLD) {
    weaponState.ohm_Foil = FoilADC.getValue();
    weaponState.ohm_FoilMax = FoilADC.getDecayMaxValue();
    weaponState.tLastConnect = t_now;
    weaponState.cableDC = false;
  } else {
    weaponState.ohm_Foil = OPEN_CIRCUIT_VALUE;
  }
  if (EpeeADC.getRawValue() < CABLE_DISCONNECT_THRESHOLD) {
    weaponState.ohm_Epee = EpeeADC.getValue();
    weaponState.ohm_EpeeMax = EpeeADC.getDecayMaxValue();
    weaponState.tLastConnect = t_now;
    weaponState.cableDC = false;
  } else {
    weaponState.ohm_Epee = OPEN_CIRCUIT_VALUE;
  }

  weaponState.ohm10xEpee = int(weaponState.ohm_Epee * 10 + 0.5);
  weaponState.ohm10xFoil = int(weaponState.ohm_Foil * 10 + 0.5);

  weaponState.lineAC = (WeaponAC.getRawValue() < shortADCthreshold);

  if ( (weaponState.ohm_Epee == OPEN_CIRCUIT_VALUE) && (weaponState.ohm_Foil == OPEN_CIRCUIT_VALUE) ) {
    if ((t_now - weaponState.tLastConnect) > weaponDisconnectTimeOut) {
      weaponState.cableDC = true;
      //Serial.println("No weapon connected");
    }
  }
}

//ADC_Channel ProbeArray[6] {0, 2, 5, 1, 0, 2}; // Epee (AB), Foil (CB), WepGnd (AC), BPrA, APrA,  CPrA
void updateProbe()
{  //The order of filter and data address must match the ADC and mux configuration in ProbeArray.
  arm_biquad_casd_df1_inst_f32* filterAddr[6]={&(probeData.EpeeLPF),&(probeData.FoilLPF),&(probeData.WpnACLPF),&(probeData.ProbeBLPF),&(probeData.ProbeALPF),&(probeData.ProbeCLPF)};
  float* dataAddr[6]={&(probeData.ohm_Epee),&(probeData.ohm_Foil),&(probeData.ohm_WpnAC),&(probeData.ohm_BPr),&(probeData.ohm_APr),&(probeData.ohm_CPr)};
  float tempValue;

  for (int k=0; k<6; k++) {
      if (!ProbeArray[k].valueReady) {
        return; //All channels aren't ready to update, bail out
      }
  }

  for (int k=0; k<6; k++) {
    if (ProbeArray[k].getRawValue() < CABLE_DISCONNECT_THRESHOLD) {
      tempValue = ProbeArray[k].getValue();
      arm_biquad_cascade_df1_f32(filterAddr[k], &tempValue, dataAddr[k], 1);    
    }
    else {
      *dataAddr[k] = OPEN_CIRCUIT_VALUE;
    }
  }
}

void updateWeaponState() {
  //Update this code to use the A, B, C line displays
  // A = red/Epee
  // B = yellow/ Intermittent
  // C = green / Foil

  long t_now = millis();
  //static bool oldEpeeState = false;
  //static bool oldFoilState = false;

  bool epeeState = EpeeADC.lastValue<maxADCthreshold;
  bool foilState = FoilADC.lastValue<maxADCthreshold;

  if (foilState != weaponState.foilOn) {
    
    //tLastActive=t_now;
    if ((t_now - weaponState.tFoilTrigger) > weaponFoilDebounce) {
      //Serial.println("Foil trigger");
      tLastActive = t_now;
      weaponState.foilOn = foilState;
      weaponState.tFoilInterOn = t_now;
      weaponState.foilInterOn = true;
      //Serial.println("Foil trigger");
    }
  }

  if (epeeState != weaponState.epeeOn) {
    //tLastActive=t_now;
    if ((t_now - weaponState.tEpeeTrigger) > weaponEpeeDebounce) {
      //Serial.println("Epee trigger");
      tLastActive = t_now;
      weaponState.epeeOn = epeeState;
      weaponState.tEpeeInterOn = t_now;
      weaponState.epeeInterOn = true;
      //Serial.println("Epee trigger");
    }
  }

  if ( (weaponState.epeeInterOn) &&  ((t_now - weaponState.tEpeeInterOn) > weaponState.tLightChange) ) {
    weaponState.epeeInterOn = false;
    //Serial.println("Epee trigger");
  }
  if ( (weaponState.foilInterOn) &&  ((t_now - weaponState.tFoilInterOn) > weaponState.tLightChange) ) {
    weaponState.foilInterOn = false;
  }
}


void updateWeaponStateDigital() {
  //Update this code to use the A, B, C line displays
  // A = red/Epee
  // B = yellow/ Intermittent
  // C = green / Foil

  long t_now = millis();
  //long tic=micros();

  bool epeeState = !nrf_gpio_pin_read(LineADetect); //Inputs are HIGH, pulled LOW by line B
  bool foilState = !nrf_gpio_pin_read(LineCDetect); //Inputs are HIGH, pulled LOW by line B

  numSamples++;

  if (foilState != weaponState.foilOn) {
    if ((t_now - weaponState.tFoilTrigger) > weaponFoilDebounce) {
      weaponState.foilOn = foilState;
      weaponState.tFoilInterOn = t_now;
      weaponState.foilInterOn = true;
      tLastActive=t_now;
    }
  }

  if (epeeState != weaponState.epeeOn) {
    if ((t_now - weaponState.tEpeeTrigger) > weaponEpeeDebounce) {
      weaponState.epeeOn = epeeState;
      weaponState.tEpeeInterOn = t_now;
      weaponState.epeeInterOn = true;
      tLastActive=t_now;
    }
  }

  if ( (weaponState.epeeInterOn) &&  ((t_now - weaponState.tEpeeInterOn) > weaponState.tLightChange) ) {
    weaponState.epeeInterOn = false;
    //Serial.println("Epee trigger");
  }
  if ( (weaponState.foilInterOn) &&  ((t_now - weaponState.tFoilInterOn) > weaponState.tLightChange) ) {
    weaponState.foilInterOn = false;
  }


  //weaponState.cableDC=true;

}

void checkButtonState() {
  long t_now = millis();
  bool newState = digitalRead(BUTTON_PIN);
  static long tButtonPress = 0;
  static bool buttonState = LOW;
  static long tSwitch = 0;
  bool calibrationMode = false;

  if (newState != buttonState) {
    //Serial.print(F("tButtonPress="));  Serial.println(tButtonPress);
    //Serial.print(F("t_now=")); Serial.println(t_now);
    tLastActive = t_now;
    if (newState == LOW) { //Button has been released
      if (((t_now - tButtonPress) > weaponFoilDebounce) && ((t_now - tSwitch) > tModeSwitchLockOut) ) {
        Serial.println("Button pressed");
        switch (BoxState) {
          case CABLE:
            setBoxMode(WPN_GRAPH);
            break;
          case WPN_GRAPH:
            setBoxMode(HIT_CAPTURE);
            break;
          case HIT_CAPTURE:
            setBoxMode(WPN_TEST);
            break;
          case WPN_TEST:
            setBoxMode(PROBE);
            break;
          case PROBE:
            setBoxMode(CABLE);
            break;
          case BOX_IDLE:
            setBoxMode(CABLE);
            break;
        }
        tSwitch = millis();
      }
    }
    tButtonPress = t_now;
    buttonState = newState;
  }

  if (buttonState == HIGH) { //Button press goes high
    if ((t_now - tButtonPress) > tPowerOffPress) {
      //lcd.clear();
      //lcd.setCursor(0, 0);
      //lcd.print(F("  Power off  "));
      tftDisplayMessage("Power off");
      while (digitalRead(BUTTON_PIN) == HIGH) {
        NRF_WDT->RR[0] = WDT_RR_RR_Reload; //Reload watchdog register 0
        delay(100);        
        if ((millis() - tButtonPress) > tEnterCalibrationMode) {
          //lcd.clear();
          //lcd.setCursor(0, 0);
          //lcd.print(F("Calibration"));
          tftDisplayMessage("Calibration");
          calibrationMode = true;
        }
      }
      if (calibrationMode) {
        //Start the watchdog override
        //app_timer_start(wdtOverrideTimer,6554,true); 
        calibrateSystem();
        //app_timer_stop(wdtOverrideTimer);
      } else {
        setPowerOff();
      }
    }
  }
}


void setPowerOff() {
  //setBoxMode('w');
  #if FAST_LED_ACTIVE
  lameLED=CRGB::Black;
  FastLED.show();
  #endif
  tft.fillScreen(BLACK);
  tft.setCursor(25, 50);
  tft.setTextSize(3);
  tft.setTextColor(CYAN);
  tft.print("Good-bye");
  delay(200);
  digitalWrite(POWER_CONTROL, LOW); //Turn the box off*/
  for (int k=0; k<10; k++) {
    NRF_WDT->RR[0] = WDT_RR_RR_Reload; //Reload watchdog register 0
    delay(1000);    
  }
}

void CheckBatteryStatus() {
  int batteryCnts = 0;
  bool ADCActive = false;
  long ave_data = 0;
  int sampleCount = 0;
  int adc_val = 0;
  byte numCycles = 4;
  byte oldAIn = 0;
  nrf_saadc_value_t *adcRead;

  switch (BoxState) {
    case CABLE:
    case HIT_CAPTURE:
    case WPN_GRAPH:
      ADCActive = true;
      StopADC();
      oldAIn = NRF_SAADC->CH[ADC_UNIT].PSELP;
      break;
    case WPN_TEST:
      ADCActive = false;
      break;
  }
  //BatteryCheck=true;

  NRF_SAADC->CH[ADC_UNIT].PSELP = 7; //AIN 6+1
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
      sampleCount++;
    }
  }

  batteryCnts = ave_data / sampleCount;
  //BatteryCheck=false;
  if (ADCActive) {
    NRF_SAADC->CH[ADC_UNIT].PSELP = oldAIn;
    StartADC();
  }

  //Serial.print(F("Battery counts = ")); Serial.println(batteryCnts);
  batteryVoltage = 2.0 * (float(batteryCnts) * 0.6 * 4 / 4095); //Assuming internal reference

}
