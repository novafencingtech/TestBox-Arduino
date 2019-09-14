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

  tLastActive = millis();
  BoxState = 'w';
}

void setWeaponResistanceMode() {
  setCableTestMode();
  StopADC();
  ActiveCh = &(FoilADC);

  BoxState = 'r';
  StartADC();

}

void setCableTestMode() {
  detachInterrupt(LineADetect);
  detachInterrupt(LineCDetect);

  digitalWrite(MUX_LATCH, LOW); //equivalent to digitalWrite(4, LOW); Toggle the SPI
  shiftOut(MUX_DATA, MUX_CLK, MSBFIRST, (byte) 0x0);
  digitalWrite(MUX_LATCH, HIGH);

  StopADC();

  // Re-initialize the ADC
  InitializeADC();
  loadCalibrationData();

  ActiveCh = &(ChanArray[0]);

  BoxState = 'c';
  tLastActive = millis();
  //updateCableState();
  StartADC();
}

void updateCableState() {
  uint16_t statusCheck;
  //static long tLastConnect = 0;
  //long tNow = millis();
  static uint16_t priorStatus = 0;
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
    arm_biquad_cascade_df1_f32(&(cableState.LineALowPass), &tempValue, &(cableState.ohm_CC), 1);
    cableState.ohm_CCMax = ChanArray[8].getDecayMaxValue();
  }
  else {
    cableState.ohm_CC = OPEN_CIRCUIT_VALUE;
    cableState.ohm_CCMax = OPEN_CIRCUIT_VALUE;
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
    cableState.cableOhm[k] = ChanArray[k].getValue();
    ChanArray[k].valueReady = false;
  }

  bitWrite(cableState.statusByte, BITAA, cableState.ohm_AAMax > HIGH_RESISTANCE_THRESHOLD);
  bitWrite(cableState.statusByte, BITBB, cableState.ohm_BBMax > HIGH_RESISTANCE_THRESHOLD);
  bitWrite(cableState.statusByte, BITCC, cableState.ohm_CCMax > HIGH_RESISTANCE_THRESHOLD);

  bitWrite(cableState.statusByte, BITAB, cableState.line_AB < shortADCthreshold);
  bitWrite(cableState.statusByte, BITAC, cableState.line_AC < shortADCthreshold);
  bitWrite(cableState.statusByte, BITBA, cableState.line_BA < shortADCthreshold);
  bitWrite(cableState.statusByte, BITBC, cableState.line_BC < shortADCthreshold);
  bitWrite(cableState.statusByte, BITCA, cableState.line_CA < shortADCthreshold);
  bitWrite(cableState.statusByte, BITCB, cableState.line_CB < shortADCthreshold);

  if (priorStatus != cableState.statusByte) {
    tLastActive = millis();
    priorStatus = cableState.statusByte;
  }

  if (cableState.cableDC) {
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
    if ((millis() - cableState.tLastConnect) > idleDisconnectTime) {
      cableState.cableDC = true;
      cableState.lameMode = false;
      cableState.maskMode = false;
    }
  } else {
    cableState.tLastConnect = millis();
    cableState.cableDC = false;
  }

  //Check if only testing a mask cord (line A-A only)
  statusCheck = ((1 << BITAA) | (1 << BITBB));
  if ( (cableState.statusByte & ~(1 << BITCC)) == statusCheck) {
    if ((cableState.ohm_AA >= OPEN_CIRCUIT_VALUE) && ((cableState.ohm_BB >= OPEN_CIRCUIT_VALUE) && (cableState.ohm_CC < OPEN_CIRCUIT_VALUE)) ) {
      cableState.lameMode = true;
      cableState.maskMode = false;
    }
  } else {
    cableState.lameMode = false;
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
  StopADC();

  attachInterrupt(LineADetect, &ISR_EpeeHitDetect, CHANGE);
  attachInterrupt(LineCDetect, &ISR_FoilHitDetect, CHANGE);

}


void setBoxMode(char mode) {
  switch (mode) {
    case 'c':
      setCableTestMode();
      break;
    case 'w':
      setWeaponTestMode();
      break;
    case 'r':
      setWeaponResistanceMode();
      break;
    case 'i':
      setIdleMode();
      break;
  }
}

void setIdleMode() {
  
}

void updateIdleMode() {  

  for (int k = 0; k < NUM_ADC_SCAN_CHANNELS; k++) {
    while (!ChanArray[k].valueReady) {
      delay(5);
    }
  }
  updateCableState();

  //if cableState


  StopADC();

  digitalWrite(MUX_LATCH, LOW); //equivalent to digitalWrite(4, LOW); Toggle the SPI
  shiftOut(MUX_DATA, MUX_CLK, MSBFIRST, MUX_WEAPON_MODE);
  digitalWrite(MUX_LATCH, HIGH); //equivalent to digitalWrite(4, HIGH); Toggle the SPI

  nrf_gpio_cfg(LineADetect, NRF_GPIO_PIN_DIR_INPUT, NRF_GPIO_PIN_INPUT_CONNECT, NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_S0S1, NRF_GPIO_PIN_SENSE_LOW);
  nrf_gpio_cfg(LineCDetect, NRF_GPIO_PIN_DIR_INPUT, NRF_GPIO_PIN_INPUT_CONNECT, NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_S0S1, NRF_GPIO_PIN_SENSE_HIGH);

  setWeaponInterrupts();
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

  weaponState.ohm10xEpee = int(weaponState.ohm_Epee * 10+0.5);
  weaponState.ohm10xFoil = int(weaponState.ohm_Foil * 10+0.5);

  weaponState.lineAC = (WeaponAC.getRawValue() < shortADCthreshold);

  if ( (weaponState.ohm_Epee == OPEN_CIRCUIT_VALUE) && (weaponState.ohm_Foil == OPEN_CIRCUIT_VALUE) ) {
    if ((t_now - weaponState.tLastConnect) > idleDisconnectTime) {
      weaponState.cableDC = true;
    }
  }

}

void updateWeaponState() {
  //Update this code to use the A, B, C line displays
  // A = red/Epee
  // B = yellow/ Intermittent
  // C = green / Foil

  long t_now = millis();
  static bool oldEpeeState = false;
  static bool oldFoilState = false;

  bool epeeState = EpeeADC.hsBuffer.CheckTriggerLastSample();
  bool foilState = FoilADC.hsBuffer.CheckTriggerLastSample();

  if (foilState != weaponState.foilOn) {
    if ((t_now - weaponState.tFoilTrigger) > weaponFoilDebounce) {
      weaponState.foilOn = foilState;
      weaponState.tFoilInterOn = t_now;
      weaponState.foilInterOn = true;
      //Serial.println("Foil trigger");
    }
  }

  if (epeeState != weaponState.epeeOn) {
    if ((t_now - weaponState.tEpeeTrigger) > weaponEpeeDebounce) {
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
    }
  }

  if (epeeState != weaponState.epeeOn) {
    if ((t_now - weaponState.tEpeeTrigger) > weaponEpeeDebounce) {
      weaponState.epeeOn = epeeState;
      weaponState.tEpeeInterOn = t_now;
      weaponState.epeeInterOn = true;
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

void checkButtonState() {
  long t_now = millis();
  bool newState = digitalRead(BUTTON_PIN);
  static long tButtonPress = 0;
  static bool buttonState = LOW;
  static long tSwitch = 0;
  bool calibrationMode = false;

  if (newState != buttonState) {
    //Serial.print(F("PINE = ")); Serial.println(PINE);
    //Serial.print(F("tButtonPress="));  Serial.println(tButtonPress);
    //Serial.print(F("t_now=")); Serial.println(t_now);
    tLastActive = t_now;
    if (newState == LOW) { //Button has been released
      if (((t_now - tButtonPress) > weaponFoilDebounce) && ((t_now - tSwitch) > tModeSwitchLockOut) ) {
        switch (BoxState) {
          case 'c':
            setBoxMode('r');
            break;
          case 'r':
            setBoxMode('w');
            break;
          case 'w':
            setBoxMode('c');
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
      while (digitalRead(BUTTON_PIN) == HIGH) {
        delay(100);
        if ((millis() - tButtonPress) > tEnterCalibrationMode) {
          //lcd.clear();
          //lcd.setCursor(0, 0);
          //lcd.print(F("Calibration"));
          calibrationMode = true;
        }
      }
      if (calibrationMode) {
        calibrateSystem();
      } else {
        setPowerOff();
      }
    }
  }
}


void setPowerOff() {
  //setBoxMode('w');
  /*lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(F("  Good bye  "));
    delay(500);
    CheckBatteryStatus();
    displayBatteryStatus();*/
  delay(2000);
  digitalWrite(POWER_CONTROL, LOW); //Turn the box off*/
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
    case 'c':
    case 'r':
      ADCActive = true;
      StopADC();
      oldAIn = NRF_SAADC->CH[ADC_UNIT].PSELP;
      break;
    case 'w':
      ADCActive = false;
      break;
  }

  NRF_SAADC->CH[ADC_UNIT].PSELP = 7; //AIN 6+1
  nrf_saadc_task_trigger(NRF_SAADC_TASK_START);
  ave_data = 0;
  sampleCount = 0;
  tic = micros();
  for (int j = 0; j < numCycles; j++) {
    nrf_saadc_buffer_init(ADC_Buffer1, ADC_BUFFER_SIZE);
    nrf_saadc_event_clear(NRF_SAADC_EVENT_RESULTDONE);
    nrf_saadc_event_clear(NRF_SAADC_EVENT_END);
    nrf_saadc_task_trigger(NRF_SAADC_TASK_START);
    while (!nrf_saadc_event_check(NRF_SAADC_EVENT_END)) {
      nrf_saadc_task_trigger(NRF_SAADC_TASK_SAMPLE);
      while (nrf_saadc_busy_check()) {}
    }
    adcRead = nrf_saadc_buffer_pointer_get();
    for (int m = 0; m < nrf_saadc_amount_get(); m++) {
      adc_val = adcRead[m];
      ave_data += adc_val;
      sampleCount++;
    }
  }

  batteryCnts = ave_data / sampleCount;
  if (ADCActive) {
    NRF_SAADC->CH[ADC_UNIT].PSELP = oldAIn;
    StartADC();
  }

  //Serial.print(F("Battery counts = ")); Serial.println(batteryCnts);
  batteryVoltage = 2.0 * (float(batteryCnts) * 0.6 * 4 / 4095); //Assuming internal reference

}
