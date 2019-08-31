void BlinkLEDThenPowerOff() {
  digitalWrite(LED2_PIN,LOW);
  for (int k=0;k<10;k++) {
      digitalWrite(LED2_PIN,HIGH);
      delay(100);
      digitalWrite(LED2_PIN,LOW);
      delay(100);
      digitalWrite(LED2_PIN,HIGH);
      delay(100);
      digitalWrite(LED2_PIN,LOW);
      delay(700);
  }
  digitalWrite(POWER_CONTROL,LOW);
}


void setWeaponTestMode() {
  StopADC();

  digitalWrite(MUX_LATCH,LOW); //equivalent to digitalWrite(4, LOW); Toggle the SPI
  shiftOut(MUX_DATA, MUX_CLK, MSBFIRST, MUX_WEAPON_MODE);
  digitalWrite(MUX_LATCH,HIGH); //equivalent to digitalWrite(4, HIGH); Toggle the SPI

  nrf_gpio_cfg(LineADetect, NRF_GPIO_PIN_DIR_INPUT, NRF_GPIO_PIN_INPUT_CONNECT, NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_S0S1, NRF_GPIO_PIN_SENSE_LOW);
  nrf_gpio_cfg(LineCDetect, NRF_GPIO_PIN_DIR_INPUT, NRF_GPIO_PIN_INPUT_CONNECT, NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_S0S1, NRF_GPIO_PIN_SENSE_HIGH);

  setWeaponInterrupts();

  //weaponState.foilOn = ~((BANANA_DIG_IN & bananaC.digitalInMask) > 0);
  //weaponState.epeeOn = ((BANANA_DIG_IN & bananaA.digitalInMask) == 0);
  //weaponState.tFoilTrigger = 0;
  //weaponState.update_flag = true;
  //weaponState.tEpeeTrigger = 0;

  tLastActive = millis();
  BoxState = 'w';
}
/*
void setWeaponResistanceMode() {
  setCableTestMode();
  StopADC();
  ActiveCh=&(FoilADC);

  BoxState = 'r';
  StartADC();

}*/

void setCableTestMode() {
  //nrfx_gpiote_in_event_disable(LineADetect);
  //nrfx_gpiote_in_event_disable(LineCDetect);
  detachInterrupt(LineADetect);
  detachInterrupt(LineCDetect);

  digitalWrite(MUX_LATCH,LOW); //equivalent to digitalWrite(4, LOW); Toggle the SPI
  shiftOut(MUX_DATA, MUX_CLK, MSBFIRST, (byte) 0x0);
  digitalWrite(MUX_LATCH,HIGH);

  StopADC();

  // Re-initialize the ADC
  InitializeADC();
  loadCalibrationData();

  ActiveCh=&(ChanArray[0]);
  
  BoxState = 'c';
  tLastActive = millis();
  //updateCableState();
  StartADC();
}

void updateCableState() {
  uint16_t statusCheck;
  //static long tLastConnect = 0;
  //long tNow = millis();
  static uint16_t priorStatus=0;
  float tempValue=0;

  if (ChanArray[0].getRawValue()<CABLE_DISCONNECT_THRESHOLD) {
    tempValue = ChanArray[0].getValue();
    arm_biquad_cascade_df1_f32(&(cableState.LineALowPass),&tempValue,&(cableState.ohm_AA),1);
    cableState.ohm_AAMax = ChanArray[0].getDecayMaxValue();
  }
  else {
    cableState.ohm_AA = OPEN_CIRCUIT_VALUE;
    cableState.ohm_AAMax = OPEN_CIRCUIT_VALUE;
  }

  if (ChanArray[4].getRawValue()<CABLE_DISCONNECT_THRESHOLD) {
    tempValue = ChanArray[4].getValue();
    arm_biquad_cascade_df1_f32(&(cableState.LineBLowPass),&tempValue,&(cableState.ohm_BB),1);
    cableState.ohm_BBMax = ChanArray[4].getDecayMaxValue();
  }
  else {
    cableState.ohm_BB = OPEN_CIRCUIT_VALUE;
    cableState.ohm_BBMax = OPEN_CIRCUIT_VALUE;
  }

  if (ChanArray[8].getRawValue()<CABLE_DISCONNECT_THRESHOLD) {
    tempValue = ChanArray[8].getValue();
    arm_biquad_cascade_df1_f32(&(cableState.LineALowPass),&tempValue,&(cableState.ohm_CC),1);
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

  for (int k=0;k<NUM_ADC_SCAN_CHANNELS;k++) {
    cableState.cableOhm[k]=ChanArray[k].getValue();
    ChanArray[k].valueReady=false;
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

  if (priorStatus!=cableState.statusByte) {
    tLastActive=millis();
    priorStatus=cableState.statusByte;
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
  if ( (cableState.statusByte & ~(1<<BITCC)) ==statusCheck) {
    if ((cableState.ohm_AA >= OPEN_CIRCUIT_VALUE) && ((cableState.ohm_BB >= OPEN_CIRCUIT_VALUE) && (cableState.ohm_CC < OPEN_CIRCUIT_VALUE)) ) {
      cableState.lameMode = true;
      cableState.maskMode = false;
    }
  } else {
    cableState.lameMode = false;
  }  

  statusCheck = ((1 << BITBB) | (1 << BITCC));
  if ( (cableState.statusByte & ~(1<<BITAA)) ==statusCheck) {
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

  /*if (!nrfx_gpiote_is_init()) {
    nrfx_gpiote_init();
  }*/

  attachInterrupt(LineADetect,&ISR_EpeeHitDetect,CHANGE);
  attachInterrupt(LineCDetect,&ISR_FoilHitDetect,CHANGE);
  //nrfx_gpiote_in_init(LineADetect, &weaponPinConfig,&ISR_EpeeHitDetect);
  //nrfx_gpiote_in_init(LineCDetect, &weaponPinConfig,&ISR_EpeeHitDetect);
  //nrfx_gpiote_in_event_enable(LineADetect,true); 
  //nrfx_gpiote_in_event_enable(LineCDetect,true); 

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
      //setWeaponResistanceMode();
      break;
  }
}


void updateWeaponResistance() {
  long t_now=millis();
/*
  if (FoilADC.getRawValue()<CABLE_DISCONNECT_THRESHOLD) {
    weaponState.ohm_Foil = FoilADC.getValue();
    weaponState.ohm_FoilMax = FoilADC.getDecayMaxValue();
    weaponState.tLastConnect=t_now;
    weaponState.cableDC=false;
  } else {
    weaponState.ohm_Foil=OPEN_CIRCUIT_VALUE;
  }
  if (EpeeADC.getRawValue()<CABLE_DISCONNECT_THRESHOLD) {
    weaponState.ohm_Epee = EpeeADC.getValue();
    weaponState.ohm_EpeeMax = EpeeADC.getDecayMaxValue();
    weaponState.tLastConnect=t_now;
    weaponState.cableDC=false;
  } else {
    weaponState.ohm_Epee=OPEN_CIRCUIT_VALUE;
  }

  if ( (weaponState.ohm_Epee==OPEN_CIRCUIT_VALUE) && (weaponState.ohm_Foil==OPEN_CIRCUIT_VALUE) ) {
    if ((t_now-weaponState.tLastConnect)>idleDisconnectTime) {
      weaponState.cableDC=true;
    }
  }*/

}

void updateWeaponState() {
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
    }
  }

  if (epeeState != weaponState.epeeOn) {
    if ((t_now - weaponState.tEpeeTrigger) > weaponEpeeDebounce) {
      weaponState.epeeOn = epeeState;
      weaponState.tEpeeInterOn = t_now;
    }
  }

  /*if (lineBGauge.isOn()) {
    if (((t_now - weaponState.tFoilInterOn) > weaponStateHoldTime) &&
        ((t_now - weaponState.tEpeeInterOn) > weaponStateHoldTime)) {
      //lineBGauge.setOffOn(false);
      //lcd.setCursor(0, 1);
      //lcd.print(F("                "));
    }
  }*/
}

void checkButtonState() {
  long t_now = millis();
  bool newState=digitalRead(BUTTON_PIN);
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
      if (((t_now - tButtonPress) > weaponFoilDebounce) && (t_now - tSwitch) > tModeSwitchLockOut) {
        switch (BoxState) {
          case 'c':
            setBoxMode('w');
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
    buttonState=newState;
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
        //calibrateSystem();
      } else {
        setPowerOff();
      }
    }
  }
}


void setPowerOff() {
  setBoxMode('w');
  /*lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("  Good bye  "));
  delay(500);
  CheckBatteryStatus();
  displayBatteryStatus();*/
  delay(2000);
  digitalWrite(POWER_CONTROL,LOW); //Turn the box off*/
}

void CheckBatteryStatus() {
  /*byte oldADMUX = ADMUX;
  byte lowADC = 0;
  word highADC = 0;
  int batteryCnts = 0;
  bool ADCinterruptFlag = bitRead(ADCSRA, ADIE);

  if (ADCinterruptFlag) {
    ADCSRA &= ~(1 << ADIE); //Disable interrupts (leave ADC running)
  }
  bitClear(ADCSRA, ADATE); //Disable auto-trigger
  //Serial.print(F("Analog Read = ")); Serial.println(analogRead(A3));

  while (bit_is_set(ADCSRA, ADSC));  //Let any conversion complete
  
  ADMUX = (B11100100); //Select ADC4 for battery voltage
  delayMicroseconds(500); //Allow Sample and Hold capacitor to charge
  
  for (int k=0; k<ADC_STABLE_CYCLES; k++) {
    bitSet(ADCSRA, ADSC);
    while (bitRead(ADCSRA,ADSC)) {
      delayMicroseconds(25);
    }
    //delayMicroseconds(500);
    //delayMicroseconds(25);
    lowADC = ADCL;
    highADC = ADCH;
  }

  
  //Serial.print(highADC); Serial.print(" "); Serial.print(lowADC);
  batteryCnts = (highADC << 2) | (lowADC >> 6);
  //batteryCnts = ((int)highADC<<8)|(lowADC);
  
  //lowADC=ADCL;
  //highADC=ADCH;
  //batteryCnts=ADCH;
  //batteryCnts=analogRead(A3); //Not sure why analogRead works when other code doesn't
  //Serial.print(F("Battery counts = ")); Serial.println(batteryCnts);
  batteryVoltage = 2.0 * (float(batteryCnts) * 2.56 / 1023); //Assuming internal reference
  //batteryVoltage = 2.0 * (float(batteryCnts) * 2.56 / 256); //Assuming internal reference
  //Serial.print(F("Battery (V) = ")); Serial.println(batteryVoltage);
  //Serial.print(F("Analog Read = ")); Serial.println(analogRead(A3));

  ADMUX = oldADMUX; //Reset ADMUX

  bitWrite(ADCSRA, ADIE, ADCinterruptFlag); //Reset interrupt flag

  if (BoxState == 'c') {
    delayMicroseconds(150); //Allow to stabilize
    bitSet(ADCSRA, ADSC); //Re-start the scanning
  }*/
}
