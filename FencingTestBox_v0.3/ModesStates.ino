void setWeaponTestMode() {
  StopADC();

  bitClear(PORTD, MUX_LATCH); //equivalent to digitalWrite(4, LOW); Toggle the SPI
  SPI.beginTransaction(MUX_SPI_SETTINGS);
  SPI.transfer(MUX_WEAPON_MODE);
  SPI.endTransaction();
  bitSet(PORTD, MUX_LATCH); //equivalent to digitalWrite(4, HIGH); Toggle the SPI

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("  Weapon Mode"));

  lineAGauge.setColor(ledColorRed);
  lineBGauge.setColor(ledColorYellow);
  lineCGauge.setColor(ledColorGreen);

  lineAGauge.setOffOn(true);
  lineBGauge.setOffOn(true);
  lineCGauge.setOffOn(true);

  FastLED.show();
  delay(500);

  lineAGauge.setOffOn(false);
  lineBGauge.setOffOn(false);
  lineCGauge.setOffOn(false);
  FastLED.show();

  setWeaponInterrupts();

  weaponState.foilOn = ~((BANANA_DIG_IN & bananaC.digitalInMask) > 0);
  weaponState.epeeOn = ((BANANA_DIG_IN & bananaA.digitalInMask) == 0);
  weaponState.tFoilTrigger = 0;
  weaponState.update_flag = true;
  weaponState.tEpeeTrigger = 0;

  /*if (Serial.availableForWrite()==64) {
    Serial.println(F("Weapon Mode Activated"));
    }*/
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
  //Disable weapon interrupts
  PCMSK0 &= ~(1 << weaponState.epeeInterruptBit);
  PCMSK0 &= ~(1 << weaponState.foilInterruptBit);

  //bitClear(PCIFR, PCIF0); //Enable interrupts
  bitClear(PCICR, PCIE0); //Disable pin change interrupts

  bitClear(PORTD, MUX_LATCH); //equivalent to digitalWrite(4, LOW); Toggle the SPI
  SPI.beginTransaction(MUX_SPI_SETTINGS);
  SPI.transfer(B00000000); //Disable all MUX outputs to get offset
  SPI.endTransaction();
  bitSet(PORTD, MUX_LATCH);

  StopADC();

  bananaA.analogIn->maxval = 0;
  bananaB.analogIn->maxval = 0;
  bananaC.analogIn->maxval = 0;

  //noInterrupts();

  //Set to inputs
  DDRB &= ~(1 << bananaA.directionBit);
  DDRB &= ~(1 << bananaB.directionBit);
  DDRB &= ~(1 << bananaC.directionBit);

  //Pull all outputs low
  PORTB &= ~(1 << bananaA.stateBit);
  PORTB &= ~(1 << bananaB.stateBit);
  PORTB &= ~(1 << bananaC.stateBit);

  //Reset the LEDs
  //InitializeCableLEDs();

  //FastLED.show();
  //lcd.clear();
  //lcd.setCursor(0,0);
  //lcd.print(F("  Cable Test"));

  // Re-initialize the ADC
  InitializeADC();
  loadCalibrationData();

  ActiveCh=ChanArray[0].nextChannel;
  //ChanArray[0].setADCChannelActive();
  BoxState = 'c';
  tLastActive = millis();
  updateCableState();
  StartADC();
}

void updateCableState() {
  byte statusCheck;
  //static long tLastConnect = 0;
  //long tNow = millis();
  static byte priorStatus=0;

  if (ChanArray[0].getRawValue()<CABLE_DISCONNECT_THRESHOLD) {
    cableState.ohm_AA = ChanArray[0].getValue();
    cableState.ohm_AAMax = ChanArray[0].getDecayMaxValue();
  }
  else {
    cableState.ohm_AA = OPEN_CIRCUIT_VALUE;
    cableState.ohm_AAMax = OPEN_CIRCUIT_VALUE;
  }

  if (ChanArray[4].getRawValue()<CABLE_DISCONNECT_THRESHOLD) {
    cableState.ohm_BB = ChanArray[4].getValue();
    cableState.ohm_BBMax = ChanArray[4].getDecayMaxValue();
  }
  else {
    cableState.ohm_BB = OPEN_CIRCUIT_VALUE;
    cableState.ohm_BBMax = OPEN_CIRCUIT_VALUE;
  }

  if (ChanArray[5].getRawValue()<CABLE_DISCONNECT_THRESHOLD) {
    cableState.ohm_CC = ChanArray[5].getValue();
    cableState.ohm_CCMax = ChanArray[5].getDecayMaxValue();
  }
  else {
    cableState.ohm_CC = OPEN_CIRCUIT_VALUE;
    cableState.ohm_CCMax = OPEN_CIRCUIT_VALUE;
  }

  cableState.line_AB = ChanArray[1].decay_min;
  cableState.line_AC = ChanArray[3].decay_min;
  cableState.line_BC = ChanArray[2].decay_min;

  bitWrite(cableState.statusByte, BITAA, cableState.ohm_AAMax > HIGH_RESISTANCE_THRESHOLD);
  bitWrite(cableState.statusByte, BITBB, cableState.ohm_BBMax > HIGH_RESISTANCE_THRESHOLD);
  bitWrite(cableState.statusByte, BITCC, cableState.ohm_CCMax > HIGH_RESISTANCE_THRESHOLD);

  bitWrite(cableState.statusByte, BITAB, cableState.line_AB < maxADCthreshold);
  bitWrite(cableState.statusByte, BITAC, cableState.line_AC < maxADCthreshold);
  bitWrite(cableState.statusByte, BITBC, cableState.line_BC < maxADCthreshold);

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
    }
  } else {
    cableState.tLastConnect = millis();
    cableState.cableDC = false;
  }
  
  statusCheck = ((1 << BITBB) | (1 << BITCC));
  if ( (cableState.statusByte & ~(1<<BITAA)) ==statusCheck) {
    if ((cableState.ohm_AA < OPEN_CIRCUIT_VALUE) && ((cableState.ohm_BB >= OPEN_CIRCUIT_VALUE) && (cableState.ohm_CC >= OPEN_CIRCUIT_VALUE)) ) {
      cableState.lameMode = true;
    }
  } else {
    cableState.lameMode = false;
  }  

}

void setWeaponInterrupts() {

  StopADC();
  noInterrupts(); //Disable interrupts while messing with pins

  bitClear(PORTB, bananaB.stateBit); //Set B low

  bitClear(DDRB, bananaA.directionBit); //Set A as input
  bitSet(DDRB, bananaB.directionBit); //Set B as output
  bitClear(DDRB, bananaC.directionBit); //Set C as input

  //All lines have pull down resistors
  bitClear(PORTB, bananaA.stateBit); //Disable pull-up
  bitSet(PORTB, bananaB.stateBit); //Set B High
  bitClear(PORTB, bananaC.stateBit); //Disable pull-up

  PCMSK0 |= (1 << weaponState.epeeInterruptBit);
  PCMSK0 |= (1 << weaponState.foilInterruptBit);

  //PCIFR |= (1 << PCIF0); //Enable interrupts
  PCICR |= (1 << PCIE0);

  interrupts();
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

/*
  bool weaponTestAC() {
  byte DDRold = DDRB;
  byte PORTold = PORTB;
  byte PinChangeState = PCICR;

  PCICR &= (0 << PCIE0); //Disable pin change interrupts

  DDRB |= (1 << bananaA.directionBit); //Set A as output
  DDRB &= ~(1 << bananaB.directionBit); //B as input
  DDRB &= ~(1 << bananaC.directionBit); //C as input

  PORTB |= (1 << bananaB.stateBit); //Enable B pull-up to pull H
  PORTB |= (1 << bananaC.stateBit); //Enable C pull-up to pull H
  PORTB &= ~(1 << bananaA.stateBit); //Set A low
  delayMicroseconds(20); //Delay for stabilization

  bool return_val = ~(BANANA_DIG_IN & bananaC.digitalInMask);

  DDRB = DDRold; //Reset to original configuration
  PORTold = PORTB;
  delayMicroseconds(20); //Wait to stabilize before returning

  PCICR = PinChangeState;

  return return_val;
  }*/

/*
void updateWeaponResistance() {
  long t_now=millis();

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
  }

}*/

void updateWeaponState() {
  //Update this code to use the A, B, C line displays
  // A = red/Epee
  // B = yellow/ Intermittent
  // C = green / Foil

  long t_now = millis();
  //long tic=micros();

  byte state = BANANA_DIG_IN;
  bool foilState = ((state & bananaC.digitalInMask) > 0); //Inputs are low, pulled H by line B
  bool epeeState = ((state & bananaA.digitalInMask) > 0); //Inputs are low, pulled H by line B

  numSamples++;

  if (foilState != weaponState.foilOn) {
    //Serial.println(t_now);
    if ((t_now - weaponState.tFoilTrigger) > weaponFoilDebounce) {
      weaponState.foilOn = foilState;
      //foilLED.setOffOn(foilState);
      lineCGauge.setOffOn(foilState);
      //weaponState.tFoilTrigger=t_now;
      weaponState.tFoilInterOn = t_now;
      lineBGauge.setOffOn(true);
      //lcd.setCursor(0, 1);
      //lcd.print(F("  Foil Hit!  "));
    }
  }

  if (epeeState != weaponState.epeeOn) {
    //Serial.println(t_now);
    if ((t_now - weaponState.tEpeeTrigger) > weaponEpeeDebounce) {
      weaponState.epeeOn = epeeState;
      //epeeLED.setOffOn(epeeState);
      lineAGauge.setOffOn(epeeState);
      //weaponState.tFoilTrigger=t_now;
      weaponState.tEpeeInterOn = t_now;
      //epeeInterLED.setOffOn(true);
      lineBGauge.setOffOn(true);
      //lcd.setCursor(0, 1);
      //lcd.print(F("  Epee Hit!  "));
    }
  }

  if (lineBGauge.isOn()) {
    if (((t_now - weaponState.tFoilInterOn) > weaponStateHoldTime) &&
        ((t_now - weaponState.tEpeeInterOn) > weaponStateHoldTime)) {
      lineBGauge.setOffOn(false);
      //lcd.setCursor(0, 1);
      //lcd.print(F("                "));
    }
  }

  //long toc=micros();

  //Serial.print("Loop time= ");Serial.print(toc-tic);Serial.println(" us");
}

void checkButtonState() {
  long t_now = millis();
  bool newState=bitRead(PINE,PINE6);
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
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print(F("  Power off  "));
      while (bitRead(PINE, PINE6) == HIGH) {
        delay(100);
        if ((millis() - tButtonPress) > tEnterCalibrationMode) {
          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print(F("Calibration"));
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
  setBoxMode('w');
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("  Good bye  "));
  delay(500);
  CheckBatteryStatus();
  displayBatteryStatus();
  delay(2000);
  bitClear(PORTD, PORTD7); //Turn the box off
}

void CheckBatteryStatus() {
  byte oldADMUX = ADMUX;
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
  }
}
