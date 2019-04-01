void InitializeADC() {
  // Configures the basic ADC, does not set ADMUX channels.

  ADMUX = 0;              // clear ADMUX register
  ADMUX |= (1 << REFS1);  // set reference voltage
  ADMUX |= (1 << REFS0);  // set reference voltage to internal 2.56V
  ADMUX |= (1 << ADLAR);  // left align ADC value to 8 bits from ADCH register

  ADCSRA = 0;             // clear ADCSRA register
  ADCSRA |= (1 << ADEN);  // enable ADC
  // sampling rate is [ADC clock] / [prescaler] / [conversion clock cycles]
  // for Arduino Uno ADC clock is 16 MHz and a conversion takes 13 clock cycles
  ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);    // 128 prescaler for 9.6 KHz
  //ADCSRA |= (1 << ADPS2) | (1 << ADPS1);    // 64 prescaler for 18.5 kHz
  //ADCSRA |= (1 << ADPS2) | (1 << ADPS0);    // 32 prescaler for 38.5 KHz
  //ADCSRA |= (1 << ADPS2);                     // 16 prescaler for 76.9 KHz
  //ADCSRA |= (1 << ADPS1) | (1 << ADPS0);    // 8 prescaler for 153.8 KHz
  ADCSRA &= ~(1 << ADATE); // disable auto trigger
  ADCSRA |= (1 << ADIE);  // enable interrupts when measurement complete

  ADCSRB = 0;             // clear ADCSRB register

  DIDR0 = B11110000;  //Disable all digital inputs on analog input channels
  //DIDR2 = B00000111;  //Disable all digital inputs on analog channels
  //bitSet(ADCSRA, ADSC); //Set the first sample
}

void StartADC() {
  ADCSRA &= ~(1 << ADIE); //Disable interrupts
  ADCSRA &= ~(1 << ADATE); //Disable auto trigger
  ADCSRA |= (1 << ADEN); //Enable ADC
  ADCSRA |= (1 << ADIE); //Enable interrupts
  ADCSRA |= (1 << ADSC);  //Start ADC measurements
}

void StopADC() {
  ADCSRA &= ~(1 << ADIE); //Disable interrupts (leave ADC running)
  ADCSRA &= ~(1 << ADATE); // disable auto trigger
}

void InitializeChannels() {
  //PORTB=0;
  //DDRB=0;

  snprintf(ChanArray[0].ch_label,4,"AAA");
  ChanArray[0].muxSetting = MUX_CABLE_AA;
  
  snprintf(ChanArray[1].ch_label,4," AB");
  ChanArray[1].muxSetting = MUX_CABLE_AB;
  
  snprintf(ChanArray[2].ch_label,4," AC");
  ChanArray[2].muxSetting = MUX_CABLE_AC;
 
  snprintf(ChanArray[3].ch_label,4," BA");
  ChanArray[3].muxSetting = MUX_CABLE_BA;

  snprintf(ChanArray[4].ch_label,4,"BBB");
  ChanArray[4].muxSetting = MUX_CABLE_BB;

  snprintf(ChanArray[5].ch_label,4," BC");
  ChanArray[5].muxSetting = MUX_CABLE_BC;

  snprintf(ChanArray[6].ch_label,4," CA");
  ChanArray[6].muxSetting = MUX_CABLE_CA;

  snprintf(ChanArray[7].ch_label,4," CB");
  ChanArray[7].muxSetting = MUX_CABLE_CB;

  snprintf(ChanArray[8].ch_label,4,"CCC");
  ChanArray[8].muxSetting = MUX_CABLE_CC;
  
  for (int k = 0; k < (NUM_ADC_SCAN_CHANNELS); k++) {
    ChanArray[k].nextChannel = &(ChanArray[ChannelScanOrder[k]]);
    ChanArray[k].ADC_Scan = true;
  }
  ActiveCh = ChanArray; //Pointer to the first channel
  
  //ChanArray[NUM_ADC_SCAN_CHANNELS - 1].nextChannel = &(ChanArray[0]); //Loop back to the 1st item
  //ActiveCh = ChanArray; //Pointer to the first channel

  FoilADC.muxSetting=MUX_WEAPON_FOIL;
  FoilADC.nextChannel=&(EpeeADC);
  snprintf(FoilADC.ch_label,4,"FBC");
  
  EpeeADC.muxSetting=MUX_WEAPON_EPEE;
  EpeeADC.nextChannel=&(FoilADC);
  snprintf(EpeeADC.ch_label,4,"AAB");
  
  /*
  for (int k=0;k<NUM_ADC_SCAN_CHANNELS;k++) {
    Serial.print("Channel ");Serial.print(k);Serial.print("=");Serial.print((long)&(ChanArray[k]),HEX);
    Serial.print("\tNext=");Serial.println((long)ChanArray[k].nextChannel,HEX);
  }*/

  bananaA.directionBit = DDB4;
  bananaA.stateBit = PORTB4;
  bananaA.digitalInMask = (1 << PINB4);

  bananaB.directionBit = DDB5;
  bananaB.stateBit = PORTB5;
  bananaB.digitalInMask = (1 << PINB5);

  bananaC.directionBit = DDB6;
  bananaC.stateBit = PORTB6;
  bananaC.digitalInMask = (1 << PINB6);
}

void calibrateSystem() {
  bool cal_valid = false;
  unsigned int ave_data = 0;
  int adc_val;
  long tPress = 0;
  byte errorCount=0;
  byte lb=0;
  int hb=0;

  setCableTestMode();

  ADCSRA &= ~(1 << ADIE); //Disable interrupts (leave ADC running)
  ADCSRA &= ~(1 << ADATE); //Disable auto trigger

  for (int k = 0; k < NUM_ADC_SCAN_CHANNELS; k++) {
    switch (ChanArray[k].ch_label[0]) {
      case 'A':
      case 'B':
      case 'C':        

        //Set the proper channel MUX
        bitClear(PORTD, MUX_LATCH); //equivalent to digitalWrite(4, LOW); Toggle the SPI
        SPI.beginTransaction(MUX_SPI_SETTINGS);
        SPI.transfer(ChanArray[k].muxSetting);
        SPI.endTransaction();
        bitSet(PORTD, MUX_LATCH);  //equivalent to digitalWrite(4,HIGH);
        ChanArray[k].setADCChannelActive();
        
        cal_valid = false;
        errorCount=0;
        while (!cal_valid) {        
                    
          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print(F("Short Line "));
          lcd.print(ChanArray[k].ch_label[0]);
          lcd.setCursor(0, 1);
          lcd.print(F("Push button"));

          while (bitRead(PINE, PINE6) == LOW) {}; //Wait until button pressed

          ave_data = 0;
          for (int j = 0; j < ADC_Channel::NUM_AVE; j++) {
            bitSet(ADCSRA, ADSC);
            while (bitRead(ADCSRA, ADSC)) {
              //delayMicroseconds(500);
            }
            lb=(ADCL>>6); //Need to left shift by 6 places
            hb=ADCH;
            adc_val = (hb<<2)+lb;
            ave_data += adc_val;
          }
          adc_val = (ave_data >> ADC_Channel::NUM_AVE_POW2);

          lcd.clear();
          lcd.setCursor(0, 1);
          snprintf(lcdString2, 16, "Cal %c: %u", ChanArray[k].ch_label[0], adc_val);
          lcd.print(lcdString2);
          if (adc_val < calibrationErrorValue) {
            errorCount=0;
            lcd.setCursor(0, 0);
            lcd.print(F("Cal Success"));
            ChanArray[k].setTrim(adc_val);
            cal_valid = true;
            delay(1500);
          } else {
            errorCount++;
            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print(F("Cal Error"));
            delay(2000);
            cal_valid = false;
            if (errorCount>=calibrationRetries) {
              lcd.clear();
              lcd.setCursor(0, 0);
              lcd.print(F("Cal Failed"));
              lcd.setCursor(0, 1);
              lcd.print(F("Exiting..."));
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
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("Tap to save"));
  lcd.setCursor(0, 1);
  lcd.print(F("Hold to discard"));

  while (bitRead(PINE, PINE6) == LOW) {}; //Wait until button pressed
  tPress = millis();
  while (bitRead(PINE, PINE6) == HIGH) {
    if ((millis() - tPress) > tPowerOffPress) {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print(F("Discarding"));
    }
    delay(100);
  }

  if ((millis() - tPress) < tPowerOffPress) {
    lcd.clear();
    lcd.setCursor(0, 0);
    writeCalibrationData();
    lcd.print(F("Cal saved."));
    delay(1000);
  } else {
    loadCalibrationData();
    delay(500);
  }
  // Automatically go to cable test mode
  setCableTestMode();
}
