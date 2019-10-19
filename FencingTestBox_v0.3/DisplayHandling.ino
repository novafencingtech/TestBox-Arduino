void updateLEDDisplay(char BoxMode) {
  switch (BoxMode) {
    case 'c':
      updateCableLED();
      break;
    case 'w':
      break;
    case 'r':
      updateWeaponResistanceLED();
      break;
  }
}

void updateLCDDisplay(char BoxMode) {

  switch (BoxMode) {
    case 'c':
      updateCableLCD();
      break;
    case 'w':
      //updateWeaponLCD();
      break;
    case 'r':
      updateWeaponResistanceLCD();
      break;
  }
}

void updateWeaponResistanceLCD() {
  char numString[OHM_FIELD_WIDTH + 1] = "\0";

  strcpy_P(lcdString1, PSTR("Epee = "));

  strcpy_P(numString, PSTR(" -- "));
  if (weaponState.ohm_Epee < OPEN_CIRCUIT_VALUE) {
    dtostrf(weaponState.ohm_Epee, OHM_FIELD_WIDTH, 2, numString);
  }
  strncat(lcdString1, numString, OHM_FIELD_WIDTH);

  strcpy_P(lcdString2, PSTR("Foil = "));

  strcpy_P(numString, PSTR(" -- "));
  if (weaponState.ohm_Foil < OPEN_CIRCUIT_VALUE) {
    dtostrf(weaponState.ohm_Foil, OHM_FIELD_WIDTH, 2, numString);
  }
  strncat(lcdString2, numString, OHM_FIELD_WIDTH);
  //strncat_P(lcdString2,PSTR("  "),2);

  while (strlen(lcdString1) < LCD_TEXT_COLS) {
    strcat(lcdString1, " ");
  }
  while (strlen(lcdString2) < LCD_TEXT_COLS) {
    strcat(lcdString2, " ");
  }
  lcd.setCursor(0, 0);
  lcd.print(lcdString1);
  lcd.setCursor(0, 1);
  lcd.print(lcdString2);
}
/*
  void updateWeaponLCD(){
    lcd.setCursor(0,1);

    if (foilInterLED.isOn() && epeeInterLED.isOn() ){
      lcd.print(F("!!  AC Short  !!"));
      return;
    }
    if (foilInterLED.isOn()) {
      lcd.print(F("  Foil Hit  "));
      return;
    }
    if (epeeInterLED.isOn()) {
      lcd.print(F("  Epee Hit  "));
    }
    lcd.print(F("                "));
  }*/

bool CheckCablestatusWord(uint16_t errorCheck) {
  return ((cableState.statusWord & errorCheck) == errorCheck);
}

void updateCableLCD() {
  static uint16_t priorStatus;
  uint16_t statusCheck = ((1 << BITAA) | (1 << BITBB) | (1 << BITCC));
  char numString[OHM_FIELD_WIDTH + 1] = "";

  strcpy_P(lcdString2, PSTR(""));

  strcpy_P(numString, PSTR(" -- "));
  if (cableState.ohm_AA < OPEN_CIRCUIT_VALUE) {
    dtostrf(cableState.ohm_AA, OHM_FIELD_WIDTH, 2, numString);
  }

  strncat(lcdString2, numString, OHM_FIELD_WIDTH);
  strncat_P(lcdString2, PSTR("  "), 2);

  strcpy_P(numString, PSTR(" -- "));
  if (cableState.ohm_BB < OPEN_CIRCUIT_VALUE) {
    dtostrf(cableState.ohm_BB, OHM_FIELD_WIDTH, 2, numString);
  }
  strncat(lcdString2, numString, OHM_FIELD_WIDTH);
  strncat_P(lcdString2, PSTR("  "), 2);

  strcpy_P(numString, PSTR(" -- "));
  if (cableState.ohm_CC < OPEN_CIRCUIT_VALUE) {
    dtostrf(cableState.ohm_CC, OHM_FIELD_WIDTH, 2, numString);
  }
  strncat(lcdString2, numString, OHM_FIELD_WIDTH);

  lcd.setCursor(0, 1);
  lcd.print(lcdString2);
  //strcat(lcdString2,"\n");
  //Serial.write(lcdString2);

  if (cableState.statusWord == 0) {
    lcd.setCursor(0, 0);
    lcd.print(F("   Cable GOOD   "));
    return;
  }

  if (!(cableState.statusWord == priorStatus)) {
    priorStatus = cableState.statusWord;
    tLastActive = millis();
  }

  //Clear all bits other than the main lines, check if they're all disconnected
  if (cableState.cableDC) {
    lcd.setCursor(0, 0);
    lcd.print(F("Cable disconnect"));
    return;
  }

  if (cableState.lameMode) {
    lcd.setCursor(0, 0);
    if (!bitRead(cableState.statusWord, BITAA)) {
      lcd.print(F("    Lame Good   "));
    } else {
      lcd.print(F(" Bad spot/cable "));
    }
    return;
  }

  strcpy(lcdString1, "!");
  statusCheck = ((1 << BITAA) | (1 << BITBB) | (1 << BITCC));
  if ((cableState.statusWord & statusCheck) > 0) {
    if bitRead(cableState.statusWord, BITAA) {
      strcat(lcdString1, "A");
    }
    if bitRead(cableState.statusWord, BITBB) {
      strcat(lcdString1, "B");
    }
    if bitRead(cableState.statusWord, BITCC) {
      strcat(lcdString1, "C");
    }
    strcat(lcdString1, " ");
  }

  statusCheck = ( (1 << BITAB) );
  if ( (CheckCablestatusWord(statusCheck)) && (strlen(lcdString1) < (LCD_TEXT_COLS - 4)) ) {
    sprintf(numString, "%c%c%c ", 'A', 126, 'B');
    strcat(lcdString1, numString);
  }
  statusCheck = ( (1 << BITBA) );
  if ( (CheckCablestatusWord(statusCheck)) && (strlen(lcdString1) < (LCD_TEXT_COLS - 4)) ) {
    sprintf(numString, "%c%c%c ", 'B', 126, 'A');
    strcat(lcdString1, numString);
  }

  statusCheck = ( (1 << BITAC) );
  if ( (CheckCablestatusWord(statusCheck)) && (strlen(lcdString1) < (LCD_TEXT_COLS - 4)) ) {
    sprintf(numString, "%c%c%c ", 'A', 126, 'C');
    strcat(lcdString1, numString);
  }
  statusCheck = ( (1 << BITCA) );
  if ( (CheckCablestatusWord(statusCheck)) && (strlen(lcdString1) < (LCD_TEXT_COLS - 4)) ) {
    sprintf(numString, "%c%c%c ", 'C', 126, 'A');
    strcat(lcdString1, numString);
  }

  statusCheck = ( (1 << BITBC) );
  if ( (CheckCablestatusWord(statusCheck)) && (strlen(lcdString1) < (LCD_TEXT_COLS - 4)) ) {
    sprintf(numString, "%c%c%c ", 'B', 126, 'C');
    strcat(lcdString1, numString);
  }
  statusCheck = ( (1 << BITCB) );
  if ( (CheckCablestatusWord(statusCheck)) && (strlen(lcdString1) < (LCD_TEXT_COLS - 3)) ) {
    sprintf(numString, "%c%c%c", 'C', 126, 'B');
    strcat(lcdString1, numString);
  }

  while (strlen(lcdString1) < LCD_TEXT_COLS) {
    strcat(lcdString1, " ");
  }
  lcd.setCursor(0, 0);
  lcd.print(lcdString1);
}


void displayBatteryStatus() {
  char voltString[5] = "\0";
  char percString[5] = "\0";
  float battPercent = 0;

  if (batteryVoltage > 3.4) {
    battPercent = (batteryVoltage - 3.4) / (4.2 - 3.4) * 100;
  }

  lcd.clear();
  dtostrf(batteryVoltage, 4, 2, voltString);
  dtostrf(battPercent, 3, 0, percString);
  lcd.print(F("  Battery   "));
  lcd.setCursor(0, 1);
  snprintf(lcdString2, 16, " %sV  %s%% ", voltString, percString);
  lcd.print(lcdString2);
}


void CreateDisplay() {
  lcd.begin(LCD_TEXT_COLS, LCD_TEXT_ROWS);
  lcd.setBacklight(100);
  lcd.home(); lcd.clear();
  lcd.print(F("Fencing Test Box"));
  lcd.setCursor(0, 1);
  lcd.print(F("by Graham Allen"));
}

/*
void writeSerialOutput(char Mode) {
  // Use Generic format "WF01E00CA00.0B00.0C00.0AB etc
  // First letter: W/C indicates mode
  // F - Foil, On/Off, Intermittent light
  // E - Epee , On/Off, Intermittent light
  // C - Cable, 4-digit float
  const long tHeartBeatInterval = 5000; //Time in ms
  const byte tempBufferSize = 8;
  byte bufferSize = SERIAL_MAX_LEN;
  static char tempString1[tempBufferSize];
  static char tempString2[tempBufferSize];
  static long tHeartBeat = 0;

  //char printStringBuffer[SERIAL_MAX_LEN];

  long t_now = millis();
  static long t_last_upd = 0;
  float EffSampleRate = 0;


  tempString1[0] = '\0'; //Reset the temp String
  tempString2[0] = '\0'; //Reset the temp String
  outputString[0] = '\0'; //Reset the output string

  if ((t_now - tHeartBeat) > tHeartBeatInterval) {
    float dt = (float(t_now - t_last_upd) * 1.0e-3);
    EffSampleRate = float(numSamples) / dt;

    dtostrf(EffSampleRate, 5, 0, tempString1); //Effective sample rate
    dtostrf(batteryVoltage, 5, 2, tempString2);

    snprintf(outputString, bufferSize, "t=%ld,N=%ld,%sHz,Bat=%sV\r\n", t_now, numSamples, tempString1, tempString2);
    if (Serial.availableForWrite() > strlen(outputString)) {
      Serial.write(outputString);
    }
    tHeartBeat = millis();
    t_last_upd = millis();
    numSamples = 0;
  }

  outputString[0] = '\0'; //Reset the outputString
  switch (Mode) {
    case 'c':
      //Print the status byte

      dtostrf(cableState.ohm_AA, 5, 2, tempString1);
      dtostrf(cableState.ohm_AAMax, 5, 2, tempString2);

      snprintf(outputString, bufferSize, "C1,%ld,%u,A,%s,%s,B,", t_now, cableState.statusWord, tempString1, tempString2);
      dtostrf(cableState.ohm_BB, 5, 2, tempString1);
      dtostrf(cableState.ohm_BBMax, 5, 2, tempString2);
      strncat(outputString, tempString1, bufferSize); strncat(outputString, ",", bufferSize); strncat(outputString, tempString2, bufferSize);
      dtostrf(cableState.ohm_CC, 5, 2, tempString1);
      dtostrf(cableState.ohm_CCMax, 5, 2, tempString2);
      strncat(outputString, ",C,", bufferSize); strncat(outputString, tempString1, bufferSize); strncat(outputString, ",", bufferSize); strncat(outputString, tempString2, bufferSize);
      strncat(outputString, "\r\n", bufferSize);

      if (Serial.availableForWrite() > strlen(outputString)) {
        Serial.write(outputString);
      }

      snprintf(outputString, bufferSize, "C2,%ld,%u,", t_now, cableState.statusWord);
      snprintf(tempString1, tempBufferSize, "AB,%u", cableState.line_AB);
      strncat(outputString, tempString1, bufferSize);
      snprintf(tempString1, tempBufferSize, ",AC,%u", cableState.line_AC);
      strncat(outputString, tempString1, bufferSize);
      snprintf(tempString1, tempBufferSize, ",BA,%u", cableState.line_BA);
      strncat(outputString, tempString1, bufferSize);
      snprintf(tempString1, tempBufferSize, ",BC,%u", cableState.line_BC);
      strncat(outputString, tempString1, bufferSize);
      snprintf(tempString1, tempBufferSize, ",CA,%u", cableState.line_CA);
      strncat(outputString, tempString1, bufferSize);
      snprintf(tempString1, tempBufferSize, ",CB,%u", cableState.line_CB);
      strncat(outputString, tempString1, bufferSize);
      strcat(outputString, "\r\n");

      if (Serial.availableForWrite() > strlen(outputString)) {
        Serial.write(outputString);
      }

      break;
    case 'w':
      //Format is "W,time,E,Light On/Off,t last epee trigger, F, Foil light On/Off, t last foil trigger
      snprintf(outputString, bufferSize, "W,%ld,E,%u,%ld,F,%u,%ld\r\n", t_now, weaponState.epeeOn, weaponState.tEpeeTrigger, weaponState.foilOn, weaponState.tFoilTrigger);
      if (Serial.availableForWrite() > strlen(outputString)) {
        Serial.write(outputString);
      }
      break;
    case 'r':
      //Format is "R,time,Epee resistance, Foil resistance"
      snprintf(outputString, bufferSize, "R,%ld,E,", t_now);
      dtostrf(weaponState.ohm_Epee, 5, 2, tempString1);
      dtostrf(weaponState.ohm_Foil, 5, 2, tempString2);
      strncat(outputString, tempString1, bufferSize); strncat(outputString, ",F,", bufferSize); strncat(outputString, tempString2, bufferSize); strncat(outputString, "\r\n", bufferSize);
      if (Serial.availableForWrite() > strlen(outputString)) {
        Serial.write(outputString);
      }
      break;
  }
}*/
