
bool CheckCableStatusByte(uint16_t errorCheck){
  return ((cableState.statusByte & errorCheck)==errorCheck);
}
/*
void updateCableLCD() {
  static byte priorStatus;
  byte statusCheck=((1<<BITAA)|(1<<BITBB)|(1<<BITCC));
  char numString[OHM_FIELD_WIDTH+1]="";
  //int strLen;

  strcpy_P(lcdString2,PSTR(""));

  strcpy_P(numString,PSTR(" -- "));
  if (cableState.ohm_AA<OPEN_CIRCUIT_VALUE) {
    dtostrf(cableState.ohm_AA,OHM_FIELD_WIDTH,2,numString);  
  }
  
  strncat(lcdString2,numString,OHM_FIELD_WIDTH);  
  strncat_P(lcdString2,PSTR("  "),2);
  
  strcpy_P(numString,PSTR(" -- "));
  if (cableState.ohm_BB<OPEN_CIRCUIT_VALUE) {
    dtostrf(cableState.ohm_BB,OHM_FIELD_WIDTH,2,numString);  
  }
  strncat(lcdString2,numString,OHM_FIELD_WIDTH);
  strncat_P(lcdString2,PSTR("  "),2);

  strcpy_P(numString,PSTR(" -- "));
  if (cableState.ohm_CC<OPEN_CIRCUIT_VALUE) {
    dtostrf(cableState.ohm_CC,OHM_FIELD_WIDTH,2,numString);  
  }
  strncat(lcdString2,numString,OHM_FIELD_WIDTH);
  
  lcd.setCursor(0,1);
  lcd.print(lcdString2);
  //strcat(lcdString2,"\n");
  //Serial.write(lcdString2);

  if (cableState.statusByte==0) {
    lcd.setCursor(0,0);
    lcd.print(F("   Cable GOOD   "));
    return;
  }

  if (!(cableState.statusByte==priorStatus)) {
    priorStatus=cableState.statusByte;
    tLastActive=millis();
  }
  
  //Clear all bits other than the main lines, check if they're all disconnected
  if (cableState.cableDC) {
      lcd.setCursor(0,0);
      lcd.print(F("Cable disconnect"));
      return;
  }

  if (cableState.lameMode) {
    lcd.setCursor(0,0);
    if (!bitRead(cableState.statusByte,BITAA)) {
      lcd.print(F("    Lame Good   "));
    } else {
      lcd.print(F(" Bad spot/cable "));
    }
    return;
  }

  statusCheck=((1<<BITAA)|(1<<BITBB)|(1<<BITCC));
  if ((cableState.statusByte & statusCheck)>0) {
    strcpy_P(lcdString1,PSTR(" Line "));
    if bitRead(cableState.statusByte,BITAA) {
      strncat_P(lcdString1,PSTR("A"),1);
    }
    if bitRead(cableState.statusByte,BITBB) {
      strncat_P(lcdString1,PSTR("B"),1);
    }
    if bitRead(cableState.statusByte,BITCC) {
      strncat_P(lcdString1,PSTR("C"),1);
    }
    strncat_P(lcdString1,PSTR(" high"),8);
  }
  
  statusCheck=( (1<<BITAB) );
  if (CheckCableStatusByte(statusCheck)) {
    strncpy_P(lcdString1,PSTR("Line AB short"),LCD_TEXT_COLS);
  }
  statusCheck=( (1<<BITBC) );
  if (CheckCableStatusByte(statusCheck)) {
    strncpy_P(lcdString1,PSTR("Line BC short"),LCD_TEXT_COLS);
  }
  statusCheck=( (1<<BITAC) );
  if (CheckCableStatusByte(statusCheck)) {
    strncpy_P(lcdString1,PSTR("Line AC short"),LCD_TEXT_COLS);
  }
  statusCheck=( (1<<BITAA)|(1<<BITBB)|(1<<BITAB) );
  if (CheckCableStatusByte(statusCheck)) {
    strncpy_P(lcdString1,PSTR("Line AB cross"),LCD_TEXT_COLS);
  }
  statusCheck=( (1<<BITBB)|(1<<BITBC)|(1<<BITBC) );
  if (CheckCableStatusByte(statusCheck)) {
    strncpy_P(lcdString1,PSTR("Line BC cross"),LCD_TEXT_COLS);
  }
  statusCheck=( (1<<BITAA)|(1<<BITCC)|(1<<BITAC) );
  if (CheckCableStatusByte(statusCheck)) {
    strncpy_P(lcdString1,PSTR("Line AC cross"),LCD_TEXT_COLS);
  }
  statusCheck=((1<<BITAA)|(1<<BITBB)|(1<<BITCC)|(1<<BITAB)|(1<<BITBC));
    if (CheckCableStatusByte(statusCheck)) {
      strncpy(lcdString1,"BAD! A->B B->C  ",LCD_TEXT_COLS);
    }
  statusCheck=((1<<BITAA)|(1<<BITBB)|(1<<BITCC)|(1<<BITAC));
    if (CheckCableStatusByte(statusCheck)) {
      strncpy(lcdString1,"BAD! A->C  B->A",LCD_TEXT_COLS);
    }    

  while (strlen(lcdString1)<LCD_TEXT_COLS) {
    strcat(lcdString1," ");
  }
  lcd.setCursor(0,0);
  lcd.print(lcdString1);      
}*/


void displayBatteryStatus() {
  char voltString[5]="\0";
  char percString[5]="\0";
  float battPercent=0;

  if (batteryVoltage>3.4) {
    battPercent=(batteryVoltage-3.4)/(4.2-3.4)*100;
  }  
  /*
  lcd.clear();
  dtostrf(batteryVoltage,4,2,voltString);
  dtostrf(battPercent,3,0,percString);
  lcd.print(F("  Battery   "));
  lcd.setCursor(0,1);
  snprintf(lcdString2,16," %sV  %s%% ",voltString,percString);
  lcd.print(lcdString2);*/
}


void CreateDisplay() {

  /*
  lcd.begin(LCD_TEXT_COLS, LCD_TEXT_ROWS);
  lcd.setBacklight(100);
  lcd.home(); lcd.clear();
  lcd.print(F("Fencing Test Box"));
  lcd.setCursor(0,1);
  lcd.print(F("by Graham Allen"));*/
}

void writeSerialOutput(char Mode){
  // Use Generic format "WF01E00CA00.0B00.0C00.0AB etc
  // First letter: W/C indicates mode
  // F - Foil, On/Off, Intermittent light 
  // E - Epee , On/Off, Intermittent light 
  // C - Cable, 4-digit float
  const long tHeartBeatInterval = 5000; //Time in ms
  const byte tempBufferSize=16;
  int bufferSize=SERIAL_BUFFER_SIZE;
  char tempString1[tempBufferSize];
  char tempString2[tempBufferSize];
  static long tHeartBeat=0;
  
  //char printStringBuffer[SERIAL_BUFFER_SIZE];
  
  long t_now=millis();
  static long t_last_upd=0;
  float EffSampleRate=0;
  
  
  tempString1[0]='\0';  //Reset the temp String
  tempString2[0]='\0';  //Reset the temp String
  //tempString3[0]='\0';  //Reset the temp String
  outputString[0]='\0';  //Reset the output string

  if ((t_now-tHeartBeat)>tHeartBeatInterval) {
    float dt = (float(t_now - t_last_upd) * 1.0e-3);
    EffSampleRate = float(numSamples) / dt;

    dtostrf(EffSampleRate,5,0,tempString1); //Effective sample rate
    dtostrf(batteryVoltage,5,2,tempString2);
    snprintf(outputString,bufferSize,"t=%ld, N=%ld, %sHz, %c, Bat=%sV\r\n",t_now,numSamples,tempString1,BoxState,tempString2);
    Serial.write(outputString);
    tHeartBeat=millis();
    t_last_upd=millis();
    numSamples=0;
  }
  
  outputString[0]='\0'; //Reset the outputString  
  switch (Mode) {
    case 'c':
      //Print the status byte
      dtostrf(cableState.ohm_AA,5,2,tempString1);
      dtostrf(cableState.ohm_AAMax,5,2,tempString2);

      snprintf(outputString,bufferSize,"C1,%ld,%u,A,%s,%s,B,",t_now,cableState.statusByte,tempString1,tempString2);
      dtostrf(cableState.ohm_BB,5,2,tempString1);
      dtostrf(cableState.ohm_BBMax,5,2,tempString2);
      strncat(outputString,tempString1,bufferSize);strncat(outputString,",",bufferSize);strncat(outputString,tempString2,bufferSize);
      dtostrf(cableState.ohm_CC,5,2,tempString1);
      dtostrf(cableState.ohm_CCMax,5,2,tempString2);
      strncat(outputString,",C,",bufferSize); strncat(outputString,tempString1,bufferSize);strncat(outputString,",",bufferSize);strncat(outputString,tempString2,bufferSize);
      strncat(outputString,"\r\n",bufferSize);
      if (Serial) {
         Serial.write(outputString);
         Serial.flush();
      }
      snprintf(outputString,bufferSize,"C2,%ld,%u",t_now,cableState.statusByte);
      snprintf(tempString1,tempBufferSize,",AA,%d",cableState.line_AA);
      strncat(outputString,tempString1,bufferSize);
      snprintf(tempString1,tempBufferSize,",AB,%d",cableState.line_AB);
      strncat(outputString,tempString1,bufferSize);
      snprintf(tempString1,tempBufferSize,",AC,%d",cableState.line_AC);
      strncat(outputString,tempString1,bufferSize);
      snprintf(tempString1,tempBufferSize,",BA,%d",cableState.line_BA);
      strncat(outputString,tempString1,bufferSize);
      snprintf(tempString1,tempBufferSize,",BB,%d",cableState.line_BB);
      strncat(outputString,tempString1,bufferSize);
      snprintf(tempString1,tempBufferSize,",BC,%d",cableState.line_BC);
      strncat(outputString,tempString1,bufferSize);
      snprintf(tempString1,tempBufferSize,",CA,%d",cableState.line_CA);
      strncat(outputString,tempString1,bufferSize);
      snprintf(tempString1,tempBufferSize,",CB,%d",cableState.line_CB);
      strncat(outputString,tempString1,bufferSize);
      snprintf(tempString1,tempBufferSize,",CC,%d",cableState.line_CC);
      strncat(outputString,tempString1,bufferSize);strncat(outputString,"\r\n",bufferSize);
      /*if (Serial.availableForWrite()>strlen(outputString) ){
         Serial.write(outputString);
      }*/
      /*
      dtostrf(cableState.ohm_AA,5,2,tempString1);
      dtostrf(cableState.ohm_BB,5,2,tempString2);
      dtostrf(cableState.ohm_CC,5,2,tempString3);
      //Serial.print(tempString1); Serial.print(tempString2); Serial.println(tempString3);
      snprintf(outputString,bufferSize,"Rave (A B C) =%u, %s, %s, %s\r\n\0",cableState.statusByte,tempString1,tempString2,tempString3); 
      Serial.write(outputString);
      
      dtostrf(cableState.ohm_AAMax,5,2,tempString1);
      dtostrf(cableState.ohm_BBMax,5,2,tempString2);
      dtostrf(cableState.ohm_CCMax,5,2,tempString3);      
      snprintf(outputString,bufferSize,"Rmax =, %s, %s, %s\r\n\0",tempString1,tempString2,tempString3); 
      Serial.write(outputString);
            
      snprintf(outputString,bufferSize,"Xadc (AB AC BC)=, %d, %d, %d\r\n\0",cableState.line_AB,cableState.line_AC,cableState.line_BC);
      Serial.write(outputString); 
      */
      break; 
    case 'w':
      //Format is "W,time,E,Light On/Off,t last epee trigger, F, Foil light On/Off, t last foil trigger
      snprintf(outputString,bufferSize,"W,%ld,E,%u,%ld,F,%u,%ld\r\n",t_now,weaponState.epeeOn,weaponState.tEpeeTrigger,weaponState.foilOn,weaponState.tFoilTrigger);
      //Serial.write(outputString);
      break;
    case 'r':
      //Format is "R,time,Epee resistance, Foil resistance"      
      snprintf(outputString,bufferSize,"R,%ld,E,",t_now);
      dtostrf(weaponState.ohm_Epee,5,2,tempString1);
      dtostrf(weaponState.ohm_Foil,5,2,tempString2);
      strncat(outputString,tempString1,bufferSize);strncat(outputString,",F,",bufferSize);strncat(outputString,tempString2,bufferSize);
      break;      
  }
  
  Serial.write(outputString);
  
  /*if (SerialBT.available()){
    SerialBT.print(outputString);  
  }*/
}
