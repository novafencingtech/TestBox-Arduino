void setStatusText(char *msg, SM_RGB color) {
  backgroundLayer.fillRectangle(20,23,43,31,LED_BLACK);
  backgroundLayer.setFont(font3x5);  
  backgroundLayer.drawString(statusTextLocX, statusTextLocY, LED_BLACK, "    ");
  backgroundLayer.drawString(statusTextLocX, statusTextLocY, color, msg);
  backgroundLayer.swapBuffers();
}

void updateLights() {
  static SM_RGB lampLColor = LED_BLACK;
  static SM_RGB lampRColor = LED_BLACK;
  static SM_RGB faultLColor = LED_BLACK;
  static SM_RGB faultRColor = LED_BLACK;

  lampLColor = LED_BLACK;
  if (fa05Lights.offTargetL == true) {
    //Serial.println("Off-target L");
    lampLColor = LED_WHITE_MED;
  }
  if (fa05Lights.touchL == true) {
    //Serial.println("Touch L");
    lampLColor = LED_RED_HIGH;
  }
  faultLColor = LED_BLACK;
  if (fa05Lights.gndFaultL) {
    faultLColor = LED_ORANGE_HIGH;
  }
  if (fa05Lights.lightsOnly) {
    backgroundLayer.fillRectangle(0, 0, 27, 14, LED_BLACK);
    backgroundLayer.fillRectangle(0, 18, 27, 31, LED_BLACK);
    if (fa05Lights.offTargetL == true) {      
      backgroundLayer.fillRectangle(0, 18, 27, 31, LED_WHITE_MED);
    }
    if (fa05Lights.touchL == true) {
      backgroundLayer.fillRectangle(0, 0, 27, 14, LED_RED_HIGH);
    }
    backgroundLayer.fillRectangle(0, 15, 27, 17, faultLColor);
  } else {
    backgroundLayer.fillRectangle(0, 0, 23, 9, lampLColor);
    backgroundLayer.fillRectangle(0, 10, 23, 11, faultLColor);
  }

  lampRColor = LED_BLACK;
  if (fa05Lights.offTargetR == true) {
    //Serial.println("Off-target R");
    lampRColor = LED_WHITE_MED;
  }
  if (fa05Lights.touchR == true) {
    //Serial.println("Touch R");
    lampRColor = LED_GREEN_HIGH;
  }
  faultRColor = LED_BLACK;
  if (fa05Lights.gndFaultR) {
    faultRColor = LED_ORANGE_HIGH;
  }
  if (fa05Lights.lightsOnly) {
    backgroundLayer.fillRectangle(63 - 27, 0, 63, 14, LED_BLACK);
    backgroundLayer.fillRectangle(63 - 27, 18, 63, 31, LED_BLACK);
    if (fa05Lights.offTargetR == true) {
      backgroundLayer.fillRectangle(63 - 27, 18, 63, 31, LED_WHITE_MED);
    }
    if (fa05Lights.touchR == true) {
      backgroundLayer.fillRectangle(63 - 27, 0, 63, 14, LED_GREEN_HIGH);
    }
    backgroundLayer.fillRectangle(63 - 27, 15, 63, 17, faultRColor);
  } else {    
    backgroundLayer.fillRectangle(63-23, 0, 63, 9, lampRColor);
    backgroundLayer.fillRectangle(63-23, 10, 63, 11, faultRColor);
    //backgroundLayer.swapBuffers();
    //Serial.println("Update R Lights");
  }
  Serial.println("Lights updated");
  backgroundLayer.swapBuffers();
}

void updateScore() {
  static uint8_t cardState = 0xff;
  static uint16_t timeRemainSec = 0;
  static uint8_t period = 1;
  static uint8_t scoreL = 0;
  static uint8_t scoreR = 0;
  SM_RGB clockColor = LED_GREEN_MED;
  char buf[5];

  //Process cards first
  if (cardState != fa05Score.cardState) {
    updateCards();
    cardState = fa05Score.cardState;
  }
  // Process clock next
  clockColor = LED_ORANGE_HIGH;
  if (fa05Score.clockRunning) {
    clockColor = LED_GREEN_HIGH;
  }
  backgroundLayer.setFont(font6x10);
  backgroundLayer.fillRectangle(27,1,26+2*6,0+9, LED_BLACK);
  sprintf(buf, "P%1.1u", fa05Score.matchCount);
  backgroundLayer.drawString(27, 1, LED_BLUE_HIGH, buf);
  //backgroundLayer.swapBuffers();  

  backgroundLayer.setFont(gohufont11);
  backgroundLayer.fillRectangle(21,12,21+4*6,20, LED_BLACK);
  sprintf(buf, "%0.1u:%0.2u", fa05Score.timeRemainMin, fa05Score.timeRemainSec);
  backgroundLayer.drawString(21, 12, clockColor, buf);

  // Process score changes
  backgroundLayer.fillRectangle(5,31-8,5+16,31,LED_BLACK);
  //backgroundLayer.setFont(gohufont11b);
  backgroundLayer.setFont(font8x13);
  sprintf(buf, "%-2u", fa05Score.scoreL);
  backgroundLayer.drawString(5, 31 - 10, LED_WHITE_MED, buf);

  backgroundLayer.fillRectangle(63-(16+4),23,58,31,LED_BLACK);
  backgroundLayer.setFont(font8x13);
  sprintf(buf, "%2u", fa05Score.scoreR);
  backgroundLayer.drawString( 63 - (16 + 4), 31 - 10, LED_WHITE_MED, buf);
  backgroundLayer.swapBuffers();
}
void updateCards() {
  SM_RGB cardColor = LED_BLACK;
  
  backgroundLayer.setFont(font5x7);
  if (fa05Score.priorityL)
    backgroundLayer.drawString(4,16,LED_GREEN_MED,"Pr");
  else {
    backgroundLayer.fillRectangle(4,16,4+2*5,16+7,LED_BLACK);
  }
  if (fa05Score.priorityR)
    backgroundLayer.drawString(63-2-2*5,16,LED_GREEN_MED,"Pr");
  else {
    backgroundLayer.fillRectangle(63-2-2*5,16,63-7,16+7,LED_BLACK);
  }

  cardColor = (fa05Score.yellowL) ? LED_YELLOW_MED : LED_BLACK;
  backgroundLayer.fillRectangle(0, 31 - 8, 3, 31 - 5, cardColor);

  cardColor = (fa05Score.redL) ? LED_RED_MED : LED_BLACK;
  backgroundLayer.fillRectangle(0, 31 - 3, 3, 31, cardColor);

  cardColor = (fa05Score.yellowL) ? LED_YELLOW_MED : LED_BLACK;
  backgroundLayer.fillRectangle(63 - 3, 31 - 8, 63, 31 - 5, cardColor);

  cardColor = (fa05Score.redL) ? LED_RED_MED : LED_BLACK;
  backgroundLayer.fillRectangle(63 - 3, 31 - 3, 63, 31, cardColor);
  //backgroundLayer.swapBuffers();
}

void runDemo() {
  backgroundLayer.fillScreen(LED_WHITE_LOW);
  backgroundLayer.swapBuffers();
  delay(500);
  backgroundLayer.fillScreen(LED_BLACK);
  backgroundLayer.swapBuffers();

  setStatusText("Demo",LED_BLUE_MED);
  
  tLastActive = millis();
  fa05Lights.lightsOnly=0;
  fa05Lights.touchL = 1;
  fa05Score.scoreL = 1;
  fa05Score.priorityL=0;
  fa05Score.priorityR=0;
  fa05Lights.gndFaultL = 0;
  fa05Lights.gndFaultR = 0;
  updateLights();
  updateScore();
  delay(5000);

  fa05Lights.offTargetR = 1;
  fa05Lights.touchL = 0;
  fa05Score.scoreR = 88;
  fa05Score.scoreL = 88;
  fa05Score.matchCount = 1;
  fa05Score.timeRemainMin = 2;
  fa05Score.timeRemainSec = 58;
  updateLights();
  updateScore();
  delay(2000);

  fa05Lights.offTargetR = 0;
  fa05Lights.offTargetL = 0;
  fa05Lights.touchL = 1;
  fa05Lights.touchR = 1;
  fa05Score.scoreR = 0;
  fa05Score.scoreL = 0;  
  fa05Score.matchCount = 2;
  updateLights();
  updateScore();
  delay(1000);

  fa05Score.timeRemainMin = 0;
  fa05Score.clockRunning=1;
  fa05Score.timeRemainSec = 07;
  updateScore();
  for (int k=99;k>=0;k--) {
    fa05Score.timeRemainSec = k;
    updateScore();
    delay(50);    
  }
  fa05Score.clockRunning=0;
  
  fa05Lights.touchL = 0;
  fa05Lights.touchR = 0;  
  fa05Lights.offTargetR = 1;
  fa05Lights.offTargetL = 1;
  fa05Lights.gndFaultL = 1;
  fa05Lights.gndFaultR = 1;
  fa05Score.scoreL = 0;
  fa05Score.scoreR = 0;
  fa05Score.yellowL = 1;
  fa05Score.yellowR = 1;
  fa05Score.matchCount = 3;
  fa05Score.priorityL=1;
  fa05Score.priorityR=1;
  updateLights();
  updateScore();
  delay(500);
  
  fa05Score.redL = 1;
  fa05Score.redR = 1;
  updateLights();
  updateScore();
  delay(5000);

  backgroundLayer.fillScreen(LED_BLACK);
  backgroundLayer.swapBuffers();
  fa05Lights.lightsOnly=1;
  fa05Lights.touchL = 1;
  fa05Lights.touchR = 1;  
  fa05Lights.offTargetR = 1;
  fa05Lights.offTargetL = 1;
  fa05Lights.gndFaultL = 1;
  fa05Lights.gndFaultR = 1;
  updateLights();
  delay(5000);
  //updateScore();
}
