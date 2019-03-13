void InitializeCableLEDs() {
  lineAGauge.setDisplayMode('g');
  lineAGauge.setValueListMode(false);
  //lineAGauge.setColor(CRGB::Black);
  lineAGauge.setScaleMax(12.0);
  lineAGauge.setMaxEnable(true);

  lineAGauge.clearThresholds();
  lineAGauge.setThreshold(0, 5, ledColorGreen);
  lineAGauge.setThreshold(1, 7, ledColorYellow);
  lineAGauge.setThreshold(2, 9, ledColorOrange);
  lineAGauge.setThreshold(3, 10, ledColorRed);
  lineAGauge.setValue(0.01);

  lineBGauge.setDisplayMode('g');
  lineBGauge.setValueListMode(false);
  //lineBGauge.setColor(CRGB::Black);
  lineBGauge.setScaleMax(12.0);
  lineBGauge.setMaxEnable(true);

  lineBGauge.clearThresholds();
  lineBGauge.setThreshold(0, 5, ledColorGreen);
  lineBGauge.setThreshold(1, 7, ledColorYellow);
  lineBGauge.setThreshold(2, 9, ledColorOrange);
  lineBGauge.setThreshold(3, 10, ledColorRed);
  lineBGauge.setValue(0.01);

  lineCGauge.setDisplayMode('g');
  lineCGauge.setValueListMode(false);
  //lineCGauge.setColor(CRGB::Black);
  lineCGauge.setScaleMax(12);
  lineCGauge.setMaxEnable(true);

  lineCGauge.clearThresholds();
  lineCGauge.setThreshold(0, 5, ledColorGreen);
  lineCGauge.setThreshold(1, 7, ledColorYellow);
  lineCGauge.setThreshold(2, 9, ledColorOrange);
  lineCGauge.setThreshold(3, 10, ledColorRed);
  lineCGauge.setValue(0.01);

  //epeeInterLED.setOffOn(false);
  //foilInterLED.setOffOn(false);
  //FastLED.show();
}

void InitializeLEDs() {
  FastLED.addLeds<LED_TYPE, DATA_PIN_LED_BLOCK1, COLOR_ORDER>(LED_block1, NUM_LEDS_BLOCK1);
  FastLED.setCorrection( TypicalLEDStrip );

  //LED_Display test{LED_block1,12,1,'b'};
  //InitializeCableLEDs();

  //powerLED.setColor(ledColorBlue);
  //FastLED.show();
  //delay(250);
  /*
    epeeLED.setColor(ledColorRed);
    epeeInterLED.setColor(ledColorYellow);
    foilLED.setColor(ledColorGreen);
    foilInterLED.setColor(ledColorYellow);
  */
  //FastLED.show();
  //delay(250);

  lineAGauge.setColor(ledColorRed);
  lineBGauge.setColor(ledColorYellow);
  lineCGauge.setColor(ledColorGreen);
  FastLED.show();
  delay(250);
}

void updateCableLED() {
  static bool isOn = true;
  //static long tNow=millis();
  //static long tLast=millis();
  //static float val=0.0;
  //static byte count=1;
/*
  if ((tNow-tLast)>2000) {
    val=float(random(0,80))*0.1;
    if (count>10) {
      count=1;
    }
  }
  lineAGauge.setValue(val);
  lineBGauge.setValue(count);
  lineCGauge.setValue(count+5);
*/
  
  if (cableState.cableDC) {
    if (isOn) {
      lineAGauge.setOffOn(false);
      lineBGauge.setOffOn(false);
      lineCGauge.setOffOn(false);
      //epeeLED.setOffOn(false);
      //foilLED.setOffOn(false);
      //FastLED.show();
      isOn = false;
    }
    return;
  } else {
    if (!isOn) {
      lineAGauge.setOffOn(true);
      lineBGauge.setOffOn(true);
      lineCGauge.setOffOn(true);
      isOn = true;
    }
  }

  lineAGauge.setValue(cableState.ohm_AA);
  lineAGauge.setMaxValue(cableState.ohm_AAMax);

  if (!(cableState.lameMode)) {
    lineBGauge.setValue(cableState.ohm_BB);
    lineBGauge.setMaxValue(cableState.ohm_BBMax);

    lineCGauge.setValue(cableState.ohm_CC);
    lineCGauge.setMaxValue(cableState.ohm_CCMax);
  } else {
    lineBGauge.setOffOn(false);
    lineCGauge.setOffOn(false);
  }
}

void updateWeaponResistanceLED() {
  static bool isOn = true;

  if (weaponState.cableDC) {
    if (isOn) {
      lineAGauge.setOffOn(false);
      lineBGauge.setOffOn(false);
      lineCGauge.setOffOn(false);
      isOn = false;
    }
    return;
  } else {
    if (!isOn) {
      lineAGauge.setOffOn(true);
      lineBGauge.setOffOn(false);
      lineCGauge.setOffOn(true);
      isOn = true;
    }
  }

  lineAGauge.setValue(weaponState.ohm_Epee);
  lineAGauge.setMaxValue(weaponState.ohm_EpeeMax);

  lineCGauge.setValue(weaponState.ohm_Foil);
  lineCGauge.setMaxValue(weaponState.ohm_FoilMax);
}
