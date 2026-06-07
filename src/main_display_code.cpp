#ifdef BUILD_INCLUDE_MAIN_DISPLAY_CODE


void setScreenBrightness(uint16_t brightness)
{
  if (currentBrightnessSet != brightness)
  {
    amoled.setBrightness(brightness);
    currentBrightnessSet = brightness;
  }
}

void startupScreen()
{
  if (compositeSprite)
  {
    compositeSprite->fillSprite(TFT_YELLOW);
    compositeSprite->setTextColor(TFT_BLUE);
    resetCompositeSpriteCursor();
    compositeSprite->printf("Initialising Sensors...");
    mapScreen->copyCompositeSpriteToDisplay();
    delay(1000);
  }
}

void writePowerStatsToCompositeSprite()
{
  PowersSY6970& power = static_cast<PowersSY6970&>(amoled.SY);

  uint16_t vBattVoltage = power.getBattVoltage();
  uint16_t vSystemVoltage = power.getSystemVoltage();
  uint16_t vBusVoltage = power.getVbusVoltage();
  uint16_t constcurr = power.getChargerConstantCurr();
  uint16_t tarvolts = power.getChargeTargetVoltage();

  int limited = 0;
  uint16_t inputLimitI2C = 0;
  int autoDetect = 0;
  
  compositeSprite->printf("Batt = %hu mV\n",vBattVoltage);
  compositeSprite->printf("%s\n", power.getChargeStatusString());
  compositeSprite->printf("%s\n", power.getBusStatusString());
  compositeSprite->printf("Bus V = %hu mV\n",vBusVoltage);
  compositeSprite->printf("Sys V = %hu mV\n",vSystemVoltage);

  compositeSprite->printf("Const Curr = %hu mA\n",constcurr);
  compositeSprite->printf("Tar Volts = %hu mV\n",tarvolts);
}

void recoveryScreen()
{
  if (compositeSprite)
  {
    compositeSprite->fillSprite(TFT_DARKGREEN);
    compositeSprite->setTextColor(TFT_WHITE);
    resetCompositeSpriteCursor();
    compositeSprite->printf("Press Top Button\nfor Recovery OTA\n\n");
    writePowerStatsToCompositeSprite();
    mapScreen->copyCompositeSpriteToDisplay();
/*
    PowersSY6970& power = static_cast<PowersSY6970&>(amoled);

//    power.setChargerConstantCurr(1024); // was 1024
//    power.disableCurrentLimitPin();
//    power.setAutoChargerTypeDetectionEnabled(false);
//    power.setCurrentInputLimit(1500);  // was 1500

    uint16_t vBattVoltage = power.getBattVoltage();
    uint16_t vSystemVoltage = power.getSystemVoltage();
    uint16_t vBusVoltage = power.getVbusVoltage();
    uint16_t constcurr = power.getChargerConstantCurr();
    uint16_t tarvolts = power.getChargeTargetVoltage();

    int limited = 0;
    uint16_t inputLimitI2C = 0;
    int autoDetect = 0;
//    int limited = power.isEnableCurrentLimitPin();
//    uint16_t inputLimitI2C = power.getCurrentInputLimit();
//    int autoDetect = power.getAutoChargeDetectionEnabled();
    
    compositeSprite->printf("Batt = %hu mV\n",vBattVoltage);
    compositeSprite->printf("Charge = %s\n", power.getChargeStatusString());
    compositeSprite->printf("Bus = %s\n", power.getBusStatusString());
    compositeSprite->printf("Bus V = %hu mV\n",vBusVoltage);
    compositeSprite->printf("Sys V = %hu mV\n",vSystemVoltage);

    compositeSprite->printf("Const Curr = %hu mA\n",constcurr);
    compositeSprite->printf("Tar Volts = %hu mV\n",tarvolts);
    compositeSprite->printf("Limited Pin = %i\n",limited);
    compositeSprite->printf("Input Limit = %hu\n",inputLimitI2C);
    compositeSprite->printf("AutoDetect = %i\n",autoDetect);
    
    mapScreen->copyCompositeSpriteToDisplay();
    */
  }

  const uint32_t end = millis() + 3000;
  while (end > millis())
  {
    checkGoProButtons();
    delay(20);
  }

  if (doInitialSerialReceiveEchoTest)
  {
    compositeSprite->fillSprite(TFT_BLUE);
    mapScreen->copyCompositeSpriteToDisplay();

    while(true)
    {
      if (Serial1.available())
      {
          char r = Serial1.read();
          Serial1.write(r);
          compositeSprite->println(r);
          mapScreen->copyCompositeSpriteToDisplay();
      }
    }
  }

  if (doInitialSerialTransmitTest)
  {
    compositeSprite->fillSprite(TFT_ORANGE);
    mapScreen->copyCompositeSpriteToDisplay();
 // test transmit from oceanic to reef
    const char start_char = '!';
    const char end_char = '_';
    char j=start_char;
    for (int i=0;i<10000000; i++)
    {
      Serial1.write(j);
      j++;
      if (j>end_char)
      {
        j=start_char;
      }
//      delay(5);
    }
  }

}

void waitingForMakoESPNowScreen()
{
  compositeSprite->fillSprite(espScanBackColour);
  resetCompositeSpriteCursor();
  compositeSprite->setTextColor(espScanForeColour,espScanBackColour);
  compositeSprite->println("ESP Now Initialised\nAwait Mako ESP Now Message\n");
  writePowerStatsToCompositeSprite();
  mapScreen->copyCompositeSpriteToDisplay();
}

void awaitingFirstFixScreen(int msgsReceived)
{
  compositeSprite->setTextColor(TFT_YELLOW,espScanBackColour);
  compositeSprite->fillSprite(espScanBackColour);
  compositeSprite->setTextWrap(false,true);
  compositeSprite->printf("\nAwaiting first fix\nNo Fix messages received: %d\n",msgsReceived);
  mapScreen->copyCompositeSpriteToDisplay();
}

void resetMap()
{
  mapScreen->drawDiverOnBestFeaturesMapAtCurrentZoom(latitude, longitude, heading);
}

void testMapDisplay()
{
  double latitude = 51.4605855;    // lightning boat
  double longitude = -0.54890166666666; 
  double heading = 0.0;
  mapScreen->drawDiverOnBestFeaturesMapAtCurrentZoom(latitude,longitude,heading);
}


void executeTests(bool refreshMap)
{
   if (diveTrackTest)
  {
    mapScreen->drawDiverOnBestFeaturesMapAtCurrentZoom(diveTrack[trackIndex]._la,diveTrack[trackIndex]._lo,diveTrack[trackIndex]._h + (correctForReversedCompassTrackTest ? 180 : 0));
    cycleTrackIndex();
  }

  if (diveTraceTest)
  {
    if (latitudeDelta != 0 || longitudeDelta != 0)
    {
      latitude+=latitudeDelta;
      longitude+=longitudeDelta;
      mapScreen->drawDiverOnBestFeaturesMapAtCurrentZoom(latitude,longitude,0.0);

      delay(diveTraceTrackStepPause);
    }
    else if (refreshMap)
    {
      mapScreen->drawDiverOnBestFeaturesMapAtCurrentZoom(latitude,longitude,0.0);
    }
    else
    {
      delay (50);
    }
  }
}

void cycleTrackIndex()
{
  delay(diveTraceTrackStepPause);
  trackIndex = (trackIndex + 1) % trackLength;
}

void placePinTest()
{
  const double depth = 0.0;

  if (diveTrackTest)
  {
    mapScreen->placePin(diveTrack[trackIndex]._la,diveTrack[trackIndex]._lo,diveTrack[trackIndex]._h, depth);
  }

  if (diveTraceTest)
  {
    mapScreen->placePin(latitude,longitude,heading, depth);
  }
}

#endif
