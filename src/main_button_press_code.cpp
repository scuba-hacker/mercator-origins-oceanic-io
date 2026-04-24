#ifdef BUILD_INCLUDE_MAIN_BUTTON_PRESS_CODE

bool checkForDualButtonPresses()
{
  static bool action1000msReached = false;
  static bool action3000msReached = false;
  static bool action8000msReached = false;
  static bool anyActionTriggered = false;
  static uint32_t lastActionTime = 0;
  
  bool triggered = false;
  
  // Prevent any action within 10 seconds of the last action
  if (millis() - lastActionTime < 10000)
  {
    return false;
  }
  
  // Check if both buttons are currently pressed
  bool bothPressed = p_primaryButton->isPressed() && p_secondButton->isPressed();
  
  if (!bothPressed)
  {
    // Buttons released - check which action to trigger based on how long they were held
    if (!anyActionTriggered && (action1000msReached || action3000msReached || action8000msReached))
    {
      anyActionTriggered = true;  // Set this first to prevent re-entry
      lastActionTime = millis();  // Record time of action execution
      
      if (action8000msReached)
      {
        // 8 second hold completed - reboot
        // esp_restart();
        triggered = true;
      }
      else if (action3000msReached)
      {
        // 3 second hold completed - toggle diagnostic screens
        // skipDiagnosticDisplays = !skipDiagnosticDisplays;
        // saveToEEPROMSkipDiagnosticDisplays();  // Save to EEPROM
        // M5.Lcd.setCursor(5, M5.Lcd.height() - 90);
        // M5.Lcd.printf("Skip Mode: %s", skipDiagnosticDisplays ? "ON" : "OFF");
        compositeSprite->fillSprite(TFT_GREENYELLOW);
        resetCompositeSpriteCursor();
        compositeSprite->print("3s Both Buttons");
        mapScreen->copyCompositeSpriteToDisplay();
        delay(800);
        triggered = true;
      }
      else if (action1000msReached)
      {
        delay(800);
        updateButtons();
        forceDeepSleep(); // does not continue from here
        triggered = true;
      }
    }
    
    // Only reset flags if both buttons are completely released
    if (p_primaryButton->isReleased() && p_secondButton->isReleased())
    {
      action1000msReached = false;
      action3000msReached = false;
      action8000msReached = false;
      anyActionTriggered = false;
    }
    
    return triggered;
  }
  
  // Buttons are pressed - track which thresholds have been reached
  // Only set the highest threshold reached to ensure mutual exclusivity
  if (p_primaryButton->pressedFor(8000) && p_secondButton->pressedFor(8000))
  {
    action8000msReached = true;
    action3000msReached = false;  // Clear lower thresholds
    action1000msReached = false;
  }
  else if (p_primaryButton->pressedFor(3000) && p_secondButton->pressedFor(3000))
  {
    action3000msReached = true;
    action1000msReached = false;   // Clear lower threshold
  }
  else if (p_primaryButton->pressedFor(500) && p_secondButton->pressedFor(500))
  {
    action1000msReached = true;
  }

  return false; // No action triggered while buttons are still pressed
}


bool checkGoProButtons()
{
  bool changeMade = false;

  bool buttonTop;
  uint32_t activationTime=0;
    
  updateButtons();

/*
  // press primary button for 10 second to clear breadcrumbtrail
  if (p_primaryButton->wasReleasefor(10000))
  {
    activationTime = lastPrimaryButtonPressLasted;
    buttonTop = false;
    changeMade = true;

    mapScreen->clearBreadCrumbTrail();
  }
  // press primary button for 2 second to toggle draw all features
  else if (p_primaryButton->wasReleasefor(2000))
  {
    activationTime = lastPrimaryButtonPressLasted;
    buttonTop = true;
    changeMade = true;

    mapScreen->toggleDrawAllFeatures();
    mapScreen->drawDiverOnBestFeaturesMapAtCurrentZoom(latitude, longitude, heading);
  }
  // press primary button for 0.5 second to toggle show bread crumb trail
  else if (p_primaryButton->wasReleasefor(500))
  {
    activationTime = lastPrimaryButtonPressLasted;
    buttonTop = true;
    changeMade = true;

    mapScreen->toggleShowBreadCrumbTrail();
    mapScreen->drawDiverOnBestFeaturesMapAtCurrentZoom(latitude, longitude, heading);
  }
  */
/*
  // press both buttons for 1 second for deep sleep, side button to wake-up
  if (p_primaryButton->pressedFor(1000) && 
      p_secondButton->pressedFor(1000))
  {
    forceDeepSleep();
  }
*/

  if (checkForDualButtonPresses())
    return true;
  
  // short press primary button cycle zoom if not at startup, otherwise activate OTA.
  if (p_primaryButton->wasReleasefor(30))
  {
    if (!setupComplete) // null before recovery ota screen done at startup
    {
      activationTime = lastPrimaryButtonPressLasted;
      buttonTop = true;
      changeMade = true;

      const bool clearScreen = true;
      switchToPersistentOTAMode(clearScreen);
    }
    else
    {
      activationTime = lastPrimaryButtonPressLasted;
      buttonTop = true;

      mapScreen->cycleZoom(); changeMade = true;
      USB_SERIAL_PRINTLN("########################   CYCLE ZOOM %i #####################################",mapScreen->getZoom());
      uint32_t start = micros();
      mapScreen->drawDiverOnBestFeaturesMapAtCurrentZoom(latitude, longitude, heading);
      USB_SERIAL_PRINTLN("########################   Draw Diver %i ms ##################################", (micros() - start) / 1000);
    }
  }

  // press second button for 10 seconds to restart
  // press second button for 5 seconds to attempt WiFi connect and enable OTA
  // press second button for 1 seconds for map legend.
  // press second button for 100ms  to start/stop breadcrumb trail
  // press second button for 100ms at boot recovery screen to put ESP32 into deep sleep for charging
  if (p_secondButton->wasReleasefor(10000))
  { 
    compositeSprite->fillSprite(TFT_RED);
    resetCompositeSpriteCursor();
    compositeSprite->setTextColor(TFT_WHITE,TFT_RED);
    compositeSprite->println("Restart By Button Press");
    mapScreen->copyCompositeSpriteToDisplay();
    delay(10000);
    esp_restart();
  }
  else if (p_secondButton->wasReleasefor(5000))
  { 
    activationTime = lastSecondButtonPressLasted;
    buttonTop = false;

    const bool clearScreen = false;
    switchToPersistentOTAMode(clearScreen);
    changeMade = true;
  }
  // Display Map Legend
  else if (setupComplete && p_secondButton->wasReleasefor(1000))
  {
    activationTime = lastSecondButtonPressLasted;
    buttonTop = false;
    changeMade = true;

    mapScreen->displayMapLegend();    // two second blocking display of legend screen -
    mapScreen->drawDiverOnBestFeaturesMapAtCurrentZoom(latitude, longitude, heading);
  }
  // Toggle Breadcrumb Trail
  else if (p_secondButton->wasReleasefor(30))
  {
    if (!setupComplete) // null before recovery ota screen done at startup
    {
      delay(1000);
      updateButtons();
      forceDeepSleep();
      // never goes beyond here
    }
    else
    {
      activationTime = lastSecondButtonPressLasted;
      buttonTop = false;
      changeMade = true;
      mapScreen->toggleRecordBreadCrumbTrail();
    }
  }

  if (BootButton.isPressed())
  {
      activationTime = lastPrimaryButtonPressLasted;
      buttonTop = true;
      changeMade = true;
      switchToPersistentOTAMode(true);
  }

  /*
  if (activationTime > 0)
  {
    USB_SERIAL_PRINTLN("Reed Activated...");

    publishToMakoReedActivation(reedSwitchTop, activationTime);
  }*/
  
  return changeMade;
}


void clearButtons()
{
  p_primaryButton->read();
  p_secondButton->read();
  BootButton.read();
}

void updateButtons()
{
  p_primaryButton->read();
  p_secondButton->read();
  BootButton.read();

  if (p_primaryButton->isPressed())
  {
    if (!primaryButtonIsPressed)
    {
      primaryButtonIsPressed=true;
      primaryButtonPressedTime=millis();
    }
  }
  else
  {
    if (primaryButtonIsPressed)
    {
      lastPrimaryButtonPressLasted = millis() - primaryButtonPressedTime;
      primaryButtonIsPressed=false;
      primaryButtonPressedTime=0;
    }
  }

  if (p_secondButton->isPressed())
  {
    if (!secondButtonIsPressed)
    {
      secondButtonIsPressed=true;
      secondButtonPressedTime=millis();
    }
  }
  else
  {
    if (secondButtonIsPressed)
    {
      lastSecondButtonPressLasted = millis() - secondButtonPressedTime;
      secondButtonIsPressed=false;
      secondButtonPressedTime=0;
    }
  }
}

void readAndTestGoProSwitches()
{
  updateButtons();

  bool btnTopPressed = p_primaryButton->pressedFor(15);
  bool btnSidePressed = p_secondButton->pressedFor(15);

  if (btnTopPressed && btnSidePressed)
  {
    sideCount++;
    topCount++;
    compositeSprite->setCursor(5, 5);
    compositeSprite->printf("TOP+SIDE %d %d", topCount, sideCount);
  }
  else if (btnTopPressed)
  {
    topCount++;
    compositeSprite->setCursor(5, 5);
    compositeSprite->printf("TOP %d", topCount);
  }
  else if (btnSidePressed)
  {
    sideCount++;
    compositeSprite->setCursor(5, 5);
    compositeSprite->printf("SIDE %d", sideCount);
  }
}

#endif
