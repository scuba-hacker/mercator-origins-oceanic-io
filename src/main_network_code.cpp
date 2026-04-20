#ifdef BUILD_INCLUDE_NETWORK_CODE

bool disableESPNowandEnableOTA()
{
  TeardownESPNow();
  isPairedWithMako = false;

  // enable OTA
  const bool wifiOnly = false;
  compositeSprite->fillSprite(otaScreenBackColour);
  resetCompositeSpriteCursor();
  compositeSprite->println("Start\n  OTA\n\n");
  delay(250);
  const int maxWifiScanAttempts = 3;
  connectToWiFiAndInitOTA(wifiOnly,maxWifiScanAttempts);

  return true;
}


void switchToPersistentOTAMode(bool clearScreen)
{
    disableESPNowandEnableOTA();
    if (clearScreen)
    {
      compositeSprite->fillSprite(wifiScanBackColour);
      resetCompositeSpriteCursor();
      compositeSprite->setTextColor(wifiScanForeColour, wifiScanBackColour);
    }
    compositeSprite->println("Ready for OTA update\n");
    mapScreen->copyCompositeSpriteToDisplay();

    const bool showPowerStatus = false;

    if (showPowerStatus)
    {
      uint32_t waitPeriod = 5000;
      uint32_t end = millis()+waitPeriod;
      String status, prevStatus;
      int line=0;
      while (end > millis())
      {
          uint16_t vbus = amoled.getVbusVoltage();
          uint16_t vbatt = amoled.getBattVoltage();
          uint16_t vsys = amoled.getSystemVoltage();

          compositeSprite->printf("vbus %.3f vbatt %.3fvsys %.3f\n", vbus/1000.0,vbatt/1000.0,vsys/1000.0);
          mapScreen->copyCompositeSpriteToDisplay();
          if (line++ == 7)
          {
            line = 0;
            compositeSprite->fillSprite(wifiScanBackColour);
            resetCompositeSpriteCursor();
            compositeSprite->setTextColor(wifiScanForeColour, wifiScanBackColour);
          }
          delay(2000);

        checkGoProButtons();
      }
    }
}

const char* scanForKnownNetwork() // return first known network found
{
  const char* network = nullptr;

  compositeSprite->println("Scan WiFi SSIDs...");
  mapScreen->copyCompositeSpriteToDisplay();

  int8_t scanResults = WiFi.scanNetworks();

  if (scanResults != 0)
  {
    for (int i = 0; i < scanResults; ++i) 
    {
      String SSID = WiFi.SSID(i);
      
      // Check if the current device starts with the peerSSIDPrefix
      if (strcmp(SSID.c_str(), ssid_1) == 0)
        network=ssid_1;
      else if (strcmp(SSID.c_str(), ssid_2) == 0)
        network=ssid_2;
      else if (strcmp(SSID.c_str(), ssid_3) == 0)
        network=ssid_3;

      if (network)
        break;
    }    
  }

  if (network)
  {
      compositeSprite->printf("Found: %s\n",network);

    USB_SERIAL_PRINTF("Found:\n%s\n",network);
  }
  else
  {
    compositeSprite->println("None\nFound");
    USB_SERIAL_PRINTLN("No networks Found\n");
  }
  mapScreen->copyCompositeSpriteToDisplay();

  // clean up ram
  WiFi.scanDelete();

  return network;
}

bool setupOTAWebServer(const char* _ssid, const char* _password, const char* label, uint32_t timeout, bool wifiOnly)
{
  if (wifiOnly && WiFi.status() == WL_CONNECTED)
  {
    USB_SERIAL_PRINTF("setupOTAWebServer: attempt to connect wifiOnly, already connected - otaActive=%i\n",otaActive);
    return true;
  }

  USB_SERIAL_PRINTF("setupOTAWebServer: attempt to connect %s wifiOnly=%i when otaActive=%i\n",_ssid, wifiOnly,otaActive);

  bool forcedCancellation = false;

  // mapscreen does not get deleted with T4 - sufficient core RAM
  bool connected = false;
  WiFi.mode(WIFI_STA);
  WiFi.config(INADDR_NONE, INADDR_NONE, INADDR_NONE, INADDR_NONE);
  WiFi.setHostname("oceanic");

  WiFi.begin(_ssid, _password);

  // Wait for connection for max of timeout/1000 seconds
  compositeSprite->printf("%s try connect...\n", label);
  mapScreen->copyCompositeSpriteToDisplay();

  int count = timeout / 500;
  while (WiFi.status() != WL_CONNECTED && --count > 0)
  {
    // check for cancellation button - top button.

/*
    updateButtons();
    if (p_primaryButton->isPressed()) // cancel connection attempts
    {
      forcedCancellation = true;
      break;
    }
*/
    compositeSprite->print(".");
    mapScreen->copyCompositeSpriteToDisplay();
    delay(500);
    checkGoProButtons();
  }
  compositeSprite->print("\n");

  delay(100);

  if (WiFi.status() == WL_CONNECTED )
  {
    if (wifiOnly == false && !otaActive)
    {
      dumpHeapUsage("setupOTAWebServer(): after WiFi connect");

      USB_SERIAL_PRINTLN("setupOTAWebServer: WiFi connected ok, starting up OTA");

      USB_SERIAL_PRINTLN("setupOTAWebServer: calling asyncWebServer.on");

      asyncWebServer.on("/", HTTP_GET, [](AsyncWebServerRequest * request) {
        request->send(200, "text/plain", "To upload firmware use /update");
      });
        
      USB_SERIAL_PRINTLN("setupOTAWebServer: calling MercatorElegantOta.begin");

      MercatorElegantOta.setID(MERCATOR_OTA_DEVICE_LABEL);
      MercatorElegantOta.begin(&httpQueue, &asyncWebServer);    // Start MercatorElegantOta


      #ifdef USE_WEBSERIAL
      static bool webSerialInitialised = false;

      if (!webSerialInitialised)
      {
        WebSerial.begin(&asyncWebServer);
        WebSerial.msgCallback(webSerialReceiveMessage);
        webSerialInitialised = true;
      }
      #endif

      USB_SERIAL_PRINTLN("setupOTAWebServer: calling asyncWebServer.begin");

      asyncWebServer.begin();

      dumpHeapUsage("setupOTAWebServer(): after asyncWebServer.begin");

      USB_SERIAL_PRINTLN("setupOTAWebServer: OTA setup complete");


      compositeSprite->printf("%s\n",WiFi.localIP().toString());
      compositeSprite->printf("%s\n",WiFi.macAddress().c_str());
      mapScreen->copyCompositeSpriteToDisplay();

      connected = true;
      otaActive = true;
      
      setupFTPServer();

      delay(1000);

      connected = true;

      /*
      updateButtons();
      if (p_secondButton->isPressed())
      {
        compositeSprite->print("\n\n20\nsecond pause");
        mapScreen->copyCompositeSpriteToDisplay();
        delay(20000);
      }*/
    }
  }
  else
  {
    compositeSprite->fillSprite(TFT_RED);
    mapScreen->copyCompositeSpriteToDisplay();
    if (forcedCancellation)
      compositeSprite->print("\nCancelled\nConnect\nAttempts");
    else
    {
      USB_SERIAL_PRINTF("setupOTAWebServer: WiFi failed to connect %s\n",_ssid);

      compositeSprite->fillSprite(TFT_RED);
      resetCompositeSpriteCursor();
      compositeSprite->print("No Connect\n");
    }
    mapScreen->copyCompositeSpriteToDisplay();
    delay(2000);
  }

  dumpHeapUsage("setupOTAWebServer(): end of function");

  return connected;
}

bool connectToWiFiAndInitOTA(const bool wifiOnly, int repeatScanAttempts)
{
  if (wifiOnly && WiFi.status() == WL_CONNECTED)
    return true;

  compositeSprite->setCursor(0, 30);
  compositeSprite->fillSprite(wifiScanBackColour);
  compositeSprite->setTextSize(2);
  compositeSprite->setTextColor(wifiScanForeColour,wifiScanBackColour);

  while (repeatScanAttempts-- &&
         (WiFi.status() != WL_CONNECTED ||
          WiFi.status() == WL_CONNECTED && wifiOnly == false && otaActive == false ) )
  {
    const char* network = scanForKnownNetwork();

    if (!network)
    {
      delay(500);
      continue;
    }
  
    int connectToFoundNetworkAttempts = 3;
    const int repeatDelay = 500;
      
    if (strcmp(network,ssid_1) == 0)
    {
      while (connectToFoundNetworkAttempts-- && !setupOTAWebServer(ssid_1, password_1, label_1, timeout_1, wifiOnly))
        delay(repeatDelay);
    }
    else if (strcmp(network,ssid_2) == 0)
    {
      while (connectToFoundNetworkAttempts-- && !setupOTAWebServer(ssid_2, password_2, label_2, timeout_2, wifiOnly))
        delay(repeatDelay);
    }
    else if (strcmp(network,ssid_3) == 0)
    {
      while (connectToFoundNetworkAttempts-- && !setupOTAWebServer(ssid_3, password_3, label_3, timeout_3, wifiOnly))
        delay(repeatDelay);
    }
    
    checkGoProButtons();
    delay(500);
  }

  bool connected=WiFi.status() == WL_CONNECTED;
  
  if (connected)
  {
    ssid_connected = WiFi.SSID();
  }
  else
  {
    ssid_connected = ssid_not_connected;
  }
  
  return connected;
}

int8_t scanWiFiForSSIDs()
{
  return WiFi.scanNetworks(false,false,false,150U);
}

#endif
