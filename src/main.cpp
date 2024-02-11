#include <Arduino.h>
//#include <Wire.h>

#include <MapScreen_T4.h>
#include <LilyGo_AMOLED.h>
#include <TFT_eSPI.h>

#include <esp_now.h>
#include <WiFi.h>
#include <freertos/queue.h>
#include <memory>
#include <time.h>
#include <queue>



#include <fonts/NotoSansBold36.h>
//#include <fonts/Final_Frontier_28.h>
//#include <fonts/NotoSansMonoSCB20.h>

// rename the git file "mercator_secrets_template.c" to the filename below, filling in your wifi credentials etc.
#include "mercator_secrets.c"

#include "Button.h"

bool writeLogToSerial=true;
bool testPreCannedLatLong=false;
bool diveTrackTest = false;
bool diveTraceTest = false;
bool enableOTATimer=false;
uint32_t otaTimerExpired = 60000;

const bool enableOTAServerAtStartup=false;
const bool enableESPNow = !enableOTAServerAtStartup;

#include "dive_track.h"
extern const location diveTrack[];
extern "C" int getSizeOfDiveTrack();

extern const char track_and_trace_html_content[];


int trackIndex=0;
int trackLength=getSizeOfDiveTrack();
void cycleTrackIndex();

// TODO: create Oceanic banner
//#define MERCATOR_ELEGANTOTA_TIGER_BANNER
#define MERCATOR_OTA_DEVICE_LABEL "OCEANIC-IO"

#include <Update.h>             // OTA updates
#include <AsyncTCP.h>           // OTA updates
#include <ESPAsyncWebServer.h>  // OTA updates
#include <MercatorElegantOta.h>    // OTA updates

MercatorElegantOtaClass MercatorElegantOta;



const String ssid_not_connected = "-";
String ssid_connected;

// ************** ESPNow variables **************

uint16_t ESPNowMessagesDelivered = 0;
uint16_t ESPNowMessagesFailedToDeliver = 0;

const uint8_t ESPNOW_CHANNEL=1;
const uint8_t ESPNOW_NO_PEER_CHANNEL_FLAG = 0xFF;
const uint8_t ESPNOW_PRINTSCANRESULTS = 0;
const uint8_t ESPNOW_DELETEBEFOREPAIR = 0;

esp_now_peer_info_t ESPNow_mako_peer;
bool isPairedWithMako = false;

const int RESET_ESPNOW_SEND_RESULT = 0xFF;
esp_err_t ESPNowSendResult=(esp_err_t)RESET_ESPNOW_SEND_RESULT;

char mako_espnow_buffer[256];               // MBJ REFACTOR  
char currentTime[9];               // MBJ REFACTOR  

QueueHandle_t msgsESPNowReceivedQueue=nullptr;

bool ESPNowActive = false;

const uint8_t BUTTON_REED_TOP_PIN=38;             // UPDATE FOR T4 - rename REED
const uint8_t BUTTON_REED_SIDE_PIN=48;             // UPDATE FOR T4 - rename REED
const uint8_t BUTTON_BOOT=0;
const uint32_t MERCATOR_DEBOUNCE_MS=10;    

Button ReedSwitchGoProTop = Button(BUTTON_REED_TOP_PIN, true, MERCATOR_DEBOUNCE_MS);    // from utility/Button.h for M5 Stick C Plus
Button ReedSwitchGoProSide = Button(BUTTON_REED_SIDE_PIN, true, MERCATOR_DEBOUNCE_MS); // from utility/Button.h for M5 Stick C Plus
Button  BootButton =  Button(BUTTON_BOOT, true, MERCATOR_DEBOUNCE_MS);                  // from utility/Button.h for M5 Stick C Plus

uint16_t sideCount = 0, topCount = 0;

const uint16_t mode_label_y_offset = 170;

AsyncWebServer asyncWebServer(80);      // OTA updates
bool otaActive=false;   // OTA updates toggle

Button* p_primaryButton = NULL;
Button* p_secondButton = NULL;

bool primaryButtonIsPressed = false;
uint32_t primaryButtonPressedTime = 0;
uint32_t lastPrimaryButtonPressLasted = 0;

bool secondButtonIsPressed = false;
uint32_t secondButtonPressedTime = 0;
uint32_t lastSecondButtonPressLasted = 0;

bool primaryButtonIndicatorNeedsClearing = false;
bool secondButtonIndicatorNeedsClearing = false;

TFT_eSPI tft = TFT_eSPI();
LilyGo_AMOLED amoled;
std::unique_ptr<MapScreen_T4> mapScreen;

TFT_eSprite* compositeSprite = nullptr;

const double startLatitude = 51.4605855;    // lightning boat
const double startLongitude=-0.548316;

double latitude  = startLatitude;    // lightning boat
double longitude = startLongitude;
double heading=0.0;

double latitudeDelta = 0.0;
double longitudeDelta = 0.0;

int otaScreenBackColour = TFT_GREEN;
int otaScreenForeColour = TFT_BLUE;
int espScanBackColour = TFT_PURPLE;
int espScanForeColour = TFT_WHITE;
int wifiScanBackColour = TFT_CYAN;
int wifiScanForeColour = TFT_BLUE;

#define USB_SERIAL Serial

const int defaultBrightness = 255;

char rxQueueESPNowItemBuffer[256];
const uint8_t queueESPNowLength=10;

std::queue<std::string> httpQueue;

char currentTarget[128];
char previousTarget[128];
bool refreshTargetShown = false;

bool checkReedSwitches();
void publishToMakoTestMessage(const char* testMessage);
void publishToMakoReedActivation(const bool topReed, const uint32_t ms);

void resetCurrentTarget();

void resetMap();
void resetClock();

const char* scanForKnownNetwork();
bool setupOTAWebServer(const char* _ssid, const char* _password, const char* label, uint32_t timeout, bool wifiOnly = false);
void toggleOTAActiveAndWifiIfUSBPowerOff();
void updateButtonsAndBuzzer();
void readAndTestGoProReedSwitches();
void InitESPNow();
void configAndStartUpESPNow();
void configESPNowDeviceAP();
void OnESPNowDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);
void OnESPNowDataRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len);
bool ESPNowScanForPeer(esp_now_peer_info_t& peer, const char* peerSSIDPrefix, const bool suppressPeerFoundMsg = true);
bool pairWithMako();
bool pairWithPeer(esp_now_peer_info_t& peer, const char* peerSSIDPrefix, int maxAttempts);
bool connectToWiFiAndInitOTA(const bool wifiOnly, int repeatScanAttempts);
bool ESPNowManagePeer(esp_now_peer_info_t& peer);
void ESPNowDeletePeer(esp_now_peer_info_t& peer);
bool TeardownESPNow();

void dumpHeapUsage(const char* msg)
{  
  if (writeLogToSerial)
  {
    multi_heap_info_t info;
    heap_caps_get_info(&info, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT); // internal RAM, memory capable to store data or to create new task
    USB_SERIAL.printf("\n%s : free heap bytes: %i  largest free heap block: %i min free ever: %i\n",  msg, info.total_free_bytes, info.largest_free_block, info.minimum_free_bytes);
  }
}

void testMapDisplay();
bool disableESPNowandEnableOTA();
void switchToPersistentOTAMode(bool clearScreen);

void resetCompositeSpriteCursor()
{
  compositeSprite->setCursor(0,30);
}

bool checkButtons()
{
  bool result=false;

  BootButton.read();
  ReedSwitchGoProTop.read();
  ReedSwitchGoProSide.read();
/*
  if (ReedSwitchGoProSide.isPressed() && ReedSwitchGoProTop.isPressed())
  {
    compositeSprite->fillSprite(TFT_RED);
    mapScreen->copyCompositeSpriteToDisplay();
    delay(3000);
  }
*/
  if (BootButton.wasReleasefor(100) || ReedSwitchGoProTop.wasReleasefor(3000)) // switch to OTA mode
  {    
    amoled.setBrightness(50);
    switchToPersistentOTAMode(true);    // never returns - loops waiting for OTA
    result = true;
  }

  if (ReedSwitchGoProSide.wasReleasefor(3000))
  {
    ESP.restart();
  }
  
  return result;
}

void recoveryScreen()
{
  if (compositeSprite)
  {
    compositeSprite->fillSprite(TFT_DARKGREEN);
    resetCompositeSpriteCursor();
    compositeSprite->println("Press Boot Button\nfor Recovery OTA");
    mapScreen->copyCompositeSpriteToDisplay();
  }

  const uint32_t end = millis() + 5000;
  while (end > millis())
  {
    checkReedSwitches();
    delay(20);
  }
}

void setup()
{
  char c = track_and_trace_html_content[0];

  p_primaryButton = &ReedSwitchGoProTop;
  p_secondButton = &ReedSwitchGoProSide;

  dumpHeapUsage("Setup(): at startup ");
  amoled.begin();
  dumpHeapUsage("Setup(): after amoled.begin() ");

  if (writeLogToSerial)
  {
    USB_SERIAL.begin(115200);
    Serial.flush();
    delay(50);
  }
  dumpHeapUsage("Setup(): after USB serial port started ");

  amoled.setBrightness(defaultBrightness);
  mapScreen = std::make_unique<MapScreen_T4>(tft,amoled);

  compositeSprite = &mapScreen->getCompositeSprite();
  compositeSprite->loadFont(NotoSansBold36);     // use smooth font    -D SMOOTH_FONT=1

  recoveryScreen();

  msgsESPNowReceivedQueue = xQueueCreate(queueESPNowLength,sizeof(rxQueueESPNowItemBuffer));

  if (enableOTAServerAtStartup)
  {
    compositeSprite->fillSprite(TFT_DARKGREY);
    resetCompositeSpriteCursor();
    compositeSprite->println("Start OTA\n\n");
    delay(1000);
    const bool wifiOnly = false;
    const int maxWifiScanAttempts = 3;
    connectToWiFiAndInitOTA(wifiOnly,maxWifiScanAttempts);
  }

  if (!otaActive && enableESPNow && msgsESPNowReceivedQueue)
  {
    configAndStartUpESPNow();
    compositeSprite->fillSprite(espScanBackColour);
    resetCompositeSpriteCursor();
    compositeSprite->setTextColor(espScanForeColour, espScanBackColour);
    compositeSprite->println("ESP Now Initialised\nAwait Mako ESP Now Message");
    mapScreen->copyCompositeSpriteToDisplay();
    // defer pairing with mako for sending messages to mako until first message received from mako.
  }

  dumpHeapUsage("Setup(): end ");
}

bool pairWithMako()
{
  if (ESPNowActive && !isPairedWithMako)
  {
    compositeSprite->fillSprite(espScanBackColour);
    compositeSprite->setTextColor(espScanForeColour,espScanBackColour);
    resetCompositeSpriteCursor();
    mapScreen->copyCompositeSpriteToDisplay();

    const int pairAttempts = 5;
    isPairedWithMako = pairWithPeer(ESPNow_mako_peer,"Mako",pairAttempts); // 5 connection attempts

    if (isPairedWithMako)
    {
      // send message to tiger to give first target
      publishToMakoTestMessage("Conn Ok");
      delay(1000);
    }
  }

  return isPairedWithMako;
}

void readAndTestGoProReedSwitches()
{
  updateButtonsAndBuzzer();

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

void resetMap()
{
  mapScreen->drawDiverOnBestFeaturesMapAtCurrentZoom(latitude, longitude, heading);
}

bool disableESPNowandEnableOTA()
{
  TeardownESPNow();
  isPairedWithMako = false;

  // enable OTA
  const bool wifiOnly = false;
  compositeSprite->fillSprite(otaScreenBackColour);
  resetCompositeSpriteCursor();
  compositeSprite->println("Start\n  OTA\n\n");
  delay(1000);
  const int maxWifiScanAttempts = 3;
  connectToWiFiAndInitOTA(wifiOnly,maxWifiScanAttempts);

  return true;
}

bool checkReedSwitches()
{
  bool changeMade = false;

  bool reedSwitchTop;
  uint32_t activationTime=0;
    
  updateButtonsAndBuzzer();
/*
  int pressedPrimaryButtonX, pressedPrimaryButtonY, pressedSecondButtonX, pressedSecondButtonY;

  pressedPrimaryButtonX = 110;
  pressedPrimaryButtonY = 5; 

  pressedSecondButtonX = 5;
  pressedSecondButtonY = 210;
    
  if (primaryButtonIsPressed && millis()-primaryButtonPressedTime > 250)
  {
    compositeSprite->setTextSize(3);
    compositeSprite->setTextColor(TFT_WHITE, TFT_RED);
    compositeSprite->setCursor(pressedPrimaryButtonX,pressedPrimaryButtonY);
    compositeSprite->printf("%i",(millis()-primaryButtonPressedTime)/1000);
    compositeSprite->setTextColor(TFT_WHITE, TFT_BLACK);
    primaryButtonIndicatorNeedsClearing=true;
  }
  else
  {
    if (primaryButtonIndicatorNeedsClearing)
    {
      primaryButtonIndicatorNeedsClearing = false;
      compositeSprite->setTextSize(3);
      compositeSprite->setTextColor(TFT_WHITE, TFT_BLACK);
      compositeSprite->setCursor(pressedPrimaryButtonX,pressedPrimaryButtonY);
      compositeSprite->print(" ");
    }
  }

  if (secondButtonIsPressed && millis()-secondButtonPressedTime > 250)
  {
    compositeSprite->setTextSize(3);
    compositeSprite->setTextColor(TFT_WHITE, TFT_BLUE);
    compositeSprite->setCursor(pressedSecondButtonX,pressedSecondButtonY);
    compositeSprite->printf("%i",(millis()-secondButtonPressedTime)/1000);
    compositeSprite->setTextColor(TFT_WHITE, TFT_BLACK);
    secondButtonIndicatorNeedsClearing=true;
  }
  else
  {
    if (secondButtonIndicatorNeedsClearing)
    {
      secondButtonIndicatorNeedsClearing = false;
      
      compositeSprite->setTextSize(3);
      compositeSprite->setTextColor(TFT_WHITE, TFT_BLACK);
      compositeSprite->setCursor(pressedSecondButtonX,pressedSecondButtonY);
      compositeSprite->print(" ");
    }
  }
*/

  if (p_primaryButton->wasReleasefor(100) && msgsESPNowReceivedQueue == nullptr) // null before recovery ota screen done at startup
  {
    activationTime = lastPrimaryButtonPressLasted;
    reedSwitchTop = true;
    changeMade = true;
    switchToPersistentOTAMode(true);
  }

  // press second button for 10 seconds to restart
  // press second button for 5 seconds to attempt WiFi connect and enable OTA
  if (p_secondButton->wasReleasefor(5000))
  { 
     esp_restart();
  }
  else if (p_secondButton->wasReleasefor(2000))
  { 
      activationTime = lastSecondButtonPressLasted;
      reedSwitchTop = false;

      switchToPersistentOTAMode(false);
      changeMade = true;
//      const bool refreshCurrentScreen=true;
  //    cycleDisplays(refreshCurrentScreen);
  }
  // press second button for 1 second...
  else if (p_secondButton->wasReleasefor(500))
  {
    activationTime = lastSecondButtonPressLasted;
    reedSwitchTop = false;

    mapScreen->toggleDrawAllFeatures();
    mapScreen->drawDiverOnBestFeaturesMapAtCurrentZoom(latitude, longitude, heading);
    changeMade = true;
  }
  // press second button for 0.1 second...
  else if (p_secondButton->wasReleasefor(100))
  {
    activationTime = lastSecondButtonPressLasted;
    reedSwitchTop = false;

    mapScreen->cycleZoom(); changeMade = true;
    mapScreen->drawDiverOnBestFeaturesMapAtCurrentZoom(latitude, longitude, heading);
  }
  /*
  if (activationTime > 0)
  {
    if (writeLogToSerial)
      USB_SERIAL.println("Reed Activated...");

    publishToMakoReedActivation(reedSwitchTop, activationTime);
  }*/
  
  return changeMade;
}

void publishToMakoTestMessage(const char* testMessage)
{
  if (isPairedWithMako && ESPNow_mako_peer.channel == ESPNOW_CHANNEL)
  {
    snprintf(mako_espnow_buffer,sizeof(mako_espnow_buffer),"T%s",testMessage);
    if (writeLogToSerial)
    {
      USB_SERIAL.println("Sending ESP T msg to Mako...");
      USB_SERIAL.println(mako_espnow_buffer);
    }

    ESPNowSendResult = esp_now_send(ESPNow_mako_peer.peer_addr, reinterpret_cast<uint8_t*>(mako_espnow_buffer), strlen(mako_espnow_buffer)+1);
  }
}

void publishToMakoReedActivation(const bool topReed, const uint32_t ms)
{
  if (isPairedWithMako && ESPNow_mako_peer.channel == ESPNOW_CHANNEL)
  {
    snprintf(mako_espnow_buffer,sizeof(mako_espnow_buffer),"R%c%lu       ",(topReed ? 'T' : 'B'),ms);
    if (writeLogToSerial)
    {
      USB_SERIAL.println("Sending ESP R msg to Mako...");
      USB_SERIAL.println(mako_espnow_buffer);
    }
    ESPNowSendResult = esp_now_send(ESPNow_mako_peer.peer_addr, reinterpret_cast<uint8_t*>(mako_espnow_buffer), strlen(mako_espnow_buffer)+1);
  }
  else
  {
    if (writeLogToSerial)
      USB_SERIAL.println("ESPNow inactive - not sending ESP R msg to Mako...");
  }
}

uint32_t nextBattUpdateTime = 0;
uint32_t battUpdateCadence = 10000;

bool isMakoMessage(char m)
{
  return (m == 'X' || m == 'c');
}

void loop()
{ 
  // do not process queued messages if ota is active, mapscreen is deleted and espnow will be shutdown anyway.
  if (msgsESPNowReceivedQueue && !otaActive)
  {
    if (xQueueReceive(msgsESPNowReceivedQueue,&(rxQueueESPNowItemBuffer),(TickType_t)0))
    {
      char messageType = rxQueueESPNowItemBuffer[0];
      if (isMakoMessage(messageType) && !isPairedWithMako) // only pair with Mako once first message received from Mako.
      {
        pairWithMako();
      }

      switch (messageType)
      {
        case 'X':   // location, heading and current Target info.
        {
          // format: targetCode[7],lat,long,heading,targetText
          const int targetCodeOffset = 1;
          const int latitudeOffset = 8;
          const int longitudeOffset = 16;
          const int headingOffset = 24;
          const int currentTargetOffset = 32;
                    
          char targetCode[7];

          double old_latitude = latitude;
          double old_longitude = longitude;
          double old_heading = heading;

          strncpy(targetCode,rxQueueESPNowItemBuffer + targetCodeOffset,sizeof(targetCode));
          memcpy(&latitude,  rxQueueESPNowItemBuffer + latitudeOffset,  sizeof(double));
          memcpy(&longitude, rxQueueESPNowItemBuffer + longitudeOffset, sizeof(double));
          memcpy(&heading,   rxQueueESPNowItemBuffer + headingOffset, sizeof(double));

          if (strcmp(rxQueueESPNowItemBuffer+currentTargetOffset,currentTarget) != 0)
          {
            strncpy(previousTarget,currentTarget,sizeof(previousTarget));
            strncpy(currentTarget,rxQueueESPNowItemBuffer+currentTargetOffset,sizeof(currentTarget));
            refreshTargetShown = true;
          }

          if (writeLogToSerial)
          {
            USB_SERIAL.printf("targetCode: %s\n",targetCode);
            USB_SERIAL.printf("latitude: %f\n",latitude);
            USB_SERIAL.printf("longitude: %f\n",longitude);
            USB_SERIAL.printf("heading: %f\n",heading);
            USB_SERIAL.printf("currentTarget: %s\n",currentTarget);
          }

          mapScreen->setTargetWaypointByLabel(targetCode);

          if (testPreCannedLatLong)
          {
            latitude = old_latitude;
            longitude = old_longitude+0.00001;
            heading = static_cast<int>((old_heading + 5)) % 360;
          }

          mapScreen->drawDiverOnBestFeaturesMapAtCurrentZoom(latitude, longitude, heading);
        }

        case 'c':   // current target
        {
          if (strcmp(rxQueueESPNowItemBuffer+1,currentTarget) != 0)
          {
            strncpy(previousTarget,currentTarget,sizeof(previousTarget));
            strncpy(currentTarget,rxQueueESPNowItemBuffer+1,sizeof(currentTarget));
            refreshTargetShown = true;
          }
          break;
        }

        default:
        {
          break;
        }
      }
    }

    if (!isPairedWithMako && millis() > nextBattUpdateTime)
    {
      nextBattUpdateTime += battUpdateCadence;

      compositeSprite->fillSprite(espScanBackColour);
      resetCompositeSpriteCursor();
      compositeSprite->setTextColor(espScanForeColour,espScanBackColour);
      compositeSprite->println("ESP Now Initialised\nAwait Mako ESP Now Message");
      compositeSprite->printf("vbatt: %.2f\n", static_cast<float>(amoled.getBattVoltage())/1000.0);
      mapScreen->copyCompositeSpriteToDisplay();
    }
  }

  // for dev without buttons attached to device.
  if (enableOTATimer && otaTimerExpired < millis())
  {
    USB_SERIAL.println("disable ESP Now, enable OTA");
    switchToPersistentOTAMode(true);
  }

  checkReedSwitches();

  bool refreshMap = false;

  if (!httpQueue.empty())
  {
    std::string str = httpQueue.back();
    httpQueue.pop();
    if (str == std::string("track") || str == std::string("trackButton"))
    {
      // disable espnow
      diveTrackTest = true;
      diveTraceTest = false;
      latitudeDelta = 0;
      longitudeDelta = 0;
    }
    else if (str == std::string("trace") || str == std::string("traceButton"))
    {
      latitude=startLatitude;
      longitude=startLongitude;
      diveTrackTest = false;
      diveTraceTest = true;
      latitudeDelta = 0;
      longitudeDelta = 0;
      refreshMap = true;
    }
    else if (str == std::string("u") || str == std::string("upButton"))
    {
      latitudeDelta=0.00002;
      longitudeDelta=0;
      diveTraceTest = true;
    }
    else if (str == std::string("d") || str == std::string("downButton"))
    {
      latitudeDelta=-0.00002;
      longitudeDelta=0;
      diveTrackTest = false;
      diveTraceTest = true;
    }
    else if (str == std::string("l") || str == std::string("leftButton"))
    {
      latitudeDelta=0;
      longitudeDelta=-0.00002;
      diveTrackTest = false;
      diveTraceTest = true;
    }
    else if (str == std::string("r") || str == std::string("rightButton"))
    {
      latitudeDelta=0;
      longitudeDelta=0.00002;
      diveTrackTest = false;
      diveTraceTest = true;
    }
    else if (str == std::string("reset") || str == std::string("resetButton"))
    {
      latitude=startLatitude;
      longitude=startLongitude;
      diveTrackTest = false;
      refreshMap = true;
    }
    else if (str == std::string("stop") || str == std::string("stopButton"))
    {
      latitudeDelta=0;
      longitudeDelta=0;
      diveTrackTest = false;
    }
    else if (str == std::string("allButton"))
    {
      mapScreen->setAllLakeShown(true);
      refreshMap = true;
    }
    else if (str == std::string("x1Button"))
    {
      mapScreen->setZoom(1);   // 
      refreshMap = true;
    }
    else if (str == std::string("x2Button"))
    {
      mapScreen->setZoom(2);   // 
      refreshMap = true;
    }
    else if (str == std::string("x3Button"))
    {
      mapScreen->setZoom(3);   // 
      refreshMap = true;
    }
    else if (str == std::string("x4Button"))
    {
      mapScreen->setZoom(4);   // 
      refreshMap = true;
    }
    else if (str == std::string("dim"))
    {
      amoled.setBrightness(10);
    }
    else if (str == std::string("bright"))
    {
      amoled.setBrightness(255);
    }
  }

  if (diveTrackTest)
  {
    mapScreen->drawDiverOnBestFeaturesMapAtCurrentZoom(diveTrack[trackIndex]._la,diveTrack[trackIndex]._lo,diveTrack[trackIndex]._h);
    cycleTrackIndex();
  }

  if (diveTraceTest)
  {
    if (latitudeDelta != 0 || longitudeDelta != 0)
    {
      latitude+=latitudeDelta;
      longitude+=longitudeDelta;
      mapScreen->drawDiverOnBestFeaturesMapAtCurrentZoom(latitude,longitude,0.0);

      delay(50);
//      uint32_t now = millis();
//      while (millis() < now + 100)
//        yield();
    }
    else if (refreshMap)
    {
      mapScreen->drawDiverOnBestFeaturesMapAtCurrentZoom(latitude,longitude,0.0);
    }
    else
    {
      delay (50);
    }
//       yield();

  }
}

void cycleTrackIndex()
{
  trackIndex = (trackIndex + 1) % trackLength;
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

    uint32_t waitPeriod = 5000;
    uint32_t end = millis()+waitPeriod;
    String status, prevStatus;
    int line=0;
    while (end < millis())
    {
      String status = amoled.getChargeStatusString();

      if (status != prevStatus)
      {
        uint16_t vbus = amoled.getVbusVoltage();
        uint16_t vbatt = amoled.getBattVoltage();
        uint16_t vsys = amoled.getSystemVoltage();

        compositeSprite->printf("%s\n\nvbus:  %.3f\nvbatt: %.3f\nvsys:   %.3f\n", status.c_str(),vbus/1000.0,vbatt/1000.0,vsys/1000.0);
        mapScreen->copyCompositeSpriteToDisplay();
        if (line++ == 7)
        {
          line = 0;
          compositeSprite->fillSprite(otaScreenBackColour);
          compositeSprite->setCursor(0,30);
        }
        prevStatus = status;
        delay(500);
      }
      checkReedSwitches();
    }
}

void testMapDisplay()
{
  double latitude = 51.4605855;    // lightning boat
  double longitude = -0.54890166666666; 
  double heading = 0.0;
  mapScreen->drawDiverOnBestFeaturesMapAtCurrentZoom(latitude,longitude,heading);
}

void updateButtonsAndBuzzer()
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

void configAndStartUpESPNow()
{  
  //Set device in AP mode to begin with
  WiFi.mode(WIFI_AP);
  
  // configure device AP mode
  configESPNowDeviceAP();
  
  // This is the mac address of this peer in AP Mode
  if (writeLogToSerial)
  {
    USB_SERIAL.print("AP MAC: "); 
    USB_SERIAL.println(WiFi.softAPmacAddress());
  }
  // Init ESPNow with a fallback logic
  InitESPNow();
  
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info.
  esp_now_register_send_cb(OnESPNowDataSent);
  esp_now_register_recv_cb(OnESPNowDataRecv);
}

void configESPNowDeviceAP()
{
  String Prefix = "Oceanic:";
  String Mac = WiFi.macAddress();
  String SSID = Prefix + Mac;
  String Password = "123456789";
  bool result = WiFi.softAP(SSID.c_str(), Password.c_str(), ESPNOW_CHANNEL, 0);

  if (writeLogToSerial)
  {
    if (!result)
    {
      USB_SERIAL.println("AP Config failed.");
    }
    else
    {
      USB_SERIAL.printf("AP Config Success. Broadcasting with AP: %s\n",String(SSID).c_str());
      USB_SERIAL.printf("WiFi Channel: %d\n",WiFi.channel());
    }
  }
}

void InitESPNow()
{
  WiFi.disconnect();
  if (esp_now_init() == ESP_OK)
  {
    if (writeLogToSerial)
      USB_SERIAL.println("ESPNow Init Success");
    ESPNowActive = true;
  }
  else
  {
    if (writeLogToSerial)
      USB_SERIAL.println("ESPNow Init Failed");
    ESPNowActive = false;
  }
}

// callback when data is sent from Master to Peer
void OnESPNowDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
  if (status == ESP_NOW_SEND_SUCCESS)
  {
    ESPNowMessagesDelivered++;
  }
  else
  {
    ESPNowMessagesFailedToDeliver++;
  }
}

// callback when data is recv from Master
void OnESPNowDataRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len)
{
  if (writeLogToSerial)
  {
    char macStr[18];
    snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
             mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
    USB_SERIAL.printf("Last Packet Recv from: %s\n",macStr);
    USB_SERIAL.printf("Last Packet Recv 1st Byte: '%c'\n",*data);
    USB_SERIAL.printf("Last Packet Recv Length: %d\n",data_len);
  }

  xQueueSend(msgsESPNowReceivedQueue, data, (TickType_t)0);  // don't block on enqueue, just drop if queue is full
}

bool TeardownESPNow()
{
  bool result = false;

  if (enableESPNow && ESPNowActive)
  {
    WiFi.disconnect();
    ESPNowActive = false;
    result = true;
  }
  
  return result;
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

    if (writeLogToSerial)
      USB_SERIAL.printf("Found:\n%s\n",network);
  }
  else
  {
    compositeSprite->println("None\nFound");
    if (writeLogToSerial)
      USB_SERIAL.println("No networks Found\n");
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
    if (writeLogToSerial)
      USB_SERIAL.printf("setupOTAWebServer: attempt to connect wifiOnly, already connected - otaActive=%i\n",otaActive);

    return true;
  }

  if (writeLogToSerial)
    USB_SERIAL.printf("setupOTAWebServer: attempt to connect %s wifiOnly=%i when otaActive=%i\n",_ssid, wifiOnly,otaActive);

  bool forcedCancellation = false;

  // mapscreen does not get deleted with T4 - sufficient core RAM
  bool connected = false;
  WiFi.mode(WIFI_STA);

  WiFi.begin(_ssid, _password);

  // Wait for connection for max of timeout/1000 seconds
  compositeSprite->printf("%s try connect...\n", label);
  mapScreen->copyCompositeSpriteToDisplay();

  int count = timeout / 500;
  while (WiFi.status() != WL_CONNECTED && --count > 0)
  {
    // check for cancellation button - top button.

/*
    updateButtonsAndBuzzer();
    if (p_primaryButton->isPressed()) // cancel connection attempts
    {
      forcedCancellation = true;
      break;
    }
*/
    compositeSprite->print(".");
    mapScreen->copyCompositeSpriteToDisplay();
    delay(500);
    checkReedSwitches();
  }
  compositeSprite->print("\n");

  delay(100);

  if (WiFi.status() == WL_CONNECTED )
  {
    if (wifiOnly == false && !otaActive)
    {
      dumpHeapUsage("setupOTAWebServer(): after WiFi connect");

      if (writeLogToSerial)
        USB_SERIAL.println("setupOTAWebServer: WiFi connected ok, starting up OTA");

      if (writeLogToSerial)
        USB_SERIAL.println("setupOTAWebServer: calling asyncWebServer.on");

      asyncWebServer.on("/", HTTP_GET, [](AsyncWebServerRequest * request) {
        request->send(200, "text/plain", "To upload firmware use /update");
      });
        
      if (writeLogToSerial)
        USB_SERIAL.println("setupOTAWebServer: calling MercatorElegantOta.begin");

      MercatorElegantOta.setID(MERCATOR_OTA_DEVICE_LABEL);
      MercatorElegantOta.begin(&httpQueue, &asyncWebServer);    // Start MercatorElegantOta

      if (writeLogToSerial)
        USB_SERIAL.println("setupOTAWebServer: calling asyncWebServer.begin");

      asyncWebServer.begin();

      dumpHeapUsage("setupOTAWebServer(): after asyncWebServer.begin");

      if (writeLogToSerial)
        USB_SERIAL.println("setupOTAWebServer: OTA setup complete");

      compositeSprite->printf("%s\n",WiFi.localIP().toString());
      compositeSprite->println(WiFi.macAddress());
      mapScreen->copyCompositeSpriteToDisplay();

      connected = true;
      otaActive = true;
    
      delay(1000);

      connected = true;
  
      /*
      updateButtonsAndBuzzer();
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
      if (writeLogToSerial)
        USB_SERIAL.printf("setupOTAWebServer: WiFi failed to connect %s\n",_ssid);

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
      delay(1000);
      continue;
    }
  
    int connectToFoundNetworkAttempts = 3;
    const int repeatDelay = 1000;
      
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
    
    checkReedSwitches();
    delay(1000);
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

// Scan for peers in AP mode
bool ESPNowScanForPeer(esp_now_peer_info_t& peer, const char* peerSSIDPrefix, const bool suppressPeerFoundMsg)
{
  bool peerFound = false;
  
  compositeSprite->printf("Scan For %s\n",peerSSIDPrefix);
  mapScreen->copyCompositeSpriteToDisplay();
  
  int8_t scanResults = WiFi.scanNetworks();
  
  // reset on each scan 
  memset(&peer, 0, sizeof(peer));

  if (writeLogToSerial)
    USB_SERIAL.println("");

  if (scanResults == 0) 
  {   
    if (writeLogToSerial)
      USB_SERIAL.println("No WiFi devices in AP Mode found");

    peer.channel = ESPNOW_NO_PEER_CHANNEL_FLAG;
  } 
  else 
  {
    if (writeLogToSerial)
    {
      USB_SERIAL.print("Found "); USB_SERIAL.print(scanResults); USB_SERIAL.println(" devices ");
    }
    
    for (int i = 0; i < scanResults; ++i) 
    {
      // Print SSID and RSSI for each device found
      String SSID = WiFi.SSID(i);
      int32_t RSSI = WiFi.RSSI(i);
      String BSSIDstr = WiFi.BSSIDstr(i);

      if (writeLogToSerial && ESPNOW_PRINTSCANRESULTS) 
      {
        USB_SERIAL.print(i + 1);
        USB_SERIAL.print(": ");
        USB_SERIAL.print(SSID);
        USB_SERIAL.print(" (");
        USB_SERIAL.print(RSSI);
        USB_SERIAL.print(")");
        USB_SERIAL.println("");
      }
      
      delay(10);
      
      // Check if the current device starts with the peerSSIDPrefix
      if (SSID.indexOf(peerSSIDPrefix) == 0) 
      {
        if (writeLogToSerial)
        {
          // SSID of interest
          USB_SERIAL.println("Found a peer.");
          USB_SERIAL.print(i + 1); USB_SERIAL.print(": "); USB_SERIAL.print(SSID); USB_SERIAL.print(" ["); USB_SERIAL.print(BSSIDstr); USB_SERIAL.print("]"); USB_SERIAL.print(" ("); USB_SERIAL.print(RSSI); USB_SERIAL.print(")"); USB_SERIAL.println("");
        }
                
        // Get BSSID => Mac Address of the Slave
        const int macLength = 6;
        std::array <int, macLength> mac;

        if ( macLength == sscanf(BSSIDstr.c_str(), "%x:%x:%x:%x:%x:%x",  &mac[0], &mac[1], &mac[2], &mac[3], &mac[4], &mac[5] ) ) 
        {
          for (int ii = 0; ii < mac.size(); ++ii ) 
            peer.peer_addr[ii] = static_cast<uint8_t>(mac[ii]);
        }

        peer.channel = ESPNOW_CHANNEL; // pick a channel
        peer.encrypt = 0; // no encryption

        peer.priv = reinterpret_cast<void*>(const_cast<char*>(peerSSIDPrefix));   // distinguish between different peers

        peerFound = true;
        // we are planning to have only one slave in this example;
        // Hence, break after we find one, to be a bit efficient
        break;
      }
    }
  }

  if (!suppressPeerFoundMsg)
  {
    if (peerFound)
    {
      compositeSprite->println("Peer Found");
      if (writeLogToSerial)
        USB_SERIAL.println("Peer Found, processing..");
    } 
    else 
    {
      compositeSprite->println("Peer Not Found");
      if (writeLogToSerial)
        USB_SERIAL.println("Peer Not Found, trying again.");
    }
  }
  mapScreen->copyCompositeSpriteToDisplay();
    
  // clean up ram
  WiFi.scanDelete();

  return peerFound;
}

bool pairWithPeer(esp_now_peer_info_t& peer, const char* peerSSIDPrefix, int maxAttempts)
{
  bool isPaired = false;
  while(maxAttempts-- && !isPaired)
  {
    bool result = ESPNowScanForPeer(peer,peerSSIDPrefix);

    // check if peer channel is defined
    if (result && peer.channel == ESPNOW_CHANNEL)
    { 
      isPaired = ESPNowManagePeer(peer);
      compositeSprite->setTextColor(TFT_GREEN,espScanBackColour);
      compositeSprite->printf("%s Pair ok\n",peerSSIDPrefix);
      compositeSprite->setTextColor(espScanForeColour,espScanBackColour);
    }
    else
    {
      peer.channel = ESPNOW_NO_PEER_CHANNEL_FLAG;
      compositeSprite->setTextColor(TFT_RED,espScanBackColour);
      compositeSprite->printf("%s Pair fail\n",peerSSIDPrefix);
      compositeSprite->setTextColor(espScanForeColour,espScanBackColour);
    }
    mapScreen->copyCompositeSpriteToDisplay();
    checkReedSwitches();  
  }

  delay(1000);
  
  return isPaired;
}

// Check if the peer is already paired with the master.
// If not, pair the peer with master
bool ESPNowManagePeer(esp_now_peer_info_t& peer)
{
  bool result = false;

  if (peer.channel == ESPNOW_CHANNEL)
  {
    if (ESPNOW_DELETEBEFOREPAIR)
    {
      ESPNowDeletePeer(peer);
    }

    if (writeLogToSerial)
      USB_SERIAL.print("Peer Status: ");

    // check if the peer exists
    bool exists = esp_now_is_peer_exist(peer.peer_addr);

    if (exists)
    {
      // Peer already paired.
      if (writeLogToSerial)
        USB_SERIAL.println("Already Paired");

      compositeSprite->println("Already paired");
      result = true;
    }
    else
    {
      // Peer not paired, attempt pair
      esp_err_t addStatus = esp_now_add_peer(&peer);

      if (addStatus == ESP_OK)
      {
        // Pair success
        if (writeLogToSerial)
          USB_SERIAL.println("Pair success");
        compositeSprite->println("Pair success");
        result = true;
      }
      else if (addStatus == ESP_ERR_ESPNOW_NOT_INIT)
      {
        // How did we get so far!!
        if (writeLogToSerial)
          USB_SERIAL.println("ESPNOW Not Init");
        result = false;
      }
      else if (addStatus == ESP_ERR_ESPNOW_ARG)
      {
        if (writeLogToSerial)
            USB_SERIAL.println("Invalid Argument");
        result = false;
      }
      else if (addStatus == ESP_ERR_ESPNOW_FULL)
      {
        if (writeLogToSerial)
            USB_SERIAL.println("Peer list full");
        result = false;
      }
      else if (addStatus == ESP_ERR_ESPNOW_NO_MEM)
      {
        if (writeLogToSerial)
          USB_SERIAL.println("Out of memory");
        result = false;
      }
      else if (addStatus == ESP_ERR_ESPNOW_EXIST)
      {
        if (writeLogToSerial)
          USB_SERIAL.println("Peer Exists");
        result = true;
      }
      else
      {
        if (writeLogToSerial)
          USB_SERIAL.println("Not sure what happened");
        result = false;
      }
    }
  }
  else
  {
    // No peer found to process
    if (writeLogToSerial)
      USB_SERIAL.println("No Peer found to process");

    compositeSprite->println("No Peer found to process");
    result = false;
  }
  mapScreen->copyCompositeSpriteToDisplay();

  return result;
}

void ESPNowDeletePeer(esp_now_peer_info_t& peer)
{
  if (peer.channel != ESPNOW_NO_PEER_CHANNEL_FLAG)
  {
    esp_err_t delStatus = esp_now_del_peer(peer.peer_addr);

    if (writeLogToSerial)
    {
      USB_SERIAL.print("Peer Delete Status: ");
      if (delStatus == ESP_OK)
      {
        // Delete success
        USB_SERIAL.println("ESPNowDeletePeer::Success");
      }
      else if (delStatus == ESP_ERR_ESPNOW_NOT_INIT)
      {
        // How did we get so far!!
        USB_SERIAL.println("ESPNowDeletePeer::ESPNOW Not Init");
      }
      else if (delStatus == ESP_ERR_ESPNOW_ARG)
      {
        USB_SERIAL.println("ESPNowDeletePeer::Invalid Argument");
      }
      else if (delStatus == ESP_ERR_ESPNOW_NOT_FOUND)
      {
        USB_SERIAL.println("ESPNowDeletePeer::Peer not found.");
      }
      else
      {
        USB_SERIAL.println("Not sure what happened");
      }
    }
  }
}
