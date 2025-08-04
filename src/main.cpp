#include <Arduino.h>
//#include <Wire.h>
#include <esp_sleep.h>
#include "driver/rtc_io.h"

#include <MapScreen_T4.h>
#include <LilyGo_AMOLED.h>
#include <TFT_eSPI.h>

#include <Adafruit_AHTX0.h>
#ifdef COMPILE_TOF
#include <vl53l4cx_class.h>
#endif

/*
Documentation needed:

Button Controls

  // press both buttons for 1 second for deep sleep, side button to wake-up
  // tap   top button cycle zoom if not at startup, otherwise activate OTA.

  // press side button for 10 seconds to restart
  // press side button for 5 seconds to attempt WiFi connect and enable OTA
  // press side button for 1 seconds for map legend.
  // press side button for 100ms  to start/stop breadcrumb trail
  // press side button for 100ms at boot recovery screen to put ESP32 into deep sleep for charging

Serial Commands

serial-off
serial-on


HTTP Endpoints

/boot      reboot
/track     run track test - diver follows recorded route
/trace     run trace test - user controls diver
/up        diver North
/do        diver South
/le        diver West
/ri        diver East
/stop      stop running track/trace
/reset     back to start of track/trace test
/dim       Dim   brightness
/night     Night brightness
/bright    Max Brightness
/sleep     Deep Sleep for battery charging
/test      no function - logging only

Additional web page button controls

Key         Button Message
-           Slower
+/=         Faster
r           reset
s           stop
k           track
c           trace
UP          upButton
DOWN        downButton
LEFT        leftButton
RIGHT       rightButton
u           updateButton
0           allButton
1           x1Button
2           x2Button
3           x3Button
4           x4Button
            startSerialButton
            stopSerialButton

ESPNow Commands 

BY        - recording breadcrumb trail
BN        - not recording breadcrumb trail
T<string> - test message
R<T|B><t> - Button activation (T)op and (B)ottom, followed by milliseconds

Additional to add - add pin placed.

*/

// 0 means not set (was previously set to A1 == 2)
#define XSHUT_PIN 0

TwoWire *DEV_I2C = &Wire;

#ifdef COMPILE_TOF
VL53L4CX sensor_vl53l4cx_sat;
#endif

#include <esp_now.h>
#include <WiFi.h>
#include <freertos/queue.h>
#include <memory>
#include <time.h>
#include <queue>

#include <fonts/NotoSansBold36.h>
//#include <fonts/Final_Frontier_28.h>
#include <fonts/NotoSansMonoSCB20.h>

// rename the git file "mercator_secrets_template.c" to the filename below, filling in your wifi credentials etc.
#include "mercator_secrets.c"

//#define USE_WEBSERIAL


#ifdef USE_WEBSERIAL
#include <WebSerial.h>
#endif

#include "Button.h"

bool writeLogToSerial=true;
bool enableToFSensor=false;
bool doInitialSerialTransmitTest=false;
bool doInitialSerialReceiveEchoTest=false;
bool testPreCannedLatLong=false;
bool diveTrackTest = false;
bool diveTraceTest = false;
uint32_t diveTraceTrackStepPause = 100;
const uint32_t diveTraceTrackStepIncrement = 50;
bool enableOTATimer=false;
uint32_t otaTimerExpired = 60000;

const bool correctForReversedCompassTrackTest = true;

const bool enableOTAServerAtStartup=false;
const bool enableESPNow = !enableOTAServerAtStartup;

uint8_t newLidarDataReady = 0;

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
#include <MercatorElegantOTA.h>    // OTA updates

MercatorElegantOtaClass MercatorElegantOta;

Adafruit_AHTX0 ahtSensor;

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

// Free GPIOs: 38

const uint8_t BUTTON_TOP_GPIO=48;
const uint8_t BUTTON_SIDE_GPIO=21;
const uint8_t BUTTON_BOOT=0;
const uint32_t MERCATOR_DEBOUNCE_MS=10;    

const uint8_t LEAK_SENSOR_GPIO=47; // not connected

const int BEAGLE_UART_RX_GPIO = 44;
const int BEAGLE_UART_TX_GPIO = 43;
const int BEAGLE_BAUD = 115200;

Button SwitchGoProTop = Button(BUTTON_TOP_GPIO, true, MERCATOR_DEBOUNCE_MS);    // from utility/Button.h for M5 Stick C Plus
Button SwitchGoProSide = Button(BUTTON_SIDE_GPIO, true, MERCATOR_DEBOUNCE_MS); // from utility/Button.h for M5 Stick C Plus
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

#ifdef USE_WEBSERIAL
  #define USB_SERIAL WebSerial
#else
  #define USB_SERIAL Serial
#endif

const int offBrightness = 0;
const int dayBrightness = 255;
const int nightBrightness = 100;
const int dimBrightness = 10;

uint16_t currentBrightnessReceived = 0;
int currentBrightnessSet = 0;

const int dayBrightnessThreshold = 10;
const int nightBrightnessThreshold = 8;

char rxQueueESPNowItemBuffer[256];
const uint8_t queueESPNowLength=20;

std::queue<std::string> httpQueue;

char currentTarget[256];
char previousTarget[256];
bool refreshTargetShown = false;

bool checkGoProButtons();
void publishToMakoBreadCrumbRecord(const bool record);
void publishToMakoTestMessage(const char* testMessage);
void publishToMakoReedActivation(const bool topReed, const uint32_t ms);
void placePinTest();

void resetCurrentTarget();

void resetMap();
void resetClock();

const char* scanForKnownNetwork();
bool setupOTAWebServer(const char* _ssid, const char* _password, const char* label, uint32_t timeout, bool wifiOnly = false);
void toggleOTAActiveAndWifiIfUSBPowerOff();
void updateButtons();
void readAndTestGoProSwitches();
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
void forceDeepSleep();

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

#ifdef COMPILE_TOF

/* VL53L4CX_RangeStatusCode --------------------------------------------------*/
String VL53L4CX_RangeStatusCode(uint8_t status)
{
  switch (status)
  {
    case VL53L4CX_RANGESTATUS_RANGE_VALID:
      return "VL53L4CX_RANGESTATUS_RANGE_VALID";
    case VL53L4CX_RANGESTATUS_SIGMA_FAIL:
      return "VL53L4CX_RANGESTATUS_SIGMA_FAIL";
    case VL53L4CX_RANGESTATUS_RANGE_VALID_MIN_RANGE_CLIPPED:
      return "VL53L4CX_RANGESTATUS_RANGE_VALID_MIN_RANGE_CLIPPED";
    case VL53L4CX_RANGESTATUS_OUTOFBOUNDS_FAIL:
      return "VL53L4CX_RANGESTATUS_OUTOFBOUNDS_FAIL";
    case VL53L4CX_RANGESTATUS_HARDWARE_FAIL:
      return "VL53L4CX_RANGESTATUS_HARDWARE_FAIL";
    case VL53L4CX_RANGESTATUS_RANGE_VALID_NO_WRAP_CHECK_FAIL:
      return "VL53L4CX_RANGESTATUS_RANGE_VALID_NO_WRAP_CHECK_FAIL";
    case VL53L4CX_RANGESTATUS_WRAP_TARGET_FAIL:
      return "VL53L4CX_RANGESTATUS_WRAP_TARGET_FAIL";
    case VL53L4CX_RANGESTATUS_PROCESSING_FAIL:
      return "VL53L4CX_RANGESTATUS_PROCESSING_FAIL";
    case VL53L4CX_RANGESTATUS_XTALK_SIGNAL_FAIL:
      return "VL53L4CX_RANGESTATUS_XTALK_SIGNAL_FAIL";
    case VL53L4CX_RANGESTATUS_SYNCRONISATION_INT:
      return "VL53L4CX_RANGESTATUS_SYNCRONISATION_INT";
    case VL53L4CX_RANGESTATUS_RANGE_VALID_MERGED_PULSE:
      return "VL53L4CX_RANGESTATUS_RANGE_VALID_MERGED_PULSE";
    case VL53L4CX_RANGESTATUS_TARGET_PRESENT_LACK_OF_SIGNAL:
      return "VL53L4CX_RANGESTATUS_TARGET_PRESENT_LACK_OF_SIGNAL";
    case VL53L4CX_RANGESTATUS_MIN_RANGE_FAIL:
      return "VL53L4CX_RANGESTATUS_MIN_RANGE_FAIL";
    case VL53L4CX_RANGESTATUS_RANGE_INVALID:
      return "VL53L4CX_RANGESTATUS_RANGE_INVALID";
    case VL53L4CX_RANGESTATUS_NONE:
      return "VL53L4CX_RANGESTATUS_NONE";
    default:
      return ("UNKNOWN STATUS: " + String(status));
  }
}
#endif

/* I2CDeviceAvailable --------------------------------------------------------*/
bool I2CDeviceAvailable(uint8_t address, TwoWire **wire)
{
  byte error = 1;
  bool available = false;

  // Check if device is available at the expected address
  Wire.begin();
  Wire.beginTransmission(address);
  error = Wire.endTransmission();

  if (error == 0) {
    *wire = &Wire;
    available = true;

    Serial.print("  I2c Device Found at Address 0x");
    Serial.print(address, HEX); Serial.println(" on Wire");
  }
  Wire.end();

  return available;
}

void resetCompositeSpriteCursor()
{
  compositeSprite->setCursor(0,30);
}

bool ahtStatus=false;   // Adafruit AHT20
bool tofStatus=false;   // Adafruit VL53L4CX  https://github.com/stm32duino/VL53L4CX

void startupScreen()
{
  if (compositeSprite)
  {
    compositeSprite->fillSprite(TFT_YELLOW);
    resetCompositeSpriteCursor();
    compositeSprite->printf("Initialising Sensors...");
    mapScreen->copyCompositeSpriteToDisplay();
  }
}

void recoveryScreen()
{
  if (compositeSprite)
  {
    compositeSprite->fillSprite(TFT_DARKGREEN);
    resetCompositeSpriteCursor();
    compositeSprite->printf("Press Boot Button\nfor Recovery OTA\n");

    PowersSY6970& power = static_cast<PowersSY6970&>(amoled);

    power.setChargerConstantCurr(1024); // was 1024
    power.disableCurrentLimitPin();
    power.setAutoChargerTypeDetectionEnabled(false);
    power.setCurrentInputLimit(1500);  // was 1500

    uint16_t vBatt = power.getBattVoltage();
    uint16_t constcurr = power.getChargerConstantCurr();
    uint16_t tarvolts = power.getChargeTargetVoltage();
    int limited = power.isEnableCurrentLimitPin();
    uint16_t inputLimitI2C = power.getCurrentInputLimit();
    int autoDetect = power.getAutoChargeDetectionEnabled();
    
    compositeSprite->printf("vBatt = %hu mV\n",vBatt);
    compositeSprite->printf("Const Curr = %hu mA\n",constcurr);
    compositeSprite->printf("Tar Volts = %hu mV\n",tarvolts);
    compositeSprite->printf("Limited Pin = %i\n",limited);
    compositeSprite->printf("Input Limit = %hu\n",inputLimitI2C);
    compositeSprite->printf("AutoDetect = %i\n",autoDetect);
    compositeSprite->printf("Charge = %s", power.getChargeStatusString());

    mapScreen->copyCompositeSpriteToDisplay();
  }

  const uint32_t end = millis() + 10000;
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

void testReceiveFromReefToOceanic()
{
  Serial1.println("test\n");

  char i[2] = "a";
  int j=10;
  while(Serial1.available() && j--)
  {
    i[0] = Serial1.read();
    compositeSprite->printf(i);
    mapScreen->copyCompositeSpriteToDisplay();
  }

  compositeSprite->println("done reading serial1");
  mapScreen->copyCompositeSpriteToDisplay();
  delay(3000);
}

void onOTAUpdateStart(AsyncElegantOtaClass* elegantOTA)
{
  // Stop all trace/track activity so that OTA proceeds at full speed.
  httpQueue.push("stop");

  // disable all sensor readings too
  ahtStatus = false;
  tofStatus = false;

  //ws.closeAll();          // close all websocket connections for test page
#ifdef USE_WEBSERIAL
  WebSerial.closeAll();   // close all websocket connetions for WebSerial
#endif
}

void initI2C()
{
  const int gpioSDA = 6;
  const int gpioSCL = 7;

  DEV_I2C->setPins(gpioSDA, gpioSCL);
}

void initHumiditySensor()
{
  ahtStatus = ahtSensor.begin(DEV_I2C);
}

#ifdef COMPILE_TOF

void initToFSensor()
{

  VL53L4CX_Error error = VL53L4CX_ERROR_NONE;

  if (I2CDeviceAvailable(VL53L4CX_DEFAULT_DEVICE_ADDRESS >> 1, &DEV_I2C)) 
  {
    DEV_I2C->begin();
    sensor_vl53l4cx_sat.setI2cDevice(DEV_I2C);
    sensor_vl53l4cx_sat.setXShutPin(XSHUT_PIN);

    // Configure VL53L4CX satellite component.
    sensor_vl53l4cx_sat.begin();
    
    // Switch off VL53L4CX satellite component.
    sensor_vl53l4cx_sat.VL53L4CX_Off();

    //Initialize VL53L4CX satellite component.
    error = sensor_vl53l4cx_sat.InitSensor(VL53L4CX_DEFAULT_DEVICE_ADDRESS);

    if (error != VL53L4CX_ERROR_NONE)
    {
      if (writeLogToSerial)
      {
        USB_SERIAL.print("Error Initializing Sensor: ");
        USB_SERIAL.println(error);
      }
    }
    else
    {
      error = sensor_vl53l4cx_sat.VL53L4CX_SetDistanceMode(VL53L4CX_DISTANCEMODE_LONG);

      if (error != VL53L4CX_ERROR_NONE)
      {
        USB_SERIAL.print("Error Initializing Distance Mode: ");
        USB_SERIAL.println(error);
      }
      else
      {
        // Start Measurements
        sensor_vl53l4cx_sat.VL53L4CX_StartMeasurement();

        tofStatus = true;
      }
    }
  }
}
#endif

void setScreenBrightness(uint16_t brightness)
{
  if (currentBrightnessSet != brightness)
  {
    amoled.setBrightness(brightness);
    currentBrightnessSet = brightness;
  }
}

void setup()
{
 if (writeLogToSerial)
  {
    #ifndef USE_WEBSERIAL
      USB_SERIAL.begin(115200);
      Serial.flush();
      delay(50);
    #endif
  }
  
  dumpHeapUsage("Setup(): at startup ");
  amoled.begin();
  setScreenBrightness(dayBrightness);
  mapScreen = std::make_unique<MapScreen_T4>(tft,amoled);

  if (writeLogToSerial)
    USB_SERIAL.println("created MapScreen_T4");

  mapScreen->registerBreadCrumbRecordActionCallback(&publishToMakoBreadCrumbRecord);

  compositeSprite = &mapScreen->getCompositeSprite();
  compositeSprite->loadFont(NotoSansBold36);     // use smooth font    -D SMOOTH_FONT=1

  startupScreen();

  dumpHeapUsage("Setup(): after amoled.begin() ");

 

  Serial1.begin(BEAGLE_BAUD,SERIAL_8N1,BEAGLE_UART_RX_GPIO,BEAGLE_UART_TX_GPIO);

  dumpHeapUsage("Setup(): after USB serial port started ");

  MercatorElegantOta.setUploadBeginCallback(onOTAUpdateStart);

  initI2C();
  initHumiditySensor();

#ifdef COMPILE_TOF

  if (enableToFSensor)
    initToFSensor();
#endif

  p_primaryButton = &SwitchGoProTop;
  p_secondButton = &SwitchGoProSide;

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
      publishToMakoTestMessage("Conn Ok");
      mapScreen->clearBreadCrumbTrail();    // will turn off any recording and notify mako that recording is off.
      delay(500);
    }
  }

  return isPairedWithMako;
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
  delay(250);
  const int maxWifiScanAttempts = 3;
  connectToWiFiAndInitOTA(wifiOnly,maxWifiScanAttempts);

  return true;
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
  // press both buttons for 1 second for deep sleep, side button to wake-up
  if (p_primaryButton->pressedFor(1000) && 
      p_secondButton->pressedFor(1000))
  {
    forceDeepSleep();
  }

  // short press primary button cycle zoom if not at startup, otherwise activate OTA.
  if (p_primaryButton->wasReleasefor(100))
  {
    if (msgsESPNowReceivedQueue == nullptr) // null before recovery ota screen done at startup
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
      mapScreen->drawDiverOnBestFeaturesMapAtCurrentZoom(latitude, longitude, heading);
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
  else if (p_secondButton->wasReleasefor(1000))
  {
    activationTime = lastSecondButtonPressLasted;
    buttonTop = false;
    changeMade = true;

    mapScreen->displayMapLegend();
    mapScreen->drawDiverOnBestFeaturesMapAtCurrentZoom(latitude, longitude, heading);
  }
  // Toggle Breadcrumb Trail
  else if (p_secondButton->wasReleasefor(100))
  {
    if (msgsESPNowReceivedQueue == nullptr) // null before recovery ota screen done at startup
    {
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
    if (writeLogToSerial)
      USB_SERIAL.println("Reed Activated...");

    publishToMakoReedActivation(reedSwitchTop, activationTime);
  }*/
  
  return changeMade;
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

void publishToMakoBreadCrumbRecord(const bool record)
{
  if (isPairedWithMako && ESPNow_mako_peer.channel == ESPNOW_CHANNEL)
  {
    snprintf(mako_espnow_buffer,sizeof(mako_espnow_buffer),"B%c",(record ? 'Y' : 'N'));
    if (writeLogToSerial)
    {
      USB_SERIAL.println("Sending ESP B msg to Mako...");
      USB_SERIAL.println(mako_espnow_buffer);
    }

    ESPNowSendResult = esp_now_send(ESPNow_mako_peer.peer_addr, reinterpret_cast<uint8_t*>(mako_espnow_buffer), strlen(mako_espnow_buffer)+1);
  }
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

uint32_t nextReadHumiditySensorTime = 0;
const uint32_t timeBetweenHumidityReads = 5000;
uint32_t nextToFSensorTime = 0;
const uint32_t timeBetweenToFReads = 100;

void acquireHumidityAndTemperatureReadings()
{
    float relative_humidity = -1.0;
    float temperature = -1.0;

    if (ahtStatus)
    {
      sensors_event_t humidity, temp;
      ahtSensor.getEvent(&humidity, &temp);
      relative_humidity = humidity.relative_humidity;
      temperature = temp.temperature;
    }

    mapScreen->setHumidityAndTemp(relative_humidity,temperature);
}

#ifdef COMPILE_TOF
void acquireLidarDistanceReading()
{
  if (!tofStatus)
  {
    mapScreen->setLidarDistance(-1.0);
    return;
  }

  float maxDistance = 0.0;

  VL53L4CX_Error status = sensor_vl53l4cx_sat.VL53L4CX_GetMeasurementDataReady(&newLidarDataReady);

  if (writeLogToSerial)
    USB_SERIAL.printf("GetMeasurementDataReady Status: %s\n",VL53L4CX_RangeStatusCode(status).c_str());

  if (!newLidarDataReady)
  {
    if (writeLogToSerial)
      USB_SERIAL.printf("Data Not Ready, returning\n");
    return;
  }
  else
  {
    if (writeLogToSerial)
      USB_SERIAL.printf("Data Ready, extract objects\n");
  }

  VL53L4CX_MultiRangingData_t MultiRangingData;
  VL53L4CX_MultiRangingData_t *pMultiRangingData = &MultiRangingData;

  if (!status)
  {
    status = sensor_vl53l4cx_sat.VL53L4CX_GetMultiRangingData(pMultiRangingData);

    if (writeLogToSerial)
      USB_SERIAL.printf("GetMultiRangingData Status: %s\n",VL53L4CX_RangeStatusCode(status).c_str());

    int no_of_object_found = pMultiRangingData->NumberOfObjectsFound;

    if (writeLogToSerial)
      USB_SERIAL.printf("VL53L4CX Satellite: Count=%d, #Objs=%1d \n", pMultiRangingData->StreamCount, no_of_object_found);
      
    maxDistance = -0.5;
    for (int j = 0; j < no_of_object_found; j++) 
    {
      if (pMultiRangingData->RangeData[j].RangeMilliMeter > maxDistance)
        maxDistance = ((float)pMultiRangingData->RangeData[j].RangeMilliMeter)/1000.0;

      if (writeLogToSerial)
      {
        if (j != 0) 
          USB_SERIAL.print("\r\n                               ");

        USB_SERIAL.print("status=");
        USB_SERIAL.print(pMultiRangingData->RangeData[j].RangeStatus);
        USB_SERIAL.print(", D=");
        USB_SERIAL.print(pMultiRangingData->RangeData[j].RangeMilliMeter);
        USB_SERIAL.print("mm");
        USB_SERIAL.print(", Signal=");
        USB_SERIAL.print((float)pMultiRangingData->RangeData[j].SignalRateRtnMegaCps / 65536.0);
        USB_SERIAL.print(" Mcps, Ambient=");
        USB_SERIAL.print((float)pMultiRangingData->RangeData[j].AmbientRateRtnMegaCps / 65536.0);
        USB_SERIAL.print(" Mcps, ");
        USB_SERIAL.print(VL53L4CX_RangeStatusCode(pMultiRangingData->RangeData[j].RangeStatus));
      }
    }
  }

  if (status == 0)
  {
    status = sensor_vl53l4cx_sat.VL53L4CX_ClearInterruptAndStartMeasurement();
    if (writeLogToSerial)
      USB_SERIAL.printf("VL53L4CX_ClearInterruptAndStartMeasurement Status: %s\n",VL53L4CX_RangeStatusCode(status).c_str());
  }

  mapScreen->setLidarDistance(maxDistance);

  newLidarDataReady = 0;
}
#endif

void loop()
{ 
  #ifdef COMPILE_TOF
  if (enableToFSensor && millis() > nextToFSensorTime)
  {
    nextToFSensorTime = millis() + timeBetweenToFReads;
    acquireLidarDistanceReading();
  }
  #endif

  if (millis() > nextReadHumiditySensorTime)
  {
    nextReadHumiditySensorTime = millis() + timeBetweenHumidityReads;
    acquireHumidityAndTemperatureReadings();
  }

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

          if (!mapScreen->isLocationInitialised())
          {
            mapScreen->setLocationLatLong(latitude, longitude);
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

        case 'B':   // record bread crumb trail on/off from Mako
        {
          mapScreen->setBreadCrumbTrailRecord(rxQueueESPNowItemBuffer[1] == 'Y');
          break;
        }

        case 'P':   // record bread crumb trail on/off from Mako
        {
          char* pinMessage = rxQueueESPNowItemBuffer+1;

          char* next = nullptr;

          double lat = strtod(pinMessage,&next);
          double lng = strtod(next,&next);
          double head = strtod(next,&next);
          double dep = strtod(next,&next);

          mapScreen->placePin(lat, lng, head, dep);
          break;
        }

        case 'D': // Receive current brightness from Mako
        {
          currentBrightnessReceived = *(rxQueueESPNowItemBuffer + 1)+((*(rxQueueESPNowItemBuffer + 2)) << 8);

          if (currentBrightnessSet != dimBrightness)
          {
            if (currentBrightnessReceived < nightBrightnessThreshold)
            {
              setScreenBrightness(nightBrightness);
            }
            else if (currentBrightnessReceived > dayBrightnessThreshold)
            {
              setScreenBrightness(dayBrightness);
            }
          }
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
    if (writeLogToSerial)
      USB_SERIAL.println("disable ESP Now, enable OTA");
    switchToPersistentOTAMode(true);
  }

  checkGoProButtons();

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
    else if (str == std::string("-") || str == std::string("slowerButton"))
    {
      diveTraceTrackStepPause+=diveTraceTrackStepIncrement;
      diveTrackTest = false;
      diveTraceTest = true;
    }
    else if (str == std::string("+") || str == std::string("fasterButton"))
    {
      if (diveTraceTrackStepPause > diveTraceTrackStepIncrement)
        diveTraceTrackStepPause-=diveTraceTrackStepIncrement;
      
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
    else if (str == std::string("rebootButton"))
    {
      esp_restart();
    }
    else if (str == std::string("startSerialButton"))
    {
      writeLogToSerial = true;
    }
    else if (str == std::string("stopSerialButton"))
    {
      writeLogToSerial = false;
    #ifdef USE_WEBSERIAL
      WebSerial.closeAll();
    #endif
    }
    else if (str == std::string("dim"))
    {
      setScreenBrightness(dimBrightness);
    }
    else if (str == std::string("night"))
    {
      setScreenBrightness(nightBrightness);
    }
    else if (str == std::string("bright"))
    {
      setScreenBrightness(dayBrightness);
    }
    else if (str == std::string("sleep"))
    {
      forceDeepSleep();
      // never goes beyond here
    }
  }

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
    while (end > millis())
    {
//      String status = amoled.getChargeStatusString();

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
//        prevStatus = status;
        delay(2000);

      checkGoProButtons();
    }
}

void testMapDisplay()
{
  double latitude = 51.4605855;    // lightning boat
  double longitude = -0.54890166666666; 
  double heading = 0.0;
  mapScreen->drawDiverOnBestFeaturesMapAtCurrentZoom(latitude,longitude,heading);
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

#ifdef USE_WEBSERIAL
void webSerialReceiveMessage(uint8_t *data, size_t len){
  WebSerial.println("Received Data...");
  String d = "";
  for(int i=0; i < len; i++){
    d += char(data[i]);
  }

  WebSerial.println(d);

  if (d == "serial-off")
  {
    writeLogToSerial = false;
    WebSerial.closeAll();
  }
  else if (d == "serial-on")
  {
    writeLogToSerial = true;
    // force page refresh how?
  }
}
#endif

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


      #ifdef USE_WEBSERIAL
      static bool webSerialInitialised = false;

      if (!webSerialInitialised)
      {
        WebSerial.begin(&asyncWebServer);
        WebSerial.msgCallback(webSerialReceiveMessage);
        webSerialInitialised = true;
      }
      #endif

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
    checkGoProButtons();  
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

void forceDeepSleep()
{
  // put ESP32 into deep sleep - faster charging
  compositeSprite->fillSprite(TFT_RED);
  resetCompositeSpriteCursor();
  compositeSprite->print("Sleep for Charging...");
  mapScreen->copyCompositeSpriteToDisplay();
  delay(3000);

  setScreenBrightness(offBrightness);
  WiFi.mode(WIFI_MODE_NULL);  // disable wifi
  amoled.sleep();

  esp_sleep_enable_ext0_wakeup((gpio_num_t)BUTTON_SIDE_GPIO, 0);  //1 = High, 0 = Low

  esp_deep_sleep_start();
  // never goes beyond here
}