#include <Arduino.h>

/////////////////// DIAG SERIAL SETTINGS /////////////////////
bool writeLogToSerial=false;
bool testPNG=false;

//#define USE_WEBSERIAL

#include "SerialConfig.h"   // Serial configuration and buffer logging system

/////////////////////////////////////////////////////////////

// **** CONCURRENT APP LOAD WITH t4-i2s ****

// Upload Oceanic via USB to app0, and then use OTA to upload t4-i2s to app1.
// If t4-i2s is uploaded via USB, then there is no OTA capability in that app to upload Oceanic.
// USB upload always uploads to app0 and overwrites the otadata partition with boot_app0.bin so all 
// trace of the other OTA app are gone.
// To install the apps side-by-side, Oceanic must be uploaded by USB, and then t4-i2s by OTA.

bool enableOtaPartitionSwitch = true;  // Long-press top button switches to the other OTA app partition

/////////////////////////////////////////////////////////////


#include <esp_sleep.h>
#include "driver/rtc_io.h"

#include <MapScreen_T4.h>
#include <NavigationWaypoints.h>
#include <LilyGo_AMOLED.h>
#include <TFT_eSPI.h>

#include <Adafruit_AHTX0.h>
#ifdef COMPILE_TOF
#include <vl53l4cx_class.h>
#endif

/* 

BUGS:
1. sleep for charging ends up waking up quickly
*/
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

/test      Home Page - all diagnostics and tests
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
/boot      reboot

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

#include <esp_ota_ops.h>
#include <esp_partition.h>

#include <fonts/NotoSansBold36.h>
//#include <fonts/Final_Frontier_28.h>
#include <fonts/NotoSansMonoSCB20.h>

// rename the git file "mercator_secrets_template.c" to the filename below, filling in your wifi credentials etc.
#include "mercator_secrets.c"

#include "Button.h"

bool enableToFSensor=false;
bool doInitialSerialTransmitTest=false;
bool doInitialSerialReceiveEchoTest=false;
bool testPreCannedLatLong=false;
bool diveTrackTest = false;
bool diveTraceTest = false;
uint32_t diveTraceTrackStepPause = 1;
const uint32_t diveTraceTrackStepIncrement = 50;

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
//
#include <AsyncTCP.h>           // OTA updates
#include <ESPAsyncWebServer.h>  // OTA updates
#include <MercatorElegantOTA.h>    // OTA updates

MercatorElegantOtaClass MercatorElegantOta;

#include <FS.h>
#include <LittleFS.h>
#include <SimpleFTPServer.h>

FtpServer ftpServer;
bool ftpActive = false;

void _ftpConnectCallback(FtpOperation ftpOperation, uint32_t freeSpace, uint32_t totalSpace);
void _ftpTransferCallback(FtpTransferOperation ftpOperation, const char* name, uint32_t transferredSize);

Adafruit_AHTX0 ahtSensor;

const String ssid_not_connected = "-";
String ssid_connected;

// ************** ESPNow variables **************

uint16_t ESPNowMessagesReceived = 0;
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


const double uninitialisedLatitude = 0.0;
const double uninitialisedLongitude = 0.0;

const double startTraceTestLatitude = 51.4605855;    // lightning boat
const double startTraceTestLongitude=-0.548316;

//double latitude  = startTraceTestLatitude;    // lightning boat
//double longitude = startTraceTestLongitude;

double latitude = uninitialisedLatitude;
double longitude = uninitialisedLongitude;

double heading=0.0;
uint32_t x_message_flags = 0;
uint32_t X_MESSAGE_FIX_FLAG = 0x01;
float depth=0.0;
float course=0.0;
float targetHeadingFromMako=0.0;
float targetDistanceFromMako=0.0;

bool locationHasFix = false;
int fixMessagesReceived = 0, noFixMessagesReceived = 0;

double latitudeDelta = 0.0;
double longitudeDelta = 0.0;

int otaScreenBackColour = TFT_GREEN;
int otaScreenForeColour = TFT_BLUE;
int espScanBackColour = TFT_PURPLE;
int espScanForeColour = TFT_WHITE;
int wifiScanBackColour = TFT_CYAN;
int wifiScanForeColour = TFT_BLUE;

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

HttpRequestQueue httpQueue;

char currentTarget[256];
char previousTarget[256];
bool refreshTargetShown = false;

bool checkGoProButtons();
void publishToMakoBreadCrumbRecord(const bool record);
void publishToMakoTestMessage(const char* testMessage);
void publishToMakoReedActivation(const bool topReed, const uint32_t ms);
void placePinTest();

void initI2C();
void initHumiditySensor();

uint32_t nextBattUpdateTime = 0;
uint32_t battUpdateCadence = 1000;

uint32_t nextReadHumiditySensorTime = 0;
const uint32_t timeBetweenHumidityReads = 5000;
uint32_t nextToFSensorTime = 0;
const uint32_t timeBetweenToFReads = 100;


void resetCurrentTarget();

void resetMap();
void resetClock();

void setupFTPServer();
const char* scanForKnownNetwork();
bool setupOTAWebServer(const char* _ssid, const char* _password, const char* label, uint32_t timeout, bool wifiOnly = false);
void toggleOTAActiveAndWifiIfUSBPowerOff();
void clearButtons();
void updateButtons();
void readAndTestGoProSwitches();
bool InitESPNow();
bool configAndStartUpESPNow();
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

void readSensors();
void executeTests(bool refreshMap);

void setScreenBrightness(uint16_t brightness);
void startupScreen();
void recoveryScreen();

void dumpHeapUsage(const char* msg)
{  
  multi_heap_info_t info;
  heap_caps_get_info(&info, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT); // internal RAM, memory capable to store data or to create new task
  USB_SERIAL_PRINTF("\n%s : free heap bytes: %i  largest free heap block: %i min free ever: %i\n",  msg, info.total_free_bytes, info.largest_free_block, info.minimum_free_bytes);
}

void testMapDisplay();
bool disableESPNowandEnableOTA();
void switchToPersistentOTAMode(bool clearScreen);
void processReceivedESPNowMessages();
bool processReceivedHTTPRequests();

void resetCompositeSpriteCursor()
{
  compositeSprite->setCursor(0,30);
}

bool ahtStatus=false;   // Adafruit AHT20
bool tofStatus=false;   // Adafruit VL53L4CX  https://github.com/stm32duino/VL53L4CX

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

bool  setupComplete = false;

void setup()
{
  #ifndef USE_WEBSERIAL
    USB_SERIAL.begin(115200);
    delay(50);
  #endif
  
  dumpHeapUsage("Setup(): at startup ");

  if (LittleFS.begin(true)) {
    USB_SERIAL.println("Setup(): LittleFS mounted OK");
  } else {
    USB_SERIAL.println("Setup(): LittleFS mount FAILED - PNG maps will not display");
  }

  amoled.begin();
  setScreenBrightness(dayBrightness);
  mapScreen = std::make_unique<MapScreen_T4>(tft,amoled);

  USB_SERIAL_PRINTLN("created MapScreen_T4");
    
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

  if (mapScreen)
    mapScreen->setLocationLatLong(uninitialisedLatitude, uninitialisedLongitude);

  p_primaryButton = &SwitchGoProTop;
  p_secondButton = &SwitchGoProSide;

  clearButtons();
  
  recoveryScreen();

  msgsESPNowReceivedQueue = xQueueCreate(queueESPNowLength,sizeof(rxQueueESPNowItemBuffer));

  if (enableOTAServerAtStartup)
  {
    compositeSprite->fillSprite(TFT_DARKGREY);
    resetCompositeSpriteCursor();
    compositeSprite->println("Start OTA\nAutomatically\n");
    mapScreen->copyCompositeSpriteToDisplay();
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
  else
  {
    resetCompositeSpriteCursor();
    compositeSprite->fillSprite(TFT_PINK);
    compositeSprite->setTextColor(TFT_BLACK);
    compositeSprite->printf("ESPNow Not Initialised\notaActive=%i\nenableESPNow=%i\nmsgESPNowReceivedQueue=%i\n",otaActive,enableESPNow,(msgsESPNowReceivedQueue!=nullptr));
    mapScreen->copyCompositeSpriteToDisplay();
    delay(3000);
  }
  setupComplete = true;
  dumpHeapUsage("Setup(): end ");
}

void loop()
{ 
  #ifdef COMPILE_TOF
  if (enableToFSensor && millis() > nextToFSensorTime)
  {
    nextToFSensorTime = millis() + timeBetweenToFReads;
    acquireLidarDistanceReading();
  }
  #endif
  
  if (testPNG)
  {
    bool swapBytes = false;
    mapScreen->testDrawPNG("/maps/home_middle.png", swapBytes);
    mapScreen->testDrawPNG("/maps/home_all.png", swapBytes);
    mapScreen->testDrawPNG("/maps/vobster_all.png", swapBytes);
    mapScreen->testDrawPNG("/maps/vobster_centre.png", swapBytes);
    mapScreen->testDrawPNG("/maps/lily_wraysbury_all.png", swapBytes);
    mapScreen->testDrawPNG("/maps/lily_wraysbury_N.png", swapBytes);
    mapScreen->testDrawPNG("/maps/lily_wraysbury_S.png", swapBytes);
    mapScreen->testDrawPNG("/maps/lily_wraysbury_SE.png", swapBytes);
    mapScreen->testDrawPNG("/maps/lily_wraysbury_SW.png", swapBytes);
    mapScreen->testDrawPNG("/maps/lily_wraysbury_W.png", swapBytes);
    return;
  }

  readSensors();

  processReceivedESPNowMessages();

  checkGoProButtons();

  bool refreshMap = processReceivedHTTPRequests();

  if (diveTrackTest || diveTraceTest)
    executeTests(refreshMap);

  if (ftpActive)
    ftpServer.handleFTP();
}

bool processReceivedHTTPRequests()
{
  bool refreshMap = false;
  std::string str;
  while (httpQueue.pop(str))
  {
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
      latitude=startTraceTestLatitude;
      longitude=startTraceTestLongitude;
      mapScreen->setLocationLatLong(latitude,longitude);
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
      diveTrackTest = false;
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
      latitude=startTraceTestLatitude;
      longitude=startTraceTestLongitude;
      diveTrackTest = false;
      diveTraceTest = false;
      refreshMap = true;
    }
    else if (str == std::string("stop") || str == std::string("stopButton"))
    {
      latitudeDelta=0;
      longitudeDelta=0;
      diveTrackTest = false;
      diveTraceTest = true;
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
    else if (str.rfind("zwp:", 0) == 0)
    {
      int waypointIndex = atoi(str.c_str() + 4);
      if (waypointIndex >= 0 && waypointIndex < WraysburyWaypoints::getWaypointsCount())
      {
        latitude = WraysburyWaypoints::waypoints[waypointIndex]._lat;
        longitude = WraysburyWaypoints::waypoints[waypointIndex]._long;
        mapScreen->setLocationLatLong(latitude,longitude);
        diveTrackTest = false;
        diveTraceTest = true;
        latitudeDelta = 0;
        longitudeDelta = 0;
        refreshMap = true;
      }
    }
  }

  return refreshMap;
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

void forceDeepSleep()
{
  // put ESP32 into deep sleep - faster charging
  compositeSprite->fillSprite(TFT_RED);
  resetCompositeSpriteCursor();
  compositeSprite->setTextColor(TFT_WHITE);
  compositeSprite->print("Triggered Deep Sleep...");
  mapScreen->copyCompositeSpriteToDisplay();
  delay(3000);

  setScreenBrightness(offBrightness);
  WiFi.mode(WIFI_MODE_NULL);  // disable wifi
  amoled.sleep();

// Add before esp_sleep_enable_ext0_wakeup
  rtc_gpio_init((gpio_num_t)BUTTON_SIDE_GPIO);
  rtc_gpio_set_direction((gpio_num_t)BUTTON_SIDE_GPIO, RTC_GPIO_MODE_INPUT_ONLY);
  rtc_gpio_pullup_en((gpio_num_t)BUTTON_SIDE_GPIO);
  rtc_gpio_pulldown_dis((gpio_num_t)BUTTON_SIDE_GPIO);
  esp_sleep_enable_ext0_wakeup((gpio_num_t)BUTTON_SIDE_GPIO, 0);  //1 = High, 0 = Low

  esp_deep_sleep_start();
  // never goes beyond here
}

bool switchToApp(esp_partition_subtype_t subtype) {
  const esp_partition_t *target = esp_partition_find_first(
      ESP_PARTITION_TYPE_APP, subtype, NULL);
  if (target == NULL) {
    USB_SERIAL_PRINTLN("OTA switch: target partition not found");
    return false;
  }
  esp_app_desc_t desc;
  if (esp_ota_get_partition_description(target, &desc) != ESP_OK) {
    USB_SERIAL_PRINTLN("OTA switch: target partition empty or invalid");
    return false;
  }
  
  USB_SERIAL_PRINTF("OTA switch: rebooting into %s\n", desc.project_name);
  
  if (esp_ota_set_boot_partition(target) != ESP_OK) {
    USB_SERIAL_PRINTLN("OTA switch: esp_ota_set_boot_partition failed");
    return false;
  }
  esp_restart();
  return true; // never get here!
}

bool switchToOtherOtaPartition() {
    const esp_partition_t *running = esp_ota_get_running_partition();
    const esp_partition_subtype_t target =
        (running->subtype == ESP_PARTITION_SUBTYPE_APP_OTA_0)
            ? ESP_PARTITION_SUBTYPE_APP_OTA_1
            : ESP_PARTITION_SUBTYPE_APP_OTA_0;
    // only returns if there is a failure.
    return switchToApp(target);
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

#define BUILD_INCLUDE_MAIN_DISPLAY_CODE
#include "main_display_code.cpp"

#define BUILD_INCLUDE_MAIN_BUTTON_PRESS_CODE
#include "main_button_press_code.cpp"

#define BUILD_INCLUDE_MAIN_SENSOR_CODE
#include "main_sensor_code.cpp"

#define BUILD_INCLUDE_NETWORK_CODE
#include "main_network_code.cpp"

#define BUILD_INCLUDE_ESP_NOW_MESSAGE_CODE
#include "main_esp_now_message_code.cpp"

#define BUILD_INCLUDE_MAIN_ESP_NOW_CODE
#include "main_esp_now_code.cpp"

#define BUILD_INCLUDE_FTP_SERVER_CODE
#include "main_ftp_server_code.cpp"