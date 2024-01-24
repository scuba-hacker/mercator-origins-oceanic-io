#include <Arduino.h>
#include <Wire.h>

#include "MapScreen_T4.h"

#include "LilyGo_amoled.h"
#include "LilyGo_Display.h"

#include <TFT_eSPI.h>

TFT_eSPI tft = TFT_eSPI();
LilyGo_AMOLED amoled;
std::unique_ptr<MapScreen_T4> mapScreen;

extern const unsigned short lily_wraysbury_all[];
extern const unsigned short lily_wraysbury_N[];
extern const unsigned short lily_wraysbury_SE[];
extern const unsigned short lily_wraysbury_S[];
extern const unsigned short lily_wraysbury_SW[];
extern const unsigned short lily_wraysbury_W[];

void testMapDisplay();

void setup()
{
  Serial.begin(115200);
  Serial.flush();
  delay(50);

  amoled.begin();
  amoled.setBrightness(255);
  mapScreen.reset(new MapScreen_T4(&tft,&amoled));

  mapScreen->setDrawAllFeatures(true);

  mapScreen->setUseDiverHeading(true);

  testMapDisplay();
//  mapScreen->fillScreen(TFT_RED);
}

void loop()
{ 
}

void testMapDisplay()
{
  double latitude = 51.4605855;    // lightning boat
  double longitude = -0.54890166666666; 
  double heading = 0.0;
  mapScreen->drawDiverOnBestFeaturesMapAtCurrentZoom(latitude,longitude,heading);
}