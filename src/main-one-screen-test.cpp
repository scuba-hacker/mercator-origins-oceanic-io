
#ifdef RUN_SINGLE_SCREEN_TEST

#include <Arduino.h>

#include "MapScreen_T4.h"

#include "LilyGo_amoled.h"

#include <TFT_eSPI.h>

#include <lv_conf.h>
#include <LV_Helper.h>
//#include <lvgl.h>

TFT_eSPI tft = TFT_eSPI();
LilyGo_AMOLED amoled;
std::unique_ptr<MapScreen_T4> mapScreen;

extern const unsigned short lily_wraysbury_all[];
extern const unsigned short lily_wraysbury_N[];
extern const unsigned short lily_wraysbury_SE[];
extern const unsigned short lily_wraysbury_S[];
extern const unsigned short lily_wraysbury_SW[];
extern const unsigned short lily_wraysbury_W[];

void setup()
{
  Serial.begin(115200);
  Serial.flush();
  delay(50);

  amoled.begin();
  amoled.setBrightness(255);
  mapScreen.reset(new MapScreen_T4(&tft,&amoled));
}

void loop()
{ 

}
#endif
