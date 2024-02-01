#include <MapScreen_T4.h>

#include <TFT_eSPI.h>

#include "LilyGo_amoled.h"

extern const uint16_t lily_wraysbury_N[];
extern const uint16_t lily_wraysbury_W[];
extern const uint16_t lily_wraysbury_SW[];
extern const uint16_t lily_wraysbury_S[];
extern const uint16_t lily_wraysbury_SE[];
extern const uint16_t lily_wraysbury_All[];

const geo_map MapScreen_T4::s_maps[] =
{
  [0] = { .mapData = lily_wraysbury_N, .label="North", .backColour=TFT_BLACK, .backText="", .surveyMap=false, .swapBytes=false, .mapLongitudeLeft = -0.5503, .mapLongitudeRight = -0.5473, .mapLatitudeBottom = 51.4613},
  [1] = { .mapData = lily_wraysbury_W, .label="West", .backColour=TFT_BLACK, .backText="", .surveyMap=false, .swapBytes=false, .mapLongitudeLeft = -0.5501, .mapLongitudeRight = -0.5471, .mapLatitudeBottom = 51.4606},
  [2] = { .mapData = lily_wraysbury_SW, .label="South West", .backColour=TFT_BLACK, .backText="", .surveyMap=false, .swapBytes=false, .mapLongitudeLeft = -0.5494, .mapLongitudeRight = -0.5464, .mapLatitudeBottom = 51.4597},
  [3] = { .mapData = lily_wraysbury_S, .label="South", .backColour=TFT_BLACK, .backText="", .surveyMap=false, .swapBytes=false, .mapLongitudeLeft = -0.5491, .mapLongitudeRight = -0.5461, .mapLatitudeBottom = 51.4591},
  [4] = { .mapData = lily_wraysbury_SE, .label="South East", .backColour=TFT_BLACK, .backText="", .surveyMap=false, .swapBytes=false, .mapLongitudeLeft = -0.548, .mapLongitudeRight = -0.545, .mapLatitudeBottom = 51.4588},
  [5] = { .mapData = lily_wraysbury_All, .label="All", .backColour=TFT_BLACK, .backText="", .surveyMap=false, .swapBytes=false, .mapLongitudeLeft = -0.5517, .mapLongitudeRight = -0.5437, .mapLatitudeBottom = 51.4588},
  [6] = { .mapData = nullptr, .label="Canoe", .backColour=TFT_CYAN, .backText="Canoe",.surveyMap=true, .swapBytes=false, .mapLongitudeLeft = -0.54910, .mapLongitudeRight = -0.54880, .mapLatitudeBottom = 51.46190}, // Canoe area
  [7] = { .mapData = nullptr, .label="Sub",  .backColour=TFT_CYAN, .backText="Sub",.surveyMap=true, .swapBytes=false, .mapLongitudeLeft = -0.54931, .mapLongitudeRight = -0.54900, .mapLatitudeBottom = 51.4608}, // Sub area
};

const std::array<MapScreen_ex::pixel, MapScreen_T4::s_registrationPixelsSize> MapScreen_T4::s_registrationPixels
{{
[0]=  {  .x=o,     .y=o,     .colour=0xFF00},
[1]=  {  .x=hX_t3-o,  .y=o,     .colour=0xFF00},
[2]=  {  .x=o,     .y=hY_t3-o,  .colour=0xFF00},
[3]=  {  .x=hX_t3-o,  .y=hY_t3-o,  .colour=0xFF00},
  
[4]=  {  .x=hX_t3+o,  .y=o,     .colour=0xFFFF},
[5]=  {  .x=mX_t3-o,  .y=o,     .colour=0xFFFF},
[6]=  {  .x=hX_t3+o,  .y=hY_t3-o,  .colour=0xFFFF},
[7]=  {  .x=mX_t3-o,  .y=hY_t3-o,  .colour=0xFFFF},

[8]=  {  .x=o,     .y=hY_t3+o,  .colour=0x00FF},
[9]=  {  .x=hX_t3-o,  .y=hY_t3+o,  .colour=0x00FF},
[10]= {  .x=o,     .y=mY_t3-o,  .colour=0x00FF},
[11]= {  .x=hX_t3-o,  .y=mY_t3-o,  0x00FF},

[12]= {  .x=hX_t3+o,  .y=hY_t3+o,  .colour=0x0000},
[13]= {  .x=mX_t3-o,  .y=hY_t3+o,  .colour=0x0000},
[14]= {  .x=hX_t3+o,  .y=mY_t3-o,  .colour=0x0000},
[15]= {  .x=mX_t3-o,  .y=mY_t3-o,  .colour=0x0000}
}};     // How weird this is an older syntax from C++11 which requires an extra open and close brace.

MapScreen_ex::pixel MapScreen_T4::getRegistrationMarkLocation(int index) 
{ 
    if (index < s_registrationPixelsSize)
        return s_registrationPixels[index];
    else 
        return pixel(-1,-1);
}

void MapScreen_T4::initFeatureToMapsLookup()
{
  for (int i=0; i<getWaypointsCount(); i++)                    // MBJ REFACTOR  - needs range and enumerate in C++20
  {
    initMapsForFeature(waypoints[i],_featureToMaps[i]);     // index i used here
  }
}
void MapScreen_T4::initMapsForFeature(const navigationWaypoint& waypoint, geoRef& ref)
{
  int refIndex = 0;
  
  pixel p;
  
  for (uint8_t i = getFirstDetailMapIndex(); i < getEndDetailMaps(); i++)    // MBJ REFACTOR  - needs range and enumerate in C++20 (with break at getEndDetailMaps())
  {
    p = convertGeoToPixelDouble(waypoint._lat, waypoint._long, s_maps[i]);
    if (p.x >= 0 && p.x < getTFTWidth() && p.y >=0 && p.y < getTFTHeight())
    {
      ref.geoMaps[refIndex++] = i;    // index i used here
    }
    else
    {
      ref.geoMaps[refIndex++] = -1;
    }
  }
}

MapScreen_T4::MapScreen_T4(TFT_eSPI& tft, LilyGo_AMOLED& lilygoT3) : MapScreen_ex(tft),_amoled(lilygoT3)
{
  initMapScreen();

  initFeatureToMapsLookup();

  _scratchPadSprite = std::make_unique<TFT_eSprite>(&tft);  
  _scratchPadSprite->createSprite(getTFTWidth(),getTFTHeight());
}

int MapScreen_T4::getFirstDetailMapIndex()
{
  return _NMapIndex;
}

int MapScreen_T4::getEndDetailMaps()
{
  return _allLakeMapIndex;
}

int MapScreen_T4::getAllMapIndex()
{
  return _allLakeMapIndex;
}

const geo_map* MapScreen_T4::getMaps()
{
  return s_maps;
}

void MapScreen_T4::copyFullScreenSpriteToDisplay(TFT_eSprite& sprite)
{
    _amoled.pushColors(0,0,getTFTWidth(),getTFTHeight(),reinterpret_cast<uint16_t*>(sprite.getPointer()));
}

void MapScreen_T4::fillScreen(int colour)
{
  _scratchPadSprite->fillSprite(colour);
  copyFullScreenSpriteToDisplay(*_scratchPadSprite);
}

// This needs customising for the T4 maps. Writes text to the canoe/sub zoomed in zones
void MapScreen_T4::writeMapTitleToSprite(TFT_eSprite& sprite, const geo_map& map)
{
    if (map.backText)
    {
      sprite.setCursor(5,30);
      sprite.setTextSize(3);
      sprite.setTextColor(TFT_BLUE, map.backColour);
      sprite.println(map.backText);
    }
}

// This needs customising for the T4 maps. Currently switches when within 30 pixels of screen edge.
const geo_map* MapScreen_T4::getNextMapByPixelLocation(MapScreen_ex::pixel loc, const geo_map* thisMap)
{
  const geo_map* nextMap = thisMap;

  if (thisMap == _allLakeMap)
    return _allLakeMap;

  if ((thisMap == _canoeZoneMap || thisMap == _subZoneMap) && isPixelOutsideScreenExtent(loc))
  {
    nextMap = (thisMap == _canoeZoneMap ? _NMap : _SWMap);
    _zoom = _priorToZoneZoom;
  }
  else if (thisMap == _NMap)   // go right from 0 to 1
  {
    if (isPixelInCanoeZone(loc, *thisMap))
    {
      _priorToZoneZoom=_zoom;
      _zoom = 1;
      nextMap = _canoeZoneMap;
    }
    else if (isPixelInSubZone(loc, *thisMap))
    {
      _priorToZoneZoom=_zoom;
      _zoom = 1;
      nextMap = _subZoneMap;
    }
    else if (loc.y >= 370)
    {
      nextMap=_WMap;
    }
  }
  else if (thisMap == _WMap)
  { 
    if (isPixelInCanoeZone(loc, *thisMap))
    {
      _priorToZoneZoom=_zoom;
      _zoom = 1;
      nextMap = _canoeZoneMap;
    }
    else if (isPixelInSubZone(loc, *thisMap))
    {
      _priorToZoneZoom=_zoom;
      _zoom = 1;
      nextMap = _subZoneMap;
    }
    else if (loc.x >= 620 || loc.y >= 370 )
    {
      nextMap=_SWMap;
    }
    else if (loc.x <=10 || loc.y <= 10)
    {
      nextMap=_NMap;
    }
  }
  else if (thisMap == _SWMap)
  {
    if (loc.x >= 620 || loc.y >=370)
      nextMap=_SMap;
    else if (loc.x <= 30 || loc.y <= 30)
      nextMap=_NMap;          // go left from 2 to 1
  }
  else if (thisMap == _SMap)
  {
    if  (loc.x <= 30 || loc.y <= 30) // go left from 3 to 2
      nextMap = _SWMap;
    else if (loc.x >= 570 || loc.y >= 370)
      nextMap = _SEMap;
  }

  return nextMap;
}

// MBJ RECALCULATE pixel bounding box for canoe FOR T4 DISPLAY
// This is the M5 canoe bounding box - needs updating for T4
// BOUNDING BOX FOR CANOE M5: TOP-LEFT (62, 51) BOT-RIGHT (79, 71) 
const std::array<MapScreen_ex::MapScreen_ex::BoundingBox, 1> MapScreen_T4::boundingBoxesCanoe = {{{{62,51},{79,71},{*MapScreen_T4::_NMap}}}};
bool MapScreen_T4::isPixelInCanoeZone(const MapScreen_ex::pixel loc, const geo_map& thisMap) const
{
  return false; // temp - remove when new bounding box pixels are coded above

  for (auto& box : boundingBoxesCanoe)
  {
    if (box.withinBox(loc, thisMap))
      return true;
  }

  return false;
}

// MBJ RECALCULATE pixel bounding box for sub FOR T4 DISPLAY
// BOUNDING BOX FOR SUB M5 North Map: TOP-LEFT (48, 168) BOT-RIGHT (65, 191)
// BOUNDING BOX FOR SUB M5 Cafe Jetty Map: TOP-LEFT (12, 53) BOT-RIGHT (31, 72)
const std::array<MapScreen_ex::MapScreen_ex::BoundingBox, 2> MapScreen_T4::boundingBoxesSub = { {
                  {{48,168},{65,191},{*MapScreen_T4::_NMap}},
                  {{12,53},{31,72},{*MapScreen_T4::_NMap/*_cafeJettyMap*/}}
                  } };

bool MapScreen_T4::isPixelInSubZone(const MapScreen_ex::pixel loc, const geo_map& thisMap) const
{
  return false;  // temp - remove when new bounding box pixels are coded above

  for (auto& box : boundingBoxesSub)
  {
    if (box.withinBox(loc, thisMap))
      return true;
  }

  return false;
}

