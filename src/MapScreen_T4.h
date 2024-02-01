#ifndef MAPSCREEN_T4_H
#define MAPSCREEN_T4_H

#include <MapScreen_ex.h>

#include "navigation_waypoints.h"

class LilyGo_AMOLED;

class MapScreen_T4 : public MapScreen_ex
{
    private:
        static constexpr int s_registrationPixelsSize = 16;
        static const std::array<MapScreen_ex::pixel, s_registrationPixelsSize> s_registrationPixels;

        static constexpr int16_t mX_t3 = 600,  hX_t3 = 300;
        static constexpr int16_t mY_t3 = 450,  hY_t3 = 225;
        static constexpr int16_t o = 30;

        static const int s_allLakeMapIndex = 5;
        static const int s_canoeMapIndex = 6;
        static const int s_subMapIndex = 7;
        static const int s_initialZoom = 1;

        LilyGo_AMOLED& _amoled;
    
        std::unique_ptr<TFT_eSprite> _scratchPadSprite;

        virtual int getFirstDetailMapIndex() override;
        virtual int getEndDetailMaps() override;
        virtual int getAllMapIndex() override;
        virtual const geo_map* getMaps() override;

        virtual const geo_map* getNextMapByPixelLocation(MapScreen_ex::pixel loc, const geo_map* thisMap) override;

        static const geo_map s_maps[];

        static constexpr const geo_map* _NMap=s_maps;          static const uint8_t _NMapIndex = 0;
        static constexpr const geo_map* _WMap=s_maps+1;        static const uint8_t _WMapIndex = 1;
        static constexpr const geo_map* _SWMap=s_maps+2;       static const uint8_t _SWMapIndex = 2;
        static constexpr const geo_map* _SMap=s_maps+3;        static const uint8_t _SMapIndex = 3;
        static constexpr const geo_map* _SEMap=s_maps+4;       static const uint8_t _SEMapIndex = 4;
        static constexpr const geo_map* _allLakeMap=s_maps+5;  static const uint8_t _allLakeMapIndex = 5;
        static constexpr const geo_map* _canoeZoneMap=s_maps+6;static const uint8_t _canoeZoneMapIndex = 6;
        static constexpr const geo_map* _subZoneMap=s_maps+7;  static const uint8_t _subZoneMapIndex = 7;

        static const std::array<MapScreen_ex::MapScreen_ex::BoundingBox, 1> boundingBoxesCanoe;
        static const std::array<MapScreen_ex::MapScreen_ex::BoundingBox, 2> boundingBoxesSub;

        static const int maxFeatures = 255;
        std::array<geoRef, maxFeatures>    _featureToMaps;

    public:
        MapScreen_T4(TFT_eSPI& tft, LilyGo_AMOLED& lilygoT3);

        virtual MapScreen_ex::pixel getRegistrationMarkLocation(int index) override;

        virtual int getRegistrationMarkLocationsSize() override { return s_registrationPixelsSize; }

        void initFeatureToMapsLookup();
        void initMapsForFeature(const navigationWaypoint& waypoint, geoRef& ref);

        virtual int16_t getTFTWidth() const override {return 600;}
        virtual int16_t getTFTHeight() const override {return 450; }

        virtual void fillScreen(int colour) override;
        virtual void copyFullScreenSpriteToDisplay(TFT_eSprite& sprite) override;
        virtual void writeMapTitleToSprite(TFT_eSprite& sprite, const geo_map& map) override;

        virtual bool isPixelInCanoeZone(const MapScreen_ex::pixel loc, const geo_map& thisMap) const override;
        virtual bool isPixelInSubZone(const MapScreen_ex::pixel loc, const geo_map& thisMap) const override;
};
#endif

