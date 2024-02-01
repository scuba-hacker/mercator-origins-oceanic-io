#ifndef navigation_waypoints_h
#define navigation_waypoints_h

#include <stdint.h>
#include <array>

class navigationWaypoint
{
  public:
    const char*  _label;
    double _lat;
    double _long;

    navigationWaypoint()
    {
      _label = nullptr;
      _lat = _long = 0.0;
    }

    navigationWaypoint(const char*  label, double latitude, double longitude) : _label(label), _lat(latitude), _long(longitude)
    {
    }
};

class geoRef
{
  static const int geoMapsSize=10;  // should be same number as number of maps
  public:
    int geoMaps[geoMapsSize];    // corresponds to number of maps being used - set to arbitrarily large size.
};


extern const uint8_t waypointCount;
extern const std::array<navigationWaypoint,113> waypoints;
extern uint8_t getWaypointsCount();

extern const uint8_t exitWaypointIndicesSize;
extern std::array<int,10> exitWaypointIndices;
extern int exitWaypointCount;

#endif
