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
  static const int geoMapsSize=10;  // should be same number as number of maps? - set to arbitrarily larger size.
  public:
    int geoMaps[geoMapsSize];       // MBJ REFACTOR to std::array
};


extern const uint8_t waypointCount;
extern const std::array<navigationWaypoint,113> waypoints;    // MBJ REFACTOR - get rid of hardcoding how?
extern uint8_t getWaypointsCount();

extern const uint8_t exitWaypointIndicesSize;
extern std::array<int,10> exitWaypointIndices;      // MBJ REFACTOR - get rid of hardcoding how?
extern int exitWaypointCount;

#endif
