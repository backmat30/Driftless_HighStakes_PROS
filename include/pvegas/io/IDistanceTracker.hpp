#ifndef __I_DISTANCE_TRACKER_HPP__
#define __I_DISTANCE_TRACKER_HPP__

namespace driftless {
namespace io {
class IDistanceTracker {
 public:
  // destroyer
  virtual ~IDistanceTracker() = default;

  // initialize the distance tracker
  virtual void init() = 0;

  // reset the distance tracker
  virtual void reset() = 0;

  // get the distance recorded by the distance tracker
  virtual double getDistance() = 0;

  // sets the distance tracked by the sensor to a new value
  virtual void setDistance(double distance) = 0;
};
}  // namespace io
}  // namespace pvegas
#endif