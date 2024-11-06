#ifndef __I_RING_SENSOR_HPP__
#define __I_RING_SENSOR_HPP__

namespace pvegas {
namespace robot {
namespace subsystems {
namespace arm {
class IRingSensor {
 public:
  // destroys the ring sensor
  virtual ~IRingSensor() = default;

  // initialize the ring sensor
  virtual void init() = 0;

  // run the ring sensor
  virtual void run() = 0;

  // determines if there is a ring in front of the sensor
  virtual bool hasRing() = 0;

  // determines the hue from the color sensor 
  virtual double getHue() = 0;
};
}
}
}
}
#endif