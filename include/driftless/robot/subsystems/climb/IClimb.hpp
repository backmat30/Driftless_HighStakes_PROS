#ifndef __I_CLIMB_HPP__
#define __I_CLIMB_HPP__

namespace driftless {

  namespace robot {

    namespace subsystems {

      namespace climb {

        class IClimb {
          virtual ~IClimb() = default;

          virtual void init() = 0;

          virtual void run() = 0;

          virtual void toggleClimbing() = 0;

          virtual void pullBackClimber() = 0;

          virtual void pushForwardClimber() = 0;
        };
      }
    }
  }
}
#endif