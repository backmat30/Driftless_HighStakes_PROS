#ifndef __I_CONTROLLER_HPP__
#define __I_CONTROLLER_HPP__

#include "EControllerAnalog.hpp"
#include "EControllerDigital.hpp"
#include <string>
namespace pvegas{
    namespace io{
        class IController{
            virtual ~IController() = 0;

            virtual void init() = 0;

            virtual void run() = 0;

            virtual double getAnalog(EControllerAnalog channel) = 0;

            virtual bool getDigital(EControllerDigital channel) = 0;

            virtual bool getNewDigital(EControllerDigital channel) = 0;

            virtual void rumble(std::string pattern) = 0;
        };
    }
}
#endif