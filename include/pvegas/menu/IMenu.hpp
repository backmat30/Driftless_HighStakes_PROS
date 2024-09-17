#ifndef __I_MENU_HPP__
#define __I_MENU_HPP__

#include "pvegas/SystemConfig.hpp"
namespace pvegas{
    namespace menu{
        class IMenu{
            public:
            virtual ~IMenu() = default;

            virtual void display() = 0;

            virtual bool isStarted() = 0;

            virtual SystemConfig getSystemConfig(bool read_only = false) = 0;
        };
    }
}
#endif