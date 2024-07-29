#ifndef __PROS_DELAYER_HPP__
#define __PROS_DELAYER_HPP__

#include "pros/rtos.hpp"
#include <cstdint>
#include <memory>

namespace pvegas{
    namespace pros_adapters{
        class ProsDelayer{
            public:
            std::unique_ptr<ProsDelayer> clone() const;

            void delay(uint32_t millis);

            void delayUntil(uint32_t time);
        };
    }
}
#endif