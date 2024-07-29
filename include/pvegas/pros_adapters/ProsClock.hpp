#ifndef __PROS_CLOCK_HPP__
#define __PROS_CLOCK_HPP__

#include "pros/rtos.hpp"
#include <cstdint>
#include <memory>

namespace pvegas{
    namespace pros_adapters{
        class ProsClock{
            public:
            std::unique_ptr<ProsClock> clone() const;

            uint32_t getTime();
        };
    }
}

#endif