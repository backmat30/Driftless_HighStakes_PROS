#ifndef __MATCHCONTROLLER_HPP__
#define __MATCHCONTROLLER_HPP__

//includes
#include "pros/misc.hpp"
#include <memory>

namespace pvegas{
    class MatchController{
        std::shared_ptr<pros::Controller> controller();
    };
}
#endif