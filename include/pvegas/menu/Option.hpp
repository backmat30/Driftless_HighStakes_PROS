#ifndef __OPTION_HPP__
#define __OPTION_HPP__

#include <string>
#include <vector>
namespace pvegas{
    namespace menu{
        struct Option{
            std::string name{};

            std::vector<std::string> choices{};

            int selected{};
        };
    }
}
#endif