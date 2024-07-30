#ifndef __I_MENU_HPP__
#define __I_MENU_HPP__

namespace pvegas{
    namespace menu{
        class IMenu{
            public:
            virtual ~IMenu() = default;

            virtual void display() = 0;

            virtual bool isStarted() = 0;
        };
    }
}
#endif