#ifndef __MENU_ADAPTER_HPP__
#define __MENU_ADAPTER_HPP__

#include "IMenu.hpp"
#include "LvglMenu.hpp"

namespace pvegas {
namespace menu {
class MenuAdapter : public IMenu {
private:
  LvglMenu lvgl_menu{};

public:
  void display() override;

  bool isStarted() override;
};
} // namespace menu
} // namespace pvegas
#endif