#include "pvegas/menu/MenuAdapter.hpp"

namespace pvegas {
namespace menu {
void MenuAdapter::display() { lvgl_menu.displayMenu(); }

bool MenuAdapter::isStarted() { return lvgl_menu.selectionComplete(); }
} // namespace menu
} // namespace pvegas