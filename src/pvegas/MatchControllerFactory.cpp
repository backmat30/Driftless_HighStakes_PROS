#include "pvegas/MatchControllerFactory.hpp"
#include "pvegas/MatchController.hpp"
#include "pvegas/config/DefaultConfig.hpp"
#include "pvegas/menu/IMenu.hpp"
#include "pvegas/menu/MenuAdapter.hpp"
#include "pvegas/profiles/DefaultProfile.hpp"
#include "pvegas/profiles/IProfile.hpp"
#include "pvegas/pros_adapters/ProsClock.hpp"
#include "pvegas/rtos/IClock.hpp"
#include <memory>

namespace pvegas {
pvegas::MatchController MatchControllerFactory::createMatchController() {

  // the display menu
  std::unique_ptr<menu::IMenu> lvgl_menu{std::make_unique<menu::MenuAdapter>()};
  // add alliances

  // add auton routes

  // add configs
  std::unique_ptr<config::IConfig> default_config{std::make_unique<config::DefaultConfig>()};
  lvgl_menu->addConfig(default_config);

  // add profiles
  std::unique_ptr<profiles::IProfile> default_profile{std::make_unique<profiles::DefaultProfile>()};
  lvgl_menu->addProfile(default_profile);

  // create RTOS
  std::shared_ptr<rtos::IClock> clock{std::make_unique<pros_adapters::ProsClock>()};
  std::unique_ptr<rtos::IDelayer> delayer{std::make_unique<pros_adapters::ProsDelayer>()};

  //create and send out the match controller
  return MatchController{lvgl_menu, clock, delayer};
}
} // namespace pvegas