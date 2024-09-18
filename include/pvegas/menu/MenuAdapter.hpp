#ifndef __MENU_ADAPTER_HPP__
#define __MENU_ADAPTER_HPP__

#include "IMenu.hpp"
#include "LvglMenu.hpp"
#include "pvegas/SystemConfig.hpp"
#include "pvegas/config/IConfig.hpp"
#include "pvegas/profiles/IProfile.hpp"
#include <memory>
#include <vector>

namespace pvegas {
namespace menu {
class MenuAdapter : public IMenu {
private:
  // name used for the config settings
  static constexpr char CONFIG_OPTION_NAME[]{"CONFIG"};

  // name used for the profile settings
  static constexpr char PROFILE_OPTION_NAME[]{"PROFILE"};

  // Available configs
  std::vector<std::unique_ptr<config::IConfig>> configs{};

  // Available profiles
  std::vector<std::unique_ptr<profiles::IProfile>> profiles{};

  // display
  LvglMenu lvgl_menu{};

public:
  //add a config to the selection
  void addConfig(std::unique_ptr<config::IConfig>& config) override;

  //add a profile to the selection
  void addProfile(std::unique_ptr<profiles::IProfile>& profile) override;

  // display menu
  void display() override;

  // checks if the menu is running
  bool isStarted() override;

  //gets the systems config settings
  SystemConfig getSystemConfig(bool read_only = false) override;
};
} // namespace menu
} // namespace pvegas
#endif