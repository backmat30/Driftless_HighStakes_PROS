#ifndef __MENU_ADAPTER_HPP__
#define __MENU_ADAPTER_HPP__

#include <memory>
#include <vector>

#include "LvglMenu.hpp"
#include "driftless/SystemConfig.hpp"
#include "driftless/alliance/IAlliance.hpp"
#include "driftless/config/IConfig.hpp"
#include "driftless/menu/IMenu.hpp"
#include "driftless/profiles/IProfile.hpp"

namespace driftless {
namespace menu {
class MenuAdapter : public IMenu {
 private:
  // Name used for the alliance settings
  static constexpr char ALLIANCE_OPTION_NAME[]{"ALLIANCE"};

  static constexpr char AUTON_OPTION_NAME[]{"AUTON"};

  // name used for the config settings
  static constexpr char CONFIG_OPTION_NAME[]{"CONFIG"};

  // name used for the profile settings
  static constexpr char PROFILE_OPTION_NAME[]{"PROFILE"};

  // Available alliances
  std::vector<std::shared_ptr<alliance::IAlliance>> alliances{};

  std::vector<std::unique_ptr<auton::IAuton>> autons{};

  // Available configs
  std::vector<std::unique_ptr<config::IConfig>> configs{};

  // Available profiles
  std::vector<std::unique_ptr<profiles::IProfile>> profiles{};

  // display
  LvglMenu lvgl_menu{};

 public:
  /// @brief Adds an alliance to the menu
  /// @param alliance __std::unique_ptr<IAlliance>__ Reference to the desired
  /// alliance to add
  void addAlliance(std::shared_ptr<alliance::IAlliance>& alliance) override;

  void addAuton(std::unique_ptr<auton::IAuton>& auton) override;

  // add a config to the selection
  void addConfig(std::unique_ptr<config::IConfig>& config) override;

  // add a profile to the selection
  void addProfile(std::unique_ptr<profiles::IProfile>& profile) override;

  // display menu
  void display() override;

  // checks if the menu is running
  bool isStarted() override;

  // gets the systems config settings
  SystemConfig getSystemConfig(bool read_only = false) override;
};
}  // namespace menu
}  // namespace driftless
#endif