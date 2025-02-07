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

/// @brief Namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief Namespace for menu handling
/// @author Matthew Backman
namespace menu {

/// @brief Class to adapt the lvgl menu to the IMenu interface
/// @author Matthew Backman
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
  /// @param alliance __std::unique_ptr<alliance::IAlliance>__ Reference to the
  /// desired alliance to add
  void addAlliance(std::shared_ptr<alliance::IAlliance>& alliance) override;

  /// @brief Adds an auton to the menu
  /// @param auton __std::unique_ptr<auton::IAuton>&__ Reference to the auton
  /// being added
  void addAuton(std::unique_ptr<auton::IAuton>& auton) override;

  /// @brief Adds a config to the menu
  /// @param config __std::unique_ptr<config::IConfig>&__ Reference to the
  /// config being added
  void addConfig(std::unique_ptr<config::IConfig>& config) override;

  /// @brief Adds a profile to the menu
  /// @param profile __std::unique_ptr<profiles::IProfile>&__ Reference to the
  /// profile being added
  void addProfile(std::unique_ptr<profiles::IProfile>& profile) override;

  /// @brief Displays the menu on the brain
  void display() override;

  /// @brief Determines if the start button has been pressed
  /// @return __bool__ True if the button was pressed, false otherwise
  bool isStarted() override;

  /// @brief Gets the system configuration to use during the match
  /// @param read_only __bool__ Whether to get the system config straight from
  /// the memory, or let the user configure settings
  /// @return __SystemConfig__ The configurations to use during the match
  SystemConfig getSystemConfig(bool read_only = false) override;
};
}  // namespace menu
}  // namespace driftless
#endif