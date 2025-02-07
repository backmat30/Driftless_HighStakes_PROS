#ifndef __I_MENU_HPP__
#define __I_MENU_HPP__

#include "driftless/SystemConfig.hpp"
#include "driftless/auton/IAuton.hpp"
#include "driftless/config/IConfig.hpp"
#include "driftless/profiles/IProfile.hpp"

/// @brief Namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief Namespace for menu handling
/// @author Matthew Backman
namespace menu {

/// @brief Interface for a menu display
/// @author Matthew Backman
class IMenu {
 public:
  /// @brief Destroys the current menu object
  virtual ~IMenu() = default;

  /// @brief Displays the menu on the brain
  virtual void display() = 0;

  /// @brief Determines if the start button has been pressed
  /// @return __bool__ True if the button was pressed, false otherwise
  virtual bool isStarted() = 0;

  /// @brief Gets the system configuration to use during the match
  /// @param read_only __bool__ Whether to get the system config straight from
  /// the memory, or let the user configure settings
  /// @return __SystemConfig__ The configurations to use during the match
  virtual SystemConfig getSystemConfig(bool read_only = false) = 0;

  /// @brief Adds an alliance to the menu
  /// @param alliance __std::unique_ptr<alliance::IAlliance>__ Reference to the
  /// desired alliance to add
  virtual void addAlliance(std::shared_ptr<alliance::IAlliance>& alliance) = 0;

  /// @brief Adds an auton to the menu
  /// @param auton __std::unique_ptr<auton::IAuton>&__ Reference to the auton
  /// being added
  virtual void addAuton(std::unique_ptr<auton::IAuton>& auton) = 0;

  /// @brief Adds a config to the menu
  /// @param config __std::unique_ptr<config::IConfig>&__ Reference to the
  /// config being added
  virtual void addConfig(std::unique_ptr<config::IConfig>& config) = 0;

  /// @brief Adds a profile to the menu
  /// @param profile __std::unique_ptr<profiles::IProfile>&__ Reference to the
  /// profile being added
  virtual void addProfile(std::unique_ptr<profiles::IProfile>& profile) = 0;
};
}  // namespace menu
}  // namespace driftless
#endif