#ifndef __LVGL_MENU_HPP__
#define __LVGL_MENU_HPP__

#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include "Option.hpp"
#include "liblvgl/lvgl.h"

/// @brief Namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief Namespace for menu handling
/// @author Matthew Backman
namespace menu {

/// @brief Handles events for the start button
/// @param event __lv_event_t*__ The event
extern void startButtonEventHandler(lv_event_t *event);

/// @brief Handles events for the settings button
/// @param event __lv_event_t*__ The event
extern void settingsButtonEventHandler(lv_event_t *event);

/// @brief Handles events for the back button in the settings menu
/// @param event __lv_event_t*__ The event
extern void settingsBackButtonEventHandler(lv_event_t *event);

/// @brief Handles events for the button matrix in the settings menu
/// @param event __lv_event_t*__ The event
extern void settingsButtonMatrixEventHandler(lv_event_t *event);

/// @brief Class to control the menu display through lvgl
class LvglMenu {
 private:
  static constexpr char CONFIG_FILE[]{"/usd/system/menu_data.txt"};

  static constexpr int COLUMN_WIDTH{16};

  static constexpr int BUTTONS_PER_LINE{2};

  static lv_style_t button_default_style;

  static lv_style_t button_pressed_style;

  static lv_style_t container_default_style;

  static lv_style_t container_pressed_style;

  static lv_style_t button_matrix_main_style;

  static lv_style_t button_matrix_items_style;

  static bool style_initialized;

  std::vector<Option> options{};

  bool complete{false};

  /// @brief Initializes the styles used by the display
  static void initStyles();

 public:
  /// @brief Adds an option to the settings display
  /// @param option __Option__ The option to add
  void addOption(Option option);

  /// @brief Removes an option from the settings display
  /// @param option_name __const std::string&__ The name of the option being
  /// removed
  void removeOption(const std::string &option_name);

  /// @brief Draws the main menu
  void drawMainMenu();

  /// @brief Draws the settings menu
  void drawSettingsMenu();

  /// @brief Sets the "selection complete" flag to true
  void setComplete();

  /// @brief Reads the configuration from the memory and the selections from the
  /// user
  void readConfiguration();

  /// @brief Writes the current configuration to the memory
  void writeConfiguration();

  /// @brief Displays the menu on the brain
  void displayMenu();

  /// @brief Determines if the settings selection is complete
  /// @return __bool__ True if complete, else false
  bool selectionComplete();

  /// @brief Gets the selection for a given option
  /// @param option_name __const std::string&__ The name of the option
  /// @return __std::string__ The selected state of the option
  std::string getSelection(const std::string &option_name);
};
}  // namespace menu
}  // namespace driftless
#endif