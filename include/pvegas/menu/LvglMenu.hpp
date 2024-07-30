#ifndef __LVGL_MENU_HPP__
#define __LVGL_MENU_HPP__

#include "liblvgl/lvgl.h"

#include "Option.hpp"

#include <string>
#include <vector>
#include <memory>
#include <fstream>

namespace pvegas {
namespace menu {

extern void startButtonEventHandler(lv_event_t *event);

extern void settingsButtonEventHandler(lv_event_t *event);

extern void settingsBackButtonEventHandler(lv_event_t *event);

extern void settingsButtonMatrixEventHandler(lv_event_t *event);

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

  static void initStyles();

public:
  void addOtion(Option option);

  void removeOption(const std::string &option_name);

  void drawMainMenu();

  void drawSettingsMenu();

  void setComplete();

  void readConfiguration();

  void writeConfiguration();

  void displayMenu();

  bool selectionComplete();

  std::string getSelection(const std::string &option_name);
};
} // namespace menu
} // namespace pvegas
#endif