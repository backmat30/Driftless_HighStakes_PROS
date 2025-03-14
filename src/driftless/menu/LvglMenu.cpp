#include "driftless/menu/LvglMenu.hpp"

namespace driftless {
namespace menu {
lv_style_t LvglMenu::button_default_style;
lv_style_t LvglMenu::button_pressed_style;
lv_style_t LvglMenu::container_default_style;
lv_style_t LvglMenu::container_pressed_style;
lv_style_t LvglMenu::button_matrix_main_style;
lv_style_t LvglMenu::button_matrix_items_style;
bool LvglMenu::style_initialized = false;

void startButtonEventHandler(lv_event_t* event) {
  // find user data from liblvgl
  void** user_data{static_cast<void**>(lv_event_get_user_data(event))};
  // get the data for the menu from the user data
  LvglMenu* lvgl_menu{static_cast<LvglMenu*>(user_data[0])};

  // clears the screen before drawing
  lv_obj_clean(lv_scr_act());
  // writes the config and send flag that config is setup
  if (lvgl_menu) {
    lvgl_menu->writeConfiguration();
    lvgl_menu->setComplete();
  }
}

void settingsButtonEventHandler(lv_event_t* event) {
  // find user data from liblvgl
  void** user_data{static_cast<void**>(lv_event_get_user_data(event))};
  // get the data for the menu from the user data
  LvglMenu* lvgl_menu{static_cast<LvglMenu*>(user_data[0])};

  // clear screen and draw new menu
  lv_obj_clean(lv_scr_act());
  if (lvgl_menu) {
    lvgl_menu->drawSettingsMenu();
  }
}

void settingsBackButtonEventHandler(lv_event_t* event) {
  // find user data from liblvgl
  void** user_data{static_cast<void**>(lv_event_get_user_data(event))};
  // get the data for the main menu from the user data
  LvglMenu* lvgl_menu{static_cast<LvglMenu*>(user_data[1])};
  // get the current menu
  lv_obj_t* menu{static_cast<lv_obj_t*>(user_data[0])};
  // get the target object
  lv_obj_t* obj{lv_event_get_target(event)};

  // checks if the current target object is the back button
  if (obj == lv_menu_get_sidebar_header_back_btn(menu)) {
    lv_obj_clean(lv_scr_act());
    if (lvgl_menu) {
      lvgl_menu->drawMainMenu();
    }
  }
}

void settingsButtonMatrixEventHandler(lv_event_t* event) {
  // get the target object
  lv_obj_t* obj{lv_event_get_target(event)};
  // get the id of the pressed button
  uint32_t button_id{lv_btnmatrix_get_selected_btn(obj)};
  // get the option tied to the current menu
  Option* option{static_cast<Option*>(lv_event_get_user_data(event))};
  // set the option to the selected version
  option->selected = button_id;
}

void LvglMenu::initStyles() {
  // skip init if already done
  if (style_initialized) {
    return;
  }

  /********************
   USES WISCO'S STYLES
  ********************/

  // Create the default button style
  lv_style_init(&button_default_style);
  lv_style_set_radius(&button_default_style, 5);
  lv_style_set_bg_opa(&button_default_style, LV_OPA_100);
  lv_style_set_bg_color(&button_default_style, lv_color_make(192, 192, 192));
  lv_style_set_bg_grad_color(&button_default_style,
                             lv_color_darken(lv_color_make(192, 192, 192), 8));
  lv_style_set_border_opa(&button_default_style, LV_OPA_100);
  lv_style_set_border_width(&button_default_style, 2);
  lv_style_set_border_color(&button_default_style, lv_color_black());
  lv_style_set_text_color(&button_default_style, lv_color_black());
  lv_style_set_text_font(&button_default_style, &lv_font_montserrat_20);

  // Create the pressed button style
  lv_style_init(&button_pressed_style);
  lv_style_set_radius(&button_pressed_style, 5);
  lv_style_set_bg_opa(&button_pressed_style, LV_OPA_100);
  lv_style_set_translate_y(&button_pressed_style, 3);
  lv_style_set_shadow_ofs_y(&button_pressed_style, 3);
  lv_style_set_bg_color(&button_pressed_style,
                        lv_color_darken(lv_color_make(192, 192, 192), 16));
  lv_style_set_bg_grad_color(&button_pressed_style,
                             lv_color_darken(lv_color_make(192, 192, 192), 24));
  lv_style_set_border_opa(&button_pressed_style, LV_OPA_100);
  lv_style_set_border_width(&button_pressed_style, 2);
  lv_style_set_border_color(&button_pressed_style, lv_color_black());
  lv_style_set_text_color(&button_pressed_style, lv_color_black());
  lv_style_set_text_font(&button_pressed_style, &lv_font_montserrat_20);

  // Create the default container style
  lv_style_init(&container_default_style);
  lv_style_set_radius(&container_default_style, 0);
  lv_style_set_bg_opa(&container_default_style, LV_OPA_100);
  lv_style_set_bg_color(&container_default_style, lv_color_make(0, 104, 179));
  lv_style_set_border_width(&container_pressed_style, 0);
  lv_style_set_text_color(&container_default_style, lv_color_white());
  lv_style_set_text_align(&container_default_style, LV_TEXT_ALIGN_CENTER);
  lv_style_set_pad_ver(&container_default_style, 10);

  // Create the pressed container style
  lv_style_init(&container_pressed_style);
  lv_style_set_radius(&container_pressed_style, 0);
  lv_style_set_bg_opa(&container_pressed_style, LV_OPA_100);
  lv_style_set_bg_color(&container_pressed_style, lv_color_make(244, 115, 33));
  lv_style_set_border_width(&container_pressed_style, 0);
  lv_style_set_text_color(&container_pressed_style, lv_color_black());
  lv_style_set_text_align(&container_pressed_style, LV_TEXT_ALIGN_CENTER);
  lv_style_set_pad_ver(&container_default_style, 10);

  // Create the button matrix main style
  lv_style_init(&button_matrix_main_style);
  lv_style_set_bg_color(&button_matrix_main_style,
                        lv_color_make(173, 205, 234));
  lv_style_set_border_width(&button_matrix_main_style, 0);

  // Create the button matrix items style
  lv_style_init(&button_matrix_items_style);

  // Set the style initialization flag
  style_initialized = true;
}

void LvglMenu::addOption(Option option) {
  // pushes the new option to the back of the vector
  options.push_back(option);
}

void LvglMenu::removeOption(const std::string& option_name) {
  // iterate through options
  for (auto option{options.begin()}; option != options.end(); ++option) {
    // find specified option and remove it
    if (option_name == option->name) {
      options.erase(option);
      break;
    }
  }
}

void LvglMenu::drawMainMenu() {
  /********************
    STYLE BY WISCOBOTS
   ********************/
  // Set the background color to light blue
  lv_obj_set_style_bg_color(lv_scr_act(), lv_color_make(173, 205, 234), 0);
  lv_obj_refresh_style(lv_scr_act(), LV_PART_MAIN, LV_STYLE_BG_COLOR);

  // Create the big line at the bottom
  static lv_point_t big_line_points[] = {{0, 205}, {480, 205}};
  static lv_style_t big_line_style;
  lv_style_init(&big_line_style);
  lv_style_set_line_width(&big_line_style, 55);
  lv_style_set_line_color(&big_line_style, lv_color_make(0, 104, 179));
  lv_style_set_line_rounded(&big_line_style, false);
  lv_obj_t* big_line = lv_line_create(lv_scr_act());
  lv_line_set_points(big_line, big_line_points, 2);
  lv_obj_add_style(big_line, &big_line_style, 0);

  // Create the stripe on the line at the bottom
  static lv_point_t stripe_line_points[] = {{0, 220}, {480, 220}};
  static lv_style_t stripe_line_style;
  lv_style_init(&stripe_line_style);
  lv_style_set_line_width(&stripe_line_style, 13);
  lv_style_set_line_color(&stripe_line_style, lv_color_make(244, 115, 33));
  lv_style_set_line_rounded(&stripe_line_style, false);
  lv_obj_t* stripe_line = lv_line_create(lv_scr_act());
  lv_line_set_points(stripe_line, stripe_line_points, 2);
  lv_obj_add_style(stripe_line, &stripe_line_style, 0);

  // Create the left diagonal line
  static lv_point_t left_diagonal_line_points[] = {{320, 190}, {510, 0}};
  static lv_style_t left_diagonal_line_style;
  lv_style_init(&left_diagonal_line_style);
  lv_style_set_line_width(&left_diagonal_line_style, 23);
  lv_style_set_line_color(&left_diagonal_line_style,
                          lv_color_make(0, 104, 179));
  lv_style_set_line_rounded(&left_diagonal_line_style, false);
  lv_obj_t* left_diagonal_line = lv_line_create(lv_scr_act());
  lv_line_set_points(left_diagonal_line, left_diagonal_line_points, 2);
  lv_obj_add_style(left_diagonal_line, &left_diagonal_line_style, 0);

  // Create the right diagonal line
  static lv_point_t right_diagonal_line_points[] = {{370, 190}, {560, 0}};
  static lv_style_t right_diagonal_line_style;
  lv_style_init(&right_diagonal_line_style);
  lv_style_set_line_width(&right_diagonal_line_style, 23);
  lv_style_set_line_color(&right_diagonal_line_style,
                          lv_color_make(0, 104, 179));
  lv_style_set_line_rounded(&right_diagonal_line_style, false);
  lv_obj_t* right_diagonal_line = lv_line_create(lv_scr_act());
  lv_line_set_points(right_diagonal_line, right_diagonal_line_points, 2);
  lv_obj_add_style(right_diagonal_line, &right_diagonal_line_style, 0);

  // Add the team name text
  static lv_style_t team_name_label_style;
  lv_style_init(&team_name_label_style);
  lv_style_set_text_font(&team_name_label_style, &pros_font_dejavu_mono_30);
  lv_style_set_text_color(&team_name_label_style, lv_color_make(244, 115, 33));
  lv_obj_t* team_name_label = lv_label_create(lv_scr_act());
  lv_obj_add_style(team_name_label, &team_name_label_style, 0);
  lv_label_set_text(team_name_label, "Driftless");
  lv_obj_align(team_name_label, LV_ALIGN_BOTTOM_MID, 0, -26);

  // Add the status label
  static lv_style_t status_label_style;
  lv_style_init(&status_label_style);
  lv_style_set_border_width(&status_label_style, 2);
  lv_style_set_pad_all(&status_label_style, 3);
  lv_style_set_border_color(&status_label_style, lv_color_make(0, 104, 179));
  lv_style_set_text_color(&status_label_style, lv_color_black());
  lv_obj_t* status_label = lv_label_create(lv_scr_act());
  lv_obj_add_style(status_label, &status_label_style, 0);
  std::string status_text{};
  for (Option& option : options) {
    if (option.name != options.front().name) status_text += '\n';
    status_text += option.name;
    status_text += ":";
    for (uint8_t i{0}; i < COLUMN_WIDTH - option.name.length() - 1; ++i)
      status_text += " ";
    status_text += option.choices[option.selected];
  }
  lv_label_set_text_fmt(status_label, "%s", status_text.c_str());
  lv_obj_align(status_label, LV_ALIGN_TOP_LEFT, 20, 100);

  // Add the start button
  lv_obj_t* start_button = lv_btn_create(lv_scr_act());
  lv_obj_remove_style_all(start_button);
  lv_obj_add_style(start_button, &button_default_style, 0);
  lv_obj_add_style(start_button, &button_pressed_style, LV_STATE_PRESSED);
  lv_obj_set_size(start_button, 160, 70);
  lv_obj_align(start_button, LV_ALIGN_TOP_LEFT, 20, 15);
  static void* start_user_data[]{this};
  lv_obj_add_event_cb(start_button, startButtonEventHandler, LV_EVENT_CLICKED,
                      start_user_data);
  lv_obj_t* start_button_label = lv_label_create(start_button);
  lv_label_set_text(start_button_label, "START");
  lv_obj_center(start_button_label);

  // Add the settings button
  lv_obj_t* settings_button = lv_btn_create(lv_scr_act());
  lv_obj_remove_style_all(settings_button);
  lv_obj_add_style(settings_button, &button_default_style, 0);
  lv_obj_add_style(settings_button, &button_pressed_style, LV_STATE_PRESSED);
  lv_obj_set_size(settings_button, 70, 70);
  lv_obj_align(settings_button, LV_ALIGN_TOP_LEFT, 190, 15);
  static void* settings_user_data[]{this};
  lv_obj_add_event_cb(settings_button, settingsButtonEventHandler,
                      LV_EVENT_CLICKED, settings_user_data);
  lv_obj_t* settings_button_label = lv_label_create(settings_button);
  lv_label_set_text(settings_button_label, LV_SYMBOL_SETTINGS);
  lv_obj_center(settings_button_label);
}

void LvglMenu::drawSettingsMenu() {
  /********************
    STYLE BY WISCOBOTS
   ********************/

  // Create the menu
  lv_obj_t* menu{lv_menu_create(lv_scr_act())};
  lv_menu_set_mode_root_back_btn(menu, LV_MENU_ROOT_BACK_BTN_ENABLED);
  lv_obj_set_style_bg_color(menu, lv_color_make(0, 104, 179), 0);
  lv_obj_set_size(menu, lv_disp_get_hor_res(NULL), lv_disp_get_ver_res(NULL));
  lv_obj_center(menu);

  // Create a root page
  lv_obj_t* root_page{lv_menu_page_create(menu, NULL)};
  lv_obj_set_style_pad_hor(
      root_page, lv_obj_get_style_pad_left(lv_menu_get_main_header(menu), 0),
      0);
  lv_obj_t* section{lv_menu_section_create(root_page)};
  lv_menu_set_sidebar_page(menu, root_page);

  // Create the back button
  lv_obj_t* back_btn{lv_menu_get_sidebar_header_back_btn(menu)};
  lv_obj_remove_style_all(back_btn);
  lv_obj_add_style(back_btn, &button_default_style, 0);
  lv_obj_add_style(back_btn, &button_pressed_style, LV_STATE_PRESSED);
  lv_obj_set_style_text_font(back_btn, &lv_font_montserrat_14, 0);
  lv_obj_set_style_text_font(back_btn, &lv_font_montserrat_14,
                             LV_STATE_PRESSED);
  lv_obj_set_style_pad_all(back_btn, 3, 0);
  lv_obj_set_style_pad_all(back_btn, 3, LV_STATE_PRESSED);
  lv_obj_set_style_translate_y(back_btn, 0, LV_STATE_PRESSED);
  lv_obj_set_style_shadow_ofs_y(back_btn, 0, LV_STATE_PRESSED);
  lv_obj_t* back_btn_label{lv_label_create(back_btn)};
  lv_label_set_text(back_btn_label, "   Back");
  static void* back_user_data[]{nullptr, nullptr};
  back_user_data[0] = menu;
  back_user_data[1] = this;
  lv_obj_add_event_cb(menu, settingsBackButtonEventHandler, LV_EVENT_CLICKED,
                      back_user_data);

  // list of button maps tied to each option
  static std::vector<std::shared_ptr<std::vector<const char*>>>
      option_button_matrix_maps{};
  // list of button data
  static std::vector<void*> option_button_matrix_user_data{};
  for (Option& option : options) {
    // creates a new child page of the main menu for the selected option
    lv_obj_t* option_page{lv_menu_page_create(menu, NULL)};
    lv_obj_set_style_pad_hor(
        option_page,
        lv_obj_get_style_pad_left(lv_menu_get_main_header(menu), 0), 0);
    lv_menu_separator_create(option_page);

    // creates a button map for the choices tied to the current option
    std::shared_ptr<std::vector<const char*>> option_button_matrix_map{
        std::make_shared<std::vector<const char*>>()};
    uint8_t line_counter{};
    for (std::string& choice : option.choices) {
      // pushes buttons to the next line if they run past the allotted space
      if (line_counter >= BUTTONS_PER_LINE) {
        option_button_matrix_map->push_back("\n");
        line_counter = 0;
      }
      // add the current choice to the buttons
      option_button_matrix_map->push_back(choice.c_str());
      ++line_counter;
    }
    // adds terminator
    option_button_matrix_map->push_back("");
    // adds the button map to the list of button maps for various options
    option_button_matrix_maps.push_back(option_button_matrix_map);

    // creates the button matrix using data gathered
    lv_obj_t* option_button_matrix{lv_btnmatrix_create(option_page)};
    lv_btnmatrix_set_map(option_button_matrix,
                         option_button_matrix_map->data());
    lv_btnmatrix_set_one_checked(option_button_matrix, true);
    lv_btnmatrix_set_btn_ctrl_all(option_button_matrix,
                                  LV_BTNMATRIX_CTRL_CHECKABLE);
    lv_btnmatrix_set_btn_ctrl(option_button_matrix, option.selected,
                              LV_BTNMATRIX_CTRL_CHECKED);
    lv_obj_add_style(option_button_matrix, &button_matrix_main_style,
                     LV_PART_MAIN);
    lv_obj_set_size(option_button_matrix, 300, 220);

    // gathers user data for the current option
    void* option_user_data[]{nullptr};
    option_user_data[0] = &option;
    option_button_matrix_user_data.push_back(option_user_data);
    lv_obj_add_event_cb(option_button_matrix, settingsButtonMatrixEventHandler,
                        LV_EVENT_VALUE_CHANGED, &option);

    // creates the page for the option
    lv_obj_t* option_menu_container{lv_menu_cont_create(section)};
    lv_obj_remove_style_all(option_menu_container);
    lv_obj_add_style(option_menu_container, &container_default_style, 0);
    lv_obj_add_style(option_menu_container, &container_pressed_style,
                     LV_STATE_CHECKED);
    lv_obj_t* option_menu_container_label{
        lv_label_create(option_menu_container)};
    lv_label_set_text(option_menu_container_label, option.name.c_str());
    lv_menu_set_load_page_event(menu, option_menu_container, option_page);
  }

  lv_event_send(lv_obj_get_child(
                    lv_obj_get_child(lv_menu_get_cur_sidebar_page(menu), 0), 0),
                LV_EVENT_CLICKED, NULL);
}

void LvglMenu::setComplete() { complete = true; }

void LvglMenu::readConfiguration() {
  // finds the config file to use, exits if none is found
  std::ifstream config_file{CONFIG_FILE};
  if (config_file.fail()) {
    return;
  }

  std::string option_name{};
  // reads data directly to option name each iteration of the loop
  while (config_file >> option_name) {
    std::string option_selection{};
    // reads the next item as the selected choice for the selected option
    if (config_file >> option_selection) {
      // finds the desired option
      for (Option& option : options) {
        if (option_name == option.name) {
          // finds the desired choice for the option and selects it
          for (uint8_t i{0}; i < option.choices.size(); ++i) {
            if (option_selection == option.choices[i]) {
              option.selected = i;
            }
          }
        }
      }
    }
  }
  // closes the file
  config_file.close();
}

void LvglMenu::writeConfiguration() {
  // finds the file to write to, exits if not found
  std::ofstream config_file{CONFIG_FILE};
  if (config_file.fail()) {
    return;
  }

  // writes the name and selected state for each option in the config file
  for (Option option : options) {
    config_file << option.name << ' ' << option.choices[option.selected]
                << std::endl;
  }

  // closes the file
  config_file.close();
}

void LvglMenu::displayMenu() {
  initStyles();
  readConfiguration();
  drawMainMenu();
}

bool LvglMenu::selectionComplete() { return complete; }

std::string LvglMenu::getSelection(const std::string& option_name) {
  // string to hold state of given option
  std::string selection{};
  // find option and get current state
  for (Option& option : options) {
    if (option_name == option.name) {
      selection = option.choices[option.selected];
    }
  }
  return selection;
}
}  // namespace menu
}  // namespace driftless