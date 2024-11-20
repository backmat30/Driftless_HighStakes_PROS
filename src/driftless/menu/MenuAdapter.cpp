#include "driftless/menu/MenuAdapter.hpp"
namespace driftless {
namespace menu {
void MenuAdapter::addAlliance(std::shared_ptr<alliance::IAlliance>& alliance) {
  bool unique{true};

  for (std::shared_ptr<alliance::IAlliance>& current_alliance : alliances) {
    if (current_alliance->getName() == alliance->getName()) {
      unique = false;
      break;
    }
  }

  if (unique) {
    alliances.push_back(std::move(alliance));
  }
}

void MenuAdapter::addAuton(std::unique_ptr<auton::IAuton>& auton) {
  bool unique{true};

  for (std::unique_ptr<auton::IAuton>& current_auton : autons) {
    if (current_auton->getName() == auton->getName()) {
      unique = false;
      break;
    }
  }

  if (unique) {
    autons.push_back(std::move(auton));
  }
}

void MenuAdapter::addConfig(std::unique_ptr<config::IConfig>& config) {
  bool unique{true};
  // check if the config has already been added
  for (std::unique_ptr<config::IConfig>& current_config : configs) {
    if (current_config->getName() == config->getName()) {
      unique = false;
      break;
    }
  }
  // if the config doesn't exist, add it to the list
  if (unique) {
    configs.push_back(std::move(config));
  }
}

void MenuAdapter::addProfile(std::unique_ptr<profiles::IProfile>& profile) {
  bool unique{true};
  // check if the profile has already been added
  for (std::unique_ptr<profiles::IProfile>& current_profile : profiles) {
    if (current_profile->getName() == profile->getName()) {
      unique = false;
      break;
    }
  }

  // if the profile doesn't exist yet, add it to the list
  if (unique) {
    profiles.push_back(std::move(profile));
  }
}

void MenuAdapter::display() {
  std::vector<std::string> alliance_options{};

  for (auto& alliance : alliances) {
    alliance_options.push_back(alliance->getName());
  }

  Option alliance_option{ALLIANCE_OPTION_NAME, alliance_options};
  lvgl_menu.addOption(alliance_option);

  std::vector<std::string> auton_options{};

  for (auto& auton : autons) {
    auton_options.push_back(auton->getName());
  }

  Option auton_option{AUTON_OPTION_NAME, auton_options};
  lvgl_menu.addOption(auton_option);

  // list of config names
  std::vector<std::string> config_options{};
  // fills list of config names
  for (auto& config : configs) {
    config_options.push_back(config->getName());
  }
  // turn the config list into an option and add it to the menu
  Option config_option{CONFIG_OPTION_NAME, config_options};
  lvgl_menu.addOption(config_option);

  std::vector<std::string> profile_options{};
  for (auto& profile : profiles) {
    profile_options.push_back(profile->getName());
  }
  Option profile_option{PROFILE_OPTION_NAME, profile_options};
  lvgl_menu.addOption(profile_option);

  lvgl_menu.displayMenu();
}

bool MenuAdapter::isStarted() { return lvgl_menu.selectionComplete(); }

SystemConfig MenuAdapter::getSystemConfig(bool read_only) {
  SystemConfig system_config{};
  // reads the config if input isn't needed
  if (read_only) {
    lvgl_menu.readConfiguration();
  }

  for (auto& alliance : alliances) {
    if (lvgl_menu.getSelection(ALLIANCE_OPTION_NAME) == alliance->getName()) {
      system_config.alliance = std::move(alliance);
    }
  }

  for (auto& auton : autons) {
    if (lvgl_menu.getSelection(AUTON_OPTION_NAME) == auton->getName()) {
      system_config.auton = std::move(auton);
    }
  }
  // finds the config that matches the chosen config from the display
  for (auto& config : configs) {
    if (lvgl_menu.getSelection(CONFIG_OPTION_NAME) == config->getName()) {
      system_config.config = std::move(config);
      break;
    }
  }

  // finds the profile that matches the chosen profile from the display
  for (auto& profile : profiles) {
    if (lvgl_menu.getSelection(PROFILE_OPTION_NAME) == profile->getName()) {
      system_config.profile = std::move(profile);
      break;
    }
  }

  return system_config;
}
}  // namespace menu
}  // namespace driftless