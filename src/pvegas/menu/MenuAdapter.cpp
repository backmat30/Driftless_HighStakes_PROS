#include "pvegas/menu/MenuAdapter.hpp"
#include "pvegas/SystemConfig.hpp"

namespace pvegas {
namespace menu {

void MenuAdapter::addConfig(std::unique_ptr<config::IConfig>& config){
    bool unique{true};
    // check if the config has already been added
    for(std::unique_ptr<config::IConfig>& current_config : configs){
        if(current_config->getName() == config->getName()){
            unique = false;
            break;
        }
    }
    // if the config doesn't exist, add it to the list
    if(unique){
        configs.push_back(std::move(config));
    }
}

void MenuAdapter::addProfile(std::unique_ptr<profiles::IProfile>& profile){
    bool unique{true};
    // check if the profile has already been added
    for(std::unique_ptr<profiles::IProfile>& current_profile : profiles){
        if(current_profile->getName() == profile->getName()){
            unique = false;
            break;
        }
    }

    // if the profile doesn't exist yet, add it to the list
    if(unique){
        profiles.push_back(std::move(profile));
    }
}

void MenuAdapter::display() { lvgl_menu.displayMenu(); }

bool MenuAdapter::isStarted() { return lvgl_menu.selectionComplete(); }

SystemConfig MenuAdapter::getSystemConfig(bool read_only){
    SystemConfig system_config{};
    // reads the config if input isn't needed
    if(read_only){
        lvgl_menu.readConfiguration();
    }

    // finds the config that matches the chosen config from the display
    for(auto& config : configs){
        if(lvgl_menu.getSelection(CONFIG_OPTION_NAME) == config->getName()){
            system_config.config = std::move(config);
            break;
        }
    }

    // finds the profile that matches the chosen profile from the display
    for(auto& profile : profiles){
        if(lvgl_menu.getSelection(PROFILE_OPTION_NAME) == profile->getName()){
            system_config.profile = std::move(profile);
            break;
        }
    }

    return system_config;
}
} // namespace menu
} // namespace pvegas