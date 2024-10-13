#ifndef __SYSTEM_CONFIG_HPP__
#define __SYSTEM_CONFIG_HPP__

#include <memory>

#include "config/IConfig.hpp"
#include "profiles/IProfile.hpp"
namespace pvegas {
struct SystemConfig {
  std::unique_ptr<config::IConfig> config{};

  std::unique_ptr<profiles::IProfile> profile{};
};
}  // namespace pvegas
#endif