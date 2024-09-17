#ifndef __SYSTEM_CONFIG_HPP__
#define __SYSTEM_CONFIG_HPP__

#include "config/IConfig.hpp"
#include "profiles/IProfile.hpp"
#include <memory>
namespace pvegas {
struct SystemConfig {
  std::unique_ptr<config::IConfig> config{};

  std::unique_ptr<profiles::IProfile> profile{};
};
} // namespace pvegas
#endif