#ifndef __SYSTEM_CONFIG_HPP__
#define __SYSTEM_CONFIG_HPP__

#include <memory>

#include "driftless/config/IConfig.hpp"
#include "driftless/profiles/IProfile.hpp"
namespace driftless {
struct SystemConfig {
  std::unique_ptr<config::IConfig> config{};

  std::unique_ptr<profiles::IProfile> profile{};
};
}  // namespace driftless
#endif