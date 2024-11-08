#ifndef __I_MENU_HPP__
#define __I_MENU_HPP__

#include "pvegas/SystemConfig.hpp"
#include "pvegas/config/IConfig.hpp"
#include "pvegas/profiles/IProfile.hpp"
namespace driftless {
namespace menu {
class IMenu {
 public:
  virtual ~IMenu() = default;

  virtual void display() = 0;

  virtual bool isStarted() = 0;

  virtual SystemConfig getSystemConfig(bool read_only = false) = 0;

  virtual void addConfig(std::unique_ptr<config::IConfig>& config) = 0;

  virtual void addProfile(std::unique_ptr<profiles::IProfile>& profile) = 0;
};
}  // namespace menu
}  // namespace pvegas
#endif