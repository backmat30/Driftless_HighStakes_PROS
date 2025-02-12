#include "driftless/MatchControllerFactory.hpp"

namespace driftless {
driftless::MatchController MatchControllerFactory::createMatchController() {
  // the display menu
  std::unique_ptr<menu::IMenu> lvgl_menu{std::make_unique<menu::MenuAdapter>()};
  // add alliances
  std::shared_ptr<alliance::IAlliance> blue_alliance{
      std::make_shared<alliance::BlueAlliance>()};
  lvgl_menu->addAlliance(blue_alliance);

  std::shared_ptr<alliance::IAlliance> red_alliance{
      std::make_shared<alliance::RedAlliance>()};
  lvgl_menu->addAlliance(red_alliance);

  // add auton routes
  std::unique_ptr<auton::IAuton> blue_rush_auton{
      std::make_unique<auton::BlueRushAuton>()};
  lvgl_menu->addAuton(blue_rush_auton);

  std::unique_ptr<auton::IAuton> blue_skills_auton{
      std::make_unique<auton::BlueSkillsAuton>()};
  lvgl_menu->addAuton(blue_skills_auton);

  std::unique_ptr<auton::IAuton> orange_rush_auton{
      std::make_unique<auton::OrangeRushAuton>()};
  lvgl_menu->addAuton(orange_rush_auton);

  std::unique_ptr<auton::IAuton> orange_skills_auton{
      std::make_unique<auton::OrangeSkillsAuton>()};
  lvgl_menu->addAuton(orange_skills_auton);

  // add configs
  std::unique_ptr<config::IConfig> default_config{
      std::make_unique<config::DefaultConfig>()};
  lvgl_menu->addConfig(default_config);

  std::unique_ptr<config::IConfig> orange_config{
      std::make_unique<config::OrangeConfig>()};
  lvgl_menu->addConfig(orange_config);

  // add profiles
  std::unique_ptr<profiles::IProfile> default_profile{
      std::make_unique<profiles::DefaultProfile>()};
  lvgl_menu->addProfile(default_profile);

  std::unique_ptr<profiles::IProfile> eric_profile{
      std::make_unique<driftless::profiles::EricProfile>()};
  lvgl_menu->addProfile(eric_profile);

  std::unique_ptr<profiles::IProfile> john_profile{
      std::make_unique<driftless::profiles::JohnProfile>()};
  lvgl_menu->addProfile(john_profile);

  std::unique_ptr<profiles::IProfile> john_but_arcade{
      std::make_unique<driftless::profiles::JohnButArcade>()};
  lvgl_menu->addProfile(john_but_arcade);

  // create RTOS
  std::shared_ptr<rtos::IClock> clock{
      std::make_unique<pros_adapters::ProsClock>()};
  std::unique_ptr<rtos::IDelayer> delayer{
      std::make_unique<pros_adapters::ProsDelayer>()};

  // create and send out the match controller
  return MatchController{lvgl_menu, clock, delayer};
}
}  // namespace driftless