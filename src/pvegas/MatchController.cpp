#include "pvegas/MatchController.hpp"
namespace pvegas {
MatchController::MatchController(std::unique_ptr<menu::IMenu> &new_menu,
                                 std::shared_ptr<rtos::IClock> &clock,
                                 std::unique_ptr<rtos::IDelayer> &delayer)
    : m_menu{std::move(new_menu)}, m_clock{clock},
      m_delayer{std::move(delayer)} {}
} // namespace pvegas