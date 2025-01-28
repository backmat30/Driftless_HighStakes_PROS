#include "driftless/alliance/RedAlliance.hpp"

namespace driftless {
namespace alliance {
EAlliance RedAlliance::getAlliance() { return EAlliance::RED; }

std::string RedAlliance::getName() { return "RED"; }

io::RGBValue RedAlliance::getAllianceColor() { return ALLIANCE_COLOR; }

io::RGBValue RedAlliance::getOpposingColor() { return OPPOSING_COLOR; }
}  // namespace alliance
}  // namespace driftless