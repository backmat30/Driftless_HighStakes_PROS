#ifndef __ALLIANCE_HPP__
#define __ALLIANCE_HPP__

#include <string>

namespace driftless {
namespace alliance {
struct Alliance {
  // name of the alliance
  std::string name{};

  // min and max hue for alliance ring hue
  double hue_range[2]{};
};
}  // namespace alliance
}  // namespace pvegas
#endif