#ifndef __RANGE_HPP__
#define __RANGE_HPP__

namespace driftless {
namespace utils {
/// @brief Template for a range of values given a max and minimum
/// @tparam t Type of data stored in the range
template <typename T>
struct Range {
  T min{};

  T max{};
};
}  // namespace utils
}  // namespace driftless
#endif