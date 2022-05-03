#include "phaser/backend/fusion/pyramid-struct.h"

#include <cmath>
#include <glog/logging.h>

namespace phaser_core {

PyramidStruct::PyramidStruct(
    const uint32_t n_coeffs, const uint8_t n_levels, const float div)
    : coefficients_per_level_(n_levels),
      lower_bound_per_level_(n_levels),
      upper_bound_per_level_(n_levels),
      initial_coefficient_size_(n_coeffs) {
  computeCoefficientsPerLevel(n_coeffs, n_levels, div);
}

void PyramidStruct::computeCoefficientsPerLevel(
    const uint32_t init_coeffs, const uint8_t n_levels, const float div) {
  for (uint8_t lvl = 0u; lvl < n_levels; ++lvl) {
    const uint32_t n_coeffs =
        lvl > 0u ? coefficients_per_level_[lvl - 1] : init_coeffs;
    const float lower_bound = n_coeffs / div;
    const uint32_t upper_bound = std::round(n_coeffs - lower_bound);

    lower_bound_per_level_[lvl] = std::round(lower_bound);
    upper_bound_per_level_[lvl] = upper_bound;
    coefficients_per_level_[lvl] = upper_bound - lower_bound_per_level_[lvl];
  }
}

std::vector<uint32_t> PyramidStruct::getCoefficients() const noexcept {
  return coefficients_per_level_;
}

std::vector<uint32_t> PyramidStruct::getLowerBounds() const noexcept {
  return lower_bound_per_level_;
}

std::vector<uint32_t> PyramidStruct::getUpperBounds() const noexcept {
  return upper_bound_per_level_;
}

uint32_t PyramidStruct::getCoefficientsForLevel(
    const uint8_t level) const noexcept {
  CHECK(level < coefficients_per_level_.size());
  return coefficients_per_level_[level];
}

uint32_t PyramidStruct::getLowerBoundForLevel(
    const uint8_t level) const noexcept {
  CHECK(level < lower_bound_per_level_.size());
  return lower_bound_per_level_[level];
}

uint32_t PyramidStruct::getUpperBoundForLevel(
    const uint8_t level) const noexcept {
  CHECK(level < upper_bound_per_level_.size());
  return upper_bound_per_level_[level];
}

uint32_t PyramidStruct::getInitialCoefficientSize() const noexcept {
  return initial_coefficient_size_;
}

}  // namespace phaser_core
