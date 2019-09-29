#include "packlo/model/function-value.h"

#include <numeric>
#include <algorithm>

namespace model {

FunctionValue::FunctionValue()
	: interpolation_(0.0), range_(0.0), intensity_(0.0) {}

FunctionValue::FunctionValue(double interpolation, 
		double range, double intensity) 
	: interpolation_(interpolation), 
		range_(range), intensity_(intensity) {

}

double FunctionValue::getInterpolation() const noexcept {
	return std::accumulate(interpolation_.cbegin(), interpolation_.cend(), 
			0.0) / static_cast<double>(interpolation_.size());
}

double FunctionValue::getRange() const noexcept {
	return std::accumulate(range_.cbegin(), range_.cend(), 
			0.0) / static_cast<double>(range_.size());
}

double FunctionValue::getIntensity() const noexcept {
	return std::accumulate(intensity_.cbegin(), intensity_.cend(), 
			0.0) / static_cast<double>(intensity_.size());
}

void FunctionValue::addInterpolation(const double interpolation) {
	interpolation_.emplace_back(interpolation);
}

void FunctionValue::addRange(const double range) {
	range_.emplace_back(range);

}

void FunctionValue::addIntensity(const double intensity) {
	intensity_.emplace_back(intensity);
}

} // namespace model
