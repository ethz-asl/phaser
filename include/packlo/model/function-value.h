#pragma once

#include <vector>

namespace model {

class FunctionValue {
	public:
		FunctionValue();
		explicit FunctionValue(double interpolation, 
				double range, double intensity);

		double getInterpolation() const noexcept;
		double getRange() const noexcept;
		double getIntensity() const noexcept;

		void addInterpolation(const double interpolation);
		void addRange(const double range);
		void addIntensity(const double intensity);

	private:
		std::vector<double> interpolation_;
		std::vector<double> range_; 
		std::vector<double> intensity_;
};

} // namespace model
