#pragma once

#include "packlo/common/point-types.h"


#include <Eigen/Dense>
#include <vector>

namespace model {

class FunctionValue {
	public:
		FunctionValue();
		explicit FunctionValue(double interpolation, 
				double range, double intensity);

		double getAveragedInterpolation() const noexcept;
		double getAveragedRange() const noexcept;
		double getAveragedIntensity() const noexcept;
		common::Point_t getAveragedPoint() const noexcept;

		void addInterpolation(const double interpolation);
		void addRange(const double range);
		void addIntensity(const double intensity);
		void addPoint(const common::Point_t& point);

		common::PointCloud_tPtr getAllPoints() const;

	private:
		std::vector<double> interpolation_;
		std::vector<double> range_; 
		std::vector<double> intensity_;
		common::PointCloud_tPtr points_;
};

} // namespace model
