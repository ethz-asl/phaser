#include "packlo/model/function-value.h"

#include <numeric>
#include <algorithm>

namespace model {

FunctionValue::FunctionValue()
	: interpolation_(0.0), range_(0.0), intensity_(0.0) {}

FunctionValue::FunctionValue(double interpolation, 
		double range, double intensity) 
	: interpolation_(interpolation), 
		range_(range), intensity_(intensity), 
	  points_(new common::PointCloud_t) {
	
}

double FunctionValue::getAveragedInterpolation() const noexcept {
	return std::accumulate(interpolation_.cbegin(), interpolation_.cend(), 
			0.0) / static_cast<double>(interpolation_.size());
}

double FunctionValue::getAveragedRange() const noexcept {
	return std::accumulate(range_.cbegin(), range_.cend(), 
			0.0) / static_cast<double>(range_.size());
}

double FunctionValue::getAveragedIntensity() const noexcept {
	return std::accumulate(intensity_.cbegin(), intensity_.cend(), 
			0.0) / static_cast<double>(intensity_.size());
}

common::Point_t FunctionValue::getAveragedPoint() const noexcept {
	common::Point_t avg;
	const auto& points = points_->points;
	avg = std::accumulate(points.cbegin(), points.cend(), 
			common::Point_t(),
			[] (const common::Point_t& acc, const common::Point_t& cur) {
				common::Point_t res; 
				res.x = acc.x + cur.x;
				res.y = acc.y + cur.y;
				res.z = acc.z + cur.z;
				return res;
			});
	avg.x = avg.x / points.size();
	avg.y = avg.y / points.size();
	avg.z = avg.z / points.size();
	return avg;
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

void FunctionValue::addPoint(const common::Point_t& point) {
	points_->push_back(point);
}

common::PointCloud_tPtr FunctionValue::getAllPoints() const {
	return points_;
}

} // namespace model
