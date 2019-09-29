#pragma once

#include "packlo/model/point-cloud.h"
#include "packlo/common/statistics-manager.h"

#include <memory>

namespace alignment {

class BaseAligner {
	virtual void alignRegistered(model::PointCloudPtr cloud_prev, 
			const std::vector<model::FunctionValue>& f_prev, 
			model::PointCloudPtr cloud_cur,
			const std::vector<model::FunctionValue>& f_cur) = 0;
};

using BaseAlignerPtr = std::unique_ptr<BaseAligner>;

} // namespace alignment
