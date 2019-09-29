#pragma once

#include "packlo/backend/alignment/base-aligner.h"

namespace alignment {

class RangeBasedAligner : public BaseAligner {

	virtual void alignRegistered(model::PointCloudPtr cloud_prev, 
			const std::vector<model::FunctionValue>& f_prev, 
			model::PointCloudPtr cloud_cur,
			const std::vector<model::FunctionValue>& f_cur) override;
};

} // namespace alignment
