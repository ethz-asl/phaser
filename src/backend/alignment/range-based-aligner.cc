#include "packlo/backend/alignment/range-based-aligner.h"

#include <glog/logging.h>

namespace alignment {

void RangeBasedAligner::alignRegistered(const model::PointCloud& cloud_prev, 
		const std::vector<model::FunctionValue>& f_prev,                          
    const model::PointCloud& cloud_cur,                                           
		const std::vector<model::FunctionValue>& f_cur) {
	VLOG(1) << "reached align registered";
}

} // namespace alignment
