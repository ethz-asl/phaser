#include "packlo/backend/alignment/range-based-aligner.h"

#include <glog/logging.h>

namespace alignment {

void RangeBasedAligner::alignRegistered(
    const model::PointCloud&, 
    const std::vector<model::FunctionValue>& f_prev,                          
    const model::PointCloud&,                                           
    const std::vector<model::FunctionValue>& f_cur, 
    common::Vector_t* xyz) {
  CHECK(xyz);
  const std::size_t n_values = f_prev.size();
  CHECK(n_values == f_cur.size());
  CHECK_GT(n_values, 0);
  common::Vector_t translation(0,0,0);
  for (std::size_t i = 0u; i < n_values; ++i) {
    const common::Point_t& point_pre = f_prev[i].getAveragedPoint();
    const common::Point_t& point_cur = f_cur[i].getAveragedPoint();
    translation += (common::Vector_t(point_pre.x, point_pre.y, point_pre.z)
        - common::Vector_t(point_cur.x, point_cur.y, point_cur.z));
  }
  *xyz = translation.array() / static_cast<double>(n_values); 
}

} // namespace alignment
