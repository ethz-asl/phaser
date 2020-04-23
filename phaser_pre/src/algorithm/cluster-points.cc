#include "phaser_pre/algorithm/cluster-points.h"

#include <deque>

namespace preproc {

ClusterPoints::ClusterPoints() {
  neighbors_ = {cv::Point(-1, -1), cv::Point(-1, 0), cv::Point(-1, 1),
                cv::Point(0, -1),  cv::Point(0, 0),  cv::Point(0, 1),
                cv::Point(1, -1),  cv::Point(1, 0),  cv::Point(1, 1)};
}

ClusterResult ClusterPoints::cluster(
    const cv::Mat& range_mat, const cv::Mat& signal_mat) {
  cv::Mat occ_mat = cv::Mat(
      1, settings_.N_SCAN * settings_.Horizon_SCAN, CV_32S, cv::Scalar::all(0));
  cv::Mat label_mat = cv::Mat(
      settings_.N_SCAN, settings_.Horizon_SCAN, CV_32S, cv::Scalar::all(-1));

  int label = 0;
  for (size_t i = 0u; i < settings_.N_SCAN; ++i) {
    for (size_t j = 0u; j < settings_.Horizon_SCAN; ++j) {
      const auto occ = findAndLabelSimilarPoints(
          cv::Point(j, i), ++label, range_mat, signal_mat, &label_mat);
      occ_mat.at<int>(0, label) = occ;
    }
  }
  return ClusterResult(std::move(occ_mat), std::move(label_mat));
}

uint32_t ClusterPoints::findAndLabelSimilarPoints(
    const cv::Point& p, const int label, const cv::Mat& range_mat,
    const cv::Mat& signal_mat, cv::Mat* label_mat) {
  const float range1 = range_mat.at<float>(p);
  const float signal1 = signal_mat.at<float>(p);
  uint32_t labelCounter = 0;
  if (range1 < 0.1 || signal1 < 1)
    return labelCounter;

  const float threshold = 55;
  std::deque<cv::Point> queue;
  queue.emplace_back(p);

  while (!queue.empty()) {
    auto queuePoint = queue.front();
    queue.pop_front();
    if (label_mat->at<int>(queuePoint) >= 0)
      continue;
    label_mat->at<int>(queuePoint) = label;

    for (auto& n : neighbors_) {
      auto curP = queuePoint + n;

      if (curP.x < 0 || curP.x >= settings_.Horizon_SCAN)
        continue;
      if (curP.y < 0 || curP.y >= settings_.N_SCAN)
        continue;
      if (label_mat->at<int>(curP) >= 0)
        continue;

      const float d1 =
          std::max(range_mat.at<float>(queuePoint), range_mat.at<float>(curP));
      const float d2 =
          std::min(range_mat.at<float>(queuePoint), range_mat.at<float>(curP));

      float alpha;
      if (n.x == 0)
        alpha = settings_.segmentAlphaX;
      else
        alpha = settings_.segmentAlphaY;

      const float angle =
          std::atan2(d2 * std::sin(alpha), (d1 - d2 * std::cos(alpha)));

      if (angle > settings_.segmentTheta) {
        queue.emplace_back(curP);
        ++labelCounter;
      }
      /*
        const float range2 = range_mat.at<float>(curP);
        const float signal2 = signal_mat.at<float>(curP);
        if (range2 < 0.1 || signal2 < 0.5)
          continue;

        const float diff =
            std::fabs(range1 - range2) + std::fabs(signal1 - signal2);

        if (diff >= threshold)
          continue;
        queue.emplace_back(curP);
        ++labelCounter;
        */
    }
  }
  return labelCounter;
}

}  // namespace preproc
