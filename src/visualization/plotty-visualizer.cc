#include "packlo/visualization/plotty-visualizer.h"
#include <plotty/matplotlibcpp.hpp>

#include <Eigen/Dense>
#include <glog/logging.h>

namespace visualization {

PlottyVisualizer::PlottyVisualizer(){

}

void PlottyVisualizer::createPlotFor(const common::StatisticsManager& manager, 
    std::string&& key) {
  std::vector<double> data = manager.getValuesForKey(key);
  if (data.empty()) return;
  plotty::plot(data);
  plotty::show();
}

} // namespace visualization
