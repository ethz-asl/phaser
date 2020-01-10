#include "packlo/visualization/plotty-visualizer.h"
#include <plotty/matplotlibcpp.hpp>

#include <Eigen/Dense>
#include <glog/logging.h>
#include <fstream>

namespace visualization {

PlottyVisualizer::PlottyVisualizer() {}

void PlottyVisualizer::createPlotFor(
    const common::StatisticsManager& manager, std::string&& key) {
  std::vector<double> data = manager.getValuesForKey(key);
  if (data.empty())
    return;
  plotty::plot(data);
  plotty::show();
}

void PlottyVisualizer::createPlotFor(const std::vector<double>& data) {
  if (data.empty())
    return;
  plotty::plot(data);
  plotty::show();
}

void PlottyVisualizer::storeToFile(const std::vector<double>& data) {
  std::ofstream outFile("data.txt");
  for (const double &e : data) outFile << e << "\n";
}

}  // namespace visualization
