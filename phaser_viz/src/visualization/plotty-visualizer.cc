#include "phaser/visualization/plotty-visualizer.h"

#include <Eigen/Dense>
#include <fstream>
#include <glog/logging.h>
#include <plotty/matplotlibcpp.hpp>

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

PlottyVisualizer& PlottyVisualizer::createPlotFor(
    const std::vector<double>& data) {
  if (data.empty())
    return *this;
  plotty::plot(data);
  plotty::show();
  return *this;
}

void PlottyVisualizer::storeToFile(const std::vector<double>& data) {
  std::ofstream outFile("data.txt");
  for (const double& e : data)
    outFile << e << "\n";
}

}  // namespace visualization
