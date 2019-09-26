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
	VLOG(1) << "plot size: " << data.size();
	//plotty::plot();
	//plotty::show();
}

} // namespace visualization
