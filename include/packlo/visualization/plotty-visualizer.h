#pragma once 

#include "packlo/common/statistics-manager.h"

namespace visualization {

class PlottyVisualizer {
	public:
		explicit PlottyVisualizer(const PlottyVisualizer& viz) = delete;

		void createPlotFor(const common::StatisticsManager& manager, 
				std::string&& key);
		
		// Singleton instance
     static inline void init() {
       PlottyVisualizer::getInstance();
     }
     static inline PlottyVisualizer& getInstance() {
       static PlottyVisualizer instance;
       return instance;
     }

	
	private:
		PlottyVisualizer();
};
} // namespace visualization
