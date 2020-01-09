#ifndef PACKLO_VISUALIZATION_PLOTTY_VISUALIZER_H_
#define PACKLO_VISUALIZATION_PLOTTY_VISUALIZER_H_

#include "packlo/common/statistics-manager.h"
#include <string>
#include <vector>

namespace visualization {

class PlottyVisualizer {
 public:
  explicit PlottyVisualizer(const PlottyVisualizer& viz) = delete;

  void createPlotFor(
      const common::StatisticsManager& manager, std::string&& key);
  void createPlotFor(const std::vector<double>& data);

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

}  // namespace visualization

#endif  // PACKLO_VISUALIZATION_PLOTTY_VISUALIZER_H_
