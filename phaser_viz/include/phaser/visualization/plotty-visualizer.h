#ifndef PHASER_VISUALIZATION_PLOTTY_VISUALIZER_H_
#define PHASER_VISUALIZATION_PLOTTY_VISUALIZER_H_

#include <string>
#include <vector>

#include "phaser/common/statistics-manager.h"

namespace visualization {

class PlottyVisualizer {
 public:
  explicit PlottyVisualizer(const PlottyVisualizer& viz) = delete;

  void createPlotFor(
      const common::StatisticsManager& manager, std::string&& key);
  PlottyVisualizer& createPlotFor(const std::vector<double>& data);

  void storeToFile(const std::vector<double>& data);
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

#endif  // PHASER_VISUALIZATION_PLOTTY_VISUALIZER_H_
