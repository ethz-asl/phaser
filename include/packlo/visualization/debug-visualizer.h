#pragma once

#include <packlo/model/point-cloud.h>

#include <string>
#include <vector>

namespace visualization {

class DebugVisualizer {
  public:
    explicit DebugVisualizer(const DebugVisualizer& viz) = delete;

    void writeFunctionValuesToFile(std::string &&file_name, 
        const std::vector<float> &function_values);
    void writePcdFile(std::string &&file_name, const model::PointCloud &cloud);

    void visualizePointCloud(const model::PointCloud& cloud);
    void visualizePointCloudDiff(const model::PointCloud& cloud1, 
        const model::PointCloud& cloud2);

    // Singleton instance
    static inline void init() {
      DebugVisualizer::getInstance();
    }
    static inline DebugVisualizer& getInstance() {
      static DebugVisualizer instance;
      return instance;
    }

  private:
    DebugVisualizer() {}

};
}
