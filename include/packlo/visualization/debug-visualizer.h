#pragma once

#include <string>
#include <vector>

namespace visualization {

class DebugVisualizer {
  public:
    explicit DebugVisualizer(const DebugVisualizer& viz) = delete;

    void writeFunctionValuesToFile(std::string &&file_name, 
        const std::vector<float> &function_values);

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
