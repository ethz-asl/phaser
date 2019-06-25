#include <packlo/visualization/debug-visualizer.h>
#include <fstream>

namespace visualization {
  
void DebugVisualizer::writeFunctionValuesToFile(std::string &&file_name,
        const std::vector<float>& function_values) {
  std::ofstream out_file(file_name);
  if (!out_file.is_open()) return;

  for (float value : function_values) {
    out_file << value << "\n";
    out_file << "0.0" << "\n";
  }
  
  out_file.close();
}

}
