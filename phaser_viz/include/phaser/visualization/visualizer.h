#ifndef PHASER_VISUALIZATION_VISUALIZER_H_
#define PHASER_VISUALIZATION_VISUALIZER_H_

#include <memory>

namespace visualization {

class Visualizer {
 public:
  explicit Visualizer(const Visualizer& viz) = delete;

 private:
  Visualizer();
};

}  // namespace visualization

#endif  // PHASER_VISUALIZATION_VISUALIZER_H_
