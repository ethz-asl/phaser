#ifndef PHASER_PRE_COMMON_BASE_COMMAND_H_
#define PHASER_PRE_COMMON_BASE_COMMAND_H_

#include <memory>

namespace preproc {

class BaseCommand {
 public:
  virtual void execute() = 0;
};

using BaseCommandPtr = std::unique_ptr<BaseCommand>;

}  // namespace preproc

#endif  // PHASER_PRE_COMMON_BASE_COMMAND_H_
