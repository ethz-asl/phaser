#ifndef PHASER_COMMON_BASE_WORKER_H_
#define PHASER_COMMON_BASE_WORKER_H_

namespace common {

class BaseWorker {
 public:
  virtual void run() = 0;
  bool isCompleted() const;

 protected:
  bool is_completed_ = false;
};

}  // namespace common

#endif  // PHASER_COMMON_BASE_WORKER_H_
