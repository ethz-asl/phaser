#ifndef PHASER_MODEL_POINT_H_
#define PHASER_MODEL_POINT_H_

namespace model {

class PointI {
 public:
  explicit PointI(uint16_t intensity) : intensity_(intensity) {}

  uint16_t& intensity() {
    return intensity_;
  }
  uint16_t intensity() const noexcept {
    return intensity_;
  }

 private:
  uint16_t intensity_;
};

class PointIS : public PointI {
 public:
  explicit PointIS(uint16_t intensity, uint16_t semantic)
      : PointI(intensity), semantic_(semantic) {}

  uint16_t& semantic() {
    return semantic_;
  }
  uint16_t semantic() const noexcept {
    return semantic_;
  }

 private:
  uint16_t semantic_;
};

class PointIST : public PointIS {
 public:
  explicit PointIST(uint16_t intensity, uint16_t semantic, uint16_t track)
      : PointIS(intensity, semantic), track_(track) {}

  uint16_t& track() {
    return track_;
  }
  uint16_t track() const noexcept {
    return track_;
  }

 private:
  uint16_t track_;
};

}  // namespace model

#endif  // PHASER_MODEL_POINT_H_
