#pragma once

namespace model {

template <typename T>
class PointI {
 public:
  explicit PointI(T intensity) : intensity_(intensity) {}

  T& intensity() {
    return intensity_;
  }
  T intensity() const noexcept {
    return intensity_;
  }

 private:
  T intensity_;
};

template <typename T>
class PointIS : public PointI<T> {
 public:
  explicit PointIS(T intensity, uint16_t semantic)
      : PointI<T>(intensity), semantic_(semantic) {}

  uint16_t& semantic() {
    return semantic_;
  }
  uint16_t semantic() const noexcept {
    return semantic_;
  }

 private:
  uint16_t semantic_;
};

template <typename T>
class PointIST : public PointIS<T> {
 public:
  explicit PointIST(T intensity, uint16_t semantic, uint16_t track)
      : PointIS<T>(intensity, semantic), track_(track) {}

  uint16_t& track() {
    return track_;
  }
  uint16_t track() const noexcept {
    return track_;
  }

 private:
  uint16_t track_;
};

using PointI = PointI<uint16_t>;

} // namespace model
