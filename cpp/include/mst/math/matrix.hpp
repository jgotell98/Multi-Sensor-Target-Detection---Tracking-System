#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <initializer_list>
#include <stdexcept>

namespace mst {

template <std::size_t Rows, std::size_t Cols>
class Matrix final {
 public:
  static constexpr std::size_t kRows = Rows;
  static constexpr std::size_t kCols = Cols;

  Matrix() { data_.fill(0.0); }

  Matrix(std::initializer_list<double> init) {
    data_.fill(0.0);
    std::size_t i = 0;
    for (double v : init) {
      if (i >= data_.size()) break;
      data_[i++] = v;
    }
  }

  double& operator()(std::size_t r, std::size_t c) { return data_[r * Cols + c]; }
  double operator()(std::size_t r, std::size_t c) const { return data_[r * Cols + c]; }

  const std::array<double, Rows * Cols>& data() const { return data_; }

  static Matrix identity() {
    static_assert(Rows == Cols, "identity requires square matrix");
    Matrix m;
    for (std::size_t i = 0; i < Rows; ++i) m(i, i) = 1.0;
    return m;
  }

 private:
  std::array<double, Rows * Cols> data_{};
};

template <std::size_t R, std::size_t C>
inline Matrix<R, C> operator+(const Matrix<R, C>& a, const Matrix<R, C>& b) {
  Matrix<R, C> out;
  for (std::size_t r = 0; r < R; ++r) {
    for (std::size_t c = 0; c < C; ++c) out(r, c) = a(r, c) + b(r, c);
  }
  return out;
}

template <std::size_t R, std::size_t C>
inline Matrix<R, C> operator-(const Matrix<R, C>& a, const Matrix<R, C>& b) {
  Matrix<R, C> out;
  for (std::size_t r = 0; r < R; ++r) {
    for (std::size_t c = 0; c < C; ++c) out(r, c) = a(r, c) - b(r, c);
  }
  return out;
}

template <std::size_t R, std::size_t C>
inline Matrix<R, C> operator*(double s, const Matrix<R, C>& a) {
  Matrix<R, C> out;
  for (std::size_t r = 0; r < R; ++r) {
    for (std::size_t c = 0; c < C; ++c) out(r, c) = s * a(r, c);
  }
  return out;
}

template <std::size_t R, std::size_t C>
inline Matrix<R, C> operator*(const Matrix<R, C>& a, double s) {
  return s * a;
}

template <std::size_t R, std::size_t K, std::size_t C>
inline Matrix<R, C> operator*(const Matrix<R, K>& a, const Matrix<K, C>& b) {
  Matrix<R, C> out;
  for (std::size_t r = 0; r < R; ++r) {
    for (std::size_t c = 0; c < C; ++c) {
      double sum = 0.0;
      for (std::size_t k = 0; k < K; ++k) sum += a(r, k) * b(k, c);
      out(r, c) = sum;
    }
  }
  return out;
}

template <std::size_t R, std::size_t C>
inline Matrix<C, R> transpose(const Matrix<R, C>& a) {
  Matrix<C, R> out;
  for (std::size_t r = 0; r < R; ++r) {
    for (std::size_t c = 0; c < C; ++c) out(c, r) = a(r, c);
  }
  return out;
}

inline Matrix<1, 1> inverse(const Matrix<1, 1>& a) {
  if (std::abs(a(0, 0)) < 1e-12) throw std::runtime_error("singular 1x1");
  return Matrix<1, 1>{1.0 / a(0, 0)};
}

inline Matrix<2, 2> inverse(const Matrix<2, 2>& a) {
  const double det = a(0, 0) * a(1, 1) - a(0, 1) * a(1, 0);
  if (std::abs(det) < 1e-12) throw std::runtime_error("singular 2x2");
  const double inv_det = 1.0 / det;
  return Matrix<2, 2>{inv_det * a(1, 1), -inv_det * a(0, 1), -inv_det * a(1, 0), inv_det * a(0, 0)};
}

}  // namespace mst

