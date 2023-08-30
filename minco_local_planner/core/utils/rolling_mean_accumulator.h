/**
 * @Author: Yunkai Xia
 * @Date:   2023-08-24 15:02:58
 * @Last Modified by:   Yunkai Xia
 * @Last Modified time: 2023-08-24 15:04:31
 */
#include <stdint.h>

#ifndef __ROLLING_MEAN_ACCUMULATOR_H__
#define __ROLLING_MEAN_ACCUMULATOR_H__
#include <cassert>
#include <cstddef>
#include <vector>

namespace minco_local_planner::utils {
    
template <typename T>
class RollingMeanAccumulator {
 public:
  /**
   * Constructs the rolling mean accumulator with a specified window size.
   *
   * \param[in] rolling_window_size The unsigned integral length of the
   * accumulator's window length.
   */
  explicit RollingMeanAccumulator(size_t rolling_window_size)
      : buffer_(rolling_window_size, 0.0),
        next_insert_(0),
        sum_(0.0),
        buffer_filled_(false) {}

  /**
   * Collects the provided value in the accumulator's buffer.
   *
   * \param[in] val The value to accumulate.
   */
  void accumulate(T val) {
    sum_ -= buffer_[next_insert_];
    sum_ += val;
    buffer_[next_insert_] = val;
    next_insert_++;
    buffer_filled_ |= next_insert_ >= buffer_.size();
    next_insert_ = next_insert_ % buffer_.size();
  }

  /**
   * Calculates the rolling mean accumulated insofar.
   *
   * \return Rolling mean of the accumulated values.
   */
  T getRollingMean() const {
    size_t valid_data_count =
        buffer_filled_ * buffer_.size() + !buffer_filled_ * next_insert_;
    assert(valid_data_count > 0);
    return sum_ / valid_data_count;
  }

 private:
  std::vector<T> buffer_;
  size_t next_insert_;
  T sum_;
  bool buffer_filled_;
};
}  // namespace minco_local_planner::utils
#endif /* __ROLLING_MEAN_ACCUMULATOR_H__ */
