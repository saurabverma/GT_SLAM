
#ifndef CARTO_TRANSFORM_TRANSFORM_INTERPOLATION_BUFFER_H_
#define CARTO_TRANSFORM_TRANSFORM_INTERPOLATION_BUFFER_H_

#include <vector>

#include "time.h"
#include "trajectory.pb.h"
#include "rigid_transform.h"

namespace cartographer{
namespace transform {

// A time-ordered buffer of transforms that supports interpolated lookups.
class TransformInterpolationBuffer {
 public:
  TransformInterpolationBuffer() = default;
  explicit TransformInterpolationBuffer(
      const mapping::proto::Trajectory& trajectory);

  // Adds a new transform to the buffer and removes the oldest transform if the
  // buffer size limit is exceeded.
  void Push(common::Time time, const transform::Rigid3d& transform);

  // Returns true if an interpolated transfrom can be computed at 'time'.
  bool Has(common::Time time) const;

  // Returns an interpolated transform at 'time'. CHECK()s that a transform at
  // 'time' is available.
  transform::Rigid3d Lookup(common::Time time) const;

  // Returns the timestamp of the earliest transform in the buffer or 0 if the
  // buffer is empty.
  common::Time earliest_time() const;

  // Returns the timestamp of the earliest transform in the buffer or 0 if the
  // buffer is empty.
  common::Time latest_time() const;

  // Returns true if the buffer is empty.
  bool empty() const;

 private:
  struct TimestampedTransform {
    common::Time time;
    transform::Rigid3d transform;
  };

  std::vector<TimestampedTransform> timestamped_transforms_;
};

}  // namespace transform
}  // namespace cartographer

#endif  // CARTO_TRANSFORM_TRANSFORM_INTERPOLATION_BUFFER_H_
