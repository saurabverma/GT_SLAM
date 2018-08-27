
#include "velo_scanmatch/time_conversion.h"

#include "time.h"
#include "ros/ros.h"

namespace cartographer_ros {

::ros::Time ToRos(::cartographer::common::Time time) {
  cartographer::int64 uts_timestamp = ::cartographer::common::ToUniversal(time);
  cartographer::int64 ns_since_unix_epoch =
      (uts_timestamp -
       ::cartographer::common::kUtsEpochOffsetFromUnixEpochInSeconds *
           10000000ll) *
      100ll;
  ::ros::Time ros_time;
  ros_time.fromNSec(ns_since_unix_epoch);
  return ros_time;
}

// TODO(pedrofernandez): Write test.
::cartographer::common::Time FromRos(const ::ros::Time& time) {
  // The epoch of the ICU Universal Time Scale is "0001-01-01 00:00:00.0 +0000",
  // exactly 719162 days before the Unix epoch.
  return ::cartographer::common::FromUniversal(
      (time.sec +
       ::cartographer::common::kUtsEpochOffsetFromUnixEpochInSeconds) *
          10000000ll +
      (time.nsec + 50) / 100);  // + 50 to get the rounding correct.
}

}  // namespace cartographer_ros
