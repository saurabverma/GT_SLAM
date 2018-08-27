
#ifndef CARTOGRAPHER_ROS_TIME_CONVERSION_H_
#define CARTOGRAPHER_ROS_TIME_CONVERSION_H_

#include "time.h"
#include "ros/ros.h"

namespace cartographer_ros {

::ros::Time ToRos(::cartographer::common::Time time);

::cartographer::common::Time FromRos(const ::ros::Time& time);

}  // namespace cartographer_ros

#endif  // CARTOGRAPHER_ROS_TIME_CONVERSION_H_
