/*
 * Copyright (c) 2016, The Regents of the University of California (Regents).
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 *    1. Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *
 *    2. Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *
 *    3. Neither the name of the copyright holder nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS AS IS
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Please contact the author(s) of this library if you have any questions.
 * Author: Erik Nelson            ( eanelson@eecs.berkeley.edu )
 */

///////////////////////////////////////////////////////////////////////////////
//
// This class can be used for running SLAM offline by loading messages from a
// ROS bagfile and processing them one-by-one. This allows for debugging without
// dealing with real-time issues, such as backed up message buffers or processor
// throttling. The offline processor by default will play back
//
///////////////////////////////////////////////////////////////////////////////

#ifndef BLAM_SLAM_OFFLINE_H
#define BLAM_SLAM_OFFLINE_H

// Internal

#include <boost/filesystem.hpp>
#include <parameter_utils/ParameterUtils.h>
#include <rosgraph_msgs/Clock.h>
#include <geometry_utils/Transform3.h>
#include <geometry_utils/Rotation3.h>
#include <geometry_utils/Quaternion.h>
#include <geometry_utils/Vector3.h>
#include <geometry_utils/GeometryUtils.h>
#include <geometry_utils/GeometryUtilsROS.h>

#include "LOAM_loopClose.h"

#include "ros/ros.h"
#include "ros/time.h"
#include "rosbag/bag.h"
#include "rosbag/view.h"
#include "sensor_msgs/PointCloud2.h"
#include "geometry_msgs/PoseStamped.h"

#include "glog/logging.h"
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

#include "tf2_ros/buffer.h"
#include "tf2_msgs/TFMessage.h"
#include "tf2_eigen/tf2_eigen.h"
#include "tf/transform_datatypes.h"

#include "velo_scanmatch/proto_stream.h"
#include "velo_scanmatch/trajectory.pb.h"
#include "velo_scanmatch/point_types.h"
#include "velo_scanmatch/time_conversion.h"
#include "velo_scanmatch/transform.h"
#include "velo_scanmatch/transform_interpolation_buffer.h"

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

namespace pu = parameter_utils;
namespace gu = geometry_utils;
namespace gr = gu::ros;

class BlamSlamOffline
{
public:
  BlamSlamOffline() {}
  ~BlamSlamOffline() {}

  bool Initialize(const ros::NodeHandle &n)
  {
    name_ = ros::names::append(n.getNamespace(), "BlamSlamOffline");

    if (!slam_.Initialize(n, true))
    {
      ROS_ERROR("%s: Failed to initialize BLAM SLAM.", name_.c_str());
      return false;
    }

    if (!LoadParameters(n))
    {
      ROS_ERROR("%s: Failed to load parameters.", name_.c_str());
      return false;
    }

    if (!RegisterCallbacks(n))
    {
      ROS_ERROR("%s: Failed to register callbacks.", name_.c_str());
      return false;
    }

    if (!ProcessFileData())
    {
      ROS_ERROR("%s: Failed to process bag file.", name_.c_str());
      return false;
    }

    return true;
  }

private:
  bool LoadParameters(const ros::NodeHandle &n)
  {
    // Checking if the lidar input pbstream filename exists
    if (!pu::Get("filename/pbstream_lidar_input", pbstream_lidar_input_filename_))
      return false;
    CHECK(!pbstream_lidar_input_filename_.empty()) << "-traj_filename is missing.";

    // Check that input bag file exists.
    if (!pu::Get("filename/bag", bag_filename_))
      return false;
    boost::filesystem::path bag_path(bag_filename_);
    if (!boost::filesystem::exists(bag_path))
    {
      ROS_ERROR("%s: Bag file does not exist.", name_.c_str());
      return false;
    }

    // Load bagfile topic names.
    if (!pu::Get("scan_topic", scan_topic_))
      return false;

    // Check if loop closure is compulsory.
    if (!pu::Get("enforce_manual_loopClose", enforce_manual_loopClose_))
      return false;

    // Check if final pose graph needs to be optimized.
    if (!pu::Get("enforce_final_optimize", enforce_final_optimize_))
      return false;

    // Check if the final optimized pose graph is to be stored.
    if (!pu::Get("save_poses", save_poses_))
      return false;
    if (save_poses_)
    {
      // Check that output pbstream filename exists.
      if (!pu::Get("filename/pbstream_output", pbstream_output_filename_))
        return false;
    }

    return true;
  }

  bool RegisterCallbacks(const ros::NodeHandle &n)
  {
    ros::NodeHandle nl;
    const std::string ns = ros::this_node::getNamespace();

    scan_pub_ = nl.advertise<BlamSlam::PointCloud>(scan_topic_, 10, false);
    pose_pub_ = nl.advertise<geometry_msgs::PoseStamped>("/sensor_raw_pose", 10, false);
    clock_pub_ = nl.advertise<rosgraph_msgs::Clock>("/clock", 10, false);

    return true;
  }

  bool ProcessFileData()
  {
    const std::string ns = ros::this_node::getNamespace();

    ros::WallTime wall_time_start = ros::WallTime::now();
    ros::Time bag_time_start, bag_time;
    bool bag_time_start_set = false;

    // Note: Store the timestamps of poses from rosbag; later reuse the timestamps
    // for storing the corresponding pose values
    std::vector<ros::Time> pose_time_stamps;

    /*********************************************************************
    Reading pbstream file containing stamped pose3d data in proto datatype
    *********************************************************************/

    // Read the pbstream file, get all the data in one shot as 'proto' datatype
    ROS_INFO("%s: Using trajectory (pbstream) file: %s", name_.c_str(), pbstream_lidar_input_filename_.c_str());
    cartographer::io::ProtoStreamReader reader(pbstream_lidar_input_filename_);
    cartographer::mapping::proto::Trajectory proto_traj;
    CHECK(reader.ReadProto(&proto_traj));
    ROS_INFO("%s: nodes contained %d in the pbstream file", name_.c_str(), proto_traj.node_size());

    // Create an interpolation object such that the pose can be interpolated
    // between two given time stamps
    const cartographer::transform::TransformInterpolationBuffer
        transform_interpolation_buffer(proto_traj);

    /**********************************************************************
    Reading rosbag file containing time stamped pointcloud data in ros:pcl
    datatype
    ***********************************************************************/

    // Open the rosbag file to get required velodyne pointcloud points
    rosbag::Bag bag(bag_filename_, rosbag::bagmode::Read);
    ROS_INFO("%s: Using bag file: %s", name_.c_str(), bag_filename_.c_str());

    // Get a view struct instance of the required topics from the bag file, this
    // helps read messages later easily
    std::vector<std::string> topics;
    topics.push_back(scan_topic_);
    rosbag::View view(bag, rosbag::TopicQuery(topics));

    /***************************************************************************
    Get pcl from rosbag and, interpolate robot pose at the time pcl was captured
    ***************************************************************************/

    // for each pcl message in the rosbag, perform the following (and finally
    // close the bag file)
    for (const rosbag::MessageInstance &message : view)
    {
      // Convert ros:pointcloud message to real pcl data
      sensor_msgs::PointCloud2::ConstPtr pc2_msg = message.instantiate<sensor_msgs::PointCloud2>();
      PointCloud pcl_cloud;
      pcl::fromROSMsg(*pc2_msg, pcl_cloud);

      // If pose tf interpolation is possible for the pcl data, then perform the
      // following
      pose_time_stamps.push_back(pc2_msg->header.stamp);
      if (transform_interpolation_buffer.Has(cartographer_ros::FromRos(pose_time_stamps.back())))
      {
        // Filter our infinity OR ill-defined points from the cloud
        PointCloud::Ptr pcl_cloud_noNaN_ptr = PointCloud::Ptr(new PointCloud());
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(pcl_cloud, (*pcl_cloud_noNaN_ptr), indices);

        // Get the pose data at the time stamped specified by the pcl message.
        const cartographer::transform::Rigid3d tracking_to_map =
            transform_interpolation_buffer.Lookup(cartographer_ros::FromRos(pose_time_stamps.back()));
        gu::Transform3 pose;
        pose.translation = gu::Vec3(tracking_to_map.translation().x(),
                                    tracking_to_map.translation().y(),
                                    tracking_to_map.translation().z());
        pose.rotation = gu::Rot3(gu::Quaternion(tracking_to_map.rotation().w(),
                                                tracking_to_map.rotation().x(),
                                                tracking_to_map.rotation().y(),
                                                tracking_to_map.rotation().z()));

        // Process the filtered pointcloud data
        slam_.Process_PointCloud_Pose_Message(pcl_cloud_noNaN_ptr, pose);

        // Publish data along with time stamp
        rosgraph_msgs::Clock clock_msg;
        clock_msg.clock = pose_time_stamps.back();
        // Raw point cloud publish
        if (scan_pub_.getNumSubscribers() > 0)
          scan_pub_.publish(pcl_cloud_noNaN_ptr);
        // Raw absolute pose publish
        if (pose_pub_.getNumSubscribers() > 0)
        {
          geometry_msgs::PoseStamped ros_pose;
          ros_pose.header.stamp = clock_msg.clock;
          ros_pose.pose = gr::ToRosPose(pose);
          pose_pub_.publish(ros_pose);
        }
        // ROS simulated time publish
        if (clock_pub_.getNumSubscribers() > 0)
        {
          clock_pub_.publish(clock_msg);
        }

        // Run this once to set the starting time for bag file data
        if (!bag_time_start_set)
        {
          bag_time_start = pose_time_stamps.back();
          bag_time_start_set = true;
        }
        bag_time = pose_time_stamps.back();
      }
      else
      {
        ROS_WARN("%s: Expected time stamp (%f) of scan from bag file not found in pose interpolation from pbstream file.",
                 name_.c_str(), pose_time_stamps.back().toSec());
      }
    }
    bag.close();

    // Apply overall loop closure if required
    if (enforce_manual_loopClose_)
    {
      ROS_INFO("%s: Applying loop closure between first and last pose keys.", name_.c_str());
      slam_.ForcedLoopClosure();
    }

    // Apply final graph optimization (compulsory for )
    if (enforce_final_optimize_ || save_poses_)
    {
      ROS_INFO("%s: Applying final graph optimization on demand.", name_.c_str());
      slam_.OptimizeGraph();
    }

    // Save final pose as pbstream
    if (save_poses_)
    {
      ROS_INFO("%s: Getting optimised poses from pose graph.", name_.c_str());
      std::vector<gu::Transform3> poses_new = slam_.SaveTraj();

      // Convert data to Cartographer trajectory
      cartographer::mapping::proto::Trajectory proto_traj_new;
      for (unsigned int i = 0; i < poses_new.size(); i++)
      {
        // Add a new node in the trajectory
        auto new_node = proto_traj_new.add_node();

        // Set time stamp assuming the stamps are same as the collected earlier
        new_node->set_timestamp(cartographer::common::ToUniversal(
            cartographer_ros::FromRos(pose_time_stamps.at(i))));

        // Create a new proto pose and add to the newly created trajectory node
        gu::Quaternion pose_quat = gu::RToQuat(poses_new.at(i).rotation);
        cartographer::transform::proto::Rigid3d *t_proto_rigid3d =
            new cartographer::transform::proto::Rigid3d(
                cartographer::transform::ToProto(
                    cartographer::transform::Rigid3d(
                        cartographer::transform::Rigid3d::Vector(
                            poses_new.at(i).translation(1),
                            poses_new.at(i).translation(2),
                            poses_new.at(i).translation(3)),
                        cartographer::transform::Rigid3d::Quaternion(
                            pose_quat(1),
                            pose_quat(2),
                            pose_quat(3),
                            pose_quat(4)))));
        new_node->set_allocated_pose(t_proto_rigid3d);
      }

      // Save the data and close the file
      ROS_INFO("%s: New trajectory node size: %d", name_.c_str(), proto_traj_new.node_size());
      cartographer::io::ProtoStreamWriter writer(pbstream_output_filename_);
      writer.WriteProto(proto_traj_new);
      writer.Close();
      ROS_INFO("%s: Saved trajectory from optimized pose graph to pbstream file: %s",
               name_.c_str(), pbstream_output_filename_.c_str());
    }

    // Notify completion of the bag
    const double total_wall_time =
        (ros::WallTime::now() - wall_time_start).toSec();
    const double total_bag_time = (bag_time - bag_time_start).toSec();
    ROS_INFO("%s: Completed whole process in %lf s.",
             name_.c_str(), total_wall_time);
    ROS_INFO("%s: Finished processing pose graph at %lf percent speed of real-time.",
             name_.c_str(), (total_bag_time / total_wall_time) * 100.f);

    return true;
  }

  std::string name_;

  std::string scan_topic_;
  std::string pbstream_lidar_input_filename_;
  std::string bag_filename_;
  std::string pbstream_output_filename_;

  bool enforce_manual_loopClose_;
  bool enforce_final_optimize_;
  bool save_poses_;

  BlamSlam slam_;

  ros::Publisher scan_pub_;
  ros::Publisher pose_pub_;
  ros::Publisher clock_pub_;
};

#endif
