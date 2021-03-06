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
 * Authors: Erik Nelson            ( eanelson@eecs.berkeley.edu )
 */

#include <blam_slam/LOAM_loopClose.h>
#include <parameter_utils/ParameterUtils.h>
#include <pcl_conversions/pcl_conversions.h>

namespace pu = parameter_utils;

BlamSlam::BlamSlam()
    : estimate_update_rate_(0.0), visualization_update_rate_(0.0), initialized_(false) {}

BlamSlam::~BlamSlam() {}

bool BlamSlam::Initialize(const ros::NodeHandle &n, bool from_log)
{
  name_ = ros::names::append(n.getNamespace(), "BlamSlam");

  if (!loop_closure_.Initialize(n))
  {
    ROS_ERROR("%s: Failed to initialize laser loop closure.", name_.c_str());
    return false;
  }

  if (!localization_.Initialize(n))
  {
    ROS_ERROR("%s: Failed to initialize localization.", name_.c_str());
    return false;
  }

  if (!mapper_.Initialize(n))
  {
    ROS_ERROR("%s: Failed to initialize mapper.", name_.c_str());
    return false;
  }

  if (!LoadParameters(n))
  {
    ROS_ERROR("%s: Failed to load parameters.", name_.c_str());
    return false;
  }

  if (!RegisterCallbacks(n, from_log))
  {
    ROS_ERROR("%s: Failed to register callbacks.", name_.c_str());
    return false;
  }

  return true;
}

bool BlamSlam::LoadParameters(const ros::NodeHandle &n)
{
  // Load update rates.
  if (!pu::Get("rate/estimate", estimate_update_rate_))
    return false;
  if (!pu::Get("rate/visualization", visualization_update_rate_))
    return false;

  // Load frame ids.
  if (!pu::Get("frame_id/fixed", fixed_frame_id_))
    return false;
  if (!pu::Get("frame_id/base", base_frame_id_))
    return false;

  return true;
}

bool BlamSlam::RegisterCallbacks(const ros::NodeHandle &n, bool from_log)
{
  // Create a local nodehandle to manage callback subscriptions.
  ros::NodeHandle nl(n);

  visualization_update_timer_ = nl.createTimer(
      visualization_update_rate_, &BlamSlam::VisualizationTimerCallback, this);

  if (from_log)
    return RegisterLogCallbacks(n);
}

bool BlamSlam::RegisterLogCallbacks(const ros::NodeHandle &n)
{
  ROS_INFO("%s: Registering log callbacks.", name_.c_str());
  return CreatePublishers(n);
}

bool BlamSlam::CreatePublishers(const ros::NodeHandle &n)
{
  // Create a local nodehandle to manage callback subscriptions.
  ros::NodeHandle nl(n);

  base_frame_pcld_pub_ =
      nl.advertise<PointCloud>("base_frame_point_cloud", 10, false);

  return true;
}

void BlamSlam::VisualizationTimerCallback(const ros::TimerEvent &ev)
{
  mapper_.PublishMap();
}

void BlamSlam::Process_PointCloud_Pose_Message(
    const PointCloud::ConstPtr &msg, const gu::Transform3 &pose_raw_now)
{
  // The first ever message is simply stored, otherwise this block is never used
  // again Update odometry by performing ICP.
  if (!initialized_)
  {
    // First update ever.
    initialized_ = true;

    // Insert the current pointcloud data into the map
    PointCloud::Ptr unused(new PointCloud);
    mapper_.InsertPoints(msg, unused.get());

    // Add the current pointcloud data as the new key_pose
    loop_closure_.AddKeyScanPair(0, msg);

    // Store the current pose value for future estimations
    pose_raw_prev_ = pose_raw_now;

    return;
  }

  // Calculate and use the relative changes in pose w.r.t. previous
  // values instead of absolute pose values. Thereafter, update the estimated
  // current pose based on the newly calculated incremental update.
  gu::Transform3 incremental_estimate_ = gu::PoseDelta(pose_raw_prev_, pose_raw_now);
  pose_raw_prev_ = pose_raw_now; // Store the current pose value for future estimations
  localization_.SetIntegratedEstimate(
      gu::PoseUpdate(
          localization_.GetIntegratedEstimate(),
          incremental_estimate_));

  // Check for new loop closures.
  bool new_keyframe;
  if (HandleLoopClosures(msg, incremental_estimate_, &new_keyframe))
  {
    // We found one - regenerate the 3D map i.e. the pose of each graph node is
    // already updated in "HandleLoopClosures", hence now simply update the
    // velodyne scan of each node to get a new map
    PointCloud::Ptr regenerated_map(new PointCloud);
    loop_closure_.GetMaximumLikelihoodPoints(regenerated_map.get());

    // Insert the point cloud data into the map, if possible.
    mapper_.Reset();
    PointCloud::Ptr unused(new PointCloud);
    mapper_.InsertPoints(regenerated_map, unused.get());

    // Also reset the robot's estimated position to the last pose value in the pose graph
    localization_.SetIntegratedEstimate(loop_closure_.GetLastPose());
  }
  else
  {
    // No new loop closures - but was there a new key frame? If so, add new
    // points to the map.
    if (new_keyframe)
    {
      // Transform the incoming pointcloud message to the respective pose
      PointCloud::Ptr msg_fixed(new PointCloud);
      localization_.TransformPointsToPose(*msg, msg_fixed.get(),
                                          localization_.GetIntegratedEstimate());

      // Insert the point cloud data into the map, if possible. Note that
      // 'msg_fixed' has pose defined in fixed global frame already - this is
      // where the pose obtained from pbstream file has to be inserted
      PointCloud::Ptr unused(new PointCloud);
      mapper_.InsertPoints(msg_fixed, unused.get());
    }
  }

  // Publish odometry edges, loop closure edges, nodes in the pose graph,
  // keyframe nodes in the pose graph, a loop closure checking sphere around the
  // current sensor frame and, the pose graph.
  loop_closure_.PublishPoseGraph();

  // Publish the incoming point cloud message from the base frame, only if there
  // are any subscribers
  if (base_frame_pcld_pub_.getNumSubscribers() != 0)
  {
    PointCloud base_frame_pcld = *msg;
    base_frame_pcld.header.frame_id = base_frame_id_;
    base_frame_pcld_pub_.publish(base_frame_pcld);
  }
}

bool BlamSlam::HandleLoopClosures(const PointCloud::ConstPtr &scan,
                                  const gu::Transform3 &delta, bool *new_keyframe)
{
  if (new_keyframe == NULL)
  {
    ROS_ERROR("%s: Output boolean for new keyframe is null.", name_.c_str());
    return false;
  }

  // // TODO: Calculate covariance in ICP_COV
  // const Eigen::Matrix4f T = delta;

  // pcl::PointCloud<pcl::PointNormal> scan;
  // pcl::PointCloud<pcl::PointNormal> target_normal;
  // pcl::copyPointCloud(*scan, source_normal);
  // pcl::copyPointCloud(*target, target_normal);

  // Eigen::MatrixXd ICP_COV(6, 6);
  // // ICP_COV = Eigen::MatrixXd::Zero(6, 6);
  // calculate_ICP_COV(source_normal, target_normal, T, ICP_COV);

  gu::MatrixNxNBase<double, 6> covariance;
  // for (int i = 0; i < 6; i++)
  // {
  //   for (int j = 0; j < 6; j++)
  //   {
  //     covariance(i, j) = ICP_COV(i, j);
  //   }
  // }
  covariance.Zeros();
  for (int i = 0; i < 6; ++i)
    covariance(i, i) = 1;

  // "AddBetweenFactor" decides whether a pose is to added in the pose graph (as
  // a regular node OR as a key node which decides the whole graph design during
  // optimisation)
  unsigned int pose_key;
  const ros::Time stamp = pcl_conversions::fromPCL(scan->header.stamp);
  if (!loop_closure_.AddBetweenFactor(delta, covariance, stamp, &pose_key))
  {
    return false;
  }
  *new_keyframe = true;

  // simply inset the key-scan pair to the data storage vector, this should
  // ideally always return true
  if (!loop_closure_.AddKeyScanPair(pose_key, scan))
  {
    return false;
  }

  // Find all possible loop closures with the last added key and update the pose
  // graph accordingly (i.e. update the pose of all other existing nodes based
  // on loop closure based graph optimisation)
  std::vector<unsigned int> closure_keys;
  if (!loop_closure_.FindLoopClosures(pose_key, &closure_keys))
  {
    return false;
  }

  // Simply mention which keyed poses are used for loop closure
  for (const auto &closure_key : closure_keys)
  {
    ROS_INFO("%s: Closed loop between poses %u and %u.", name_.c_str(),
             pose_key, closure_key);
  }
  return true;
}

void BlamSlam::ForcedLoopClosure()
{
  // Manually enforce loop closure
  loop_closure_.FindLoopClosures_Manual();

  // Regenerate the 3D map.
  PointCloud::Ptr regenerated_map(new PointCloud);
  loop_closure_.GetMaximumLikelihoodPoints(regenerated_map.get());

  mapper_.Reset();
  PointCloud::Ptr unused(new PointCloud);
  mapper_.InsertPoints(regenerated_map, unused.get());

  // Also reset the robot's estimated position.
  localization_.SetIntegratedEstimate(loop_closure_.GetLastPose());

  // Visualize the pose graph and current loop closure radius.
  loop_closure_.PublishPoseGraph();
}

void BlamSlam::OptimizeGraph()
{
  loop_closure_.OptimizeNow();
}

std::vector<gu::Transform3> BlamSlam::SaveTraj() const
{
  return loop_closure_.GetAllPose();
}
