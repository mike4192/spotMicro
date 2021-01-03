#pragma once //designed to include the current source file only once in a single compilation.

#include <eigen3/Eigen/Geometry>
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_eigen/tf2_eigen.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

// Utility functions for spot mico motion command node


// Convert a Eigen Matrix4f to an Affine3d
Eigen::Affine3d matrix4fToAffine3d(const Eigen::Matrix4f& in);


// Create a ROS tf2 TransformStamped from a Eigen Affine3d, 
// parent frame id and child frame id. Stamped with current time, 
// so should be broadcast ASAP
geometry_msgs::TransformStamped eigAndFramesToTrans(
    const Eigen::Affine3d& transform, 
    std::string parent_frame_id, std::string child_frame_id);


// Create a transform from a translation, rotation, and parent and 
// child frame IDs. Will stamp the transform with ros::Time::now(),
// so the returned transform should be broadcast asap
  geometry_msgs::TransformStamped createTransform(
      std::string parent_frame_id, std::string child_frame_id,
      double x, double y, double z, 
      double roll, double pitch, double yaw);