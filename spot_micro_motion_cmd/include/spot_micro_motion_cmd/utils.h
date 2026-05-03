#pragma once

#include <eigen3/Eigen/Geometry>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_eigen/tf2_eigen.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

// Utility functions for spot micro motion command node

// Convert a Eigen Matrix4f to an Affine3d
Eigen::Affine3d matrix4fToAffine3d(const Eigen::Matrix4f& in);

// Create a ROS tf2 TransformStamped from a Eigen Affine3d,
// parent frame id and child frame id. Stamped with provided time,
// so should be broadcast ASAP
geometry_msgs::msg::TransformStamped eigAndFramesToTrans(
    const Eigen::Affine3d& transform,
    std::string parent_frame_id, std::string child_frame_id,
    const rclcpp::Time& stamp);

// Create a transform from a translation, rotation, and parent and
// child frame IDs. Will stamp the transform with the provided time,
// so the returned transform should be broadcast asap
geometry_msgs::msg::TransformStamped createTransform(
    std::string parent_frame_id, std::string child_frame_id,
    double x, double y, double z,
    double roll, double pitch, double yaw,
    const rclcpp::Time& stamp);
