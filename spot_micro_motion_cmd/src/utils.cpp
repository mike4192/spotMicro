#include "utils.h"

#include <eigen3/Eigen/Geometry>
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_eigen/tf2_eigen.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using namespace Eigen;
using namespace geometry_msgs::msg;

Affine3d matrix4fToAffine3d(const Matrix4f& in) {
  return Affine3d(in.cast<double>());
}

TransformStamped eigAndFramesToTrans(
    const Affine3d& transform,
    std::string parent_frame_id, std::string child_frame_id,
    const rclcpp::Time& stamp) {

  TransformStamped transform_stamped = tf2::eigenToTransform(transform);

  transform_stamped.header.stamp = stamp;
  transform_stamped.header.frame_id = parent_frame_id;
  transform_stamped.child_frame_id = child_frame_id;

  return transform_stamped;
}

TransformStamped createTransform(
    std::string parent_frame_id, std::string child_frame_id,
    double x, double y, double z,
    double roll, double pitch, double yaw,
    const rclcpp::Time& stamp) {

  TransformStamped tr_stamped;

  tr_stamped.header.stamp = stamp;
  tr_stamped.header.frame_id = parent_frame_id;
  tr_stamped.child_frame_id = child_frame_id;

  tr_stamped.transform.translation.x = x;
  tr_stamped.transform.translation.y = y;
  tr_stamped.transform.translation.z = z;
  tf2::Quaternion q;
  q.setRPY(roll, pitch, yaw);
  tr_stamped.transform.rotation.x = q.x();
  tr_stamped.transform.rotation.y = q.y();
  tr_stamped.transform.rotation.z = q.z();
  tr_stamped.transform.rotation.w = q.w();

  return tr_stamped;
}
