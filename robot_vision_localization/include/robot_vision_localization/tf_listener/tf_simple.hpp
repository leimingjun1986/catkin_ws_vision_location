/*
 * @Description: tf监听模块
 * @Author: Pi Fan
 * @Date: 2020-8-05
 */
#ifndef AGV_NAVIGATION_TF_SIMPLE_HPP_
#define AGV_NAVIGATION_TF_SIMPLE_HPP_

#include <string>

#include <Eigen/Dense>
#include <ros/ros.h>
#include <tf/transform_listener.h>

namespace robot_vision_localization {
class TFSimple {
public:
  TFSimple(ros::NodeHandle& nh, std::string base_frame_id, std::string child_frame_id);
  TFSimple() = default;

  bool getTransform(tf::StampedTransform& transform);

private:
  //bool TransformToMatrix(const tf::StampedTransform& transform, Eigen::Matrix4f& transform_matrix);

private:
  ros::NodeHandle nh_;
  tf::TransformListener listener_;
  std::string base_frame_id_;
  std::string child_frame_id_;
};
}

#endif