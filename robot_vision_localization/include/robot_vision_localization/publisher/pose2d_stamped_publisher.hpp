/*
 * @Description: stamped pose2d publisher
 * @Author: Pi Fan
 * @Date: 2020-11-17
 */
#ifndef LIDAR_LOCALIZATION_PUBLISHER_POSE2D_STAMPED_PUBLISHER_HPP_
#define LIDAR_LOCALIZATION_PUBLISHER_POSE2D_STAMPED_PUBLISHER_HPP_

#include <string>
#include <ros/ros.h>
#include <vision_msgs/Pose2DStamped.h>
#include "robot_vision_localization/sensor_data/vision_pose2D_data.hpp"

namespace robot_vision_localization 
{
class Pose2dStampedPublisher 
{
  public:
    Pose2dStampedPublisher(ros::NodeHandle& nh, 
                           std::string topic_name, 
                           int buff_size);
    Pose2dStampedPublisher() = default;

    void Publish(const VisionPose2DData &visionPose2DData);

    bool HasSubscribers();

  public:
    void PublishData(const VisionPose2DData &visionPose2DData,ros::Time time);

  private:
    ros::NodeHandle nh_;
    ros::Publisher publisher_;
    vision_msgs::Pose2DStamped pose2dStamped_;
};
}
#endif