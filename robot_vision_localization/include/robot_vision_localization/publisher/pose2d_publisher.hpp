/*
 * @Description: path publisher
 * @Author:pifan
 * @Date: 2020-02-06 21:05:47
 */
#ifndef LIDAR_LOCALIZATION_PUBLISHER_POSE2D_PUBLISHER_HPP_
#define LIDAR_LOCALIZATION_PUBLISHER_POSE2D_PUBLISHER_HPP_

#include <string>
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose2D.h>
#include "robot_vision_localization/sensor_data/vision_pose2D_data.hpp"

namespace robot_vision_localization 
{
class Pose2dPublisher 
{
  public:
    Pose2dPublisher(ros::NodeHandle& nh, 
                    std::string topic_name, 
                    int buff_size);
    Pose2dPublisher() = default;

    void Publish(const VisionPose2DData &visionPose2DData);

    bool HasSubscribers();

  private:
    void PublishData(const VisionPose2DData &visionPose2DData);

  private:
    ros::NodeHandle nh_;
    ros::Publisher publisher_;
    geometry_msgs::Pose2D pose2d_;
};
}
#endif