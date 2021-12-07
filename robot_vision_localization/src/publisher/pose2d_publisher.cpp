/*
 * @Description: path信息发布
 * @Author: pifan
 * @Date: 2020-8-12
 */
#include "robot_vision_localization/publisher/pose2d_publisher.hpp"

namespace robot_vision_localization 
{
Pose2dPublisher::Pose2dPublisher(ros::NodeHandle& nh, 
                             std::string topic_name, 
                             int buff_size):nh_(nh) 
{

    publisher_ = nh_.advertise<geometry_msgs::Pose2D>(topic_name, buff_size, true);
}


void Pose2dPublisher::Publish(const VisionPose2DData &visionPose2DData) 
{
    PublishData(visionPose2DData);
}

void Pose2dPublisher::PublishData(const VisionPose2DData &visionPose2DData) 
{
    pose2d_.x =  visionPose2DData.x;
    pose2d_.y =  visionPose2DData.y;
    pose2d_.theta =  visionPose2DData.theta;
    publisher_.publish(pose2d_);
}

bool Pose2dPublisher::HasSubscribers() {
    return publisher_.getNumSubscribers() != 0;
}
}