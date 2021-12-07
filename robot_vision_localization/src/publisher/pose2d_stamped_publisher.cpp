/*
 * @Description: stamped pose2d publisher
 * @Author: Pi Fan
 * @Date: 2020-11-17
 */
#include "robot_vision_localization/publisher/pose2d_stamped_publisher.hpp"

namespace robot_vision_localization 
{
Pose2dStampedPublisher::Pose2dStampedPublisher(ros::NodeHandle& nh, 
                                               std::string topic_name, 
                                               int buff_size):nh_(nh) 
{

    publisher_ = nh_.advertise<vision_msgs::Pose2DStamped>(topic_name, buff_size, true);
}


void Pose2dStampedPublisher::Publish(const VisionPose2DData &visionPose2DData) 
{
    PublishData(visionPose2DData, ros::Time::now());
}

void Pose2dStampedPublisher::PublishData(const VisionPose2DData &visionPose2DData, ros::Time time) 
{
    pose2dStamped_.header.stamp = time;
    pose2dStamped_.pose2D.x =  visionPose2DData.x;
    pose2dStamped_.pose2D.y =  visionPose2DData.y;
    pose2dStamped_.pose2D.theta =  visionPose2DData.theta;

     std::cout<<"pose2dStamped:"<<pose2dStamped_.header.stamp <<"   x:"<<pose2dStamped_.pose2D.x ;
    std::cout<<"  y:"<<  pose2dStamped_.pose2D.y  <<"   theta:"<<    pose2dStamped_.pose2D.theta<<std::endl;
    publisher_.publish(pose2dStamped_);
}

bool Pose2dStampedPublisher::HasSubscribers() {
    return publisher_.getNumSubscribers() != 0;
}
}