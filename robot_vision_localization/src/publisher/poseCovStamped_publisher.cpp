/*
 * @Description: PoseCovrianceStamped信息发布
 * @Author: pifan
 * @Date: 2020-10-29
 */
#include "robot_vision_localization/publisher/poseCovStamped_publisher.hpp"

namespace robot_vision_localization 
{
PoseCovStampedPublisher::PoseCovStampedPublisher(ros::NodeHandle& nh, 
                                                 std::string topic_name,
                                                 std::string frame_id,  
                                                 int buff_size):nh_(nh) 
{

    publisher_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>(topic_name, buff_size, true);
    poseCovStamped_.header.frame_id = frame_id;

}


void PoseCovStampedPublisher::Publish(const geometry_msgs::PoseStamped &poseStampedMsg) 
{
    PublishData(poseStampedMsg, ros::Time::now());
}

void PoseCovStampedPublisher::PublishData(const geometry_msgs::PoseStamped &poseStampedMsg, ros::Time time) 
{
    poseCovStamped_.header.stamp = time;
    poseCovStamped_.pose.pose = poseStampedMsg.pose;
    publisher_.publish(poseCovStamped_);
}

bool PoseCovStampedPublisher::HasSubscribers() {
    return publisher_.getNumSubscribers() != 0;
}
}