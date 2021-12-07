/*
 * @Description: path信息发布
 * @Author: pifan
 * @Date: 2020-8-12
 */
#include "robot_vision_localization/publisher/path_publisher.hpp"

namespace robot_vision_localization 
{
PathPublisher::PathPublisher(ros::NodeHandle& nh, 
                             std::string topic_name,
                             std::string frame_id,  
                             int buff_size):nh_(nh) 
{

    publisher_ = nh_.advertise<nav_msgs::Path>(topic_name, buff_size, true);
    path_.header.frame_id = frame_id;
    count_ = 0;

}


void PathPublisher::Publish(const geometry_msgs::PoseStamped &poseStampedMsg) 
{
    PublishData(poseStampedMsg, ros::Time::now());
}

void PathPublisher::PublishData(const geometry_msgs::PoseStamped &poseStampedMsg, ros::Time time) 
{
    path_.header.stamp = time;
    path_.poses.push_back(poseStampedMsg);
    publisher_.publish(path_);
    count_++;
    if (count_%200)
    {
        // 定期清理
        path_.poses.clear();
    }
}

bool PathPublisher::HasSubscribers() {
    return publisher_.getNumSubscribers() != 0;
}
}