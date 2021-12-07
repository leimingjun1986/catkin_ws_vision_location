/*
 * @Description: camera pose marker publisher for rviz
 * @Author: pifan
 * @Date: 2020-8-12
 */
#include "robot_vision_localization/publisher/marker_publisher.hpp"

namespace robot_vision_localization 
{
// MarkerPublisher::MarkerPublisher(ros::NodeHandle& nh, 
//                                  std::string topic_name,
//                                  std::string frame_id,  
//                                  int buff_size):nh_(nh) 
// {

//     publisher_ = nh_.advertise<visualization_msgs::Marker>(topic_name, buff_size, true);
//     marker_.header.frame_id = frame_id;
//     id_ = 0;

// }


// void MarkerPublisher::Publish(const geometry_msgs::PoseStamped &poseStampedMsg) 
// {
//     PublishData(poseStampedMsg, ros::Time::now());
// }

// void MarkerPublisher::PublishData(const geometry_msgs::PoseStamped &poseStampedMsg, ros::Time time) 
// {
//     marker_.header.stamp = time;
//     marker_.ns = "basic_shapes";
//     marker_.id = id_++;
//     marker_.type = visualization_msgs::Marker::ARROW;
//     marker_.action = visualization_msgs::Marker::ADD;
//     marker_.pose.position.x = poseStampedMsg.pose.position.x;
//     marker_.pose.position.y = poseStampedMsg.pose.position.y;
//     marker_.pose.position.z = poseStampedMsg.pose.position.z;
//     marker_.pose.orientation.x = poseStampedMsg.pose.orientation.x;
//     marker_.pose.orientation.y = poseStampedMsg.pose.orientation.y;
//     marker_.pose.orientation.z = poseStampedMsg.pose.orientation.z;
//     marker_.pose.orientation.w = poseStampedMsg.pose.orientation.w;

//     marker_.scale.x = 0.1;
//     marker_.scale.y = 0.05;
//     marker_.scale.z = 0.05;

//     marker_.color.r = 0.0f;
//     marker_.color.g = 1.0f;
//     marker_.color.b = 0.0f;
//     marker_.color.a = 1.0;

//     marker_.lifetime = ros::Duration();

//     publisher_.publish(marker_);

// }

// bool MarkerPublisher::HasSubscribers() {
//     return publisher_.getNumSubscribers() != 0;
// }
}