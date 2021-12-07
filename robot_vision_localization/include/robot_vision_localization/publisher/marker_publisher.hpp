/*
 * @Description: camera pose marker publisher for rviz
 * @Author: PIFAN
 * @Date: 2020-11-4
 */
#ifndef AGV_NAVIGATION_PUBLISHER_MARKER_PUBLISHER_HPP_
#define AGV_NAVIGATION_PUBLISHER_MARKER_PUBLISHER_HPP_

#include <string>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>
#include "robot_vision_localization/template/publisher_stamped_template.hpp"

namespace robot_vision_localization 
{
class MarkerPublisher : public PublisherStamped<visualization_msgs::Marker>
{
public:
  MarkerPublisher(ros::NodeHandle& nh, std::string topic_name, std::string frame_id,  int buff_size):PublisherStamped<visualization_msgs::Marker>(nh, topic_name, frame_id, buff_size){}
  MarkerPublisher() = default;
  void publishMarker(const geometry_msgs::PoseStamped &poseStampedMsg, const ros::Time &time)
  {
      visualization_msgs::Marker data;
      data.header.stamp = time;
      data.ns = "basic_shapes";

      // data.id = id_++;
      data.type = visualization_msgs::Marker::ARROW;
      data.action = visualization_msgs::Marker::ADD;
      data.pose.position.x = poseStampedMsg.pose.position.x;
      data.pose.position.y = poseStampedMsg.pose.position.y;
      data.pose.position.z = poseStampedMsg.pose.position.z;
      data.pose.orientation.x = poseStampedMsg.pose.orientation.x;
      data.pose.orientation.y = poseStampedMsg.pose.orientation.y;
      data.pose.orientation.z = poseStampedMsg.pose.orientation.z;
      data.pose.orientation.w = poseStampedMsg.pose.orientation.w;

      data.scale.x = 0.1;
      data.scale.y = 0.05;
      data.scale.z = 0.05;

      data.color.r = 0.0f;
      data.color.g = 1.0f;
      data.color.b = 0.0f;
      data.color.a = 1.0;

      data.lifetime = ros::Duration();

      publisher_.publish(data);
  }

  // public:
  //   MarkerPublisher(ros::NodeHandle& nh, 
  //                 std::string topic_name, 
  //                 std::string frame_id,  
  //                 int buff_size);
  //   MarkerPublisher() = default;

  //   void Publish(const geometry_msgs::PoseStamped &poseStampedMsg);

  //   bool HasSubscribers();

  // private:
  //   void PublishData(const geometry_msgs::PoseStamped &poseStampedMsg, ros::Time time);

  // private:
  //   ros::NodeHandle nh_;
  //   ros::Publisher publisher_;
  //   visualization_msgs::Marker data;
  //   int id_;
};
}
#endif