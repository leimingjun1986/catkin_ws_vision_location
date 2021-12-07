/*
 * @Description: path publisher
 * @Author: PIFAN
 * @Date: 2020-10
 */
#ifndef AGV_NAVIGATION_PUBLISHER_PATH_PUBLISHER_HPP_
#define AGV_NAVIGATION_PUBLISHER_PATH_PUBLISHER_HPP_

#include <string>
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

namespace robot_vision_localization 
{
class PathPublisher 
{
  public:
    PathPublisher(ros::NodeHandle& nh, 
                  std::string topic_name, 
                  std::string frame_id,  
                  int buff_size);
    PathPublisher() = default;

    void Publish(const geometry_msgs::PoseStamped &poseStampedMsg);

    bool HasSubscribers();

  private:
    void PublishData(const geometry_msgs::PoseStamped &poseStampedMsg, ros::Time time);

  private:
    ros::NodeHandle nh_;
    ros::Publisher publisher_;
    nav_msgs::Path path_;
    int count_;
};
}
#endif