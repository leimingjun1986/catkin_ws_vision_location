/*
 * @Description: pose with covariance stamped publisher for ekf localization 
 * @Author: Pi Fan
 * @Date: 2020-10-29
 */
#ifndef AGV_NAVIGATION_PUBLISHER_POSE_COV_STAMPED_HPP_
#define AGV_NAVIGATION_PUBLISHER_POSE_COV_STAMPED_HPP_

#include <string>
#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>

namespace robot_vision_localization 
{
class PoseCovStampedPublisher 
{
  public:
    PoseCovStampedPublisher(ros::NodeHandle& nh, 
                            std::string topic_name, 
                            std::string frame_id,  
                            int buff_size);
    PoseCovStampedPublisher() = default;

    void Publish(const geometry_msgs::PoseStamped &poseStampedMsg);

    bool HasSubscribers();

  private:
    void PublishData(const geometry_msgs::PoseStamped &poseStampedMsg, ros::Time time);

  private:
    ros::NodeHandle nh_;
    ros::Publisher publisher_;
    geometry_msgs::PoseWithCovarianceStamped poseCovStamped_;
};
}
#endif