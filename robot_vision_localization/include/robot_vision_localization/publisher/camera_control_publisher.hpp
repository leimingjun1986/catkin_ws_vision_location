/*
 * @Description: camera control publisher
 * @Author: pifan
 * @Date: 2020-02-06 21:05:47
 */
#ifndef AGV_NAVIGATION_PUBLISHER_CAMERA_CONTROL_PUBLISHER_HPP_
#define AGV_NAVIGATION_PUBLISHER_CAMERA_CONTROL_PUBLISHER_HPP_

#include <string>
#include <ros/ros.h>
#include <vision_msgs/CameraControl.h>


namespace robot_vision_localization 
{
class CameraControlPublisher 
{
  public:
    CameraControlPublisher(ros::NodeHandle& nh, 
                           std::string topic_name, 
                           int buff_size);
    CameraControlPublisher() = default;

    void Publish(const uint8_t &controlByte);

    bool HasSubscribers();

  private:
    void PublishData(const uint8_t &controlByte);

  private:
    ros::NodeHandle nh_;
    ros::Publisher publisher_;
    vision_msgs::CameraControl cameraControl_;
};
}
#endif