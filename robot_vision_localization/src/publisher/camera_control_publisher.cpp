/*
 * @Description: path信息发布
 * @Author: pifan
 * @Date: 2020-8-12
 */
#include "robot_vision_localization/publisher/camera_control_publisher.hpp"

namespace robot_vision_localization 
{
CameraControlPublisher::CameraControlPublisher(ros::NodeHandle& nh, 
                                               std::string topic_name, 
                                               int buff_size):nh_(nh) 
{

    publisher_ = nh_.advertise<vision_msgs::CameraControl>(topic_name, buff_size, true);
}


void CameraControlPublisher::Publish(const uint8_t &controlByte) 
{
    PublishData(controlByte);
}

void CameraControlPublisher::PublishData(const uint8_t &controlByte) 
{
    cameraControl_.controlByte = controlByte;
    publisher_.publish(cameraControl_);
}

bool CameraControlPublisher::HasSubscribers() 
{
    return publisher_.getNumSubscribers() != 0;
}
}