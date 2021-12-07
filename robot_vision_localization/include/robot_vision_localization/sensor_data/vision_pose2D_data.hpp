/*
 * @Description: 
 * @Author: Pi Fan
 * @Date: 2019-04-23 
 */
#ifndef AGV_NAVIGATION_SENSOR_DATA_CAMERA_POSE_DATA_HPP_
#define AGV_NAVIGATION_SENSOR_DATA_CAMERA_POSE_DATA_HPP_
#include <ros/ros.h>

namespace robot_vision_localization 
{
class VisionPose2DData 
{
public:
    //double time;
    ros::Time time;
    int quantityOfMarkers;
    float x;
    float y;
    float theta;
public:
   void clearData()
   {
       time = ros::Time(0.0);
       quantityOfMarkers = 0;
       x = 0;
       y = 0;
       theta = 0;
   }

};
}
#endif