/*
 * @Description: tf监听模块
 * @Author: Pi Fan
 * @Date: 2020-6-17 
 */
#include "robot_vision_localization/tf_listener/tf_simple.hpp"

#include <Eigen/Geometry>

namespace robot_vision_localization {
TFSimple::TFSimple(ros::NodeHandle& nh, std::string base_frame_id, std::string child_frame_id) 
    :nh_(nh), base_frame_id_(base_frame_id), child_frame_id_(child_frame_id) 
{
}

bool TFSimple::getTransform(tf::StampedTransform& transform) 
{
    try 
    {
        listener_.lookupTransform(base_frame_id_, 
                                  child_frame_id_, 
                                  ros::Time(0), 
                                  transform);
        return true;
    } catch (tf::TransformException &ex) 
    {
        return false;
    }
}
}