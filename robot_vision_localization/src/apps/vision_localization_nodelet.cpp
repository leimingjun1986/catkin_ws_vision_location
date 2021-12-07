#include   <robot_vision_localization/vision_localization_process/vision_localization_nodelet.hpp>
 #include   <pluginlib/class_list_macros.h>

 using  namespace   robot_vision_localization;

Vision_Localization_Nodelet::Vision_Localization_Nodelet()
{
    ;
}

Vision_Localization_Nodelet::~Vision_Localization_Nodelet(){
    ;
}

//nodelet 动态加载入口
void Vision_Localization_Nodelet::onInit()
{
  ros::NodeHandle nh=getNodeHandle();
  ros::NodeHandle nh_priv=getPrivateNodeHandle();
  vision_location_interface_ptr.reset(new  VisionLocatlizationInterFace(nh));    
}

PLUGINLIB_EXPORT_CLASS(robot_vision_localization::Vision_Localization_Nodelet, nodelet::Nodelet)