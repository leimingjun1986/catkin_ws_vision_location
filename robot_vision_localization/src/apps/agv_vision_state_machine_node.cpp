/*
 * @Descripttion: 
 * @version: 
 * @Author: pifan
 * @Date: 2021-01-05 15:14:25
 * @LastEditors: pifan
 * @LastEditTime: 2021-03-02 14:14:52
 */
/*
* date:2020-8-1 14:08
* author: pifan
生成可执行文件以后，为了能够实现软关机，需要对可执行文件修改权限
sudo chown root:root robot_vision_localization && sudo chmod 4775 robot_vision_localization
revisin log:
*2020-11-11: reject localization for big angle
*2020-11-12：增加一个pose2d的消息发送，作为底层里程计校准数据，话题名字/odomPoseCalibration 类型：geometry_msgs/Pose2D
*2020-11-27: rewrite the main function as a class
*/
#include "robot_vision_localization/vision_localization_process/vision_localization_process.hpp"
#include "glog/logging.h"
#include "robot_vision_localization/global_defination/global_defination.h"

using namespace robot_vision_localization;

int main(int argc, char **argv)
{
    // 这个是记录日志的相关代码
  
   //机器人是视觉定位 入口程序
    ros::init(argc, argv, "robot_vision_localization");
    ros::NodeHandle nh;
    std::shared_ptr<VisionLocatlizationInterFace> visionLocalizationPtr = std::make_shared<VisionLocatlizationInterFace>(nh);
}