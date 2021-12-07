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

#define RATE 10.0

//尝试标定 功能没有用到
int main(int argc, char **argv)
{
    // 这个是记录日志的相关代码
    google::InitGoogleLogging(argv[0]);
    FLAGS_log_dir = WORK_SPACE_PATH + "/Log";
    FLAGS_alsologtostderr = true;
    FLAGS_colorlogtostderr = true;

    ros::init(argc, argv, "robot_vision_localization");
    ros::NodeHandle nh;
    VisionLocalizationParam  
    std::shared_ptr<VisionLocalizationProcess> visionLocalizationProcessPtr = std::make_shared<VisionLocalizationProcess>(nh);

    ros::Rate loop_rate(RATE); 
    while(ros::ok())
    {
        visionLocalizationProcessPtr->cameraCalibration();
        ros::spinOnce();
        loop_rate.sleep();
    }
}