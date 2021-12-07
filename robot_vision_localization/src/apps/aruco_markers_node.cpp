/*
 * @Descripttion: 
 * @version: 
 * @Author: pifan
 * @Date: 2021-03-01 19:36:23
 * @LastEditors: Please set LastEditors
 * @LastEditTime: 2021-03-24 16:13:15
 */
#include "aruco_markers/aruco_markers.hpp"
#include "glog/logging.h"
#include "robot_vision_localization/global_defination/global_defination.h"

using namespace aruco_markers;
using namespace robot_vision_localization;

int main(int argc, char **argv)
{
    // 这个是记录日志的相关代码
    google::InitGoogleLogging(argv[0]);
    FLAGS_log_dir = WORK_SPACE_PATH + "/Log";
    FLAGS_alsologtostderr = true;
    FLAGS_colorlogtostderr = true;

    ros::init(argc, argv, "aruco_markers");
    ros::NodeHandle nh;
   std::shared_ptr<ArucoMarkers> arucoMarkersPtr_ = std::make_shared<ArucoMarkers>(nh);
    ros::spin();
}