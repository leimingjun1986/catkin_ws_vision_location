/*
 * @Descripttion: 
 * 在CollectDataState状态收集定位数据
 * 在CalibrateOdomState标定底层里程计
 * @version: 
 * @Author: pifan
 * @Date: 2021-01-05 15:14:25
 * @LastEditors: 皮钒
 * @LastEditTime: 2021-03-27 11:15:20
 */

#include "robot_vision_localization/vision_localization_process/vision_localization_process.hpp"
#include "robot_vision_localization/global_defination/global_defination.h"


//using namespace state_machine;  机器人视觉定位   命名空间
namespace robot_vision_localization 
{
    //构造函数很长   主要是类成员变量初始化列表
    //CExample(): a(0),b(8.8)
   //  {}
    
VisionLocalizationProcess::VisionLocalizationProcess(ros::NodeHandle& nh, std::string hardwareID, double desired_freq, VisionLocalizationParam& param):
    Component(desired_freq),node_nh_(nh), node_priv_("~"),desired_freq_(desired_freq), hardwareID_(hardwareID),  softwareStatusTask_ (hardwareID_, desired_freq_)                                                                                                                                                                                                                                                                                                                                                                  
{
     
     endVisionEnableSubPtr_=std::make_shared<SubscriberSingle<std_msgs::Bool>>(node_nh_, param.EndVisionEnableTopic, 1)  ;//视觉定位使能话题订阅    运动控制模块发出    调试时可手动发送
      //三个相机的到车身的TF变换
    tfBase2WorldPtr_ = std::make_shared<TFSimple>(node_nh_, param.tfBaseToWorld.first, param.tfBaseToWorld.second) ;  //订阅车身变到相机坐标系下的tF变换    launch 文件中配置
    odomSubPtr_=std::make_shared<SubscriberSingle<nav_msgs::Odometry>>(node_nh_,  param.OdomTopic,  1);   //订阅车身里程计信息  做视觉定位是否正确判断
    cameraControlSubPtr_ = std::make_shared<SubscriberSingle<vision_msgs::CameraControl>>(node_nh_,  param.CameraControlTopic,  1);
    pose2dPubPtr_   =std::make_shared<Pose2dPublisher>(node_nh_, param.BasePoseTopic, 1);   // 输出  全局车身定位  无限制条件  调试用
    poseCalibrationPubPtr_=std::make_shared<Pose2dStampedPublisher>(node_nh_,param.PoseCalibrationTopic, 1);  //发送定位输出  与basePose2d一致  加了限制条件
    poseCovStampedPubPtr_=std::make_shared<PoseCovStampedPublisher>(node_nh_, param.BasePoseCovStampTopic,  param.frame_id,  1);  //没有用到
    world2MarkerPtr_=std::make_shared<TFSimple>(node_nh_, param.tfWorldToMarker.first,  param.tfWorldToMarker.second);     //订阅世界坐标系到二维码TF变换
  
   ROS_INFO("VisionLocalizationProcess constructor  param.CamNum:%d",param.CamNum);
   for(int cam_id=0;cam_id<param.CamNum;cam_id++){
               std::shared_ptr<ArucoSub>  aruco_sub_ptr  =  std::make_shared<ArucoSub>(node_nh_, param.MarkPoseTopics[cam_id], 1);
               arucoMarkersPtrVector_.push_back(aruco_sub_ptr);
              std::shared_ptr<TFSimple>   tfsimple  =  std::make_shared<TFSimple>(node_nh_, param.tfCamerasToBase[cam_id].first,  param.tfCamerasToBase[cam_id].second );
              cameraToBaseTfPtrVector_.push_back(tfsimple);
   }
      //三个相机的到车身的TF变换                                                                                                                                                                                                                                                                                                                                                           
    debug_=param.debug;
    // 这里必须指定大小
    cameraToBaseTfVector_.resize(cameraToBaseTfPtrVector_.size());
    arucoMarkersBuffer_.resize(arucoMarkersPtrVector_.size());
  
    initConfig();
   ROS_INFO("VisionLocalizationProcess constructor  finish....");
}

/**
 * @name: 
 * @brief: main thread
 * @Date: 2021-03-05 16:40:42
 * @params {*}
 * @return {*}
 */

//状态机
void VisionLocalizationProcess::ControlThread()
{
    ros::Rate r(desired_freq_);

    while (ros::ok())
    {
        switch (iState)
        {
            case INIT_STATE:
                InitState();
                break;
            case STANDBY_STATE:
                break;
            case READY_STATE:
                ReadyState();
                break;
            case COLLECT_DATA_STATE:
                CollectDataState();
                break;
            case CALIBRATE_ODOM_STATE:
                CalibrateOdomState();
                break;
            default:
                break;
        }
        ROS_INFO_STREAM_THROTTLE(1, "iState: " << GetStateString() << std::endl);
        AllState();
        // 刷新一次sub的缓冲
        ros::spinOnce();
        r.sleep();
    }
}


/**
 * @name: 
 * @brief: 将相机的定位数据存储起来
 * @Date: 2021-03-03 19:27:45
 * @params {*}
 * @return {*}
 */
void VisionLocalizationProcess::CollectDataState()
{
    // 用deque的目的是缓冲里面有数据就放到deque里面
    // 一旦读到一次的话就会把缓冲里面的数据清掉
    // 获取轮式里程计和相机控制
   // ROS_INFO("CollectDataState 00000000");
    vision_msgs::ArucoMarkersMapPose arucoMarkersMapPoseTemp;
    odomSubPtr_->parseData(currentWheelEncoderOdom_);
    cameraControlSubPtr_->parseData(cameraControl_);
    endVisionEnableSubPtr_->parseData(isEndVisionEnable_);

   // ROS_INFO("CollectDataState 11111111");
    // 是否读到数据
    bool bufferHasData = false;
    cameraEnum cameraId;
    switch (cameraControl_.controlByte)
    {
        case 1:
            cameraId = RIGHT_CAMERA;
            break;
        case 2:
            cameraId = LEFT_CAMERA;
            break;
        case 4:
            cameraId = REAR_CAMERA;
            break;
        default:
            // 默认是右边相机
            cameraId = RIGHT_CAMERA;
            break;    
    }
  
      //ROS_INFO("CollectDataState cameraId: "<<cameraId);
    // 根据不同相机来处理不同的定位缓存
    handleArucoMarkersBuffer(cameraId, bufferHasData, arucoMarkersMapPoseTemp);
    //ROS_INFO("CollectDataState 2222222");
    // 防止在切换相机的时候，数据缓冲没有清理
    static uint8_t lastCameraIndex = 0x01;
    static uint8_t currentCameraIndex = 0x01;
    currentCameraIndex = cameraControl_.controlByte;
    if (lastCameraIndex != currentCameraIndex)
    {
        visionPose2DData_.clearData();
        lastCameraIndex = currentCameraIndex;
        bufferHasData = false;
    }
   //ROS_INFO("CollectDataState 333333333333");
    if (bufferHasData == true)
    {
       // 一旦有数据,检查定位数据是否在指定的地点附近
       /*
        bool isCalibrationPoint = checkAllCalibrationPoint(visionPose2DData_, calibrationPoints_, currentWheelEncoderOdom_);
        if (isCalibrationPoint)
        {
            // 将定位缓存起来的同时将速度也缓存起来
            speedBuffer_.push_back(currentWheelEncoderOdom_.twist.twist.linear.x);
            visionLocBuffer_.push_back(visionPose2DData_);
        }*/

        // maually check
        if (pose2dPubPtr_->HasSubscribers())
        {
            pose2dPubPtr_->Publish(visionPose2DData_);
        }
        //ROS_INFO("2222222222222");
        // 当末端（起始和结束）需要定位的时候 主动获取定位  没有条件限制
        if (isEndVisionEnable_.data)
        {
            ROS_INFO("2222233333333333");
            cout<<"isEndVisionEnable_**************888"<<endl;
            if (poseCalibrationPubPtr_->HasSubscribers())
            {
                // 2021年3月21日12:10:13，镇江场景
                 cout<<"poseCalibrationPubPtr_->HasSubscribers()**************888"<<endl;
                VisionPose2DData temp = visionPose2DData_;
                if (temp.theta > 180)
                    temp.theta = temp.theta-360;         
                    
                                
                poseCalibrationPubPtr_->PublishData(temp, arucoMarkersMapPoseTemp.header.stamp);
            } 
        }
        
     //   ROS_INFO("33333333333");
    }

    // 如果定位超时，说明已经了一段二维码，这个时候就可以将视觉定位用来标定轮式里程计  放送给底层  跟新轮式里程计
    // 没有定位的时候这里会有问题
    ros::Time currentTime = ros::Time::now();
    if (currentTime - latesdLocTime_ > ros::Duration(totalToleranceTime_))
    {
        if (visionLocBuffer_.size() > 0)
        {
            // for (auto i:speedBuffer_)
            // {
            //     ROS_ERROR_STREAM("SPEEDs: " << i << std::endl);
            // }
            float minValue = *std::min_element(speedBuffer_.begin(),speedBuffer_.end()); 
            float maxValue = *std::max_element(speedBuffer_.begin(),speedBuffer_.end()); 
            // ROS_ERROR_STREAM("SPEED DIFF: " << maxValue-minValue << std::endl);
            // 加线速度大小判断，防止原地旋转的时候标定
            if ( ((maxValue-minValue) < speedRange_)&& 
                              (fabs(maxValue) > 0.2)&& 
                              (fabs(minValue) > 0.2))
            { 
                ROS_INFO_STREAM("----------------------------");
                ROS_INFO_STREAM("visionLocBuffer_.size : " << visionLocBuffer_.size() << std::endl);
                ROS_INFO_STREAM("----------------------------");
                SwitchToState(CALIBRATE_ODOM_STATE);     //改变状态机状态 发送视觉里程计标定数据
            }
            else
            {
                ROS_ERROR_STREAM("FAILED TO CALIBRATE FOR SPEED " << "maxValue: " << maxValue << "minValue" << minValue);
                visionLocBuffer_.clear();
                speedBuffer_.clear();              
            }       
        } 
    }
}

/**
 * @name: 
 * @brief: 用视觉定位来标定底层里程计的定位，重要的函数   很少发
 * @Date: 2021-03-03 19:27:12
 * @params {*}
 * @return {*}
 */
void VisionLocalizationProcess::CalibrateOdomState()
{
    //return false;
    float angleSum = 0;
    float angleMean = 0;
    auto vectorSize = visionLocBuffer_.size();
    std::vector<VisionPose2DData> goodVisionLocation;
    ros::Time currentVisionCalibrationTime_ = ros::Time::now();
    // 两次视觉标定如果时间太短，那就认为是无效数据
    if ((currentVisionCalibrationTime_ - latestVisionCalibrationTime_) < ros::Duration(5.0))
    {
        ROS_ERROR_STREAM("THE TIME IS TOO CLOSE FOR VISION CALIBRATION!");
        SwitchToState(READY_STATE);
        return; 
    }
    
    latestVisionCalibrationTime_ = currentVisionCalibrationTime_;

    // 检查buffer大小，防止为空
    if (vectorSize == 0)
    {
        SwitchToState(READY_STATE);
        return; 
    }

    // 筛选数据
    for (const auto& localization : visionLocBuffer_ )
    {
        if (localization.quantityOfMarkers >= 1)
        {
            goodVisionLocation.push_back(localization);
        }
    }
    
    auto goodVisionLocationSize = goodVisionLocation.size();
    if (goodVisionLocationSize == 0)
    {
        SwitchToState(READY_STATE);
        return;
    }
        
    for (const auto& localization : goodVisionLocation)
    {
        angleSum += localization.theta;
    }
    angleMean = angleSum/goodVisionLocationSize;
    // 计算标准差
    // to do:可以根据标准差来判断数据的好坏，进行取舍
    double accum = 0;
    for (const auto& localization : goodVisionLocation)
    {
        accum += (localization.theta-angleMean)*(localization.theta-angleMean);
    }
    double stdDev = sqrt(accum/(goodVisionLocationSize-1));
    if (debug_) ROS_INFO_STREAM("stdDev: " << stdDev << endl);

    // 取序列中间的值
    auto index = goodVisionLocationSize/2;

    VisionPose2DData visionPose2DDataResult;
    visionPose2DDataResult.x = goodVisionLocation.at(index).x;
    visionPose2DDataResult.y = goodVisionLocation.at(index).y;
    // 2021年3月21日12:05:42，镇江场景添加
    angleMean>180 ? visionPose2DDataResult.theta = angleMean-360 : visionPose2DDataResult.theta = angleMean; 
    
    
    // 加这个时间判断主要是为了防止在经过二维码的时候刚好因为障碍物停车了
    // 导致运动补偿有问题,小于一定时间说明没有停车而是连续经过二维码
    auto visionPastTime = ros::Time::now() - visionLocBuffer_.at(index).time;
    // ROS_ERROR_STREAM("visionPastTime: " << visionPastTime << std::endl);
    if( visionPastTime < ros::Duration(visionPastTimeLimit_))
    {
        if (poseCalibrationPubPtr_->HasSubscribers())
        {
            poseCalibrationPubPtr_->PublishData(visionPose2DDataResult, visionLocBuffer_.at(index).time);
            cout<<"poseCalibration:"<<visionPose2DDataResult.x<<visionPose2DDataResult.y<<visionPose2DDataResult.theta<<endl;
        } 
    }
    else
    {
        ROS_ERROR_STREAM("FAILED TO CALIBRATE, visionPastTime: " << visionPastTime << std::endl);
    }
    // if (poseCalibrationPubPtr_->HasSubscribers())
    // {
    //     poseCalibrationPubPtr_->PublishData(visionPose2DDataResult, visionLocBuffer_.at(index).time);
    // } 

    // clear the vector buffer
    visionLocBuffer_.clear();
    speedBuffer_.clear();
    SwitchToState(COLLECT_DATA_STATE);   

}
/*
* 文件路径初始化
*/
bool VisionLocalizationProcess::initFilePath()
{
    std::string data_path = WORK_SPACE_PATH;
    if (!FileManager::CreateDirectory(data_path + "/data"))
        return false;
    std::string localization_data_path = data_path + "/data/localization_data";
    if (!FileManager::InitDirectory(localization_data_path, "视觉定位数据"))
        return false;
    if (!FileManager::CreateFile(localization_data_ofs_, localization_data_path + "/localization_data.txt"))
        return false;
    return true;
}

/*
* 参数的初始化
*/
bool VisionLocalizationProcess::initConfig()
{
    std::string configFilePath;
   // node_priv_.param<std::string>("config_file_path", configFilePath, "multi_paths_waypoints.yaml");

   // initCalibrationPoints(YAML::LoadFile(configFilePath));
    // Diagnostics init
    initializeDiagnostics();
    // log file path init
    initFilePath();
    iState = INIT_STATE;
    // 这种类型怎么初始化（非赋值）
    isEndVisionEnable_.data = false;

    return true;
}


/*
* 根据配置文件初始化需要定位的指定区域
*/
/*
bool VisionLocalizationProcess::initCalibrationPoints(const YAML::Node& config_node)
{
    for (auto it = config_node["calibration_points"].begin(); it !=config_node["calibration_points"].end(); ++it)
    {
        CalibrationPoint calibrationPoint;
        calibrationPoint.x = it->second[0].as<float>();
        calibrationPoint.y = it->second[1].as<float>();
        calibrationPoint.theta = it->second[2].as<float>();
        calibrationPoint.isAngleNeedToCalib = it->second[3].as<bool>();
        calibrationPoints_[it->first.as<std::string>()] = calibrationPoint;
    }
    return true;
}*/


/*
* 将PoseStamped消息转换成TF
*/
tf::Transform VisionLocalizationProcess::poseMsg2Tf(const geometry_msgs::PoseStamped &poseMsg)
{
    Eigen::Quaterniond q(poseMsg.pose.orientation.w, poseMsg.pose.orientation.x, \
                            poseMsg.pose.orientation.y, poseMsg.pose.orientation.z);
    Eigen::Matrix3d matrix = q.matrix().cast<double>();
    
    tf::Matrix3x3 tf_rot(matrix(0,0), matrix(0,1), matrix(0,2),
                         matrix(1,0), matrix(1,1), matrix(1,2),
                         matrix(2,0), matrix(2,1), matrix(2,2));

    tf::Vector3 tf_orig(poseMsg.pose.position.x, poseMsg.pose.position.y, poseMsg.pose.position.z);

    return tf::Transform(tf_rot, tf_orig);
}

/*
* 从TF关系得到 X Y Z 和 yaw pitch roll
*/
void VisionLocalizationProcess::TF2XyzYpr(const tf::Transform &transform, Pose& pose)
{
    geometry_msgs::PoseStamped poseMsg;
    tf::poseTFToMsg(transform, poseMsg.pose);

    pose.x = poseMsg.pose.position.x;
    pose.y = poseMsg.pose.position.y;
    pose.z = poseMsg.pose.position.z;

    Eigen::Quaterniond q(poseMsg.pose.orientation.w, poseMsg.pose.orientation.x, \
                            poseMsg.pose.orientation.y, poseMsg.pose.orientation.z);
    Eigen::Matrix3f rotMatrix = q.matrix().cast<float>();;
    Eigen::Vector3f eulerAngle = rotMatrix.eulerAngles(2, 1, 0);// yaw pitch roll

    pose.yaw = eulerAngle.transpose()[0];
    pose.pitch = eulerAngle.transpose()[1];
    pose.roll = eulerAngle.transpose()[2];
}

/*
* TF转换得到X Y YAW
*/
void VisionLocalizationProcess::TF2XyYaw(const tf::Transform &transform, Pose& pose)
{
    //geometry_msgs::PoseStamped poseMsg;
    //tf::poseTFToMsg(transform, poseMsg.pose);
    #if 0
    // msg------------>Eigen
    Eigen::Matrix4f poseMatrix = Eigen::Matrix4f::Identity();
    poseMatrix(0, 3) = poseMsg.pose.position.x;
    poseMatrix(1, 3) = poseMsg.pose.position.y;
    poseMatrix(2, 3) = poseMsg.pose.position.z;
    x = poseMsg.pose.position.x;
    y = poseMsg.pose.position.y;
    z = poseMsg.pose.position.z;
    Eigen::Quaterniond q(poseMsg.pose.orientation.w, poseMsg.pose.orientation.x, \
                            poseMsg.pose.orientation.y, poseMsg.pose.orientation.z);
    Eigen::Matrix3f rotMatrix = q.matrix().cast<float>();;
    Eigen::Vector3f eulerAngle = rotMatrix.eulerAngles(2, 1, 0);// yaw pitch roll
    //std::cout << "the base pose YPR: " << eulerAngle.transpose()*180.0/PI << std::endl;
    yaw = eulerAngle.transpose()[0]*180/pi_;
    pitch = eulerAngle.transpose()[1]*180/pi_;
    roll = eulerAngle.transpose()[2]*180/pi_;
    yaw = eulerAngle.transpose()[0];
    pitch = eulerAngle.transpose()[1];
    roll = eulerAngle.transpose()[2];
    #endif

    geometry_msgs::Pose poseMsg;
    tf::poseTFToMsg(transform, poseMsg);

    pose.x = poseMsg.position.x;
    pose.y = poseMsg.position.y;
    pose.yaw = tf::getYaw(transform.getRotation());
}



/**
 * @name: 
 * @brief: 求取车体的位姿
 * @Date: 2021-03-08 11:06:13
 * @params {*}
 * @return {*}
 */
bool VisionLocalizationProcess::getBasePose2d(const vision_msgs::ArucoMarkersMapPose& arucoMarkersMapPose, 
                                              const tf::StampedTransform& camera2Base, 
                                              VisionPose2DData &visionPose2DData)
{

    // Eigen::Quaterniond q(arucoMarkersMapPose.pose.pose.orientation.w, 
    //                      arucoMarkersMapPose.pose.pose.orientation.x, 
    //                      arucoMarkersMapPose.pose.pose.orientation.y,
    //                      arucoMarkersMapPose.pose.pose.orientation.z);
                         
    // Eigen::Matrix3d rotMatrix = q.matrix().cast<double>();;
    // Eigen::Vector3d eulerAngle = rotMatrix.eulerAngles(2, 1, 0);// yaw pitch roll  z y x
    // std::cout << "the base pose YPR: " << eulerAngle.transpose()*180.0/pi_ << std::endl;

    // Eigen matrix ------> TF
    tf::Transform marker2Camera = poseMsg2Tf(arucoMarkersMapPose.pose);
    tf::Transform world2Base = static_cast<tf::Transform>(world2Marker_) * marker2Camera * static_cast<tf::Transform>(camera2Base);

    // float x,y,yaw;
    Pose pose;
    // tf转换为x y yaw（rad）
    TF2XyYaw(world2Base, pose);
    // fill the vsion data
    visionPose2DData.time = arucoMarkersMapPose.header.stamp;
    visionPose2DData.quantityOfMarkers = arucoMarkersMapPose.quantityOfMarkers;
    visionPose2DData.x = pose.x;
    visionPose2DData.y = pose.y;
    // if (pose.yaw*180/pi_ < -135)
    // 2021年3月21日12:01:35 镇江现场有±180°情况  yaw绕z轴偏移角度  
    pose.yaw*180/pi_ < -135 ? visionPose2DData.theta = pose.yaw*180/pi_ + 360 : visionPose2DData.theta = pose.yaw*180/pi_;    
    // visionPose2DData.theta = pose.yaw*180/pi_;

   return true;
}


/*
* brief：将odom消息有关位姿的信息转换成eigen形式的矩阵
* eigenMatrix.inverse()
*/
bool VisionLocalizationProcess::convertOdom2EigenMatrix( const nav_msgs::Odometry &odomMsg,
                                                   Eigen::Matrix4f &eigenMatrix)
{
    eigenMatrix = Eigen::Matrix4f::Identity();
    eigenMatrix(0, 3) = odomMsg.pose.pose.position.x;
    eigenMatrix(1, 3) = odomMsg.pose.pose.position.y;
    eigenMatrix(2, 3) = odomMsg.pose.pose.position.z;

    Eigen::Quaterniond q(odomMsg.pose.pose.orientation.w, odomMsg.pose.pose.orientation.x, \
                         odomMsg.pose.pose.orientation.y, odomMsg.pose.pose.orientation.z);
    eigenMatrix.block(0,0,3,3) = q.matrix().cast<float>();
    return true;
}

/*
* 由TF关系得到base在world坐标系的pose信息
*/
void VisionLocalizationProcess::getBasePoseStamped(const geometry_msgs::PoseStamped& cameraPose, 
                                                   const tf::StampedTransform& world2Marker, 
                                                   const tf::StampedTransform& camera2Base, 
                                                   geometry_msgs::PoseStamped& basePoseStamped)
{
    tf::Transform marker2Camera = poseMsg2Tf(cameraPose);
    tf::Transform world2Base = static_cast<tf::Transform>(world2Marker) * marker2Camera * static_cast<tf::Transform>(camera2Base);
    tf::poseTFToMsg(world2Base, basePoseStamped.pose);
}
/*
* 根据当前轮式里程计的值（主要是速度）和设置好的点位信息，来判断当前视觉定位是否用于标定里程计，
// * 角度为±180°的时候会有问题，将角度取绝对值，但是角度为﹣90°取绝对值就有问题
* input:current vision location ,target point, current odom(especially speed)
*/
bool VisionLocalizationProcess::calibratePoseByCordinate(const VisionPose2DData& visionPose2DData,
                                                         const CalibrationPoint& point,
                                                         const nav_msgs::Odometry& currentWheelEncoderOdom)
{
    if (fabs(currentWheelEncoderOdom.twist.twist.angular.z) >toleranceAngularSpeed_)
        return false;
    if (fabs(currentWheelEncoderOdom.twist.twist.linear.x) > toleranceLinearSpeed_)
        return false;
    if ( (visionPose2DData.x<(point.x-toleranceDistance_)) || (visionPose2DData.x>(point.x+toleranceDistance_)))
        return false;
    if ((visionPose2DData.y<(point.y-toleranceDistance_)) || (visionPose2DData.y>(point.y+toleranceDistance_)))
        return false;
    // float thetaTemp = fabs(visionPose2DData.theta);
    // visionPose2DData.theta < -90 ? thetaTemp = visionPose2DData.theta + 360 : thetaTemp = visionPose2DData.theta;
    
    //2021年3月22日13:11:33： 感觉没必要检查角度，主要检查一下位置即可，注释掉
    // if(visionPose2DData.theta<point.theta-toleranceAngle_ || visionPose2DData.theta>point.theta+toleranceAngle_)
    //     return false;
    
    // ROS_INFO("Calibrate the wheel ecnoder odom for pose!");
    return true;
}


/*
* 根据配置文件里面的第三个参数为true或者false来判断是否角度也需要标定
* 目前定位数据求平均，角度都是需要标定的
*/
bool VisionLocalizationProcess::checkAllCalibrationPoint(VisionPose2DData &visionPose2DData,
                                                         const std::map<std::string, CalibrationPoint> &calibrationPointsMap,
                                                         const nav_msgs::Odometry &currentWheelEncoderOdom)
{
    for (auto iter = calibrationPointsMap.begin(); iter !=  calibrationPointsMap.end(); iter++)
    {
        CalibrationPoint calibrationPoint = iter->second;
        if (calibratePoseByCordinate(visionPose2DData, calibrationPoint, currentWheelEncoderOdom))
            return true;   
        // CalibrationPoint calibrationPoint = iter->second;
        // if (calibrationPoint.isAngleNeedToCalib)
        // {
        //     if (calibratePoseByCordinate(visionPose2DData, calibrationPoint, currentWheelEncoderOdom))
        //         return true;    
        // }
        // else
        // {
        //     if (calibratePositionByCordinate(visionPose2DData, calibrationPoint, currentWheelEncoderOdom))
        //         return true;   
        // }
    }
    return false;
}


/*
* 判断当前视觉定位是否用于标定里程计 only position
* input:current vision location ,target point, current odom(especially speed)
* 为什么需要设置点位？因为并不是所有的视觉定位信息都是可靠的，比如说角度大、距离远的情况下定位信息往往不可靠。
* 这样有一个风险：如果小车偏离预定轨迹太多（包括距离和角度），这个时候定位是得不到校准的，这个时候只能判断为异常
*/
bool VisionLocalizationProcess::calibratePositionByCordinate(VisionPose2DData &visionPose2DData, 
                                                             const CalibrationPoint &point,  
                                                             const nav_msgs::Odometry &currentWheelEncoderOdom)
{
    if (fabs(currentWheelEncoderOdom.twist.twist.linear.x) > 1.0)
        return false;
    if ( (visionPose2DData.x<(point.x-toleranceDistance_)) || (visionPose2DData.x>(point.x+toleranceDistance_)))
        return false;
    if ((visionPose2DData.y<(point.y-toleranceDistance_)) || (visionPose2DData.y>(point.y+toleranceDistance_)))
        return false;
    if((visionPose2DData.theta < (point.theta-toleranceAngle_)) || (visionPose2DData.theta>(point.theta+toleranceAngle_)))
        return false;
    visionPose2DData.theta = 255.0f;
    // ROS_INFO("Calibrate the wheel ecnoder odom for position!");
    return true;
}


/*
* 判断当前视觉定位是否用于标定里程计
* input:current vision location ,target point, current odom(especially speed)
* 目前是没有用上的
*/
bool VisionLocalizationProcess::calibrateOdomByDisatnce(const VisionPose2DData &visionPose2DData, 
                                                        const CalibrationPoint &point,  
                                                        const nav_msgs::Odometry &currentWheelEncoderOdom)
{
    float xBias = visionPose2DData.x - point.x;
    float yBias = visionPose2DData.y - point.y;
    // 如果满足视觉标定里程计的要求，就发送该话题，另外的节点接受该话题并修改底层控制器的
    // 满足两个条件：（1）在冗余范围内 （2）角速度比较小，防止旋转+时延产生比较大角度的偏差
    if ( sqrt(xBias*xBias + yBias*yBias) < toleranceDistanceL2_)
    {
        if (fabs(currentWheelEncoderOdom.twist.twist.linear.x) <= toleranceLinearSpeed_)
        {
            if (fabs(currentWheelEncoderOdom.twist.twist.angular.z) <= toleranceAngularSpeed_)
            {  
                ROS_INFO("Calibrate the wheel ecnoder odom!");
                return true;
            }
        }
    }
    return false;
}


/*
* tf init，tf初始化，之前就是因为world2MarkerTransformReceived_这些bool型的变量没有初始化
* 导致定位异常，查bug一天
*/
bool VisionLocalizationProcess::initCalibration(void)  
{
    //shared_ptr<TFSimple> 
    if (world2MarkerPtr_->getTransform(world2Marker_))    // 获取   tf::StampedTransform 返回true
    {
        isWorld2MarkerTfReady_ = true;
    }

    
   bool  camerasToBaseReady=true;
    for (std::size_t i=0; i<cameraToBaseTfPtrVector_.size(); ++i)
    {
        if (cameraToBaseTfPtrVector_.at(i)->getTransform(cameraToBaseTfVector_.at(i)))      //:vector<tf::StampedTransform>   cameraToBaseTfVector_
        {
            isCameraToBaseTfReady_.at(i) = true;            
        }
        camerasToBaseReady  = camerasToBaseReady&& isCameraToBaseTfReady_.at(i) ;

    }
    // return true;   
    /*
    return isWorld2MarkerTfReady_&&
           isCameraToBaseTfReady_[0]&&
           isCameraToBaseTfReady_[1]&&
           isCameraToBaseTfReady_[2];*/
      cout<<"isWorld2MarkerTfReady_:"<<isWorld2MarkerTfReady_<<"camerasToBaseReady:    "<<camerasToBaseReady<<endl;
      return isWorld2MarkerTfReady_&& camerasToBaseReady;
           
}



/*
* 标定相机的外参
* B  B W M
  C  W M C
*/
/**
 * @name: 
 * @brief: 标定相机的外参
 * @Date: 2021-03-03 19:29:15
 * @params {*}
 * @return {*}
 */
bool VisionLocalizationProcess::cameraCalibration(void) const
{
    /*
    static bool tfBase2WorldRecevied = false;
    static bool tfWorld2MarkerRecevied = false;
    static tf::StampedTransform tfBase2World;
    static tf::StampedTransform tfWorld2Marker;
    //tf::Transform tfMarker2Camera = poseMsg2Tf(cameraPose);
    float x, y, z, yaw, pitch, roll;
    tf::Transform calibrationBaseToCamera;

    if (!tfBase2WorldRecevied) 
    {
        if (tfBase2WorldPtr_->getTransform(tfBase2World)) 
        {
            tfBase2WorldRecevied = true;
        }
    }

    if (!tfWorld2MarkerRecevied) 
    {
        if (world2MarkerPtr_->getTransform(tfWorld2Marker)) 
        {
            tfWorld2MarkerRecevied = true;
        }
    }
    // 如果接收到的全部的tf消息
    // 就可以根据相机在二维码坐标系的姿态将相机相对于车体的姿态结算出来
    // 更科学的办法应该是用最优化的方法
    if (tfBase2WorldRecevied && tfWorld2MarkerRecevied)
    {
        geometry_msgs::PoseStamped cameraPose;
        camera0Pose3DSubPtr_->parseData(cameraPose);
        double currentTime = ros::Time::now().toSec();
        if (fabs(currentTime - cameraPose.header.stamp.toSec()) > TOLERANCE_TIME)
        {
            return false;
        }
        tf::Transform tfMarker2Camera = poseMsg2Tf(cameraPose);

        calibrationBaseToCamera = static_cast<tf::Transform>(tfBase2World) * static_cast<tf::Transform>(tfWorld2Marker) * tfMarker2Camera;
        TF2XyzYpr(calibrationBaseToCamera, x, y, z, yaw, pitch, roll);
        ROS_INFO_STREAM("TF of base to camera: " << x << " "<< y << " " << z << " " << roll << " " << pitch << " " << yaw << std::endl);
        return true;
    }
    else
    {
        return false;
    }
    */
    return true;

}


/**
 * @name: 
 * @brief: 初始状态
 * @Date: 2021-03-03 19:29:02
 * @params {*}
 * @return {*}
 */
void VisionLocalizationProcess::InitState()
{
    // 在初始化状态里面判断是否收到全部的tf消息
    // 收到就进入READY_STATE
    if (initCalibration()) 
    {
       SwitchToState(READY_STATE);
    }
}

/**
 * @name: 
 * @brief: 预备状态
 * @Date: 2021-03-03 19:28:47
 * @params {*}
 * @return {*}
 */
void VisionLocalizationProcess::ReadyState()
{
    visionLocBuffer_.clear();
    // goodVisionLocation.clear();
    SwitchToState(COLLECT_DATA_STATE);
}

/**
 * @name: 
 * @brief: 每个周期都会执行的任务
 * @Date: 2021-03-03 19:28:25
 * @params {*}
 * @return {*}
 */
void VisionLocalizationProcess::AllState()
{
    // to-do 
    //ros::time 
    ros::Time currentTime = ros::Time::now();
    static ros::Time lastTime = ros::Time::now();
    ros::Duration periodTime = currentTime - lastTime;
    double freqency = periodTime.toSec() ? 1/periodTime.toSec() : std::numeric_limits<double>::max();
    //std::cout << "freqency: " << freqency << std::endl;
    lastTime = currentTime;
    softwareStatusTask_.updateControlFrequency(freqency);
    updateDiagnostics();
}

/**
 * @name: handleArucoMarkerBuffer
 * @brief: 根据不同的相机序号来处理定位数据
 * @Date: 2021-03-02 11:28:57
 * @params {*}
 * @return {*}
 */
void VisionLocalizationProcess::handleArucoMarkersBuffer(cameraEnum cameraId, 
                                                         bool& bufferHasData,
                                                         vision_msgs::ArucoMarkersMapPose& arucoMarkersMapPose)
{
     //std::cout << "cameraId: " << cameraId << std::endl;
     //std::cout << "arucoMarkersPtrVector_  size: " << arucoMarkersPtrVector_.size() << std::endl;   // size 为0 出错
    arucoMarkersPtrVector_.at(cameraId)->parseData(arucoMarkersBuffer_.at(cameraId));
      //ROS_INFO("handleArucoMarkersBuffer ing") ;
    if (arucoMarkersBuffer_.at(cameraId).size()>0)
    {
        arucoMarkersMapPose = arucoMarkersBuffer_.at(cameraId).front();
        arucoMarkersBuffer_.at(cameraId).pop_front();
        latesdLocTime_ = arucoMarkersMapPose.header.stamp;
        getBasePose2d(arucoMarkersMapPose, cameraToBaseTfVector_.at(cameraId), visionPose2DData_);
        bufferHasData = true;
    }
   //ROS_INFO("handleArucoMarkersBuffer finish") ;
}




// diagnostics 
void VisionLocalizationProcess::initializeDiagnostics()
{
    diagnosticUpdater_.setHardwareID(hardwareID_);
    diagnosticUpdater_.add(softwareStatusTask_);
}

// diagnostics 
void VisionLocalizationProcess::updateDiagnostics() 
{
    diagnosticUpdater_.force_update();
}
}