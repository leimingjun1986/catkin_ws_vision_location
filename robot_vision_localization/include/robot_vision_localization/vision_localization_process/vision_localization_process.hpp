/*
 * @Description: markers localization process module
 * @Author: PiFan
 * @Date: 2020-11-27
 */
#ifndef LIDAR_LOCALIZATION_VISION_LOCALIZTION_PROCESS_HPP_
#define LIDAR_LOCALIZATION_VISION_LOCALIZTION_PROCESS_HPP_

#include <ros/ros.h>

//Eigen
#include <Eigen/Dense>
// tf listener
#include "robot_vision_localization/tf_listener/tf_simple.hpp"
// subscriber
#include "robot_vision_localization/sensor_data/vision_pose2D_data.hpp"

// publisher
#include "robot_vision_localization/publisher/path_publisher.hpp"
#include "robot_vision_localization/publisher/pose2d_publisher.hpp"
#include "robot_vision_localization/publisher/pose2d_stamped_publisher.hpp"
#include "robot_vision_localization/publisher/camera_control_publisher.hpp"
#include "robot_vision_localization/publisher/poseCovStamped_publisher.hpp"
#include "robot_vision_localization/publisher/marker_publisher.hpp"

// yaml 
#include <yaml-cpp/yaml.h>
#include <math.h>
#include "state_machine/Component.h"
#include "diagnostics/agv_diagnostics.hpp"
#include "robot_vision_localization/tools/file_manager.hpp"
#include "robot_vision_localization/template/subscriber_single_template.hpp"
#include "robot_vision_localization/template/subscriber_deque_template.hpp"
#include <std_msgs/Bool.h>
#include <vision_msgs/CameraControl.h>
#include <vision_msgs/ArucoMarkersMapPose.h>
#include <nav_msgs/Odometry.h>

namespace robot_vision_localization 
{
class VisionLocalizationProcess : public Component
{
public:
    struct CalibrationPoint
    {
        float x;
        float y;
        float theta;
        bool isAngleNeedToCalib;
    };

  //类参数接口
     struct VisionLocalizationParam
    {
         std::vector<std::string>  MarkPoseTopics;
         std::string  EndVisionEnableTopic;
         std::pair<std::string,  std::string>      tfBaseToWorld;  //tf  父子坐标系名称对
         std::pair<std::string,  std::string>      tfWorldToMarker;  //  tf  
         std::vector<std::pair<std::string,  std::string>>    tfCamerasToBase;   //
         std::string   OdomTopic;
         std::string   CameraControlTopic;
         std::string   BasePoseTopic;     //车体姿态话题
         std ::string  PoseCalibrationTopic;  //加了限制条件的车体姿态话题
         std::string   BasePoseCovStampTopic;   // 没有用到
          std::string  frame_id;   // 没有用到
         int CamNum;
         bool debug;
    };

    struct Pose
    {
        float x;
        float y;
        float z;
        float roll;
        float pitch;
        float yaw;
    };
    using ArucoSub = SubscriberDeque<vision_msgs::ArucoMarkersMapPose>;
    enum cameraEnum
    {
        RIGHT_CAMERA = 0,
        LEFT_CAMERA = 1,
        REAR_CAMERA = 2 
    };

public:
    //ros::NodeHandle node_priv;
    VisionLocalizationProcess(ros::NodeHandle& nh, std::string hardwareID, double desired_freq,VisionLocalizationParam& param );
    VisionLocalizationProcess() = default;
    void run (void) const;
    bool cameraCalibration (void) const;
    void ControlThread() override;
    // 更新诊断
    void updateDiagnostics ();
private:
    bool initCalibration(void);
    tf::Transform poseMsg2Tf(const geometry_msgs::PoseStamped& poseMsg);
    void TF2XyzYpr(const tf::Transform& transform,  Pose& pose);
    void TF2XyYaw(const tf::Transform& transform, Pose& pose);
    bool getBasePose2d(const vision_msgs::ArucoMarkersMapPose& arucoMarkersMapPose, 
                       const tf::StampedTransform& camera2Base,
                       VisionPose2DData& visionPose2DData);
    void getBasePoseStamped(const geometry_msgs::PoseStamped& cameraPose, 
                            const tf::StampedTransform& world2Marker, 
                            const tf::StampedTransform& camera2Base, 
                            geometry_msgs::PoseStamped& basePoseStamped);
    bool calibratePoseByCordinate(const VisionPose2DData& visionPose2DData, 
                                  const CalibrationPoint& point,  
                                  const nav_msgs::Odometry& currentWheelEncoderOdom);
    bool calibratePositionByCordinate(VisionPose2DData& visionPose2DData, 
                                      const CalibrationPoint& point,  
                                      const nav_msgs::Odometry& currentWheelEncoderOdom);
    bool calibrateOdomByDisatnce(const VisionPose2DData& visionPose2DData, 
                                 const CalibrationPoint& point,  
                                 const nav_msgs::Odometry& currentWheelEncoderOdom);
    bool checkAllCalibrationPoint(VisionPose2DData& visionPose2DData, 
                                  const std::map<std::string, CalibrationPoint>& calibrationPointsMap, 
                                  const nav_msgs::Odometry& currentWheelEncoderOdom);	
    bool convertOdom2EigenMatrix( const nav_msgs::Odometry& odomMsg, 
                                  Eigen::Matrix4f& eigenMatrix);
    // 初始化诊断
    bool initConfig();
    void initializeDiagnostics();
    bool initCalibrationPoints(const YAML::Node& config_node);
    bool initFilePath();
    void handleArucoMarkersBuffer(cameraEnum cameraId, bool& bufferHasData, vision_msgs::ArucoMarkersMapPose& arucoMarkersMapPose);

    void InitState() override;
    void ReadyState() override;
    void AllState() override;
    // 检测到二维码时候的状态
    void CollectDataState() override;
    // 根据检测到二维码的定位数据标定里程计的状态
    void CalibrateOdomState() override;
                                                    
					   
private:
    //double desired_freq_;
    ros::NodeHandle node_nh_;
    ros::NodeHandle node_priv_;

    const double pi_ = 3.14159265358979;
    const float totalToleranceTime_ = 0.6;// 0.3-->0.6镇江场景，距离二维码太近
    const float singleToleranceTime_ = 0.2;
    const float bigAngle_ = 5.0;
    const float toleranceDistance_ = 1; // 0.5
    const float toleranceAngle_ = 5.0 ;
    const float toleranceDistanceL2_ = 0.5;
    const float toleranceLinearSpeed_ = 0.6;
    const float toleranceAngularSpeed_ = 0.01;
    const float visionPastTimeLimit_ = 6.0;
    const float speedRange_ = 0.1;

    double desired_freq_;

    std::vector<std::shared_ptr<SubscriberDeque<vision_msgs::ArucoMarkersMapPose>>> arucoMarkersPtrVector_;
    std::shared_ptr<SubscriberSingle<std_msgs::Bool>> endVisionEnableSubPtr_;
    // for calibration
    std::shared_ptr<TFSimple> tfBase2WorldPtr_;
    // 订阅轮式里程计话题
    std::shared_ptr<SubscriberSingle<nav_msgs::Odometry>> odomSubPtr_;
    // 订阅camera_control话题
    std::shared_ptr<SubscriberSingle<vision_msgs::CameraControl>> cameraControlSubPtr_;
    // publisher
    std::shared_ptr<Pose2dPublisher> pose2dPubPtr_ ;
    // 发送给底层控制器，用于标定里程计的视觉定位信息
    std::shared_ptr<Pose2dPublisher> odomPoseCalibrationPubPtr_;
    std::shared_ptr<Pose2dStampedPublisher> poseCalibrationPubPtr_;
    std::shared_ptr<PoseCovStampedPublisher> poseCovStampedPubPtr_;
    // std::shared_ptr<MarkerPublisher> markerPubPtr_ = std::make_shared<MarkerPublisher>(node_nh_, "marker", "map", 1);

    // 类类型对象成员的初始化总结如下：
    // 用下面这种初始化的方式也是编译没有问题的
    // std::shared_ptr<Pose2dPublisher> pose2dPubPtr_ = std::make_shared<Pose2dPublisher>(node_nh_, "basePose2d", 1);
    // 以下这两种形式的初始化都是可以编译通过的
    // Pose2dPublisher testClass1_ = Pose2dPublisher(node_nh_, "basePose2d", 1);
    // Pose2dPublisher testClass2_{node_nh_, "basePose2d", 1};
    // 这种出初始化形式是编译通不过的
    // Pose2dPublisher testClass3_(node_nh_, "basePose2d", 1);

    // TF
    std::shared_ptr<TFSimple> world2MarkerPtr_;
    std::vector<std::shared_ptr<TFSimple>> cameraToBaseTfPtrVector_;

    bool isWorld2MarkerTfReady_{false};
    std::vector<bool> isCameraToBaseTfReady_{false, false, false};   //3个相机的状态

    // tf以及各个坐标
    tf::StampedTransform world2Marker_;
    std::vector<tf::StampedTransform> cameraToBaseTfVector_;
    // std::vector<std::string> cameraPose3DTopics_;
    std::vector<std::deque<vision_msgs::ArucoMarkersMapPose>> arucoMarkersBuffer_;
    geometry_msgs::PoseStamped cameraPose_;
    VisionPose2DData visionPose2DData_;
    nav_msgs::Odometry currentWheelEncoderOdom_;

    // as the name
    ros::Time latesdLocTime_{0.0};
    vision_msgs::CameraControl cameraControl_;
    // vison localization deque
    std::vector<VisionPose2DData> visionLocBuffer_;
    std::vector<float> speedBuffer_;

    std::ofstream localization_data_ofs_;

    // YAML::Node configNode_;
    std::map<std::string, CalibrationPoint> calibrationPoints_;
    std_msgs::Bool isEndVisionEnable_;

    std::string hardwareID_;
    diagnostic_updater::Updater diagnosticUpdater_;
    AgvSoftwareDiagnosticsTask softwareStatusTask_;
    // 最近的一次视觉标定有效时间
    ros::Time latestVisionCalibrationTime_;
    bool debug_;
};

class VisionLocatlizationInterFace{

 public:

    int  CamNum;    //相机数目
    boost::shared_ptr<VisionLocalizationProcess>   vision_location_ptr;    //
    VisionLocalizationProcess::VisionLocalizationParam   vision_location_param;   //
   
    VisionLocatlizationInterFace(ros::NodeHandle& nh);
    VisionLocatlizationInterFace()= default;
    ~VisionLocatlizationInterFace();

    bool  import_config(string&   configFilePath_);

};

}

#endif