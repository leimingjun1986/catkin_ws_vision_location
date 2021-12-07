#include "robot_vision_localization/vision_localization_process/vision_localization_process.hpp"


namespace robot_vision_localization
{
   bool   VisionLocatlizationInterFace::import_config(string&   configFilePath_)
   {
            CamNum=0;
            YAML::Node  configNode_ = YAML::LoadFile(configFilePath_);   //  YAML::Node
            if(configNode_.IsNull())
            {
                    ROS_INFO("VisionLocation  load YAML  File  error!!!!");
                    return false ;
            } 
            YAML::iterator  it;
            it=configNode_["vision_location_config"].begin();
            vision_location_param.CamNum=it->second["camera_num"].as<int>();
            CamNum=it->second["camera_num"].as<int>();
            ROS_INFO(" vision_location_param.CamNum:%d",vision_location_param.CamNum);
            if(CamNum<=0)
            {
                    ROS_INFO("CAMERA_NUM  set  0  or   CAMERA_NUM  load  error !!!!");
                    return false ;
            } 
          //ROS_INFO("111111111111      ");
           vision_location_param.EndVisionEnableTopic=it->second["EndVisionEnableTopic"].as<std::string>();
           ROS_INFO("vision_location_param.EndVisionEnableTopic: %s",vision_location_param.EndVisionEnableTopic.c_str());

           vision_location_param.tfBaseToWorld.first=it->second["tfBaseToWorld"][0].as<std::string>();
           vision_location_param.tfBaseToWorld.second=it->second["tfBaseToWorld"][1].as<std::string>();
           ROS_INFO("vision_location_param.tfBaseToWorld: %s   %s",vision_location_param.tfBaseToWorld.first.c_str(),vision_location_param.tfBaseToWorld.second.c_str());
           vision_location_param.tfWorldToMarker.first=it->second["tfWorldToMarker"][0].as<std::string>();
           vision_location_param.tfWorldToMarker.second=it->second["tfWorldToMarker"][1].as<std::string>();
          ROS_INFO("vision_location_param.tfWorldToMarker: %s   %s",vision_location_param.tfWorldToMarker.first.c_str(),vision_location_param.tfWorldToMarker.second.c_str());
           vision_location_param.OdomTopic=it->second["OdomTopic"].as<std::string>();
           ROS_INFO("vision_location_param.OdomTopic: %s",vision_location_param.OdomTopic.c_str());
           vision_location_param.CameraControlTopic=it->second["CameraControlTopic"].as<std::string>();
           ROS_INFO("vision_location_param.CameraControlTopic: %s",vision_location_param.CameraControlTopic.c_str());
           vision_location_param.BasePoseTopic=it->second["BasePoseTopic"].as<std::string>();
           ROS_INFO("vision_location_param.BasePoseTopic: %s",vision_location_param.BasePoseTopic.c_str());
           vision_location_param.PoseCalibrationTopic=it->second["PoseCalibrationTopic"].as<std::string>();
           ROS_INFO("vision_location_param.PoseCalibrationTopic: %s",vision_location_param.PoseCalibrationTopic.c_str());
           vision_location_param.BasePoseCovStampTopic=it->second["BasePoseCovStampTopic"].as<std::string>();
           vision_location_param.frame_id=it->second["frame_id"].as<std::string>();
        // ROS_INFO("3333333333      ");
          //vision_location_param.tfCamerasToBase.reserve(CamNum);
          //vision_location_param.MarkPoseTopics.reserve(CamNum);
         
         for(int cam_id=0;cam_id<CamNum;cam_id++){
             std::pair<std::string,  std::string>  tfCamerasToBase;
             tfCamerasToBase.first=it->second["tfCamerasToBase"][2*cam_id].as<std::string>();
             tfCamerasToBase.second=it->second["tfCamerasToBase"][2*cam_id+1].as<std::string>();
             vision_location_param.tfCamerasToBase.push_back(tfCamerasToBase);
             std::string  MarkPoseTopic=it->second["markers_map_pose_topic"][cam_id].as<std::string>();
             vision_location_param.MarkPoseTopics.push_back(MarkPoseTopic);
           }

          bool debug_=it->second["debug"].as<bool>();
          ROS_INFO("VisionLocatlizationInterFace   import_config   finish");
          return  true;
}
    
    VisionLocatlizationInterFace:: VisionLocatlizationInterFace(ros::NodeHandle& nh)
    {
      std::string  file_path ="/data/vision_ws/config/vision_location_config.yaml";
      import_config(file_path);
      std::string hardwareID = "vision_state_machine";
      double desired_freq = 30.0;
      vision_location_ptr  =     boost::make_shared<VisionLocalizationProcess>(nh,hardwareID,desired_freq, vision_location_param);
      vision_location_ptr->ControlThread();   
    }

    VisionLocatlizationInterFace::~VisionLocatlizationInterFace(){
        ;
    }
}


