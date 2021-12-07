#include  <nodelet/nodelet.h>
#include  <robot_vision_localization/vision_localization_process/vision_localization_process.hpp>
 
 namespace  robot_vision_localization{

        // nodelet  接口类
        class  Vision_Localization_Nodelet:public nodelet::Nodelet      
        {
            public:
            Vision_Localization_Nodelet();
            ~ Vision_Localization_Nodelet();
            
            boost::shared_ptr<VisionLocatlizationInterFace>   vision_location_interface_ptr;

            private:
            virtual void onInit();    //此函数声明部分为固定格式，在nodelet加载此plugin会自动执行此函数

        };

 }