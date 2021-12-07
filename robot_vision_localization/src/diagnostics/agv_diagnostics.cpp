/*
 * @Description: simple diagnstics source file
 * @Author: Pi Fan
 * @Date: 2020-12-25
 * @biref:
 *  reference:
 * (1) rcomponent.cpp
 * (2) diagnostic_updater(http://wiki.ros.org/diagnostic_updater?distro=kinetic)
 * (3) husky
 * 最终决定模仿husky代码，hard
 */
#include "diagnostics/agv_diagnostics.hpp"

//#include "glog/logging.h"

namespace robot_vision_localization
{
AgvSoftwareDiagnosticsTask::AgvSoftwareDiagnosticsTask(std::string hardwareID, double target_control_freq): DiagnosticTask(hardwareID), 
                                                                                                            target_control_freq_(target_control_freq)                                                 
{
    reset();
}
void AgvSoftwareDiagnosticsTask::updateControlFrequency(double frequency)
{
    // Keep minimum observed frequency for diagnostics purposes
    assert(frequency != 0);
    //control_freq_ = std::min(control_freq_, frequency);
    control_freq_ = frequency;
}

void AgvSoftwareDiagnosticsTask::run(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
    //msg_.ros_control_loop_freq = control_freq_;
    //stat.add("ROS Control Loop Frequency", msg_.ros_control_loop_freq);
    stat.add("AGV_control_freqency", control_freq_);
    double margin = control_freq_ / target_control_freq_ * 100;

    stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "AGV vision function OK");
    if (margin < CONTROLFREQ_WARN)
    {
        std::ostringstream message;
        //std:: string message;
        message << "Control loop executing " << 100 - static_cast<int>(margin) << "% slower than desired";
        stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::WARN, message.str());
    }


    // ccc->ddd();
    // reset();
}

void AgvSoftwareDiagnosticsTask::reset()
{
    control_freq_ = std::numeric_limits<double>::max();
    
    //target_control_freq_ = 0;
}

}
