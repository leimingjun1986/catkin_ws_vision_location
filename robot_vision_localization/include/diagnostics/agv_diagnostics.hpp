/*
 * @Description: diagnistics header file
 * @Author: Pi Fan
 * @Date: 2020-12-25
 */
#ifndef AGV_DIAGNOSTICS_HPP_
#define AGV_DIAGNOSTICS_HPP_

#include <deque>
#include <ros/ros.h>

#include "diagnostic_updater/diagnostic_updater.h"

namespace robot_vision_localization 
{
class AgvSoftwareDiagnosticsTask : public diagnostic_updater::DiagnosticTask
{
public:
    // desired_freq: 期望的发送接收频率
    // minTopicFreq/maxTopicFreq: 需要检测的话题的最小/最大频率
    // hardwareID:设备ID号
    explicit AgvSoftwareDiagnosticsTask(std::string hardwareID, double target_control_freq);
    AgvSoftwareDiagnosticsTask()= default ;
    void run(diagnostic_updater::DiagnosticStatusWrapper &stat) override;

    void updateControlFrequency(double frequency);
private:
    void reset();
    double control_freq_, target_control_freq_;
    const int CONTROLFREQ_WARN = 90;

};


class AgvHardwareDiagnosticsTask : public diagnostic_updater::DiagnosticTask
{
public:
  explicit AgvHardwareDiagnosticsTask(std::string hardwareID);

  void run(diagnostic_updater::DiagnosticStatusWrapper &stat)
  {

  }

};



}
#endif