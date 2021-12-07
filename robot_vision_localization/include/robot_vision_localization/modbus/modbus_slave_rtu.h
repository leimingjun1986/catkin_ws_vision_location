/*
 * @FilePath: /src/hsgn_agv/robot_vision_localization/include/robot_vision_localization/modbus/modbus_slave_rtu.h
 * @Description: 
 * @version: 1.0
 * @Author: Zhizhong Jiang
 * @Date: 2021-03-12 16:21:56
 * @LastEditTime: 2021-03-24 16:40:12
 */
#ifndef AGV_NAVIGATION_MODBUS_SLAVE_RTU_HPP_
#define AGV_NAVIGATION_MODBUS_SLAVE_RTU_HPP_

#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include "modbus.h"
#include <memory>  
#include "glog/logging.h"
#include <ros/ros.h>
 
namespace robot_vision_localization
{
class ModbusSlaveRtu
{
public:
    ModbusSlaveRtu(std::string portName, int slaveID);
    ModbusSlaveRtu() = default;
    ~ModbusSlaveRtu();
    void loop(void);
    typedef std::unique_ptr<ModbusSlaveRtu> Ptr;
public:
    modbus_t *ctx_ = NULL;
    modbus_mapping_t *mb_mapping_ = NULL;
private:
    const int slaveID_ = 0x36;
    const size_t MOBUS_TAB_REG_NUM = 24;

};
} // namespace robot_vision_localization
#endif 
