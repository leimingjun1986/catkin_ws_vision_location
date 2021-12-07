/*
 * @FilePath: /src/hsgn_agv/robot_vision_localization/src/modbus/modbus_slave_rtu.cpp
 * @Description: 
 * @version: 1.0
 * @Author: Zhizhong Jiang
 * @Date: 2021-03-12 16:21:56
 * @LastEditTime: 2021-03-24 16:39:44
 */
#include "robot_vision_localization/modbus/modbus_slave_rtu.h"

namespace robot_vision_localization
{

// 构造函数
ModbusSlaveRtu::ModbusSlaveRtu(std::string portName, int slaveID):slaveID_(slaveID)
{
    //初始化modbus rtu
    ctx_ = modbus_new_rtu(portName.c_str(), 115200, 'N', 8, 1);// /dev/ttysWK0
    //设定从设备地址
    modbus_set_slave(ctx_, slaveID_);
    //modbus_set_debug(ctx_, TRUE);
    modbus_set_response_timeout(ctx_, 0, 20000);
    if (modbus_connect(ctx_) == -1)
    {
        ROS_ERROR_STREAM("Failed to connect as for " <<modbus_strerror(errno) );
    }
    //寄存器map初始化
    mb_mapping_ = modbus_mapping_new(0, 0,MODBUS_MAX_READ_REGISTERS, 0);                             
    if (mb_mapping_ == NULL) 
    {
        printf("Failed to allocate the mapping: %s\n",modbus_strerror(errno));
        modbus_free(ctx_);
        ctx_ = NULL;
    }
    else
    {
        for (size_t i=0; i<MOBUS_TAB_REG_NUM; i++)
        {
             mb_mapping_->tab_registers[0] = 0;
        }
    }
}



ModbusSlaveRtu::~ModbusSlaveRtu()
{
    // For RTU, skipped by TCP (no TCP connect) 
    modbus_close(ctx_);
    if (mb_mapping_ != NULL)
    {
        modbus_mapping_free(mb_mapping_);
        mb_mapping_ = NULL;
    }
    if (ctx_ != NULL)
    {
        modbus_free(ctx_);
        ctx_ = NULL;
    }
    
}

void ModbusSlaveRtu::loop(void)
{
    while( 1 )
    {
        // MODBUS_TCP_MAX_ADU_LENGTH:256
        uint8_t query[MODBUS_TCP_MAX_ADU_LENGTH];
        //轮询接收数据，并做相应处理
        int rc = modbus_receive(ctx_, query);
        if (rc > 0) 
        {
            modbus_reply(ctx_, query, rc, mb_mapping_);
        }
        
        // Connection closed by the client or error 
        else if (rc  == -1) 
        {   
            //LOG(ERROR) << "modbus error: " << modbus_strerror(errno);
            //printf("modbus error: %s\n", modbus_strerror(errno));          
            //exit(1);
        }  
    }
}

}// end of namesapce 