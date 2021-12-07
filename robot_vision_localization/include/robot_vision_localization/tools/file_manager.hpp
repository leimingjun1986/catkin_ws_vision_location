/*
 * @Descripttion: 
 * @version: 
 * @Author: pifan
 * @Date: 2021-01-06 14:29:24
 * @LastEditors: Please set LastEditors
 * @LastEditTime: 2021-03-24 16:43:20
 */
#ifndef AGV_NAVIGATION_TOOLS_FILE_MANAGER_HPP_
#define AGV_NAVIGATION_TOOLS_FILE_MANAGER_HPP_

#include <string>
#include <iostream>
#include <fstream>

// class中static的意思是什么？
namespace robot_vision_localization {
class FileManager{
  public:
    static bool CreateFile(std::ofstream& ofs, std::string file_path);
    static bool InitDirectory(std::string directory_path, std::string use_for);
    static bool CreateDirectory(std::string directory_path, std::string use_for);
    static bool CreateDirectory(std::string directory_path);
};
}

#endif
