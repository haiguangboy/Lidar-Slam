/*
 * @Description: 读写文件管理
 * @Author: Ren Qian
 * @Date: 2020-02-24 19:22:53
 */

#ifndef IMU_INTEGRATION_FILE_MANAGER_HPP_
#define IMU_INTEGRATION_FILE_MANAGER_HPP_

#include <string>
#include <iostream>
#include <fstream>

namespace imu_integration {
class FileManager{
  public:
    static bool CreateFile(std::ofstream& ofs, std::string file_path);
    static bool CreateDirectory(std::string directory_path);
};
}

#endif
