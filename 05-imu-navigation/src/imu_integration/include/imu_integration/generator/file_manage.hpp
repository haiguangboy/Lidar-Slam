/*
 * @Description: 读写文件管理
 * @Author: Ren Qian
 * @Date: 2020-02-24 19:22:53
 */
#ifndef IMU_INTEGRATION_GENERATOR_ACTIVITY_HPP_
#define IMU_INTEGRATION_GENERATOR_ACTIVITY_HPP_

#include <string>
#include <iostream>
#include <fstream>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

#include "imu_integration/config/config.hpp"

namespace mu_integration {
class FileManager{
  public:
    static bool CreateFile(std::ofstream& ofs, std::string file_path);
    static bool CreateDirectory(std::string directory_path);
};
}

#endif
