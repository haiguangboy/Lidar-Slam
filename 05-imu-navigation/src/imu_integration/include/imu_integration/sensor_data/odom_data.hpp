/*
 * @Description: odometry data
 * @Author: Ge Yao
 * @Date: 2020-11-10 14:25:03
 */
#ifndef IMU_INTEGRATION_ODOM_DATA_HPP_
#define IMU_INTEGRATION_ODOM_DATA_HPP_

#include <Eigen/Dense>
#include <Eigen/Core>
#include <deque>

namespace imu_integration {

struct OdomData {
    double time = 0.0;
    Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
    Eigen::Vector3d vel = Eigen::Vector3d::Zero();
    static bool SyncData(std::deque<OdomData>& UnsyncedData, std::deque<OdomData>& SyncedData, double sync_time){
    // 传感器数据按时间序列排列，在传感器数据中为同步的时间点找到合适的时间位置
    // 即找到与同步时间相邻的左右两个数据
    // 需要注意的是，如果左右相邻数据有一个离同步时间差值比较大，则说明数据有丢失，时间离得太远不适合做差值
    while (UnsyncedData.size() >= 1) {
        if (UnsyncedData.front().time > sync_time)
            return false;
        if (UnsyncedData.at(1).time < sync_time) {
            UnsyncedData.pop_front();
            continue;
        }
        if (sync_time - UnsyncedData.front().time > 0.2) {
            UnsyncedData.pop_front();
            return false;
        }
        if (UnsyncedData.at(1).time - sync_time > 0.2) {
            return false;
        }
        break;
    }
    if (UnsyncedData.size() < 1)
        return false;

    OdomData front_data = UnsyncedData.at(0);
    OdomData back_data = UnsyncedData.at(1);
    OdomData synced_data;
    synced_data.pose = Eigen::Matrix4d::Identity();
    
    double front_scale = (back_data.time - sync_time) / (back_data.time - front_data.time);
    double back_scale = (sync_time - front_data.time) / (back_data.time - front_data.time);
    synced_data.time = sync_time;

    synced_data.pose(0,3) = front_data.pose(0,3) * front_scale + back_data.pose(0,3) * back_scale;
    synced_data.pose(1,3) = front_data.pose(1,3) * front_scale + back_data.pose(1,3) * back_scale;
    synced_data.pose(2,3) = front_data.pose(2,3) * front_scale + back_data.pose(2,3) * back_scale;
    
    Eigen::Quaterniond q_front_data(front_data.pose.block<3,3>(0,0));
    Eigen::Quaterniond q_back_data(back_data.pose.block<3,3>(0,0));
    Eigen::Quaterniond q_synced;
    q_synced.x() = q_front_data.x() * front_scale + q_back_data.x() * back_scale;
    q_synced.y() = q_front_data.y() * front_scale + q_back_data.y() * back_scale;
    q_synced.z() = q_front_data.z() * front_scale + q_back_data.z() * back_scale;
    q_synced.w() = q_front_data.w() * front_scale + q_back_data.w() * back_scale;
    // 线性插值之后要归一化
    synced_data.pose.block<3,3>(0,0) = q_synced.normalized().toRotationMatrix();
    
    synced_data.vel.x() = front_data.vel.x() * front_scale + back_data.vel.x() * back_scale;
    synced_data.vel.y() = front_data.vel.y() * front_scale + back_data.vel.y() * back_scale;
    synced_data.vel.z() = front_data.vel.z() * front_scale + back_data.vel.z() * back_scale;

    SyncedData.push_back(synced_data);
    
    return true;
}    
    
};

} // namespace imu_integration

#endif
