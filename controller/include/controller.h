#ifndef CONTROLLER_H
#define CONTROLLER_H
#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/convert.h"
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/QR>
#include "controller_msgs/myimu.h"
#include <sensor_msgs/Imu.h>


typedef struct 
{
  float roll;
  float pitch;
  float yaw;
    
} angle;

typedef struct
{
  float j0;
  float j1;
  float j2;
  float j3;
  float j4;
  float j5;
} joint ;


class imu_controller
{
public:
    imu_controller() {
        // 构造函数中进行初始化
        P1_init << 300.0, 0.0, 0.0;
        P2_init << 250.0, 0.0, 0.0;
        P1 << 0.0, 0.0, 0.0;
        P2 << 0.0, 0.0, 0.0;
        // 其他初始化操作
    }
void orien2euler(angle &_angle,geometry_msgs::Quaternion quaternion);
void updata(void);
void imu2joint(void);
// 绕yaw、pitch和roll轴旋转的仿射矩阵
Eigen::Affine3d rotateYawPitchRollAffine(double yaw, double pitch, double roll) {
    Eigen::Affine3d affine_matrix;
    
    // 绕yaw轴旋转
    Eigen::Matrix3d rotation_yaw;
    rotation_yaw = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());

    // 绕pitch轴旋转
    Eigen::Matrix3d rotation_pitch;
    rotation_pitch = Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY());

    // 绕roll轴旋转
    Eigen::Matrix3d rotation_roll;
    rotation_roll = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());

    // 将三个旋转矩阵相乘得到总的旋转矩阵
    Eigen::Matrix3d total_rotation = rotation_yaw * rotation_pitch * rotation_roll;

    // 将旋转矩阵和平移向量设置到仿射矩阵中
    affine_matrix.linear() = total_rotation;
    // 设置平移向量为零，可以根据需要进行调整
    affine_matrix.translation().setZero();

    return affine_matrix;
}

// 将平移向量转换为只包含平移的放射变换矩阵
Eigen::Affine3d translationMatrix(const Eigen::Vector3d& translation) {
    Eigen::Affine3d affine_matrix;

    // 设置平移向量
    affine_matrix.translation() = translation;

    // 设置旋转部分为单位矩阵
    affine_matrix.linear() = Eigen::Matrix3d::Identity();

    return affine_matrix;
}

Eigen::Vector3d R2ZYZ(const Eigen::Matrix3d& R) {
    Eigen::Vector3d zyz;

    if (R(2, 2) != 1 && R(2, 2) != -1) {
        zyz(0) = std::atan2(R(1, 2) / std::sin(std::acos(R(2, 2))), R(0, 2) / std::sin(std::acos(R(2, 2))));
        zyz(1) = std::atan2(std::sqrt(R(2, 0) * R(2, 0) + R(2, 1) * R(2, 1)), R(2, 2));
        zyz(2) = std::atan2(R(2, 1) / std::sin(std::acos(R(2, 2))), R(2, 0) / std::sin(std::acos(R(2, 2))));
    } else {
        if (R(2, 2) == 1) {
            zyz(0) = 0;
            zyz(1) = 0;
            zyz(2) = std::atan2(-R(0, 1), R(0, 0));
        }
        if (R(2, 2) == -1) {
            zyz(0) = 0;
            zyz(1) = 0;
            zyz(2) = std::atan2(-R(0, 1), R(0, 0));
        }
    }
    return zyz;
}

controller_msgs::myimu myimu;
private:
joint _joint;
angle imu1_angle;
angle imu2_angle;
angle imu3_angle;
Eigen::Vector3d P1_init;
Eigen::Vector3d P2_init;
Eigen::Vector3d P1;
Eigen::Vector3d P2;
Eigen::Vector3d P3;
Eigen::Vector3d ZYZ;
Eigen::Matrix3d R1;
Eigen::Matrix3d R2;
Eigen::Matrix3d R3;
Eigen::Matrix3d R04;
Eigen::Matrix3d R04_1;
Eigen::Matrix3d R46;
Eigen::Matrix3d R06;

Eigen::Affine3d T;
Eigen::Affine3d T01;
Eigen::Affine3d T12;
Eigen::Affine3d T23;
Eigen::Affine3d T34;
Eigen::Affine3d T04;


};



#endif