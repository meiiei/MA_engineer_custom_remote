#include "../include/controller.h"

#define PI               3.14159265358979f


void imu_controller::orien2euler(angle &_angle,geometry_msgs::Quaternion quaternion)
{
    // 使用tf2库将四元数转为欧拉角
    tf2::Quaternion tf2_quaternion;
    tf2::convert(quaternion, tf2_quaternion);

    // 用于存储欧拉角的临时变量
    double roll, pitch, yaw;

    // 调用getRPY函数
    tf2::Matrix3x3(tf2_quaternion).getRPY(roll, pitch, yaw);

    // 将欧拉角赋值给_angle对象
    _angle.roll = roll;
    _angle.pitch = pitch;
    _angle.yaw = yaw;
}

void imu_controller::updata(void)
{
    Eigen::Quaterniond quaternion1(myimu.IMU1.orientation.x, myimu.IMU1.orientation.y, myimu.IMU1.orientation.z, myimu.IMU1.orientation.w); // (w, x, y, z)
    Eigen::Quaterniond quaternion2(myimu.IMU2.orientation.x, myimu.IMU2.orientation.y, myimu.IMU2.orientation.z, myimu.IMU2.orientation.w); // (w, x, y, z)
    Eigen::Quaterniond quaternion3(myimu.IMU3.orientation.x, myimu.IMU3.orientation.y, myimu.IMU3.orientation.z, myimu.IMU3.orientation.w); // (w, x, y, z)
    R1 = quaternion1.toRotationMatrix();
    R2 = quaternion2.toRotationMatrix();
    R3 = quaternion3.toRotationMatrix();
    P1 = R1 * P1_init;
    P2 = R2 * P2_init;
    P3 = P1 + P2;
    T.linear() = R3;
    T.translation() = P3;
}

void imu_controller::imu2joint(void)
{
    if(P3[2]/P3[0]<11.43)
    {
        _joint.j2 = atan(P3[2]/P3[0]);
    }
    else
    {
        _joint.j2 = 60/180*3.14;
    }
    _joint.j0 = P3[2] -249*sin(_joint.j2);
    _joint.j1 = P3[0] -249*cos(_joint.j2);
    Eigen::Vector3d translation1(0,0,0);
    Eigen::Vector3d translation2(0,0,_joint.j0);
    Eigen::Vector3d translation3(0,0,_joint.j1);
    Eigen::Vector3d translation4(0,0,249);

    T01 = rotateYawPitchRollAffine(0,0,0)*translationMatrix(translation1)*rotateYawPitchRollAffine(0,0,0)*translationMatrix(translation2);
    T12 = rotateYawPitchRollAffine(-PI/2,0,0)*translationMatrix(translation1)*rotateYawPitchRollAffine(0,0,0)*translationMatrix(translation3);
    T23 = rotateYawPitchRollAffine(-PI/2,0,0)*translationMatrix(translation1)*rotateYawPitchRollAffine(0,0,_joint.j2)*translationMatrix(translation1);
    T34 = rotateYawPitchRollAffine(PI/2,0,0)*translationMatrix(translation1)*rotateYawPitchRollAffine(0,0,0)*translationMatrix(translation4);
    T04 = T01*T12*T23*T34;
    R04 = T04.linear();
    R04_1 = R04.transpose();
    R06 = T.linear();
    R46 = R04_1 * R06;
    ZYZ = R2ZYZ(R46);
    _joint.j3 = ZYZ[0];
    _joint.j4 = ZYZ[1];
    _joint.j5 = ZYZ[2];
    ROS_INFO("JOINT data: joint0=%f, joint1=%f, joint2=%f, joint3=%f, joint4=%f, joint5=%f",
           _joint.j0,
           _joint.j1,
           _joint.j2,
           _joint.j3,
           _joint.j4,
           _joint.j5);    
}

