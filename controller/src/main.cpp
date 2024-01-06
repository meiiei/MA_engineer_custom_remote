#include "../include/controller.h"




imu_controller imu_controller;

void imu1_work(const sensor_msgs::Imu::ConstPtr& imu_msg)
{
    imu_controller.myimu.IMU1 = *imu_msg;
    // ROS_INFO("Received IMU orientation data: x=%f, y=%f, z=%f, w=%f",
    //        imu_controller.myimu.IMU1.orientation.x,
    //        imu_controller.myimu.IMU1.orientation.y,
    //        imu_controller.myimu.IMU1.orientation.z,
    //        imu_controller.myimu.IMU1.orientation.w);    
}
void imu2_work(const sensor_msgs::Imu::ConstPtr& imu_msg)
{
    imu_controller.myimu.IMU2 = *imu_msg;
    // ROS_INFO("Received IMU orientation data: x=%f, y=%f, z=%f, w=%f",
    //        imu_controller.myimu.IMU1.orientation.x,
    //        imu_controller.myimu.IMU1.orientation.y,
    //        imu_controller.myimu.IMU1.orientation.z,
    //        imu_controller.myimu.IMU1.orientation.w);    
}
void imu3_work(const sensor_msgs::Imu::ConstPtr& imu_msg)
{
    imu_controller.myimu.IMU3 = *imu_msg;
    // ROS_INFO("Received IMU orientation data: x=%f, y=%f, z=%f, w=%f",
    //        imu_controller.myimu.IMU1.orientation.x,
    //        imu_controller.myimu.IMU1.orientation.y,
    //        imu_controller.myimu.IMU1.orientation.z,
    //        imu_controller.myimu.IMU1.orientation.w);    
}
int main(int argc,char **argv)
{
    ros::init(argc,argv,"controller");
    ros::NodeHandle n;
    ros::Subscriber sub1 = n.subscribe("imu1",10,imu1_work);
    ros::Subscriber sub2 = n.subscribe("imu2",10,imu2_work);
    ros::Subscriber sub3 = n.subscribe("imu3",10,imu3_work);


        while (ros::ok()) {

        imu_controller.updata();
        imu_controller.imu2joint();

        ros::spinOnce();
        ros::Duration(0.1).sleep();
        }
}
