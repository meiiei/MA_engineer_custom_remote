#include "../include/controller.h"
#include "../include/ma_serial.hpp"
#include "../include/crc.hpp"


imu_controller imu_controller;
serial::Serial ser;

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
    ros::NodeHandle nh;
    ros::Subscriber sub1 = nh.subscribe("imu1",10,imu1_work);
    ros::Subscriber sub2 = nh.subscribe("imu2",10,imu2_work);
    ros::Subscriber sub3 = nh.subscribe("imu3",10,imu3_work);
    ma_serial_packet::SendPacket packet;
    std::string port;
    int baud_rate;
    if (nh.getParam("/ma_serial/port", port) 
        && nh.getParam("/ma_serial/baud_rate", baud_rate))
    {

    }
        try {
        ser.setPort(port);
        ser.setBaudrate(baud_rate);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
    } catch (serial::IOException& e) {
        ROS_ERROR_STREAM("Unable to open the serial port: " << e.what());
        return 1;
    }

    if (ser.isOpen()) {
        ROS_INFO_STREAM("Serial port initialized.");
    }else{
        return -1;
    }

        while (ros::ok()) {

        imu_controller.updata();
        imu_controller.imu2joint();
        imu_controller.get_joint(packet);
        crc16::append_crc16_checksum(reinterpret_cast<uint8_t *>(&packet), sizeof(packet));
        uint8_t* packet_ptr = reinterpret_cast<uint8_t*>(&packet);
        ser.write(packet_ptr, sizeof(packet));
        ros::spinOnce();
        ros::Duration(0.1).sleep();
        }
}
