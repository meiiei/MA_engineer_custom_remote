#ifndef __MA_SERIAL_H__
#define __MA_SERIAL_H__

#include <serial/serial.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include "controller.h"
namespace ma_serial_packet{

/**
 * SendPacket
 * 
*/

typedef struct
{
  float j0;
  float j1;
  float j2;
  float j3;
  float j4;
  float j5;
} joint ;

struct SendPacket{
    uint8_t head = 0x5A;
    joint   _joint;
    uint16_t checksum = 0;
}__attribute__((packed));


/***
 * @details: Receive odometry msg.
 * @param: head, v_x, v_y, v_z, p_x, p_y, p_z, checksum
*/
struct ReceviePacket{
   uint8_t head = 0x5A;
   int16_t v_x;
   int16_t v_y;
   int16_t a_z;
   int16_t p_x;
   int16_t p_y;
   int16_t p_z;
   uint16_t checksum = 0;
}__attribute__((packed));

/**
 * Alter vector above into struct and utilize it.
*/
inline ReceviePacket fromVector(const std::vector<uint8_t> & data)
{
  ReceviePacket packet;
  std::copy(data.begin(), data.end(), reinterpret_cast<uint8_t *>(&packet));
  return packet;
}

/**
 * Alter struct object above into vector and utilize it.
*/
inline std::vector<uint8_t> toVector(const SendPacket & data)
{
  std::vector<uint8_t> packet(sizeof(SendPacket));
  std::copy(
    reinterpret_cast<const uint8_t *>(&data),
    reinterpret_cast<const uint8_t *>(&data) + sizeof(SendPacket), packet.begin());
  return packet;
}


}

#endif