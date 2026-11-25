#ifndef RM_SERIAL_DRIVER__TRAJECTORY_PACKET_HPP_
#define RM_SERIAL_DRIVER__TRAJECTORY_PACKET_HPP_
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include "rm_serial_driver/crc.hpp"
namespace rm_serial_driver
{
struct TrajectoryCommand {
  bool valid=false; uint16_t sequence=0; uint32_t host_ms=0; uint16_t age_ms=0,ttl_ms=40;
  float yaw=0,pitch=0,yaw_speed=0,pitch_speed=0,yaw_acc=0,pitch_acc=0;
};
constexpr std::size_t TRAJECTORY_PACKET_SIZE=38;
inline void write_u16_le(uint8_t *p,uint16_t v){p[0]=static_cast<uint8_t>(v);p[1]=static_cast<uint8_t>(v>>8);}
inline void write_u32_le(uint8_t *p,uint32_t v){write_u16_le(p,static_cast<uint16_t>(v));write_u16_le(p+2,static_cast<uint16_t>(v>>16));}
inline void write_float_le(uint8_t *p,float v){static_assert(sizeof(float)==4,"A6 requires binary32");uint32_t b=0;std::memcpy(&b,&v,4);write_u32_le(p,b);}
inline std::array<uint8_t,TRAJECTORY_PACKET_SIZE> pack_trajectory(const TrajectoryCommand &c){
  if(!c.ttl_ms)throw std::invalid_argument("trajectory ttl must be nonzero");
  std::array<uint8_t,TRAJECTORY_PACKET_SIZE> p{};p[0]=0xA6;p[1]=c.valid?1U:0U;
  write_u16_le(p.data()+2,c.sequence);write_u32_le(p.data()+4,c.host_ms);
  write_u16_le(p.data()+8,c.age_ms);write_u16_le(p.data()+10,c.ttl_ms);
  const float x[]={c.yaw,c.pitch,c.yaw_speed,c.pitch_speed,c.yaw_acc,c.pitch_acc};
  for(std::size_t i=0;i<6;++i){if(!std::isfinite(x[i]))throw std::domain_error("trajectory contains a non-finite value");write_float_le(p.data()+12+4*i,x[i]);}
  crc16::Append_CRC16_Check_Sum(p.data(),p.size());return p;
}
}
#endif
