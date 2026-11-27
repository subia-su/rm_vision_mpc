// Host-only vectors: use the production C++ A6 serializer, never a serial port.
#include <cmath>
#include <iomanip>
#include <iostream>
#include "rm_serial_driver/world_trajectory_contract.hpp"
using namespace rm_serial_driver;
struct Fixture {
  const char *name;
  float raw_yaw, world_pitch, mechanical, yaw, pitch, vy, vp, ay, ap;
  int expected_reason;
  bool valid, corrupt;
};
int main()
{
  const Fixture fixtures[] = {
    {"yaw_zero",0,.14F,-.15F,0,.14F,0,0,0,0,1,true,false},
    {"yaw_plus_one",1,.14F,-.15F,1,.14F,0,0,0,0,1,true,false},
    {"yaw_minus_one",-1,.14F,-.15F,-1,.14F,0,0,0,0,1,true,false},
    {"yaw_near_plus_pi",3.13F,.14F,-.15F,3.13F,.14F,0,0,0,0,1,true,false},
    {"yaw_near_minus_pi",-3.13F,.14F,-.15F,-3.13F,.14F,0,0,0,0,1,true,false},
    // Isolate the pi-origin issue from the independent host/MCU sign question.
    {"yaw_sign_isolated",1,.14F,-.15F,-1,.14F,0,0,0,0,1,true,false},
    {"pitch_logged_reachable",0,.14F,-.15F,0,.268172F,0,0,0,0,1,true,false},
    {"pitch_logged_unreachable",0,.14F,-.28F,0,.268172F,0,0,0,0,9,true,false},
    {"derivatives",.3F,.14F,-.15F,.3F,.24F,1.25F,.2F,-2,-.3F,1,true,false},
    {"derivative_limits",.3F,.14F,-.15F,.3F,.24F,5,.6F,20,8,1,true,false},
    {"host_revocation",0,.14F,-.15F,0,.14F,0,0,0,0,2,false,false},
    {"bad_crc",0,.14F,-.15F,0,.14F,0,0,0,0,0,true,true},
  };
  std::cout << std::setprecision(9);
  for (const auto &f : fixtures) {
    TrajectoryCommand c;
    c.valid=f.valid;c.sequence=1;c.yaw=f.yaw;c.pitch=f.pitch;
    c.yaw_speed=f.vy;c.pitch_speed=f.vp;c.yaw_acc=f.ay;c.pitch_acc=f.ap;
    if (!finalize_world_trajectory(c,WorldTrajectoryPolicy{},0).empty()) return 2;
    auto bytes=pack_trajectory(c);
    if(f.corrupt) bytes[37]^=1;
    std::cout<<f.name<<','<<f.raw_yaw<<','<<f.world_pitch<<','<<f.mechanical
             <<','<<f.expected_reason<<',';
    const char *hex="0123456789abcdef";
    for(auto b:bytes) std::cout<<hex[b>>4]<<hex[b&15];
    std::cout<<'\n';
  }
}
