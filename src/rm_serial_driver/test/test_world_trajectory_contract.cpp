#include <cassert>
#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>
#include "rm_serial_driver/world_trajectory_contract.hpp"
using namespace rm_serial_driver;
static unsigned checks;
#define CHECK(x) do {++checks; assert(x);} while (0)
static TrajectoryCommand valid(float pitch=0.268172F)
{
  TrajectoryCommand c; c.valid=true; c.sequence=1; c.pitch=pitch; return c;
}
static void invalid_policy(WorldTrajectoryPolicy p)
{
  bool bad=false; try {validate_world_policy(p,"world");}
  catch (const std::invalid_argument &) {bad=true;}
  CHECK(bad);
}
int main(int argc,char ** argv)
{
  WorldTrajectoryPolicy p; validate_world_policy(p,"world");
  const float logged[]={0.268172F,0.266214F,0.257770F,0.246818F,0.226271F,
    0.209574F,0.192580F,0.173229F,0.132287F,0.106861F};
  std::ofstream vectors;
  if(argc==2) {vectors.open(argv[1],std::ios::binary);CHECK(vectors.good());}
  for (unsigned i=0;i<10;++i) {
    auto c=valid(logged[i]); c.sequence=static_cast<uint16_t>(i+1);
    CHECK(finalize_world_trajectory(c,p,0).empty()); CHECK(c.valid&&c.pitch==logged[i]);
    auto bytes=pack_trajectory(c); CHECK(bytes[0]==0xa6&&bytes[1]==1&&bytes.size()==38);
    CHECK(crc16::Verify_CRC16_Check_Sum(bytes.data(),bytes.size()));
    if(vectors.is_open()) {vectors.write(reinterpret_cast<const char *>(bytes.data()),bytes.size());}
  }
  const auto emit=[&](TrajectoryCommand c,bool corrupt=false) {
      auto bytes=pack_trajectory(c); if(corrupt) {bytes[37]^=1;}
      if(vectors.is_open()) {vectors.write(reinterpret_cast<const char *>(bytes.data()),bytes.size());}
    };
  auto revoke=valid(); revoke.sequence=11;revoke.valid=false;revoke.pitch=std::numeric_limits<float>::quiet_NaN();
  CHECK(finalize_world_trajectory(revoke,p,0).empty());CHECK(!revoke.valid&&revoke.pitch==0);emit(revoke);
  auto crc=valid();crc.sequence=12;finalize_world_trajectory(crc,p,0);emit(crc,true);
  auto unreachable=valid();unreachable.sequence=13;finalize_world_trajectory(unreachable,p,0);emit(unreachable);
  auto reachable=valid(0.106861F);reachable.sequence=14;finalize_world_trajectory(reachable,p,0);emit(reachable);
  auto failed=valid();failed.sequence=15;revoke_trajectory(failed);finalize_world_trajectory(failed,p,0);emit(failed);
  if(vectors.is_open()) {vectors.close();}
  auto c=valid();CHECK(finalize_world_trajectory(c,p,35).empty()&&c.valid&&c.age_ms==37);
  c=valid();CHECK(finalize_world_trajectory(c,p,35.001)=="a6_age_budget_exhausted"&&!c.valid&&c.age_ms==38);
  c=valid();CHECK(finalize_world_trajectory(c,p,0.001).empty()&&c.age_ms==3);
  c=valid();CHECK(finalize_world_trajectory(c,p,1e9)=="a6_age_budget_exhausted"&&c.age_ms==65535&&!c.valid);
  c=valid();CHECK(finalize_world_trajectory(c,p,-1)=="invalid_planning_clock"&&!c.valid);
  c=valid();CHECK(finalize_world_trajectory(c,p,std::numeric_limits<double>::infinity())=="invalid_planning_clock");
  for(float pitch : {static_cast<float>(p.pitch_min),static_cast<float>(p.pitch_max)}) {
    c=valid(pitch);CHECK(finalize_world_trajectory(c,p,0).empty()&&c.valid);
  }
  for(float pitch : {std::nextafter(static_cast<float>(p.pitch_min),-INFINITY),
      std::nextafter(static_cast<float>(p.pitch_max),INFINITY)}) {
    c=valid(pitch);CHECK(finalize_world_trajectory(c,p,0)=="world_pitch_out_of_domain"&&!c.valid&&c.pitch==0);
  }
  for(float bad : {std::numeric_limits<float>::quiet_NaN(),INFINITY}) {
    c=valid();c.pitch=bad;CHECK(finalize_world_trajectory(c,p,0)=="nonfinite_plan"&&!c.valid);
    CHECK(pack_trajectory(c)[1]==0);
  }
  c=valid();c.pitch_speed=0.6F;c.pitch_acc=8;CHECK(finalize_world_trajectory(c,p,0).empty());
  c=valid();c.pitch_speed=std::nextafter(0.6F,INFINITY);CHECK(finalize_world_trajectory(c,p,0)=="plan_derivative_limit");
  c=valid();c.yaw_acc=20.01F;CHECK(finalize_world_trajectory(c,p,0)=="plan_derivative_limit");
  CHECK(target_age_error(0.05,p).empty()); CHECK(target_age_error(0.050001,p)=="stale_target");
  CHECK(target_age_error(-0.001,p)=="invalid_or_future_target_stamp");
  CHECK(target_age_error(std::numeric_limits<double>::quiet_NaN(),p)=="invalid_or_future_target_stamp");
  c=valid();c.sequence=0;c.host_ms=0;CHECK(pack_trajectory(c)[2]==0);c.sequence=65535;CHECK(pack_trajectory(c)[3]==255);
  auto bad=p;bad.ttl_ms=50;invalid_policy(bad);bad=p;bad.ttl_ms=65576;invalid_policy(bad);
  bad=p;bad.transport_age_ms=-1;invalid_policy(bad);bad=p;bad.transport_age_ms=38;invalid_policy(bad);
  bad=p;bad.pitch_min=-0.39;bad.pitch_max=0.07;validate_world_policy(bad,"world");
  c=valid();CHECK(finalize_world_trajectory(c,bad,0)=="world_pitch_out_of_domain"&&c.pitch==0);
  // Explicit world subranges reject; they never silently clamp a target.
  bad=p;bad.pitch_min=-2;invalid_policy(bad);bad=p;bad.pitch_max=NAN;invalid_policy(bad);
  bad=p;bad.limits.pitch_speed=0.61;invalid_policy(bad);bad=p;bad.target_timeout_ms=51;invalid_policy(bad);
  bool wrong_frame=false;try{validate_world_policy(p,"mechanical");}catch(const std::invalid_argument&){wrong_frame=true;}CHECK(wrong_frame);
  std::cout<<"World contract: "<<checks<<" checks PASS; 15 real A6 vectors emitted when requested\n";
}
