#ifndef RM_SERIAL_DRIVER__MPC_PLANNER_HPP_
#define RM_SERIAL_DRIVER__MPC_PLANNER_HPP_
#include <vector>
namespace rm_serial_driver
{
struct AimReference{AimReference(double y,double p):yaw(y),pitch(p){} double yaw,pitch;};
struct MpcLimits{double yaw_speed=5.0,pitch_speed=0.6,yaw_acc=20.0,pitch_acc=8.0;};
struct MpcWeights{double angle=30.0,speed=0.8,acceleration=0.04,acceleration_delta=0.2;};
struct PlannedState{double yaw=0,pitch=0,yaw_speed=0,pitch_speed=0,yaw_acc=0,pitch_acc=0;};
class MpcPlanner{
public:
  MpcPlanner(std::size_t horizon,double dt,MpcLimits limits,MpcWeights weights);
  PlannedState solve(const std::vector<AimReference>& reference);
  void reset();
private:
  struct AxisState{double angle,speed,acceleration;};
  AxisState solve_axis(double angle,double speed,double speed_limit,double acceleration_limit,
    const std::vector<double>& angle_reference,const std::vector<double>& speed_reference);
  std::size_t horizon_;double dt_;MpcLimits limits_;MpcWeights weights_;
};
}
#endif
