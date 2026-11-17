#pragma once
#include "sdk87_core.h"
#include "startup_core.h"
namespace sdk87 {
// Integrates the checked observation consumer and startup controller; hardware
// caller reads the SDK state, runs inference and handles SDK actuation.
class MotionStartup {
 Consumer consumer_; StartupController controller_;
 std::array<double,25> q0_{};bool ready_=false,pending_=false;
 public:
 bool Begin(const Packet& packet,const std::array<double,4>& reference,
            const std::array<double,25>& q0,const std::array<double,25>& scale,
            const std::array<double,25>* previous_target=nullptr){
  ready_=false;pending_=false;controller_.Reset();q0_=q0;
  // At a controller handoff, the previous action is the actual previous
  // target stored by the SDK. The observation retains the measured pose.
  ready_=consumer_.Begin(packet,reference)&&controller_.Begin(previous_target?*previous_target:packet.q,q0,scale);
  return ready_;
 }
 bool Observe(const Packet& packet,const std::array<double,4>& reference,std::array<double,87>& out){
  if(!ready_||pending_||!consumer_.Observe(packet,q0_,controller_.Previous(),reference,out)){ready_=false;return false;}
  pending_=true;return true;
 }
 bool Apply(const std::array<double,25>& raw,std::array<double,25>& shaped){
  if(!ready_||!pending_||!controller_.Apply(raw,shaped)){ready_=false;return false;}
  pending_=false;return true;
 }
 const std::array<double,870>& History()const{return consumer_.History();}
};
}
