#pragma once
#include <array>
#include <algorithm>
extern "C" int startup_initial(const double*,const double*,const double*,double*);
extern "C" int startup_handoff(const double*,const double*,int,double*);
namespace sdk87 {
// Begin only after obtaining a validated joint sample; caller checks freshness.
// Feed Previous() into Consumer::Observe BEFORE each policy inference.
// Apply() produces the request used by the unchanged effort limiter / SDK PD.
class StartupController {
 std::array<double,25> previous_{};int step_=0;bool ready_=false;
 public:
 bool Begin(const std::array<double,25>& q,const std::array<double,25>& q0,const std::array<double,25>& scale){
  ready_=startup_initial(q.data(),q0.data(),scale.data(),previous_.data());step_=0;return ready_;
 }
 bool Apply(const std::array<double,25>& raw,std::array<double,25>& shaped){
  if(!ready_||!startup_handoff(raw.data(),previous_.data(),step_,shaped.data())){ready_=false;return false;}
  previous_=shaped;step_=std::min(step_+1,10);return true;
 }
 const std::array<double,25>& Previous()const{return previous_;}
 void Reset(){ready_=false;step_=0;}
};
}
