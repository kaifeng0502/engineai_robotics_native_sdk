#pragma once
#include <array>
#include <cmath>
#include <cstdint>
#include <algorithm>
#include <memory>
extern "C" int sdk87_align(const double*,const double*,double*);
extern "C" int sdk87_build(const double*,const double*,const double*,const double*,const double*,const double*,const double*,const double*,double*);
namespace sdk87 {
// Values read from the official SDK. Source age is not exposed by this API.
struct Packet {
  std::array<double,25> q{},dq{};
  std::array<double,4> quaternion_wxyz{};
  std::array<double,3> angular_velocity_body{};
};
class Consumer {
 public:
  bool Begin(const Packet& p,const std::array<double,4>& ref){
    started_=false;history_ready_=false;
    if(!sdk87_align(p.quaternion_wxyz.data(),ref.data(),alignment_.data()))return false;
    started_=true;return true;
  }
  bool Observe(const Packet& p,const std::array<double,25>& q0,const std::array<double,25>& previous_action,const std::array<double,4>& ref,std::array<double,87>& out){
    if(!started_){started_=false;return false;}
    std::array<double,87> value{};
    if(!sdk87_build(p.q.data(),p.dq.data(),q0.data(),p.angular_velocity_body.data(),p.quaternion_wxyz.data(),previous_action.data(),ref.data(),alignment_.data(),value.data())){started_=false;return false;}
    if(!history_ready_){for(int i=0;i<10;i++)std::copy(value.begin(),value.end(),history_.begin()+87*i);history_ready_=true;}
    else{std::move(history_.begin()+87,history_.end(),history_.begin());std::copy(value.begin(),value.end(),history_.end()-87);}
    out=value;return true;
  }
  const std::array<double,870>& History()const{return history_;}
 private:
  bool started_=false,history_ready_=false;
  std::array<double,9> alignment_{};std::array<double,870> history_{};
};
}
