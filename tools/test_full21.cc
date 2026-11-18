#include "rl_tracking/blink_session.h"
#include "rl_tracking/blink_walk_return.h"
#include "rl_tracking/frozen_profile.h"
#include "rl_walking_example/rl_walking_example_runner.h"
#include "parameter/global_config_initializer.h"
#include "cnpy.h"
#include <nlohmann/json.hpp>
#include <fstream>
#include <iostream>
#include <limits>
#include <stdexcept>
void check(bool ok,const char* message){if(!ok)throw std::runtime_error(message);}
int main(int argc,char**argv){
 try{
  check(argc==3,"usage: test_full21 SDK_ROOT REPORT_JSON");
  google::InitGoogleLogging("test_full21");std::string root=argv[1];
  common::SetGlobalConfigPath(root+"/assets/config/t800");
  auto& path=common::GlobalPathManager::GetInstance();path.SetRobotName("t800");path.SetConfigPath(root+"/assets/config/t800");path.SetResourcePath(root+"/assets/resource");
  auto store=std::make_shared<data::DataStore>();const auto order=blink::SdkJointOrder(*store);
  nlohmann::json manifest;std::ifstream(root+"/assets/config/t800/blink_native/manifest.json")>>manifest;
  auto fixture=cnpy::npz_load(root+"/validation/full21/sdk_fixture.npz");
  const Eigen::VectorXd zero=Eigen::VectorXd::Zero(25);
  auto seed=[&](const double* s){
   Eigen::VectorXd q(25),dq(25);for(int i=0;i<25;i++){q[order[i]]=s[i];dq[order[i]]=s[25+i];}
   store->joint_info.SetState(q,dq,zero);data::ImuInfo imu;imu.quaternion=Eigen::Quaterniond(s[50],s[51],s[52],s[53]);imu.angular_velocity=Eigen::Vector3d(s[54],s[55],s[56]);store->imu_info.Set(imu);
  };
  auto commit=[&](const data::ControlOutput& out){const auto& c=out.GetCommand();store->joint_info.SetCommand(c.q_cmd_,c.qd_cmd_,c.tau_ff_cmd_,c.kp_,c.kd_);};
  Eigen::VectorXd initial(25),kp(25),kd(25);for(int i=0;i<25;i++){initial[order[i]]=blink::frozen_q0[i];kp[order[i]]=blink::frozen_kp[i];kd[order[i]]=blink::frozen_kd[i];}
  store->joint_info.SetCommand(initial,zero,zero,kp,kd);
  std::vector<std::unique_ptr<runner::BlinkSession>> sessions;
  for(const auto& m:manifest["motions"]){
   auto s=std::make_unique<runner::BlinkSession>("test_"+m["name"].get<std::string>(),store);
   s->SetParamTag("blink_native/motions/"+m["name"].get<std::string>());s->SetRunnerPeriod(.005);check(s->Initialize(),"session preload failed");
   if(!sessions.empty())check(s->active()->ModelIdentity()==sessions[0]->active()->ModelIdentity(),"model instance duplicated");sessions.push_back(std::move(s));
  }
  runner::RlWalkingExamplePm01Runner walking("test_walk",store);walking.SetParamTag("rl_walking_example");walking.SetRunnerPeriod(.01);
  double max_input=0,max_raw=0,max_target=0,max_walk=0;size_t total=0;
  nlohmann::json rows=nlohmann::json::array();
  for(size_t index=0;index<sessions.size();index++){
   const std::string name=manifest["motions"][index]["name"];auto& session=*sessions[index];const auto& a=fixture.at(name);const size_t count=a.shape[0],width=a.shape[1];check(width==1602&&a.word_size==8,"fixture shape/dtype");
   const auto* data=a.data<double>();int64_t base=1000000000LL+int64_t(index)*20000000000LL;
   store->joint_info.SetCommand(initial,zero,zero,kp,kd);seed(data);session.SetupContext();check(session.EnterAt(base),"entry failed");
   double local_input=0,local_raw=0;
   for(size_t f=0;f<count;f++){
    const double* row=data+f*width;
    if(f){seed(row);for(int sub=1;sub<=4;sub++)session.TickAt(base+int64_t(f-1)*20000000+sub*5000000);}
    check(session.GetRunnerState()==runner::RunnerState::kRunning&&session.GetOutput().IsActive(),"premature completion/fault");
    const auto* core=session.active();
    for(int j=0;j<1520;j++)local_input=std::max(local_input,std::abs(core->LastInput()(j,0)-row[57+j]));
    for(int j=0;j<25;j++){
     local_raw=std::max(local_raw,std::abs(core->LastRaw()[j]-row[1577+j]));
     max_target=std::max(max_target,std::abs(core->LastTarget()[j]-(blink::frozen_q0[j]+blink::frozen_scale[j]*row[1577+j])));
     check(std::abs(core->LastTarget()[j]-(blink::frozen_q0[j]+blink::frozen_scale[j]*core->LastRaw()[j]))<1e-12,"action shaping introduced");
     if(f==0)check(core->LastObservation()[56+j]==0.,"previous action not zero");
    }
    commit(session.GetOutput());++total;
   }
   seed(fixture.at(name+"_end").data<double>());
   for(int sub=1;sub<=4;sub++)session.TickAt(base+int64_t(count-1)*20000000+sub*5000000);
   check(session.stage()==runner::BlinkSession::Stage::Done&&session.PolicyTicks()==count,"wrong completion timing");
   check(session.GetRunnerState()==runner::RunnerState::kTryExit,"missing walking transition");
   commit(session.GetOutput());check(session.Exit(),"exit failed");session.TeardownContext();
   check(walking.Enter(),"walking return entry failed");walking.SetupContext();
   const auto& w=fixture.at(name+"_walk");double local_walk=0;
   for(size_t f=0;f<w.shape[0];f++){
    const auto* row=w.data<double>()+f*w.shape[1];seed(row);walking.Run();const auto& out=walking.GetOutput().GetCommand();
    for(int j=0;j<25;j++){
     local_walk=std::max(local_walk,std::abs(out.q_cmd_[order[j]]-row[57+j]));
     check(std::abs(out.kp_[order[j]]-row[82+j])<1e-10&&std::abs(out.kd_[order[j]]-row[107+j])<1e-10,"walking gains blended");
    }commit(walking.GetOutput());
   }
   walking.Exit();walking.TeardownContext();max_input=std::max(max_input,local_input);max_raw=std::max(max_raw,local_raw);max_walk=std::max(max_walk,local_walk);
   rows.push_back({{"name",name},{"actor_ticks",count},{"input_max_abs",local_input},{"raw_max_abs",local_raw},{"return_target_max_abs_rad",local_walk}});
   std::cout<<name<<" input="<<local_input<<" raw="<<local_raw<<" walk="<<local_walk<<std::endl;
  }
  // Invalid measured state must disable output instead of emitting NaNs.
  const std::string first=manifest["motions"][0]["name"];seed(fixture.at(first).data<double>());
  auto bad=zero.eval();bad[0]=std::numeric_limits<double>::quiet_NaN();store->joint_info.SetState(bad,zero,zero);
  check(!sessions[0]->EnterAt(900000000000LL)&&!sessions[0]->GetOutput().IsActive(),"nonfinite state accepted");
  nlohmann::json report={{"architecture","aarch64"},{"real_sdk_runners",true},{"shared_actor_instances",1},{"motion_count",sessions.size()},{"actor_steps",total},{"input_max_abs",max_input},{"raw_max_abs",max_raw},{"target_max_abs_rad",max_target},{"walking_target_max_abs_rad",max_walk},{"completion_and_invalid_state_checks",true},{"rows",rows}};
  report["pass"]=max_input<.002&&max_raw<.002&&max_target<.001&&max_walk<.001;
  std::ofstream(argv[2])<<report.dump(2)<<'\n';check(report["pass"],"SDK numerical parity failed; inspect report");
  std::cout<<"PASS 21 shared-policy sessions, full timing, direct return and invalid state checks"<<std::endl;return 0;
 }catch(const std::exception& e){std::cerr<<e.what()<<std::endl;return 1;}
}
