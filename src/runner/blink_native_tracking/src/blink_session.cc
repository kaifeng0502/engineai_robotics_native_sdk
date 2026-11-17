#include "rl_tracking/blink_session.h"
#include "rl_tracking/blink_walk_return.h"
#include "rl_tracking/frozen_profile.h"
#include "rl_tracking/t800_capture.h"
#include "cnpy.h"
#include "parameter/global_config_initializer.h"
#include <filesystem>
#include <stdexcept>
#include <cmath>

namespace {
void require(bool v,const char* message){if(!v)throw std::runtime_error(message);}
template<size_t N>double maximum(const std::array<double,N>& a){double v=0;for(double x:a){require(std::isfinite(x),"nonfinite session measurement");v=std::max(v,std::abs(x));}return v;}
double error(const std::array<double,25>& a,const std::array<double,25>& b){double v=0;for(int i=0;i<25;i++)v=std::max(v,std::abs(a[i]-b[i]));return v;}
std::array<double,25> firstPose(const std::filesystem::path& config){
 auto arrays=cnpy::npz_load((config.parent_path()/"motion.npz").string());auto& q=arrays.at("q");
 require(q.shape.size()==2&&q.shape[1]==25&&!q.fortran_order,"invalid session reference");
 std::array<double,25> out;for(int i=0;i<25;i++){require(q.word_size==4||q.word_size==8,"invalid session reference dtype");out[i]=q.word_size==4?q.data<float>()[i]:q.data<double>()[i];require(std::isfinite(out[i]),"nonfinite session reference");}return out;
}
}
namespace runner {
bool BlinkSession::Initialize() {
 try {
  Preload();
  return true;
 } catch (const std::exception& e) {
  LOG(ERROR) << "BlinkMimic preload failed: " << e.what();
  return false;
 }
}

void BlinkSession::Preload(){
 if(loaded_)return;
 require(std::abs(runner_period_-.005)<1e-10,"BlinkSession outer scheduler must be 200 Hz");
 require(param_tag_.rfind("blink_native/motions/",0)==0,"BlinkSession requires a BlinkMimic bundle tag");
 const auto root=common::GlobalPathManager::GetInstance().GetConfigPath();
 const auto path=std::filesystem::path(root)/param_tag_/"config.json";
 selected_=path.parent_path().filename().string();
 sdk_indices_=blink::SdkJointOrder(*data_store_);
 const auto parent=path.parent_path().parent_path();auto block=parent/"Block_front_hik/config.json";
 compensation_=std::make_shared<blink::SharedCompensation>((parent.parent_path()/"shared").string());
 auto core=[&](const std::filesystem::path& p){auto r=std::make_unique<BlinkRuntime>("native_blink_core",data_store_);r->SetRunnerPeriod(.02);r->SetBundleConfig(p.string());r->BindCompensation(compensation_,selected_);r->Preload();return r;};
 action_=core(path);
 return_=core(block);return_->SetGuardTransition(true);
 return_pose_=firstPose(block);loaded_=true;
 blink::telemetry::LogEvent(blink::MonotonicNs(),-1,0,selected_,"session_preloaded");
}
void BlinkSession::SetStage(Stage stage,int64_t now){stage_=stage;stage_ns_=now;stable_=0;blink::telemetry::LogEvent(now,int(stage),frame_,selected_,"session_stage");}
void BlinkSession::Fault(int64_t now,const std::string& reason){stage_=Stage::Fault;GetMutableOutput().Deactivate();SetRunnerState(RunnerState::kFault);blink::telemetry::LogEvent(now,int(stage_),frame_,selected_,reason);}
bool BlinkSession::EnterAt(int64_t now){
 try {
  require(loaded_,"BlinkSession was not preloaded during SDK initialization");
  const auto s=blink::ReadSdkSnapshot(*data_store_,sdk_indices_);
  // The accepted operator trigger starts the selected action directly.
  // Retain the actual preceding target for output interpolation, without
  // waiting for a prepared pose or a matching previous command/gain readback.
  policy_ticks_=0;last_phase_=-1;
  blink::telemetry::LogSnapshot(data_store_.get(),s);
  blink::telemetry::LogEvent(now,int(Stage::Action),0,selected_,"entry_sdk_state");
  compensation_->Reset(selected_);rates_.clear();frame_=0;active_=nullptr;last_tick_ns_=0;
  action_->SetGuardTransition(false);
  Start(*action_,s,now);
  Eigen::VectorXd q=Eigen::VectorXd::Zero(25),kp=q,kd=q,zero=q;
  for(int i=0;i<25;i++){const int j=sdk_indices_[i];q[j]=s.previous_target[i];kp[j]=blink::frozen_kp[i];kd[j]=blink::frozen_kd[i];}
  GetMutableOutput().SetCommand(q,zero,kp,kd,zero);GetMutableOutput().Activate();SetRunnerState(RunnerState::kRunning);SetStage(Stage::Action,now);next_policy_ns_=now;return true;
 } catch(const std::exception& e){Fault(now,e.what());return false;}
}
bool BlinkSession::Enter(){return EnterAt(blink::MonotonicNs());}
void BlinkSession::Start(BlinkRuntime& core,const blink::Snapshot& snapshot,int64_t now){
 active_=&core;
 require(active_->EnterWithSdkState(snapshot,now),"native action core entry rejected");
}
void BlinkSession::TickAt(int64_t now){
 try{
  require(GetRunnerState()==RunnerState::kRunning,"session is not running");
  require(!last_tick_ns_||(now>last_tick_ns_&&now-last_tick_ns_<=15000000),"200 Hz session missed execution deadline");last_tick_ns_=now;
  const auto s=blink::ReadSdkSnapshot(*data_store_,sdk_indices_);
  blink::telemetry::LogSnapshot(data_store_.get(),s);
  if(now<next_policy_ns_)return;
  auto phase=stage_==Stage::Action?blink::Phase::Action:stage_==Stage::Finish?blink::Phase::Finish:blink::Phase::Recover;
  active_->SetExecutionContext(phase,frame_);active_->TickWithSdkState(s,now);++policy_ticks_;last_phase_=int(phase);SetOutput(active_->GetOutput());next_policy_ns_=now+20000000;
  double dq=maximum(s.packet.dq),gyro=maximum(s.packet.angular_velocity_body);
  if(stage_==Stage::Action){
   if(active_->GetRunnerState()==RunnerState::kTryExit){require(active_->HoldFinalPose(),"motion final hold rejected");SetStage(Stage::Finish,now);}else ++frame_;
  }else if(stage_==Stage::Finish){
   if(dq<.8&&gyro<.4){return_->SetGuardTransition(true);Start(*return_,s,now);SetStage(Stage::Recover,now);rates_.clear();
    // Desktop RETURN_GUARD starts on this same measured sample, without a
    // physical reset or an extra 20 ms hold. Separate core owns fresh history.
    active_->SetExecutionContext(blink::Phase::Recover,frame_);active_->TickWithSdkState(s,now);++policy_ticks_;last_phase_=int(blink::Phase::Recover);SetOutput(active_->GetOutput());rates_.push_back(s.packet.dq);
   }else require(now-stage_ns_<=6000000000LL,"final action pose did not settle");
  }else if(stage_==Stage::Recover){
   rates_.push_back(s.packet.dq);if(rates_.size()>10)rates_.pop_front();double rms=0;
   for(int j=0;j<25;j++){double sum=0;for(const auto& v:rates_)sum+=v[j]*v[j];rms=std::max(rms,std::sqrt(sum/rates_.size()));}
   stable_=rates_.size()==10&&error(s.packet.q,return_pose_)<.28&&rms<.6&&dq<1.&&gyro<.3?stable_+1:0;
   if(now-stage_ns_>=500000000&&stable_>=10){SetStage(Stage::Done,now);SetRunnerState(RunnerState::kTryExit);}
   else require(now-stage_ns_<=6000000000LL,"return guard did not settle");
  }
 }catch(const std::exception& e){Fault(now,e.what());throw;}
}
void BlinkSession::Run(){try{TickAt(blink::MonotonicNs());}catch(const std::exception& e){LOG(ERROR)<<e.what();}}
bool BlinkSession::Exit() {
 if (stage_==Stage::Done && selected_=="Duck_Right__hik" && GetOutput().IsActive()) {
  blink::RequestWalkingPositionBlend(data_store_);
  blink::telemetry::LogEvent(blink::MonotonicNs(),int(stage_),frame_,selected_,"walk_return_position_blend_requested");
 }
 if(active_)active_->Exit();
 GetMutableOutput().Deactivate();
 active_=nullptr;
 return true;
}
}
