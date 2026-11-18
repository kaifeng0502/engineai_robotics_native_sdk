#include "rl_tracking/blink_session.h"
#include "rl_tracking/blink_walk_return.h"
#include "rl_tracking/t800_capture.h"
#include "parameter/global_config_initializer.h"
#include <filesystem>
#include <stdexcept>
namespace {void require(bool v,const char* message){if(!v)throw std::runtime_error(message);}}
namespace runner {
bool BlinkSession::Initialize(){
 try{
  if(loaded_)return true;
  require(std::abs(runner_period_-.005)<1e-10,"BlinkSession scheduler must be 200 Hz");
  require(param_tag_.rfind("blink_native/motions/",0)==0,"missing motion bundle tag");
  const auto path=std::filesystem::path(common::GlobalPathManager::GetInstance().GetConfigPath())/param_tag_/"config.json";
  selected_=path.parent_path().filename().string();sdk_indices_=blink::SdkJointOrder(*data_store_);
  action_=std::make_unique<BlinkRuntime>("native_blink_core",data_store_);
  action_->SetRunnerPeriod(.02);action_->SetBundleConfig(path.string());action_->Preload();loaded_=true;return true;
 }catch(const std::exception& e){LOG(ERROR)<<"BlinkMimic preload: "<<e.what();return false;}
}
void BlinkSession::Fault(int64_t now,const std::string& reason){stage_=Stage::Fault;GetMutableOutput().Deactivate();SetRunnerState(RunnerState::kFault);blink::telemetry::LogEvent(now,6,frame(),selected_,reason);}
bool BlinkSession::EnterAt(int64_t now){
 try{
  require(loaded_,"session was not preloaded");const auto s=blink::ReadSdkSnapshot(*data_store_,sdk_indices_);
  blink::telemetry::LogSnapshot(data_store_.get(),s);
  require(action_->EnterWithSdkState(s,now),"action entry rejected");
  // First action is computed from current measured state. No state reset,
  // previous-command seed, interpolation or auxiliary policy is involved.
  action_->TickWithSdkState(s,now);SetOutput(action_->GetOutput());
  policy_ticks_=1;stage_=Stage::Action;last_tick_ns_=now;next_policy_ns_=now+20000000;
  SetRunnerState(RunnerState::kRunning);return true;
 }catch(const std::exception& e){Fault(now,e.what());return false;}
}
bool BlinkSession::Enter(){return EnterAt(blink::MonotonicNs());}
void BlinkSession::TickAt(int64_t now){
 try{
  require(GetRunnerState()==RunnerState::kRunning,"session is not running");
  require(!last_tick_ns_||(now>=last_tick_ns_&&now-last_tick_ns_<=15000000),"200 Hz session missed execution deadline");last_tick_ns_=now;
  const auto s=blink::ReadSdkSnapshot(*data_store_,sdk_indices_);blink::telemetry::LogSnapshot(data_store_.get(),s);
  if(now<next_policy_ns_)return;
  action_->TickWithSdkState(s,now);next_policy_ns_+=20000000;
  if(action_->GetRunnerState()==RunnerState::kTryExit){
   stage_=Stage::Done;SetRunnerState(RunnerState::kTryExit);
   blink::telemetry::LogEvent(now,5,frame(),selected_,"full_clip_complete_direct_walk_return");
  }else{++policy_ticks_;SetOutput(action_->GetOutput());}
 }catch(const std::exception& e){Fault(now,e.what());throw;}
}
void BlinkSession::Run(){try{TickAt(blink::MonotonicNs());}catch(const std::exception& e){LOG(ERROR)<<e.what();}}
bool BlinkSession::Exit(){
 if(stage_!=Stage::Fault&&GetOutput().IsActive())blink::RequestWalkingDirectReturn(data_store_);
 if(action_)action_->Exit();GetMutableOutput().Deactivate();return true;
}
}
