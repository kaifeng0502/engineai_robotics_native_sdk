#pragma once
#include "rl_tracking/blink_runtime.h"
#include "basic/runner_registry.h"
namespace runner {
// 200 Hz SDK scheduler, 50 Hz actor, direct standing -> full clip -> walking.
class BlinkSession : public MotionRunner {
 public:
  BlinkSession(std::string_view name,const std::shared_ptr<data::DataStore>& store):MotionRunner(name,store){}
  bool Initialize() override;
  bool Enter() override;
  void Run() override;
  void TickAt(int64_t now);
  bool EnterAt(int64_t now);
  bool Exit() override;
  TransitionState TryExit() override{return TransitionState::kCompleted;}
  void SetupContext() override{data_store_->parallel_by_classic_parser.store(false);}
  void TeardownContext() override{data_store_->parallel_by_classic_parser.store(true);}
  enum class Stage {Action=2,Done=5,Fault=6};
  Stage stage()const{return stage_;}
  int frame()const{return action_?action_->Frame():0;}
  uint64_t PolicyTicks()const{return policy_ticks_;}
  int LastPhase()const{return 2;}
  const BlinkRuntime* active()const{return action_.get();}
 private:
  void Fault(int64_t now,const std::string& reason);
  std::string selected_;
  std::unique_ptr<BlinkRuntime> action_;
  Stage stage_=Stage::Done;
  int64_t next_policy_ns_=0,last_tick_ns_=0;
  uint64_t policy_ticks_=0;
  std::array<int,25> sdk_indices_{};
  bool loaded_=false;
};
}
REGISTER_RUNNER(BlinkSession,"blink_native_tracking_runner",kMotion)
