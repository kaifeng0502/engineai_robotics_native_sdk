#pragma once
#include "rl_tracking/blink_runtime.h"
#include <deque>

namespace runner {
// Outer scheduler: 200 Hz. Actor/compensation: 50 Hz, same as frozen desktop.
// This class only produces virtual-joint commands; the official parallel ankle
// converter and real motor transport remain the sole execution path.
class BlinkSession : public MotionRunner {
 public:
  BlinkSession(std::string_view name,const std::shared_ptr<data::DataStore>& store)
      :MotionRunner(name,store){}
  bool Initialize() override;
  bool Enter() override;
  void Run() override;
  void TickAt(int64_t now);
  bool EnterAt(int64_t now);
  bool Exit() override;
  TransitionState TryExit() override{return TransitionState::kCompleted;}
  void SetupContext() override{data_store_->parallel_by_classic_parser.store(false);}
  void TeardownContext() override{data_store_->parallel_by_classic_parser.store(true);}
  // Keep historical log IDs; the former Commit (0) and Guard (1) stages
  // no longer participate in operator-triggered action entry.
  enum class Stage { Action=2,Finish=3,Recover=4,Done=5,Fault=6 };
  Stage stage()const{return stage_;}
  int frame()const{return frame_;}
  uint64_t PolicyTicks()const{return policy_ticks_;}
  int LastPhase()const{return last_phase_;}
  const BlinkRuntime* active()const{return active_;}
 private:
  void Preload();
  void Start(BlinkRuntime& core,const blink::Snapshot& snapshot,int64_t now);
  void Fault(int64_t now,const std::string& reason);
  void SetStage(Stage stage,int64_t now);
  std::string selected_;
  std::shared_ptr<blink::SharedCompensation> compensation_;
  std::unique_ptr<BlinkRuntime> action_,return_;
  BlinkRuntime* active_=nullptr;
  std::array<double,25> return_pose_{};
  Stage stage_=Stage::Done;
  int64_t stage_ns_=0,next_policy_ns_=0,last_tick_ns_=0;
  int frame_=0,stable_=0,last_phase_=-1;
  uint64_t policy_ticks_=0;
  std::array<int,25> sdk_indices_{};
  bool loaded_=false;
  std::deque<std::array<double,25>> rates_;
};
}
REGISTER_RUNNER(BlinkSession, "blink_native_tracking_runner", kMotion)
