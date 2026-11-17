#pragma once
#include <array>
#include <memory>
#include <mutex>
#include <string>
#include <vector>
#include "basic/motion_runner.h"
#include "basic/runner_registry.h"
#include "math/mnn_model.h"
#include "motion_startup.h"
#include "blink_sdk_state.h"
#include "blink_balance.h"

namespace blink {
int64_t MonotonicNs();
}
namespace runner {
class BlinkRuntime : public MotionRunner {
 public:
  BlinkRuntime(std::string_view name,const std::shared_ptr<data::DataStore>& store):MotionRunner(name,store){}
  void Preload(); // Model/NPZ IO only; call before the periodic control loop.
  bool Enter() override;
  void Run() override;
  void TickAt(int64_t now); // Simulation clock injection; identical control body.
  TransitionState TryExit() override{return TransitionState::kCompleted;}
  bool Exit() override;
  void SetupContext() override{data_store_->parallel_by_classic_parser.store(false);}
  void TeardownContext() override{data_store_->parallel_by_classic_parser.store(true);}
  void SetBundleConfig(const std::string& path){if(config_path_!=path){config_path_=path;loaded_=false;}}
  void SetGuardTransition(bool enabled){guard_transition_=enabled;phase_=enabled?blink::Phase::Guard:blink::Phase::Action;}
  void SetGuardGoal(const float* goal){std::copy_n(goal,650,guard_goal_.data());static_guard_ready_=true;}
  bool HoldFinalPose(){if(!ready_||GetRunnerState()!=RunnerState::kTryExit)return false;guard_transition_=true;phase_=blink::Phase::Finish;static_guard_ready_=false;SetRunnerState(RunnerState::kRunning);return true;}
  void ReleaseGuard(){guard_transition_=false;frame_=0;phase_=blink::Phase::Action;}
  // Share this object through RETURN_GUARD; reset only when a new motion is triggered.
  void BindCompensation(std::shared_ptr<blink::SharedCompensation> p,const std::string& selected){compensation_=std::move(p);selected_motion_=selected;}
  void SetExecutionContext(blink::Phase phase,int frame){phase_=phase;external_frame_=frame;}
  const blink::CompensationDiagnostics& CompensationState()const{return compensation_->diagnostics();}
  void PrepareBalancePolicy(const std::string& config);
  bool ContactBalance(const float* goal,const double* reference);
  bool BalanceActive()const{return balance_active_;}
  using RecoveryCallback=void(*)(const float*,const float*,double*);
  void SetRecoveryCallback(RecoveryCallback cb){recovery_callback_=cb;}
  void SetActionResidual(const double* x){std::copy_n(x,25,residual_.begin());}
  bool EnterWithSdkState(const blink::Snapshot& snapshot,int64_t now);
  void TickWithSdkState(const blink::Snapshot& snapshot,int64_t now);
  const Eigen::MatrixXf& LastInput()const{return input_;}
  const std::array<double,87>& LastObservation()const{return observation_;}
  const std::array<double,25>& LastRaw()const{return raw_;}
  const std::array<double,25>& LastShaped()const{return shaped_;}
  const std::array<double,25>& LastTarget()const{return limited_target_;}
  int LimiterEvents()const{return limiter_events_;}
  int ActionClipEvents()const{return clip_events_;}
  int Frame()const{return frame_;}
 private:
  void Fail();
  std::string config_path_,selected_motion_;
  std::shared_ptr<blink::SharedCompensation> compensation_;
  blink::Phase phase_=blink::Phase::Action;
  int external_frame_=-1;
  std::unique_ptr<math::MNNModel> policy_;
  std::unique_ptr<math::MNNModel> balance_policy_;
  bool balance_active_=false;
  sdk87::MotionStartup startup_;
  bool loaded_=false;
  std::array<double,25> q0_{},kp_{},kd_{},scale_{},effort_{};
  std::array<double,25> raw_{},shaped_{},limited_target_{};
  std::array<double,87> observation_{};
  std::vector<std::array<double,4>> reference_quat_;
  std::vector<std::array<float,650>> goals_;
  std::array<int,25> sdk_indices_{};
  Eigen::MatrixXf input_;
  Eigen::VectorXd target_sdk_,kp_sdk_,kd_sdk_;
  int frame_=0,hold_steps_=100,limiter_events_=0,clip_events_=0;
  int64_t last_tick_ns_=0;
  std::array<float,650> guard_goal_{};
  bool static_guard_ready_=false;
  RecoveryCallback recovery_callback_=nullptr;
  std::array<double,25> residual_{};
  bool ready_=false,guard_transition_=false;
};
}
