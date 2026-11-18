#pragma once
#include <array>
#include <memory>
#include <mutex>
#include <string>
#include <vector>
#include "basic/motion_runner.h"
#include "math/mnn_model.h"
#include "sdk87_core.h"
#include "blink_sdk_state.h"
#include "blink_balance.h"

namespace blink {
int64_t MonotonicNs();
struct SharedPolicy {
  explicit SharedPolicy(const std::string& path): model(path) {}
  math::MNNModel model;
  std::mutex mutex;
};
}
namespace runner {
// Checkpoint 23000: measured state -> raw actor action -> virtual joint target.
// Actuator effort limits belong to the downstream actuator/physics loop.
class BlinkRuntime : public MotionRunner {
 public:
  BlinkRuntime(std::string_view name,const std::shared_ptr<data::DataStore>& store):MotionRunner(name,store){}
  void Preload();
  bool Enter() override;
  void Run() override;
  void TickAt(int64_t now);
  TransitionState TryExit() override{return TransitionState::kCompleted;}
  bool Exit() override;
  void SetupContext() override{data_store_->parallel_by_classic_parser.store(false);}
  void TeardownContext() override{data_store_->parallel_by_classic_parser.store(true);}
  void SetBundleConfig(const std::string& path){if(config_path_!=path){config_path_=path;loaded_=false;}}
  bool EnterWithSdkState(const blink::Snapshot& snapshot,int64_t now);
  void TickWithSdkState(const blink::Snapshot& snapshot,int64_t now);
  const Eigen::MatrixXf& LastInput()const{return input_;}
  const std::array<double,87>& LastObservation()const{return observation_;}
  const std::array<double,25>& LastRaw()const{return raw_;}
  const std::array<double,25>& LastShaped()const{return raw_;}
  const std::array<double,25>& LastTarget()const{return target_;}
  const void* ModelIdentity()const{return policy_.get();}
  int Frame()const{return frame_;}
 private:
  void Fail();
  std::string config_path_,selected_motion_;
  std::shared_ptr<blink::SharedPolicy> policy_;
  sdk87::Consumer consumer_;
  bool loaded_=false,ready_=false;
  std::array<double,25> q0_{},kp_{},kd_{},scale_{},raw_{},target_{};
  std::array<double,87> observation_{};
  std::vector<std::array<double,4>> reference_quat_;
  std::vector<std::array<float,650>> goals_;
  std::array<int,25> sdk_indices_{};
  Eigen::MatrixXf input_;
  Eigen::VectorXd target_sdk_,kp_sdk_,kd_sdk_;
  int frame_=0;
  int64_t last_tick_ns_=0;
};
}
