#include "rl_tracking/blink_runtime.h"
#include "parameter/global_config_initializer.h"
#include "cnpy.h"
#include <nlohmann/json.hpp>
#include <openssl/sha.h>
#include <algorithm>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include "frozen_profile.h"
#include "rl_tracking/t800_capture.h"
namespace {
void require(bool ok,const std::string& msg){if(!ok)throw std::runtime_error("BlinkMimic: "+msg);}
std::string sha256(const std::filesystem::path& p){
 std::ifstream f(p,std::ios::binary);require(f.good(),"missing file "+p.string());
 SHA256_CTX ctx;SHA256_Init(&ctx);std::array<char,65536> buf;
 while(f){f.read(buf.data(),buf.size());if(f.gcount())SHA256_Update(&ctx,buf.data(),f.gcount());}
 unsigned char h[32];SHA256_Final(h,&ctx);std::ostringstream out;
 for(auto x:h)out<<std::hex<<std::setw(2)<<std::setfill('0')<<int(x);return out.str();
}
double value(const cnpy::NpyArray& a,size_t i){
 require(!a.fortran_order,"Fortran-order NPZ array is unsupported");
 if(a.word_size==4)return a.data<float>()[i];
 if(a.word_size==8)return a.data<double>()[i];
 throw std::runtime_error("BlinkMimic: expected floating-point NPZ array");
}
}
namespace blink {
int64_t MonotonicNs(){return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();}
}
namespace runner {
void BlinkRuntime::Fail(){blink::telemetry::LogEvent(last_tick_ns_,int(phase_),frame_,selected_motion_,"runner_fault");ready_=false;GetMutableOutput().Deactivate();SetRunnerState(RunnerState::kFault);}

// Load before motion starts, so contact handling performs no disk IO/model load.
void BlinkRuntime::PrepareBalancePolicy(const std::string& config) {
 std::filesystem::path path=config;nlohmann::json c;
 std::ifstream f(path);require(f.good(),"missing balance config");f>>c;
 require(c.at("obs_dim")==87&&c.at("history_frames")==10&&c.at("goal_dim")==650&&c.at("input_dim")==1520&&c.at("action_dim")==25,"balance dimensions mismatch");
 require(c.at("policy_hz")==50&&c.at("physics_hz")==200&&c.at("execution_hz")==200,"balance frequency mismatch");
 for(int i=0;i<25;i++){
  require(c.at("joint_names")[i]==blink::frozen_names[i]&&c.at("default_joint_pos")[i]==blink::frozen_q0[i]&&c.at("joint_stiffness")[i]==blink::frozen_kp[i]&&c.at("joint_damping")[i]==blink::frozen_kd[i]&&c.at("action_scale")[i]==blink::frozen_scale[i]&&c.at("effort_limit")[i]==blink::frozen_effort[i],"balance controller profile mismatch");
 }
 auto model=path.parent_path()/c.at("policy_file").get<std::string>();
 require(sha256(model)==c.at("policy_sha256"),"balance policy digest mismatch");
 balance_policy_=std::make_unique<math::MNNModel>(model.string());
}

bool BlinkRuntime::ContactBalance(const float* goal,const double* reference) {
 if(!ready_||GetRunnerState()!=RunnerState::kRunning||!balance_policy_||balance_active_)return false;
 for(int i=0;i<650;i++)if(!std::isfinite(goal[i]))return false;
 double norm=0;for(int i=0;i<4;i++){if(!std::isfinite(reference[i]))return false;norm+=reference[i]*reference[i];}
 if(std::abs(norm-1.)>.001)return false;
 policy_.swap(balance_policy_); // Retain old model until ordinary teardown.
 SetGuardGoal(goal);guard_transition_=true;balance_active_=true;
 const int f=std::min(frame_,int(reference_quat_.size())-1);
 std::copy_n(reference,4,reference_quat_[f].begin());
 // Keep real observation history, previous issued action, SDK state, timestamp
 // and startup state. The interrupted frame never advances/resumes.
 return true;
}

void BlinkRuntime::Preload(){
  if(loaded_)return;
  blink::telemetry::Initialize();require(blink::telemetry::Ready(),"automatic capture is unavailable");
  require(std::abs(runner_period_-.02)<1e-10,"policy scheduling must be 50 Hz");
  std::filesystem::path path=config_path_;
  if(path.empty()){
   require(!param_tag_.empty(),"missing motion bundle tag");
   path=std::filesystem::path(common::GlobalPathManager::GetInstance().GetConfigPath())/param_tag_/"config.json";
  }
  nlohmann::json c;std::ifstream stream(path);require(stream.good(),"cannot read config");stream>>c;
  require(c.at("schema")=="t800_blinkmimic_sdk_v1"&&c.at("policy_type")=="blinkmimic87","incorrect policy contract");
  require(c.at("obs_dim")==87&&c.at("history_frames")==10&&c.at("goal_dim")==650&&c.at("input_dim")==1520&&c.at("action_dim")==25,"incorrect network dimensions");
  require(c.at("policy_hz")==50&&c.at("execution_hz")==200&&c.at("physics_hz")==200&&c.at("startup_steps")==10&&c.at("action_clip")==100.,"frequency/startup contract changed");
  require(c.at("imu_install_bias")==nlohmann::json::array({0,0,0}),"mount transform requires a separately validated adapter");
  auto names=c.at("joint_names").get<std::vector<std::string>>();require(names.size()==25,"joint count");sdk_indices_=blink::SdkJointOrder(*data_store_);
  for(int i=0;i<25;i++){
   require(names[i]==blink::frozen_names[i],"joint order mismatch");
   q0_[i]=c.at("default_joint_pos")[i];kp_[i]=c.at("joint_stiffness")[i];kd_[i]=c.at("joint_damping")[i];scale_[i]=c.at("action_scale")[i];effort_[i]=c.at("effort_limit")[i];
   require(q0_[i]==blink::frozen_q0[i]&&kp_[i]==blink::frozen_kp[i]&&kd_[i]==blink::frozen_kd[i]&&scale_[i]==blink::frozen_scale[i]&&effort_[i]==blink::frozen_effort[i],"frozen controller profile mismatch");
  }
  const auto model=path.parent_path()/c.at("policy_file").get<std::string>();
  const auto motion=path.parent_path()/c.at("motion_file").get<std::string>();
  require(sha256(model)==c.at("policy_sha256")&&sha256(motion)==c.at("motion_sha256"),"bundle digest mismatch");
  auto npz=cnpy::npz_load(motion.string());const auto& quat=npz.at("root_quat");const auto& goal=npz.at("goal");
  require(quat.shape.size()==2&&quat.shape[1]==4&&quat.shape[0]>1,"reference quaternion shape");
  require(goal.shape==std::vector<size_t>({quat.shape[0],650}),"reference goal shape");
  reference_quat_.resize(quat.shape[0]);goals_.resize(quat.shape[0]);
  for(size_t f=0;f<quat.shape[0];f++){
   for(int j=0;j<4;j++){double v=value(quat,f*4+j);require(std::isfinite(v),"nonfinite reference quaternion");reference_quat_[f][j]=v;}
   for(int j=0;j<650;j++){double v=value(goal,f*650+j);require(std::isfinite(v),"nonfinite reference goal");goals_[f][j]=float(v);}
  }
  require(c.at("hold_seconds")==2.,"hold contract changed");hold_steps_=100;
  require(c.at("startup_initial_action_source")=="verified_previous_virtual_joint_target","incorrect startup source");
  if(!compensation_){
   const char* root=std::getenv("BLINK_SHARED_ROOT");
   auto folder=root&&*root?std::filesystem::path(root):path.parent_path().parent_path().parent_path()/"shared";
   compensation_=std::make_shared<blink::SharedCompensation>(folder.string());
   selected_motion_=path.parent_path().filename().string();compensation_->Reset(selected_motion_);
  }
  policy_=std::make_unique<math::MNNModel>(model.string());input_.resize(1520,1);
  const auto& initial=npz.at("q");require(initial.shape==std::vector<size_t>({quat.shape[0],25}),"reference joint shape");
  loaded_=true;
}
bool BlinkRuntime::Enter(){
 try {
  Preload();
  return EnterWithSdkState(blink::ReadSdkSnapshot(*data_store_,sdk_indices_),blink::MonotonicNs());
 } catch(const std::exception& e) {LOG(ERROR)<<e.what();Fail();return false;}
}
bool BlinkRuntime::EnterWithSdkState(const blink::Snapshot& snapshot,int64_t now){
 try {
  ready_=false;
  GetMutableOutput().Deactivate();
  require(loaded_,"Blink policy was not preloaded");
  for(int i=0;i<25;i++){
   require(std::isfinite(snapshot.previous_target[i])&&std::isfinite(snapshot.packet.q[i])&&std::isfinite(snapshot.packet.dq[i]),"nonfinite handoff state");
  }
  // The new controller supplies its own gains. The preceding controller's
  // gains do not gate entry; its actual target still seeds the output blend.
  // Validate data without pose, joint-rate, gyro or upright thresholds.
  for(double w:snapshot.packet.angular_velocity_body)require(std::isfinite(w),"nonfinite entry gyro");
  require(startup_.Begin(snapshot.packet,reference_quat_[0],q0_,scale_,&snapshot.previous_target),"invalid initial snapshot");
  const int count=model_param_->num_total_joints;require(count==25,"this artifact controls exactly 25 joints");
  target_sdk_.setZero(count);kp_sdk_.setZero(count);kd_sdk_.setZero(count);
  for(int i=0;i<25;i++){int j=sdk_indices_[i];target_sdk_[j]=snapshot.previous_target[i];kp_sdk_[j]=kp_[i];kd_sdk_[j]=kd_[i];}
  frame_=0;last_tick_ns_=0;limiter_events_=clip_events_=0;ready_=true;SetRunnerState(RunnerState::kRunning);
  blink::telemetry::LogEvent(now,int(phase_),0,selected_motion_,"runner_enter");
  return true;
 }catch(const std::exception& e){LOG(ERROR)<<e.what();Fail();return false;}
}
void BlinkRuntime::TickAt(int64_t now){
 try {
  TickWithSdkState(blink::ReadSdkSnapshot(*data_store_,sdk_indices_),now);
 } catch (...) {Fail();throw;}
}
void BlinkRuntime::TickWithSdkState(const blink::Snapshot& s,int64_t now){
 try{
  require(ready_&&GetRunnerState()==RunnerState::kRunning,"runner not ready");
  require(!last_tick_ns_||(now>last_tick_ns_&&now-last_tick_ns_<=40000000),"invalid policy tick interval");
  const int f=std::min(frame_,int(goals_.size())-1);
  require(startup_.Observe(s.packet,reference_quat_[f],observation_),"invalid SDK observation");
  const auto& h=startup_.History();for(int i=0;i<870;i++)input_(i,0)=float(h[i]);
  for(int i=0;i<650;i++)input_(870+i,0)=guard_transition_&&static_guard_ready_?guard_goal_[i]:goals_[f][i];
  require(input_.allFinite(),"nonfinite model input");
  Eigen::VectorXf output=policy_->Inference(input_);require(output.size()==25&&output.allFinite(),"invalid MNN output");
  // One compensation path only: native frozen v2 replaces the Python callback.
  residual_=compensation_->Step(input_.data(),s.packet.quaternion_wxyz.data(),s.packet.angular_velocity_body.data(),phase_,external_frame_>=0?external_frame_:frame_,scale_);
  for(double x:residual_)require(std::isfinite(x),"nonfinite recovery residual");
  std::array<double,25> clipped;for(int i=0;i<25;i++){raw_[i]=double(output[i])+residual_[i];clipped[i]=std::clamp(raw_[i],-100.,100.);clip_events_+=clipped[i]!=raw_[i];}
  require(startup_.Apply(clipped,shaped_),"startup handoff failed");
  for(int i=0;i<25;i++){
   double target=q0_[i]+scale_[i]*shaped_[i];double tau=kp_[i]*(target-s.packet.q[i])-kd_[i]*s.packet.dq[i];
   double capped=std::clamp(tau,-effort_[i],effort_[i]);
   if(capped!=tau){target=s.packet.q[i]+(capped+kd_[i]*s.packet.dq[i])/kp_[i];++limiter_events_;}
   limited_target_[i]=target;target_sdk_[sdk_indices_[i]]=target;
  }
  auto zero=Eigen::VectorXd::Zero(model_param_->num_total_joints);
  GetMutableOutput().SetCommand(target_sdk_,zero,kp_sdk_,kd_sdk_,zero);GetMutableOutput().Activate();last_tick_ns_=now;
  blink::telemetry::LogPolicy(now,int(phase_),external_frame_>=0?external_frame_:frame_,selected_motion_,s,input_.data(),observation_.data(),raw_.data(),shaped_.data(),limited_target_.data(),compensation_->diagnostics());
  if(guard_transition_)return;
  if(frame_>=int(goals_.size())-1+hold_steps_)SetRunnerState(RunnerState::kTryExit);else ++frame_;
 }catch(...){Fail();throw;}
}
void BlinkRuntime::Run(){try{TickAt(blink::MonotonicNs());}catch(const std::exception& e){LOG(ERROR)<<e.what();}}
bool BlinkRuntime::Exit(){blink::telemetry::LogEvent(last_tick_ns_,int(phase_),frame_,selected_motion_,"runner_exit");ready_=false;GetMutableOutput().Deactivate();return true;}
}
