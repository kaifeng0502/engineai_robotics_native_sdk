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
#include <map>
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
void BlinkRuntime::Fail(){
 blink::telemetry::LogEvent(last_tick_ns_,2,frame_,selected_motion_,"runner_fault");
 ready_=false;GetMutableOutput().Deactivate();SetRunnerState(RunnerState::kFault);
}
void BlinkRuntime::Preload(){
 if(loaded_)return;
 blink::telemetry::Initialize();require(blink::telemetry::Ready(),"automatic capture is unavailable");
 require(std::abs(runner_period_-.02)<1e-10,"policy scheduling must be 50 Hz");
 std::filesystem::path path=config_path_;
 if(path.empty())path=std::filesystem::path(common::GlobalPathManager::GetInstance().GetConfigPath())/param_tag_/"config.json";
 nlohmann::json c;std::ifstream stream(path);require(stream.good(),"cannot read config");stream>>c;
 require(c.at("schema")=="t800_blinkmimic_sdk_v2"&&c.at("policy_type")=="blinkmimic87"&&c.at("checkpoint")==23000,"incorrect policy contract");
 require(c.at("obs_dim")==87&&c.at("history_frames")==10&&c.at("goal_dim")==650&&c.at("input_dim")==1520&&c.at("action_dim")==25,"incorrect network dimensions");
 require(c.at("policy_hz")==50&&c.at("execution_hz")==200&&c.at("startup_steps")==0&&c.at("hold_seconds")==0&&c.at("action_clip").is_null(),"direct execution contract changed");
 require(c.at("startup_initial_action_source")=="zero","incorrect initial previous action");
 require(c.at("imu_install_bias")==nlohmann::json::array({0,0,0}),"unsupported IMU mount transform");
 sdk_indices_=blink::SdkJointOrder(*data_store_);
 for(int i=0;i<25;i++){
  require(c.at("joint_names")[i]==blink::frozen_names[i],"joint order mismatch");
  q0_[i]=c.at("default_joint_pos")[i];kp_[i]=c.at("joint_stiffness")[i];kd_[i]=c.at("joint_damping")[i];scale_[i]=c.at("action_scale")[i];
  require(q0_[i]==blink::frozen_q0[i]&&kp_[i]==blink::frozen_kp[i]&&kd_[i]==blink::frozen_kd[i]&&scale_[i]==blink::frozen_scale[i]&&c.at("effort_limit")[i]==blink::frozen_effort[i],"controller profile mismatch");
 }
 const auto model=std::filesystem::canonical(path.parent_path()/c.at("policy_file").get<std::string>());
 const auto motion=path.parent_path()/c.at("motion_file").get<std::string>();
 require(sha256(model)==c.at("policy_sha256")&&sha256(motion)==c.at("motion_sha256"),"bundle digest mismatch");
 auto npz=cnpy::npz_load(motion.string());const auto& quat=npz.at("root_quat");const auto& goal=npz.at("goal");
 require(quat.shape.size()==2&&quat.shape[1]==4&&quat.shape[0]>1&&c.at("frames")==quat.shape[0],"reference quaternion shape");
 require(goal.shape==std::vector<size_t>({quat.shape[0],650})&&npz.at("q").shape==std::vector<size_t>({quat.shape[0],25}),"reference shape");
 reference_quat_.resize(quat.shape[0]);goals_.resize(quat.shape[0]);
 for(size_t f=0;f<quat.shape[0];f++){
  double norm=0;
  for(int j=0;j<4;j++){double v=value(quat,f*4+j);require(std::isfinite(v),"nonfinite reference quaternion");reference_quat_[f][j]=v;norm+=v*v;}
  require(std::abs(norm-1)<.001,"reference quaternion norm");
  for(int j=0;j<650;j++){double v=value(goal,f*650+j);require(std::isfinite(v),"nonfinite reference goal");goals_[f][j]=float(v);}
 }
 // All 21 motion runners share one model and one inference session. Histories
 // and references stay per motion. Nothing is loaded at trigger/return time.
 static std::mutex cache_mutex;
 static std::map<std::string,std::weak_ptr<blink::SharedPolicy>> cache;
 {std::lock_guard<std::mutex> lock(cache_mutex);
  auto& entry=cache[model.string()+":"+c.at("policy_sha256").get<std::string>()];
  policy_=entry.lock();if(!policy_){policy_=std::make_shared<blink::SharedPolicy>(model.string());entry=policy_;}
 }
 selected_motion_=path.parent_path().filename().string();input_.resize(1520,1);loaded_=true;
}
bool BlinkRuntime::Enter(){
 try{Preload();return EnterWithSdkState(blink::ReadSdkSnapshot(*data_store_,sdk_indices_),blink::MonotonicNs());}
 catch(const std::exception& e){LOG(ERROR)<<e.what();Fail();return false;}
}
bool BlinkRuntime::EnterWithSdkState(const blink::Snapshot& s,int64_t now){
 try{
  ready_=false;GetMutableOutput().Deactivate();require(loaded_,"policy not preloaded");
  for(int i=0;i<25;i++)require(std::isfinite(s.packet.q[i])&&std::isfinite(s.packet.dq[i])&&std::isfinite(s.previous_target[i]),"invalid handoff state");
  for(double w:s.packet.angular_velocity_body)require(std::isfinite(w),"invalid entry gyro");
  require(consumer_.Begin(s.packet,reference_quat_[0]),"invalid initial orientation");
  require(model_param_->num_total_joints==25,"expected 25 SDK joints");
  target_sdk_.setZero(25);kp_sdk_.setZero(25);kd_sdk_.setZero(25);
  for(int i=0;i<25;i++){kp_sdk_[sdk_indices_[i]]=kp_[i];kd_sdk_[sdk_indices_[i]]=kd_[i];}
  raw_.fill(0);frame_=0;last_tick_ns_=0;ready_=true;SetRunnerState(RunnerState::kRunning);
  blink::telemetry::LogEvent(now,2,0,selected_motion_,"runner_enter_zero_previous_action");return true;
 }catch(const std::exception& e){LOG(ERROR)<<e.what();Fail();return false;}
}
void BlinkRuntime::TickAt(int64_t now){
 try{TickWithSdkState(blink::ReadSdkSnapshot(*data_store_,sdk_indices_),now);}catch(...){Fail();throw;}
}
void BlinkRuntime::TickWithSdkState(const blink::Snapshot& s,int64_t now){
 try{
  require(ready_&&GetRunnerState()==RunnerState::kRunning,"runner not ready");
  require(!last_tick_ns_||(now>last_tick_ns_&&now-last_tick_ns_<=40000000),"invalid policy tick interval");
  // N reference samples define N-1 physical intervals. The final measured
  // sample marks completion; it does not add a hold or another actor step.
  if(frame_>=int(goals_.size())-1){SetRunnerState(RunnerState::kTryExit);return;}
  require(consumer_.Observe(s.packet,q0_,raw_,reference_quat_[frame_],observation_),"invalid SDK observation");
  const auto& h=consumer_.History();for(int i=0;i<870;i++)input_(i,0)=float(h[i]);
  for(int i=0;i<650;i++)input_(870+i,0)=goals_[frame_][i];
  require(input_.allFinite(),"nonfinite model input");
  Eigen::VectorXf output;
  {std::lock_guard<std::mutex> lock(policy_->mutex);output=policy_->model.Inference(input_);}
  require(output.size()==25&&output.allFinite(),"invalid MNN output");
  for(int i=0;i<25;i++){raw_[i]=output[i];target_[i]=q0_[i]+scale_[i]*raw_[i];target_sdk_[sdk_indices_[i]]=target_[i];}
  const auto zero=Eigen::VectorXd::Zero(25).eval();
  GetMutableOutput().SetCommand(target_sdk_,zero,kp_sdk_,kd_sdk_,zero);GetMutableOutput().Activate();last_tick_ns_=now;
  const blink::CompensationDiagnostics none{};
  blink::telemetry::LogPolicy(now,2,frame_,selected_motion_,s,input_.data(),observation_.data(),raw_.data(),raw_.data(),target_.data(),none);
  ++frame_;
 }catch(...){Fail();throw;}
}
void BlinkRuntime::Run(){try{TickAt(blink::MonotonicNs());}catch(const std::exception& e){LOG(ERROR)<<e.what();}}
bool BlinkRuntime::Exit(){blink::telemetry::LogEvent(last_tick_ns_,2,frame_,selected_motion_,"runner_exit");ready_=false;GetMutableOutput().Deactivate();return true;}
}
