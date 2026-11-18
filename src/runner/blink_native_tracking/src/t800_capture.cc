#include "rl_tracking/t800_capture.h"
#include "rl_tracking/blink_runtime.h"
#include "rl_tracking/blink_balance.h"
#include <nlohmann/json.hpp>
#include "frozen_profile.h"
#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <limits>
#include <mutex>
#include <thread>
#include <unistd.h>

namespace blink::telemetry { namespace {
uint64_t mono(){return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();}
uint64_t wall(){return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count();}
constexpr size_t Capacity=2048,MaxBytes=sizeof(PolicyRecord);
static_assert(sizeof(MotorRecord)<MaxBytes&&sizeof(SensorRecord)<MaxBytes);
struct Slot {uint32_t type,size;std::array<unsigned char,MaxBytes> payload;};
struct Recorder {
 std::unique_ptr<Slot[]> queue{new Slot[Capacity]};std::atomic<uint64_t> written{0},read{0};
 std::atomic_flag producer=ATOMIC_FLAG_INIT;std::atomic<bool> stop{false},failed{false};
 std::atomic<uint64_t> dropped{0},io_errors{0};std::array<std::atomic<uint64_t>,5> received{};
 std::thread worker;FILE* file=nullptr;uint64_t bytes=0,total=0,part=0;
 const uint64_t rotate_bytes=128ull*1024*1024,max_bytes=8ull*1024*1024*1024;
 std::string directory;uint64_t started=wall();
 explicit Recorder(){
  const char* root=std::getenv("BLINK_LOG_DIR");auto base=std::filesystem::path(root&&*root?root:"t800_logs");
  directory=std::filesystem::absolute(base/(std::to_string(started)+"_"+std::to_string(getpid()))).string();
  std::filesystem::create_directories(directory);Open();
  const char* domain=std::getenv("BLINK_CAPTURE_DOMAIN");
  nlohmann::json meta={{"schema","t800_blink_capture_v1"},{"pid",getpid()},{"started_wall_ns",started},
    {"byte_order","little"},{"policy_record_size",sizeof(PolicyRecord)},{"sensor_record_size",sizeof(SensorRecord)},
    {"motor_record_size",sizeof(MotorRecord)},{"event_record_size",sizeof(EventRecord)},
    {"policy_hz",50},{"execution_hz",200},{"checkpoint",23000},{"delivery","full21_direct"},{"frame_observation_dim",87},{"model_input_dim",1520},
    {"domain",domain?domain:"sdk_view"},
    {"feedback_provenance","official_sdk_cache"},
    {"motor_sdk_view_is_not_bus_send_confirmation",true},{"missing_extra_motor_fields","NaN plus cleared validity bit"},
    {"rotation_bytes",rotate_bytes},{"session_max_bytes",max_bytes},{"old_logs_deleted",false}};
  std::ofstream m(std::filesystem::path(directory)/"metadata.json");m<<meta.dump(2)<<'\n';m.close();
  if(!m)throw std::runtime_error("Cannot write capture metadata");
  worker=std::thread([this]{Drain();});
 }
 void Open(){
  std::string path=directory+"/capture_"+std::to_string(part++)+".bin";file=std::fopen(path.c_str(),"wb");
  if(!file)throw std::runtime_error("Cannot open capture file "+path);
  const char magic[8]={'T','8','B','L','O','G','1','\0'};uint32_t header[2]={1,0x01020304};
  if(std::fwrite(magic,1,8,file)!=8||std::fwrite(header,4,2,file)!=2){std::fclose(file);file=nullptr;throw std::runtime_error("Cannot write capture header");}
  bytes=16;total+=16;
 }
 template<class T>void Push(uint32_t type,const T& x)noexcept{
  if(failed.load(std::memory_order_relaxed)){dropped.fetch_add(1,std::memory_order_relaxed);return;}
  if(producer.test_and_set(std::memory_order_acquire)){dropped.fetch_add(1,std::memory_order_relaxed);return;}
  auto w=written.load(std::memory_order_relaxed);
  if(w-read.load(std::memory_order_acquire)>=Capacity){++dropped;producer.clear(std::memory_order_release);return;}
  auto& slot=queue[w%Capacity];slot.type=type;slot.size=sizeof(T);std::memcpy(slot.payload.data(),&x,sizeof(T));
  received[type].fetch_add(1,std::memory_order_relaxed);written.store(w+1,std::memory_order_release);producer.clear(std::memory_order_release);
 }
 void Status(bool final=false)noexcept{
  try{
   nlohmann::json j={{"state",failed?"logging_failed":final?"closed":"recording"},{"dropped",dropped.load()},
     {"io_errors",io_errors.load()},{"written_bytes",total},{"policy_records",received[1].load()},
     {"sensor_records",received[2].load()},{"motor_records",received[3].load()},{"event_records",received[4].load()},
     {"updated_wall_ns",wall()},{"control_is_not_blocked_by_log_writer",true}};
   auto path=std::filesystem::path(directory)/"status.json";auto tmp=path;tmp+=".tmp";
   {std::ofstream f(tmp);f<<j.dump(2)<<'\n';if(!f)throw std::runtime_error("status write failed");}
   std::filesystem::rename(tmp,path);
  }catch(...){++io_errors;}
 }
 void Drain()noexcept{
  uint64_t last_status=0;
  for(;;){
   auto r=read.load(std::memory_order_relaxed),w=written.load(std::memory_order_acquire);
   while(r<w){
    auto& s=queue[r%Capacity];
    if(!failed){
     try{
      if(total+s.size+8>max_bytes)throw std::runtime_error("Capture session reached 8 GiB; previous logs retained");
      if(bytes+s.size+8>rotate_bytes){std::fclose(file);file=nullptr;Open();}
      uint32_t head[2]={s.type,s.size};
      if(std::fwrite(head,4,2,file)!=2||std::fwrite(s.payload.data(),1,s.size,file)!=s.size)throw std::runtime_error("Capture write failed");
      bytes+=s.size+8;total+=s.size+8;
     }catch(const std::exception& e){failed=true;++io_errors;std::fprintf(stderr,"T800 CAPTURE ERROR: %s; robot control continues\n",e.what());}
    }else ++dropped;
    ++r;
   }
   read.store(r,std::memory_order_release);
   if(file&&!failed&&std::fflush(file)!=0){failed=true;++io_errors;}
   if(mono()-last_status>1000000000ull){Status();last_status=mono();}
   if(stop.load(std::memory_order_acquire)&&r==written.load(std::memory_order_acquire))break;
   std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  Status(true);
 }
 ~Recorder(){stop=true;if(worker.joinable())worker.join();if(file)std::fclose(file);}
};
std::mutex init_mutex;std::unique_ptr<Recorder> holder;std::atomic<Recorder*> current{nullptr};
void copytext(char* dst,size_t n,const std::string& s){std::memcpy(dst,s.data(),std::min(n-1,s.size()));}
void SDKMotorView(const data::DataStore* owner)noexcept{
 try{
  auto* store=const_cast<data::DataStore*>(owner);if(!store||store->model_param->num_total_joints!=25)return;
  MotorRecord r=EmptyMotorRecord(MotorStage::SdkView);
  // The public SDK store has no hardware packet IDs or timestamps. Do not
  // present a polling counter as source identity or a prepared target as sent.
  thread_local std::array<Eigen::VectorXd,9> v;
  const data::JointInfoType states[]={data::JointInfoType::kPosition,data::JointInfoType::kVelocity,data::JointInfoType::kTorque};
  const data::JointInfoType commands[]={data::JointInfoType::kPosition,data::JointInfoType::kVelocity,data::JointInfoType::kFeedForwardTorque,data::JointInfoType::kStiffness,data::JointInfoType::kDamping,data::JointInfoType::kTorque};
  for(int i=0;i<3;i++)store->motor_info.GetState(states[i],v[i]);
  for(int i=0;i<6;i++)store->motor_info.GetCommand(commands[i],v[i+3]);
  double* arrays[]={r.position,r.velocity,r.torque,r.command_position,r.command_velocity,r.feed_forward,r.kp,r.kd,r.command_torque};
  uint32_t masks[]={Position,Velocity,Torque,CommandPosition,CommandVelocity,FeedForward,Kp,Kd,CommandTorque};
  for(int field=0;field<9;field++){
   if(v[field].size()!=25)continue;
   for(int j=0;j<25;j++){
    int idx=store->model_param->joint_id_in_total_limb.at(blink::frozen_names[j]);arrays[field][j]=v[field][idx];
   }
   r.valid_fields|=masks[field];
  }
  LogMotor(r);
 }catch(...){if(auto* p=current.load())++p->dropped;}
}
}
void Initialize(){
 if(current.load(std::memory_order_acquire))return;
 std::lock_guard<std::mutex> lock(init_mutex);if(!holder){holder=std::make_unique<Recorder>();current.store(holder.get(),std::memory_order_release);}
}
bool Ready()noexcept{auto* p=current.load(std::memory_order_acquire);return p&&!p->failed.load();}
const std::string& SessionDirectory(){Initialize();return holder->directory;}
void LogPolicy(int64_t now,int mode,int frame,const std::string& motion,const Snapshot& s,const float* input,const double* obs,const double* raw,const double* shaped,const double* target,const CompensationDiagnostics& d)noexcept{
 auto* logger=current.load(std::memory_order_acquire);if(!logger)return;PolicyRecord r{};
 r.captured_ns=mono();r.wall_ns=wall();r.control_ns=now;
 r.mode=mode;r.frame=frame;copytext(r.motion,sizeof(r.motion),motion);
 std::copy_n(input,1520,r.input);std::copy_n(obs,87,r.observation);std::copy_n(raw,25,r.raw);std::copy_n(shaped,25,r.shaped);std::copy_n(target,25,r.target);
 std::copy(d.fixed_rad.begin(),d.fixed_rad.end(),r.fixed_rad);std::copy(d.shared_rad.begin(),d.shared_rad.end(),r.shared_rad);
 r.diagnostics[0]=d.score;r.diagnostics[1]=d.ema;r.diagnostics[2]=d.activation;r.diagnostics[3]=d.speed_error;r.diagnostics[4]=d.tilt_error;logger->Push(1,r);
}
void LogSnapshot(const data::DataStore* owner,const Snapshot& s)noexcept{
 auto* logger=current.load(std::memory_order_acquire);if(!logger)return;SensorRecord r{};
 r.captured_ns=mono();r.wall_ns=wall();
 std::copy(s.packet.q.begin(),s.packet.q.end(),r.q);std::copy(s.packet.dq.begin(),s.packet.dq.end(),r.dq);
 std::copy(s.packet.quaternion_wxyz.begin(),s.packet.quaternion_wxyz.end(),r.quaternion);
 std::copy(s.packet.angular_velocity_body.begin(),s.packet.angular_velocity_body.end(),r.gyro);
 std::copy(s.previous_target.begin(),s.previous_target.end(),r.previous_target);std::copy(s.previous_kp.begin(),s.previous_kp.end(),r.previous_kp);std::copy(s.previous_kd.begin(),s.previous_kd.end(),r.previous_kd);
 logger->Push(2,r);SDKMotorView(owner);
}
MotorRecord EmptyMotorRecord(MotorStage stage)noexcept{
 MotorRecord r{};r.captured_ns=mono();r.wall_ns=wall();r.stage=uint32_t(stage);
 double* arrays[]={r.position,r.velocity,r.torque,r.command_position,r.command_velocity,r.feed_forward,r.kp,r.kd,r.command_torque,r.current,r.temperature,r.voltage};
 for(double* a:arrays)std::fill_n(a,25,std::numeric_limits<double>::quiet_NaN());return r;
}
void LogMotor(const MotorRecord& r)noexcept{if(auto* logger=current.load(std::memory_order_acquire))logger->Push(3,r);}
void LogEvent(int64_t now,int mode,int frame,const std::string& motion,const std::string& message)noexcept{
 if(auto* logger=current.load(std::memory_order_acquire)){EventRecord r{};r.captured_ns=mono();r.wall_ns=wall();r.control_ns=now;r.mode=mode;r.frame=frame;copytext(r.motion,sizeof(r.motion),motion);copytext(r.message,sizeof(r.message),message);logger->Push(4,r);}
}
}
