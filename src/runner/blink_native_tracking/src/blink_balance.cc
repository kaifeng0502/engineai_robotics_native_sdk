#include "rl_tracking/blink_balance.h"
#include <MNN/Interpreter.hpp>
#include <MNN/Tensor.hpp>
#include <cstring>
#include "cnpy.h"
#include "shared_hashes.h"
#include <openssl/evp.h>
#include <iomanip>
#include <sstream>
#include <Eigen/Dense>
#include <nlohmann/json.hpp>
#include <dlfcn.h>
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <map>
#include <stdexcept>
#include <vector>

namespace blink { namespace {
using V3=Eigen::Vector3d;using M3=Eigen::Matrix3d;
double clamp(double x,double lo,double hi){return std::clamp(x,lo,hi);}
void require(bool v,const char* text){if(!v)throw std::runtime_error(text);}
double number(const cnpy::NpyArray& a,size_t i){
 require(!a.fortran_order,"Compensation NPZ must use C order");
 if(a.word_size==4)return a.data<float>()[i];
 if(a.word_size==8)return a.data<double>()[i];
 throw std::runtime_error("Unsupported compensation array type");
}
struct Network {
 std::unique_ptr<MNN::Interpreter> network;MNN::Session* session=nullptr;
 MNN::Tensor *input=nullptr,*output=nullptr;
 Network(const std::filesystem::path& path){
  network.reset(MNN::Interpreter::createFromFile(path.c_str()));require(bool(network),"Cannot load frozen MNN auxiliary model");
  MNN::BackendConfig backend;backend.precision=MNN::BackendConfig::Precision_High;
  MNN::ScheduleConfig config;config.type=MNN_FORWARD_CPU;config.numThread=1;config.backendConfig=&backend;
  session=network->createSession(config);require(session!=nullptr,"Cannot create MNN auxiliary session");
  input=network->getSessionInput(session,nullptr);output=network->getSessionOutput(session,nullptr);
  require(input&&output,"Missing auxiliary tensors");
 }
 std::vector<float> Run(const float* x,size_t n,size_t m){
  require(input->elementSize()==n&&output->elementSize()==m,"Auxiliary MNN dimensions mismatch");
  MNN::Tensor host_in(input,MNN::Tensor::CAFFE);std::memcpy(host_in.host<float>(),x,n*sizeof(float));
  require(input->copyFromHostTensor(&host_in)&&network->runSession(session)==MNN::NO_ERROR,"Auxiliary MNN inference failed");
  MNN::Tensor host_out(output,MNN::Tensor::CAFFE);require(output->copyToHostTensor(&host_out),"Auxiliary MNN output transfer failed");
  const auto* data=host_out.host<float>();std::vector<float> result(data,data+m);
  for(auto v:result)require(std::isfinite(v),"Nonfinite auxiliary output");return result;
 }
};
Eigen::Matrix3f matrix6(const float* v){
 Eigen::Matrix3f R;for(int i=0;i<3;i++){R(i,0)=v[2*i];R(i,1)=v[2*i+1];}
 R.col(2)=R.col(0).cross(R.col(1));return R;
}
M3 futureToMeasured(const float* h,const float* goal){
 auto relative=matrix6(h+81);auto future=matrix6(goal+3);
 Eigen::Vector3f g(h[53],h[54],h[55]);Eigen::Vector3f gr=relative.transpose()*g;
 float roll=std::atan2(-gr[1],-gr[2]);float pitch=std::atan2(gr[0],std::hypot(gr[1],gr[2]));
 float cr=std::cos(roll),sr=std::sin(roll),cp=std::cos(pitch),sp=std::sin(pitch);
 M3 tilt;tilt<<cp,sp*sr,sp*cr,0.,cr,-sr,-sp,cp*sr,cp*cr;
 return relative.cast<double>()*tilt.transpose()*future.cast<double>();
}
struct FixedProfile {Eigen::Matrix<double,10,4> weights;cnpy::npz_t arrays;};
}
struct SharedCompensation::Impl {
 std::string directory,motion;Network detector,velocity;
 nlohmann::json config;Eigen::Matrix<double,25,6> weights;
 std::map<std::string,FixedProfile> profiles;
 std::array<double,25> fixed{},shared{};CompensationDiagnostics diag;
 double threshold,width,release,velocity_threshold,tilt_threshold,floor;
 Impl(const std::string& root):directory(root),detector(std::filesystem::path(root)/"detector_v3.mnn"),velocity(std::filesystem::path(root)/"velocity_estimator.mnn"){
  std::ifstream f(std::filesystem::path(root)/"shared_final.json");require(bool(f),"Missing frozen shared configuration");f>>config;
  require(config.at("gain")==0.&&config.at("reference_frame_corrected")==true&&config.at("preserve_installed_feedback")==true,"Unsupported compensation contract");
  require(config.at("detector_history")==2&&config.at("use_velocity")==true,"Unsupported predictor contract");
  require(!config.contains("stance_weights")&&!config.contains("context_weights")&&!config.contains("support_weighting"),"This release only supports the reviewed v2 configuration");
  threshold=config.at("threshold");width=config.at("width");release=config.at("release_seconds");
  velocity_threshold=config.at("velocity_threshold");tilt_threshold=config.at("tilt_threshold");floor=config.at("activation_floor");
  for(int i=0;i<25;i++)for(int j=0;j<6;j++)weights(i,j)=config.at("feedback_weights").at(i).at(j);
  for(const auto& entry:std::filesystem::directory_iterator(std::filesystem::path(root)/"fixed_feedback")){
   if(!entry.is_directory())continue;FixedProfile p;
   auto parameters=cnpy::npz_load((entry.path()/"parameters.npz").string());
   const auto& a=parameters.at("parameters");require(a.num_vals==40,"Fixed feedback weights shape mismatch");
   for(int i=0;i<10;i++)for(int j=0;j<4;j++)p.weights(i,j)=number(a,i*4+j);
   p.arrays=cnpy::npz_load((entry.path()/"nominal.npz").string());profiles.emplace(entry.path().filename().string(),std::move(p));
  }
 }
};
SharedCompensation::SharedCompensation(const std::string& root){
 for(const auto& item:shared_hashes){
  std::ifstream f(std::filesystem::path(root)/item[0],std::ios::binary);require(bool(f),"Missing frozen compensation file");
  auto* ctx=EVP_MD_CTX_new();require(ctx!=nullptr,"Digest context allocation failed");EVP_DigestInit_ex(ctx,EVP_sha256(),nullptr);
  std::array<char,65536> block;while(f){f.read(block.data(),block.size());if(f.gcount())EVP_DigestUpdate(ctx,block.data(),f.gcount());}
  unsigned char digest[32];unsigned int n=0;EVP_DigestFinal_ex(ctx,digest,&n);EVP_MD_CTX_free(ctx);std::ostringstream hash;
  for(auto byte:digest)hash<<std::hex<<std::setw(2)<<std::setfill('0')<<int(byte);
  require(hash.str()==item[1],"Frozen compensation hash mismatch");
 }
 p_=std::make_unique<Impl>(root);
}
SharedCompensation::~SharedCompensation()=default;
void SharedCompensation::Reset(const std::string& motion){p_->motion=motion;p_->fixed.fill(0);p_->shared.fill(0);p_->diag={};}
const CompensationDiagnostics& SharedCompensation::diagnostics()const{return p_->diag;}
const std::string& SharedCompensation::root()const{return p_->directory;}
std::array<double,25> SharedCompensation::Step(const float* x,const double* quat,const double* gyro,Phase phase,int frame,const std::array<double,25>& scale){
 auto& p=*p_;const float* h=x+9*87;const float* prev=x+8*87;
 // Installed three-motion feedback operates on raw SDK IMU doubles, before
 // inference. It is retained as a distinct bounded controller.
 std::array<double,25> target{};auto profile=p.profiles.find(p.motion);
 if(profile!=p.profiles.end()&&(phase==Phase::Action||phase==Phase::Finish)){
  const auto& arrays=profile->second.arrays;const auto& samples=arrays.at("sensors");
  size_t f=std::min(size_t(std::max(frame,0)),samples.shape[0]-1);
  Eigen::Quaterniond q(quat[0],quat[1],quat[2],quat[3]);M3 R=q.toRotationMatrix();
  Eigen::Vector4d sensed(R(2,0),R(2,1),gyro[0],gyro[1]),features;
  double dead[4]={.035,.035,.20,.20},norm[4]={.12,.12,.8,.8};
  for(int i=0;i<4;i++){
   double reference=number(samples,f*4+i);
   if(arrays.count("lower"))reference=clamp(sensed[i],number(arrays.at("lower"),f*4+i),number(arrays.at("upper"),f*4+i));
   double error=sensed[i]-reference;features[i]=clamp(std::copysign(std::max(std::abs(error)-dead[i],0.),error)/norm[i],-3.,3.);
  }
  auto response=(profile->second.weights*features).eval();
  int joints[10]={0,1,3,4,5,6,7,9,10,11};double bounds[5]={.20,.16,.20,.12,.10};
  for(int i=0;i<10;i++)target[joints[i]]=bounds[i%5]*std::tanh(response[i]*.4)*number(arrays.at("support"),f*2+i/5);
 }
 for(int i=0;i<25;i++)p.fixed[i]+=clamp(.2*(target[i]-p.fixed[i]),-.02,.02);
 std::array<float,199> input;std::copy_n(x+7*87,174,input.begin());std::copy_n(h+56,25,input.begin()+174);
 auto predicted=p.detector.Run(input.data(),199,6);auto estimated=p.velocity.Run(x,870,3);
 Eigen::Vector3f e1,e2;
 for(int i=0;i<3;i++){e1[i]=predicted[i]-(h[50+i]-prev[50+i]);e2[i]=predicted[3+i]-(h[53+i]-prev[53+i])*20.f;}
 p.diag.score=double(e1.norm()+e2.norm());p.diag.ema=.7*p.diag.ema+.3*p.diag.score;
 double desired=clamp((p.diag.ema-p.threshold)/p.width,0.,1.);
 auto relative=matrix6(h+81);M3 transform=futureToMeasured(h,x+870);
 V3 requested=transform*V3(x[879],x[880],x[881]);V3 ev(estimated[0],estimated[1],estimated[2]);
 V3 error=ev-requested,g(h[53],h[54],h[55]);V3 horizontal=error-g*g.dot(error);
 p.diag.speed_error=horizontal.norm();p.diag.tilt_error=double(std::acos(std::clamp(relative(2,2),-1.f,1.f)));
 double need=std::max(clamp((p.diag.speed_error-p.velocity_threshold)/.3,0.,1.),clamp((p.diag.tilt_error-p.tilt_threshold)/.12,0.,1.));
 desired*=need;
 if(phase==Phase::Guard||frame<15){desired=0;p.diag.ema=0;p.diag.activation=0;}
 if(desired<p.floor)desired=0;
 p.diag.activation=std::max(desired,p.diag.activation*std::exp(-.02/p.release));
 V3 ge=V3(h[50],h[51],h[52])-transform*V3(x[882],x[883],x[884]);
 Eigen::Matrix<double,6,1> features;features<<relative(2,0)/.12,relative(2,1)/.12,ge[0]/.8,ge[1]/.8,horizontal[0]/.6,horizontal[1]/.6;
 for(auto& v:features)v=clamp(v,-3,3);auto response=(p.weights*features).eval();
 const double bounds[25]={.30,.25,.20,.40,.22,.18,.30,.25,.20,.40,.22,.18,.25,.28,.28,.25,.30,.20,.28,.28,.25,.30,.20,.10,.10};
 std::array<double,25> result;
 for(int i=0;i<25;i++){
  double output=clamp(std::tanh(response[i]*.4)*p.diag.activation,-1.,1.);
  p.shared[i]+=clamp(.25*(output*bounds[i]-p.shared[i]),-.02,.02);
  result[i]=p.fixed[i]/scale[i]+p.shared[i]/scale[i];require(std::isfinite(result[i]),"Non-finite shared compensation output");
  p.diag.normalized_output[i]=output;p.diag.fixed_rad[i]=p.fixed[i];p.diag.shared_rad[i]=p.shared[i];
 }
 for(int i=0;i<3;i++){p.diag.estimated_velocity[i]=ev[i];p.diag.requested_velocity[i]=requested[i];}
 return result;
}
}
