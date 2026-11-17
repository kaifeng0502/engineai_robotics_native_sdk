// Portable SDK consumer core. Quaternion is w,x,y,z; gyro is body-frame rad/s.
// q/dq/default/previous-action arrays MUST already be in model joint-name order.
// No position/velocity estimate, simulator state, or motor publication is used.
#include <cmath>
#include <algorithm>
static bool rot(const double* q,double* r){
 double n=0;for(int i=0;i<4;i++){if(!std::isfinite(q[i]))return false;n+=q[i]*q[i];}
 if(n<.81||n>1.21)return false;n=std::sqrt(n);
 double w=q[0]/n,x=q[1]/n,y=q[2]/n,z=q[3]/n;
 double a[]={1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w),2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w),2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)};
 std::copy_n(a,9,r);return true;
}
extern "C" int sdk87_align(const double* measured_quat,const double* reference_quat,double* alignment){
 double r[9],ref[9];if(!rot(measured_quat,r)||!rot(reference_quat,ref))return 0;
 double a=std::atan2(r[3],r[0])-std::atan2(ref[3],ref[0]),c=std::cos(a),s=std::sin(a);
 double h[]={c,-s,0,s,c,0,0,0,1};std::copy_n(h,9,alignment);return 1;
}
extern "C" int sdk87_build(const double* q,const double* dq,const double* q0,const double* gyro,const double* quat,const double* previous_action,const double* reference_quat,const double* alignment,double* output){
 double r[9],ref[9],aligned[9]{},relative[9]{};if(!rot(quat,r)||!rot(reference_quat,ref))return 0;
 for(int i=0;i<3;i++)for(int j=0;j<3;j++)for(int k=0;k<3;k++)aligned[i*3+j]+=alignment[i*3+k]*ref[k*3+j];
 for(int i=0;i<3;i++)for(int j=0;j<3;j++)for(int k=0;k<3;k++)relative[i*3+j]+=r[k*3+i]*aligned[k*3+j];
 for(int i=0;i<25;i++){output[i]=q[i]-q0[i];output[25+i]=dq[i];output[56+i]=previous_action[i];}
 for(int i=0;i<3;i++){output[50+i]=gyro[i];output[53+i]=-r[6+i];output[81+i*2]=relative[i*3];output[82+i*2]=relative[i*3+1];}
 for(int i=0;i<87;i++)if(!std::isfinite(output[i]))return 0;
 return 1;
}
