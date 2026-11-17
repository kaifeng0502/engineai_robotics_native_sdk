#include <cmath>
#include <algorithm>
extern "C" int startup_initial(const double* q,const double* q0,const double* scale,double* out){
 for(int i=0;i<25;i++)if(!std::isfinite(q[i])||!std::isfinite(q0[i])||!std::isfinite(scale[i])||scale[i]<=0)return 0;
 for(int i=0;i<25;i++)out[i]=(q[i]-q0[i])/scale[i];
 return 1;
}
extern "C" int startup_handoff(const double* raw,const double* previous,int step,double* out){
 if(step<0)return 0;
 for(int i=0;i<25;i++)if(!std::isfinite(raw[i])||!std::isfinite(previous[i]))return 0;
 double x=std::min(1.,step/10.);double w=x*x*x*(10+x*(-15+6*x));
 for(int i=0;i<25;i++)out[i]=step>=10?raw[i]:previous[i]+w*(raw[i]-previous[i]);
 return 1;
}
