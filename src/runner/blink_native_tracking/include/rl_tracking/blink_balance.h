#pragma once
#include <array>
#include <memory>
#include <string>

namespace blink {
enum class Phase : int { Guard=0, Action=1, Finish=2, Recover=3 };
struct CompensationDiagnostics {
  double score=0,ema=0,activation=0,speed_error=0,tilt_error=0;
  std::array<double,3> estimated_velocity{},requested_velocity{};
  std::array<double,25> fixed_rad{},shared_rad{},normalized_output{};
};
// Exact frozen v2 formula. Only SDK history, reference goal and controller
// phase enter Step; no contact state or simulator base velocity is accepted.
class SharedCompensation {
 public:
  explicit SharedCompensation(const std::string& directory);
  ~SharedCompensation();
  SharedCompensation(const SharedCompensation&)=delete;
  SharedCompensation& operator=(const SharedCompensation&)=delete;
  void Reset(const std::string& motion);
  std::array<double,25> Step(const float* input1520,const double* quaternion_wxyz,
      const double* angular_velocity_body,Phase phase,int motion_frame,
      const std::array<double,25>& action_scale);
  const CompensationDiagnostics& diagnostics() const;
  const std::string& root() const;
 private:
  struct Impl;std::unique_ptr<Impl> p_;
};
}
