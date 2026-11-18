#pragma once
#include <array>
#include <memory>
#include <string>

namespace blink {
// Reserved zero-filled fields retain compatibility with capture v1 readers.
struct CompensationDiagnostics {
  double score=0,ema=0,activation=0,speed_error=0,tilt_error=0;
  std::array<double,3> estimated_velocity{},requested_velocity{};
  std::array<double,25> fixed_rad{},shared_rad{},normalized_output{};
};
}
