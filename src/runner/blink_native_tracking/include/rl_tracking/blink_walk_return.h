#pragma once

#include <atomic>
#include <chrono>
#include <memory>

#include "data_store/data_store.h"
#include "variant_store/variant_store.h"

namespace blink {

struct WalkingPositionBlend {
  std::weak_ptr<const data::DataStore> owner;
  std::chrono::steady_clock::time_point requested_at;
  Eigen::VectorXd preceding_target;
  std::atomic<bool> pending{true};
};

inline constexpr char kWalkingPositionBlend[] = "blinkmimic.walking_position_blend";

inline void RequestWalkingPositionBlend(const std::shared_ptr<data::DataStore>& owner) {
  auto handoff = std::make_shared<WalkingPositionBlend>();
  handoff->owner = owner;
  handoff->requested_at = std::chrono::steady_clock::now();
  owner->joint_info.GetCommand(data::JointInfoType::kPosition, handoff->preceding_target);
  data::VariantStore::GetInstance().SetShared(kWalkingPositionBlend, std::move(handoff));
}

inline bool TakeWalkingPositionBlend(const std::shared_ptr<data::DataStore>& owner,
                                    const Eigen::VectorXd& preceding_target) {
  auto handoff = data::VariantStore::GetInstance().GetMutable<WalkingPositionBlend>(kWalkingPositionBlend);
  if (!handoff || handoff->owner.lock() != owner || !handoff->pending.exchange(false)) {
    return false;
  }
  return std::chrono::steady_clock::now() - handoff->requested_at <= std::chrono::milliseconds(150) &&
         handoff->preceding_target.size() == preceding_target.size() &&
         handoff->preceding_target.isApprox(preceding_target, 1e-12);
}

}  // namespace blink
