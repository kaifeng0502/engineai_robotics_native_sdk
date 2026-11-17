#include "rl_tracking/blink_sdk_state.h"
#include "rl_tracking/frozen_profile.h"

#include <stdexcept>

namespace blink {
std::array<int,25> SdkJointOrder(const data::DataStore& store) {
  if (store.model_param->num_total_joints != 25) {
    throw std::runtime_error("BlinkMimic requires the 25-joint T800 model");
  }
  std::array<int,25> indices{};
  for (size_t i = 0; i < indices.size(); ++i) {
    indices[i] = store.model_param->joint_id_in_total_limb.at(frozen_names[i]);
  }
  return indices;
}

Snapshot ReadSdkSnapshot(data::DataStore& store,const std::array<int,25>& sdk_order) {
  const auto imu = store.imu_info.Get();
  if (!imu || !imu->quaternion.coeffs().allFinite() || !imu->angular_velocity.allFinite()) {
    throw std::runtime_error("invalid SDK IMU state");
  }
  using data::JointInfoType;
  thread_local Eigen::VectorXd q, dq, target, kp, kd;
  auto& joints = store.joint_info;
  joints.GetState(JointInfoType::kPosition, q);
  joints.GetState(JointInfoType::kVelocity, dq);
  joints.GetCommand(JointInfoType::kPosition, target);
  joints.GetCommand(JointInfoType::kStiffness, kp);
  joints.GetCommand(JointInfoType::kDamping, kd);
  for (const auto* values : {&q,&dq,&target,&kp,&kd}) {
    if (values->size() != 25 || !values->allFinite()) {
      throw std::runtime_error("invalid SDK joint state or command");
    }
  }
  Snapshot sample;
  for (size_t i = 0; i < sdk_order.size(); ++i) {
    const int j = sdk_order[i];
    sample.packet.q[i] = q[j];
    sample.packet.dq[i] = dq[j];
    sample.previous_target[i] = target[j];
    sample.previous_kp[i] = kp[j];
    sample.previous_kd[i] = kd[j];
  }
  const auto& orientation = imu->quaternion;
  sample.packet.quaternion_wxyz = {orientation.w(),orientation.x(),orientation.y(),orientation.z()};
  for (int i = 0; i < 3; ++i) sample.packet.angular_velocity_body[i] = imu->angular_velocity[i];
  return sample;
}
}
