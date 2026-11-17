#pragma once

#include "data_store/data_store.h"
#include "sdk87_core.h"

namespace blink {
struct Snapshot {
  sdk87::Packet packet;
  std::array<double,25> previous_target{},previous_kp{},previous_kd{};
};

std::array<int,25> SdkJointOrder(const data::DataStore& store);
Snapshot ReadSdkSnapshot(data::DataStore& store,const std::array<int,25>& sdk_order);
}
