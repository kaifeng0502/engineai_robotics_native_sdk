#pragma once
#include <array>
namespace blink {
inline constexpr const char* frozen_names[25]={"J00_HIP_PITCH_L","J01_HIP_ROLL_L","J02_HIP_YAW_L","J03_KNEE_PITCH_L","J04_ANKLE_PITCH_L","J05_ANKLE_ROLL_L","J06_HIP_PITCH_R","J07_HIP_ROLL_R","J08_HIP_YAW_R","J09_KNEE_PITCH_R","J10_ANKLE_PITCH_R","J11_ANKLE_ROLL_R","J12_TORSO_YAW","J13_SHOULDER_PITCH_L","J14_SHOULDER_ROLL_L","J15_SHOULDER_YAW_L","J16_ELBOW_PITCH_L","J17_ELBOW_YAW_L","J18_SHOULDER_PITCH_R","J19_SHOULDER_ROLL_R","J20_SHOULDER_YAW_R","J21_ELBOW_PITCH_R","J22_ELBOW_YAW_R","J23_HEAD_PITCH","J24_HEAD_YAW"};
inline constexpr std::array<double,25> frozen_q0={-0.06,0.0,0.0,0.12,-0.06,0.0,-0.06,0.0,0.0,0.12,-0.06,0.0,0.0,0.0,0.15,0.0,-0.25,0.0,0.0,-0.15,0.0,-0.25,0.0,0.0,0.0};
inline constexpr std::array<double,25> frozen_kp={180.0,100.0,100.0,180.0,40.0,40.0,180.0,100.0,100.0,180.0,40.0,40.0,100.0,50.0,50.0,50.0,50.0,50.0,50.0,50.0,50.0,50.0,50.0,50.0,50.0};
inline constexpr std::array<double,25> frozen_kd={5.0,3.0,3.0,5.0,2.0,2.0,5.0,3.0,3.0,5.0,2.0,2.0,3.0,0.3,0.3,0.3,0.3,0.3,0.3,0.3,0.3,0.3,0.3,0.3,0.3};
inline constexpr std::array<double,25> frozen_scale={0.5,0.2,0.2,0.5,0.5,0.2,0.5,0.2,0.2,0.5,0.5,0.2,0.2,0.2,0.2,0.05,0.2,0.2,0.2,0.2,0.05,0.2,0.2,0.2,0.2};
inline constexpr std::array<double,25> frozen_effort={394.25,370.0,222.0,394.25,160.0,160.0,394.25,370.0,222.0,394.25,160.0,160.0,222.0,160.0,160.0,160.0,160.0,52.0,160.0,160.0,160.0,160.0,52.0,52.0,52.0};
}
