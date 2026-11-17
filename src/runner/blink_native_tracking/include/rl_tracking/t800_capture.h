#pragma once
#include <array>
#include <cstdint>
#include <string>
namespace data {class DataStore;}
namespace blink {struct Snapshot;struct CompensationDiagnostics;}
namespace blink::telemetry {
enum class MotorStage:uint32_t { SdkView=1,ReceivedFeedback=2,SentCommand=3 };
enum MotorField:uint32_t { Position=1,Velocity=2,Torque=4,CommandPosition=8,
 CommandVelocity=16,FeedForward=32,Kp=64,Kd=128,CommandTorque=256,Current=512,Temperature=1024,Voltage=2048 };
#pragma pack(push,1)
struct PolicyRecord {
 uint64_t captured_ns,wall_ns;int64_t control_ns,imu_ns,encoder_ns,previous_command_ns;
 uint64_t imu_sequence,imu_epoch;uint32_t source_flags;int32_t mode,frame;char motion[64];
 float input[1520];double observation[87],raw[25],shaped[25],target[25];
 double fixed_rad[25],shared_rad[25],diagnostics[5];
};
struct SensorRecord {
 uint64_t captured_ns,wall_ns;int64_t imu_ns,encoder_ns,previous_command_ns;
 uint64_t imu_sequence,imu_epoch;uint32_t source_flags;
 double q[25],dq[25],quaternion[4],gyro[3],previous_target[25],previous_kp[25],previous_kd[25];
};
struct MotorRecord {
 uint64_t captured_ns,wall_ns,sequence,epoch;int64_t source_ns;
 uint32_t stage,valid_fields,source_verified,reserved;
 // All arrays use config.json joint_names (J00..J24). For ankle entries these
 // are RAW MOTOR channels, not virtual pitch/roll feedback.
 double position[25],velocity[25],torque[25],command_position[25],command_velocity[25];
 double feed_forward[25],kp[25],kd[25],command_torque[25],current[25],temperature[25],voltage[25];
};
struct EventRecord {uint64_t captured_ns,wall_ns;int64_t control_ns;int32_t mode,frame;char motion[64],message[256];};
#pragma pack(pop)
// Call before arming. Opening the directory or starting the writer may fail;
// policy Enter rejects that failure. Subsequent disk errors never block control.
void Initialize();
bool Ready() noexcept;
const std::string& SessionDirectory();
void LogPolicy(int64_t control_ns,int mode,int frame,const std::string& motion,
    const Snapshot&,const float* input,const double* observation,const double* raw,
    const double* shaped,const double* target,const CompensationDiagnostics&) noexcept;
void LogSnapshot(const data::DataStore*,const Snapshot&) noexcept;
// Call with the real receive/send packet after a complete receive or a
// successful send. Missing fields remain NaN with their validity bits cleared.
MotorRecord EmptyMotorRecord(MotorStage stage) noexcept;
void LogMotor(const MotorRecord&) noexcept;
void LogEvent(int64_t control_ns,int mode,int frame,const std::string& motion,const std::string& message) noexcept;
}
