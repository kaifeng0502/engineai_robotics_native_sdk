# T800 官方134维 · 12000轮 · 部署工程源码包

本包动作学习策略仅含累计12000轮检查点；23个动作共用一个MNN。SDK基准为 c8a23eee6adf1276cabcab27219f8d16db104a39，保留原SDK源码、构建/运行脚本、原虚拟遥控器、T800模型网格纹理和69个官方core依赖库。SDK原有行走/起身模型是基础功能依赖，不是其他轮次的动作学习策略。

## 模型与动作

- 官方134维输入，6个观测字段的历史均为1帧，输出25维，归一化已内置；不添加137维重力字段或650维未来输入。
- 观测顺序：command(50)、motion_anchor_ori_b(6)、base_ang_vel(3)、joint_pos(25)、joint_vel(25)、actions(25)。
- 策略周期20ms（50Hz）。每段包含0.5秒默认站姿入场插补、完整原NPZ动作、1秒回默认站姿、1秒稳定保持。
- 模型：assets/config/t800/official134_12000/shared/tracking_policy.mnn。
- 23个动作及对应配置：assets/config/t800/official134_12000/motions/。
- 所有配置与模型均为12000轮版；J17/J22动作缩放为0.05，与训练和RL Lab导出配置一致。

## 使用

沿用SDK依赖环境，在仓库根目录按build.sh构建。默认mode.yaml为sim。先通过原遥控器进入pd_stand（LB+A），等待默认站姿稳定后，再按《动作按键表.csv》选择动作；动作完成后转回pd_stand。跟踪任务使用官方rl_dance_example_runner和显式参数tag/scope映射，C++观测注册表与runner保持官方实现。入场插补是训练参考轨迹的一部分，不会将机器人姿态重置到动作首帧。

训练使用四踝速度观测噪声±3rad/s，以及每次指令随机6/7/8/9ms响应延迟；这些是训练扰动，本部署配置不向真实控制信号人工加噪或延时。

## 交付检查和范围

已核对原12000轮检查点身份、203组PyTorch/ONNX/MNN输出一致性、23个动作资源与按键、参数scope、134→25维度和文件SHA256。校验及该轮已有仿真评估在validation/official134_012000/，详细元数据在SOURCE_DELIVERY.json。尚未进行本源码包的完整原生SDK编译或真机验证。

排除训练.pt、ONNX、训练环境、build/install/dist、.git、训练日志、视频和其他机器人资源。SHA256SUMS覆盖包内文件；SDK原始通用说明仅供框架参考，以本文件与official134_12000/manifest.json为当前版本依据。

## GitHub分支

仓库：kaifeng0502/engineai_robotics_native_sdk。分支：feat/t800-official134-12000-20261119。源码直接位于仓库根目录；可通过GitHub的Code → Download ZIP下载。本次动作策略仅12000轮，23段动作共享同一模型。SDK通用构建说明见 docs/SDK_FRAMEWORK_README_CN.md。
