# T800：21 个完整动作，共用第 23000 轮模型

本交付基于 `feat/blink-direct-start`（基线提交 `d4a57c26d6ef4e8150ea189ae63559ea35361157`），交付分支为 `feat/blink-full21-20261118`。

## 定版范围

- 21 个动作只使用一个动作模型：`assets/config/t800/blink_native/shared/tracking_policy.mnn`，FP32，34,849,828 字节。21 个动作 runner 共享同一个 MNN 推理实例。
- 另有每个动作的完整参考数据，共 3252 帧。模型与参考配置合计约 38.36 MB（十进制）。遥控触发时使用当前实测状态，不重置机器人，不增加入拳架、保持拳架或收势动作。完成后直接交回原生 SDK 行走/站立策略。
- 排除「勾拳接上勾拳」（`05_hook_uppercut`）和「右勾拳接左反手拳」（`17_right_hook_left_backfist_final`）。交付中没有它们的参考文件、任务入口或遥控按钮。共享模型曾对全部 23 个动作训练，此次仅提供获准的 21 个动作入口。
- **540 度旋踢保留，但仍有腿部碰撞**。本次 6 次仿真均完成并回站立，腿部接触冲量超过严格检查阈值，未标为完全通过。
- 单模型指上述 21 个动作模型。SDK 原有行走/站立等功能的模型仍保留，不能删除行走模型。

## 验证结果与边界

| 验证 | 结果 |
|---|---|
| 转换前 ONNX 与 FP32 MNN，64 组输入 | 最大原始动作差异 0.00007391 |
| 转换后 MNN：21 动作 × 6 个站立起点/朝向 | 126/126 完成，126/126 稳定接回站立 |
| 严格动作检查：完成、回站稳定、腿部接触、转角 | 120/126 通过；仅旋踢的 6 次未通过 |
| ARM64 真实 SDK runner，完整逐帧固定输入回放 | 21 个动作、3231 步通过；目标角最大差异 0.00006402 rad（约 0.0037°） |
| SDK 回站立控制的前 20 步 × 21 动作 | 目标角最大差异 0.00004566 rad |
| SDK 模型实例共享、完成时序、非有限值处理 | 通过 |

交付 XML 的物理步长已从 SDK 原默认 2 ms 对齐到本次验证的 1 ms。闭环仿真采用原生 T800 MuJoCo 模型、自碰撞开启、1 ms 物理步长、50 Hz 动作策略、100 Hz 行走策略。第一组站立时间 3/3.37/3.74 秒、初始朝向 0°；第二组 1.5/4.11/7.03 秒、初始朝向 90°；每次动作后继续验证 5 秒。严格阈值和原始数据见 `validation/full21/mnn_primary.json` 与 `mnn_holdout.json`；这组结论针对本次单模型，不是旧的多模型拼接结果。

ARM64 测试使用真实 C++ SDK 的状态读取、观测历史、模型推理、任务完成及行走 runner，输入为仿真记录；并非完整机器人通信链路的闭环测试。尚未在实机执行。SDK runner 发出原始虚拟关节目标；MuJoCo 在每个物理步执行 PD 并按 XML 的执行器范围限矩。部署方需核对实机驱动的限矩、并联踝转换与 PD 参数（包含上肢 Kd=0.3）是否与配置一致。不要把旧版 50 Hz 的目标限矩改写或额外补偿重新加入此模型输入/输出链路。

## 编译与打包

在源码仓库根目录：

```bash
bash build.sh -a aarch64 -r humble -t release -j 4
bash scripts/cross_compile_env.sh run 'python3 tools/test_full21.py'
python3 tools/package_full21.py
```

实测环境为本机 Docker、官方 Humble 交叉编译镜像、4 个编译线程：首次完整编译 **168.66 秒（2 分 49 秒）**；修改 runner 后的增量编译 **19.21 秒**。镜像已缓存，这两个时间均不含首次下载镜像。部署方硬件与缓存不同，耗时会不同。模型已转换好，编译不需要重新训练或重新转换模型。

本地打包工具只产生 `dist/T800_full21_23000_20261118_arm64.tar.gz`、JSON 清单和 SHA256 文件；不连接机器人。压缩包包含 ARM64 编译产物、运行依赖、T800 资源、21 个动作、遥控器、指标工具与验证报告；不包含训练目录。本次 ARM64 压缩包约 **133 MB**，解压普通文件合计约 **254.7 MB**（十进制；文件系统占用会因符号链接、块大小而不同）。精确值以归档同名 JSON 的 `archive_bytes` 和 `unpacked_regular_file_bytes` 为准。

## 部署方使用

1. 解压交付包，在包根目录执行 `sha256sum -c SHA256SUMS`。
2. 包面向 ARM64 Linux / ROS 2 Humble；仍需要部署方机器上的 ROS 和 EngineAI 硬件驱动。包的 `assets/config/t800/mode.yaml` 默认保持 `active_mode: sim`。实机模式由部署方按其已验证的硬件部署流程配置。
3. 包内 `run_robot.sh` 应在包根目录运行；它沿用官方启动流程。仿真启动使用 `ENGINEAI_ROBOTICS_RUNTIME_ENV=sim bash run_robot.sh t800`，并连接部署方的原生 MuJoCo 仿真端。此 ARM64 包不包含 Mac 或 x86_64 的仿真可执行程序。
4. 虚拟遥控器：安装 `tools/virtual_gamepad/requirements.txt` 后，运行 `python3 tools/virtual_gamepad/virtual_gamepad.py`。连接地址应与 SDK 的 LCM 配置一致，默认 `udpm://239.255.76.67:7667`。21 个按钮直接读取交付清单，无需手动改代码。
5. 按 SDK 原流程进入 `walk`、松开摇杆站稳，再触发动作。`LB+A` 为 PD 站立，`LB+B` 为行走/站立，`LB+RB` 为被动模式。动作中不接受第二个动作触发；允许切回 walk 或 passive。完整结束后自动切回 walk。
6. 实体手柄先按住表中的两个修饰键，再按第三键触发；松开第三键后再松修饰键。虚拟遥控器已提供完整组合按钮。

## 动作与遥控映射

| SDK 任务 | 动作 | 完整片段时长 | 组合键 |
|---|---|---:|---|
| `blink_01` | 前方格挡 | 3.18 s | RB + START + A |
| `blink_02` | 540°旋踢（仍有腿部接触） | 4.14 s | RB + START + B |
| `blink_03` | 左闪躲 | 3.04 s | RB + START + X |
| `blink_04` | 右闪躲 | 2.96 s | RB + START + Y |
| `blink_05` | 右闪躲接左踢 | 3.34 s | RB + START + CROSS_X_UP |
| `blink_06` | 后闪躲 | 3.90 s | RB + START + CROSS_X_DOWN |
| `blink_07` | 左闪躲接右踢 | 4.22 s | RB + START + CROSS_Y_LEFT |
| `blink_08` | 左臂下砸 | 2.46 s | RB + START + CROSS_Y_RIGHT |
| `blink_09` | 右踢接膝击 | 2.70 s | RB + BACK + A |
| `blink_10` | 左侧格挡 | 2.48 s | RB + BACK + B |
| `blink_11` | 左侧踢 | 2.54 s | RB + BACK + X |
| `blink_12` | 左肘击 | 2.36 s | RB + BACK + Y |
| `blink_13` | 左勾拳接右反手拳 | 3.22 s | RB + BACK + CROSS_X_UP |
| `blink_14` | 左踢 | 3.24 s | RB + BACK + CROSS_X_DOWN |
| `blink_15` | 左膝击 | 2.86 s | RB + BACK + CROSS_Y_LEFT |
| `blink_16` | 左拳接右踢 | 4.04 s | RB + BACK + CROSS_Y_RIGHT |
| `blink_17` | 右低踢 | 3.46 s | LB + BACK + A |
| `blink_18` | 右臂下砸 | 2.78 s | LB + BACK + B |
| `blink_19` | 右肘击 | 2.36 s | LB + BACK + X |
| `blink_20` | 右踢 | 2.64 s | LB + BACK + Y |
| `blink_21` | 右膝击 | 2.70 s | LB + BACK + CROSS_X_UP |

方向键 `CROSS_X_UP/DOWN` 表示上/下，`CROSS_Y_LEFT/RIGHT` 表示左/右。动作名称中的“540”保留原始命名，参考文件根部净转角约 717°；验收按完整 NPZ 的实际转角判断。

## 实时 TensorBoard 指标

SDK 自动记录到 `BLINK_LOG_DIR` 指定目录，未设置时为工作目录下的 `t800_logs`。每次进程创建一个独立会话目录。读取工具只读日志，独立于控制线程：

```bash
python3 -m pip install numpy tensorboardX tensorboard
python3 tools/capture_tensorboard.py t800_logs/实际会话目录 --output runs/full21 --follow
tensorboard --logdir runs/full21 --port 6006
```

每个动作在 `tracking/动作名/` 下显示：

- `joint_rmse_deg`：当前测量关节角与对应参考帧的均方根误差，越小越好。
- `root_orientation_error_deg`：触发时航向对齐后的躯干朝向误差，越小越好。
- `progress`：完整参考片段的执行进度；收到 SDK 完成事件时为 1。
- `completed`：收到该次动作完成事件为 1；不能单独代表动作质量合格。
- `invocation`：当前是第几次触发。

这些指标来自 SDK 的实测关节和 IMU，不是策略输出自评。实机 SDK 日志没有可直接证明腿部接触冲量的信号，因此未伪造仿真接触/离地高度指标。详细脚部碰撞结论见 MuJoCo 回归报告。停止日志读取按 Ctrl+C，不会操作机器人。

## 推理契约与版本识别

- 原始 checkpoint：23000；原始 checkpoint 和 ONNX 的 SHA256、交付 MNN SHA256 均在 `assets/config/t800/blink_native/manifest.json`。
- 输入展平为 1520：10×87 的观测历史，后接 650 维参考目标；输出 25 维原始动作。MNN 的单输入适配仅加入切片/重排，没有再训练或量化。
- 87 维观测：关节角偏差 25、关节速度 25、机体系角速度 3、投影重力 3、上次原始动作 25、当前与参考相对旋转前两列 6。
- 触发：上次原始动作置零；观测历史用当前实测观测填满；只对齐参考航向。SDK 的上一控制目标不会混入动作。
- 输出：`q_target = default_joint_pos + action_scale * raw_action`。不裁剪原始动作，不混合入场目标，不运行检测器、速度估计器或额外补偿网络。
- N 帧参考构成 N−1 个 20 ms 物理区间；最后一帧为片段终点，不再追加停留。SDK 外层调度 200 Hz、动作推理 50 Hz。
- 动作结束重新初始化行走策略的历史与上次动作；行走模型缓存复用，并对本次动作返回关闭旧版站立插值。
- 模型及参考文件在 SDK 初始化时加载并校验 SHA256，触发时不读取模型文件。参考 NPZ 使用 ZIP32 文件头以兼容原 SDK cnpy 读取器。
- `validation/full21/sdk_fixture.npz` 为源码验证夹具，包含获准 21 个动作的固定输入。为减小部署包，不复制进运行归档。

训练方法参考 [BlinkMimic](https://github.com/fuyichen0725/blinkmimic)；SDK 沿用仓库原有许可证，模型为本项目训练的 checkpoint 23000。
