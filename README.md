# video-learning-infra

`video-learning-infra` 是一个围绕“视频感知 -> 姿态/动作理解 -> 机械系统执行”的实验型仓库。

当前仓库已经完成第一轮结构整理，形成了一个比较稳定的双子系统架构：

- `arm_control/`
  负责机械臂侧能力，包括标定、位姿变换、IK、安全检查、仿真、真机控制、FastAPI 和 Web 可视化。
- `hand_control/`
  负责灵巧手侧能力，包括手部检测、retargeting、关节映射和手侧控制。
- `assets/` 与 `third_party/`
  负责承接真实共享的机器人模型和 SDK。

这份 README 按“项目交接文档”来写，目标不是介绍所有细节，而是帮助新接手的人尽快回答下面这些问题：

- 这个仓库现在的真实边界是什么
- 我应该从哪里启动 arm / hand
- 哪些配置文件最关键
- 如果我只想跑通一条链路，第一步该做什么
- 如果我要继续开发，当前有哪些明确约定

## 1. 一分钟总览

你可以把当前仓库理解成三层：

1. 顶层稳定入口层  
   用 `scripts/arm/run.py` 和 `scripts/hand/run.py` 作为推荐启动入口。
2. 子系统实现层  
   `arm_control/` 和 `hand_control/` 各自保留内部组织，不强行揉成一个大包。
3. 共享资源层  
   真实共享的机器人模型在 `assets/robots/`，真实共享的 SDK 在 `third_party/robotic_arm_package/`。

当前已经明确的项目原则：

- `arm_control/` 和 `hand_control/` 是平级子系统
- 顶层 `scripts/` 是稳定入口层
- 只有真实共享的内容才会上移到顶层
- 私有配置、私有示例、私有调试脚本先留在各自子系统内部

## 2. 当前仓库能做什么

### 2.1 arm 侧

当前 arm 侧已经具备这些能力：

- 读取相机标定参数，把 `camera frame` 下的目标位姿转换到机器人 `base frame`
- 做位姿映射、滤波、工作空间安全检查和 IK 求解
- 在 SAPIEN 中做最小仿真验证
- 通过 RealMan SDK 连真机执行
- 通过 FastAPI 暴露后端接口
- 通过 React + Vite 前端做基础可视化
- 通过若干 debug 脚本做 bring-up 和联调

### 2.2 hand 侧

当前 hand 侧已经具备这些能力：

- 从 RealSense、本地摄像头、视频文件或 MJPEG 流读取手部图像
- 做单手检测和 retargeting
- 把手部动作映射为 6 路手侧控制量
- 直接驱动灵巧手
- 也可以把数据先通过 HTTP 流暴露出来，再由控制端读取

### 2.3 shared 层

当前已经提取出来的共享内容：

- `assets/robots/arms/`
- `assets/robots/hands/`
- `third_party/robotic_arm_package/`

## 3. 当前目录结构

```text
video-learning-infra/
  .vscode/
  arm_control/
  hand_control/
  assets/
    robots/
      arms/
      hands/
  third_party/
    robotic_arm_package/
  scripts/
    arm/
      run.py
    hand/
      run.py
  repo_paths.py
  README.md
```

当前目录语义：

- `repo_paths.py`
  仓库级路径真源。仓库根、子系统根和共享目录定位从这里出发。
- `scripts/`
  推荐启动入口层。后续即使子系统内部目录继续调整，也优先保持这里稳定。
- `arm_control/`
  arm 侧真实项目根。
- `hand_control/`
  hand 侧真实项目根。

## 4. 建议的阅读顺序

如果你是第一次接手这个项目，建议按下面顺序阅读：

1. 先读本文件，了解仓库级边界和推荐入口
2. 如果要接 arm，读 `arm_control/README.md`
3. 如果要接 hand，读 `hand_control/README.md`

## 5. 环境准备

### 5.1 Python

建议使用：

- `Python 3.10+`

说明：

- `arm_control/requirements.txt` 已经给出了 arm 侧最低依赖集合
- hand 侧当前没有单独整理出完整 requirements 文件，依赖主要来自运行链路本身

arm 侧当前显式依赖：

- `numpy`
- `PyYAML`
- `pytest`
- `sapien==3.0.0b0`
- `fastapi`
- `uvicorn`

hand 侧运行时通常还需要：

- `opencv-python`
- `requests`
- `pyyaml`
- `fastapi`
- `uvicorn`
- `pyrealsense2`
- `dex_retargeting` 相关依赖

### 5.2 Node.js

如果你要跑 arm 的 Web 前端，还需要：

- `Node.js`
- `npm`

前端目录在：

- `arm_control/web/`

### 5.3 硬件与本地依赖

如果你要接真机，还需要确认：

- RealMan 机械臂网络可达
- RealSense 或其他相机设备可正常访问
- Windows 下 RealMan SDK 动态库可用

## 6. 最推荐的启动方式

统一约定：

- 始终从仓库根目录启动
- 优先使用顶层 wrapper，不直接在子目录随手运行脚本

### 6.1 arm 侧

启动 FastAPI 后端：

```bash
python scripts/arm/run.py api
```

启动最小仿真入口：

```bash
python scripts/arm/run.py sim
```

打开 RealSense 预览并写回相机内参：

```bash
python scripts/arm/run.py camera --write-config
```

验证像素 + 深度到机器人 base 坐标的换算：

```bash
python scripts/arm/run.py calibration-val
```

常用 debug 命令：

```bash
python scripts/arm/run.py debug-realman-terminal
python scripts/arm/run.py debug-pipeline-realman
python scripts/arm/run.py debug-click-to-realman
python scripts/arm/run.py debug-sapien-scene
python scripts/arm/run.py debug-sapien-camera
```

### 6.2 hand 侧

本地一体化模式：

```bash
python scripts/hand/run.py controller --input_mode local --source realsense --hand_type right --show_window
```

只打印映射后的关节值：

```bash
python scripts/hand/run.py controller --input_mode local --debug_joints --source realsense
```

把手部数据作为 HTTP 流暴露出来：

```bash
python scripts/hand/run.py stream --mode http --source realsense --show_window
```

再由控制端读取 HTTP 流并执行：

```bash
python scripts/hand/run.py controller --input_mode http --stream_url http://127.0.0.1:50001/hand_data
```

## 7. 第一次接手时最应该检查的配置

### 7.1 arm 侧

优先检查：

- `arm_control/config/robot.yaml`
- `arm_control/config/calibration.yaml`
- `arm_control/config/workspace.yaml`

它们分别负责：

- `robot.yaml`
  机器人模型、URDF、IK、RealMan 连接、仿真参数、映射参数
- `calibration.yaml`
  相机内参、外参与仿真相机位姿
- `workspace.yaml`
  工作空间边界和安全策略(暂时没有使用)

如果 arm 真机连不上，先看：

- `arm_control/config/robot.yaml` 中的 `realman` 配置

如果相机坐标变换异常，先看：

- `arm_control/config/calibration.yaml`

### 7.2 hand 侧

优先检查：

- `hand_control/configs/arm_connection.yaml`
- `hand_control/configs/revo2/revo2_hand_left_dexpilot.yml`
- `hand_control/configs/revo2/revo2_hand_right_dexpilot.yml`

说明：

- `arm_connection.yaml`
  当前 hand 使用的机械臂连接配置
- `revo2/*.yml`
  手部 retargeting 配置

兼容文件：

- `hand_control/configs/settings.yaml`
  当前只是兼容清单，不再是主配置入口
- `hand_control/configs/legacy_settings_unused.yaml`
  旧设置归档，当前运行链路不依赖

## 8. 子系统内部该从哪里读代码

### 8.1 arm 侧建议入口

如果你要理解 arm 主链路，最重要的文件是：

- `arm_control/core/pipeline.py`
  核心执行主线
- `arm_control/core/calibration.py`
  相机系到机器人基坐标系的变换逻辑
- `arm_control/core/robot_controller.py`
  推荐使用的 RealMan 真机控制封装
- `arm_control/api/state.py`
  后端运行态与配置装配
- `arm_control/sim/sapien_controller.py`
  SAPIEN 侧最核心控制器

### 8.2 hand 侧建议入口

如果你要理解 hand 主链路，最重要的文件是：

- `hand_control/example/revo2_retargeting/teleop.py`
  手部检测 + retargeting + 数据流入口
- `hand_control/example/revo2_retargeting/brainco_controller.py`
  手侧控制入口
- `hand_control/example/revo2_retargeting/single_hand_detector.py`
  手部检测模块

## 9. Web 前端

arm 的前端位于：

- `arm_control/web/`

安装并启动：

```bash
cd arm_control/web
npm install
npm run dev
```

如果只想构建：

```bash
cd arm_control/web
npm run build
```

## 10. 测试

当前测试主要集中在 arm 侧，目录在：

- `arm_control/tests/`

从仓库根运行：

```bash
pytest arm_control/tests
```

注意：

- 某些测试依赖本地 Python 环境是否装好了 `fastapi` 等包
- 涉及 SAPIEN、真机、RealSense 的链路仍更偏本地集成验证，而不是纯单元测试

## 11. 当前开发约定

这是当前仓库最重要的几条约定：

1. 新入口优先放到顶层 `scripts/`
2. 新共享资源优先放到顶层 `assets/` 或 `third_party/`
3. 配置只有在 arm / hand 真实复用时才提升到顶层
4. 子系统私有配置和私有调试脚本先保留在原地
5. 路径定位优先围绕仓库根来组织，不新增依赖当前工作目录的隐式写法

当前路径规则的真实来源：

- `repo_paths.py`

如果看到 `arm_control/paths.py`，可以把它理解成兼容层，而不是新的路径真源。

## 12. 常见问题

### 12.1 为什么推荐从顶层 wrapper 启动

因为仓库已经做过一轮目录调整，直接从子目录跑旧脚本很容易踩到：

- `cwd` 依赖
- 相对路径解析不一致
- `third_party` 无法导入

用顶层 wrapper 的目的，就是把这些路径差异先屏蔽掉。

### 12.2 如果 IDE 提示 `robotic_arm_package` 无法解析怎么办

这是静态分析路径问题，不一定是运行时真的导不进来。

当前工作区已经在 `.vscode/settings.json` 里补了：

- 仓库根
- `third_party`
- `hand_control/example/revo2_retargeting`

如果 IDE 还没刷新，一般重载窗口或重启 Python language server 即可。

### 12.3 如果 hand 侧链路跑不起来，先查哪里

优先检查：

- `hand_control/configs/arm_connection.yaml`
- `pyrealsense2` 是否可用
- `dex_retargeting` 是否装好
- `scripts/hand/run.py` 是否从仓库根执行

### 12.4 如果 arm 真机链路跑不起来，先查哪里

优先检查：

- `arm_control/config/robot.yaml`
- `third_party/robotic_arm_package/`
- 本机能否连到 RealMan IP
- 是否先用 `debug-realman-terminal` 做最小连通性验证

## 13. 还没彻底收尾的地方

当前架构已经能作为阶段性稳定版本使用，但还不是终局：

- `config` / `configs`
- `example` / `examples`
- 某些 hand 侧“example”目录里的文件实际是运行代码
- 部分文档仍然更偏子系统视角，需要后续继续收口

这些不影响当前继续开发和做版本管理，但意味着后面仍会有小步整理。

## 14. 进一步阅读

如果你要继续深入，可以看：

- `arm_control/README.md`
  arm 子系统更细的交接文档
- `hand_control/README.md`
  hand 子系统说明

## 15. 安全说明

这个仓库涉及真实机械臂、灵巧手和相机设备。

在真机环境下调试时，建议始终遵守：

1. 先仿真、再真机
2. 先低速、再放开速度
3. 先小范围动作、再扩大动作范围
4. 优先用 dry-run 和最小调试入口验证链路
5. 修改 `robot.yaml`、`workspace.yaml`、`arm_connection.yaml` 后先做最小连通性检查

如果你只记住一句话，那就是：

从仓库根使用 `scripts/arm/run.py` 和 `scripts/hand/run.py` 启动，并把 `arm_control/` / `hand_control/` 当作两个平级子系统来理解。

