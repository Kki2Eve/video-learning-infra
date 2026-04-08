# arm_control

`arm_control/` 是当前仓库里的 arm 子系统，负责把上游目标位姿或交互输入转换成机械臂可执行动作。

如果只用一句话概括这部分代码，它做的是：

`目标位姿/点击输入 -> 标定变换 -> 位姿映射 -> 滤波与安全检查 -> IK -> 仿真或真机执行`

这份 README 按“arm 子系统交接文档”来写，重点帮助接手的人快速回答这些问题：

- arm 侧的真实入口在哪里
- 仿真、API、真机和前端分别怎么启动
- 第一次运行前该检查哪几个配置
- 如果要改 arm 主流程，应该从哪些文件开始看

## 1. 当前边界

当前 arm 子系统的边界是清楚的：

- `arm_control/` 是 arm 侧真实项目根
- 推荐统一从仓库根使用顶层入口 `python scripts/arm/run.py ...`
- 机器人模型会复用顶层 `assets/robots/`
- RealMan SDK 会复用顶层 `third_party/robotic_arm_package/`
- arm 私有配置仍保留在 `arm_control/config/`

也就是说，当前 arm 侧不是一个单独仓库，而是顶层 monorepo 里的一个平级子系统。

## 2. 当前能做什么

当前 arm 侧已经具备这些能力：

- 读取相机标定参数，把 `camera frame` 下的目标位姿转换到机器人 `base frame`
- 做位姿映射、滤波、工作空间安全检查和 IK 求解
- 在 SAPIEN 中做最小仿真验证
- 通过 RealMan SDK 连真机执行
- 通过 FastAPI 暴露调试接口
- 通过 React + Vite 前端做基础可视化
- 通过多种 debug 脚本做 bring-up 和联调

当前主要支持两类工作流：

1. 上游算法输出 `camera frame` 下的目标位姿，arm 侧完成变换、IK、安全检查，并在仿真或真机执行。
2. 操作者在 RealSense 画面中点击像素点，结合深度恢复 3D 点，再构造目标末端姿态驱动真机。

## 3. 主链路概览

当前统一执行主线由 `TeleopPipeline` 负责，核心流程如下：

```text
pose_cam
  -> calibration transform
  -> pose mapping
  -> pose filtering
  -> safety check
  -> IK
  -> execution (sim / real)
```

如果输入本来已经是机器人 `base frame` 下的目标位姿，则会跳过相机系到机器人基坐标系的变换，直接从 IK 开始执行。

## 4. 目录结构

下面只列出当前最重要的目录和文件：

```text
arm_control/
  config/
    calibration.yaml
    robot.yaml
    workspace.yaml
  core/
    calibration.py
    ik_solver.py
    pipeline.py
    pose_filter.py
    pose_mapper.py
    pose_types.py
    robot_controller.py
    safety_guard.py
    transform_utils.py
  api/
    app.py
    schemas.py
    services.py
    state.py
  sim/
    sapien_controller.py
    sim_controller.py
    visualizer.py
  debug/
    debug_realman_controller_terminal.py
    debug_pipeline_realman_runner.py
    debug_click_to_realman.py
    debug_sapien_scene.py
    debug_sapien_camera.py
  examples/
    camera.py
    calibration_val.py
    pose_cam_single.json
    trajectory_cam_demo.json
  hand_eye_calibration/
  web/
  tests/
  run_api.py
  run_sim.py
  README.md
```

这些目录的语义分别是：

- `config/`
  arm 私有配置入口
- `core/`
  主执行链路和基础算法模块
- `api/`
  FastAPI 后端
- `sim/`
  SAPIEN 仿真控制
- `debug/`
  bring-up 和联调入口
- `examples/`
  输入样例和少量调试工具
- `web/`
  React + Vite 前端
- `tests/`
  arm 侧测试

## 5. 推荐启动方式

当前统一约定：

- 始终从仓库根目录启动
- 优先使用顶层入口 [scripts/arm/run.py](/d:/video_dex_infra/video-learning-infra/scripts/arm/run.py)

### 5.1 启动 FastAPI 后端

```bash
python scripts/arm/run.py api
```

适合场景：

- 给前端提供接口
- 调试 API 层
- 验证状态组织与变换接口

### 5.2 启动最小仿真入口

```bash
python scripts/arm/run.py sim
```

适合场景：

- 不接真机时先验证整条链路
- 调试位姿映射、IK 和安全检查

### 5.3 打开 RealSense 预览并写回内参

```bash
python scripts/arm/run.py camera
python scripts/arm/run.py camera --write-config
```

适合场景：

- 检查相机画面
- 更新 `calibration.yaml` 中的内参

### 5.4 验证像素 + 深度到 base 坐标换算

```bash
python scripts/arm/run.py calibration-val --u 307 --v 276 --depth 0.79
```

适合场景：

- 检查相机标定是否合理
- 验证像素点反投影和坐标变换是否正常

### 5.5 常用 debug 命令

```bash
python scripts/arm/run.py debug-realman-terminal status
python scripts/arm/run.py debug-realman-terminal home --execute
python scripts/arm/run.py debug-pipeline-realman --kind single
python scripts/arm/run.py debug-pipeline-realman --input-file arm_control/examples/pose_cam_single.json
python scripts/arm/run.py debug-click-to-realman
python scripts/arm/run.py debug-sapien-scene
python scripts/arm/run.py debug-sapien-camera
```

## 6. 第一次运行前先检查什么

### 6.1 先检查机器人配置

优先看：

- `config/robot.yaml`

它负责：

- 机器人模型与 URDF 路径
- 受控关节
- 关节限制
- 映射参数
- IK 配置
- RealMan 真机配置
- 仿真参数

当前最关键的字段包括：

- `robot.urdf_path`
- `robot.controlled_joints`
- `mapping.hand_to_tool_euler_rpy`
- `ik.backend`
- `realman.ip`
- `realman.port`
- `realman.default_speed`

如果真机连不上，第一优先级先看：

- `config/robot.yaml` 里的 `realman.ip` 和 `realman.port`

### 6.2 再检查标定配置

优先看：

- `config/calibration.yaml`

它负责：

- 相机内参
- 相机到机器人基坐标系的外参
- 仿真相机位姿

当前最关键的字段包括：

- `camera_intrinsics`
- `camera_to_robot_base`
- `sim_camera.pose_world`

如果像素点转换到 base 坐标异常，第一优先级先看：

- `config/calibration.yaml`

### 6.3 再检查工作空间与安全限制

优先看：

- `config/workspace.yaml`

它负责：

- 工作空间边界
- 最大线速度和角速度
- `z_min_m`
- 是否 clamp 到 workspace
- IK 失败时是否停止
- 碰撞 padding

如果末端目标总被拒绝或执行被限幅，优先先看这个文件。

## 7. 主链路应该从哪里读

如果你要理解 arm 子系统，建议按下面顺序看代码：

1. `core/pose_types.py` 和 `core/transform_utils.py`
2. `core/calibration.py`
3. `core/pose_mapper.py`
4. `core/pose_filter.py`
5. `core/safety_guard.py`
6. `core/ik_solver.py`
7. `core/pipeline.py`
8. `core/robot_controller.py`
9. `api/state.py`
10. `sim/sapien_controller.py`

### 7.1 `core/pipeline.py`

这是 arm 侧最核心的执行主线，负责把“输入目标位姿”变成“可以执行的机械臂动作”。

它主要负责：

- camera frame 到 robot base 的变换
- pose mapping
- pose filter
- safety check
- IK 求解
- 仿真或真机执行

### 7.2 `core/calibration.py`

这里主要负责：

- 读取 `config/calibration.yaml`
- 像素 + 深度反投影
- 相机系到机器人基坐标系的变换

点击选点脚本和标定验证脚本都依赖这里。

### 7.3 `core/robot_controller.py`

这是当前推荐使用的 RealMan 真机控制封装，屏蔽了底层 `robotic_arm_package` 的不少细节。

新代码如果涉及真机控制，优先从这里走。

### 7.4 `api/state.py`

这里负责把配置、标定、IK、安全检查、仿真控制器这些资源组织成后端共享运行态。

如果你想理解 FastAPI 是怎么接上 arm 主链路的，这个文件很关键。

### 7.5 `sim/sapien_controller.py`

这里是当前 SAPIEN 最核心的控制器，负责：

- 从 YAML 加载机器人与相机配置
- 加载 URDF
- 建立仿真场景
- 读写仿真机械臂状态

## 8. debug 脚本如何分工

### 8.1 `debug_realman_controller_terminal.py`

适合做：

- 真机连接检查
- 状态读取
- home
- 单次 pose / joint 命令验证

### 8.2 `debug_pipeline_realman_runner.py`

适合做：

- 从 JSON 样例走完整 pipeline
- 验证 camera pose 输入到真机执行的整条链路
- 做 dry-run 或真机执行切换

### 8.3 `debug_click_to_realman.py`

适合做：

- 在 RealSense 画面中点击像素点
- 恢复 3D 点
- 生成 look-at 目标姿态并执行

### 8.4 `debug_sapien_scene.py` 与 `debug_sapien_camera.py`

适合做：

- 检查 SAPIEN 场景是否正常
- 验证仿真相机位姿与可视化是否一致

## 9. 前端

前端位于：

- `web/`

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

适合场景：

- 调试 API 联动
- 查看仿真状态和目标姿态
- 做基础可视化验证

## 10. 测试

当前测试主要集中在：

- `tests/test_transform.py`
- `tests/test_mapper.py`
- `tests/test_filter.py`
- `tests/test_ik.py`
- `tests/test_api.py`

从仓库根运行：

```bash
pytest arm_control/tests
```

需要注意：

- 这些测试更偏基础模块正确性
- 不等价于真机联调验证
- SAPIEN、RealSense 和真机链路仍更依赖本地集成验证

建议的验证顺序：

1. 单元测试通过
2. `python scripts/arm/run.py sim` 能跑
3. `python scripts/arm/run.py api` 能启动
4. 前端页面能正常显示
5. `debug-realman-terminal status` 能连真机
6. 真机单点动作成功
7. 点击执行链路再上真机

## 11. 环境与依赖

建议环境：

- `Python 3.10+`

arm 侧当前显式依赖见：

- `requirements.txt`

其中包括：

- `numpy`
- `PyYAML`
- `pytest`
- `sapien==3.0.0b0`
- `fastapi`
- `uvicorn`

如果你要接真机或相机联调，还需要本机额外具备：

- RealSense 驱动与 Python 绑定
- RealMan SDK 对应动态库可用
- 真实设备网络与驱动环境正常

## 12. 当前开发约定

当前 arm 侧有几条很重要的约定：

1. 顶层入口优先走 `scripts/arm/run.py`
2. 路径定位优先围绕仓库根来组织
3. 共享机器人模型优先放在顶层 `assets/robots/`
4. 共享 SDK 优先放在顶层 `third_party/robotic_arm_package/`
5. 新执行主逻辑优先收敛到 `TeleopPipeline`
6. 真机控制统一优先走 `robot_controller.py`
7. 新的调试脚本优先复用已有 pipeline / controller，而不是复制底层 SDK 调用

当前路径规则的真实来源：

- `repo_paths.py`

如果看到 `arm_control/paths.py`，可以把它理解成兼容层，而不是新的路径真源。

## 13. 常见问题

### 13.1 为什么推荐从顶层 wrapper 启动

因为仓库已经做过一轮目录整理，直接从子目录跑脚本更容易踩到：

- `cwd` 依赖
- 相对路径解析不一致
- `third_party` 导入问题

### 13.2 如果仿真跑不起来，先查哪里

优先检查：

- `config/robot.yaml`
- `config/calibration.yaml`
- `assets/robots/arms/`
- `sapien` 是否正确安装

### 13.3 如果真机链路跑不起来，先查哪里

优先检查：

- `config/robot.yaml` 中的 `realman.ip` 和 `realman.port`
- `third_party/robotic_arm_package/`
- 本机能否连到 RealMan
- 是否先用 `debug-realman-terminal` 做最小连通性验证

### 13.4 如果点击执行结果不合理，先查哪里

优先检查：

- `config/calibration.yaml`
- `config/workspace.yaml`
- `debug_click_to_realman.py` 当前姿态生成策略

## 14. 当前还不算终局的地方

当前 arm 子系统已经能稳定运行，但还有几处是“阶段性结构”：

- `examples/` 目录里混合了 JSON 样例和少量工具脚本
- `hand_eye_calibration/` 仍保留为历史标定目录
- 文档和调试入口还可以继续收口，但不影响当前继续开发

## 15. 安全说明

arm 侧最终会驱动真实机械臂，所以调试时建议始终遵守：

1. 先仿真、再真机
2. 先低速、再放开速度
3. 先单点、再轨迹
4. 优先用 dry-run 和最小调试入口验证链路
5. 修改 `robot.yaml`、`workspace.yaml`、`calibration.yaml` 后先做最小连通性验证

如果你只记住一句话，那就是：

从仓库根用 `python scripts/arm/run.py ...` 启动，并把 `config/robot.yaml`、`config/calibration.yaml`、`config/workspace.yaml` 当作 arm 侧第一优先级配置。
