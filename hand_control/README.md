# dex_controller

`hand_control/` 是当前仓库里的 hand 子系统，负责把相机采集到的手部动作转换成灵巧手控制指令。

如果只用一句话概括这部分代码，它做的是：

`视频/相机输入 -> 手部检测 -> retargeting -> 6 路关节映射 -> 灵巧手控制`

这份 README 按“hand 子系统交接文档”来写，重点帮助接手的人快速回答这些问题：

- hand 侧的真实入口在哪里
- `controller` 和 `stream` 两条链路分别做什么
- 第一次运行前该检查哪几个配置
- 如果要改 hand 主流程，应该从哪些文件开始看

## 1. 当前边界

当前 hand 子系统的边界是清楚的：

- `hand_control/` 是 hand 侧真实项目根
- 推荐统一从仓库根使用顶层入口 `python scripts/hand/run.py ...`
- 机器人模型会复用顶层 `assets/robots/`
- 机械臂 SDK 会复用顶层 `third_party/robotic_arm_package/`
- hand 私有配置仍保留在 `hand_control/configs/`

也就是说，当前 hand 侧不是一个完全独立仓库，而是顶层 monorepo 里的一个平级子系统。

## 2. 当前能做什么

当前 hand 侧主要支持两条运行模式：

1. `local`
   检测、retargeting、映射和控制在一个进程里完成。
2. `http`
   先把手部关节数据作为 HTTP 流输出，再由控制端读取并执行。

你可以把当前两个主入口理解成：

- `teleop.py`
  负责“看手”和“算手”
- `brainco_controller.py`
  负责“发控制”

## 3. 目录结构

下面只列出当前最重要的目录和文件：

```text
hand_control/
  configs/
    arm_connection.yaml
    settings.yaml
    legacy_settings_unused.yaml
    revo2/
      revo2_hand_left_dexpilot.yml
      revo2_hand_right_dexpilot.yml
  example/
    revo2_retargeting/
      brainco_controller.py
      teleop.py
      single_hand_detector.py
      sim_hand_viewer.py
  README.md
```

这些文件的作用分别是：

- `configs/arm_connection.yaml`
  hand 当前实际使用的机械臂连接配置
- `configs/settings.yaml`
  兼容清单，当前不是主配置入口
- `configs/legacy_settings_unused.yaml`
  旧设置归档，当前运行链路不依赖
- `configs/revo2/*.yml`
  hand retargeting 配置
- `example/revo2_retargeting/teleop.py`
  手部检测、retargeting 和数据流主入口
- `example/revo2_retargeting/brainco_controller.py`
  灵巧手控制入口
- `example/revo2_retargeting/single_hand_detector.py`
  单手检测模块

虽然目录里还叫 `example/`，但这里面的核心 Python 文件现在更接近“运行代码”，不是单纯示例。

## 4. 推荐启动方式

当前统一约定：

- 始终从仓库根目录启动
- 优先使用顶层入口 [scripts/hand/run.py](/d:/video_dex_infra/video-learning-infra/scripts/hand/run.py)

### 4.1 本地一体化模式

```bash
python scripts/hand/run.py controller \
  --input_mode local \
  --source realsense \
  --hand_type right \
  --show_window
```

适合场景：

- 单机 bring-up
- 验证检测到控制的全链路
- 初次接入设备时做最小闭环验证

### 4.2 只打印关节值

```bash
python scripts/hand/run.py controller \
  --input_mode local \
  --debug_joints \
  --source realsense
```

适合场景：

- 不接灵巧手，先看 retargeting 输出
- 判断输入流和映射结果是否合理

### 4.3 HTTP 数据流模式

先启动数据流端：

```bash
python scripts/hand/run.py stream --mode http --source realsense --show_window
```

再启动控制端：

```bash
python scripts/hand/run.py controller \
  --input_mode http \
  --stream_url http://127.0.0.1:50001/hand_data
```

适合场景：

- 检测侧和控制侧拆开调试
- 后续接 system 层或远程链路

## 5. 第一次运行前先检查什么

### 5.1 先检查连接配置

优先看：

- `configs/arm_connection.yaml`

最关键的是：

- `realman.left.ip`
- `realman.left.side`

当前 `brainco_controller.py` 只真正依赖左臂连接信息，所以这份配置是最关键的。

### 5.2 再检查 retargeting 配置

优先看：

- `configs/revo2/revo2_hand_left_dexpilot.yml`
- `configs/revo2/revo2_hand_right_dexpilot.yml`

这些文件决定：

- 手型
- target link 映射
- retargeting 行为

### 5.3 再检查输入设备

优先确认：

- `pyrealsense2` 是否可用
- 相机设备是否能正常打开
- 如果用本地摄像头，`--source 0` / `--source 1` 是否能读到画面

## 6. 输入源说明

`--source` 当前支持：

- `realsense` 或 `rs`
- `0`、`1` 等本地摄像头索引
- 本地视频文件路径
- MJPEG URL

对应逻辑主要在：

- `example/revo2_retargeting/teleop.py`

## 7. 主链路应该从哪里读

如果你要理解 hand 子系统，建议按下面顺序看代码：

1. `example/revo2_retargeting/teleop.py`
   先理解输入源、检测、retargeting 和数据输出
2. `example/revo2_retargeting/brainco_controller.py`
   再理解 hand 控制与 HTTP / local 两种模式
3. `example/revo2_retargeting/single_hand_detector.py`
   最后看检测模块实现细节

### 7.1 `teleop.py`

这里主要负责：

- 打开输入源
- 跑单手检测
- 跑 retargeting
- 把结果映射成 6 路 0-100 的关节值
- 在 `print` 或 `http` 模式下输出

### 7.2 `brainco_controller.py`

这里主要负责：

- 读取 `arm_connection.yaml`
- 初始化 RealMan + 灵巧手控制链路
- 在 `local` 模式下直接消费 `teleop.py` 的映射结果
- 在 `http` 模式下读取 `/hand_data` 流

### 7.3 `single_hand_detector.py`

这里主要负责：

- 单手检测
- 关键点提取
- 可视化绘制

## 8. 常用参数

### 8.1 `brainco_controller.py`

最常用的参数有：

- `--arm-config`
  默认 `configs/arm_connection.yaml`
- `--input_mode`
  可选 `local` / `http`
- `--stream_url`
  仅 `http` 模式使用
- `--source`
  仅 `local` 模式使用
- `--hand_type`
  `left` 或 `right`
- `--config_path`
  显式指定 retargeting 配置路径
- `--show_window`
  显示可视化窗口
- `--debug_joints`
  打印映射后的 6 路关节值
- `--control_hz`
  控制频率
- `--deadband`
  发送死区阈值

### 8.2 `teleop.py`

最常用的参数有：

- `--source`
  输入源
- `--mode`
  `print` / `http`
- `--host`
  HTTP 服务绑定地址
- `--port`
  HTTP 服务端口
- `--hand_type`
  左手 / 右手
- `--scaling_factor`
  retargeting 缩放系数
- `--config_path`
  显式指定 retargeting 配置

## 9. 环境与依赖

建议环境：

- `Python 3.10+`

当前 hand 侧运行通常需要这些 Python 包：

- `opencv-python`
- `numpy`
- `requests`
- `fastapi`
- `uvicorn`
- `pyyaml`
- `pyrealsense2`
- `dex_retargeting` 相关依赖

如果你使用 RealSense，`pyrealsense2` 是必需的。

## 10. 当前开发约定

当前 hand 侧有几条很重要的约定：

1. 顶层入口优先走 `scripts/hand/run.py`
2. `configs/arm_connection.yaml` 是当前真实连接配置
3. `configs/settings.yaml` 只是兼容清单，不要再往里面加新主配置
4. 手部模型优先从顶层 `assets/robots/` 读取
5. SDK 优先从顶层 `third_party/robotic_arm_package/` 读取

## 11. 常见问题

### 11.1 为什么 IDE 提示 `robotic_arm_package` 无法解析

这通常是静态分析路径问题，不代表运行时真的导不进来。

当前工作区已经在 `.vscode/settings.json` 中补了：

- 仓库根
- `third_party`
- `hand_control/example/revo2_retargeting`

如果 VS Code 里红线没有立刻消失，一般重载窗口或重启 Python language server 即可。

### 11.2 为什么我不建议直接运行子目录脚本

因为 hand 侧现在会复用顶层 `assets/` 和 `third_party/`，直接从子目录随手运行更容易踩到：

- `cwd` 依赖
- 相对路径定位问题
- IDE 和运行时路径不一致

### 11.3 如果本地模式跑不起来，先查哪里

优先检查：

- `configs/arm_connection.yaml`
- `pyrealsense2`
- `dex_retargeting`
- `scripts/hand/run.py` 是否从仓库根启动

### 11.4 如果 HTTP 模式跑不起来，先查哪里

优先检查：

- `teleop.py` 的 `--mode http` 是否已经启动
- `--stream_url` 是否正确
- 端口 `50001` 是否被占用

## 12. 当前还不算终局的地方

当前 hand 子系统已经能稳定运行，但还有几处是“阶段性结构”：

- `example/` 目录名还没有完全和真实语义对齐
- `configs/settings.yaml` 还保留为兼容文件
- 还没有单独整理出一份 hand 侧完整 requirements 文件

这些不影响当前继续开发，但说明后面仍会有小步整理。

## 13. 安全说明

hand 侧链路最终会驱动真实设备，所以调试时建议始终遵守：

1. 先只看 `--debug_joints`
2. 再接入真实控制
3. 先低频、小幅度验证
4. 修改连接配置后先做最小闭环测试

如果你只记住一句话，那就是：

从仓库根用 `python scripts/hand/run.py ...` 启动，并把 `configs/arm_connection.yaml` 当作 hand 侧第一优先级配置。
