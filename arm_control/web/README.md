# Web Pose Debugger

这个前端是一个第一版的位姿调试页面，用来验证：

- 相机坐标系输入 `pose_cam`
- 通过后端 `/api/transform_pose` 变换后的 `pose_base`
- 当前外参 `T_base_cam`
- 相机系轨迹 `trajectory_cam`
- 通过后端 `/api/transform_trajectory` 变换后的 `trajectory_base`
- 当前 SAPIEN 仿真中的 `actual_ee_pose`
- 当前下发给仿真的 `target_pose_base`

当前版本已经支持轮询 SAPIEN 当前状态，并显示 target/actual frame、actual ee trajectory，以及基于 URDF `visual mesh` 的机械臂渲染。

现在网页还会根据后端 `qpos` 渲染机械臂 mesh，帮助观察：

- 末端是否接近目标
- 姿态是否合理
- 机械臂构型是否异常
- IK 是否可能出现跳解

## 页面结构

页面分为两部分：

- 左侧：控制面板
- 右侧：3D 视图

## 当前页面功能

### 1. 后端数据读取

页面启动时会自动请求：

- `GET /api/calibration`
- `GET /api/sim/state`

用于读取：

- `T_base_cam`
- `R_offset_hand_to_tool`

页面中的 `Status` 面板会显示：

- `pose_cam.position`
- `pose_cam.euler`
- `pose_base.position`
- `pose_base.euler`
- `T_base_cam`
- `actual_ee_pose`
- `base_pose`
- `camera_pose`
- `position error`
- `orientation error`
- `qpos`
- `ik status`
- `playback status`
- `current index`
- `reject / fail count`

### 2. 模式切换

左侧顶部有两种模式：

- `Single Pose Mode`
- `Trajectory Mode`

#### Single Pose Mode

- 用于调试单个 `pose_cam`
- 会调用：
  - `POST /api/transform_pose`
  - `POST /api/sim/set_target_pose_base`

#### Trajectory Mode

- 用于调试一段 `trajectory_cam`
- 会调用：
  - `POST /api/transform_trajectory`
  - `POST /api/sim/set_target_trajectory_base`
  - `POST /api/sim/playback`

### 3. pose_cam 滑块调节

左侧有 6 个 slider：

- `tx`
- `ty`
- `tz`
- `roll`
- `pitch`
- `yaw`

它们对应发送给后端的：

```json
{
  "pose_cam": {
    "position": [tx, ty, tz],
    "euler": [roll, pitch, yaw]
  }
}
```

每次你拖动 slider，前端会自动延迟约 `120ms` 调用：

- `POST /api/transform_pose`

返回结果里的 `pose_base` 会刷新：

- 左侧 `Status`
- 右侧 `target frame`

### 4. Reset Pose 按钮

作用：

- 将当前 `pose_cam` 重置为全 0
- 即：
  - `tx = 0`
  - `ty = 0`
  - `tz = 0`
  - `roll = 0`
  - `pitch = 0`
  - `yaw = 0`

重置后会再次自动触发一次 `/api/transform_pose`。

### 5. Random Pose 按钮

作用：

- 调用：
  - `POST /api/sample_pose_cam`
- 获取一个后端生成的随机合法相机系 pose
- 将这个返回的 `pose_cam` 写回 slider
- slider 更新后，页面会自动再次调用 `/api/transform_pose`

如果这个按钮看起来“没反应”，通常优先检查：

- 后端是否真的运行
- `/api/sample_pose_cam` 是否可访问
- 浏览器开发者工具 Network 里是否有 404 / 500
- 左侧 `Status` 是否已经出现后端错误

### 6. Trajectory Mode 按钮和控件

#### Import Trajectory JSON

作用：

- 导入一段 JSON 轨迹
- 轨迹格式为：

```json
[
  {
    "position": [0.1, 0.0, 0.3],
    "euler": [0.0, 0.0, 0.0],
    "t": 0.0
  }
]
```

导入成功后会：

- 自动切换到 `Trajectory Mode`
- 调用：
  - `POST /api/transform_trajectory`

#### Append Current Pose

作用：

- 将当前 slider 对应的 `pose_cam` 追加到轨迹末尾
- 新点时间戳 `t` 以 `0.1s` 递增

#### Random Trajectory

作用：

- 在前端本地生成一段随机测试轨迹
- 自动切换到 `Trajectory Mode`
- 再调用：
  - `POST /api/transform_trajectory`
  - `POST /api/sim/set_target_trajectory_base`

#### Clear Trajectory

作用：

- 清空当前轨迹输入
- 清空当前轨迹变换结果

#### Play / Pause / Reset / Step

作用：

- `Play`
  - 让后端进入 `playing`
  - 按轨迹时间戳逐步推进当前目标索引
- `Pause`
  - 暂停在当前目标点
- `Reset`
  - 机械臂回到配置中的 `initial_qpos`
  - 清空 actual ee trajectory
  - 当前索引回到 0
- `Step`
  - 在暂停状态下只执行一个采样点

对应后端接口：

- `POST /api/sim/playback`

#### Show camera trajectory / Show base trajectory

作用：

- 控制右侧 3D 视图是否显示：
  - 相机系轨迹
  - base 系轨迹

#### Current sample index

作用：

- 选择当前高亮的轨迹采样点
- `Status` 面板显示的 `pose_cam` / `pose_base`
  会跟着当前采样点切换
- 右侧高亮点和 `target frame` 也会跟着切换

## 右侧 3D 视图说明

当前 3D 视图固定显示 4 个坐标系：

- `world frame`
- `base frame`
- `camera frame`
- `target frame`
- `actual ee frame`
- `robot`

在 `Trajectory Mode` 下还可以显示三条轨迹：

- `camera trajectory`
- `base trajectory`
- `actual ee trajectory`

另外还可以通过左侧开关控制是否显示：

- `show robot`
- `show world/base/camera`
- `show target frame`
- `show actual ee frame`
- `show trajectories`
- `show workspace box`

### 坐标轴颜色定义

- x 轴：红色
- y 轴：绿色
- z 轴：蓝色

### frame 的含义

- `world frame`
  - 固定为单位矩阵
- `base frame`
  - 来自后端 `/api/sim/state` 的 `base_pose`
- `camera frame`
  - 来自后端 `/api/sim/state` 的 `camera_pose`
- `target frame`
  - 来自后端 `/api/sim/state` 的 `target_pose_base`
  - 在前端会结合 `base_pose` 转成 world 中的目标 frame
- `actual ee frame`
  - 来自后端 `/api/sim/state` 的 `actual_ee_pose`
- `robot`
  - 由前端根据 `/api/sim/state` 返回的 `joint_names + qpos + robot_urdf_url` 更新
  - 优先使用 URDF 中的 STL mesh 做渲染

### 轨迹显示说明

- `camera trajectory`
  - 来自前端输入的 `trajectory_cam`
  - 为了在同一个 base/world 场景中显示，前端会使用 `T_base_cam`
    将这些相机系点放到 base/world 视图中
- `base trajectory`
  - 直接来自后端 `/api/transform_trajectory` 返回的 `trajectory_base`
- `actual ee trajectory`
  - 直接来自后端 `/api/sim/state`
  - 表示 SAPIEN 当前真实末端位姿历史

## Web 机械臂渲染

当前网页中的机械臂不是物理仿真本体，而是一个前端调试渲染层：

- 物理执行仍然在 SAPIEN
- Web 只负责读取状态并做几何显示

数据流：

`qpos from backend -> RobotView update`

实现位置：

- [RobotView.tsx](/nvme-ssd1/wlq/vnc_space/dex-retargeting/arm_control/web/src/components/RobotView.tsx)
- [urdf.ts](/nvme-ssd1/wlq/vnc_space/dex-retargeting/arm_control/web/src/utils/urdf.ts)

当前实现会：

- 从后端 `/api/sim/state` 读取 `robot_urdf_url`
- 在前端解析 URDF 关节链和 `visual mesh`
- 根据实时 `qpos` 更新各 joint 对应的 mesh 姿态

如果 URDF 或 STL 资源加载失败，前端会退回到更简化的调试显示。

## 第一版完整执行闭环

当前网页已经支持这条完整链路：

`trajectory_cam -> /api/transform_trajectory -> target_trajectory_base -> /api/sim/playback -> /api/sim/state -> Web render`

网页自身不做物理执行，只负责：

- 轨迹输入和导入
- 播放控制
- 机器人 mesh 渲染
- target / actual frame 对比
- target trajectory / actual ee trajectory 对比
- 调试状态面板显示

## 当前已知限制

### 1. `world frame` 和 `base frame` 可能看起来像一个

这是当前版本的预期行为，不是没画出来。

如果当前仿真里机器人 base 正好放在 world 原点，姿态也一致，那么：

- `world frame` 是单位矩阵
- `base frame` 也会非常接近单位矩阵

所以它们会完全重合。

### 2. 点击 frame “没有反应”

当前版本没有实现：

- frame 选中
- frame 高亮
- frame 切换
- frame 拖拽
- 点击交互

所以现在右侧只是静态可视化加轨道相机浏览，不支持点击切换。

也就是说：

- 能拖动画面看场景
- 但点击 frame 本身不会触发任何业务逻辑

### 3. 改变位姿或轨迹后右边变化不明显

这通常有几个原因：

- `world` 和 `base` 本来重合
- `target frame` 移动幅度不大
- 相机视角比较远
- 当前只画坐标轴，没有实体模型做参照

所以数值已经变了，但视觉上不一定很明显。

### 4. `tz` 区间和 `tx/ty` 不一样

这是当前前端 slider 的设计结果：

- `tx/ty`: `[-1.0, 1.0]`
- `tz`: `[-1.0, 1.5]`

这不是后端 API 自动推出来的，而是前端里手工设定的范围。

这么做的原因通常是：

- z 方向更容易需要更高的工作空间
- 想让调试时更容易看到上下抬升

但它目前还没有和 `workspace.yaml` 做严格联动。

### 5. `Status` 里出现 `Request failed: 404 Not Found`

这说明前端请求的某个 `/api/...` 路径没有被正确转发到 FastAPI。

最常见原因：

- 后端没有启动
- 前端不是通过 Vite dev server 启动的
- 没有 `/api` 代理
- `VITE_API_BASE_URL` 没设置对

当前前端默认假设：

- 页面通过 Vite 运行
- 并且 `/api/*` 通过 Vite proxy 转发到：
  - `vite.config.ts` 里配置的后端地址

如果你不是通过 `npm run dev` 跑，而是直接用别的方法打开静态页面，那么：

- `/api/calibration`
- `/api/transform_pose`
- `/api/sample_pose_cam`
- `/api/transform_trajectory`

都很可能变成访问“当前静态服务器”自身，从而得到 404。

## 验收时建议检查的接口

建议你用浏览器开发者工具 Network 或 curl 逐项确认：

### 1. 标定接口

```bash
curl http://127.0.0.1:8007/api/calibration
```

### 2. 变换接口

```bash
curl -X POST http://127.0.0.1:8007/api/transform_pose \
  -H "Content-Type: application/json" \
  -d '{"pose_cam":{"position":[0.1,0.0,0.3],"euler":[0.0,0.0,0.0]}}'
```

### 3. 随机位姿接口

```bash
curl -X POST http://127.0.0.1:8007/api/sample_pose_cam
```

### 4. 轨迹变换接口

```bash
curl -X POST http://127.0.0.1:8007/api/transform_trajectory \
  -H "Content-Type: application/json" \
  -d '{"trajectory_cam":[{"position":[0.1,0.0,0.3],"euler":[0.0,0.0,0.0],"t":0.0},{"position":[0.2,0.1,0.4],"euler":[0.1,0.0,0.2],"t":0.1}]}'
```

### 5. 仿真状态接口

```bash
curl http://127.0.0.1:8007/api/sim/state
```

### 6. 播放控制接口

```bash
curl -X POST http://127.0.0.1:8007/api/sim/playback \
  -H "Content-Type: application/json" \
  -d '{"action":"play"}'
```

### 7. 设置单点目标接口

```bash
curl -X POST http://127.0.0.1:8007/api/sim/set_target_pose_base \
  -H "Content-Type: application/json" \
  -d '{"target_pose_base":{"position":[0.2,0.0,0.4],"euler":[0.0,0.0,0.0]}}'
```

如果这些接口都正常，而网页还是 404，那么问题通常在前端请求地址或代理配置，不在后端逻辑。

## 当前前端真正已经实现的交互

- slider 改 `pose_cam`
- `Reset Pose`
- `Random Pose`
- `Trajectory Mode`
- 导入轨迹 JSON
- 追加轨迹点
- 清空轨迹
- 随机轨迹
- `Play / Pause / Reset / Step`
- 轨迹显示开关
- 当前采样点索引切换
- 左侧状态显示
- 右侧轨道相机拖动浏览
- 机器人 mesh 渲染
- SAPIEN 状态轮询和 target/actual 对比

## 当前前端还没有实现的交互

- 点击 frame 选中
- frame 开关显示/隐藏
- frame 标签
- frame 切换聚焦
- world/base 分离显示
- 点击 frame 直接编辑目标
- websocket 推送
- 真实机械臂接口

## 建议的排查顺序

1. 先确认 FastAPI 是否启动。
2. 单独测试 `/api/calibration`、`/api/transform_pose`、`/api/sample_pose_cam`、`/api/transform_trajectory`。
3. 再确认网页是否通过正确的 dev server 或正确的 API 基地址访问后端。
4. 最后再看 3D 视图的显示是否符合当前版本设计。

