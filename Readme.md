# AHRS 项目坐标系约定 (v2.0)

本文档是项目中所有坐标系定义的“单一事实来源”（Single Source of Truth）。所有模块的实现都**必须**严格遵守此约定，以确保数据的一-致性和算法的正确性。一个清晰、一致的坐标系约定是避免耗费大量时间调试数据问题的关键。

## 1. 核心原则

- **右手坐标系**: 本项目中的所有坐标系均为右手坐标系。
- **内部计算标准**: 所有AHRS算法的内部状态演化和计算，**必须**在本文档定义的 **`FRD (Forward-Right-Down)`** 坐标系中进行。
- **数据转换**: 所有输入数据（如EuRoC）必须在加载时被转换到内部计算系。所有输出数据（如TUM轨迹）必须在保存时从内部计算系转换到目标格式约定的坐标系。

---

## 2. 坐标系约定 (Coordinate System Conventions)

为了明确项目中的选择并方便与外部工具和数据集交互，下表列出了所有相关的坐标系定义。**本项目中使用的坐标系已加粗显示**，并在“项目角色”列中说明了其具体用途。

### 2.1 世界坐标系 (World / Inertial Frames)

世界坐标系是一个固定的参考系，通常用于描述机体（或传感器）的全局位置和姿态。

| 名称 (Name) | 项目角色 (Project Role) | 轴定义 (Axes Definition)                                                              | 主要应用领域 (Primary Use Cases)                                     |
| :---------- | :---------------------- | :------------------------------------------------------------------------------------ | :------------------------------------------------------------------- |
| **ENU**     | **世界坐标系 (World, `W`)** | `+X` -> 东方 (East)<br>`+Y` -> 北方 (North)<br>`+Z` -> 天顶 (Up)                         | GPS, 地图, ROS 2 (REP-103), 大多数现代机器人应用                     |
| NED         | -                       | `+X` -> 北方 (North)<br>`+Y` -> 东方 (East)<br>`+Z` -> 地心 (Down)                       | 航空航天, 航海, 传统惯性导航 (INS), ArduPilot, PX4                   |

### 2.2 机体与传感器坐标系 (Body and Sensor Frames)

机体坐标系是固连于运动物体（如无人机）的逻辑参考系，而传感器坐标系则固连于传感器芯片本身。它们之间可能存在一个固定的旋转关系，即**外参 (Extrinsics)**。下表列出了常见的定义，并给出了从该坐标系转换到本项目标准机体坐标系（FRD）的旋转矩阵，这个矩阵本质上就是传感器到机体的外参。

| 名称 (Name) | 项目角色 (Project Role) | 轴定义 (Axes Definition)                                                              | 常用领域/别名 (Use Cases / Aliases) | 转换到FRD的旋转矩阵 (R_frd_from_this) |
| :---------- | :---------------------- | :------------------------------------------------------------------------------------ | :---------------------------------- |:----------- |
| **FRD**     | **内部计算系 (`Internal`)** | `+X` -> 前 (Forward)<br>`+Y` -> 右 (Right)<br>`+Z` -> 下 (Down)                          | 航空标准 (Aerospace), 飞控 (PX4, ArduPilot) | `[1, 0, 0]`<br>`[0, 1, 0]`<br>`[0, 0, 1]` |
| FLU         | -                       | `+X` -> 前 (Forward)<br>`+Y` -> 左 (Left)<br>`+Z` -> 上 (Up)                             | ROS `base_link`, 汽车 (Automotive) | `[1, 0, 0]`<br>`[0,-1, 0]`<br>`[0, 0,-1]` |
| **RDF**     | **输出/可视化系 (`TUM_Viz`)** | `+X` -> 右 (Right)<br>`+Y` -> 下 (Down)<br>`+Z` -> 前 (Forward)                          | 计算机视觉 (CV), 相机 (Camera), OpenCV | `[0, 1, 0]`<br>`[0, 0, 1]`<br>`[1, 0, 0]` |
| **RUF**     | **EuRoC IMU 输入系 (`IMU_Orig`)** | `+X` -> 右 (Right)<br>`+Y` -> 上 (Up)<br>`+Z` -> 前 (Forward)                            | Kalibr 工具箱, EuRoC IMU | `[ 0,  0, 1]`<br>`[-1,  0, 0]`<br>`[ 0, -1, 0]` |
| NWU         | -                       | `+X` -> 北 (North)<br>`+Y` -> 西 (West)<br>`+Z` -> 上 (Up)                               | 简单AHRS/互补滤波 | `[1, 0, 0]`<br>`[0,-1, 0]`<br>`[0, 0,-1]` |
| LFD         | -                       | `+X` -> 左 (Left)<br>`+Y` -> 前 (Forward)<br>`+Z` -> 下 (Down)                           | Intel Realsense D400 系列 IMU | `[0, 1, 0]`<br>`[-1,0, 0]`<br>`[0, 0, 1]` |
| RUB         | -                       | `+X` -> 右 (Right)<br>`+Y` -> 上 (Up)<br>`+Z` -> 后 (Back)                               | 移动设备 (Mobile Devices) | `[0, 0,-1]`<br>`[1, 0, 0]`<br>`[0,-1, 0]` |

#### 关于算法的隐式坐标系假设

许多姿态估计算法，特别是那些代码中直接使用重力向量进行修正的算法（如基础的互补滤波或EKF），会包含一个**隐式的坐标系假设**。

*   如果代码中将参考重力向量硬编码为 `g = [0, 0, 9.81]` (或 `[0, 0, 1]`)，则它**隐式地假设**机体的 `+Z` 轴在水平静止时指天（Up）。这对应于上表中的 `FLU`, `RUF`, `NWU`, `RUB` 等坐标系。
*   如果代码中将参考重力向量硬编码为 `g = [0, 0, -9.81]` (或 `[0, 0, -1]`)，则它**隐式地假设**机体的 `+Z` 轴在水平静止时指地（Down）。这对应于上表中的 `FRD`, `RDF`, `LFD` 等坐标系。

在使用任何第三方算法或参考实现时，**必须**首先检查其对重力向量的假设，以确定其内部工作坐标系，从而进行正确的数据输入转换。

---

---

## 3. 本项目坐标系约定

基于上述标准，本项目做出以下明确约定：

### 3.1 世界坐标系 (World Frame, `W`): ENU

- **约定**: `ENU (East-North-Up)`
- **理由**: 与 ROS 2、GPS 和许多现代机器人工具链兼容，便于可视化和集成。

### 3.2 内部逻辑机体坐标系 (Internal Compute Frame, `Internal`): FRD

- **约定**: `FRD (Forward-Right-Down)`
- **目的**: 这是所有 AHRS 算法（积分、滤波）进行状态更新的**唯一**标准。
- **理由**: 这是航空航天和无人机领域的标准，与许多高级 IMU 传感器和飞控固件（如 PX4/ArduPilot）的约定一致，便于理论和实践的结合。
**此外，FRD坐标系的姿态角（Roll, Pitch, Yaw）定义与航空标准的横滚、俯仰、偏航角直接对应，避免了符号混淆。**

### 3.3 输入数据坐标系 (Input Data Frames)

#### 3.3.1 EuRoC IMU 传感器物理坐标系 (`IMU_Orig`): RUF

- **约定**: `RUF (Right-Up-Forward)`
- **来源**: EuRoC 数据集 IMU 传感器 (ADIS16448) 的原始坐标系，由 Kalibr 等工具确定。
- **处理**: 在 `EurocDataLoader` 中，所有原始 IMU 测量值（角速度和加速度）都**必须**从 `IMU_Orig` (RUF) 转换到 `Internal` (FRD)。

### 3.4 输出/可视化坐标系 (Output/Visualization Frame, `TUM_Viz`): RDF

- **约定**: `RDF (Right-Down-Forward)`
- **目的**: 用于所有输出的轨迹文件（`.txt`），以兼容 **TUM 轨迹格式**和 EVO 等评估工具。
- **理由**: 这是计算机视觉领域的标准，便于与相机数据和相关工具集成。

### 3.5 数据集特定约定 (EuRoC)

- **世界坐标系 (Ground Truth World Frame):** EuRoC的真值数据（来自Vicon动捕系统）是在一个 **ENU** 世界坐标系下给出的。这与我们项目选择的世界系一致。
- **IMU传感器坐标系 (IMU Sensor Frame):** EuRoC数据集使用的IMU (ADIS16448) 其自身的坐标系为 **RUF (Right-Up-Forward)**。这就是我们代码中 `IMU_Orig` 的来源。
- **相机坐标系 (Camera Frame):** EuRoC的相机坐标系为 **RDF (Right-Down-Forward)**，这是计算机视觉领域的标准。
- **传感器到机体的转换 (Sensor-to-Body Transformation):** 这是您困惑的核心。我们的 `Internal` (FRD) 坐标系是为算法计算定义的**逻辑机体坐标系**。而IMU传感器以 `RUF` 方式安装在该逻辑机体上。因此，从传感器读出的原始数据必须经过一个固定的旋转（即您提到的“标定矩阵”或外参）才能转换到我们统一的 `FRD` 计算坐标系中。这个转换在 `EurocDataLoader` 中完成，其旋转矩阵正是上表中的 `R_frd_from_ruf`。

---

## 4. 核心坐标变换

### 4.1 输入转换: `IMU_Orig` (RUF) -> `Internal` (FRD)

此变换在加载 EuRoC 数据时使用。

- **变换关系**:
  - `Internal.X (Fwd)`  = `IMU_Orig.Z (Fwd)`
  - `Internal.Y (Right)` = `IMU_Orig.X (Right)`
  - `Internal.Z (Down)`  = `-IMU_Orig.Y (Up)`

- **旋转矩阵 `C_internal_from_orig`**:
  ```
  [ F ]   [ 0  0  1 ] [ R ]
  [ R ] = [ 1  0  0 ] [ U ]
  [ D ]   [ 0 -1  0 ] [ F ]_orig
  ```

- **C++ 实现**: 对于一个在 `IMU_Orig` 系下的向量 `v_orig = [vx, vy, vz]`，其在 `Internal` 系下的表示 `v_internal = [vx', vy', vz']` 为：
  ```cpp
  vx' =  vz;
  vy' =  vx;
  vz' = -vy;
  ```

### 4.2 输出转换: `Internal` (FRD) -> `TUM_Viz` (RDF)

此变换在保存 TUM 格式的轨迹前使用。

- **背景**: 算法输出的姿态是从 `Internal` (FRD) 系到 `World` (ENU) 系的旋转，记为 `q_world_from_internal`。TUM 格式要求提供 `TUM_Viz` (RDF) 系相对于 `World` 系的姿态，记为 `q_world_from_viz`。
- **推导**: `q_world_from_viz = q_world_from_internal * q_internal_from_viz`，其中 `q_internal_from_viz` 是从 `TUM_Viz` 到 `Internal` 的旋转。

- **变换关系**:
  - `TUM_Viz.X (Right)` = `Internal.Y (Right)`
  - `TUM_Viz.Y (Down)`  = `Internal.Z (Down)`
  - `TUM_Viz.Z (Fwd)`   = `Internal.X (Fwd)`

- **旋转矩阵 `C_viz_from_internal`**:
  ```
  [ R ]   [ 0  1  0 ] [ F ]
  [ D ] = [ 0  0  1 ] [ R ]
  [ F ]   [ 1  0  0 ] [ D ]_internal
  ```

- **C++ 实现**: 在 `main.cpp` 中，我们预先计算 `q_viz_from_internal`，然后通过 `q_world_from_internal * q_viz_from_internal.inverse()` 求得 `q_world_from_viz`。

---

## 5. SLAM 和其他应用的说明

- **Visual SLAM/VIO**: 在视觉 SLAM 或视觉惯性里程计中，相机的坐标系几乎普遍采用 `RDF (Right-Down-Forward)`。然而，IMU 的坐标系各不相同。因此，任何 VIO 系统中的一个关键步骤都是标定相机和 IMU 之间的外参（Extrinsic Transformation），即 `T_cam_imu`。

- **ROS**: ROS 框架有其自身的坐标系约定 (如 REP-103)。地面机器人的 `base_link` 坐标系通常是 `FLU (Forward-Left-Up)`。对于相机，会使用一个 "optical" 坐标系，即 `RDF`。世界坐标系 (`map` 或 `odom`) 通常是 `ENU`。在与 ROS 集成时，必须在接口边界处进行数据转换，以符合 ROS 的标准。
