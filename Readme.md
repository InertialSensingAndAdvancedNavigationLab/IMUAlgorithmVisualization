# AHRS 项目坐标系约定 (v1.0)

本文档是项目中所有坐标系定义的“单一事实来源”。所有模块的实现都必须严格遵守此约定，以确保数据的一致性和算法的正确性。

## 1. 核心原则

- **右手坐标系**: 本项目中的所有坐标系均为右手坐标系。
- **内部计算标准**: 所有AHRS算法的内部状态演化和计算，**必须**在本文档定义的 **`Internal Compute Frame (FRD)`** 中进行。
- **数据转换**: 所有输入数据（如EuRoC）必须在加载时被转换到内部计算系。所有输出数据（如TUM轨迹）必须在保存时从内部计算系转换到目标格式约定的坐标系。

---

## 2. 坐标系定义

### 2.1 世界坐标系 (World Frame, W)

- **类型**: 固定的惯性参考系。
- **约定**: 我们采用 **East-North-Up (ENU)** 约定，这与许多地理和导航应用一致。
  - **`+X`**: 指向东方 (East)
  - **`+Y`**: 指向北方 (North)
  - **`+Z`**: 指向天顶 (Up)

### 2.2 EuRoC IMU 原始机体系 (IMU Original Frame, `IMU_Orig`)

- **来源**: 这是EuRoC数据集IMU传感器(ADIS16448)的原始坐标系。
- **约定**: 根据社区（如 Kalibr 工具箱）的广泛共识，该坐标系定义如下：
  - **`+X`**: 指向传感器右侧 (Right)
  - **`+Y`**: 指向传感器上方 (Up)
  - **`+Z`**: 指向传感器前方 (Forward)

### 2.3 内部计算机体系 (Internal Compute Frame, `Internal`)

- **目的**: 这是本项目所有算法进行内部姿态积分、滤波和状态更新所使用的**唯一**标准机体坐标系。
- **约定**: 我们采用航空航天和机器人领域常见的 **Forward-Right-Down (FRD)** 约定。
  - **`+X`**: 指向机体前方 (Forward)
  - **`+Y`**: 指向机体右侧 (Right)
  - **`+Z`**: 指向机体下方 (Down)

### 2.4 TUM / 可视化坐标系 (TUM/Visualization Frame, `TUM_Viz`)

- **目的**: 这是所有输出的 `.txt` 轨迹文件所采用的坐标系，以确保与TUM数据集格式的社区惯例（如EVO评估工具）和您的MATLAB脚本兼容。
- **约定**: 我们采用计算机视觉和相机领域常见的约定。
  - **`+X`**: 指向右侧 (Right)
  - **`+Y`**: 指向下方 (Down)
  - **`+Z`**: 指向前方 (Forward)

---

## 3. 核心转换关系

### 3.1 输入转换: `IMU_Orig` -> `Internal`

在 `EurocDataLoader` 中，我们必须将从CSV文件读取的原始IMU测量值（角速度 `ω` 和加速度 `a`）从 `IMU_Orig` 系转换到 `Internal` 系。

- **变换关系**:
  - `Internal.X (Fwd)`  = `IMU_Orig.Z (Fwd)`
  - `Internal.Y (Right)` = `IMU_Orig.X (Right)`
  - `Internal.Z (Down)`  = `-IMU_Orig.Y (Up)`

- **实现 (C++)**:
  对于一个在 `IMU_Orig` 系下的向量 `v_orig = [vx, vy, vz]`，其在 `Internal` 系下的表示 `v_internal = [vx', vy', vz']` 为：
  ```cpp
  vx' = vz;
  vy' = vx;
  vz' = -vy;