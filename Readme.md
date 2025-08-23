<div align="right">
  <a href="./Readme.zh.md">中文</a>
</div>

# IMU Algorithm Visualization

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

A ROS package designed to visualize various IMU (Inertial Measurement Unit) properties, errors, and algorithm outputs in Rviz. This tool helps in the analysis and debugging of attitude estimation algorithms and IMU sensor data.

## 🚀 Features

-   Visualize ground truth and estimated poses.
-   Display IMU measurement vectors (gravity, acceleration).
-   Quantify and visualize attitude and measurement errors.
-   Provides nodes for gravity calibration and data simulation.
-   Highly configurable through ROS parameters and launch files.

## 📦 Installation & Usage

1.  **Prerequisites**:
    *   ROS (tested on Melodic & Noetic)
    *   A catkin workspace

2.  **Build**:
    ```bash
    # Navigate to your catkin workspace's source directory
    cd /path/to/your/catkin_ws/src

    # Clone the repository
    git clone https://github.com/InertialSensingAndAdvancedNavigationLab/IMUAlgorithmVisualization.git

    # Build the package
    cd ..
    catkin_make
    ```

3.  **Run**:
    The package comes with several launch files for different scenarios. The main launch file is:
    ```bash
    roslaunch imu_algorithm_visualization run_visualization.launch
    ```
    You can also run individual nodes or use other launch files in the `launch/` directory for specific analyses.

## 📊 Nodes

Here is a detailed list of the nodes provided in this package.

---

### 1. Ground Truth Visualizer

| Item | Description |
| :--- | :--- |
| **Node Name** | `ground_truth_visualizer_node` |
| **Description** | Visualizes the ground truth pose as a detailed drone model. It allows locking the position or attitude for specific test scenarios and corrects the coordinate system from URF to FLU. |
| **Subscriptions** | `~ground_truth_pose_topic` (`/ground_truth/pose`, `geometry_msgs/PoseStamped`) |
| **Publications** | `visualization/ground_truth_body` (`visualization_msgs/Marker`) (Latched) |
| **Services** | `set_attitude`, `set_location`, `reset_pose` |
| **TF** | Publishes `world` -> `ground_truth_body` transform. |

---

### 2. IMU Visualizer

| Item | Description |
| :--- | :--- |
| **Node Name** | `imu_visualizer_node` |
| **Description** | Visualizes raw IMU data in the context of the ground truth. It shows the drone's trajectory, model, true attitude vector, and the uncertainty of the accelerometer measurement. |
| **Subscriptions** | `~imu_topic` (`/imu0`, `sensor_msgs/Imu`) <br> `~ground_truth_pose_topic` (`/ground_truth/pose`, `geometry_msgs/PoseStamped`) |
| **Publications** | `visualization/trajectory` (`visualization_msgs/Marker`) <br> `visualization/drone_model` (`visualization_msgs/Marker`) <br> `visualization/true_attitude` (`visualization_msgs/Marker`) <br> `visualization/accel_uncertainty` (`visualization_msgs/Marker`) |

---

### 3. AHRS Pose Visualizer

| Item | Description |
| :--- | :--- |
| **Node Name** | `ahrs_pose_visualizer` |
| **Description** | Visualizes the output of an AHRS algorithm. It displays a blue drone model whose orientation is controlled by the AHRS data but whose position follows the ground truth. |
| **Subscriptions** | `~pose_topic` (`/ahrs/pose`, `geometry_msgs/PoseStamped`) <br> `/ground_truth/pose` (`geometry_msgs/PoseStamped`) |
| **Publications** | `~model_topic` (`/visualization/ahrs_drone`, `visualization_msgs/Marker`) |
| **TF** | Publishes `world` -> `ahrs_body` transform. |

---

### 4. Gravity Measurement Visualizer

| Item | Description |
| :--- | :--- |
| **Node Name** | `gravity_measurement_visualizer_node` |
| **Description** | Visualizes the measured gravity vector from the IMU and provides a service to calibrate it. The calibrated vector is published for other nodes to use. |
| **Subscriptions** | `~imu_topic` (`/imu0`, `sensor_msgs/Imu`) <br> `~ground_truth_pose_topic` (`/ground_truth/pose`, `geometry_msgs/PoseStamped`) |
| **Publications** | `visualization/measured_gravity` (`visualization_msgs/Marker`) <br> `/calibration/gravity_vector` (`geometry_msgs/Vector3Stamped`) (Latched) |
| **Services** | `/calibrate_gravity` (`std_srvs/Trigger`) |

---

### 5. Attitude Error Visualizer

| Item | Description |
| :--- | :--- |
| **Node Name** | `attitude_error_visualizer` |
| **Description** | Visualizes the error between the true gravity vector and the IMU-measured gravity vector in the body frame. |
| **Subscriptions** | `/imu0` (`sensor_msgs/Imu`) <br> `/ground_truth/pose` (`geometry_msgs/PoseStamped`) <br> `/calibration/gravity_vector` (`geometry_msgs/Vector3Stamped`) |
| **Publications** | `visualization/attitude_error` (`visualization_msgs/MarkerArray`) |
| **TF** | Listens for the transform from the IMU frame to the `ground_truth_body` frame. |

---

### 6. IMU Error Visualizer

| Item | Description |
| :--- | :--- |
| **Node Name** | `imu_error_visualizer` |
| **Description** | Calculates and visualizes the total IMU measurement error by comparing the actual IMU reading to a calculated ideal reading based on ground truth motion and gravity. Uses a time synchronizer for high precision. |
| **Subscriptions (Sync)** | `/ground_truth/pose` (`geometry_msgs/PoseStamped`) <br> `/ground_truth/accel` (`geometry_msgs/AccelStamped`) <br> `/imu0` (`sensor_msgs/Imu`) |
| **Subscriptions (Normal)** | `/calibration/gravity_vector` (`geometry_msgs/Vector3Stamped`) |
| **Publications** | `visualization/imu_error` (`visualization_msgs/Marker`) |

---

### 7. Dummy IMU Publisher

| Item | Description |
| :--- | :--- |
| **Node Name** | `dummy_imu_publisher` |
| **Description** | A simple node for testing. It publishes simulated IMU data at 50Hz without requiring any input. |
| **Publications** | `/imu/data` (`sensor_msgs/Imu`) |

## 📜 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.
