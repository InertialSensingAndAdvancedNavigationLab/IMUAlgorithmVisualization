# Coordinate Systems and Transformations in IMUAlgorithmVisualization

This document outlines the coordinate systems used within the `IMUAlgorithmVisualization` package and the transformations applied between them. Understanding these conventions is crucial for correct interpretation of sensor data, ground truth, and algorithm outputs.

## 1. World Frame (`world`)

*   **Description:** The global, inertial reference frame. All ground truth poses and calibrated gravity vectors are expressed in this frame.
*   **Convention:** Right-Handed (RHS), typically ENU (East-North-Up) or NED (North-East-Down) depending on the dataset. For EuRoC, it's generally NED.
    *   **X-axis:** North
    *   **Y-axis:** East
    *   **Z-axis:** Down

## 2. Ground Truth Body Frame (`ground_truth_body`)

*   **Description:** The body frame of the ground truth (e.g., the drone's center of mass). Its origin is at the vehicle's center, and its orientation changes with the vehicle's attitude.
*   **Convention:** FLU (Forward-Left-Up) - Right-Handed (RHS)
    *   **X-axis:** Forward (along the vehicle's main axis of motion)
    *   **Y-axis:** Left
    *   **Z-axis:** Up
*   **Transformation from World:** The ground truth pose (`/ground_truth/pose` topic) provides the transformation from `world` to `ground_truth_body`.

## 3. IMU Frame (`imu0`)

*   **Description:** The sensor frame of the IMU. This is the frame in which raw IMU measurements (angular velocity, linear acceleration) are reported.
*   **Convention:** Typically FRD (Forward-Right-Down) or FLU (Forward-Left-Up) depending on the sensor mounting. For EuRoC, the raw IMU data is often in a specific sensor-defined frame.
*   **Transformation to Body:** A static transform is published from `ground_truth_body` to `imu0`. In `full_analysis.launch`, this is currently an identity transform, implying `imu0` is coincident with `ground_truth_body` for the EuRoC dataset. If the physical mounting of the IMU changes, this transform needs to be updated.

## 4. URF (Up-Right-Forward) Convention

*   **Description:** A common convention for some datasets or internal representations where:
    *   **X-axis:** Up
    *   **Y-axis:** Right
    *   **Z-axis:** Forward
*   **Transformation to FLU:** The `urf_to_flu_quat` parameter (defined in `config/transforms.yaml`) is used to convert orientations from URF to FLU. This is applied to the ground truth pose data to ensure consistency with the `ground_truth_body` FLU convention.
    *   `urf_to_flu_quat` (x, y, z, w): (0.70710678, 0.0, 0.70710678, 0.0) - This corresponds to a 90-degree rotation around the Y-axis, followed by a 90-degree rotation around the new X-axis.

## Key Transformations Summary:

*   **`world` -> `ground_truth_body`:** Provided by ground truth pose messages, with `urf_to_flu_quat` applied for orientation correction.
*   **`ground_truth_body` -> `imu0`:** Defined by a static TF publisher (currently identity in `full_analysis.launch`).

## Future Considerations:

*   **Configurable IMU-to-Body Transform:** If the IMU is not always mounted coincident with the body frame, the `body_to_imu0_broadcaster` in `full_analysis.launch` should be made configurable, potentially by loading parameters from `transforms.yaml`.
*   **Algorithm-Specific Frames:** As new AHRS algorithms are integrated, their internal coordinate system assumptions should be documented here, along with any necessary transformations to/from the standard frames.
*   **NED vs. ENU:** Clarify the exact world frame convention (NED or ENU) for different datasets if they vary. For EuRoC, it's typically NED.
