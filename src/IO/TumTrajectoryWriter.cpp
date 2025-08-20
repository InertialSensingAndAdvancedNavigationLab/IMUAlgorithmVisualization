/// 包含类头文件
#include "IO/TumTrajectoryWriter.hpp"

/// 包含标准库
#include <fstream>
#include <iostream>
#include <iomanip>


/**
 * @brief       将一个位姿轨迹保存为TUM格式的文件 (静态方法)
 *  @details     此函数接收在内部FRD坐标系下的轨迹数据，并将其转换为
 * TUM/可视化(RDF)坐标系后进行保存。
 *
 *  @warning     自动处理四元数的坐标系变换和存储顺序(w,x,y,z -> x,y,z,w)转换。
 *
 * @param       参数名称: filePath                      数据类型:        const std::string&
 * @param       参数名称: trajectoryInFrd               数据类型:        const std::vector<PoseStamped>&
 *  @details     在内部FRD坐标系下计算出的轨迹数据。
 *
 * @return      保存成功返回 true                      数据类型:        bool
 **/
bool TumTrajectoryWriter::save(const std::string& filePath, const std::vector<PoseStamped>& trajectoryInFrd)
{
    std::ofstream outFile(filePath);
    if (!outFile.is_open())
    {
        std::cerr << "Error: Could not open TUM trajectory file for writing: " << filePath << std::endl;
        return false;
    }

    /// --- 定义从 Internal (FRD) 到 TUM/Viz (RDF) 的修正旋转 ---
    Eigen::Matrix3d R_rdf_from_frd;
    R_rdf_from_frd << 0, 1, 0,  // RDF.X (Right) = FRD.Y (Right)
                      0, 0, 1,  // RDF.Y (Down)  = FRD.Z (Down)
                      1, 0, 0;  // RDF.Z (Fwd)   = FRD.X (Fwd)
    Quaternion q_rdf_from_frd(R_rdf_from_frd);

    /// 设置输出流的浮点数精度
    outFile << std::fixed << std::setprecision(9);
    outFile << "# TUM Trajectory File from AHRS project" << std::endl;
    outFile << "# Format: timestamp tx ty tz qx qy qz qw" << std::endl;
    outFile << "# Coordinate System: Right-Down-Forward (X,Y,Z)" << std::endl;

    /// 遍历轨迹中的每一个FRD位姿点
    for (const auto& pose_frd : trajectoryInFrd)
    {
        /// --- 对位姿进行坐标系转换 ---
        /// 姿态变换: q_rdf = q_rdf_frd * q_frd
        Quaternion q_rdf = q_rdf_from_frd * pose_frd.orientation;
        /// 位置变换: p_rdf = q_rdf_frd * p_frd
        Vector3d p_rdf = q_rdf_from_frd * pose_frd.position;

        /// --- 写入文件 ---
        outFile << pose_frd.timestamp << " "
                << p_rdf.x() << " " << p_rdf.y() << " " << p_rdf.z() << " "
                << q_rdf.x() << " " << q_rdf.y() << " " << q_rdf.z() << " " << q_rdf.w() << std::endl;
    }

    outFile.close();
    return true;
}