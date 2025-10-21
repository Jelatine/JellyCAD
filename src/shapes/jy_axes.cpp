/*
 * Copyright (c) 2024. Li Jianbin. All rights reserved.
 * MIT License
 */
#include "jy_axes.h"
#include <Geom_Axis2Placement.hxx>
#include <gp_Quaternion.hxx>

/**
 * @brief 通过位姿数组构造坐标轴对象
 *
 * 该构造函数接收一个6维位姿数组，将其转换为内部的变换矩阵表示。
 * 旋转角度从度数转换为弧度，并使用四元数表示旋转。
 *
 * @param pose 6维位姿数组 [x, y, z, rx, ry, rz]，旋转角度单位为度
 * @param length 坐标轴长度，用于可视化显示
 */
JyAxes::JyAxes(const std::array<double, 6> pose, const double &length) : length_(length) {
    gp_Quaternion quaternion;
    // 将旋转角度从度转换为弧度
    const auto deg_rx = pose[3] * M_PI / 180.0;
    const auto deg_ry = pose[4] * M_PI / 180.0;
    const auto deg_rz = pose[5] * M_PI / 180.0;
    // 使用欧拉角（YawPitchRoll顺序）设置四元数：Yaw(rz), Pitch(ry), Roll(rx)
    quaternion.SetEulerAngles(gp_YawPitchRoll, deg_rz, deg_ry, deg_rx);
    transformation.SetRotationPart(quaternion);
    transformation.SetTranslationPart(gp_XYZ(pose[0], pose[1], pose[2]));
}

/**
 * @brief 计算从当前关节坐标系到子形状的相对位姿
 *
 * 该方法计算子形状相对于当前关节坐标系的位姿变换。
 * 通过将当前坐标系变换求逆，然后左乘子形状的变换矩阵得到相对变换。
 *
 * @param child_shape 子形状对象
 * @return 相对位姿数组 [x, y, z, rx, ry, rz]
 */
std::array<double, 6> JyAxes::link2joint(const JyShape &child_shape) const {
    const auto child_shape_trsf = child_shape.s_.Location().Transformation();
    const auto trsf = transformation;
    // 计算相对变换：T_joint^(-1) * T_child
    const auto ref = trsf.Inverted() * child_shape_trsf;
    return transform(ref);
}
/**
 * @brief 计算从父关节到当前关节的相对位姿
 *
 * 该方法计算当前关节相对于父关节的位姿变换。
 * 通过将父关节变换求逆，然后左乘当前关节的变换矩阵得到相对变换。
 *
 * @param parent_joint 父关节坐标轴对象
 * @return 相对位姿数组 [x, y, z, rx, ry, rz]
 */
std::array<double, 6> JyAxes::joint2joint(const JyAxes &parent_joint) const {
    const auto parent_joint_trsf = parent_joint.transformation;
    // 计算相对变换：T_parent^(-1) * T_current
    const auto ref = parent_joint_trsf.Inverted() * transformation;
    return transform(ref);
}

/**
 * @brief 移动坐标轴到指定的参考位姿
 *
 * 该方法将当前坐标系进行额外的变换，使其移动到指定的参考位姿。
 * 变换是相对于当前坐标系的，通过右乘参考变换实现。
 *
 * @param ref 参考位姿 [x, y, z, rx, ry, rz]，旋转角度单位为度
 * @return 当前对象的引用，支持链式调用
 */
JyAxes &JyAxes::move(const std::array<double, 6> ref) {
    gp_Trsf trsf;  // 从当前位姿到参考位姿的变换
    trsf.SetTranslationPart(gp_XYZ(ref[0], ref[1], ref[2]));
    gp_Quaternion quaternion;
    // 将旋转角度从度转换为弧度
    const auto deg_rx = ref[3] * M_PI / 180.0;
    const auto deg_ry = ref[4] * M_PI / 180.0;
    const auto deg_rz = ref[5] * M_PI / 180.0;
    quaternion.SetEulerAngles(gp_YawPitchRoll, deg_rz, deg_ry, deg_rx);
    trsf.SetRotationPart(quaternion);
    // 右乘变换，实现相对于当前坐标系的移动
    transformation = transformation * trsf;
    return *this;
}


/**
 * @brief 使用标准DH参数设置坐标系变换
 *
 * 标准DH（Denavit-Hartenberg）参数法的变换顺序：
 * 1. 绕Z轴旋转theta角（关节角）
 * 2. 沿Z轴平移d（连杆偏距）
 * 3. 沿X轴平移a（连杆长度）
 * 4. 绕X轴旋转alpha角（连杆扭角）
 *
 * 变换矩阵：T = T_current * Rot(Z,theta) * Trans(Z,d) * Trans(X,a) * Rot(X,alpha)
 *
 * @param alpha 连杆扭角，绕X轴旋转，单位：度
 * @param a 连杆长度，沿X轴平移
 * @param d 连杆偏距，沿Z轴平移
 * @param theta 关节角，绕Z轴旋转，单位：度
 * @return 当前对象的引用，支持链式调用
 */
JyAxes &JyAxes::sdh(const double &alpha, const double &a, const double &d, const double &theta) {
    // 构造绕X轴旋转alpha角的变换
    gp_Trsf rot_alpha;
    gp_Quaternion q_alpha;
    q_alpha.SetEulerAngles(gp_YawPitchRoll, 0, 0, (alpha * M_PI / 180.0));  // Roll(alpha)
    rot_alpha.SetRotation(q_alpha);
    // 构造沿X轴平移a的变换
    gp_Trsf trans_a;
    trans_a.SetTranslation(gp_XYZ(a, 0, 0));
    // 构造沿Z轴平移d的变换
    gp_Trsf trans_d;
    trans_d.SetTranslation(gp_XYZ(0, 0, d));
    // 构造绕Z轴旋转theta角的变换
    gp_Trsf rot_theta;
    gp_Quaternion q_theta;
    q_theta.SetEulerAngles(gp_YawPitchRoll, (theta * M_PI / 180.0), 0, 0);  // Yaw(theta)
    rot_theta.SetRotation(q_theta);
    // 按标准DH顺序组合变换
    transformation = transformation * rot_theta * trans_d * trans_a * rot_alpha;
    return *this;
}

/**
 * @brief 使用修正DH参数设置坐标系变换
 *
 * 修正DH（Modified Denavit-Hartenberg）参数法的变换顺序：
 * 1. 绕X轴旋转alpha角（连杆扭角）
 * 2. 沿X轴平移a（连杆长度）
 * 3. 绕Z轴旋转theta角（关节角）
 * 4. 沿Z轴平移d（连杆偏距）
 *
 * 变换矩阵：T = T_current * Rot(X,alpha) * Trans(X,a) * Rot(Z,theta) * Trans(Z,d)
 *
 * 修正DH与标准DH的区别：变换顺序不同，修正DH先处理X轴变换，再处理Z轴变换
 *
 * @param alpha 连杆扭角，绕X轴旋转，单位：度
 * @param a 连杆长度，沿X轴平移
 * @param d 连杆偏距，沿Z轴平移
 * @param theta 关节角，绕Z轴旋转，单位：度
 * @return 当前对象的引用，支持链式调用
 */
JyAxes &JyAxes::mdh(const double &alpha, const double &a, const double &d, const double &theta) {
    // 构造绕X轴旋转alpha角的变换
    gp_Trsf rot_alpha;
    gp_Quaternion q_alpha;
    q_alpha.SetEulerAngles(gp_YawPitchRoll, 0, 0, (alpha * M_PI / 180.0));  // Roll(alpha)
    rot_alpha.SetRotation(q_alpha);
    // 构造沿X轴平移a的变换
    gp_Trsf trans_a;
    trans_a.SetTranslation(gp_XYZ(a, 0, 0));
    // 构造沿Z轴平移d的变换
    gp_Trsf trans_d;
    trans_d.SetTranslation(gp_XYZ(0, 0, d));
    // 构造绕Z轴旋转theta角的变换
    gp_Trsf rot_theta;
    gp_Quaternion q_theta;
    q_theta.SetEulerAngles(gp_YawPitchRoll, (theta * M_PI / 180.0), 0, 0);  // Yaw(theta)
    rot_theta.SetRotation(q_theta);
    // 按修正DH顺序组合变换
    transformation = transformation * rot_alpha * trans_a * rot_theta * trans_d;
    return *this;
}

/**
 * @brief 将变换矩阵转换为6维位姿数组
 *
 * 该静态方法从变换矩阵中提取位置和旋转信息，并转换为6维数组表示。
 * 旋转部分通过四元数提取，然后转换为欧拉角（YawPitchRoll顺序）。
 *
 * @param trsf 变换矩阵
 * @return 6维位姿数组 [x, y, z, rx, ry, rz]，旋转角度单位为弧度
 */
std::array<double, 6> JyAxes::transform(const gp_Trsf &trsf) {
    // 提取旋转部分（四元数）
    gp_Quaternion quaternion = trsf.GetRotation();
    // 提取平移部分
    Standard_Real x, y, z;
    gp_XYZ theCoord;
    trsf.Transforms(theCoord);
    x = theCoord.X();
    y = theCoord.Y();
    z = theCoord.Z();
    // 将四元数转换为欧拉角（YawPitchRoll顺序）
    double rx, ry, rz;
    quaternion.GetEulerAngles(gp_YawPitchRoll, rz, ry, rx);
    return {x, y, z, rx, ry, rz};
}