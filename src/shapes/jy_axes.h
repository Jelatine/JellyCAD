/*
 * Copyright (c) 2024. Li Jianbin. All rights reserved.
 * MIT License
 */
#ifndef JY_AXES_H
#define JY_AXES_H

#include "jy_shape.h"
#include <array>

/**
 * @class JyAxes
 * @brief 坐标轴类，用于表示机器人的关节坐标系或参考坐标系
 *
 * 该类主要用于机器人运动学建模，支持标准DH和修正DH参数表示法。
 * 坐标系的姿态使用 gp_Trsf 变换矩阵表示。
 */
class JyAxes {
    gp_Trsf transformation;// 坐标系变换矩阵
    double length_{1};     // 坐标轴长度（用于可视化显示）

public:
    static sol::usertype<JyAxes> configure_usertype(sol::state &lua);

public:
    /**
     * @brief 默认构造函数
     * @param length 坐标轴长度，默认为1
     */
    JyAxes(const double &length = 1) : length_(length) {}

    /**
     * @brief 通过位姿数组构造坐标轴
     * @param pose 6维位姿数组 [x, y, z, rx, ry, rz]，其中前3个为位置，后3个为旋转角度
     * @param length 坐标轴长度，默认为1
     */
    JyAxes(const std::array<double, 6> pose, const double &length = 1);

    /**
     * @brief 获取变换矩阵
     * @return 当前坐标系的变换矩阵
     */
    gp_Trsf data() const { return transformation; }

    /**
     * @brief 获取坐标轴长度
     * @return 坐标轴长度
     */
    double length() const { return length_; }

    /**
     * @brief 计算从当前坐标系到子形状的相对位姿
     * @param child_shape 子形状对象
     * @return 6维相对位姿数组 [x, y, z, rx, ry, rz]
     */
    std::array<double, 6> link2joint(const JyShape &child_shape) const;

    /**
     * @brief 计算从父关节到当前关节的相对位姿
     * @param parent_joint 父关节坐标轴
     * @return 6维相对位姿数组 [x, y, z, rx, ry, rz]
     */
    std::array<double, 6> joint2joint(const JyAxes &parent_joint) const;

    /**
     * @brief 移动坐标轴到指定参考位姿
     * @param ref 目标位姿 [x, y, z, rx, ry, rz]
     * @return 当前对象的引用（支持链式调用）
     */
    JyAxes &move(const std::array<double, 6> ref);

    /**
     * @brief 使用标准DH参数设置坐标系变换
     * @param alpha 连杆扭角（绕x轴旋转，单位：角度）
     * @param a 连杆长度（沿x轴平移）
     * @param d 连杆偏距（沿z轴平移）
     * @param theta 关节角（绕z轴旋转，单位：角度）
     * @return 当前对象的引用（支持链式调用）
     *
     * 标准DH变换顺序：Rot(Z, theta) -> Trans(Z, d) -> Trans(X, a) -> Rot(X, alpha)
     */
    JyAxes &sdh(const double &alpha, const double &a, const double &d, const double &theta);

    /**
     * @brief 使用修正DH参数设置坐标系变换
     * @param alpha 连杆扭角（绕x轴旋转，单位：角度）
     * @param a 连杆长度（沿x轴平移）
     * @param d 连杆偏距（沿z轴平移）
     * @param theta 关节角（绕z轴旋转，单位：角度）
     * @return 当前对象的引用（支持链式调用）
     *
     * 修正DH变换顺序：Trans(X, a) -> Rot(X, alpha) -> Rot(Z, theta) -> Trans(Z, d)
     */
    JyAxes &mdh(const double &alpha, const double &a, const double &d, const double &theta);

private:
    /**
     * @brief 将变换矩阵转换为6维位姿数组
     * @param trsf 变换矩阵
     * @return 6维位姿数组 [x, y, z, rx, ry, rz]
     */
    static std::array<double, 6> transform(const gp_Trsf &trsf);
};

#endif