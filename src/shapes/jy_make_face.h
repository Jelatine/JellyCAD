/*
 * Copyright (c) 2024. Li Jianbin. All rights reserved.
 * MIT License
 */
#ifndef JY_MAKE_FACE_H
#define JY_MAKE_FACE_H
#include "jy_shape.h"
/**
 * @brief 面类
 */
class JyFace : public JyShape {
public:
    static void configure_usertype(sol::state &lua);
    /**
     * @brief 默认构造函数
     */
    JyFace() = default;

    /**
     * @brief 根据形状构造面
     * @param _shape 形状对象
     */
    explicit JyFace(const JyShape &_shape);
};


class JyPlane : public JyFace {
public:
    /**
     * @brief 根据位置、方向和UV参数构造平面
     * @param pos 平面上的一个点
     * @param dir 平面的法线向量
     * @param uv UV参数，{umin, umax, vmin, vmax} U:X轴限位, V:Y轴限位
     */
    explicit JyPlane(const std::array<double, 3> pos, const std::array<double, 3> dir, const std::array<double, 4> uv);
};

class JyCylindrical : public JyFace {
public:
    /**
     * @brief 根据位置、方向、半径和UV参数构造圆柱面
     * @param pos 圆柱中心位置
     * @param dir 圆柱方向向量
     * @param r 圆柱半径
     * @param uv UV参数，{umin, umax, vmin, vmax} U:圆弧限位(度), V:高度限位
     */
    explicit JyCylindrical(const std::array<double, 3> pos, const std::array<double, 3> dir, const double &r, const std::array<double, 4> uv);

    /**
     * @brief 根据位置、方向、半径和高度构造圆柱面
     * @param pos 圆柱中心位置
     * @param dir 圆柱方向向量
     * @param r 圆柱半径
     * @param h 圆柱高度
     */
    explicit JyCylindrical(const std::array<double, 3> pos, const std::array<double, 3> dir, const double &r, const double &h);
};

class JyConical : public JyFace {
public:
    /**
     * @brief 根据位置、方向、角度、半径和UV参数构造圆锥面
     * @param pos 圆锥中心位置
     * @param dir 圆锥方向向量
     * @param angle 圆锥角度(度)
     * @param r 圆锥半径
     * @param uv UV参数，{umin, umax, vmin, vmax} U:圆弧限位(度), V:高度限位
     */
    explicit JyConical(const std::array<double, 3> pos, const std::array<double, 3> dir, const double &angle, const double &r, const std::array<double, 4> uv);
};
#endif//JY_MAKE_FACE_H