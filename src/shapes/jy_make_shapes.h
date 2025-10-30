/*
 * Copyright (c) 2024. Li Jianbin. All rights reserved.
 * MIT License
 */
#ifndef JY_MAKE_SHAPES_H
#define JY_MAKE_SHAPES_H
#include "jy_make_edge.h"
#include "jy_make_face.h"
#include "jy_shape.h"

class JyMakeShapes {
public:
    static void configure_usertype(sol::state &lua);
};

// ==================== 三维基本形状 ====================
/**
 * @brief 长方体类
 */
class JyShapeBox : public JyShape {
public:
    /**
     * @brief 构造指定尺寸的长方体
     * @param _x 长度
     * @param _y 宽度
     * @param _z 高度
     */
    explicit JyShapeBox(const double &_x = 1, const double &_y = 1, const double &_z = 1);
};

/**
 * @brief 圆柱体类
 */
class JyCylinder : public JyShape {
public:
    /**
     * @brief 构造指定半径和高度的圆柱体
     * @param _r 半径
     * @param _h 高度
     */
    explicit JyCylinder(const double &_r = 1, const double &_h = 1);
};

/**
 * @brief 圆锥体类
 */
class JyCone : public JyShape {
public:
    /**
     * @brief 构造指定底部半径、顶部半径和高度的圆锥体
     * @param R1 底部半径
     * @param R2 顶部半径
     * @param H 高度
     */
    explicit JyCone(const double &R1 = 1, const double &R2 = 0, const double &H = 1);
};

/**
 * @brief 球体类
 */
class JySphere : public JyShape {
public:
    /**
     * @brief 构造指定半径的球体
     * @param _r 半径
     */
    explicit JySphere(const double &_r = 1);
};

class JyTorus : public JyShape {
public:
    /**
     * @brief 构造指定主半径、次半径和角度的圆环
     * @param R1 从管道中心到环面中心的距离
     * @param R2 管道半径
     * @param angle 角度(deg)
     */
    explicit JyTorus(const double &R1 = 2, const double &R2 = 1, const double &angle = 360);
};
class JyWedge : public JyShape {
public:
    /**
     * @brief 楔形
     * @param dx X方向长度
     * @param dy Y方向长度
     * @param dz Z方向长度
     * @param ltx 楔形中心到X轴的距离
     */
    explicit JyWedge(const double &dx = 1, const double &dy = 1, const double &dz = 1, const double &ltx = 0);

    explicit JyWedge(const double &dx, const double &dy, const double &dz, const double &xmin, const double &zmin, const double &xmax, const double &zmax);
};
class JyVertex : public JyShape {
public:
    /**
     * @brief 构造指定位置的顶点
     */
    explicit JyVertex(const double &x, const double &y, const double &z);
};

/**
 * @brief 线框类
 */
class JyWire : public JyShape {
public:
    /**
     * @brief 默认构造函数
     */
    explicit JyWire() = default;

    /**
     * @brief 根据参数表构造线框
     * @param _param 参数表
     */
    explicit JyWire(const sol::table &_param);

    explicit JyWire(const JyEdge &edge);
};

/**
 * @brief 多边形类
 */
class JyPolygon : public JyWire {
public:
    /**
     * @brief 根据参数表构造多边形
     * @param _vertices 顶点列表
     */
    explicit JyPolygon(const std::vector<std::array<double, 3>> _vertices = {});
};

/**
 * @brief 文本形状类
 */
class JyText : public JyShape {
public:
    /**
     * @brief 构造指定文本和字体大小的文本形状
     * @param _text 文本内容
     * @param _size 字体大小
     */
    explicit JyText(const std::string &_text = "", const double &_size = 1);
};
#endif// JY_MAKE_SHAPES_H