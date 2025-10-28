/*
 * Copyright (c) 2024. Li Jianbin. All rights reserved.
 * MIT License
 */
#ifndef JY_MAKE_SHAPES_H
#define JY_MAKE_SHAPES_H
#include "jy_shape.h"
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
 * @brief 边类
 */
class JyEdge : public JyShape {
public:
    /**
     * @brief 根据类型和参数构造边
     * @param _type 边类型
     * @param _vec1 起点或中心点
     * @param _vec2 终点或方向向量
     * @param _r1 半径1（可选）
     * @param _r2 半径2（可选）
     */
    explicit JyEdge(const std::string &_type, const std::array<double, 3> _vec1, const std::array<double, 3> _vec2,
                    const double &_r1 = -1, const double &_r2 = -1);
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
 * @brief 面类
 */
class JyFace : public JyShape {
public:
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

class JyPipe : public JyShape {
public:
    /**
     * @brief 构造指定半径和高度的管道
     * @param wire 管道的 wire 对象
     * @param shape 管道的 shape 对象
     */
    explicit JyPipe(const JyWire &wire, const JyShape &shape);
};
#endif// JY_MAKE_SHAPES_H