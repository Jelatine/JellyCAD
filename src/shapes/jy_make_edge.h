/*
 * Copyright (c) 2024. Li Jianbin. All rights reserved.
 * MIT License
 */
#ifndef JY_MAKE_EDGE_H
#define JY_MAKE_EDGE_H
#include "jy_shape.h"
/**
 * @brief 边类
 */
class JyEdge : public JyShape {
public:
    static void configure_usertype(sol::state &lua);

    JyEdge() = default;
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

class JyBezier : public JyEdge {
public:
    explicit JyBezier(const std::vector<std::array<double, 3>> poles);
    explicit JyBezier(const std::vector<std::array<double, 3>> poles, const std::vector<double> weights);
};

class JyBSpline : public JyEdge {
public:
    explicit JyBSpline(const std::vector<std::array<double, 3>> poles,
                       const std::vector<double> knots,
                       const std::vector<int> multiplicities,
                       const int &degree);
};

class JyLine : public JyEdge {
public:
    explicit JyLine(const std::array<double, 3> p1, const std::array<double, 3> p2);
};

class JyCircle : public JyEdge {
public:
    explicit JyCircle(const std::array<double, 3> center, const std::array<double, 3> normal, const double &radius);
};

class JyEllipse : public JyEdge {
public:
    explicit JyEllipse(const std::array<double, 3> center, const std::array<double, 3> normal, const double &radius1, const double &radius2);
};

class JyHyperbola : public JyEdge {
public:
    explicit JyHyperbola(const std::array<double, 3> center, const std::array<double, 3> normal,
                         const double &radius1, const double &radius2, const double &p1, const double &p2);
};

class JyParabola : public JyEdge {
public:
    explicit JyParabola(const std::array<double, 3> center, const std::array<double, 3> normal,
                        const double &radius, const double &p1, const double &p2);
};

#endif