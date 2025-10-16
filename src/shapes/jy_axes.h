/*
 * Copyright (c) 2024. Li Jianbin. All rights reserved.
 * MIT License
 */
#ifndef JY_AXES_H
#define JY_AXES_H

#include "jy_shape.h"
#include <array>

class JyAxes {
    gp_Trsf transformation;
    double length_{1};// length of the axes

public:
    JyAxes(const double &length = 1) : length_(length) {}

    JyAxes(const std::array<double, 6> pose, const double &length = 1);

    gp_Trsf data() const { return transformation; }

    double length() const { return length_; }

    std::array<double, 6> link2joint(const JyShape &child_shape) const;

    std::array<double, 6> joint2joint(const JyAxes &parent_joint) const;

    JyAxes &move(const std::array<double, 6> ref);// move the axes to the reference pose

    JyAxes &sdh(const double &alpha, const double &a, const double &d, const double &theta);

    JyAxes &mdh(const double &alpha, const double &a, const double &d, const double &theta);

private:
    static std::array<double, 6> transform(const gp_Trsf &trsf);
};

#endif