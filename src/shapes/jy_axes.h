/*
 * Copyright (c) 2024. Li Jianbin. All rights reserved.
 * MIT License
 */
#ifndef JY_AXES_H
#define JY_AXES_H

#include "jy_shape.h"
#include <AIS_Trihedron.hxx>
#include <array>

class JyAxes {
    opencascade::handle<AIS_Trihedron> trihedron;

    gp_Trsf transformation;
    // friend class JyShape;

public:
    JyAxes(const std::array<double, 6> pose, const double &length);

    opencascade::handle<AIS_Trihedron> data() const { return trihedron; }

    std::array<double, 6> link2joint(const JyShape &child_shape) const;

    std::array<double, 6> joint2joint(const JyAxes &parent_joint) const;

private:
    static std::array<double, 6> transform(const gp_Trsf &trsf);
};

#endif