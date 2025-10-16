/*
 * Copyright (c) 2024. Li Jianbin. All rights reserved.
 * MIT License
 */
#include "jy_axes.h"
#include <Geom_Axis2Placement.hxx>
#include <gp_Quaternion.hxx>

JyAxes::JyAxes(const std::array<double, 6> pose, const double &length) : length_(length) {
    gp_Quaternion quaternion;
    const auto deg_rx = pose[3] * M_PI / 180.0;
    const auto deg_ry = pose[4] * M_PI / 180.0;
    const auto deg_rz = pose[5] * M_PI / 180.0;
    quaternion.SetEulerAngles(gp_YawPitchRoll, deg_rz, deg_ry, deg_rx);
    transformation.SetRotationPart(quaternion);
    transformation.SetTranslationPart(gp_XYZ(pose[0], pose[1], pose[2]));
}

std::array<double, 6> JyAxes::link2joint(const JyShape &child_shape) const {
    const auto child_shape_trsf = child_shape.s_.Location().Transformation();
    const auto trsf = transformation;
    const auto ref = trsf.Inverted() * child_shape_trsf;
    return transform(ref);
}
std::array<double, 6> JyAxes::joint2joint(const JyAxes &parent_joint) const {
    const auto parent_joint_trsf = parent_joint.transformation;
    const auto ref = parent_joint_trsf.Inverted() * transformation;
    return transform(ref);
}

JyAxes &JyAxes::move(const std::array<double, 6> ref) {
    gp_Trsf trsf;// transform from current pose to reference pose
    trsf.SetTranslationPart(gp_XYZ(ref[0], ref[1], ref[2]));
    gp_Quaternion quaternion;
    const auto deg_rx = ref[3] * M_PI / 180.0;
    const auto deg_ry = ref[4] * M_PI / 180.0;
    const auto deg_rz = ref[5] * M_PI / 180.0;
    quaternion.SetEulerAngles(gp_YawPitchRoll, deg_rz, deg_ry, deg_rx);
    trsf.SetRotationPart(quaternion);
    transformation = transformation * trsf;
    return *this;
}


JyAxes &JyAxes::sdh(const double &alpha, const double &a, const double &d, const double &theta) {
    gp_Trsf rot_alpha;
    gp_Quaternion q_alpha;
    q_alpha.SetEulerAngles(gp_YawPitchRoll, 0, 0, (alpha * M_PI / 180.0));
    rot_alpha.SetRotation(q_alpha);
    gp_Trsf trans_a;
    trans_a.SetTranslation(gp_XYZ(a, 0, 0));
    gp_Trsf trans_d;
    trans_d.SetTranslation(gp_XYZ(0, 0, d));
    gp_Trsf rot_theta;
    gp_Quaternion q_theta;
    q_theta.SetEulerAngles(gp_YawPitchRoll, (theta * M_PI / 180.0), 0, 0);
    rot_theta.SetRotation(q_theta);
    transformation = transformation * rot_theta * trans_d * trans_a * rot_alpha;
    return *this;
}

JyAxes &JyAxes::mdh(const double &alpha, const double &a, const double &d, const double &theta) {
    gp_Trsf rot_alpha;
    gp_Quaternion q_alpha;
    q_alpha.SetEulerAngles(gp_YawPitchRoll, 0, 0, (alpha * M_PI / 180.0));
    rot_alpha.SetRotation(q_alpha);
    gp_Trsf trans_a;
    trans_a.SetTranslation(gp_XYZ(a, 0, 0));
    gp_Trsf trans_d;
    trans_d.SetTranslation(gp_XYZ(0, 0, d));
    gp_Trsf rot_theta;
    gp_Quaternion q_theta;
    q_theta.SetEulerAngles(gp_YawPitchRoll, (theta * M_PI / 180.0), 0, 0);
    rot_theta.SetRotation(q_theta);
    transformation = transformation * rot_alpha * trans_a * rot_theta * trans_d;
    return *this;
}

std::array<double, 6> JyAxes::transform(const gp_Trsf &trsf) {
    gp_Quaternion quaternion = trsf.GetRotation();
    Standard_Real x, y, z;
    gp_XYZ theCoord;
    trsf.Transforms(theCoord);
    x = theCoord.X();
    y = theCoord.Y();
    z = theCoord.Z();
    double rx, ry, rz;
    quaternion.GetEulerAngles(gp_YawPitchRoll, rz, ry, rx);
    return {x, y, z, rx, ry, rz};
}