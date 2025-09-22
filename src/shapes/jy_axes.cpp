/*
 * Copyright (c) 2024. Li Jianbin. All rights reserved.
 * MIT License
 */
#include "jy_axes.h"
#include <Geom_Axis2Placement.hxx>
#include <gp_Quaternion.hxx>

JyAxes::JyAxes(const std::array<double, 6> pose, const double &length) {
    gp_Quaternion quaternion;
    const auto deg_rx = pose[3] * M_PI / 180.0;
    const auto deg_ry = pose[4] * M_PI / 180.0;
    const auto deg_rz = pose[5] * M_PI / 180.0;
    quaternion.SetEulerAngles(gp_YawPitchRoll, deg_rz, deg_ry, deg_rx);
    transformation.SetRotationPart(quaternion);
    transformation.SetTranslationPart(gp_XYZ(pose[0], pose[1], pose[2]));
    gp_Ax2 ax2;
    ax2.Transform(transformation);
    opencascade::handle<Geom_Axis2Placement> axis = new Geom_Axis2Placement(ax2);
    trihedron = new AIS_Trihedron(axis);
    trihedron->SetDatumDisplayMode(Prs3d_DM_WireFrame);
    trihedron->SetDrawArrows(false);
    trihedron->Attributes()->DatumAspect()->LineAspect(Prs3d_DatumParts_XAxis)->SetWidth(2.5);
    trihedron->Attributes()->DatumAspect()->LineAspect(Prs3d_DatumParts_YAxis)->SetWidth(2.5);
    trihedron->Attributes()->DatumAspect()->LineAspect(Prs3d_DatumParts_ZAxis)->SetWidth(2.5);
    trihedron->SetDatumPartColor(Prs3d_DatumParts_XAxis, Quantity_NOC_RED2);
    trihedron->SetDatumPartColor(Prs3d_DatumParts_YAxis, Quantity_NOC_GREEN2);
    trihedron->SetDatumPartColor(Prs3d_DatumParts_ZAxis, Quantity_NOC_BLUE2);
    trihedron->SetLabel(Prs3d_DatumParts_XAxis, "");
    trihedron->SetLabel(Prs3d_DatumParts_YAxis, "");
    trihedron->SetLabel(Prs3d_DatumParts_ZAxis, "");
    trihedron->SetSize(length);
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