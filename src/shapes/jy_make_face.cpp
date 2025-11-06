/*
 * Copyright (c) 2024. Li Jianbin. All rights reserved.
 * MIT License
 */
#include "jy_make_face.h"
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <GC_MakeTrimmedCylinder.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Wire.hxx>
#include <gp_Cone.hxx>
#include <gp_Cylinder.hxx>

void JyFace::configure_usertype(sol::state &lua) {
    const auto face_ctor = sol::constructors<JyFace(),
                                             JyFace(const JyFace &),
                                             JyFace(const JyShape &)>();
    lua.new_usertype<JyFace>("face", face_ctor, sol::base_classes, sol::bases<JyShape>());
    const auto plane_ctor = sol::constructors<JyPlane(const JyPlane &),
                                              JyPlane(const std::array<double, 3>, const std::array<double, 3>, const std::array<double, 4>)>();
    lua.new_usertype<JyPlane>("plane", plane_ctor, sol::base_classes, sol::bases<JyFace, JyShape>());
    const auto cylindrical_ctor = sol::constructors<JyCylindrical(const JyCylindrical &),
                                                    JyCylindrical(const std::array<double, 3>, const std::array<double, 3>, const double &, const std::array<double, 4>),
                                                    JyCylindrical(const std::array<double, 3>, const std::array<double, 3>, const double &, const double &)>();
    lua.new_usertype<JyCylindrical>("cylindrical", cylindrical_ctor, sol::base_classes, sol::bases<JyFace, JyShape>());
    const auto conical_ctor = sol::constructors<JyConical(const JyConical &),
                                                JyConical(const std::array<double, 3>, const std::array<double, 3>, const double &, const double &, const std::array<double, 4>)>();
    lua.new_usertype<JyConical>("conical", conical_ctor, sol::base_classes, sol::bases<JyFace, JyShape>());
}

JyFace::JyFace(const JyShape &_shape) {
    const auto shape_type = _shape.s_.ShapeType();
    if (shape_type == TopAbs_WIRE) {
        const auto wire = TopoDS::Wire(_shape.s_);
        BRepBuilderAPI_MakeFace make_face(wire);
        s_ = make_face;
    } else if (shape_type == TopAbs_EDGE) {
        const auto edge = TopoDS::Edge(_shape.s_);
        BRepBuilderAPI_MakeWire make_wire;
        make_wire.Add(edge);
        if (!make_wire.IsDone()) { throw std::runtime_error("Failed make wire!"); }
        BRepBuilderAPI_MakeFace make_face(make_wire);
        s_ = make_face;
    } else {
        throw std::runtime_error("Face: Not Support Shape Type!");
    }
}

JyPlane::JyPlane(const std::array<double, 3> pos, const std::array<double, 3> dir, const std::array<double, 4> uv) {
    // 示例：plane.new({ 1, 1, 1 }, { 0, 0, 1 }, { -1, 1, -1, 1 }):show()
    gp_Pln pln(gp_Pnt(pos[0], pos[1], pos[2]), gp_Dir(dir[0], dir[1], dir[2]));
    BRepBuilderAPI_MakeFace make_face(pln, uv[0], uv[1], uv[2], uv[3]);
    s_ = make_face;
}

JyCylindrical::JyCylindrical(const std::array<double, 3> pos, const std::array<double, 3> dir, const double &r, const std::array<double, 4> uv) {
    // 示例：cylindrical.new({ 1, 1, 1 }, { 0, 0, 1 }, 3, { 0, 360, -1, 2 }):show()
    gp_Cylinder cylinder(gp_Ax2(gp_Pnt(pos[0], pos[1], pos[2]), gp_Dir(dir[0], dir[1], dir[2])), r);
    BRepBuilderAPI_MakeFace make_face(cylinder, uv[0] * M_PI / 180, uv[1] * M_PI / 180, uv[2], uv[3]);
    s_ = make_face;
}

JyCylindrical::JyCylindrical(const std::array<double, 3> pos, const std::array<double, 3> dir, const double &r, const double &h) {
    // 示例：cylindrical.new({ 1, 1, 1 }, { 0, 0, 1 }, 3, 5):show()
    const Handle(Geom_RectangularTrimmedSurface) TrimmedSurface = GC_MakeTrimmedCylinder({gp_Pnt(pos[0], pos[1], pos[2]), gp_Dir(dir[0], dir[1], dir[2])}, r, h);
    BRepBuilderAPI_MakeFace make_face(TrimmedSurface, 1e-6);
    s_ = make_face;
}

JyConical::JyConical(const std::array<double, 3> pos, const std::array<double, 3> dir, const double &angle, const double &r, const std::array<double, 4> uv) {
    // 示例：conical.new({ 1, 1, 1 }, { 0, 0, 1 }, 45, 3, { 0, 270, -1, 2 }):show()
    const auto rad = angle * M_PI / 180;
    if (std::abs(rad) < gp::Resolution() || std::abs(rad) >= M_PI / 2 - gp::Resolution()) {
        // Raises ConstructionError if Abs(theAng) < Resolution from gp or Abs(theAng) >= PI/2 -Resolution
        throw std::runtime_error("Conical: The absolute value of the angle must be greater than 0 and less than 90 deg!");
    }
    gp_Cone conical(gp_Ax2(gp_Pnt(pos[0], pos[1], pos[2]), gp_Dir(dir[0], dir[1], dir[2])), rad, r);
    BRepBuilderAPI_MakeFace make_face(conical, uv[0] * M_PI / 180, uv[1] * M_PI / 180, uv[2], uv[3]);
    s_ = make_face;
}