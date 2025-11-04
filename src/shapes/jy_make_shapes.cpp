/*
 * Copyright (c) 2024. Li Jianbin. All rights reserved.
 * MIT License
 */
#include "jy_make_shapes.h"
#include <BRepBuilderAPI_MakePolygon.hxx>
#include <BRepBuilderAPI_MakeVertex.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeCone.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRepPrimAPI_MakeSphere.hxx>
#include <BRepPrimAPI_MakeTorus.hxx>
#include <BRepPrimAPI_MakeWedge.hxx>
#include <Font_BRepTextBuilder.hxx>
#include <Geom_Line.hxx>
#include <TopoDS.hxx>

void JyMakeShapes::configure_usertype(sol::state &lua) {
    const auto box_ctor = sol::constructors<JyShapeBox(),
                                            JyShapeBox(const JyShapeBox &),
                                            JyShapeBox(const std::array<double, 3>, const std::array<double, 3>),
                                            JyShapeBox(const double &, const double &, const double &)>();
    const auto cylinder_ctor = sol::constructors<JyCylinder(),
                                                 JyCylinder(const JyCylinder &),
                                                 JyCylinder(const double &, const double &)>();
    const auto cone_ctor = sol::constructors<JyCone(),
                                             JyCone(const JyCone &),
                                             JyCone(const double &, const double &, const double &)>();
    const auto sphere_ctor = sol::constructors<JySphere(),
                                               JySphere(const JySphere &),
                                               JySphere(const double &)>();
    const auto torus_ctor = sol::constructors<JyTorus(),
                                              JyTorus(const JyTorus &),
                                              JyTorus(const double &, const double &),
                                              JyTorus(const double &, const double &, const double &)>();
    const auto wedge_ctor = sol::constructors<JyWedge(),
                                              JyWedge(const JyWedge &),
                                              JyWedge(const double &, const double &, const double &, const double &),
                                              JyWedge(const double &, const double &, const double &, const double &, const double &, const double &, const double &)>();
    lua.new_usertype<JyShapeBox>("box", box_ctor, sol::base_classes, sol::bases<JyShape>());
    lua.new_usertype<JyCylinder>("cylinder", cylinder_ctor, sol::base_classes, sol::bases<JyShape>());
    lua.new_usertype<JyCone>("cone", cone_ctor, sol::base_classes, sol::bases<JyShape>());
    lua.new_usertype<JySphere>("sphere", sphere_ctor, sol::base_classes, sol::bases<JyShape>());
    lua.new_usertype<JyTorus>("torus", torus_ctor, sol::base_classes, sol::bases<JyShape>());
    lua.new_usertype<JyWedge>("wedge", wedge_ctor, sol::base_classes, sol::bases<JyShape>());

    const auto vertex_ctor = sol::constructors<JyVertex(const JyVertex &),
                                               JyVertex(const double &, const double &, const double &)>();
    lua.new_usertype<JyVertex>("vertex", vertex_ctor, sol::base_classes, sol::bases<JyShape>());

    const auto wire_ctor = sol::constructors<JyWire(),
                                             JyWire(const JyWire &),
                                             JyWire(const JyEdge &),
                                             JyWire(const sol::table &)>();
    lua.new_usertype<JyWire>("wire", wire_ctor, sol::base_classes, sol::bases<JyShape>());
    const auto polygon_ctor = sol::constructors<JyPolygon(),
                                                JyPolygon(const JyPolygon &),
                                                JyPolygon(const std::vector<std::array<double, 3>>)>();
    lua.new_usertype<JyPolygon>("polygon", polygon_ctor, sol::base_classes, sol::bases<JyWire, JyShape>());

    const auto text_ctor = sol::constructors<JyText(),
                                             JyText(const std::string &),
                                             JyText(const std::string &, const double &),
                                             JyText(const JyText &)>();
    lua.new_usertype<JyText>("text", text_ctor, sol::base_classes, sol::bases<JyShape>());
}

JyShapeBox::JyShapeBox(const double &_x, const double &_y, const double &_z) {
    BRepPrimAPI_MakeBox make_box(_x, _y, _z);
    if (make_box.Wedge().IsDegeneratedShape()) { throw std::runtime_error("Is Degenerated Shape!"); }
    s_ = make_box;
}


JyShapeBox::JyShapeBox(const std::array<double, 3> p1, const std::array<double, 3> p2) {
    const gp_Pnt pnt1(p1[0], p1[1], p1[2]);
    const gp_Pnt pnt2(p2[0], p2[1], p2[2]);
    BRepPrimAPI_MakeBox make_box(pnt1, pnt2);
    if (make_box.Wedge().IsDegeneratedShape()) { throw std::runtime_error("Is Degenerated Shape!"); }
    s_ = make_box;
}

JyCylinder::JyCylinder(const double &_r, const double &_h) {
    BRepPrimAPI_MakeCylinder make_cylinder(_r, _h);
    s_ = make_cylinder;
}

JyCone::JyCone(const double &R1, const double &R2, const double &H) {
    if (std::abs(R1 - R2) < 1e-4) { throw std::runtime_error("R1==R2"); }
    BRepPrimAPI_MakeCone make_cone(R1, R2, H);
    s_ = make_cone;
}

JySphere::JySphere(const double &_r) {
    BRepPrimAPI_MakeSphere make_sphere(_r);
    s_ = make_sphere;
}

JyTorus::JyTorus(const double &R1, const double &R2, const double &angle) {
    BRepPrimAPI_MakeTorus make_torus(R1, R2, angle * M_PI / 180);
    s_ = make_torus;
}

JyWedge::JyWedge(const double &dx, const double &dy, const double &dz, const double &ltx) {
    BRepPrimAPI_MakeWedge make_wedge(dx, dy, dz, ltx);
    s_ = make_wedge;
}

JyWedge::JyWedge(const double &dx, const double &dy, const double &dz, const double &xmin, const double &zmin, const double &xmax, const double &zmax) {
    BRepPrimAPI_MakeWedge make_wedge(dx, dy, dz, xmin, zmin, xmax, zmax);
    s_ = make_wedge;
}

JyVertex::JyVertex(const double &x, const double &y, const double &z) {
    BRepBuilderAPI_MakeVertex make_vertex(gp_Pnt(x, y, z));
    s_ = make_vertex;
}

JyWire::JyWire(const sol::table &_param) {

    BRepBuilderAPI_MakeWire make_wire;
    for (const auto &p: _param) {
        if (!p.first.is<int>()) { continue; }
        if (p.second.is<JyEdge>()) {
            const auto shape = p.second.as<JyEdge>();
            if (shape.s_.IsNull()) { continue; }
            const auto edge = TopoDS::Edge(shape.s_);
            make_wire.Add(edge);
        } else if (p.second.is<JyWire>()) {
            const auto shape = p.second.as<JyWire>();
            if (shape.s_.IsNull()) { continue; }
            const auto wire = TopoDS::Wire(shape.s_);
            make_wire.Add(wire);
        } else {
        }
    }
    if (!make_wire.IsDone()) { return; }
    s_ = make_wire;
}


JyWire::JyWire(const JyEdge &edge) {
    BRepBuilderAPI_MakeWire make_wire(TopoDS::Edge(edge.s_));
    s_ = make_wire;
}

JyPolygon::JyPolygon(const std::vector<std::array<double, 3>> _vertices) {
    // 示例：polygon.new({ { 0, 0, 0 }, { 1, 0, 0 }, { 1.5, 1, 0 }, { 0.5, 1.5, 0 }, { -0.5, 1, 0 } }):show()
    BRepBuilderAPI_MakePolygon make_polygon;
    for (const auto &p: _vertices) {
        make_polygon.Add(gp_Pnt(p[0], p[1], p[2]));
    }
    if (!make_polygon.Added()) { throw std::runtime_error("Polygon Last Vertex NOT Added!"); }
    make_polygon.Close();
    s_ = make_polygon;
}

JyText::JyText(const std::string &_text, const double &_size) {
    // 创建3D文本
    StdPrs_BRepFont aFont;
    const NCollection_String aFontName("Arial");
    // 尝试加载字体
    if (!aFont.Init(aFontName, Font_FontAspect_Regular, _size)) {
        throw std::runtime_error("Failed init font!");
    }
    // 使用Font_BRepTextBuilder创建文本形状
    Font_BRepTextBuilder aTextBuilder;
    NCollection_String aText(_text.c_str());
    s_ = aTextBuilder.Perform(aFont, aText);
}
