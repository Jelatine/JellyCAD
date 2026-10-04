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

JyShapeBox::JyShapeBox(const double &width, const double &depth, const double &height) {
    const gp_Pnt pnt1(-width / 2, -depth / 2, 0);
    const gp_Pnt pnt2(width / 2, depth / 2, height);
    BRepPrimAPI_MakeBox make_box(pnt1, pnt2);
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
JyCylinder::JyCylinder(const std::array<double, 3> pos, const std::array<double, 3> dir, const double &_r, const double &_h) {
    const gp_Pnt pnt(pos[0], pos[1], pos[2]);
    const gp_Dir dir_(dir[0], dir[1], dir[2]);
    BRepPrimAPI_MakeCylinder make_cylinder(gp_Ax2(pnt, dir_), _r, _h);
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

JyWire::JyWire(const std::vector<JyShape> shapes) {
    BRepBuilderAPI_MakeWire make_wire;
    for (const auto &shape: shapes) {
        const auto s = shape.s_;
        if (s.IsNull()) { continue; }
        if (s.ShapeType() == TopAbs_EDGE) {
            make_wire.Add(TopoDS::Edge(s));
        } else if (s.ShapeType() == TopAbs_WIRE) {
            make_wire.Add(TopoDS::Wire(s));
        } else {
            throw std::runtime_error("wire: shape type not supported!");
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

JyText::JyText(const std::string &_text, const double &_size, const std::string &_font) {
    // 创建3D文本
    StdPrs_BRepFont aFont;
    const NCollection_String aFontName(_font.c_str());
    // 尝试加载字体
    if (!aFont.Init(aFontName, Font_FontAspect_Regular, _size)) {
        throw std::runtime_error("Failed init font!");
    }
    // 使用Font_BRepTextBuilder创建文本形状
    Font_BRepTextBuilder aTextBuilder;
    NCollection_String aText(_text.c_str());
    s_ = aTextBuilder.Perform(aFont, aText);
}
