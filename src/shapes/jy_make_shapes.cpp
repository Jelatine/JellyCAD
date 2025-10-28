/*
 * Copyright (c) 2024. Li Jianbin. All rights reserved.
 * MIT License
 */
#include "jy_make_shapes.h"
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakePolygon.hxx>
#include <BRepBuilderAPI_MakeVertex.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepOffsetAPI_MakePipe.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeCone.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRepPrimAPI_MakeSphere.hxx>
#include <BRepPrimAPI_MakeTorus.hxx>
#include <BRepPrimAPI_MakeWedge.hxx>
#include <Font_BRepTextBuilder.hxx>
#include <Geom_Line.hxx>
#include <TopoDS.hxx>
JyShapeBox::JyShapeBox(const double &_x, const double &_y, const double &_z) {
    BRepPrimAPI_MakeBox make_box(_x, _y, _z);
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

JyEdge::JyEdge(const std::string &_type, const std::array<double, 3> _vec1, const std::array<double, 3> _vec2,
               const double &_r1, const double &_r2) {
    const auto pos = gp_Pnt(_vec1[0], _vec1[1], _vec1[2]);
    const auto dir = gp_Dir(_vec2[0], _vec2[1], _vec2[2]);
    if (_type == "lin") {
        const auto end = gp_Pnt(_vec2[0], _vec2[1], _vec2[2]);
        BRepBuilderAPI_MakeEdge make_edge_lin(pos, end);
        s_ = make_edge_lin;
    } else if (_type == "circ") {
        // 示例：edge.new('circ', { 0, 0, 0 }, { 0, 0, 1 }, 1);
        if (_r1 <= 0) { throw std::invalid_argument("Invalid circle radius!"); }
        gp_Circ circ({pos, dir}, _r1);
        BRepBuilderAPI_MakeEdge make_edge_circ(circ);
        s_ = make_edge_circ;
    } else if (_type == "elips") {
        // R1: 大半轴  R2: 小半轴，R1必须大于R2
        if (_r1 <= 0 || _r2 <= 0 || _r1 < _r2) {
            throw std::invalid_argument("Invalid ellipse parameters! R1 > R2 > 0");
        }
        // 示例：edge.new('elips', { 0, 0, 0 }, { 0, 0, 1 }, 4, 2);
        gp_Elips elips({pos, dir}, _r1, _r2);
        BRepBuilderAPI_MakeEdge make_edge_elips(elips);
        s_ = make_edge_elips;
    } else if (_type == "hypr") {
        gp_Hypr hypr({pos, dir}, _r1, _r2);
        BRepBuilderAPI_MakeEdge make_edge_hypr(hypr);
        s_ = make_edge_hypr;
    } else if (_type == "parab") {
        gp_Parab parab({pos, dir}, _r1);
        BRepBuilderAPI_MakeEdge make_edge_parab(parab);
        s_ = make_edge_parab;
    } else {
        throw std::runtime_error("Edge: wrong type!");
    }
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

JyPolygon::JyPolygon(const std::vector<std::array<double, 3>> _vertices) {
    BRepBuilderAPI_MakePolygon make_polygon;
    for (const auto &p: _vertices) {
        make_polygon.Add(gp_Pnt(p[0], p[1], p[2]));
    }
    if (!make_polygon.Added()) { throw std::runtime_error("Polygon Last Vertex NOT Added!"); }
    make_polygon.Close();
    s_ = make_polygon;
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

JyPipe::JyPipe(const JyWire &wire, const JyShape &shape) {
    /* 示例：管道
    local e = edge.new('circ', { 0, 0, 0 }, { 0, 0, 1 }, 1)
    local w = polygon.new({{0,0,0},{5,5,5}})
    pipe.new(w,e):show()
    */
    const TopoDS_Wire wire_ = TopoDS::Wire(wire.s_);
    BRepOffsetAPI_MakePipe make_pipe(wire_, shape.s_);
    s_ = make_pipe;
}
