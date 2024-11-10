/*
 * Copyright (c) 2024. Li Jianbin. All rights reserved.
 * MIT License
 */
#include "jy_shape.h"
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRepPrimAPI_MakeCone.hxx>
#include <BRepAlgoAPI_Fuse.hxx>
#include <BRepAlgoAPI_Cut.hxx>
#include <BRepAlgoAPI_Common.hxx>
#include <StlAPI.hxx>
#include <STEPControl_Writer.hxx>
#include <gp_Quaternion.hxx>
#include <BRepFilletAPI_MakeFillet.hxx>
#include <TopoDS.hxx>
#include <TopExp_Explorer.hxx>
#include <Geom_Line.hxx>
#include <BRepFilletAPI_MakeChamfer.hxx>
#include <BRepAdaptor_Curve.hxx>
#include <BRepPrimAPI_MakeSphere.hxx>
#include <BRepPrimAPI_MakePrism.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakePolygon.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <IGESControl_Writer.hxx>

void JyShape::process_opt(const sol::table &_opt) const {
    if (!_opt) { return; }
    if (_opt["color"].is<std::string>()) {
        color(_opt["color"]);
    }
    locate(_opt);
}

std::string JyShape::type() const {
    if (!s_) { return "null"; }
    const auto t = s_->Shape().ShapeType();
    switch (t) {
        case TopAbs_VERTEX:
            return "vertex";
        case TopAbs_EDGE:
            return "edge";
        case TopAbs_FACE:
            return "face";
        case TopAbs_SHELL:
            return "shell";
        case TopAbs_WIRE:
            return "wire";
        case TopAbs_SOLID:
            return "solid";
        case TopAbs_COMPOUND:
            return "compound";
        case TopAbs_COMPSOLID:
            return "compsolid";
        case TopAbs_SHAPE:
            return "shape";
    }
    return "unknown";
}

bool JyShape::fuse(const JyShape &_other) const {
    return algo<BRepAlgoAPI_Fuse>(_other);
}

bool JyShape::cut(const JyShape &_other) const {
    return algo<BRepAlgoAPI_Cut>(_other);
}

bool JyShape::common(const JyShape &_other) const {
    return algo<BRepAlgoAPI_Common>(_other);
}

bool JyShape::fillet(const double &_r, const sol::table &_cond) const {
    BRepFilletAPI_MakeFillet MF(s_->Shape());
    for (TopExp_Explorer ex(s_->Shape(), TopAbs_EDGE); ex.More(); ex.Next()) {
        TopoDS_Edge edge = TopoDS::Edge(ex.Current());
        if (edge_filter(edge, _cond)) { MF.Add(_r, TopoDS::Edge(ex.Current())); }
    }
    if (MF.NbContours() == 0) { return false; }
    MF.Build();
    if (!MF.IsDone()) { return false; }
    s_->SetShape(MF.Shape());
    return true;
}

bool JyShape::chamfer(const double &_dis, const sol::table &_cond) const {
    BRepFilletAPI_MakeChamfer MC(s_->Shape());
    TopTools_IndexedDataMapOfShapeListOfShape M;
    TopExp::MapShapesAndAncestors(s_->Shape(), TopAbs_EDGE, TopAbs_FACE, M);
    for (Standard_Integer i = 1; i < M.Extent(); i++) {
        TopoDS_Edge E = TopoDS::Edge(M.FindKey(i));
        TopoDS_Face F = TopoDS::Face(M.FindFromIndex(i).First());
        if (edge_filter(E, _cond)) { MC.Add(_dis, _dis, E, F); }
    }
    if (MC.NbContours() == 0) { return false; }
    MC.Build();
    if (!MC.IsDone()) { return false; }
    s_->SetShape(MC.Shape());
    return true;
}


bool JyShape::edge_filter(const TopoDS_Edge &_edge, const sol::table &_cond) {
    //!< 过滤曲线类型
    if (_cond["type"].is<std::string>()) {
        const std::string type_name = _cond["type"];
        BRepAdaptor_Curve C;
        C.Initialize(_edge);
        switch (C.GetType()) {
            case GeomAbs_Line:
                if (type_name != "line") { return false; }
                break;
            case GeomAbs_Circle:
                if (type_name != "circle") { return false; }
                break;
            case GeomAbs_Ellipse:
                if (type_name != "ellipse") { return false; }
                break;
            case GeomAbs_Hyperbola:
                if (type_name != "hyperbola") { return false; }
                break;
            case GeomAbs_Parabola:
                if (type_name != "parabola") { return false; }
                break;
            case GeomAbs_BezierCurve:
                if (type_name != "bezier_curve") { return false; }
                break;
            default:
                return false;
        }
    }
    //!< 曲线方向
    if (_cond["dir"].is<std::string>()) {
        const std::string dir_name = _cond["dir"];
        TopLoc_Location aLoc;
        Standard_Real first, last;
        Handle(Geom_Curve) curve = BRep_Tool::Curve(_edge, aLoc, first, last);
        if (curve->IsKind(STANDARD_TYPE(Geom_Line))) {
            Handle(Geom_Line) line = Handle(Geom_Line)::DownCast(curve);
            if (dir_name == "x") {
                if (!line->Position().Direction().IsEqual(gp::DX(), 1e-3)) { return false; }
            } else if (dir_name == "y") {
                if (!line->Position().Direction().IsEqual(gp::DY(), 1e-3)) { return false; }
            } else if (dir_name == "z") {
                if (!line->Position().Direction().IsEqual(gp::DZ(), 1e-3)) { return false; }
            }
        }
    }
    //!< 曲线顶点最小值
    if (_cond["min"].is<sol::table>()) {
        std::vector<double> min_data;
        if (get_double_vector(_cond["min"], min_data) && min_data.size() == 3) {
            const gp_Pnt &sp = BRep_Tool::Pnt(TopExp::FirstVertex(_edge));
            const gp_Pnt &ep = BRep_Tool::Pnt(TopExp::LastVertex(_edge));
            if (sp.X() <= min_data[0] || ep.X() <= min_data[0]) { return false; }
            if (sp.Y() <= min_data[1] || ep.Y() <= min_data[1]) { return false; }
            if (sp.Z() <= min_data[2] || ep.Z() <= min_data[2]) { return false; }
        }
    }
    //!< 曲线顶点最大值
    if (_cond["max"].is<sol::table>()) {
        std::vector<double> max_data;
        if (get_double_vector(_cond["max"], max_data) && max_data.size() == 3) {
            const gp_Pnt &sp = BRep_Tool::Pnt(TopExp::FirstVertex(_edge));
            const gp_Pnt &ep = BRep_Tool::Pnt(TopExp::LastVertex(_edge));
            if (sp.X() >= max_data[0] || ep.X() >= max_data[0]) { return false; }
            if (sp.Y() >= max_data[1] || ep.Y() >= max_data[1]) { return false; }
            if (sp.Z() >= max_data[2] || ep.Z() >= max_data[2]) { return false; }
        }
    }
    return true;
}

bool JyShape::get_double_vector(const sol::table &_t, std::vector<double> &_v) {
    for (int i = 1; i <= _t.size(); ++i) {
        if (!_t[i].is<double>()) { return false; }
        _v.push_back(_t[i]);
    }
    return true;
}

gp_Pnt JyShape::get_point_3d(const sol::table &_t) {
    if (!_t[1].is<double>() || !_t[2].is<double>() || !_t[3].is<double>()) {
        throw std::invalid_argument("Invalid point data");
    }
    return {_t[1], _t[2], _t[3]};
}

gp_Dir JyShape::get_dir_3d(const sol::table &_t) {
    if (!_t[1].is<double>() || !_t[2].is<double>() || !_t[3].is<double>()) {
        throw std::invalid_argument("Invalid directory data");
    }
    return {_t[1], _t[2], _t[3]};
}

void JyShape::locate(const sol::table &_t) const {
    if (_t["pos"].is<sol::table>()) {
        sol::table pos3 = _t["pos"];
        std::vector<double> pos_data;
        if (get_double_vector(pos3, pos_data) && pos_data.size() == 3) {
            locate_base(LocateType::TRANSLATE_ALL, pos_data[0], pos_data[1], pos_data[2]);
        }
    }
    if (_t["rot"].is<sol::table>()) {
        sol::table rot3 = _t["rot"];
        std::vector<double> rot_data;
        if (get_double_vector(rot3, rot_data) && rot_data.size() == 3) {
            locate_base(LocateType::ROTATE_ALL, rot_data[0], rot_data[1], rot_data[2]);
        }
    }
    if (_t["x"].is<double>()) { locate_base(LocateType::TRANSLATE_X, _t["x"], 0, 0); }
    if (_t["y"].is<double>()) { locate_base(LocateType::TRANSLATE_Y, 0, _t["y"], 0); }
    if (_t["z"].is<double>()) { locate_base(LocateType::TRANSLATE_Z, 0, 0, _t["z"]); }
    if (_t["rx"].is<double>()) { locate_base(LocateType::ROTATE_X, _t["rx"], 0, 0); }
    if (_t["ry"].is<double>()) { locate_base(LocateType::ROTATE_Y, 0, _t["ry"], 0); }
    if (_t["rz"].is<double>()) { locate_base(LocateType::ROTATE_Z, 0, 0, _t["rz"]); }
}

void JyShape::locate_base(const LocateType &_type, const double &_x, const double &_y, const double &_z,
                          const bool &_abs) const {
    TopoDS_Shape topology = s_->Shape();
    gp_Trsf transformation = _abs ? topology.Location().Transformation() : gp_Trsf();
    gp_XYZ pos;
    transformation.Transforms(pos);
    gp_Quaternion quaternion = transformation.GetRotation();
    double deg_rx, deg_ry, deg_rz;
    quaternion.GetEulerAngles(gp_YawPitchRoll, deg_rz, deg_ry, deg_rx);
    switch (_type) {
        case LocateType::TRANSLATE_ALL:
            transformation.SetTranslationPart(gp_XYZ(_x, _y, _z));
            break;
        case LocateType::TRANSLATE_X:
            pos.SetX(_x);
            transformation.SetTranslationPart(pos);
            break;
        case LocateType::TRANSLATE_Y:
            pos.SetY(_y);
            transformation.SetTranslationPart(pos);
            break;
        case LocateType::TRANSLATE_Z:
            pos.SetZ(_z);
            transformation.SetTranslationPart(pos);
            break;
        case LocateType::ROTATE_ALL:
            deg_rx = _x * M_PI / 180.0;
            deg_ry = _y * M_PI / 180.0;
            deg_rz = _z * M_PI / 180.0;
            quaternion.SetEulerAngles(gp_YawPitchRoll, deg_rz, deg_ry, deg_rx);
            transformation.SetRotationPart(quaternion);
            break;
        case LocateType::ROTATE_X:
            deg_rx = _x * M_PI / 180.0;
            quaternion.SetEulerAngles(gp_YawPitchRoll, deg_rz, deg_ry, deg_rx);
            transformation.SetRotationPart(quaternion);
            break;
        case LocateType::ROTATE_Y:
            deg_ry = _y * M_PI / 180.0;
            quaternion.SetEulerAngles(gp_YawPitchRoll, deg_rz, deg_ry, deg_rx);
            transformation.SetRotationPart(quaternion);
            break;
        case LocateType::ROTATE_Z:
            deg_rz = _z * M_PI / 180.0;
            quaternion.SetEulerAngles(gp_YawPitchRoll, deg_rz, deg_ry, deg_rx);
            transformation.SetRotationPart(quaternion);
            break;

    }
    if (_abs) {
        TopLoc_Location location(transformation);
        s_->SetShape(topology.Located(location));
    } else {
        TopLoc_Location location(transformation);
        s_->SetShape(topology.Moved(location));
    }
}

void JyShape::prism(const double &_x, const double &_y, const double &_z) const {
    gp_Vec prism_dir{_x, _y, _z};
    if (!s_) { return; }
    TopoDS_Shape result = BRepPrimAPI_MakePrism(s_->Shape(), prism_dir);
    s_->SetShape(result);
}

void JyShape::translate(const double &_x, const double &_y, const double &_z) const {
    locate_base(LocateType::TRANSLATE_ALL, _x, _y, _z, false);
}

void JyShape::rotate(const double &_rx, const double &_ry, const double &_rz) const {
    locate_base(LocateType::ROTATE_ALL, _rx, _ry, _rz, false);
}

void JyShape::color(const std::string &_name_or_hex) const {
    if (!s_) { return; }
    Quantity_Color target_color;
    if (Quantity_Color::ColorFromHex(_name_or_hex.c_str(), target_color)) {
        s_->SetColor(target_color);
    } else if (Quantity_Color::ColorFromName(_name_or_hex.c_str(), target_color)) {
        s_->SetColor(target_color);
    } else {}
}

void JyShape::transparency(const double &_value) const {
    if ((_value < 0) || (_value > 1)) { throw std::invalid_argument("Invalid transparency value![0,1]"); }
    s_->SetTransparency(_value);
}

void JyShape::set_stl_radian(const sol::table &_opt) const {
    if (_opt["radian"].is<double>()) {
        s_->SetAngleAndDeviation(_opt["radian"]);
    }
}

void JyShape::export_stl(const std::string &_filename, const sol::table &_opt) const {
    Standard_Boolean theAsciiMode = Standard_True;
    if (_opt && _opt["type"].is<std::string>()) {
        const std::string type_name = _opt["type"];
        if (type_name == "ascii") {
            theAsciiMode = Standard_True;
        } else if (type_name == "binary") {
            theAsciiMode = Standard_False;
        } else {}
    }
    if (StlAPI::Write(s_->Shape(), _filename.c_str(), theAsciiMode)) { return; } // 成功
    throw std::runtime_error("Failed to export stl!");
}

void JyShape::export_step(const std::string &_filename) const {
    STEPControl_Writer writer;
    writer.Transfer(s_->Shape(), STEPControl_AsIs);
    if (writer.Write(_filename.c_str())) { return; } // 成功
    throw std::runtime_error("Failed to export STEP!");
}

void JyShape::export_iges(const std::string &_filename) const {
    IGESControl_Writer writer;
    if (!writer.AddShape(s_->Shape())) {
        throw std::runtime_error("Failed to add shape!");
    }
    if (writer.Write(_filename.c_str())) { return; }    // 成功
    throw std::runtime_error("Failed to export IGES!");
}

JyShapeBox::JyShapeBox(const double &_x, const double &_y, const double &_z, const sol::table &_opt) {
    BRepPrimAPI_MakeBox make_box(_x, _y, _z);
    if (make_box.Wedge().IsDegeneratedShape()) { throw std::runtime_error("Is Degenerated Shape!"); }
    s_ = new AIS_Shape(make_box);
    process_opt(_opt);
}

JyCylinder::JyCylinder(const double &_r, const double &_h, const sol::table &_opt) {
    BRepPrimAPI_MakeCylinder make_cylinder(_r, _h);
    s_ = new AIS_Shape(make_cylinder);
    process_opt(_opt);
}

JyCone::JyCone(const double &R1, const double &R2, const double &H, const sol::table &_opt) {
    BRepPrimAPI_MakeCone make_cone(R1, R2, H);
    s_ = new AIS_Shape(make_cone);
    process_opt(_opt);
}

JySphere::JySphere(const double &_r, const sol::table &_opt) {
    BRepPrimAPI_MakeSphere make_sphere(_r);
    s_ = new AIS_Shape(make_sphere);
    process_opt(_opt);
}

JyEdge::JyEdge(const std::string &_type, const sol::table &_vec1, const sol::table &_vec2,
               const double &_r1, const double &_r2) {
    if (_type == "lin") {
        const auto p1 = get_point_3d(_vec1);
        const auto p2 = get_point_3d(_vec2);
        BRepBuilderAPI_MakeEdge make_edge_lin(p1, p2);
        s_ = new AIS_Shape(make_edge_lin);
    } else if (_type == "circ") {
        const auto pos = get_point_3d(_vec1);
        const auto dir = get_dir_3d(_vec2);
        gp_Circ circ({pos, dir}, _r1);
        BRepBuilderAPI_MakeEdge make_edge_circ(circ);
        s_ = new AIS_Shape(make_edge_circ);
    } else if (_type == "elips") {
        const auto pos = get_point_3d(_vec1);
        const auto dir = get_dir_3d(_vec2);
        gp_Elips elips({pos, dir}, _r1, _r2);
        BRepBuilderAPI_MakeEdge make_edge_elips(elips);
        s_ = new AIS_Shape(make_edge_elips);
    } else if (_type == "hypr") {
        const auto pos = get_point_3d(_vec1);
        const auto dir = get_dir_3d(_vec2);
        gp_Hypr hypr({pos, dir}, _r1, _r2);
        BRepBuilderAPI_MakeEdge make_edge_hypr(hypr);
        s_ = new AIS_Shape(make_edge_hypr);
    } else if (_type == "parab") {
        const auto pos = get_point_3d(_vec1);
        const auto dir = get_dir_3d(_vec2);
        gp_Parab parab({pos, dir}, _r1);
        BRepBuilderAPI_MakeEdge make_edge_parab(parab);
        s_ = new AIS_Shape(make_edge_parab);
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
            if (!shape.s_) { continue; }
            const auto edge = TopoDS::Edge(shape.s_->Shape());
            make_wire.Add(edge);
        } else if (p.second.is<JyWire>()) {
            const auto shape = p.second.as<JyWire>();
            if (!shape.s_) { continue; }
            const auto wire = TopoDS::Wire(shape.s_->Shape());
            make_wire.Add(wire);
        } else {}
    }
    if (!make_wire.IsDone()) { return; }
    s_ = new AIS_Shape(make_wire);
}

JyPolygon::JyPolygon(const sol::table &_param) {
    BRepBuilderAPI_MakePolygon make_polygon;
    for (const auto &p: _param) {
        if (!p.first.is<int>()) { continue; }
        if (!p.second.is<sol::table>()) { continue; }
        sol::table point = p.second.as<sol::table>();
        if (point.size() != 3) { continue; }
        double x{0}, y{0}, z{0};
        if (point[1].is<double>()) { x = point[1]; }
        if (point[2].is<double>()) { y = point[2]; }
        if (point[3].is<double>()) { z = point[3]; }
        make_polygon.Add(gp_Pnt(x, y, z));
    }
    if (!make_polygon.Added()) { throw std::runtime_error("Polygon Last Vertex NOT Added!"); }
    make_polygon.Close();
    s_ = new AIS_Shape(make_polygon);
}

JyFace::JyFace(const JyShape &_shape) {
    const auto shape_type = _shape.s_->Shape().ShapeType();
    if (shape_type == TopAbs_WIRE) {
        const auto wire = TopoDS::Wire(_shape.s_->Shape());
        BRepBuilderAPI_MakeFace make_face(wire);
        s_ = new AIS_Shape(make_face);
    } else if (shape_type == TopAbs_EDGE) {
        const auto edge = TopoDS::Edge(_shape.s_->Shape());
        BRepBuilderAPI_MakeWire make_wire;
        make_wire.Add(edge);
        if (!make_wire.IsDone()) { throw std::runtime_error("Failed make wire!"); }
        BRepBuilderAPI_MakeFace make_face(make_wire);
        s_ = new AIS_Shape(make_face);
    } else {
        throw std::runtime_error("Face: Not Support Shape Type!");
    }
}