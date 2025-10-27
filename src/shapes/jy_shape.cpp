/*
 * Copyright (c) 2024. Li Jianbin. All rights reserved.
 * MIT License
 */
#include "jy_shape.h"
#include <AIS_InteractiveContext.hxx>
#include <BRepAdaptor_Curve.hxx>
#include <BRepAlgoAPI_Common.hxx>
#include <BRepAlgoAPI_Cut.hxx>
#include <BRepAlgoAPI_Fuse.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakePolygon.hxx>
#include <BRepBuilderAPI_MakeVertex.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <BRepFilletAPI_MakeChamfer.hxx>
#include <BRepFilletAPI_MakeFillet.hxx>
#include <BRepGProp.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeCone.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRepPrimAPI_MakePrism.hxx>
#include <BRepPrimAPI_MakeRevol.hxx>
#include <BRepPrimAPI_MakeSphere.hxx>
#include <BRepPrimAPI_MakeTorus.hxx>
#include <BRepPrimAPI_MakeWedge.hxx>
#include <Font_BRepTextBuilder.hxx>
#include <GProp_GProps.hxx>
#include <Geom_Line.hxx>
#include <IGESControl_Writer.hxx>
#include <STEPControl_Reader.hxx>
#include <STEPControl_Writer.hxx>
#include <StlAPI.hxx>
#include <StlAPI_Reader.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <gp_Quaternion.hxx>

JyShape::JyShape(const std::string &_filename) {
    if (_filename.empty()) { throw std::runtime_error("Filename is empty!"); }
    const auto ends_with = [&_filename](const std::string &suffix) {
        if (suffix.length() > _filename.length()) { return false; }
        return (_filename.rfind(suffix) == (_filename.length() - suffix.length()));
    };
    if (ends_with(".step") || ends_with(".STEP")) {
        STEPControl_Reader reader;
        const auto &res = reader.ReadFile(_filename.c_str());// 加载文件只是记忆数据，不转换
        if (res != IFSelect_RetDone) { throw std::runtime_error("Failed Import STEP file!"); }
        reader.PrintCheckLoad(Standard_False, IFSelect_ItemsByEntity);// 检查加载的文件(不是强制性)
        //加载step文件
        Standard_Integer NbRoots = reader.NbRootsForTransfer();
        Standard_Integer num = reader.TransferRoots();
        s_ = reader.OneShape();
    } else if (ends_with(".stl") || ends_with(".STL")) {
        TopoDS_Shape topology_shape;
#if 1
        StlAPI_Reader reader;
        if (!reader.Read(topology_shape, _filename.c_str())) { throw std::runtime_error("Failed Import STL file!"); }
#else
        //!< DEPRECATED
        if (!StlAPI::Read(topology_shape, _filename.c_str())) { throw std::runtime_error("Failed Import STL file!"); }
#endif
        s_ = topology_shape;
    } else {
        throw std::runtime_error("Not support file!");
    }
}

std::string JyShape::type() const {
    const auto t = s_.ShapeType();
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

JyShape &JyShape::fuse(const JyShape &_other) {
    return algo<BRepAlgoAPI_Fuse>(_other);
}

JyShape &JyShape::cut(const JyShape &_other) {
    return algo<BRepAlgoAPI_Cut>(_other);
}

JyShape &JyShape::common(const JyShape &_other) {
    return algo<BRepAlgoAPI_Common>(_other);
}

JyShape &JyShape::fillet(const double &_r, const sol::table &_cond) {
    BRepFilletAPI_MakeFillet MF(s_);
    for (TopExp_Explorer ex(s_, TopAbs_EDGE); ex.More(); ex.Next()) {
        TopoDS_Edge edge = TopoDS::Edge(ex.Current());
        if (!_cond || edge_filter(edge, _cond)) { MF.Add(_r, TopoDS::Edge(ex.Current())); }
    }
    if (MF.NbContours() == 0) { return *this; }
    MF.Build();
    if (!MF.IsDone()) { return *this; }
    s_ = MF.Shape();
    return *this;
}

JyShape &JyShape::chamfer(const double &_dis, const sol::table &_cond) {
    BRepFilletAPI_MakeChamfer MC(s_);
    TopTools_IndexedDataMapOfShapeListOfShape M;
    TopExp::MapShapesAndAncestors(s_, TopAbs_EDGE, TopAbs_FACE, M);
    for (Standard_Integer i = 1; i < M.Extent(); i++) {
        TopoDS_Edge E = TopoDS::Edge(M.FindKey(i));
        TopoDS_Face F = TopoDS::Face(M.FindFromIndex(i).First());
        if (!_cond || edge_filter(E, _cond)) { MC.Add(_dis, _dis, E, F); }
    }
    if (MC.NbContours() == 0) { return *this; }
    MC.Build();
    if (!MC.IsDone()) { return *this; }
    s_ = MC.Shape();
    return *this;
}


bool JyShape::edge_filter(const TopoDS_Edge &_edge, const sol::table &_cond) {
    const double tol = _cond["tol"].get_or(1e-3);
    const std::array<double, 3> first = _cond["first"].get_or(std::array<double, 3>{0, 0, 0});
    const std::array<double, 3> last = _cond["last"].get_or(std::array<double, 3>{0, 0, 0});
    static const std::unordered_map<int, std::string> type_map = {
            {GeomAbs_Line, "line"},
            {GeomAbs_Circle, "circle"},
            {GeomAbs_Ellipse, "ellipse"},
            {GeomAbs_Hyperbola, "hyperbola"},
            {GeomAbs_Parabola, "parabola"},
            {GeomAbs_BezierCurve, "bezier_curve"},
            {GeomAbs_BSplineCurve, "bspline_curve"},
            {GeomAbs_OffsetCurve, "offset_curve"},
            {GeomAbs_OtherCurve, "other_curve"}};
    //!< 过滤曲线类型
    if (_cond["type"].is<std::string>()) {
        const std::string type_name = _cond["type"];
        BRepAdaptor_Curve C(_edge);
        auto it = type_map.find(C.GetType());
        if (it == type_map.end() || it->second != type_name) { return false; }
    }
    const gp_Pnt &sp = BRep_Tool::Pnt(TopExp::FirstVertex(_edge));
    const gp_Pnt &ep = BRep_Tool::Pnt(TopExp::LastVertex(_edge));
    // 起点坐标
    if (_cond["first"].is<sol::table>()) {
        if (std::abs(sp.X() - first[0]) > tol || std::abs(sp.Y() - first[1]) > tol || std::abs(sp.Z() - first[2]) > tol) { return false; }
    }
    if (_cond["last"].is<sol::table>()) {
        if (std::abs(ep.X() - last[0]) > tol || std::abs(ep.Y() - last[1]) > tol || std::abs(ep.Z() - last[2]) > tol) { return false; }
    }
    //!< 曲线顶点最小值
    if (_cond["min"].is<sol::table>()) {
        std::vector<double> min_data;
        if (get_double_vector(_cond["min"], min_data) && min_data.size() == 3) {
            if (sp.X() <= min_data[0] || ep.X() <= min_data[0]) { return false; }
            if (sp.Y() <= min_data[1] || ep.Y() <= min_data[1]) { return false; }
            if (sp.Z() <= min_data[2] || ep.Z() <= min_data[2]) { return false; }
        }
    }
    //!< 曲线顶点最大值
    if (_cond["max"].is<sol::table>()) {
        std::vector<double> max_data;
        if (get_double_vector(_cond["max"], max_data) && max_data.size() == 3) {
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

JyShape &JyShape::locate(const double &x, const double &y, const double &z, const double &rx, const double &ry, const double &rz) {
    gp_Trsf transformation;
    transformation.SetTranslationPart(gp_XYZ(x, y, z));
    double deg_rx = rx * M_PI / 180.0;
    double deg_ry = ry * M_PI / 180.0;
    double deg_rz = rz * M_PI / 180.0;
    gp_Quaternion quaternion(gp_YawPitchRoll, deg_rz, deg_ry, deg_rx);
    transformation.SetRotationPart(quaternion);
    s_.Location(transformation);
    return *this;
}

JyShape &JyShape::locate(const JyShape &_base) {
    s_.Location(_base.s_.Location());
    return *this;
}

JyShape &JyShape::move(const std::string &_move_type, const double &_x, const double &_y, const double &_z) {
    if (_move_type == "pos") {
        return locate_base(LocateType::TRANSLATE_ALL, _x, _y, _z, false);
    } else if (_move_type == "rot") {
        return locate_base(LocateType::ROTATE_ALL, _x, _y, _z, false);
    } else {
        throw std::invalid_argument("move_type must be 'pos' or 'rot'");
    }
    return *this;
}
JyShape &JyShape::move(const std::string &_move_type, const double &value) {
    if (_move_type == "x") {
        return locate_base(LocateType::TRANSLATE_X, value, 0, 0, false);
    } else if (_move_type == "y") {
        return locate_base(LocateType::TRANSLATE_Y, 0, value, 0, false);
    } else if (_move_type == "z") {
        return locate_base(LocateType::TRANSLATE_Z, 0, 0, value, false);
    } else if (_move_type == "rx") {
        return locate_base(LocateType::ROTATE_X, value, 0, 0, false);
    } else if (_move_type == "ry") {
        return locate_base(LocateType::ROTATE_Y, 0, value, 0, false);
    } else if (_move_type == "rz") {
        return locate_base(LocateType::ROTATE_Z, 0, 0, value, false);
    } else {
        throw std::invalid_argument("move_type must be 'x', 'y', 'z', 'rx', 'ry' or 'rz'");
    }
}

JyShape &JyShape::zero() {
    return locate(0, 0, 0, 0, 0, 0);
}

JyShape &JyShape::locate_base(const LocateType &_type, const double &_x, const double &_y, const double &_z, const bool &_abs) {
    gp_Trsf transformation = _abs ? s_.Location().Transformation() : gp_Trsf();
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
        const TopLoc_Location location(transformation);
        s_.Location(location);
    } else {
        const TopLoc_Location location(transformation);
        s_.Move(location);
    }
    return *this;
    // s_->Redisplay();
    // if (s_->InteractiveContext()) { s_->InteractiveContext()->Update(s_, true); }
}

JyShape &JyShape::prism(const double &_x, const double &_y, const double &_z) {
    gp_Vec prism_dir{_x, _y, _z};
    if (s_.IsNull()) { return *this; }
    TopoDS_Shape result = BRepPrimAPI_MakePrism(s_, prism_dir);
    s_ = result;
    return *this;
}

JyShape &JyShape::revol(const std::array<double, 3> _pos, const std::array<double, 3> _dir, const double &_angle) {
    if (s_.IsNull()) { return *this; }
    gp_Ax1 axis{gp_Pnt(_pos[0], _pos[1], _pos[2]), gp_Dir(_dir[0], _dir[1], _dir[2])};
    TopoDS_Shape result = BRepPrimAPI_MakeRevol(s_, axis, _angle * M_PI / 180.0);
    s_ = result;
    return *this;
}

JyShape &JyShape::scale(const double &factor) {
    gp_Trsf transformation;
    transformation.SetScale(gp_Pnt(0, 0, 0), factor);
    BRepBuilderAPI_Transform transformer(s_, transformation);
    s_ = transformer.Shape();
    return *this;
}

JyShape &JyShape::color(const std::string &_name_or_hex) {
    if (s_.IsNull()) { return *this; }
    Quantity_Color target_color;
    if (Quantity_Color::ColorFromHex(_name_or_hex.c_str(), target_color)) {
        color_ = target_color;
    } else if (Quantity_Color::ColorFromName(_name_or_hex.c_str(), target_color)) {
        color_ = target_color;
    } else {
    }
    // if (s_->InteractiveContext()) { s_->InteractiveContext()->Update(s_, true); }
    return *this;
}

JyShape &JyShape::transparency(const double &_value) {
    if ((_value < 0) || (_value > 1)) { throw std::invalid_argument("Invalid transparency value![0,1]"); }
    transparency_ = _value;
    return *this;
}


JyShape &JyShape::mass(const double &_mass) {
    if (_mass <= 0) { throw std::invalid_argument("Invalid mass value!"); }
    if (s_.IsNull()) { return *this; }
    GProp_GProps system;
    BRepGProp::VolumeProperties(s_, system);
    const double current_mass = system.Mass();
    if (current_mass <= 0) { throw std::runtime_error("Current mass is zero!"); }
    density_ = _mass / current_mass;
    return *this;
}

JyShape &JyShape::export_stl(const std::string &_filename, const sol::table &_opt) {
    Standard_Boolean theAsciiMode = Standard_True;
    if (_opt && _opt["type"].is<std::string>()) {
        const std::string type_name = _opt["type"];
        if (type_name == "ascii") {
            theAsciiMode = Standard_True;
        } else if (type_name == "binary") {
            theAsciiMode = Standard_False;
        } else {
            throw std::runtime_error("Invalid type!");
        }
    }
    const double theLinDeflection = (_opt && _opt["radian"].is<double>()) ? _opt["radian"] : 0.1;
    return export_stl_common(_filename, theAsciiMode, theLinDeflection);
}

JyShape &JyShape::export_stl_common(const std::string &_filename, const bool is_ascii, const double &lin) {
    BRepMesh_IncrementalMesh aMesh(s_, lin);
    if (!StlAPI::Write(s_, _filename.c_str(), is_ascii)) { throw std::runtime_error("Failed to export stl!"); }
    return *this;
}

JyShape &JyShape::export_step(const std::string &_filename) {
    STEPControl_Writer writer;
    writer.Transfer(s_, STEPControl_AsIs);
    if (!writer.Write(_filename.c_str())) { throw std::runtime_error("Failed to export STEP!"); }
    return *this;
}

JyShape &JyShape::export_iges(const std::string &_filename) {
    IGESControl_Writer writer;
    if (!writer.AddShape(s_)) {
        throw std::runtime_error("Failed to add shape!");
    }
    if (!writer.Write(_filename.c_str())) { throw std::runtime_error("Failed to export IGES!"); }
    return *this;
}

std::array<double, 4> JyShape::rgba() const {
    return {color_.Red(), color_.Green(), color_.Blue(), (1.0 - transparency_)};
}

JyShape JyShape::make_compound(const std::vector<JyShape> &_shapes) {
    if (_shapes.empty()) { return JyShape(); }
    BRep_Builder builder;
    TopoDS_Compound compound;
    builder.MakeCompound(compound);
    bool is_all_null = true;
    for (const auto &shape: _shapes) {
        if (!shape.s_.IsNull()) {
            builder.Add(compound, shape.s_);
            is_all_null = false;
        }
    }
    if (is_all_null) { return JyShape(); }
    JyShape compound_shape = _shapes[0];
    compound_shape.s_ = compound;
    return compound_shape;
}

InertialProperties JyShape::inertial(const JyShape &_shape) {
    GProp_GProps props;
    BRepGProp::VolumeProperties(_shape.s_, props);
    gp_Pnt centerOfMass = props.CentreOfMass();
    gp_Mat inertiaMatrix = props.MatrixOfInertia();
    Standard_Real Ixx, Iyy, Izz, Ixy, Ixz, Iyz;
    Ixx = inertiaMatrix.Value(1, 1);
    Iyy = inertiaMatrix.Value(2, 2);
    Izz = inertiaMatrix.Value(3, 3);
    Ixy = inertiaMatrix.Value(1, 2);
    Ixz = inertiaMatrix.Value(1, 3);
    Iyz = inertiaMatrix.Value(2, 3);
    return {
            props.Mass(),
            {centerOfMass.X(), centerOfMass.Y(), centerOfMass.Z()},
            {Ixx, Iyy, Izz, Ixy, Ixz, Iyz}};
}

InertialProperties JyShape::inertial(const std::vector<JyShape> &_shapes) {
    GProp_GProps system;
    for (const auto &shape: _shapes) {
        if (shape.s_.IsNull()) { continue; }
        GProp_GProps one_system;
        BRepGProp::VolumeProperties(shape.s_, one_system);
        system.Add(one_system, shape.density_);
    }
    gp_Pnt centerOfMass = system.CentreOfMass();
    gp_Mat inertiaMatrix = system.MatrixOfInertia();
    Standard_Real Ixx, Iyy, Izz, Ixy, Ixz, Iyz;
    Ixx = inertiaMatrix.Value(1, 1);
    Iyy = inertiaMatrix.Value(2, 2);
    Izz = inertiaMatrix.Value(3, 3);
    Ixy = inertiaMatrix.Value(1, 2);
    Ixz = inertiaMatrix.Value(1, 3);
    Iyz = inertiaMatrix.Value(2, 3);
    return {
            system.Mass(),
            {centerOfMass.X(), centerOfMass.Y(), centerOfMass.Z()},
            {Ixx, Iyy, Izz, Ixy, Ixz, Iyz}};
}

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
