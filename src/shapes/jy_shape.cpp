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
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <BRepFilletAPI_MakeChamfer.hxx>
#include <BRepFilletAPI_MakeFillet.hxx>
#include <BRepGProp.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <BRepOffsetAPI_MakePipe.hxx>
#include <BRepOffsetAPI_MakeThickSolid.hxx>
#include <BRepPrimAPI_MakePrism.hxx>
#include <BRepPrimAPI_MakeRevol.hxx>
#include <BRepTools.hxx>
#include <BRep_Tool.hxx>
#include <GCPnts_AbscissaPoint.hxx>
#include <GProp_GProps.hxx>
#include <IGESControl_Reader.hxx>
#include <IGESControl_Writer.hxx>
#include <STEPControl_Reader.hxx>
#include <STEPControl_Writer.hxx>
#include <StlAPI.hxx>
#include <StlAPI_Reader.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <gp_Quaternion.hxx>

sol::usertype<JyShape> JyShape::configure_usertype(sol::state &lua) {
    auto shape_user = lua.new_usertype<JyShape>("shape", sol::constructors<JyShape(),
                                                                           JyShape(const std::string &)>());
    shape_user["copy"] = [](const JyShape &self) { return JyShape(self); };
    shape_user["type"] = &JyShape::type;
    shape_user["empty"] = &JyShape::empty;
    shape_user["get_edge"] = &JyShape::get_edge;
    shape_user["get_face"] = &JyShape::get_face;
    // 布尔运算
    shape_user["fuse"] = &JyShape::fuse;
    shape_user["cut"] = &JyShape::cut;
    shape_user["common"] = &JyShape::common;
    // 几何变换
    shape_user["fillet"] = sol::overload(
            static_cast<JyShape &(JyShape::*) (const double &)>(&JyShape::fillet),
            static_cast<JyShape &(JyShape::*) (const double &, const JyShape &)>(&JyShape::fillet),
            static_cast<JyShape &(JyShape::*) (const double &, const sol::table &)>(&JyShape::fillet));
    shape_user["chamfer"] = sol::overload(
            static_cast<JyShape &(JyShape::*) (const double &)>(&JyShape::chamfer),
            static_cast<JyShape &(JyShape::*) (const double &, const sol::table &)>(&JyShape::chamfer));
    shape_user["prism"] = &JyShape::prism;
    shape_user["revol"] = &JyShape::revol;
    shape_user["pipe"] = &JyShape::pipe;
    shape_user["thick"] = &JyShape::thick;
    shape_user["scale"] = &JyShape::scale;
    shape_user["mirror"] = &JyShape::mirror;
    // 位置姿态调整
    shape_user["x"] = &JyShape::x;
    shape_user["y"] = &JyShape::y;
    shape_user["z"] = &JyShape::z;
    shape_user["rx"] = &JyShape::rx;
    shape_user["ry"] = &JyShape::ry;
    shape_user["rz"] = &JyShape::rz;
    shape_user["pos"] = &JyShape::pos;
    shape_user["rot"] = &JyShape::rot;
    shape_user["move"] = sol::overload(
            static_cast<JyShape &(JyShape::*) (const std::string &, const double &, const double &, const double &)>(&JyShape::move),
            static_cast<JyShape &(JyShape::*) (const std::string &, const double &)>(&JyShape::move));
    shape_user["zero"] = &JyShape::zero;
    shape_user["locate"] = sol::overload(
            static_cast<JyShape &(JyShape::*) (const JyShape &)>(&JyShape::locate),
            static_cast<JyShape &(JyShape::*) (const double &, const double &, const double &, const double &, const double &, const double &)>(&JyShape::locate));
    // 属性设置
    shape_user["color"] = &JyShape::color;
    shape_user["transparency"] = &JyShape::transparency;
    shape_user["mass"] = &JyShape::mass;
    shape_user["export_step"] = &JyShape::export_step;
    shape_user["export_iges"] = &JyShape::export_iges;
    const auto overload_export_stl = sol::overload(
            static_cast<JyShape &(JyShape::*) (const std::string &_filename)>(&JyShape::export_stl),
            static_cast<JyShape &(JyShape::*) (const std::string &_filename, const sol::table &_opt)>(&JyShape::export_stl));
    shape_user["export_stl"] = overload_export_stl;
    return shape_user;
}

static bool checkSuffix(const std::string &filename, const std::string &suffix) {
    // 查找最后一个点的位置
    size_t dotPos = filename.find_last_of('.');
    // 如果没有找到点，或者点在开头（隐藏文件），返回空字符串
    if (dotPos == std::string::npos || dotPos == 0) { return ""; }
    std::string extension = filename.substr(dotPos + 1);// 从点的下一个位置开始提取（不包含点）
    // 转换为小写
    std::transform(extension.begin(), extension.end(), extension.begin(), [](unsigned char c) { return std::tolower(c); });
    return extension == suffix;
}

JyShape::JyShape(const std::string &_filename) {
    if (_filename.empty()) { throw std::runtime_error("Filename is empty!"); }
    if (checkSuffix(_filename, "step") || checkSuffix(_filename, "stp")) {
        STEPControl_Reader reader;
        const auto &res = reader.ReadFile(_filename.c_str());// 加载文件只是记忆数据，不转换
        if (res != IFSelect_RetDone) { throw std::runtime_error("Failed Import STEP file!"); }
        reader.PrintCheckLoad(Standard_False, IFSelect_ItemsByEntity);// 检查加载的文件(不是强制性)
        //加载step文件
        Standard_Integer NbRoots = reader.NbRootsForTransfer();
        Standard_Integer num = reader.TransferRoots();
        s_ = reader.OneShape();
    } else if (checkSuffix(_filename, "iges") || checkSuffix(_filename, "igs")) {
        IGESControl_Reader reader;
        const auto &res = reader.ReadFile(_filename.c_str());// 加载文件
        if (res != IFSelect_RetDone) { throw std::runtime_error("Failed Import IGES file!"); }
        reader.PrintCheckLoad(Standard_False, IFSelect_ItemsByEntity);// 检查加载的文件(不是强制性)
        //加载iges文件
        Standard_Integer NbRoots = reader.NbRootsForTransfer();
        Standard_Integer num = reader.TransferRoots();
        s_ = reader.OneShape();
    } else if (checkSuffix(_filename, "stl")) {
        TopoDS_Shape topology_shape;
        StlAPI_Reader reader;
        if (!reader.Read(topology_shape, _filename.c_str())) { throw std::runtime_error("Failed Import STL file!"); }
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


std::array<double, 6> JyShape::get_pose() const {
    gp_Trsf trsf = s_.Location().Transformation();
    gp_XYZ translation = trsf.TranslationPart();
    gp_Quaternion quat = trsf.GetRotation();
    double rz, ry, rx;// yaw, pitch, roll
    quat.GetEulerAngles(gp_YawPitchRoll, rz, ry, rx);
    double roll = rx * 180.0 / M_PI;
    double pitch = ry * 180.0 / M_PI;
    double yaw = rz * 180.0 / M_PI;
    return {translation.X(), translation.Y(), translation.Z(), roll, pitch, yaw};
}

JyShape JyShape::get_edge(const sol::table &_cond) const {
    TopoDS_Edge edge;
    for (TopExp_Explorer ex(s_, TopAbs_EDGE); ex.More(); ex.Next()) {
        TopoDS_Edge edge = TopoDS::Edge(ex.Current());
        if (!_cond || edge_filter(edge, _cond)) { return JyShape(edge); }
    }
    throw std::runtime_error("No edge found!");
}

JyShape JyShape::get_face(std::string type, double area, std::array<double, 3> center, std::array<double, 4> uv) const {
    const auto epsilon = 1e-3;
    for (TopExp_Explorer ex(s_, TopAbs_FACE); ex.More(); ex.Next()) {
        const auto current_face = JyShape::face_properties(JyShape(ex.Current()));
        const auto area_match = std::abs(current_face.area - area) < epsilon;
        const auto center_match = std::equal(current_face.center.begin(), current_face.center.end(), center.begin(),
                                             [epsilon](double x, double y) { return std::abs(x - y) < epsilon; });
        const auto uv_match = std::equal(current_face.uv.begin(), current_face.uv.end(), uv.begin(),
                                         [epsilon](double x, double y) { return std::abs(x - y) < epsilon; });
        if (current_face.type == type && area_match && center_match && uv_match) {
            return JyShape(ex.Current());
        }
    }
    throw std::runtime_error("No face found!");
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


JyShape &JyShape::fillet(const double &_r, const JyShape &edge_shape) {
    /*
    b = box.new()
    edge_info = { type = 'line', first = { 0, 0, 1 }, last = { 1, 0, 1 }, tol = 1e-3 }
    b:fillet(0.2, b:get_edge(edge_info)):show()
    */
    BRepFilletAPI_MakeFillet MF(s_);
    TopoDS_Edge edge = TopoDS::Edge(edge_shape.s_);
    MF.Add(_r, edge);
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


JyShape &JyShape::pipe(const JyShape &wire) {
    // 示例：edge.new('circ', { 0, 0, 0 }, { 0, 0, 1 }, 1):pipe(line.new({ 0, 0, 0 }, { 5, 5, 5 })):show()
    if (s_.IsNull() || wire.s_.IsNull()) { throw std::runtime_error("JyShape::pipe: shape or wire is null!"); }
    if (s_.ShapeType() <= TopAbs_SOLID) { throw std::runtime_error("DomainError :profile is a solid or composite solid."); }
    if (wire.s_.ShapeType() == TopAbs_EDGE) {
        BRepBuilderAPI_MakeWire wire_builder(TopoDS::Edge(wire.s_));
        BRepOffsetAPI_MakePipe make_pipe(wire_builder.Wire(), s_);
        s_ = make_pipe.Shape();
        return *this;
    } else if (wire.s_.ShapeType() != TopAbs_WIRE) {
        throw std::runtime_error("DomainError :wire must be a wire or edge.");
    }
    BRepOffsetAPI_MakePipe make_pipe(TopoDS::Wire(wire.s_), s_);
    s_ = make_pipe.Shape();
    return *this;
}


JyShape &JyShape::thick(const JyShape &face, const double &offset) {
    /* 示例：抽壳
    b = box.new()
    f = b:get_face('plane', 1, { 0.5, 0.5, 1 }, { 0, 1, 0, 1 })
    b:thick(f, -0.1):show()
    */
    if (s_.IsNull() || face.s_.IsNull()) { throw std::runtime_error("thick: shape or face is null!"); }
    if (s_.ShapeType() > TopAbs_SOLID) { throw std::runtime_error("DomainError :profile is a solid or composite solid."); }
    if (face.s_.ShapeType() != TopAbs_FACE) { throw std::runtime_error("DomainError :face must be a face."); }
    BRepOffsetAPI_MakeThickSolid make_thick;// 基于偏置创建实体（抽壳或加厚）
    TopTools_ListOfShape faces;
    faces.Append(TopoDS::Face(face.s_));
    make_thick.MakeThickSolidByJoin(s_, faces, offset, 1.e-3);
    s_ = make_thick.Shape();
    return *this;
}

JyShape &JyShape::scale(const double &factor) {
    gp_Trsf transformation;
    transformation.SetScale(gp_Pnt(0, 0, 0), factor);
    BRepBuilderAPI_Transform transformer(s_, transformation);
    s_ = transformer.Shape();
    return *this;
}

JyShape &JyShape::mirror(const std::array<double, 3> _pos, const std::array<double, 3> _dir) {
    // 示例：box.new():mirror({ 0, 0, 0 }, { 0, 0, 1 }):show()
    if (s_.IsNull()) { return *this; }
    gp_Trsf transformation;
    gp_Ax1 xAxis = gp_Ax1(gp_Pnt(_pos[0], _pos[1], _pos[2]), gp_Dir(_dir[0], _dir[1], _dir[2]));
    transformation.SetMirror(xAxis);
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

JyShape::InertialProperties JyShape::inertial(const JyShape &_shape) {
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

JyShape::InertialProperties JyShape::inertial(const std::vector<JyShape> &_shapes) {
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
std::array<double, 3> JyShape::vertex_properties(const JyShape &_shape) {
    const auto vertex = TopoDS::Vertex(_shape.s_);
    gp_Pnt pnt = BRep_Tool::Pnt(vertex);
    return {pnt.X(), pnt.Y(), pnt.Z()};
}

JyShape::EdgeProperties JyShape::edge_properties(const JyShape &_shape) {
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
    const auto edge = TopoDS::Edge(_shape.s_);
    // 获取参数范围
    Standard_Real first, last;
    Handle(Geom_Curve) curve = BRep_Tool::Curve(edge, first, last);
    if (!curve.IsNull()) {
        // 获取曲线类型
        GeomAdaptor_Curve adaptor(curve);
        GeomAbs_CurveType curveType = adaptor.GetType();
        // 计算边的长度
        BRepAdaptor_Curve brep_adaptor(edge);
        Standard_Real length = GCPnts_AbscissaPoint::Length(brep_adaptor);
        // 获取起点和终点
        gp_Pnt firstPnt = adaptor.Value(first);
        gp_Pnt lastPnt = adaptor.Value(last);
        return {type_map.at(curveType), length, {firstPnt.X(), firstPnt.Y(), firstPnt.Z()}, {lastPnt.X(), lastPnt.Y(), lastPnt.Z()}};
    }
    return EdgeProperties{};
}

JyShape::FaceProperties JyShape::face_properties(const JyShape &_shape) {
    static const std::unordered_map<int, std::string> type_map = {
            {GeomAbs_Plane, "plane"},
            {GeomAbs_Cylinder, "cylinder"},
            {GeomAbs_Cone, "cone"},
            {GeomAbs_Sphere, "sphere"},
            {GeomAbs_Torus, "torus"},
            {GeomAbs_BezierSurface, "bezier_surface"},
            {GeomAbs_BSplineSurface, "bspline_surface"},
            {GeomAbs_SurfaceOfRevolution, "surface_of_revolution"},
            {GeomAbs_SurfaceOfExtrusion, "surface_of_extrusion"},
            {GeomAbs_OffsetSurface, "offset_surface"},
            {GeomAbs_OtherSurface, "other_surface"}};
    const auto face = TopoDS::Face(_shape.s_);
    // 获取曲面
    Handle(Geom_Surface) surface = BRep_Tool::Surface(face);
    if (!surface.IsNull()) {
        // 获取曲面类型
        GeomAdaptor_Surface adaptor(surface);
        GeomAbs_SurfaceType surfaceType = adaptor.GetType();
        // 获取参数范围
        Standard_Real uMin, uMax, vMin, vMax;
        BRepTools::UVBounds(face, uMin, uMax, vMin, vMax);
        // 计算面积和重心
        GProp_GProps props;
        BRepGProp::SurfaceProperties(face, props);
        Standard_Real area = props.Mass();
        gp_Pnt centerOfMass = props.CentreOfMass();
        return {
                type_map.at(surfaceType),
                area,
                {centerOfMass.X(), centerOfMass.Y(), centerOfMass.Z()},
                {uMin, uMax, vMin, vMax},
        };
    }
    return FaceProperties{};
}