/*
 * Copyright (c) 2024. Li Jianbin. All rights reserved.
 * MIT License
 */
#include "jy_lua_virtual_machine.h"
#include <QElapsedTimer>

JyLuaVirtualMachine::JyLuaVirtualMachine() {
    lua.open_libraries();
    current_path_ = lua["package"]["path"].get<std::string>();
    lua["print"] = [=](const sol::object &v) { this->lua_print(v); };

    auto shape_user = lua.new_usertype<JyShape>("shape", sol::constructors<JyShape(const std::string &)>());
    shape_user["copy"] = [](const JyShape &self) { return JyShape(self); };
    shape_user["type"] = &JyShape::type;
    // 布尔运算
    shape_user["fuse"] = &JyShape::fuse;
    shape_user["cut"] = &JyShape::cut;
    shape_user["common"] = &JyShape::common;
    // 几何变换
    shape_user["fillet"] = &JyShape::fillet;
    shape_user["chamfer"] = &JyShape::chamfer;
    shape_user["prism"] = &JyShape::prism;
    shape_user["revol"] = &JyShape::revol;
    // 位置姿态调整
    shape_user["x"] = &JyShape::x;
    shape_user["y"] = &JyShape::y;
    shape_user["z"] = &JyShape::z;
    shape_user["rx"] = &JyShape::rx;
    shape_user["ry"] = &JyShape::ry;
    shape_user["rz"] = &JyShape::rz;
    shape_user["pos"] = &JyShape::pos;
    shape_user["rot"] = &JyShape::rot;
    shape_user["move"] = &JyShape::move;
    // 属性设置
    shape_user["color"] = &JyShape::color;
    shape_user["transparency"] = &JyShape::transparency;
    shape_user["mass"] = &JyShape::mass;

    // 全局函数
    const auto show_one = [=](const JyShape &s) { emit display(s); };
    const auto show_multi = [=](const sol::table &_list) {
        for (int i = 1; i <= _list.size(); ++i) {
            const JyShape &s = _list[i];
            emit display(s);
        }
    };
    lua["show"] = sol::overload(show_one, show_multi);

    const auto stl_default = [=](const JyShape &s, const std::string &_filename) {
        emit display(s);
        s.export_stl(_filename, {});
    };
    const auto stl_option = [=](JyShape &s, const std::string &_filename, const sol::table &_opt) {
        if (_opt["radian"].is<double>()) {
            s.deviation_ = _opt["radian"];
            s.need_update_deviation_ = true;
        }
        emit display(s);
        s.export_stl(_filename, _opt);
    };
    lua["export_stl"] = sol::overload(stl_default, stl_option);
    lua["export_step"] = [=](const JyShape &s, const std::string &_filename) {
        emit display(s);
        s.export_step(_filename);
    };
    lua["export_iges"] = [=](const JyShape &s, const std::string &_filename) {
        emit display(s);
        s.export_iges(_filename);
    };
    lua["group_mass"] = [=](const sol::table &_list) -> double {
        std::vector<JyShape> shapes;
        for (int i = 1; i <= _list.size(); ++i) {
            const JyShape &s = _list[i];
            shapes.push_back(s);
        }
        return JyShape::group_mass_properties(shapes);
    };

    const auto box_ctor = sol::constructors<JyShapeBox(),
            JyShapeBox(const JyShapeBox &),
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
    lua.new_usertype<JyShapeBox>("box", box_ctor, sol::base_classes, sol::bases<JyShape>());
    lua.new_usertype<JyCylinder>("cylinder", cylinder_ctor, sol::base_classes, sol::bases<JyShape>());
    lua.new_usertype<JyCone>("cone", cone_ctor, sol::base_classes, sol::bases<JyShape>());
    lua.new_usertype<JySphere>("sphere", sphere_ctor, sol::base_classes, sol::bases<JyShape>());

    const auto edge_ctor = sol::constructors<JyEdge(const JyEdge &),
            JyEdge(const std::string &, const std::array<double, 3>, const std::array<double, 3>),
            JyEdge(const std::string &, const std::array<double, 3>, const std::array<double, 3>, const double &),
            JyEdge(const std::string &, const std::array<double, 3>, const std::array<double, 3>, const double &, const double &)>();
    lua.new_usertype<JyEdge>("edge", edge_ctor, sol::base_classes, sol::bases<JyShape>());

    const auto wire_ctor = sol::constructors<JyWire(),
            JyWire(const JyWire &),
            JyWire(const sol::table &)>();
    lua.new_usertype<JyWire>("wire", wire_ctor, sol::base_classes, sol::bases<JyShape>());
    const auto polygon_ctor = sol::constructors<JyPolygon(),
            JyPolygon(const JyPolygon &),
            JyPolygon(const sol::table &)>();
    lua.new_usertype<JyPolygon>("polygon", polygon_ctor, sol::base_classes, sol::bases<JyWire, JyShape>());

    const auto face_ctor = sol::constructors<JyFace(),
            JyFace(const JyFace &),
            JyFace(const JyShape &)>();
    lua.new_usertype<JyFace>("face", face_ctor, sol::base_classes, sol::bases<JyShape>());
}

void JyLuaVirtualMachine::run(const std::string &_file_path) {
    QElapsedTimer localTimer;
    localTimer.start();
    lua.collect_gc();   // 运行前做一次完整的垃圾收集循环
    auto result = lua.script_file(_file_path, sol::script_pass_on_error);
    const qint64 elapsed = localTimer.elapsed();
    if (!result.valid()) {
        const QString message = result.get<sol::error>().what();
        emit sig_show_message(message, -1);
    } else {
        emit sig_show_message(QString("success✅ elapsed: %1 ms").arg(elapsed),1);
    }
    lua.collect_gc();   // 运行完做一次完整的垃圾收集循环
}


void JyLuaVirtualMachine::exec_code(const std::string &_code) {
    auto result = lua.safe_script(_code, sol::script_pass_on_error);
    if (!result.valid()) {
        const QString message = result.get<sol::error>().what();
        emit sig_show_message(message, -2);
    } else {}
}

void JyLuaVirtualMachine::add_package_path(const std::string &_path) {
    lua["package"]["path"] = current_path_ + ";" + _path;
}


void JyLuaVirtualMachine::lua_print(const sol::object &v) {
    if (v.get_type() == sol::type::string) {
        emit sig_show_message(QString::fromStdString(v.as<std::string>()), 0);
    } else if (v.get_type() == sol::type::number) {
        if (v.is<int>()) { emit sig_show_message(QString::number(v.as<int>()), 0); }
        else if (v.is<double>()) { emit sig_show_message(QString::number(v.as<double>()), 0); }
        else {}
    } else if (v.get_type() == sol::type::table) {
        const auto addr = QString::number((qulonglong) v.as<sol::table>().pointer(), 16).toUpper();
        emit sig_show_message(("table: " + addr), 0);
    } else if (v.get_type() == sol::type::nil) {
        emit sig_show_message("nil", 0);
    } else if (v.get_type() == sol::type::boolean) {
        emit sig_show_message((v.as<bool>() ? "true" : "false"), 0);
    } else if (v.get_type() == sol::type::userdata) {
        const auto addr = QString::number((qulonglong) v.as<sol::userdata>().pointer(), 16).toUpper();
        emit sig_show_message(("userdata: " + addr), 0);
    } else if (v.get_type() == sol::type::function) {
        const auto addr = QString::number((qulonglong) v.as<sol::function>().pointer(), 16).toUpper();
        emit sig_show_message(("function: " + addr), 0);
    } else if (v.get_type() == sol::type::thread) {
        const auto addr = QString::number((qulonglong) v.as<sol::thread>().pointer(), 16).toUpper();
        emit sig_show_message(("thread: " + addr), 0);
    } else {}
}