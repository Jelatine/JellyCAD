/*
 * Copyright (c) 2024. Li Jianbin. All rights reserved.
 * MIT License
 */
#include "jy_lua_virtual_machine.h"
#include "jy_urdf_generator.h"
#include <QElapsedTimer>

std::atomic<bool> should_exit(false);
// 调试钩子函数
void debug_hook(lua_State *L, lua_Debug *ar) {
    if (should_exit.load()) {
        luaL_error(L, "Script interrupted by external command");
    }
}

JyLuaVirtualMachine::JyLuaVirtualMachine() {
    lua.open_libraries();
    current_path_ = lua["package"]["path"].get<std::string>();
    lua_sethook(lua.lua_state(), debug_hook, LUA_MASKCOUNT, 1);
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
    shape_user["scale"] = &JyShape::scale;
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
    shape_user["zero"] = &JyShape::zero;
    // 属性设置
    shape_user["color"] = &JyShape::color;
    shape_user["transparency"] = &JyShape::transparency;
    shape_user["mass"] = &JyShape::mass;
    shape_user["export_step"] = &JyShape::export_step;
    shape_user["export_iges"] = &JyShape::export_iges;
    const auto overload_export_stl = sol::overload(
            static_cast<void (JyShape::*)(const std::string &_filename) const>(&JyShape::export_stl),
            static_cast<void (JyShape::*)(const std::string &_filename, const sol::table &_opt) const>(&JyShape::export_stl));
    shape_user["export_stl"] = overload_export_stl;
    shape_user["show"] = [this](const JyShape &self) { return emit this->display(self); };

    // 全局函数
    const auto show_one = [=](const JyShape &s) { emit display(s); };
    const auto show_multi = [=](const sol::table &_list) {
        for (int i = 1; i <= _list.size(); ++i) {
            if (_list[i].is<JyShape>()) {
                const JyShape &s = _list[i];
                emit display(s);
            } else if (_list[i].is<JyAxes>()) {
                const JyAxes &a = _list[i];
                emit displayAxes(a);
            } else {
                throw std::runtime_error("Wrong type!");
            }
        }
    };
    lua["show"] = sol::overload(show_one, show_multi);

    lua["export_stl"] = overload_export_stl;
    lua["export_step"] = &JyShape::export_step;
    lua["export_iges"] = &JyShape::export_iges;

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
    const auto text_ctor = sol::constructors<JyText(),
                                             JyText(const std::string &),
                                             JyText(const std::string &, const double &),
                                             JyText(const JyText &)>();
    lua.new_usertype<JyText>("text", text_ctor, sol::base_classes, sol::bases<JyShape>());

    // ----- Axes -----
    auto axes_user = lua.new_usertype<JyAxes>("axes", sol::constructors<JyAxes(const std::array<double, 6>, const double &)>());
    axes_user["show"] = [this](const JyAxes &self) { return emit this->displayAxes(self); };

    // ----- URDF -----
    auto link_user = lua.new_usertype<Link>("link", sol::constructors<Link(const std::string &, const JyShape &),
                                                                      Link(const std::string &, const sol::table &)>());
    link_user["export"] = sol::overload(
            static_cast<void (Link::*)(const std::string &robot_name) const>(&Link::export_urdf),
            static_cast<void (Link::*)(const sol::table &params) const>(&Link::export_urdf));
    link_user["add"] = sol::overload(
            static_cast<Joint &(Link::*) (const Joint &)>(&Link::add),
            static_cast<Joint &(Link::*) (const std::string &, const JyAxes &, const std::string &)>(&Link::add),
            static_cast<Joint &(Link::*) (const std::string &, const JyAxes &, const std::string &, sol::table &)>(&Link::add));
    auto joint_user = lua.new_usertype<Joint>("joint", sol::constructors<Joint(const std::string &, const JyAxes &, const std::string &),
                                                                         Joint(const std::string &, const JyAxes &, const std::string &, sol::table &)>());
    joint_user["next"] = sol::overload(
            static_cast<Link &(Joint::*) (const Link &)>(&Joint::next),
            static_cast<Link &(Joint::*) (const std::string &, const JyShape &)>(&Joint::next));
}

void JyLuaVirtualMachine::runScript(const QString &_file_path) {
    QElapsedTimer localTimer;
    localTimer.start();
    lua.collect_gc();// 运行前做一次完整的垃圾收集循环
    auto result = lua.script_file(_file_path.toStdString(), sol::script_pass_on_error);
    if (!result.valid()) {
        const QString message = result.get<sol::error>().what();
        lua.collect_gc();// 运行完做一次完整的垃圾收集循环
        emit scriptError("❌" + message);
        qDebug() << "\033[31m" << message << "\033[0m";
    } else {
        lua.collect_gc();// 运行完做一次完整的垃圾收集循环
        const auto message = QString("success, elapsed: %1 ms").arg(localTimer.elapsed());
        emit scriptFinished("✅" + message);
        qDebug() << "\033[32m" << message << "\033[0m";
    }
}


void JyLuaVirtualMachine::exec_code(const QString &_code) {
    QMutexLocker locker(&m_mutex);
    if (QThread::isRunning()) {
        return;// 已经在运行
    }
    m_fileName = _code;
    script_mode = 1;// 字符串模式
    should_exit = false;
    start();
}

void JyLuaVirtualMachine::add_package_path(const std::string &_path) {
    lua["package"]["path"] = current_path_ + ";" + _path;
}


void JyLuaVirtualMachine::lua_print(const sol::object &v) {
    QString msg;
    if (v.get_type() == sol::type::string) {
        msg = QString::fromStdString(v.as<std::string>());
    } else if (v.get_type() == sol::type::number) {
        msg = v.is<int>() ? QString::number(v.as<int>()) : QString::number(v.as<double>());
    } else if (v.get_type() == sol::type::table) {
        msg = "table: " + QString::number((qulonglong) v.as<sol::table>().pointer(), 16).toUpper();
    } else if (v.get_type() == sol::type::nil) {
        msg = "nil";
    } else if (v.get_type() == sol::type::boolean) {
        msg = v.as<bool>() ? "true" : "false";
    } else if (v.get_type() == sol::type::userdata) {
        msg = "userdata: " + QString::number((qulonglong) v.as<sol::userdata>().pointer(), 16).toUpper();
    } else if (v.get_type() == sol::type::function) {
        msg = "function: " + QString::number((qulonglong) v.as<sol::function>().pointer(), 16).toUpper();
    } else if (v.get_type() == sol::type::thread) {
        msg = "thread: " + QString::number((qulonglong) v.as<sol::thread>().pointer(), 16).toUpper();
    }
    if (!msg.isEmpty()) { emit scriptOutput(msg); }
}


void JyLuaVirtualMachine::executeScript(const QString &fileName) {
    QMutexLocker locker(&m_mutex);
    if (QThread::isRunning()) {
        return;// 已经在运行
    }
    m_fileName = fileName;
    script_mode = 0;// 文件模式
    should_exit = false;
    start();
}

void JyLuaVirtualMachine::stopScript() {
    should_exit = true;
    if (!wait(3000)) { terminate(); }
}

bool JyLuaVirtualMachine::isRunning() const {
    return QThread::isRunning();
}

void JyLuaVirtualMachine::run() {
    emit scriptStarted(m_fileName);
    if (script_mode.loadRelaxed() == 0) {
        runScript(m_fileName);
    } else {
        auto result = lua.safe_script(m_fileName.toStdString(), sol::script_pass_on_error);
        if (!result.valid()) {
            const QString message = result.get<sol::error>().what();
            emit scriptError("❌" + message);
        } else {
            emit scriptFinished("");
        }
    }
}