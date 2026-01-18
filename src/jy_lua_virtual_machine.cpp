/*
 * Copyright (c) 2024. Li Jianbin. All rights reserved.
 * MIT License
 */
#include "jy_lua_virtual_machine.h"
#include "jy_make_shapes.h"
#include "jy_urdf_generator.h"
#include <QDebug>
#include <QDir>
#include <QElapsedTimer>
#include <QFileInfo>

std::atomic<bool> should_exit(false);
// 调试钩子函数
void debug_hook(lua_State *L, lua_Debug *ar) {
    if (should_exit.load()) {
        luaL_error(L, "Script interrupted by external command");
    }
}

void JyLuaVirtualMachine::registerBindings() {
    lua.open_libraries();
    lua_sethook(lua.lua_state(), debug_hook, LUA_MASKCOUNT, 1);
    if(!lua_checkstack(lua.lua_state(), 1000)){
        throw std::runtime_error("Lua stack overflow!");
    }
    lua["print"] = [=](const sol::object &v) { this->lua_print(v); };
    auto shape_user = JyShape::configure_usertype(lua);
    shape_user["show"] = [this](JyShape &self) -> JyShape & { emit this->displayShape(self);return self; };
    JyEdge::configure_usertype(lua);
    JyFace::configure_usertype(lua);
    JyMakeShapes::configure_usertype(lua);
    // ----- Axes -----
    auto axes_user = JyAxes::configure_usertype(lua);
    axes_user["show"] = [this](const JyAxes &self) { return emit this->displayAxes(self); };
    // ----- URDF -----
    Link::configure_usertype(lua);

    // 全局函数
    const auto show_one = [=](const JyShape &s) { emit displayShape(s); };
    const auto show_multi = [=](const sol::table &_list) {
        for (int i = 1; i <= _list.size(); ++i) {
            if (_list[i].is<JyShape>()) {
                const JyShape &s = _list[i];
                emit displayShape(s);
            } else if (_list[i].is<JyAxes>()) {
                const JyAxes &a = _list[i];
                emit displayAxes(a);
            } else {
                throw std::runtime_error("Wrong type!");
            }
        }
    };
    lua["show"] = sol::overload(show_one, show_multi);
}

void JyLuaVirtualMachine::runScript(const QString &_file_path, const bool &is_file) {
    QElapsedTimer localTimer;
    localTimer.start();
    lua = sol::state();// 重新赋值会自动清理旧的
    registerBindings();

    // 设置 arg 表
    sol::table arg = lua.create_table();
    if (is_file) {
        arg[0] = _file_path.toStdString();
    } else {
        arg[0] = "=(code)";
    }
    lua["arg"] = arg;

    sol::protected_function_result result;
    if (is_file) {
        QFileInfo fileInfo(_file_path);
        QString dirPath = fileInfo.absolutePath();
        QDir::setCurrent(dirPath);
        lua["package"]["path"] = lua["package"]["path"].get<std::string>() + ";" + dirPath.toStdString() + "/?.lua";
        result = lua.script_file(_file_path.toStdString(), sol::script_pass_on_error);
    } else {
        result = lua.safe_script(_file_path.toStdString(), sol::script_pass_on_error);
    }
    if (!result.valid()) {
        const QString message = result.get<sol::error>().what();
        emit scriptError("❌" + message);
        qDebug() << "\033[31m" << message << "\033[0m";
    } else {
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


void JyLuaVirtualMachine::lua_print(const sol::object &v) {
    QString msg;
    if (v.get_type() == sol::type::string) {
        msg = QString::fromStdString(v.as<std::string>());
    } else if (v.get_type() == sol::type::number) {
        msg = v.is<int>() ? QString::number(v.as<int>()) : QString::number(v.as<double>());
    } else if (v.get_type() == sol::type::table) {
        msg = "table: " + QString::number((qulonglong) v.as<sol::table>().pointer(), 16).toUpper();
    } else if (v.get_type() == sol::type::lua_nil) {
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
    qDebug() << msg;
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
    // if (!wait(3000)) { terminate(); }
}

void JyLuaVirtualMachine::run() {
    emit scriptStarted();
    if (script_mode.loadRelaxed() == 0) {
        runScript(m_fileName);
    } else {
        // 为代码执行模式设置 arg 表
        sol::table arg = lua.create_table();
        arg[0] = "=(code)";
        lua["arg"] = arg;

        auto result = lua.safe_script(m_fileName.toStdString(), sol::script_pass_on_error);
        if (!result.valid()) {
            const QString message = result.get<sol::error>().what();
            emit scriptError("❌" + message);
        } else {
            emit scriptFinished("");
        }
    }
}