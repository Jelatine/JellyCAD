/*
 * Copyright (c) 2024. Li Jianbin. All rights reserved.
 * MIT License
 */
#ifndef JY_LUA_VIRTUAL_MACHINE
#define JY_LUA_VIRTUAL_MACHINE

#define SOL_ALL_SAFETIES_ON 1

#include <sol/sol.hpp>
#include <QObject>
#include "jy_shape.h"

class JyLuaVirtualMachine : public QObject {
Q_OBJECT
    sol::state lua;
public:
    explicit JyLuaVirtualMachine();

    void run(const std::string &_file_path);

    void exec_code(const std::string &_code);

    void add_package_path(const std::string &_path);

private:
    void lua_print(const sol::object &v);

    std::string current_path_;

signals:

    void display(const JyShape &theIObj, const bool &with_coord = false);

    /**
     * show message in terminal window and status bar
     * @param _result
     * @param _type -2:error(code), -1:error(file), 0:info, 1:success
     */
    void sig_show_message(const QString &_result, const int &_type);
};

#endif //JY_LUA_VIRTUAL_MACHINE
