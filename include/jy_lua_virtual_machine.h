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

signals:

    void display(const JyShape &theIObj);

    void sig_show_message(const QString &_message, int timeout = 0);
};

#endif //JY_LUA_VIRTUAL_MACHINE
