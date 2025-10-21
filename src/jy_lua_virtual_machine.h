/*
 * Copyright (c) 2024. Li Jianbin. All rights reserved.
 * MIT License
 */
#ifndef JY_LUA_VIRTUAL_MACHINE
#define JY_LUA_VIRTUAL_MACHINE

#define SOL_ALL_SAFETIES_ON 1

#include "jy_axes.h"
#include "jy_shape.h"
#include <QAtomicInt>
#include <QMutex>
#include <QThread>
#include <sol/sol.hpp>

class JyLuaVirtualMachine : public QThread {
    Q_OBJECT
    sol::state lua;

public:
    explicit JyLuaVirtualMachine();

    void runScript(const QString &_file_path, const bool &is_file = true);

    void exec_code(const QString &_code);

    void add_package_path(const std::string &_path);

    void executeScript(const QString &fileName);
    void stopScript();
    bool isRunning() const;

signals:
    void scriptStarted(const QString &fileName);
    void scriptFinished(const QString &message);
    void scriptError(const QString &error);
    void scriptOutput(const QString &output);


protected:
    void run() override;

private:
    void lua_print(const sol::object &v);

    std::string current_path_;

    QString m_fileName;
    QAtomicInt script_mode;// 文件模式: 0 文件, 1 字符串
    QMutex m_mutex;

signals:

    void displayShape(const JyShape &theIObj);

    void displayAxes(const JyAxes &theAxes);
};

#endif//JY_LUA_VIRTUAL_MACHINE
