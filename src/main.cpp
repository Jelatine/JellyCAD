/*
 * Copyright (c) 2024. Li Jianbin. All rights reserved.
 * MIT License
 */
#include "jy_main_window.h"
#include <QApplication>
#include <QCommandLineOption>
#include <QCommandLineParser>

/**
 * @brief JellyCAD 应用程序主入口
 *
 * 支持三种运行模式：
 * 1. 脚本文件模式：通过 -f/--file 参数执行Lua脚本文件
 * 2. 代码字符串模式：通过 -c/--code 参数执行Lua代码字符串
 * 3. GUI模式：直接启动图形界面
 */
int main(int argc, char *argv[]) {
    // 创建Qt应用程序实例
    QApplication a(argc, argv);

    // 设置应用程序基本信息
    QCoreApplication::setApplicationName("JellyCAD");
    QCoreApplication::setApplicationVersion(JELLY_CAD_VERSION);

    // 配置命令行参数解析器
    QCommandLineParser parser;
    parser.addHelpOption();    // 添加 -h/--help 选项
    parser.addVersionOption(); // 添加 -v/--version 选项

    // 添加脚本文件执行选项
    QCommandLineOption file_option(QStringList() << "f" << "file", "Script file to execute", "file");
    parser.addOption(file_option);

    // 添加代码字符串执行选项
    QCommandLineOption code_option(QStringList() << "c" << "code", "Script code string to execute", "code");
    parser.addOption(code_option);

    // 解析命令行参数
    parser.process(a);

    // 注册自定义Qt元类型，用于信号槽跨线程传递
    qRegisterMetaType<JyShape>("JyShape");
    qRegisterMetaType<JyAxes>("JyAxes");

    // 创建主窗口实例
    JyMainWindow w;

    // 根据命令行参数选择运行模式
    if (parser.isSet(file_option)) {
        // 模式1：执行脚本文件（无GUI）
        const auto lvm = new JyLuaVirtualMachine();
        lvm->runScript(parser.value(file_option));
    } else if (parser.isSet(code_option)) {
        // 模式2：执行代码字符串（无GUI）
        const auto lvm = new JyLuaVirtualMachine();
        lvm->runScript(parser.value(code_option), false);
    } else {
        // 模式3：启动GUI界面

        // 加载并应用QSS颜色样式
        QFile style_file(":/style.qss");
        style_file.open(QFile::ReadOnly);
        if (style_file.isOpen()) {
            const auto style_str = style_file.readAll();
            a.setStyleSheet(style_str);
            style_file.close();
        }

        // 显示主窗口并进入事件循环
        w.show();
        QApplication::exec();
    }
}