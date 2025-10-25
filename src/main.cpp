/*
 * Copyright (c) 2024. Li Jianbin. All rights reserved.
 * MIT License
 */
#include "jy_main_window.h"
#include <QApplication>
#include <QCommandLineOption>
#include <QCommandLineParser>

int main(int argc, char *argv[]) {
    QApplication a(argc, argv);
    QCoreApplication::setApplicationName("JellyCAD");
    QCoreApplication::setApplicationVersion(JELLY_CAD_VERSION);
    QCommandLineParser parser;
    parser.addHelpOption();
    parser.addVersionOption();
    QCommandLineOption file_option(QStringList() << "f" << "file", "Script file to execute", "file");
    parser.addOption(file_option);
    QCommandLineOption code_option(QStringList() << "c" << "code", "Script code string to execute", "code");
    parser.addOption(code_option);
    parser.process(a);
    qRegisterMetaType<JyShape>("JyShape");
    qRegisterMetaType<JyAxes>("JyAxes");
    JyMainWindow w;
    if (parser.isSet(file_option)) {
        const auto lvm = new JyLuaVirtualMachine();
        lvm->runScript(parser.value(file_option));
    } else if (parser.isSet(code_option)) {
        const auto lvm = new JyLuaVirtualMachine();
        lvm->runScript(parser.value(code_option), false);
    } else {
        // 颜色样式
        QFile style_file(":/style.qss");
        style_file.open(QFile::ReadOnly);
        if (style_file.isOpen()) {
            const auto style_str = style_file.readAll();
            a.setStyleSheet(style_str);
            style_file.close();
        }
        // 显示主窗口
        w.show();
        QApplication::exec();
    }
}