/*
 * Copyright (c) 2024. Li Jianbin. All rights reserved.
 * MIT License
 */
#include "jy_main_window.h"
#include "jy_theme.h"
#include <QApplication>
#include <memory>
#include <iostream>
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
    bool headless = false;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--") break;
        if (arg == "-f" || arg == "--file" || arg == "-c" || arg == "--code" ||
            arg.rfind("--file=", 0) == 0 || arg.rfind("--code=", 0) == 0 ||
            arg == "--help" || arg == "-h" || arg == "--version" || arg == "-v") headless = true;
    }
    std::unique_ptr<QCoreApplication> app;
    if (headless) app = std::make_unique<QCoreApplication>(argc, argv);
    else app = std::make_unique<QApplication>(argc, argv);

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
    parser.process(*app);

    if (parser.isSet(file_option) || parser.isSet(code_option)) {
        jelly::RunRequest request;
        request.isFile = parser.isSet(file_option);
        request.source = parser.value(request.isFile ? file_option : code_option).toStdString();
        request.trigger = jelly::RunSource::CommandLine;
        std::atomic<bool> cancel{false};
        const auto result = jelly::execute(request, cancel, [](jelly::RunEvent event) {
            if (auto *text = std::get_if<std::string>(&event)) std::cout << *text << '\n';
        });
        if (result.status != jelly::RunStatus::Success) std::cerr << result.message << '\n';
        return result.status == jelly::RunStatus::Success ? 0 : 1;
    }
    // 模式3：启动GUI界面

    // 加载并应用QSS样式（替换其中的设计变量）
    static_cast<QApplication *>(app.get())->setStyleSheet(JyTheme::styleSheet());

    // 显示主窗口并进入事件循环
    JyMainWindow w;
    w.show();
    return QApplication::exec();
}