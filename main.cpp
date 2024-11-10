/*
 * Copyright (c) 2024. Li Jianbin. All rights reserved.
 * MIT License
 */
#include <QApplication>
#include "jy_main_window.h"

int main(int argc, char *argv[]) {
    QApplication a(argc, argv);
    bool script_mode{false};
    QString path_script;
    for (const auto &x: QApplication::arguments()) {
        if (script_mode) {
            path_script = x;
            break;
        }
        if (x == "-f") { script_mode = true; }
    }
    JyMainWindow w;
    if (script_mode && !path_script.isEmpty()) {
        w.run_script(path_script);
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