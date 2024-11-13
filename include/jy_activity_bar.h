/*
 * Copyright (c) 2024. Li Jianbin. All rights reserved.
 * MIT License
 */
#ifndef JY_ACTIVITY_BAR_H
#define JY_ACTIVITY_BAR_H

#include <QToolBar>
#include <QPushButton>
#include <QVBoxLayout>
#include <QStyle>
#include <QApplication>

class JyActivityBar : public QToolBar {
Q_OBJECT
public:
    explicit JyActivityBar(QWidget *parent = nullptr) : QToolBar(parent) {
        setMovable(false); // 工具栏不可移动
        setContextMenuPolicy(Qt::CustomContextMenu);    // 禁止右键菜单
        const auto button_script = new QPushButton();   // 创建脚本显示按钮
        button_script->setIcon(QApplication::style()->standardIcon(QStyle::SP_FileIcon));
        button_script->setToolTip(tr("Script"));   // 设置提示文字
        button_script->setIconSize(QSize(24, 24));   // 设置图标大小
        button_script->setCheckable(true);    // 可选中
        button_script->setChecked(true);      // 默认选中
        // 转发按钮信号到主窗口，用于显示/隐藏脚本编辑界面
        connect(button_script, &QPushButton::clicked, this, &JyActivityBar::sig_show_script);
        // 布局
        const auto widget_tool_buttons = new QWidget(this); // 按钮容器
        const auto vbox_layout = new QVBoxLayout(widget_tool_buttons); // 垂直排列按钮
        vbox_layout->setContentsMargins(0, 0, 0, 0);
        vbox_layout->setSpacing(0);
        vbox_layout->addWidget(button_script);  // 加入脚本按钮
        addWidget(widget_tool_buttons); //工具栏加入按钮容器
    }

signals:

    void sig_show_script(bool); // 显示(true)或隐藏(false)脚本编辑界面的信号
};

#endif //JY_ACTIVITY_BAR_H
