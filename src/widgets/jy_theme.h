/*
 * Copyright (c) 2024. Li Jianbin. All rights reserved.
 * MIT License
 */
#ifndef JY_THEME_H
#define JY_THEME_H

#include <QColor>
#include <QIcon>
#include <QString>

/**
 * @brief 界面主题：集中管理设计变量（颜色）与图标
 *
 * style.qss 中使用 @token 形式引用颜色，例如 @accent、@text-muted，
 * 加载时由 styleSheet() 替换为实际颜色值。
 * 图标为白色线条 PNG（Lucide），由 icon() 按主题色着色。
 */
namespace JyTheme {
    //! 获取设计变量对应的颜色，如 color("accent")
    QColor color(const QString &token);

    //! 读取 :/style.qss 并替换其中的 @token
    QString styleSheet();

    //! 获取着色后的图标，name 为 :/icons/<name>.png 的文件名
    //! 普通状态使用 text-muted，悬停/选中使用 text，禁用使用 text-disabled
    QIcon icon(const QString &name);

    //! 获取单一颜色的图标
    QIcon icon(const QString &name, const QColor &color);
}// namespace JyTheme

#endif//JY_THEME_H
