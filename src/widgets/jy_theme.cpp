/*
 * Copyright (c) 2024. Li Jianbin. All rights reserved.
 * MIT License
 */
#include "jy_theme.h"
#include <QFile>
#include <QMap>
#include <QPainter>
#include <QPixmap>
#include <QRegularExpression>

namespace {
    // 深色主题设计变量
    const QMap<QString, QString> &tokens() {
        static const QMap<QString, QString> map = {
                // 背景层级：base(编辑区/视图) < panel(侧栏/工具栏) < elevated(菜单/弹窗)
                {"bg-base", "#1E1F22"},
                {"bg-panel", "#2B2D30"},
                {"bg-elevated", "#313338"},
                {"bg-input", "#1E1F22"},
                {"bg-alt", "#2E3034"},
                {"bg-hover", "#393B40"},
                {"bg-active", "#43454A"},
                // 边框
                {"border", "#393B40"},
                {"border-strong", "#4E5157"},
                // 文字
                {"text", "#DFE1E5"},
                {"text-muted", "#9DA0A8"},
                {"text-disabled", "#6F737A"},
                {"text-on-accent", "#FFFFFF"},
                // 强调色
                {"accent", "#3574F0"},
                {"accent-hover", "#4682FA"},
                {"accent-pressed", "#2E64D0"},
                {"accent-subtle", "rgba(53, 116, 240, 0.22)"},
                // 状态色
                {"danger", "#E55765"},
                {"success", "#5FB865"},
        };
        return map;
    }

    QPixmap tinted(const QString &name, const QColor &color) {
        QPixmap pixmap(":/icons/" + name + ".png");
        if (pixmap.isNull()) { return pixmap; }
        QPainter painter(&pixmap);
        painter.setCompositionMode(QPainter::CompositionMode_SourceIn);
        painter.fillRect(pixmap.rect(), color);
        return pixmap;
    }
}// namespace

QColor JyTheme::color(const QString &token) {
    return QColor(tokens().value(token));
}

QString JyTheme::styleSheet() {
    QFile file(":/style.qss");
    if (!file.open(QFile::ReadOnly)) { return {}; }
    QString style = QString::fromUtf8(file.readAll());
    // 按 @token 匹配完整名称，避免 @accent 误替换 @accent-hover 的前缀
    static const QRegularExpression re("@([a-z][a-z0-9-]*)");
    QString result;
    qsizetype last = 0;
    auto it = re.globalMatch(style);
    while (it.hasNext()) {
        const auto match = it.next();
        const auto value = tokens().value(match.captured(1));
        if (value.isEmpty()) { continue; }
        result += style.mid(last, match.capturedStart() - last) + value;
        last = match.capturedEnd();
    }
    result += style.mid(last);
    return result;
}

QIcon JyTheme::icon(const QString &name) {
    QIcon icon;
    const auto normal = tinted(name, color("text-muted"));
    const auto active = tinted(name, color("text"));
    icon.addPixmap(normal, QIcon::Normal, QIcon::Off);
    icon.addPixmap(active, QIcon::Normal, QIcon::On);
    icon.addPixmap(active, QIcon::Active, QIcon::Off);
    icon.addPixmap(active, QIcon::Active, QIcon::On);
    icon.addPixmap(active, QIcon::Selected, QIcon::Off);
    icon.addPixmap(tinted(name, color("text-disabled")), QIcon::Disabled, QIcon::Off);
    return icon;
}

QIcon JyTheme::icon(const QString &name, const QColor &color) {
    return QIcon(tinted(name, color));
}
