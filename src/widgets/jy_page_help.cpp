/*
 * Copyright (c) 2024. Li Jianbin. All rights reserved.
 * MIT License
 */
#include "jy_page_help.h"
#include "lua.hpp"
#include <QDateTime>
#include <QLabel>
#include <QLocale>
#include <QTextBrowser>
#include <QVBoxLayout>

constexpr char html_intro[] = R"(
<style>
a {color: white;text-decoration: none;padding: 12px 24px;background: rgba(255, 255, 255, 0.2);margin: 4px;}
</style>
<h3>Modern open-source programmable CAD software designed for programmers, 
robotics developers, and parametric modeling enthusiasts.</h3>
<div>
    <a href="https://github.com/Jelatine/JellyCAD">📦 GitHub</a>
    <a href="https://jelatine.github.io/JellyCAD/">🏠 Home</a>
    <a href="https://jelatine.github.io/JellyCAD/guide/functions">📚 Tutorial</a>
    <a href="https://github.com/Jelatine/JellyCAD/blob/main/LICENSE">📄 License</a>
</div>
)";

JyPageHelp::JyPageHelp(QWidget *parent) : QWidget(parent) {
    setLayout(new QVBoxLayout);
    layout()->setAlignment(Qt::AlignTop);
    auto label_software = new QLabel("JellyCAD");
    label_software->setAlignment(Qt::AlignCenter);
    label_software->setFont(QFont("Microsoft YaHei", 10, QFont::Bold));
    layout()->addWidget(label_software);
    const QDate buildDate = QLocale(QLocale::English).toDate(QString(__DATE__).replace("  ", " 0"), "MMM dd yyyy");
    const QTime buildTime = QTime::fromString(__TIME__, "hh:mm:ss");
    const auto str_publish_date = tr("Build on ") + buildDate.toString("yyyy/MM/dd") + " " + buildTime.toString();
    layout()->addWidget(new QLabel(str_publish_date));
    QString version_jelly_cad("JellyCAD ");
    QString version_qt("Qt ");
    QString version_occt("OpenCASCADE ");
    QString version_sol2("sol2 ");
#ifdef JELLY_CAD_VERSION
    version_jelly_cad += QString::fromStdString(JELLY_CAD_VERSION);
#endif
#ifdef VERSION_DISPLAY_QT
    version_qt += QString::fromStdString(VERSION_DISPLAY_QT);
#endif
#ifdef VERSION_DISPLAY_OCC
    version_occt += QString::fromStdString(VERSION_DISPLAY_OCC);
#endif
#ifdef VERSION_DISPLAY_SOL2
    version_sol2 += QString::fromStdString(VERSION_DISPLAY_SOL2);
#endif
    label_software->setText(version_jelly_cad);
    layout()->addWidget(new QLabel(tr("Dependencies:")));

    // 创建依赖项容器，统一设置缩进
    auto dependencies_container = new QWidget;
    auto dependencies_layout = new QVBoxLayout(dependencies_container);
    dependencies_layout->setContentsMargins(20, 0, 0, 0);
    dependencies_layout->setSpacing(0);

    dependencies_layout->addWidget(new QLabel(version_qt));
    dependencies_layout->addWidget(new QLabel(version_occt));
    dependencies_layout->addWidget(new QLabel(version_sol2));
#ifdef LUA_VERSION
    dependencies_layout->addWidget(new QLabel(QString::fromStdString(LUA_VERSION)));
#endif

    layout()->addWidget(dependencies_container);
    auto help_doc = new QTextBrowser;
    help_doc->setOpenLinks(true);
    help_doc->setOpenExternalLinks(true);


    help_doc->setHtml(html_intro);
    layout()->addWidget(help_doc);
}