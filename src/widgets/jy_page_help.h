/*
 * Copyright (c) 2024. Li Jianbin. All rights reserved.
 * MIT License
 */
#ifndef JY_PAGE_HELP_H
#define JY_PAGE_HELP_H

#include <QWidget>
#include <QLabel>
#include <QVBoxLayout>
#include <QDateTime>
#include <QLocale>
#include <QTextBrowser>

class JyPageHelp : public QWidget {
public:
    explicit JyPageHelp(QWidget *parent = nullptr) : QWidget(parent) {
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
#ifdef JELLY_CAD_VERSION
        label_software->setText("JellyCAD " + QString::fromStdString(JELLY_CAD_VERSION));
#endif
        layout()->addWidget(new QLabel(tr("Dependencies:")));
#ifdef VERSION_DISPLAY_QT5
        layout()->addWidget(new QLabel("\tQt5 " + QString::fromStdString(VERSION_DISPLAY_QT5)));
#else
        layout()->addWidget(new QLabel("\tQt5"));
#endif
#ifdef VERSION_DISPLAY_OCC
        layout()->addWidget(new QLabel("\tOpenCASCADE " + QString::fromStdString(VERSION_DISPLAY_OCC)));
#else
        layout()->addWidget(new QLabel("\tOpenCASCADE"));
#endif
#ifdef VERSION_DISPLAY_SOL2
        layout()->addWidget(new QLabel("\tsol2 " + QString::fromStdString(VERSION_DISPLAY_SOL2)));
#else
        layout()->addWidget(new QLabel("\tsol2"));
#endif
#ifdef LUA_VERSION
        layout()->addWidget(new QLabel("\t" + QString::fromStdString(LUA_VERSION)));
#endif
        auto help_doc = new QTextBrowser;
        help_doc->setOpenLinks(true);
        help_doc->setOpenExternalLinks(true);


        QFile file_help_doc(":/help.html");
        if(file_help_doc.open(QFile::ReadOnly)){
            help_doc->setHtml(file_help_doc.readAll());
            file_help_doc.close();
        }
        layout()->addWidget(help_doc);
    }
};

#endif //JY_PAGE_HELP_H
