/*
 * Copyright (c) 2024. Li Jianbin. All rights reserved.
 * MIT License
 */
#ifndef JY_PAGE_HELP_H
#define JY_PAGE_HELP_H

#include <QWidget>
#include <QNetworkAccessManager>
#include <QNetworkReply>
#include <QPushButton>

class JyPageHelp : public QWidget {
    Q_OBJECT

public:
    explicit JyPageHelp(QWidget *parent = nullptr);

private slots:
    void onCheckUpdateClicked();
    void onUpdateCheckFinished();
    void onNetworkError(QNetworkReply::NetworkError error);

private:
    void compareVersions(const QString &latestVersion);
    bool isNewerVersion(const QString &current, const QString &latest);

    QNetworkAccessManager *m_networkManager;
    QNetworkReply *m_currentReply;
    QPushButton *m_checkUpdateButton;
    QString m_currentVersion;
};

#endif//JY_PAGE_HELP_H
