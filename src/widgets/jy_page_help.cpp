/*
 * Copyright (c) 2024. Li Jianbin. All rights reserved.
 * MIT License
 */
#include "jy_page_help.h"
#include "lua.hpp"
#include <QDateTime>
#include <QDesktopServices>
#include <QJsonDocument>
#include <QJsonObject>
#include <QLabel>
#include <QLocale>
#include <QLoggingCategory>
#include <QMessageBox>
#include <QTextBrowser>
#include <QUrl>
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

JyPageHelp::JyPageHelp(QWidget *parent)
    : QWidget(parent),
      m_networkManager(nullptr),
      m_currentReply(nullptr) {

    // Disable Qt network monitor warnings before creating QNetworkAccessManager
    QLoggingCategory::setFilterRules("qt.network.monitor=false");
    m_networkManager = new QNetworkAccessManager(this);

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
    m_currentVersion = QString::fromStdString(JELLY_CAD_VERSION);
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

    // Add check update button
    m_checkUpdateButton = new QPushButton(tr("🔄 Check for Updates"));
    m_checkUpdateButton->setMaximumWidth(200);
    connect(m_checkUpdateButton, &QPushButton::clicked, this, &JyPageHelp::onCheckUpdateClicked);
    layout()->addWidget(m_checkUpdateButton);

    auto help_doc = new QTextBrowser;
    help_doc->setOpenLinks(true);
    help_doc->setOpenExternalLinks(true);


    help_doc->setHtml(html_intro);
    layout()->addWidget(help_doc);
}

void JyPageHelp::onCheckUpdateClicked() {
    m_checkUpdateButton->setEnabled(false);
    m_checkUpdateButton->setText(tr("Checking..."));

    // Check for updates using GitHub API
    QUrl url("https://api.github.com/repos/Jelatine/JellyCAD/releases/latest");
    QNetworkRequest request(url);
    request.setHeader(QNetworkRequest::UserAgentHeader, "JellyCAD-UpdateChecker");

    if (m_currentReply) {
        m_currentReply->abort();
        m_currentReply->deleteLater();
    }

    m_currentReply = m_networkManager->get(request);

    connect(m_currentReply, &QNetworkReply::finished,
            this, &JyPageHelp::onUpdateCheckFinished);
    connect(m_currentReply, QOverload<QNetworkReply::NetworkError>::of(&QNetworkReply::errorOccurred),
            this, &JyPageHelp::onNetworkError);
}

void JyPageHelp::onUpdateCheckFinished() {
    m_checkUpdateButton->setEnabled(true);
    m_checkUpdateButton->setText(tr("🔄 Check for Updates"));

    if (!m_currentReply) return;

    if (m_currentReply->error() == QNetworkReply::NoError) {
        QByteArray response = m_currentReply->readAll();
        QJsonDocument doc = QJsonDocument::fromJson(response);

        if (doc.isObject()) {
            QJsonObject obj = doc.object();
            QString latestVersion = obj["tag_name"].toString();

            if (!latestVersion.isEmpty()) {
                compareVersions(latestVersion);
            } else {
                QMessageBox::information(this, tr("Update Check"),
                                         tr("Unable to retrieve version information."));
            }
        }
    }

    m_currentReply->deleteLater();
    m_currentReply = nullptr;
}

void JyPageHelp::onNetworkError(QNetworkReply::NetworkError error) {
    Q_UNUSED(error);

    m_checkUpdateButton->setEnabled(true);
    m_checkUpdateButton->setText(tr("🔄 Check for Updates"));

    if (m_currentReply) {
        QString errorString = m_currentReply->errorString();
        QMessageBox::warning(this, tr("Update Check Failed"),
                             tr("Failed to check for updates: %1").arg(errorString));
    }
}

void JyPageHelp::compareVersions(const QString &latestVersion) {
    // Remove 'v' prefix if present
    QString current = m_currentVersion;
    QString latest = latestVersion;

    if (current.startsWith('v') || current.startsWith('V')) {
        current = current.mid(1);
    }
    if (latest.startsWith('v') || latest.startsWith('V')) {
        latest = latest.mid(1);
    }

    if (isNewerVersion(current, latest)) {
        QMessageBox msgBox(this);
        msgBox.setWindowTitle(tr("Update Available"));
        msgBox.setText(tr("A new version is available!"));
        msgBox.setInformativeText(tr("Current version: %1\nLatest version: %2\n\nDo you want to download the update?")
                                          .arg(m_currentVersion, latestVersion));
        msgBox.setStandardButtons(QMessageBox::Yes | QMessageBox::No);
        msgBox.setDefaultButton(QMessageBox::Yes);

        if (msgBox.exec() == QMessageBox::Yes) {
            // Open releases page in browser
            QDesktopServices::openUrl(QUrl("https://github.com/Jelatine/JellyCAD/releases/latest"));
        }
    } else {
        QMessageBox::information(this, tr("No Updates"),
                                 tr("You are already using the latest version (%1).").arg(m_currentVersion));
    }
}

bool JyPageHelp::isNewerVersion(const QString &current, const QString &latest) {
    // Parse version strings (e.g., "1.2.3" or "1.2.3-beta")
    QStringList currentParts = current.split('-')[0].split('.');
    QStringList latestParts = latest.split('-')[0].split('.');

    // Compare version numbers
    int maxLen = qMax(currentParts.length(), latestParts.length());
    for (int i = 0; i < maxLen; ++i) {
        int currentNum = (i < currentParts.length()) ? currentParts[i].toInt() : 0;
        int latestNum = (i < latestParts.length()) ? latestParts[i].toInt() : 0;

        if (latestNum > currentNum) {
            return true;
        } else if (latestNum < currentNum) {
            return false;
        }
    }

    return false;// Versions are equal
}