/*
 * Copyright (c) 2024. Li Jianbin. All rights reserved.
 * MIT License
 */
#ifndef JY_LLM_DIALOG_H
#define JY_LLM_DIALOG_H

#include <QComboBox>
#include <QDialog>
#include <QLabel>
#include <QLineEdit>
#include <QNetworkAccessManager>
#include <QNetworkReply>
#include <QProgressBar>
#include <QPushButton>
#include <QTextEdit>
#include <QWidget>

class JyLlmDialog : public QDialog {
    Q_OBJECT

public:
    explicit JyLlmDialog(QWidget *parent = nullptr);
    ~JyLlmDialog() override;

    void setCurrentCode(const QString &code);
    QString getCurrentCode() const { return m_currentCode; }

signals:
    void codeStreamUpdate(const QString &deltaText);
    void codeGenerationFinished(const QString &fullCode);

private slots:
    void onSendClicked();
    void onToggleSettings();
    void onProviderChanged(int index);
    void onNetworkReplyReceived();
    void onNetworkError(QNetworkReply::NetworkError error);

private:
    void setupUi();
    void setupConnections();
    void loadSettings();
    void saveSettings();
    void sendRequest(const QString &userMessage);
    void processStreamData(const QByteArray &data);
    void updateProgress(const QString &status);

    // UI Components
    QTextEdit *m_promptInput;
    QPushButton *m_sendButton;
    QPushButton *m_settingsToggleButton;
    QWidget *m_settingsPanel;
    QComboBox *m_providerCombo;
    QLineEdit *m_apiKeyEdit;
    QLineEdit *m_customUrlEdit;
    QComboBox *m_modelCombo;
    QProgressBar *m_progressBar;
    QLabel *m_statusLabel;

    // Network
    QNetworkAccessManager *m_networkManager;
    QNetworkReply *m_currentReply;

    // State
    QString m_currentCode;
    QString m_streamBuffer;
    QString m_generatedCode;
    bool m_isStreaming;

    // Configuration
    struct ProviderConfig {
        QString name;
        QString apiUrl;
        QStringList models;
    };
    QList<ProviderConfig> m_providers;
    int m_currentProviderIndex;
};

#endif// JY_LLM_DIALOG_H
