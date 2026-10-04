#pragma once
#include <QNetworkAccessManager>
#include <QNetworkReply>
#include <QPointer>
#include <QTimer>
#include <QStringList>

class JySseDecoder {
public:
    explicit JySseDecoder(bool anthropic = false) : m_anthropic(anthropic) {}
    QStringList feed(const QByteArray &bytes);
    void finish() const;
private:
    QString decode(const QByteArray &data);
    QByteArray m_buffer, m_event;
    bool m_anthropic = false;
    bool m_done = false;
};

class JyLlmClient : public QObject {
    Q_OBJECT
public:
    // An injected transport must outlive the client (used by offline tests).
    explicit JyLlmClient(QObject *parent = nullptr, QNetworkAccessManager *transport = nullptr);
    ~JyLlmClient() override;
    void start(const QNetworkRequest &request, const QByteArray &body, bool anthropic);
    void cancel();
    void setTimeout(int ms) { m_timeout.setInterval(ms); }
signals:
    void chunk(const QString &text);
    void succeeded(const QString &code);
    void failed(const QString &message);
private:
    void consume(QNetworkReply *reply);
    void fail(const QString &message);
    QNetworkAccessManager *m_network;
    QPointer<QNetworkReply> m_reply;
    QTimer m_timeout;
    JySseDecoder m_decoder;
    QString m_code;
};
