#include "jy_llm_client.h"
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <stdexcept>

QString JySseDecoder::decode(const QByteArray &data) {
    if (m_done || data.isEmpty()) return {};
    if (data.trimmed() == "[DONE]") { m_done = true; return {}; }
    QJsonParseError error;
    const auto doc = QJsonDocument::fromJson(data, &error);
    if (error.error != QJsonParseError::NoError || !doc.isObject()) throw std::runtime_error("Invalid streaming response");
    const auto object = doc.object();
    if (object.contains("error") || object["type"].toString() == "error") throw std::runtime_error("Provider returned an error");
    if (m_anthropic) {
        if (object["type"].toString() == "message_stop") m_done = true;
        if (object["type"].toString() == "content_block_delta") return object["delta"].toObject()["text"].toString();
        return {};
    }
    const auto choices = object["choices"].toArray();
    return choices.isEmpty() ? QString() : choices.first().toObject()["delta"].toObject()["content"].toString();
}
QStringList JySseDecoder::feed(const QByteArray &bytes) {
    m_buffer += bytes;
    QStringList chunks;
    int end;
    while ((end = m_buffer.indexOf('\n')) >= 0) {
        auto line = m_buffer.left(end);
        m_buffer.remove(0, end + 1);
        if (line.endsWith('\r')) line.chop(1);
        if (line.isEmpty()) {
            const auto text = decode(m_event);
            m_event.clear();
            if (!text.isEmpty()) chunks.push_back(text);
        } else if (line.startsWith("data:")) {
            auto data = line.mid(5);
            if (data.startsWith(' ')) data.remove(0, 1);
            if (!m_event.isEmpty()) m_event += '\n';
            m_event += data;
        }
        if (m_event.size() > 1024 * 1024) throw std::runtime_error("Streaming event too large");
    }
    if (m_buffer.size() > 1024 * 1024) throw std::runtime_error("Streaming buffer too large");
    return chunks;
}
void JySseDecoder::finish() const {
    if (!m_done || !m_event.isEmpty() || !m_buffer.trimmed().isEmpty()) throw std::runtime_error("Incomplete streaming response");
}
JyLlmClient::JyLlmClient(QObject *parent, QNetworkAccessManager *transport)
    : QObject(parent), m_network(transport ? transport : new QNetworkAccessManager(this)) {
    m_timeout.setSingleShot(true);
    m_timeout.setInterval(60000);
    connect(&m_timeout, &QTimer::timeout, this, [this] { fail(tr("Request timed out")); });
}
JyLlmClient::~JyLlmClient() { cancel(); }
void JyLlmClient::cancel() {
    m_timeout.stop();
    if (!m_reply) return;
    auto reply = m_reply.data();
    m_reply.clear();
    reply->disconnect(this);
    reply->abort();
    reply->deleteLater();
}
void JyLlmClient::fail(const QString &message) { cancel(); emit failed(message); }
void JyLlmClient::consume(QNetworkReply *reply) {
    if (m_reply != reply) return;
    try {
        const auto chunks = m_decoder.feed(reply->readAll());
        for (const auto &text : chunks) {
            m_code += text;
            if (m_code.size() > 8 * 1024 * 1024) throw std::runtime_error("Generated code too large");
            emit chunk(text);
            if (m_reply != reply) return;
        }
    } catch (const std::exception &error) { fail(QString::fromUtf8(error.what())); }
}
void JyLlmClient::start(const QNetworkRequest &request, const QByteArray &body, bool anthropic) {
    cancel();
    m_decoder = JySseDecoder(anthropic);
    m_code.clear();
    auto *reply = m_network->post(request, body);
    m_reply = reply;
    m_timeout.start();
    connect(reply, &QNetworkReply::readyRead, this, [this, reply] {
        if (m_reply != reply) return;
        m_timeout.start();
        consume(reply);
    });
    connect(reply, &QNetworkReply::finished, this, [this, reply] {
        if (m_reply != reply) return;
        if (reply->error() != QNetworkReply::NoError) { fail(reply->errorString()); return; }
        consume(reply);
        if (m_reply != reply) return;
        try {
            m_decoder.finish();
            if (m_code.trimmed().isEmpty()) throw std::runtime_error("Empty code response");
        } catch (const std::exception &error) { fail(QString::fromUtf8(error.what())); return; }
        m_timeout.stop();
        m_reply.clear();
        reply->deleteLater();
        emit succeeded(m_code);
    });
}
