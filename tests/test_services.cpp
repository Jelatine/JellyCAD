#include <gtest/gtest.h>
#include "services/jy_llm_client.h"
#include "services/jy_git_service.h"
#include "test_support.h"
#include <QTemporaryDir>
TEST(Services, Utf8AcrossEveryByteBoundary) {
    const QByteArray bytes = QString::fromUtf8("data: {\"choices\":[{\"delta\":{\"content\":\"你好🌏\"}}]}\r\n\r\ndata: [DONE]\n\n").toUtf8();
    JySseDecoder decoder;
    QString output;
    for (char byte : bytes) output += decoder.feed(QByteArray(1,byte)).join("");
    EXPECT_EQ(output,QString::fromUtf8("你好🌏"));
    EXPECT_NO_THROW(decoder.finish());
}
TEST(Services, RejectsIncompleteMalformedAndErrorStreams) {
    JySseDecoder incomplete;
    incomplete.feed("data: {\"choices\":[]}");
    EXPECT_THROW(incomplete.finish(),std::runtime_error);
    JySseDecoder invalid;
    EXPECT_THROW(invalid.feed("data: bad\n\n"),std::runtime_error);
    JySseDecoder providerError;
    EXPECT_THROW(providerError.feed("data: {\"error\":{\"message\":\"bad\"}}\n\n"),std::runtime_error);
}
TEST(Services, AnthropicEvents) {
    JySseDecoder decoder(true);
    EXPECT_EQ(decoder.feed("data: {\"type\":\"content_block_delta\",\"delta\":{\"text\":\"code\"}}\n\n").join(""),"code");
    decoder.feed("data: {\"type\":\"message_stop\"}\n\n");
    EXPECT_NO_THROW(decoder.finish());
}
TEST(Services, GitQueueRecoversFromMissingExecutable) {
    ensureApp();
    JyGitService service;
    QTemporaryDir temp;
    service.setWorkingDirectory(temp.path());
    QStringList results;
    QObject::connect(&service, &JyGitService::finished, [&](const QString &type,int code,QProcess::ExitStatus,const QString &,const QString &) {
        results << type;
        if(type=="missing") EXPECT_NE(code,0);
        else EXPECT_EQ(code,0);
    });
    service.enqueue("jellycad-nonexistent-command",{},"missing");
    service.enqueue("git",{"--version"},"version");
    ASSERT_TRUE(waitUntil([&] { return results.size()==2; }));
    EXPECT_EQ(results,QStringList({"missing","version"}));
}
TEST(Services, GitIgnoresPreviousWorkspaceResults) {
    ensureApp();
    JyGitService service;
    QTemporaryDir first,second;
    service.setWorkingDirectory(first.path());
    QStringList results;
    QObject::connect(&service, &JyGitService::finished, [&](const QString &type,int,QProcess::ExitStatus,const QString &,const QString &) { results << type; });
    service.enqueue("git",{"--version"},"old");
    service.setWorkingDirectory(second.path());
    service.enqueue("git",{"--version"},"new");
    EXPECT_TRUE(waitUntil([&] { return results.contains("new"); }));
    EXPECT_EQ(results,QStringList({"new"}));
}

namespace {
class FakeReply : public QNetworkReply {
public:
    FakeReply(QObject *parent,const QByteArray &body,bool stall,bool networkError) : QNetworkReply(parent) {
        open(QIODevice::ReadOnly);
        if (stall) return;
        QTimer::singleShot(1,this,[this,body,networkError] {
            if (m_stopped) return;
            if (networkError) setError(QNetworkReply::RemoteHostClosedError,"Connection lost");
            m_body=body;
            emit readyRead();
            if (m_stopped) return;
            setFinished(true); emit finished();
        });
    }
    void abort() override {
        m_stopped=true;
        setError(QNetworkReply::OperationCanceledError,"Cancelled");
        setFinished(true); emit finished();
    }
    qint64 bytesAvailable() const override { return m_body.size() + QNetworkReply::bytesAvailable(); }
protected:
    qint64 readData(char *data,qint64 maxSize) override {
        const auto count=std::min(maxSize,qint64(m_body.size()));
        if (!count) return -1;
        memcpy(data,m_body.constData(),size_t(count)); m_body.remove(0,int(count)); return count;
    }
private:
    QByteArray m_body;
    bool m_stopped=false;
};
class FakeTransport : public QNetworkAccessManager {
public:
    QByteArray body="data: {\"choices\":[{\"delta\":{\"content\":\"complete\"}}]}\n\ndata: [DONE]\n\n";
    bool stall=false,networkError=false;
protected:
    QNetworkReply *createRequest(Operation,const QNetworkRequest &,QIODevice *) override {
        return new FakeReply(this,body,stall,networkError);
    }
};
}
TEST(Services, LlmSuccessAndNetworkFailure) {
    ensureApp();
    FakeTransport transport;
    JyLlmClient client(nullptr,&transport);
    int successes=0,failures=0;
    QObject::connect(&client,&JyLlmClient::succeeded,[&](const QString &code) { ++successes; EXPECT_EQ(code,"complete"); });
    QObject::connect(&client,&JyLlmClient::failed,[&](const QString &) { ++failures; });
    const QNetworkRequest request(QUrl("http://offline.test"));
    client.start(request,{},false);
    ASSERT_TRUE(waitUntil([&] { return successes==1; }));
    transport.networkError=true;
    client.start(request,{},false);
    ASSERT_TRUE(waitUntil([&] { return failures==1; }));
    EXPECT_EQ(successes,1);
}
TEST(Services, LlmTimeoutCancellationAndRetry) {
    ensureApp();
    FakeTransport transport;
    JyLlmClient client(nullptr,&transport);
    client.setTimeout(20);
    int successes=0,failures=0;
    QObject::connect(&client,&JyLlmClient::succeeded,[&] { ++successes; });
    QObject::connect(&client,&JyLlmClient::failed,[&] { ++failures; });
    const QNetworkRequest request(QUrl("http://offline.test"));
    transport.stall=true;
    client.start(request,{},false);
    ASSERT_TRUE(waitUntil([&] { return failures==1; }));
    transport.stall=false;
    client.start(request,{},false);
    client.cancel();
    EXPECT_FALSE(waitUntil([&] { return successes>0 || failures>1; },30));
    client.start(request,{},false);
    client.start(request,{},false); // old reply must not publish completion
    ASSERT_TRUE(waitUntil([&] { return successes==1; }));
    EXPECT_EQ(failures,1);
}
