#pragma once
#include <QObject>
#include <QProcess>
#include <QPointer>
#include <QQueue>

class JyGitService : public QObject {
    Q_OBJECT
public:
    explicit JyGitService(QObject *parent = nullptr) : QObject(parent) {}
    ~JyGitService() override;
    void setWorkingDirectory(const QString &directory);
    void enqueue(const QString &program, const QStringList &args, const QString &type, const QString &directory = {});
signals:
    void finished(const QString &type, int exitCode, QProcess::ExitStatus status, const QString &output, const QString &error);
private:
    struct Command { QString program; QStringList args; QString type, directory; quint64 generation; };
    void next();
    void complete(QProcess *process, const Command &command, int exitCode, QProcess::ExitStatus status);
    QString m_directory;
    quint64 m_generation = 0;
    QQueue<Command> m_queue;
    QPointer<QProcess> m_process;
};
