#include "jy_git_service.h"
#include <QDir>
#include <QTimer>
JyGitService::~JyGitService() {
    if (m_process) {
        m_process->disconnect(this);
        m_process->kill();
        m_process->waitForFinished(1000);
    }
}
void JyGitService::setWorkingDirectory(const QString &directory) {
    m_directory = QDir(directory).absolutePath();
    ++m_generation;
    m_queue.clear(); // Active command may complete, but cannot update the new workspace.
}
void JyGitService::enqueue(const QString &program, const QStringList &args, const QString &type, const QString &directory) {
    m_queue.enqueue({program, args, type, directory.isEmpty() ? m_directory : QDir(directory).absolutePath(), m_generation});
    next();
}
void JyGitService::next() {
    if (m_process || m_queue.isEmpty()) return;
    const auto command = m_queue.dequeue();
    auto *process = new QProcess(this);
    m_process = process;
    process->setWorkingDirectory(command.directory);
    connect(process, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished), this,
        [this, process, command](int code, QProcess::ExitStatus status) { complete(process, command, code, status); });
    connect(process, &QProcess::errorOccurred, this, [this, process, command](QProcess::ProcessError error) {
        if (error == QProcess::FailedToStart) complete(process, command, -1, QProcess::CrashExit);
    });
    auto *timeout = new QTimer(process);
    timeout->setSingleShot(true);
    connect(timeout, &QTimer::timeout, process, &QProcess::kill);
    timeout->start(120000);
    process->start(command.program, command.args);
}
void JyGitService::complete(QProcess *process, const Command &command, int exitCode, QProcess::ExitStatus status) {
    if (m_process != process) return;
    const auto output = QString::fromUtf8(process->readAllStandardOutput());
    auto error = QString::fromUtf8(process->readAllStandardError());
    if (status == QProcess::CrashExit && error.isEmpty()) error = process->errorString();
    process->disconnect(this);
    process->deleteLater();
    m_process.clear();
    if (command.generation == m_generation) emit finished(command.type, exitCode, status, output, error);
    next();
}
