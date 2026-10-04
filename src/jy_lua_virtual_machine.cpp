#include "jy_lua_virtual_machine.h"
#include <QDebug>
#include <QElapsedTimer>
#include <QFileInfo>

JyLuaVirtualMachine::JyLuaVirtualMachine(QObject *parent) : QObject(parent) {
    m_timer.setInterval(16);
    connect(&m_timer, &QTimer::timeout, this, &JyLuaVirtualMachine::drain);
}
JyLuaVirtualMachine::~JyLuaVirtualMachine() {
    // Window shutdown normally waits asynchronously for completed(). This fallback
    // remains safe because the worker cannot block on the GUI event loop.
    stopScript();
    if (m_worker) { m_worker->wait(); delete m_worker; }
}
bool JyLuaVirtualMachine::executeScript(const QString &file) {
    jelly::RunRequest request;
    request.source = QFileInfo(file).absoluteFilePath().toStdString();
    return submit(std::move(request));
}
bool JyLuaVirtualMachine::exec_code(const QString &code, const QString &directory) {
    jelly::RunRequest request;
    request.source = code.toStdString();
    request.directory = directory.toStdString();
    request.isFile = false;
    return submit(std::move(request));
}
bool JyLuaVirtualMachine::submit(jelly::RunRequest request) {
    Q_ASSERT(QThread::currentThread() == thread());
    if (isRunning()) return false;
    request.id = ++m_runId;
    m_cancel = false;
    m_peak = 0;
    m_state = State::Running;
    m_worker = QThread::create([this, request] {
        m_result = jelly::execute(request, m_cancel, [this, id = request.id](jelly::RunEvent value) {
            // Bound individual log entries as well as the number of events.
            if (auto *text = std::get_if<std::string>(&value); text && text->size() > 16384) text->resize(16384);
            std::unique_lock<std::mutex> lock(m_mutex);
            m_capacity.wait(lock, [this] { return m_cancel.load() || m_events.size() < QueueCapacity; });
            if (m_cancel.load()) return;
            m_events.push_back({id, std::move(value)});
            m_peak = std::max(m_peak.load(), m_events.size());
        });
    });
    connect(m_worker, &QThread::finished, this, [this] {
        m_worker->wait(); // finished has been delivered; this does not wait on UI work.
        m_timer.stop();
        while (!m_events.empty()) drain();
        auto *finishedWorker = m_worker;
        m_worker = nullptr;
        finishedWorker->deleteLater();
        const auto result = m_result;
        m_state = State::Idle;
        qDebug() << "Script" << result.id << "elapsed ms" << result.elapsedMs << "queue peak" << m_peak.load();
        const auto message = QString::fromStdString(result.message) + QString(" (%1 ms)").arg(result.elapsedMs);
        if (result.status == jelly::RunStatus::Failed) emit scriptError(message);
        else emit scriptFinished(message);
        emit completed(result);
    });
    emit scriptStarted();
    m_worker->start();
    m_timer.start();
    return true;
}
void JyLuaVirtualMachine::stopScript() {
    if (!isRunning()) return;
    m_state = State::Stopping;
    m_cancel = true;
    m_capacity.notify_all();
}
void JyLuaVirtualMachine::drain() {
    std::deque<Event> batch;
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        const auto count = std::min(size_t(64), m_events.size());
        for (size_t i = 0; i < count; ++i) { batch.push_back(std::move(m_events.front())); m_events.pop_front(); }
    }
    m_capacity.notify_all();
    QElapsedTimer timer;
    timer.start();
    for (const auto &event : batch) {
        if (event.id != m_runId || m_cancel.load()) continue;
        if (auto *text = std::get_if<std::string>(&event.value)) emit scriptOutput(QString::fromStdString(*text));
        else if (auto *shape = std::get_if<JyShape>(&event.value)) emit displayShape(*shape);
        else emit displayAxes(std::get<JyAxes>(event.value));
    }
    if (!batch.empty()) {
        emit batchFinished();
        if (timer.elapsed() > 50) qDebug() << "Display batch ms" << timer.elapsed() << "events" << batch.size();
    }
}
