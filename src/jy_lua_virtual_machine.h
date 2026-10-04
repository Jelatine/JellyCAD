#pragma once
#include "runtime/jy_runtime.h"
#include <QObject>
#include <QThread>
#include <QTimer>
#include <condition_variable>
#include <deque>
#include <mutex>

// GUI-thread controller. The worker never waits on a GUI signal delivery.
class JyLuaVirtualMachine : public QObject {
    Q_OBJECT
public:
    enum class State { Idle, Running, Stopping };
    explicit JyLuaVirtualMachine(QObject *parent = nullptr);
    ~JyLuaVirtualMachine() override;
    bool executeScript(const QString &fileName);
    bool exec_code(const QString &code, const QString &directory = {});
    bool submit(jelly::RunRequest request);
    void stopScript();
    bool isRunning() const { return m_state != State::Idle; }
    State state() const { return m_state; }
    quint64 runId() const { return m_runId; }
    size_t queuePeak() const { return m_peak.load(); }
    static constexpr size_t QueueCapacity = 256;
signals:
    void scriptStarted();
    void scriptFinished(const QString &message);
    void scriptError(const QString &error);
    void scriptOutput(const QString &output);
    void displayShape(const JyShape &shape);
    void displayAxes(const JyAxes &axes);
    void batchFinished();
    void completed(const jelly::RunResult &result);
private:
    void drain();
    State m_state = State::Idle;
    QThread *m_worker = nullptr;
    QTimer m_timer;
    std::atomic<bool> m_cancel{false};
    std::mutex m_mutex;
    std::condition_variable m_capacity;
    struct Event { quint64 id; jelly::RunEvent value; };
    std::deque<Event> m_events;
    jelly::RunResult m_result;
    quint64 m_runId = 0;
    std::atomic<size_t> m_peak{0};
};
