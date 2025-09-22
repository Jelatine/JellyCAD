/*
 * Copyright (c) 2024. Li Jianbin. All rights reserved.
 * MIT License
 */
#ifndef JY_CODE_EDITOR_H
#define JY_CODE_EDITOR_H

#include <QPainter>
#include <QPlainTextEdit>
#include <QRegularExpression>
#include <QSyntaxHighlighter>
#include <QTextBlock>

class JyCodeEditor : public QPlainTextEdit {
    Q_OBJECT
    QWidget *number_area_{nullptr};


    // 行号显示子界面
    class NumberArea : public QWidget {
    public:
        explicit NumberArea(JyCodeEditor *_editor) : QWidget(_editor), editor_(_editor) {}

        [[nodiscard]] QSize sizeHint() const override { return {editor_->number_area_width(), 0}; }

    protected:
        void paintEvent(QPaintEvent *event) override { editor_->paint_line_number(event); }

    private:
        JyCodeEditor *editor_{nullptr};
    };

    class Highlighter : public QSyntaxHighlighter {
    public:
        explicit Highlighter(QTextDocument *parent = nullptr) : QSyntaxHighlighter(parent) {}
        struct HighlightingRule {
            QRegularExpression pattern;
            QTextCharFormat format;
        };
        struct MultilineRule {
            QRegularExpression startPattern;
            QRegularExpression endPattern;
            QTextCharFormat format;
            int stateId;
        };
        void addRule(const HighlightingRule &rule) { highlightingRules.append(rule); }

        void addMultilineRule(const QString &startPattern, const QString &endPattern, const QTextCharFormat &format);

    protected:
        void highlightBlock(const QString &text) override;

    private:
        QVector<HighlightingRule> highlightingRules;
        QVector<MultilineRule> multilineRules;
        int nextStateId{1};
    };

    Highlighter *highlighter_;

    QStringList keyword_list_;

    void init_highlighter();

public:
    explicit JyCodeEditor(QWidget *parent = nullptr);

    void set_text(const QString &text);

    QString get_text() const;

    void setFilePath(const QString &filePath = {}) { m_filePath = filePath; }

    QString getFilePath() const { return m_filePath; }
    
    [[nodiscard]] QStringList keyword_list() const { return keyword_list_; }

private:

    int number_area_width();

    void paint_line_number(QPaintEvent *event);

    void resizeEvent(QResizeEvent *event) override;

public slots:

    void slot_update_number_width(int) { setViewportMargins(number_area_width() + 8, 0, 0, 0); }

    void slot_update_number_area(const QRect &rect, int dy);

protected:
    // 重写右键菜单事件
    void contextMenuEvent(QContextMenuEvent *event) override;

    void keyPressEvent(QKeyEvent *event) override {
        // 检测 Ctrl+/ 快捷键
        if (event->modifiers() == Qt::ControlModifier && event->key() == Qt::Key_Slash) {
            toggleComment();
            event->accept();
            return;
        }
        // 其他按键交给父类处理
        QPlainTextEdit::keyPressEvent(event);
    }

private slots:
    // 在文件资源管理器中显示
    void showInExplorer();

private:
    bool is_CRLF{false};

    QString m_filePath;
    QString m_vscodeCmd;

    void toggleComment();
};

#endif//JY_CODE_EDITOR_H
