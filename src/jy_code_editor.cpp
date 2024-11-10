/*
 * Copyright (c) 2024. Li Jianbin. All rights reserved.
 * MIT License
 */
#include "jy_code_editor.h"

JyCodeEditor::JyCodeEditor(QWidget *parent) : QPlainTextEdit(parent), number_area_(new NumberArea(this)) {
    setWordWrapMode(QTextOption::NoWrap);
    QFont t_font = font();
    t_font.setFamily("Courier New");
    t_font.setPointSize(12);
    setFont(t_font);
    QFontMetrics metrics(t_font);
    setTabStopDistance(4 * metrics.averageCharWidth());
    auto highlighter = new Highlighter(this->document());
    connect(this, &JyCodeEditor::blockCountChanged, this, &JyCodeEditor::slot_update_number_width);
    connect(this, &JyCodeEditor::updateRequest, this, &JyCodeEditor::slot_update_number_area);
    slot_update_number_width(0);
}


int JyCodeEditor::number_area_width() {
    int digits = 1;
    int max = qMax(1, blockCount());
    while (max >= 10) {
        max /= 10;
        ++digits;
    }
    int space = 3 + 16 + 3 + fontMetrics().horizontalAdvance(QLatin1Char('9')) * digits;
    return space;
}

void JyCodeEditor::paint_line_number(QPaintEvent *event) {
    if (!number_area_) { return; }
    QPainter painter(number_area_);
    painter.setFont(this->font());
    painter.fillRect(event->rect(), QColor::fromRgb(240, 240, 240));   // 行号栏底纹色


    QTextBlock t_first_visible_block = firstVisibleBlock(); // 第一个可看到的区间
    int t_block_number = t_first_visible_block.blockNumber();   // 第一个区间号
    int t_top = qRound(blockBoundingGeometry(t_first_visible_block).translated(contentOffset()).top());
    int t_bottom = t_top + qRound(blockBoundingRect(t_first_visible_block).height());

    while (t_first_visible_block.isValid() && t_top <= event->rect().bottom()) {
        if (t_first_visible_block.isVisible() && t_bottom >= event->rect().top()) {
            int t_number = t_block_number + 1;
            painter.setPen(QColor::fromRgb(150, 150, 150));    // 行号栏字体色
            const auto rect = QRect(0, t_top, number_area_->width(), fontMetrics().height());
            painter.drawText(rect, Qt::AlignRight, QString::number(t_number));
        }
        t_first_visible_block = t_first_visible_block.next();
        t_top = t_bottom;
        t_bottom = t_top + qRound(blockBoundingRect(t_first_visible_block).height());
        ++t_block_number;
    }
}


void JyCodeEditor::resizeEvent(QResizeEvent *event) {
    if (!number_area_) { return; }
    QPlainTextEdit::resizeEvent(event);
    QRect cr = contentsRect();
    number_area_->setGeometry(QRect(cr.left(), cr.top(), number_area_width(), cr.height()));
}

void JyCodeEditor::slot_update_number_area(const QRect &rect, int dy) {
    if (!number_area_) { return; }
    if (dy) {
        number_area_->scroll(0, dy);
    } else {
        number_area_->update(0, rect.y(), number_area_->width(), rect.height());
    }
    if (rect.contains(viewport()->rect())) {
        slot_update_number_width(0);
    } else {}
}


JyCodeEditor::Highlighter::Highlighter(QTextDocument *parent) : QSyntaxHighlighter(parent) {
    // 关键字
    m_append_keyword_list(QStringList() << "and" << "break" << "do" << "else" << "elseif" << "end"
                                        << "false" << "for" << "function" << "if" << "in"
                                        << "local" << "or" << "nil" << "not" << "repeat"
                                        << "return" << "then" << "true" << "until" << "while",
                          "#2985c7");  // 蓝色
    // 内置功能
    m_append_keyword_list(
            QStringList() << "assert" << "collectgarbage" << "dofile" << "error" << "getmetatable" << "ipairs"
                          << "load" << "loadfile" << "next" << "pairs" << "pcall" << "print" << "rawequal"
                          << "rawget" << "rawset" << "require" << "select" << "setmetatable" << "tonumber"
                          << "tostring" << "type" << "xpcall" << "warn",
            "#3369FF");  // 浅蓝色
    QStringList custom_class;
    custom_class << "box" << "cylinder" << "cone" << "sphere" << "edge" << "wire" << "polygon" << "face";
    m_append_keyword_list(custom_class, "#F92671");  // 浅红色
    QStringList custom_function;
    custom_function << ":type" << ":fuse" << ":cut" << ":common" << ":fillet" << ":chamfer" << ":translate"
                    << ":rotate" << ":locate" << ":color" << ":prism";
    m_append_keyword_list(custom_function, "#3369FF");  // 浅蓝色
    QStringList custom_global_func;
    custom_global_func << "export_stl" << "export_step" << "show";
    m_append_keyword_list(custom_global_func, "#800080");  // 浅紫色
    QTextCharFormat t_text_char_format;
    HighlightingRule rule;
    // 字符串（双引号）
    t_text_char_format.setForeground(QBrush("#42E2CB"));
    rule.pattern = QRegularExpression(QStringLiteral("\".*\""));
    rule.format = t_text_char_format;
    highlightingRules.append(rule);
    // 字符串（单引号）
    rule.pattern = QRegularExpression(QStringLiteral("'.*'"));
    rule.format = t_text_char_format;
    highlightingRules.append(rule);
    // 注释
    t_text_char_format.setForeground(QBrush("#008000"));    // 绿色
    rule.pattern = QRegularExpression(QStringLiteral("--.*"));
    rule.format = t_text_char_format;
    highlightingRules.append(rule);
    // 多行注释
    m_comment_start_expression = QRegularExpression(QStringLiteral("--\\[\\["));
    m_comment_end_expression = QRegularExpression(QStringLiteral("--\\]\\]"));

    m_multi_line_comment_format.setForeground(QBrush("#008000"));    // 注释绿色
}


void JyCodeEditor::Highlighter::m_append_keyword_list(const QStringList &_list, const QColor &_color) {
    HighlightingRule rule;
    QTextCharFormat t_text_char_format;
    t_text_char_format.setForeground(_color);
    for (const QString &pattern: _list) {
        rule.pattern = QRegularExpression("\\b" + pattern + "\\b");
        rule.format = t_text_char_format;
        highlightingRules.append(rule);
    }
}


void JyCodeEditor::Highlighter::highlightBlock(const QString &text) {
    for (const HighlightingRule &rule: qAsConst(highlightingRules)) {
        QRegularExpressionMatchIterator matchIterator = rule.pattern.globalMatch(text);
        while (matchIterator.hasNext()) {
            QRegularExpressionMatch match = matchIterator.next();
            setFormat(match.capturedStart(), match.capturedLength(), rule.format);
        }
    }
    QTextCharFormat t_error_format;
    t_error_format.setFontUnderline(true);
    t_error_format.setUnderlineColor(Qt::red);
    t_error_format.setUnderlineStyle(QTextCharFormat::SpellCheckUnderline);
    setCurrentBlockState(0);
    int startIndex = 0;
    if (previousBlockState() != 1) { startIndex = text.indexOf(m_comment_start_expression); }
    while (startIndex >= 0) {
        QRegularExpressionMatch match = m_comment_end_expression.match(text, startIndex);
        int endIndex = match.capturedStart();
        int commentLength = 0;
        if ((endIndex == -1) || (previousBlockState() != 1)) {
            setCurrentBlockState(1);
            commentLength = text.length() - startIndex;
        } else {
            commentLength = endIndex - startIndex + match.capturedLength();
        }
        setFormat(startIndex, commentLength, m_multi_line_comment_format);
        startIndex = text.indexOf(m_comment_start_expression, startIndex + commentLength);
    }

}