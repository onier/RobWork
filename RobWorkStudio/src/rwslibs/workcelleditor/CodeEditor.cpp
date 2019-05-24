/********************************************************************************
 * Copyright 2018 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
 * Faculty of Engineering, University of Southern Denmark
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ********************************************************************************/

#include "CodeEditor.hpp"

#include <QAbstractItemView>
#include <QCompleter>
#include <QPainter>
#include <QScrollBar>
#include <QTextBlock>

WCCodeEditor::WCCodeEditor(QWidget *parent) :
        QPlainTextEdit(parent), _c(NULL) {
    lineNumberArea = new LineNumberArea(this);

    connect(this, SIGNAL(blockCountChanged(int)), this, SLOT(updateLineNumberAreaWidth(int)));
    connect(this, SIGNAL(updateRequest(
                                 const QRect &, int)), this, SLOT(updateLineNumberArea(
                                                                          const QRect &, int)));
    connect(this, SIGNAL(cursorPositionChanged()), this, SLOT(highlightCurrentLine()));

    updateLineNumberAreaWidth(0);
    highlightCurrentLine();
}


int WCCodeEditor::lineNumberAreaWidth() {
    int digits = 1;
    int max = qMax(1, blockCount());
    while (max >= 10) {
        max /= 10;
        ++digits;
    }

    int space = 10 + fontMetrics().width(QLatin1Char('9')) * digits;

    return space;
}


void WCCodeEditor::updateLineNumberAreaWidth(int /* newBlockCount */) {
    setViewportMargins(lineNumberAreaWidth(), 0, 0, 0);
}


void WCCodeEditor::updateLineNumberArea(const QRect &rect, int dy) {
    if (dy)
        lineNumberArea->scroll(0, dy);
    else
        lineNumberArea->update(0, rect.y(), lineNumberArea->width(), rect.height());

    if (rect.contains(viewport()->rect()))
        updateLineNumberAreaWidth(0);
}


void WCCodeEditor::resizeEvent(QResizeEvent *e) {
    QPlainTextEdit::resizeEvent(e);

    QRect cr = contentsRect();
    lineNumberArea->setGeometry(QRect(cr.left(), cr.top(), lineNumberAreaWidth(), cr.height()));
}


void WCCodeEditor::highlightCurrentLine() {
    QList<QTextEdit::ExtraSelection> extraSelections;

    if (!isReadOnly()) {
        QTextEdit::ExtraSelection selection;
        QColor *lineNumberHighligtColor = new QColor(237, 238, 244);
        QColor lineColor = lineNumberHighligtColor->rgb(); //  QColor(Qt::yellow).lighter(160);

        selection.format.setBackground(lineColor);
        selection.format.setProperty(QTextFormat::FullWidthSelection, true);
        selection.cursor = textCursor();
        selection.cursor.clearSelection();
        extraSelections.append(selection);
    }

    setExtraSelections(extraSelections);
}


void WCCodeEditor::lineNumberAreaPaintEvent(QPaintEvent *event) {
    QPainter painter(lineNumberArea);
    QColor *lineNumberBgColor = new QColor(238, 238, 238);
    painter.fillRect(event->rect(), lineNumberBgColor->rgb());

    QTextBlock block = firstVisibleBlock();
    int blockNumber = block.blockNumber();
    int top = (int) blockBoundingGeometry(block).translated(contentOffset()).top();
    int bottom = top + (int) blockBoundingRect(block).height();

    while (block.isValid() && top <= event->rect().bottom()) {
        if (block.isVisible() && bottom >= event->rect().top()) {
            QString number = QString::number(blockNumber + 1);

            /*
            LineState lstate = getLineState(blockNumber);
            if(lstate==Nothing){
                painter.setPen(Qt::black);
            } else if(Executed){
                painter.setPen(Qt::green);
            } else if(HighLighted){
                painter.setPen(Qt::yellow);
            } else if(ExecutedError){
                painter.setPen(Qt::red);
            }*/
            QColor *lineNumberFgColor = new QColor(178, 178, 178);
            painter.setPen(lineNumberFgColor->rgb());
            painter.drawText(0, top, lineNumberArea->width() - 5, fontMetrics().height(),
                             Qt::AlignRight, number);
        }

        block = block.next();
        top = bottom;
        bottom = top + (int) blockBoundingRect(block).height();
        ++blockNumber;
    }
}

WCCodeEditor::LineState WCCodeEditor::getLineState(int lineNr) {
    if ((int) _executedLines.size() <= lineNr || lineNr < 0)
        return Nothing;

    return _executedLines[lineNr];
}

void WCCodeEditor::setCompleter(QCompleter *completer) {
    if (_c)
        QObject::disconnect(_c, 0, this, 0);

    _c = completer;

    if (!_c)
        return;

    _c->setWidget(this);
    _c->setCompletionMode(QCompleter::PopupCompletion);

    _c->setCaseSensitivity(Qt::CaseInsensitive);
    QObject::connect(_c, SIGNAL(activated(
                                        const QString&)),
                     this, SLOT(insertCompletion(
                                        const QString&)));
}

QCompleter *WCCodeEditor::completer() const {
    return _c;
}

void WCCodeEditor::insertXMLTextUnderCursor(const QString &text) {
    if (_c->widget() != this)
        return;
    QTextCursor tc = textCursor();
    tc.insertText(text);
    setTextCursor(tc);
}


void WCCodeEditor::insertCompletion(const QString &completion) {
    if (_c->widget() != this)
        return;
    QTextCursor tc = textCursor();
    int extra = completion.length() - _c->completionPrefix().length();
    tc.movePosition(QTextCursor::Left);
    tc.movePosition(QTextCursor::EndOfWord);
    tc.insertText(completion.right(extra));
    setTextCursor(tc);
}

QString WCCodeEditor::textUnderCursor() const {
    QTextCursor tc = textCursor();
    tc.select(QTextCursor::WordUnderCursor);
    return tc.selectedText();
}

void WCCodeEditor::focusInEvent(QFocusEvent *e) {
    if (_c)
        _c->setWidget(this);
    QPlainTextEdit::focusInEvent(e);
}

void WCCodeEditor::keyPressEvent(QKeyEvent *e) {
    if (_c && _c->popup()->isVisible()) {
        // The following keys are forwarded by the completer to the widget
        switch (e->key()) {
            case Qt::Key_Enter:
            case Qt::Key_Return:
            case Qt::Key_Escape:
            case Qt::Key_Tab:
            case Qt::Key_Backtab:
                e->ignore();
                return; // let the completer do default behavior
            default:
                break;
        }
    }

    bool isShortcut = ((e->modifiers() & Qt::ControlModifier) && e->key() == Qt::Key_E); // CTRL+E
    if (!_c || !isShortcut) // do not process the shortcut when we have a completer
        QPlainTextEdit::keyPressEvent(e);

    const bool ctrlOrShift = e->modifiers() & (Qt::ControlModifier | Qt::ShiftModifier);
    if (!_c || (ctrlOrShift && e->text().isEmpty()))
        return;

    static QString eow("~!@#$%^&*()_+{}|:\"<>?,/;'[]\\-="); // end of word
    bool hasModifier = (e->modifiers() != Qt::NoModifier) && !ctrlOrShift;
    QString completionPrefix = textUnderCursor();

    if (!isShortcut && (hasModifier || e->text().isEmpty() || completionPrefix.length() < 1
                        || eow.contains(e->text().right(1)))) {
        _c->popup()->hide();
        return;
    }

    if (completionPrefix != _c->completionPrefix()) {
        _c->setCompletionPrefix(completionPrefix);
        _c->popup()->setCurrentIndex(_c->completionModel()->index(0, 0));
    }

    QRect cr = cursorRect();
    cr.setWidth(_c->popup()->sizeHintForColumn(0)
                + _c->popup()->verticalScrollBar()->sizeHint().width());
    _c->complete(cr); // popup it up!
    //_c->popup()->show();
}
