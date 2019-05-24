/********************************************************************************
 * Copyright 2019 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#include "WorkcellHighlighter.hpp"

WorkcellHighlighter::WorkcellHighlighter(QTextDocument *parent)
        : QSyntaxHighlighter(parent) {
    HighlightingRule rule;

    /*keywordFormat.setForeground(Qt::darkBlue);
    keywordFormat.setFontWeight(QFont::Bold);
    QStringList keywordPatterns;

    keywordPatterns << "\\bWorkCell\\b" << "\\bFrame\\b" << "\\brefframe\\b"
                   << "\\bRPY\\b" << "\\bPos\\b" << "\\bSerialDevice\\b"
                   << "\\bInclude\\b";

    foreach (const QString &pattern, keywordPatterns) {
       rule.pattern = QRegularExpression(pattern);
       rule.format = keywordFormat;
       highlightingRules.append(rule);
    }*/

    QString workcell_pattern = "\\bWorkCell\\b";
    QTextCharFormat workcell_format;
    QBrush workcell_brush;
    workcell_format.setForeground(QColor(0, 94, 136));
    workcell_format.setFontWeight(QFont::Bold);
    rule.pattern = QRegularExpression(workcell_pattern);
    rule.format = workcell_format;
    highlightingRules.append(rule);

    QString frame_pattern = "\\bFrame\\b";
    QTextCharFormat frame_format;
    QBrush frame_brush;
    frame_format.setForeground(QColor(217, 0, 93));
    frame_format.setFontWeight(QFont::Bold);
    rule.pattern = QRegularExpression(frame_pattern);
    rule.format = frame_format;
    highlightingRules.append(rule);

    QString drawable_pattern = "\\bDrawable\\b";
    QTextCharFormat drawable_format;
    QBrush drawable_brush;
    drawable_format.setForeground(QColor(0, 175, 95));
    drawable_format.setFontWeight(QFont::Bold);
    rule.pattern = QRegularExpression(drawable_pattern);
    rule.format = drawable_format;
    highlightingRules.append(rule);

    QString col_model_pattern = "\\bCollisionModel\\b";
    QTextCharFormat col_model_format;
    QBrush col_model_brush;
    col_model_format.setForeground(QColor(255, 87, 79));
    col_model_format.setFontWeight(QFont::Bold);
    rule.pattern = QRegularExpression(col_model_pattern);
    rule.format = col_model_format;
    highlightingRules.append(rule);

    QString property_pattern = "\\bProperty\\b";
    QTextCharFormat property_format;
    QBrush property_brush;
    property_format.setForeground(Qt::darkGreen);
    property_format.setFontWeight(QFont::Bold);
    rule.pattern = QRegularExpression(property_pattern);
    rule.format = property_format;
    highlightingRules.append(rule);

    QString serial_device_pattern = "\\bSerialDevice\\b";
    QTextCharFormat serial_device_format;
    QBrush serial_device_brush;
    serial_device_format.setForeground(QColor(1, 135, 134));
    serial_device_format.setFontWeight(QFont::Bold);
    rule.pattern = QRegularExpression(serial_device_pattern);
    rule.format = serial_device_format;
    highlightingRules.append(rule);

    QString include_pattern = "\\bInclude\\b";
    QTextCharFormat include_format;
    QBrush include_brush;
    include_format.setForeground(QColor(46, 125, 50));
    include_format.setFontWeight(QFont::Bold);
    rule.pattern = QRegularExpression(include_pattern);
    rule.format = include_format;
    highlightingRules.append(rule);

    QString rpy_pattern = "\\bRPY\\b";
    QTextCharFormat rpy_format;
    rpy_format.setForeground(QColor(142, 93, 171));
    rpy_format.setFontWeight(QFont::Bold);
    rule.pattern = QRegularExpression(rpy_pattern);
    rule.format = rpy_format;
    highlightingRules.append(rule);

    QString pos_pattern = "\\bPos\\b";
    QTextCharFormat pos_format;
    pos_format.setForeground(QColor(142, 93, 171));
    pos_format.setFontWeight(QFont::Bold);
    rule.pattern = QRegularExpression(pos_pattern);
    rule.format = pos_format;
    highlightingRules.append(rule);

    QString polytope_pattern = "\\bPolytope\\b";
    QTextCharFormat polytope_format;
    polytope_format.setForeground(QColor(142, 93, 171));
    polytope_format.setFontWeight(QFont::Bold);
    rule.pattern = QRegularExpression(polytope_pattern);
    rule.format = polytope_format;
    highlightingRules.append(rule);

    QString rgb_pattern = "\\bRGB\\b";
    QTextCharFormat rgb_format;
    rgb_format.setForeground(QColor(142, 93, 171));
    rgb_format.setFontWeight(QFont::Bold);
    rule.pattern = QRegularExpression(rgb_pattern);
    rule.format = rgb_format;
    highlightingRules.append(rule);

    classFormat.setFontWeight(QFont::Bold);
    classFormat.setForeground(Qt::darkMagenta);
    rule.pattern = QRegularExpression("\\bQ[A-Za-z]+\\b");
    rule.format = classFormat;
    highlightingRules.append(rule);

    singleLineCommentFormat.setForeground(QColor(135, 135, 135));
    rule.pattern = QRegularExpression("--(?!\\[)[^\n]*");
    rule.format = singleLineCommentFormat;
    highlightingRules.append(rule);

    multiLineCommentFormat.setForeground(Qt::red);
    quotationFormat.setForeground(Qt::darkGreen);

    attributeFormat.setForeground(QColor(215, 95, 0));
    rule.pattern = QRegularExpression(R"**((?<range2>[\w\d\-\:]+)[ ]*=[ ]*"[^"]*")**",
                                      QRegularExpression::DotMatchesEverythingOption |
                                      QRegularExpression::MultilineOption);
    rule.format = attributeFormat;
    highlightingRules.append(rule);

    rule.pattern = QRegularExpression(R"**((?<!\\)([\"'])(.+?)(?<!\\)\1)**",
                                      QRegularExpression::DotMatchesEverythingOption |
                                      QRegularExpression::MultilineOption);
    rule.format = quotationFormat;
    highlightingRules.append(rule);

    functionFormat.setFontItalic(true);
    functionFormat.setForeground(Qt::blue);
    rule.pattern = QRegularExpression("\\b[A-Za-z0-9_]+(?=\\()");
    rule.format = functionFormat;
    highlightingRules.append(rule);

    commentStartExpression = QRegularExpression("--\\[");
    commentEndExpression = QRegularExpression("\\]");
}

void WorkcellHighlighter::highlightBlock(const QString &text) {
            foreach (const HighlightingRule &rule, highlightingRules) {
            QRegularExpressionMatchIterator matchIterator = rule.pattern.globalMatch(text);
            while (matchIterator.hasNext()) {
                QRegularExpressionMatch match = matchIterator.next();
                setFormat(match.capturedStart(), match.capturedLength(), rule.format);
            }
        }
    setCurrentBlockState(0);

    int startIndex = 0;
    if (previousBlockState() != 1)
        startIndex = text.indexOf(commentStartExpression);

    while (startIndex >= 0) {
        QRegularExpressionMatch match = commentEndExpression.match(text, startIndex);
        int endIndex = match.capturedStart();
        int commentLength = 0;
        if (endIndex == -1) {
            setCurrentBlockState(1);
            commentLength = text.length() - startIndex;
        } else {
            commentLength = endIndex - startIndex
                            + match.capturedLength();
        }
        setFormat(startIndex, commentLength, multiLineCommentFormat);
        startIndex = text.indexOf(commentStartExpression, startIndex + commentLength);
    }
}
