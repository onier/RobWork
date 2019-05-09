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

#ifndef WORKCELLHIGHLIGHTER_HPP_
#define WORKCELLHIGHLIGHTER_HPP_

#ifndef HIGHLIGHTER_H
#define HIGHLIGHTER_H

#include <QSyntaxHighlighter>
#include <QTextCharFormat>
#include <QRegularExpression>

class QTextDocument;

class WorkcellHighlighter : public QSyntaxHighlighter
{
   Q_OBJECT

public:
   WorkcellHighlighter(QTextDocument *parent = 0);

protected:
   void highlightBlock(const QString &text);

private:
   struct HighlightingRule
   {
       QRegularExpression pattern;
       QTextCharFormat format;
   };
   QVector<HighlightingRule> highlightingRules;

   QRegularExpression commentStartExpression;
   QRegularExpression commentEndExpression;

   QTextCharFormat keywordFormat;
   QTextCharFormat classFormat;
   QTextCharFormat singleLineCommentFormat;
   QTextCharFormat multiLineCommentFormat;
   QTextCharFormat quotationFormat;
   QTextCharFormat attributeFormat;
   QTextCharFormat functionFormat;
};

#endif
#endif /* WORKCELLHIGHLIGHTER_HPP_ */
