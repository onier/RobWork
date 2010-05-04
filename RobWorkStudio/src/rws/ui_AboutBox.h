/********************************************************************************
** Form generated from reading ui file 'AboutBox.ui'
**
** Created: Fri 19. Feb 21:20:53 2010
**      by: Qt User Interface Compiler version 4.4.3
**
** WARNING! All changes made in this file will be lost when recompiling ui file!
********************************************************************************/

#ifndef UI_ABOUTBOX_H
#define UI_ABOUTBOX_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QDialog>
#include <QtGui/QGridLayout>
#include <QtGui/QGroupBox>
#include <QtGui/QHBoxLayout>
#include <QtGui/QLabel>
#include <QtGui/QPushButton>
#include <QtGui/QSpacerItem>
#include <QtGui/QTabWidget>
#include <QtGui/QVBoxLayout>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_AboutBoxClass
{
public:
    QVBoxLayout *verticalLayout;
    QLabel *lblLogo;
    QLabel *label;
    QWidget *widget_2;
    QGridLayout *gridLayout;
    QLabel *lblVersionText;
    QLabel *lblVersion;
    QLabel *Revision;
    QLabel *lblRevision;
    QLabel *label_2;
    QLabel *label_3;
    QGroupBox *grpAddons;
    QVBoxLayout *verticalLayout_2;
    QTabWidget *tabWidget;
    QWidget *tab;
    QWidget *tab_2;
    QWidget *widget;
    QHBoxLayout *horizontalLayout;
    QSpacerItem *horizontalSpacer_2;
    QPushButton *btnOk;
    QSpacerItem *horizontalSpacer;

    void setupUi(QDialog *AboutBoxClass)
    {
    if (AboutBoxClass->objectName().isEmpty())
        AboutBoxClass->setObjectName(QString::fromUtf8("AboutBoxClass"));
    AboutBoxClass->resize(400, 377);
    verticalLayout = new QVBoxLayout(AboutBoxClass);
    verticalLayout->setSpacing(6);
    verticalLayout->setMargin(11);
    verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
    lblLogo = new QLabel(AboutBoxClass);
    lblLogo->setObjectName(QString::fromUtf8("lblLogo"));
    lblLogo->setAlignment(Qt::AlignCenter|Qt::AlignHCenter|Qt::AlignLeading|Qt::AlignLeft|Qt::AlignVCenter);

    verticalLayout->addWidget(lblLogo);

    label = new QLabel(AboutBoxClass);
    label->setObjectName(QString::fromUtf8("label"));
    QFont font;
    font.setFamily(QString::fromUtf8("MS Shell Dlg 2"));
    font.setPointSize(15);
    font.setBold(true);
    font.setItalic(true);
    font.setUnderline(false);
    font.setWeight(75);
    font.setStrikeOut(false);
    label->setFont(font);
    label->setAlignment(Qt::AlignCenter|Qt::AlignHCenter|Qt::AlignLeading|Qt::AlignLeft|Qt::AlignVCenter);

    verticalLayout->addWidget(label);

    widget_2 = new QWidget(AboutBoxClass);
    widget_2->setObjectName(QString::fromUtf8("widget_2"));
    gridLayout = new QGridLayout(widget_2);
    gridLayout->setSpacing(6);
    gridLayout->setMargin(11);
    gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
    lblVersionText = new QLabel(widget_2);
    lblVersionText->setObjectName(QString::fromUtf8("lblVersionText"));
    QFont font1;
    font1.setFamily(QString::fromUtf8("MS Shell Dlg 2"));
    font1.setPointSize(10);
    font1.setBold(false);
    font1.setItalic(false);
    font1.setUnderline(false);
    font1.setWeight(50);
    font1.setStrikeOut(false);
    lblVersionText->setFont(font1);
    lblVersionText->setAlignment(Qt::AlignCenter|Qt::AlignHCenter|Qt::AlignLeading|Qt::AlignLeft|Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);

    gridLayout->addWidget(lblVersionText, 1, 0, 1, 1);

    lblVersion = new QLabel(widget_2);
    lblVersion->setObjectName(QString::fromUtf8("lblVersion"));
    lblVersion->setFont(font1);

    gridLayout->addWidget(lblVersion, 1, 1, 1, 1);

    Revision = new QLabel(widget_2);
    Revision->setObjectName(QString::fromUtf8("Revision"));
    Revision->setFont(font1);
    Revision->setAlignment(Qt::AlignLeading|Qt::AlignLeft|Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);

    gridLayout->addWidget(Revision, 2, 0, 1, 1);

    lblRevision = new QLabel(widget_2);
    lblRevision->setObjectName(QString::fromUtf8("lblRevision"));
    lblRevision->setFont(font1);

    gridLayout->addWidget(lblRevision, 2, 1, 1, 1);


    verticalLayout->addWidget(widget_2);

    label_2 = new QLabel(AboutBoxClass);
    label_2->setObjectName(QString::fromUtf8("label_2"));
    label_2->setAlignment(Qt::AlignCenter|Qt::AlignHCenter|Qt::AlignLeading|Qt::AlignLeft|Qt::AlignVCenter);

    verticalLayout->addWidget(label_2);

    label_3 = new QLabel(AboutBoxClass);
    label_3->setObjectName(QString::fromUtf8("label_3"));
    label_3->setAlignment(Qt::AlignCenter|Qt::AlignHCenter|Qt::AlignLeading|Qt::AlignLeft|Qt::AlignVCenter);

    verticalLayout->addWidget(label_3);

    grpAddons = new QGroupBox(AboutBoxClass);
    grpAddons->setObjectName(QString::fromUtf8("grpAddons"));
    verticalLayout_2 = new QVBoxLayout(grpAddons);
    verticalLayout_2->setSpacing(6);
    verticalLayout_2->setMargin(11);
    verticalLayout_2->setObjectName(QString::fromUtf8("verticalLayout_2"));
    tabWidget = new QTabWidget(grpAddons);
    tabWidget->setObjectName(QString::fromUtf8("tabWidget"));
    tab = new QWidget();
    tab->setObjectName(QString::fromUtf8("tab"));
    tabWidget->addTab(tab, QString());
    tab_2 = new QWidget();
    tab_2->setObjectName(QString::fromUtf8("tab_2"));
    tabWidget->addTab(tab_2, QString());

    verticalLayout_2->addWidget(tabWidget);


    verticalLayout->addWidget(grpAddons);

    widget = new QWidget(AboutBoxClass);
    widget->setObjectName(QString::fromUtf8("widget"));
    horizontalLayout = new QHBoxLayout(widget);
    horizontalLayout->setSpacing(6);
    horizontalLayout->setMargin(11);
    horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
    horizontalSpacer_2 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

    horizontalLayout->addItem(horizontalSpacer_2);

    btnOk = new QPushButton(widget);
    btnOk->setObjectName(QString::fromUtf8("btnOk"));

    horizontalLayout->addWidget(btnOk);

    horizontalSpacer = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

    horizontalLayout->addItem(horizontalSpacer);


    verticalLayout->addWidget(widget);


    retranslateUi(AboutBoxClass);

    tabWidget->setCurrentIndex(0);


    QMetaObject::connectSlotsByName(AboutBoxClass);
    } // setupUi

    void retranslateUi(QDialog *AboutBoxClass)
    {
    AboutBoxClass->setWindowTitle(QApplication::translate("AboutBoxClass", "AboutBox", 0, QApplication::UnicodeUTF8));
    label->setText(QApplication::translate("AboutBoxClass", "RobWork", 0, QApplication::UnicodeUTF8));
    lblVersionText->setText(QApplication::translate("AboutBoxClass", "Version", 0, QApplication::UnicodeUTF8));
    lblVersion->setText(QApplication::translate("AboutBoxClass", "TextLabel", 0, QApplication::UnicodeUTF8));
    Revision->setText(QApplication::translate("AboutBoxClass", "Revision", 0, QApplication::UnicodeUTF8));
    lblRevision->setText(QApplication::translate("AboutBoxClass", "TextLabel", 0, QApplication::UnicodeUTF8));
    label_2->setText(QApplication::translate("AboutBoxClass", "\302\251 2009  The Robotics Group<br>The Maersk Mc-Kinney Moller Institute<br>Faculty of Engineering, University of Southern Denmark", 0, QApplication::UnicodeUTF8));
    label_3->setText(QApplication::translate("AboutBoxClass", "Licensed under the Apache License, Version 2.0<br>http://www.apache.org/licenses/LICENSE-2.0", 0, QApplication::UnicodeUTF8));
    grpAddons->setTitle(QApplication::translate("AboutBoxClass", "Addons", 0, QApplication::UnicodeUTF8));
    tabWidget->setTabText(tabWidget->indexOf(tab), QApplication::translate("AboutBoxClass", "Tab 1", 0, QApplication::UnicodeUTF8));
    tabWidget->setTabText(tabWidget->indexOf(tab_2), QApplication::translate("AboutBoxClass", "Tab 2", 0, QApplication::UnicodeUTF8));
    btnOk->setText(QApplication::translate("AboutBoxClass", "Ok", 0, QApplication::UnicodeUTF8));
    Q_UNUSED(AboutBoxClass);
    } // retranslateUi

};

namespace Ui {
    class AboutBoxClass: public Ui_AboutBoxClass {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_ABOUTBOX_H
