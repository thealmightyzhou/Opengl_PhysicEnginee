/********************************************************************************
** Form generated from reading UI file 'ObjLoad_1.ui'
**
** Created by: Qt User Interface Compiler version 5.12.0
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_OBJLOAD_1_H
#define UI_OBJLOAD_1_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_ObjLoad_1Class
{
public:
    QMenuBar *menuBar;
    QToolBar *mainToolBar;
    QWidget *centralWidget;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *ObjLoad_1Class)
    {
        if (ObjLoad_1Class->objectName().isEmpty())
            ObjLoad_1Class->setObjectName(QString::fromUtf8("ObjLoad_1Class"));
        ObjLoad_1Class->resize(600, 400);
        menuBar = new QMenuBar(ObjLoad_1Class);
        menuBar->setObjectName(QString::fromUtf8("menuBar"));
        ObjLoad_1Class->setMenuBar(menuBar);
        mainToolBar = new QToolBar(ObjLoad_1Class);
        mainToolBar->setObjectName(QString::fromUtf8("mainToolBar"));
        ObjLoad_1Class->addToolBar(mainToolBar);
        centralWidget = new QWidget(ObjLoad_1Class);
        centralWidget->setObjectName(QString::fromUtf8("centralWidget"));
        ObjLoad_1Class->setCentralWidget(centralWidget);
        statusBar = new QStatusBar(ObjLoad_1Class);
        statusBar->setObjectName(QString::fromUtf8("statusBar"));
        ObjLoad_1Class->setStatusBar(statusBar);

        retranslateUi(ObjLoad_1Class);

        QMetaObject::connectSlotsByName(ObjLoad_1Class);
    } // setupUi

    void retranslateUi(QMainWindow *ObjLoad_1Class)
    {
        ObjLoad_1Class->setWindowTitle(QApplication::translate("ObjLoad_1Class", "ObjLoad_1", nullptr));
    } // retranslateUi

};

namespace Ui {
    class ObjLoad_1Class: public Ui_ObjLoad_1Class {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_OBJLOAD_1_H
