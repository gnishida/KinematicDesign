/********************************************************************************
** Form generated from reading UI file 'LinkageSynthesisOptionDialog.ui'
**
** Created by: Qt User Interface Compiler version 5.6.0
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_LINKAGESYNTHESISOPTIONDIALOG_H
#define UI_LINKAGESYNTHESISOPTIONDIALOG_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QDialog>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QPushButton>

QT_BEGIN_NAMESPACE

class Ui_LinkageSynthesisOptionDialog
{
public:
    QLabel *label;
    QLineEdit *lineEditNumSamples;
    QCheckBox *checkBoxAvoidBranchDefect;
    QCheckBox *checkBoxRotatableCrank;
    QPushButton *pushButtonOK;
    QPushButton *pushButtonCancel;
    QGroupBox *groupBox;
    QLabel *label_2;
    QLineEdit *lineEditStdDevPositionFirst;
    QLabel *label_6;
    QLineEdit *lineEditStdDevPositionMiddle;
    QLabel *label_7;
    QLineEdit *lineEditStdDevPositionLast;
    QLabel *label_8;
    QLabel *label_9;
    QLineEdit *lineEditStdDevOrientationFirst;
    QLabel *label_10;
    QLabel *label_11;
    QLineEdit *lineEditStdDevOrientationMiddle;
    QLabel *label_12;
    QLabel *label_13;
    QLineEdit *lineEditStdDevOrientationLast;
    QGroupBox *groupBox_2;
    QLabel *label_3;
    QLineEdit *lineEditPositionErrorWeight;
    QLabel *label_4;
    QLineEdit *lineEditTrajectoryWeight;
    QLabel *label_5;
    QLineEdit *lineEditSizeWeight;
    QLabel *label_14;
    QLineEdit *lineEditOrientationErrorWeight;
    QLabel *label_15;
    QLineEdit *lineEditLinkageLocationWeight;

    void setupUi(QDialog *LinkageSynthesisOptionDialog)
    {
        if (LinkageSynthesisOptionDialog->objectName().isEmpty())
            LinkageSynthesisOptionDialog->setObjectName(QStringLiteral("LinkageSynthesisOptionDialog"));
        LinkageSynthesisOptionDialog->resize(352, 520);
        label = new QLabel(LinkageSynthesisOptionDialog);
        label->setObjectName(QStringLiteral("label"));
        label->setGeometry(QRect(30, 10, 61, 21));
        lineEditNumSamples = new QLineEdit(LinkageSynthesisOptionDialog);
        lineEditNumSamples->setObjectName(QStringLiteral("lineEditNumSamples"));
        lineEditNumSamples->setGeometry(QRect(220, 10, 101, 20));
        checkBoxAvoidBranchDefect = new QCheckBox(LinkageSynthesisOptionDialog);
        checkBoxAvoidBranchDefect->setObjectName(QStringLiteral("checkBoxAvoidBranchDefect"));
        checkBoxAvoidBranchDefect->setGeometry(QRect(30, 260, 131, 17));
        checkBoxRotatableCrank = new QCheckBox(LinkageSynthesisOptionDialog);
        checkBoxRotatableCrank->setObjectName(QStringLiteral("checkBoxRotatableCrank"));
        checkBoxRotatableCrank->setGeometry(QRect(30, 280, 131, 17));
        pushButtonOK = new QPushButton(LinkageSynthesisOptionDialog);
        pushButtonOK->setObjectName(QStringLiteral("pushButtonOK"));
        pushButtonOK->setGeometry(QRect(60, 460, 91, 31));
        pushButtonCancel = new QPushButton(LinkageSynthesisOptionDialog);
        pushButtonCancel->setObjectName(QStringLiteral("pushButtonCancel"));
        pushButtonCancel->setGeometry(QRect(200, 460, 91, 31));
        groupBox = new QGroupBox(LinkageSynthesisOptionDialog);
        groupBox->setObjectName(QStringLiteral("groupBox"));
        groupBox->setGeometry(QRect(20, 40, 311, 211));
        label_2 = new QLabel(groupBox);
        label_2->setObjectName(QStringLiteral("label_2"));
        label_2->setGeometry(QRect(50, 40, 131, 21));
        lineEditStdDevPositionFirst = new QLineEdit(groupBox);
        lineEditStdDevPositionFirst->setObjectName(QStringLiteral("lineEditStdDevPositionFirst"));
        lineEditStdDevPositionFirst->setGeometry(QRect(200, 40, 101, 20));
        label_6 = new QLabel(groupBox);
        label_6->setObjectName(QStringLiteral("label_6"));
        label_6->setGeometry(QRect(50, 100, 101, 21));
        lineEditStdDevPositionMiddle = new QLineEdit(groupBox);
        lineEditStdDevPositionMiddle->setObjectName(QStringLiteral("lineEditStdDevPositionMiddle"));
        lineEditStdDevPositionMiddle->setGeometry(QRect(200, 100, 101, 20));
        label_7 = new QLabel(groupBox);
        label_7->setObjectName(QStringLiteral("label_7"));
        label_7->setGeometry(QRect(50, 160, 131, 21));
        lineEditStdDevPositionLast = new QLineEdit(groupBox);
        lineEditStdDevPositionLast->setObjectName(QStringLiteral("lineEditStdDevPositionLast"));
        lineEditStdDevPositionLast->setGeometry(QRect(200, 160, 101, 20));
        label_8 = new QLabel(groupBox);
        label_8->setObjectName(QStringLiteral("label_8"));
        label_8->setGeometry(QRect(20, 20, 51, 16));
        label_9 = new QLabel(groupBox);
        label_9->setObjectName(QStringLiteral("label_9"));
        label_9->setGeometry(QRect(50, 60, 131, 21));
        lineEditStdDevOrientationFirst = new QLineEdit(groupBox);
        lineEditStdDevOrientationFirst->setObjectName(QStringLiteral("lineEditStdDevOrientationFirst"));
        lineEditStdDevOrientationFirst->setGeometry(QRect(200, 60, 101, 20));
        label_10 = new QLabel(groupBox);
        label_10->setObjectName(QStringLiteral("label_10"));
        label_10->setGeometry(QRect(20, 80, 101, 16));
        label_11 = new QLabel(groupBox);
        label_11->setObjectName(QStringLiteral("label_11"));
        label_11->setGeometry(QRect(50, 120, 121, 21));
        lineEditStdDevOrientationMiddle = new QLineEdit(groupBox);
        lineEditStdDevOrientationMiddle->setObjectName(QStringLiteral("lineEditStdDevOrientationMiddle"));
        lineEditStdDevOrientationMiddle->setGeometry(QRect(200, 120, 101, 20));
        label_12 = new QLabel(groupBox);
        label_12->setObjectName(QStringLiteral("label_12"));
        label_12->setGeometry(QRect(20, 140, 101, 16));
        label_13 = new QLabel(groupBox);
        label_13->setObjectName(QStringLiteral("label_13"));
        label_13->setGeometry(QRect(50, 180, 131, 21));
        lineEditStdDevOrientationLast = new QLineEdit(groupBox);
        lineEditStdDevOrientationLast->setObjectName(QStringLiteral("lineEditStdDevOrientationLast"));
        lineEditStdDevOrientationLast->setGeometry(QRect(200, 180, 101, 20));
        groupBox_2 = new QGroupBox(LinkageSynthesisOptionDialog);
        groupBox_2->setObjectName(QStringLiteral("groupBox_2"));
        groupBox_2->setGeometry(QRect(20, 310, 311, 131));
        label_3 = new QLabel(groupBox_2);
        label_3->setObjectName(QStringLiteral("label_3"));
        label_3->setGeometry(QRect(20, 20, 111, 21));
        lineEditPositionErrorWeight = new QLineEdit(groupBox_2);
        lineEditPositionErrorWeight->setObjectName(QStringLiteral("lineEditPositionErrorWeight"));
        lineEditPositionErrorWeight->setGeometry(QRect(200, 20, 101, 20));
        label_4 = new QLabel(groupBox_2);
        label_4->setObjectName(QStringLiteral("label_4"));
        label_4->setGeometry(QRect(20, 80, 161, 21));
        lineEditTrajectoryWeight = new QLineEdit(groupBox_2);
        lineEditTrajectoryWeight->setObjectName(QStringLiteral("lineEditTrajectoryWeight"));
        lineEditTrajectoryWeight->setGeometry(QRect(200, 80, 101, 20));
        label_5 = new QLabel(groupBox_2);
        label_5->setObjectName(QStringLiteral("label_5"));
        label_5->setGeometry(QRect(20, 100, 121, 21));
        lineEditSizeWeight = new QLineEdit(groupBox_2);
        lineEditSizeWeight->setObjectName(QStringLiteral("lineEditSizeWeight"));
        lineEditSizeWeight->setGeometry(QRect(200, 100, 101, 20));
        label_14 = new QLabel(groupBox_2);
        label_14->setObjectName(QStringLiteral("label_14"));
        label_14->setGeometry(QRect(20, 40, 111, 21));
        lineEditOrientationErrorWeight = new QLineEdit(groupBox_2);
        lineEditOrientationErrorWeight->setObjectName(QStringLiteral("lineEditOrientationErrorWeight"));
        lineEditOrientationErrorWeight->setGeometry(QRect(200, 40, 101, 20));
        label_15 = new QLabel(groupBox_2);
        label_15->setObjectName(QStringLiteral("label_15"));
        label_15->setGeometry(QRect(20, 60, 111, 21));
        lineEditLinkageLocationWeight = new QLineEdit(groupBox_2);
        lineEditLinkageLocationWeight->setObjectName(QStringLiteral("lineEditLinkageLocationWeight"));
        lineEditLinkageLocationWeight->setGeometry(QRect(200, 60, 101, 20));

        retranslateUi(LinkageSynthesisOptionDialog);

        QMetaObject::connectSlotsByName(LinkageSynthesisOptionDialog);
    } // setupUi

    void retranslateUi(QDialog *LinkageSynthesisOptionDialog)
    {
        LinkageSynthesisOptionDialog->setWindowTitle(QApplication::translate("LinkageSynthesisOptionDialog", "Synthesis Options", 0));
        label->setText(QApplication::translate("LinkageSynthesisOptionDialog", "# Samples:", 0));
        checkBoxAvoidBranchDefect->setText(QApplication::translate("LinkageSynthesisOptionDialog", "Avoid branch defect", 0));
        checkBoxRotatableCrank->setText(QApplication::translate("LinkageSynthesisOptionDialog", "Fully rotatable crank", 0));
        pushButtonOK->setText(QApplication::translate("LinkageSynthesisOptionDialog", "OK", 0));
        pushButtonCancel->setText(QApplication::translate("LinkageSynthesisOptionDialog", "Cancel", 0));
        groupBox->setTitle(QApplication::translate("LinkageSynthesisOptionDialog", "Pose tolerance", 0));
        label_2->setText(QApplication::translate("LinkageSynthesisOptionDialog", "Stdev for position:", 0));
        label_6->setText(QApplication::translate("LinkageSynthesisOptionDialog", "Stdev for position:", 0));
        label_7->setText(QApplication::translate("LinkageSynthesisOptionDialog", "Stdev for position:", 0));
        label_8->setText(QApplication::translate("LinkageSynthesisOptionDialog", "1st pose:", 0));
        label_9->setText(QApplication::translate("LinkageSynthesisOptionDialog", "Stdev for orientation:", 0));
        label_10->setText(QApplication::translate("LinkageSynthesisOptionDialog", "Intermediate poses:", 0));
        label_11->setText(QApplication::translate("LinkageSynthesisOptionDialog", "Stdev for orientation:", 0));
        label_12->setText(QApplication::translate("LinkageSynthesisOptionDialog", "Last pose:", 0));
        label_13->setText(QApplication::translate("LinkageSynthesisOptionDialog", "Stdev for orientation:", 0));
        groupBox_2->setTitle(QApplication::translate("LinkageSynthesisOptionDialog", "Weights", 0));
        label_3->setText(QApplication::translate("LinkageSynthesisOptionDialog", "Position error:", 0));
        label_4->setText(QApplication::translate("LinkageSynthesisOptionDialog", "Smoothness of trajectory:", 0));
        label_5->setText(QApplication::translate("LinkageSynthesisOptionDialog", "Linkage size:", 0));
        label_14->setText(QApplication::translate("LinkageSynthesisOptionDialog", "Orientation error:", 0));
        label_15->setText(QApplication::translate("LinkageSynthesisOptionDialog", "Linkage location:", 0));
    } // retranslateUi

};

namespace Ui {
    class LinkageSynthesisOptionDialog: public Ui_LinkageSynthesisOptionDialog {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_LINKAGESYNTHESISOPTIONDIALOG_H
