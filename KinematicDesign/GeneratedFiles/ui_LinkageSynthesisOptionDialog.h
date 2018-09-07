/********************************************************************************
** Form generated from reading UI file 'LinkageSynthesisOptionDialog.ui'
**
** Created by: Qt User Interface Compiler version 5.6.1
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
    QPushButton *pushButtonOK;
    QPushButton *pushButtonCancel;
    QGroupBox *groupBox;
    QLabel *label_6;
    QLineEdit *lineEditStdDevPosition;
    QLabel *label_11;
    QLineEdit *lineEditStdDevOrientation;
    QGroupBox *groupBox_2;
    QLabel *label_3;
    QLineEdit *lineEditPositionErrorWeight;
    QLabel *label_4;
    QLineEdit *lineEditTrajectoryWeight;
    QLabel *label_5;
    QLineEdit *lineEditSizeWeight;
    QLabel *label_15;
    QLineEdit *lineEditLinkageLocationWeight;
    QLineEdit *lineEditLinkageDepthWeight;
    QLabel *label_18;
    QGroupBox *groupBox_3;
    QLabel *label_16;
    QLabel *label_17;
    QLineEdit *lineEditNumParticles;
    QLineEdit *lineEditNumIterations;
    QCheckBox *checkBoxRecordFile;
    QLineEdit *lineEditMinTransmissionAngle;
    QLabel *label_19;
    QCheckBox *checkBox4RLinkage;
    QCheckBox *checkBoxSliderCrank;

    void setupUi(QDialog *LinkageSynthesisOptionDialog)
    {
        if (LinkageSynthesisOptionDialog->objectName().isEmpty())
            LinkageSynthesisOptionDialog->setObjectName(QStringLiteral("LinkageSynthesisOptionDialog"));
        LinkageSynthesisOptionDialog->resize(352, 481);
        label = new QLabel(LinkageSynthesisOptionDialog);
        label->setObjectName(QStringLiteral("label"));
        label->setGeometry(QRect(30, 40, 61, 21));
        lineEditNumSamples = new QLineEdit(LinkageSynthesisOptionDialog);
        lineEditNumSamples->setObjectName(QStringLiteral("lineEditNumSamples"));
        lineEditNumSamples->setGeometry(QRect(220, 40, 101, 20));
        checkBoxAvoidBranchDefect = new QCheckBox(LinkageSynthesisOptionDialog);
        checkBoxAvoidBranchDefect->setObjectName(QStringLiteral("checkBoxAvoidBranchDefect"));
        checkBoxAvoidBranchDefect->setGeometry(QRect(30, 150, 251, 17));
        pushButtonOK = new QPushButton(LinkageSynthesisOptionDialog);
        pushButtonOK->setObjectName(QStringLiteral("pushButtonOK"));
        pushButtonOK->setGeometry(QRect(60, 440, 91, 31));
        pushButtonCancel = new QPushButton(LinkageSynthesisOptionDialog);
        pushButtonCancel->setObjectName(QStringLiteral("pushButtonCancel"));
        pushButtonCancel->setGeometry(QRect(200, 440, 91, 31));
        groupBox = new QGroupBox(LinkageSynthesisOptionDialog);
        groupBox->setObjectName(QStringLiteral("groupBox"));
        groupBox->setGeometry(QRect(20, 70, 311, 71));
        label_6 = new QLabel(groupBox);
        label_6->setObjectName(QStringLiteral("label_6"));
        label_6->setGeometry(QRect(50, 20, 101, 21));
        lineEditStdDevPosition = new QLineEdit(groupBox);
        lineEditStdDevPosition->setObjectName(QStringLiteral("lineEditStdDevPosition"));
        lineEditStdDevPosition->setGeometry(QRect(200, 20, 101, 20));
        label_11 = new QLabel(groupBox);
        label_11->setObjectName(QStringLiteral("label_11"));
        label_11->setGeometry(QRect(50, 40, 121, 21));
        lineEditStdDevOrientation = new QLineEdit(groupBox);
        lineEditStdDevOrientation->setObjectName(QStringLiteral("lineEditStdDevOrientation"));
        lineEditStdDevOrientation->setGeometry(QRect(200, 40, 101, 20));
        groupBox_2 = new QGroupBox(LinkageSynthesisOptionDialog);
        groupBox_2->setObjectName(QStringLiteral("groupBox_2"));
        groupBox_2->setGeometry(QRect(20, 190, 311, 131));
        label_3 = new QLabel(groupBox_2);
        label_3->setObjectName(QStringLiteral("label_3"));
        label_3->setGeometry(QRect(20, 20, 111, 21));
        lineEditPositionErrorWeight = new QLineEdit(groupBox_2);
        lineEditPositionErrorWeight->setObjectName(QStringLiteral("lineEditPositionErrorWeight"));
        lineEditPositionErrorWeight->setGeometry(QRect(200, 20, 101, 20));
        label_4 = new QLabel(groupBox_2);
        label_4->setObjectName(QStringLiteral("label_4"));
        label_4->setGeometry(QRect(20, 60, 161, 21));
        lineEditTrajectoryWeight = new QLineEdit(groupBox_2);
        lineEditTrajectoryWeight->setObjectName(QStringLiteral("lineEditTrajectoryWeight"));
        lineEditTrajectoryWeight->setGeometry(QRect(200, 60, 101, 20));
        label_5 = new QLabel(groupBox_2);
        label_5->setObjectName(QStringLiteral("label_5"));
        label_5->setGeometry(QRect(20, 80, 121, 21));
        lineEditSizeWeight = new QLineEdit(groupBox_2);
        lineEditSizeWeight->setObjectName(QStringLiteral("lineEditSizeWeight"));
        lineEditSizeWeight->setGeometry(QRect(200, 80, 101, 20));
        label_15 = new QLabel(groupBox_2);
        label_15->setObjectName(QStringLiteral("label_15"));
        label_15->setGeometry(QRect(20, 40, 111, 21));
        lineEditLinkageLocationWeight = new QLineEdit(groupBox_2);
        lineEditLinkageLocationWeight->setObjectName(QStringLiteral("lineEditLinkageLocationWeight"));
        lineEditLinkageLocationWeight->setGeometry(QRect(200, 40, 101, 20));
        lineEditLinkageDepthWeight = new QLineEdit(groupBox_2);
        lineEditLinkageDepthWeight->setObjectName(QStringLiteral("lineEditLinkageDepthWeight"));
        lineEditLinkageDepthWeight->setGeometry(QRect(200, 100, 101, 20));
        label_18 = new QLabel(groupBox_2);
        label_18->setObjectName(QStringLiteral("label_18"));
        label_18->setGeometry(QRect(20, 100, 121, 21));
        groupBox_3 = new QGroupBox(LinkageSynthesisOptionDialog);
        groupBox_3->setObjectName(QStringLiteral("groupBox_3"));
        groupBox_3->setGeometry(QRect(20, 330, 311, 91));
        label_16 = new QLabel(groupBox_3);
        label_16->setObjectName(QStringLiteral("label_16"));
        label_16->setGeometry(QRect(20, 20, 111, 21));
        label_17 = new QLabel(groupBox_3);
        label_17->setObjectName(QStringLiteral("label_17"));
        label_17->setGeometry(QRect(20, 40, 111, 21));
        lineEditNumParticles = new QLineEdit(groupBox_3);
        lineEditNumParticles->setObjectName(QStringLiteral("lineEditNumParticles"));
        lineEditNumParticles->setGeometry(QRect(200, 20, 101, 20));
        lineEditNumIterations = new QLineEdit(groupBox_3);
        lineEditNumIterations->setObjectName(QStringLiteral("lineEditNumIterations"));
        lineEditNumIterations->setGeometry(QRect(200, 40, 101, 20));
        checkBoxRecordFile = new QCheckBox(groupBox_3);
        checkBoxRecordFile->setObjectName(QStringLiteral("checkBoxRecordFile"));
        checkBoxRecordFile->setGeometry(QRect(20, 60, 171, 21));
        lineEditMinTransmissionAngle = new QLineEdit(LinkageSynthesisOptionDialog);
        lineEditMinTransmissionAngle->setObjectName(QStringLiteral("lineEditMinTransmissionAngle"));
        lineEditMinTransmissionAngle->setGeometry(QRect(220, 170, 101, 20));
        label_19 = new QLabel(LinkageSynthesisOptionDialog);
        label_19->setObjectName(QStringLiteral("label_19"));
        label_19->setGeometry(QRect(70, 170, 131, 16));
        checkBox4RLinkage = new QCheckBox(LinkageSynthesisOptionDialog);
        checkBox4RLinkage->setObjectName(QStringLiteral("checkBox4RLinkage"));
        checkBox4RLinkage->setGeometry(QRect(20, 10, 131, 17));
        checkBoxSliderCrank = new QCheckBox(LinkageSynthesisOptionDialog);
        checkBoxSliderCrank->setObjectName(QStringLiteral("checkBoxSliderCrank"));
        checkBoxSliderCrank->setGeometry(QRect(160, 10, 131, 17));

        retranslateUi(LinkageSynthesisOptionDialog);

        QMetaObject::connectSlotsByName(LinkageSynthesisOptionDialog);
    } // setupUi

    void retranslateUi(QDialog *LinkageSynthesisOptionDialog)
    {
        LinkageSynthesisOptionDialog->setWindowTitle(QApplication::translate("LinkageSynthesisOptionDialog", "Synthesis Options", 0));
        label->setText(QApplication::translate("LinkageSynthesisOptionDialog", "# Samples:", 0));
        checkBoxAvoidBranchDefect->setText(QApplication::translate("LinkageSynthesisOptionDialog", "Avoid branch/poor transmission angle defect", 0));
        pushButtonOK->setText(QApplication::translate("LinkageSynthesisOptionDialog", "OK", 0));
        pushButtonCancel->setText(QApplication::translate("LinkageSynthesisOptionDialog", "Cancel", 0));
        groupBox->setTitle(QApplication::translate("LinkageSynthesisOptionDialog", "Pose tolerance", 0));
        label_6->setText(QApplication::translate("LinkageSynthesisOptionDialog", "Stdev for position:", 0));
        label_11->setText(QApplication::translate("LinkageSynthesisOptionDialog", "Stdev for orientation:", 0));
        groupBox_2->setTitle(QApplication::translate("LinkageSynthesisOptionDialog", "Weights", 0));
        label_3->setText(QApplication::translate("LinkageSynthesisOptionDialog", "Position error:", 0));
        label_4->setText(QApplication::translate("LinkageSynthesisOptionDialog", "Smoothness of trajectory:", 0));
        label_5->setText(QApplication::translate("LinkageSynthesisOptionDialog", "Linkage size:", 0));
        label_15->setText(QApplication::translate("LinkageSynthesisOptionDialog", "Linkage location:", 0));
        label_18->setText(QApplication::translate("LinkageSynthesisOptionDialog", "Linkage depth:", 0));
        groupBox_3->setTitle(QApplication::translate("LinkageSynthesisOptionDialog", "Partile filter", 0));
        label_16->setText(QApplication::translate("LinkageSynthesisOptionDialog", "#particles:", 0));
        label_17->setText(QApplication::translate("LinkageSynthesisOptionDialog", "#iterations:", 0));
        checkBoxRecordFile->setText(QApplication::translate("LinkageSynthesisOptionDialog", "Record cost evolution to a file", 0));
        label_19->setText(QApplication::translate("LinkageSynthesisOptionDialog", "Min. transmission angle:", 0));
        checkBox4RLinkage->setText(QApplication::translate("LinkageSynthesisOptionDialog", "4R four-bar linkage", 0));
        checkBoxSliderCrank->setText(QApplication::translate("LinkageSynthesisOptionDialog", "Slider crank", 0));
    } // retranslateUi

};

namespace Ui {
    class LinkageSynthesisOptionDialog: public Ui_LinkageSynthesisOptionDialog {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_LINKAGESYNTHESISOPTIONDIALOG_H
