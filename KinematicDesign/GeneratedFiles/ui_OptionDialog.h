/********************************************************************************
** Form generated from reading UI file 'OptionDialog.ui'
**
** Created by: Qt User Interface Compiler version 5.6.0
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_OPTIONDIALOG_H
#define UI_OPTIONDIALOG_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QDialog>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QPushButton>

QT_BEGIN_NAMESPACE

class Ui_OptionDialog
{
public:
    QGroupBox *groupBox;
    QLabel *label;
    QLineEdit *lineEditLinkWidth;
    QLabel *label_2;
    QLineEdit *lineEditLinkDepth;
    QLabel *label_3;
    QLineEdit *lineEditJointRadius;
    QLabel *label_4;
    QLabel *label_5;
    QLabel *label_6;
    QLabel *label_7;
    QLineEdit *lineEditHoleRadius;
    QLabel *label_8;
    QLabel *label_9;
    QLabel *label_10;
    QLabel *label_11;
    QLineEdit *lineEditJointCapRadius1;
    QLineEdit *lineEditJointCapRadius2;
    QLineEdit *lineEditJointCapDepth;
    QLineEdit *lineEditSliderBarWidth;
    QLineEdit *lineEditSliderBarDepth;
    QLineEdit *lineEditSliderWidth;
    QLineEdit *lineEditSliderDepth;
    QLabel *label_12;
    QLineEdit *lineEditGap;
    QPushButton *pushButtonLarge;
    QPushButton *pushButtonSmall;
    QPushButton *pushButtonTwice;
    QPushButton *pushButtonHalf;
    QLabel *label_13;
    QLineEdit *lineEditBodyMargin;
    QPushButton *pushButtonOK;
    QPushButton *pushButtonCancel;

    void setupUi(QDialog *OptionDialog)
    {
        if (OptionDialog->objectName().isEmpty())
            OptionDialog->setObjectName(QStringLiteral("OptionDialog"));
        OptionDialog->resize(291, 438);
        groupBox = new QGroupBox(OptionDialog);
        groupBox->setObjectName(QStringLiteral("groupBox"));
        groupBox->setGeometry(QRect(20, 10, 251, 361));
        label = new QLabel(groupBox);
        label->setObjectName(QStringLiteral("label"));
        label->setGeometry(QRect(10, 130, 71, 21));
        lineEditLinkWidth = new QLineEdit(groupBox);
        lineEditLinkWidth->setObjectName(QStringLiteral("lineEditLinkWidth"));
        lineEditLinkWidth->setGeometry(QRect(120, 130, 113, 20));
        label_2 = new QLabel(groupBox);
        label_2->setObjectName(QStringLiteral("label_2"));
        label_2->setGeometry(QRect(10, 150, 71, 21));
        lineEditLinkDepth = new QLineEdit(groupBox);
        lineEditLinkDepth->setObjectName(QStringLiteral("lineEditLinkDepth"));
        lineEditLinkDepth->setGeometry(QRect(120, 150, 113, 20));
        label_3 = new QLabel(groupBox);
        label_3->setObjectName(QStringLiteral("label_3"));
        label_3->setGeometry(QRect(10, 190, 71, 21));
        lineEditJointRadius = new QLineEdit(groupBox);
        lineEditJointRadius->setObjectName(QStringLiteral("lineEditJointRadius"));
        lineEditJointRadius->setGeometry(QRect(120, 190, 113, 20));
        label_4 = new QLabel(groupBox);
        label_4->setObjectName(QStringLiteral("label_4"));
        label_4->setGeometry(QRect(10, 210, 91, 21));
        label_5 = new QLabel(groupBox);
        label_5->setObjectName(QStringLiteral("label_5"));
        label_5->setGeometry(QRect(10, 230, 91, 21));
        label_6 = new QLabel(groupBox);
        label_6->setObjectName(QStringLiteral("label_6"));
        label_6->setGeometry(QRect(10, 250, 91, 21));
        label_7 = new QLabel(groupBox);
        label_7->setObjectName(QStringLiteral("label_7"));
        label_7->setGeometry(QRect(10, 170, 91, 21));
        lineEditHoleRadius = new QLineEdit(groupBox);
        lineEditHoleRadius->setObjectName(QStringLiteral("lineEditHoleRadius"));
        lineEditHoleRadius->setGeometry(QRect(120, 170, 113, 20));
        label_8 = new QLabel(groupBox);
        label_8->setObjectName(QStringLiteral("label_8"));
        label_8->setGeometry(QRect(10, 270, 91, 21));
        label_9 = new QLabel(groupBox);
        label_9->setObjectName(QStringLiteral("label_9"));
        label_9->setGeometry(QRect(10, 310, 91, 21));
        label_10 = new QLabel(groupBox);
        label_10->setObjectName(QStringLiteral("label_10"));
        label_10->setGeometry(QRect(10, 290, 91, 21));
        label_11 = new QLabel(groupBox);
        label_11->setObjectName(QStringLiteral("label_11"));
        label_11->setGeometry(QRect(10, 330, 91, 21));
        lineEditJointCapRadius1 = new QLineEdit(groupBox);
        lineEditJointCapRadius1->setObjectName(QStringLiteral("lineEditJointCapRadius1"));
        lineEditJointCapRadius1->setGeometry(QRect(120, 210, 113, 20));
        lineEditJointCapRadius2 = new QLineEdit(groupBox);
        lineEditJointCapRadius2->setObjectName(QStringLiteral("lineEditJointCapRadius2"));
        lineEditJointCapRadius2->setGeometry(QRect(120, 230, 113, 20));
        lineEditJointCapDepth = new QLineEdit(groupBox);
        lineEditJointCapDepth->setObjectName(QStringLiteral("lineEditJointCapDepth"));
        lineEditJointCapDepth->setGeometry(QRect(120, 250, 113, 20));
        lineEditSliderBarWidth = new QLineEdit(groupBox);
        lineEditSliderBarWidth->setObjectName(QStringLiteral("lineEditSliderBarWidth"));
        lineEditSliderBarWidth->setGeometry(QRect(120, 270, 113, 20));
        lineEditSliderBarDepth = new QLineEdit(groupBox);
        lineEditSliderBarDepth->setObjectName(QStringLiteral("lineEditSliderBarDepth"));
        lineEditSliderBarDepth->setGeometry(QRect(120, 290, 113, 20));
        lineEditSliderWidth = new QLineEdit(groupBox);
        lineEditSliderWidth->setObjectName(QStringLiteral("lineEditSliderWidth"));
        lineEditSliderWidth->setGeometry(QRect(120, 310, 113, 20));
        lineEditSliderDepth = new QLineEdit(groupBox);
        lineEditSliderDepth->setObjectName(QStringLiteral("lineEditSliderDepth"));
        lineEditSliderDepth->setGeometry(QRect(120, 330, 113, 20));
        label_12 = new QLabel(groupBox);
        label_12->setObjectName(QStringLiteral("label_12"));
        label_12->setGeometry(QRect(10, 110, 71, 21));
        lineEditGap = new QLineEdit(groupBox);
        lineEditGap->setObjectName(QStringLiteral("lineEditGap"));
        lineEditGap->setGeometry(QRect(120, 110, 113, 20));
        pushButtonLarge = new QPushButton(groupBox);
        pushButtonLarge->setObjectName(QStringLiteral("pushButtonLarge"));
        pushButtonLarge->setGeometry(QRect(130, 20, 101, 31));
        pushButtonSmall = new QPushButton(groupBox);
        pushButtonSmall->setObjectName(QStringLiteral("pushButtonSmall"));
        pushButtonSmall->setGeometry(QRect(20, 20, 101, 31));
        pushButtonTwice = new QPushButton(groupBox);
        pushButtonTwice->setObjectName(QStringLiteral("pushButtonTwice"));
        pushButtonTwice->setGeometry(QRect(130, 50, 101, 31));
        pushButtonHalf = new QPushButton(groupBox);
        pushButtonHalf->setObjectName(QStringLiteral("pushButtonHalf"));
        pushButtonHalf->setGeometry(QRect(20, 50, 101, 31));
        label_13 = new QLabel(groupBox);
        label_13->setObjectName(QStringLiteral("label_13"));
        label_13->setGeometry(QRect(10, 90, 71, 21));
        lineEditBodyMargin = new QLineEdit(groupBox);
        lineEditBodyMargin->setObjectName(QStringLiteral("lineEditBodyMargin"));
        lineEditBodyMargin->setGeometry(QRect(120, 90, 113, 20));
        pushButtonOK = new QPushButton(OptionDialog);
        pushButtonOK->setObjectName(QStringLiteral("pushButtonOK"));
        pushButtonOK->setGeometry(QRect(30, 390, 101, 31));
        pushButtonCancel = new QPushButton(OptionDialog);
        pushButtonCancel->setObjectName(QStringLiteral("pushButtonCancel"));
        pushButtonCancel->setGeometry(QRect(160, 390, 101, 31));
        QWidget::setTabOrder(pushButtonOK, pushButtonCancel);
        QWidget::setTabOrder(pushButtonCancel, pushButtonSmall);
        QWidget::setTabOrder(pushButtonSmall, pushButtonLarge);
        QWidget::setTabOrder(pushButtonLarge, lineEditGap);
        QWidget::setTabOrder(lineEditGap, lineEditLinkWidth);
        QWidget::setTabOrder(lineEditLinkWidth, lineEditLinkDepth);
        QWidget::setTabOrder(lineEditLinkDepth, lineEditHoleRadius);
        QWidget::setTabOrder(lineEditHoleRadius, lineEditJointRadius);
        QWidget::setTabOrder(lineEditJointRadius, lineEditJointCapRadius1);
        QWidget::setTabOrder(lineEditJointCapRadius1, lineEditJointCapRadius2);
        QWidget::setTabOrder(lineEditJointCapRadius2, lineEditJointCapDepth);
        QWidget::setTabOrder(lineEditJointCapDepth, lineEditSliderBarWidth);
        QWidget::setTabOrder(lineEditSliderBarWidth, lineEditSliderBarDepth);
        QWidget::setTabOrder(lineEditSliderBarDepth, lineEditSliderWidth);
        QWidget::setTabOrder(lineEditSliderWidth, lineEditSliderDepth);

        retranslateUi(OptionDialog);

        QMetaObject::connectSlotsByName(OptionDialog);
    } // setupUi

    void retranslateUi(QDialog *OptionDialog)
    {
        OptionDialog->setWindowTitle(QApplication::translate("OptionDialog", "OptionDialog", 0));
        groupBox->setTitle(QApplication::translate("OptionDialog", "Parameters for linkage", 0));
        label->setText(QApplication::translate("OptionDialog", "Link width:", 0));
        label_2->setText(QApplication::translate("OptionDialog", "Link depth:", 0));
        label_3->setText(QApplication::translate("OptionDialog", "Joint radius:", 0));
        label_4->setText(QApplication::translate("OptionDialog", "Joint cap radius1:", 0));
        label_5->setText(QApplication::translate("OptionDialog", "Joint cap radius 2:", 0));
        label_6->setText(QApplication::translate("OptionDialog", "Joint cap depth:", 0));
        label_7->setText(QApplication::translate("OptionDialog", "Hole radius:", 0));
        label_8->setText(QApplication::translate("OptionDialog", "Slider bar width:", 0));
        label_9->setText(QApplication::translate("OptionDialog", "Slider width:", 0));
        label_10->setText(QApplication::translate("OptionDialog", "Slider bar depth:", 0));
        label_11->setText(QApplication::translate("OptionDialog", "Slider depth:", 0));
        label_12->setText(QApplication::translate("OptionDialog", "Gap:", 0));
        pushButtonLarge->setText(QApplication::translate("OptionDialog", "Large", 0));
        pushButtonSmall->setText(QApplication::translate("OptionDialog", "Small", 0));
        pushButtonTwice->setText(QApplication::translate("OptionDialog", "x 2", 0));
        pushButtonHalf->setText(QApplication::translate("OptionDialog", "x 0.5", 0));
        label_13->setText(QApplication::translate("OptionDialog", "Body margin:", 0));
        pushButtonOK->setText(QApplication::translate("OptionDialog", "OK", 0));
        pushButtonCancel->setText(QApplication::translate("OptionDialog", "Cancel", 0));
    } // retranslateUi

};

namespace Ui {
    class OptionDialog: public Ui_OptionDialog {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_OPTIONDIALOG_H
