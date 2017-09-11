#include "LinkageSynthesisOptionDialog.h"

LinkageSynthesisOptionDialog::LinkageSynthesisOptionDialog(QWidget *parent) : QDialog(parent) {
	ui.setupUi(this);

	ui.lineEditNumSamples->setText("1000");
	ui.lineEditStdDevPositionFirst->setText("0");
	ui.lineEditStdDevOrientationFirst->setText("0");
	ui.lineEditStdDevPositionMiddle->setText("0");
	ui.lineEditStdDevOrientationMiddle->setText("0");
	ui.lineEditStdDevPositionLast->setText("0");
	ui.lineEditStdDevOrientationLast->setText("0");
	ui.checkBoxAvoidBranchDefect->setChecked(true);
	ui.checkBoxRotatableCrank->setChecked(true);
	ui.lineEditPositionErrorWeight->setText("1");
	ui.lineEditOrientationErrorWeight->setText("5");
	ui.lineEditLinkageLocationWeight->setText("10");
	ui.lineEditTrajectoryWeight->setText("1");
	ui.lineEditSizeWeight->setText("1");

	connect(ui.pushButtonOK, SIGNAL(clicked()), this, SLOT(onOK()));
	connect(ui.pushButtonCancel, SIGNAL(clicked()), this, SLOT(onCancel()));
}

LinkageSynthesisOptionDialog::~LinkageSynthesisOptionDialog() {
}

void LinkageSynthesisOptionDialog::onOK() {
	accept();
}

void LinkageSynthesisOptionDialog::onCancel() {
	reject();
}
