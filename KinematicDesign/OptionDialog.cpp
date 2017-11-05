#include "OptionDialog.h"

OptionDialog::OptionDialog(QWidget *parent) : QDialog(parent) {
	ui.setupUi(this);

	onSmall();

	connect(ui.pushButtonSmall, SIGNAL(clicked()), this, SLOT(onSmall()));
	connect(ui.pushButtonLarge, SIGNAL(clicked()), this, SLOT(onLarge()));
	connect(ui.pushButtonHalf, SIGNAL(clicked()), this, SLOT(onHalf()));
	connect(ui.pushButtonTwice, SIGNAL(clicked()), this, SLOT(onTwice()));
	connect(ui.pushButtonOK, SIGNAL(clicked()), this, SLOT(onOK()));
	connect(ui.pushButtonCancel, SIGNAL(clicked()), this, SLOT(onCancel()));
}

OptionDialog::~OptionDialog() {
}


void OptionDialog::setBodyMargin(float body_margin) {
	ui.lineEditBodyMargin->setText(QString::number(body_margin));
}

float OptionDialog::getBodyMargin() {
	return ui.lineEditBodyMargin->text().toFloat();
}

void OptionDialog::setGap(float gap) {
	ui.lineEditGap->setText(QString::number(gap));
}

float OptionDialog::getGap() {
	return ui.lineEditGap->text().toFloat();
}

void OptionDialog::setLinkWidth(float link_width) { 
	ui.lineEditLinkWidth->setText(QString::number(link_width));
}

float OptionDialog::getLinkWidth() {
	return ui.lineEditLinkWidth->text().toFloat();
}

void OptionDialog::setLinkDepth(float link_depth) {
	ui.lineEditLinkDepth->setText(QString::number(link_depth));
}

float OptionDialog::getLinkDepth() {
	return ui.lineEditLinkDepth->text().toFloat();
}

void OptionDialog::setHoleRadius(float hole_radius) {
	ui.lineEditHoleRadius->setText(QString::number(hole_radius));
}

float OptionDialog::getHoleRadius() {
	return ui.lineEditHoleRadius->text().toFloat();
}

void OptionDialog::setJointRaidus(float joint_radius) {
	ui.lineEditJointRadius->setText(QString::number(joint_radius));
}

float OptionDialog::getJointRadius() {
	return ui.lineEditJointRadius->text().toFloat();
}

void OptionDialog::setJointCapRaidus1(float joint_cap_radius1) {
	ui.lineEditJointCapRadius1->setText(QString::number(joint_cap_radius1));
}

float OptionDialog::getJointCapRadius1() {
	return ui.lineEditJointCapRadius1->text().toFloat();
}

void OptionDialog::setJointCapRaidus2(float joint_cap_radius2) {
	ui.lineEditJointCapRadius2->setText(QString::number(joint_cap_radius2));
}

float OptionDialog::getJointCapRadius2() {
	return ui.lineEditJointCapRadius2->text().toFloat();
}

void OptionDialog::setJointCapDepth(float joint_cap_depth) {
	ui.lineEditJointCapDepth->setText(QString::number(joint_cap_depth));
}

float OptionDialog::getJointCapDepth() {
	return ui.lineEditJointCapDepth->text().toFloat();
}

void OptionDialog::setSliderBarWidth(float slider_bar_width) {
	ui.lineEditSliderBarWidth->setText(QString::number(slider_bar_width));
}

float OptionDialog::getSliderBarWidth() {
	return ui.lineEditSliderBarWidth->text().toFloat();
}

void OptionDialog::setSliderBarDepth(float slider_bar_depth) {
	ui.lineEditSliderBarDepth->setText(QString::number(slider_bar_depth));
}

float OptionDialog::getSliderBarDepth() {
	return ui.lineEditSliderBarDepth->text().toFloat();
}

void OptionDialog::setSliderWidth(float slider_width) {
	ui.lineEditSliderWidth->setText(QString::number(slider_width));
}

float OptionDialog::getSliderWidth() {
	return ui.lineEditSliderWidth->text().toFloat();
}

void OptionDialog::setSliderDepth(float slider_depth) {
	ui.lineEditSliderDepth->setText(QString::number(slider_depth));
}

float OptionDialog::getSliderDepth() {
	return ui.lineEditSliderDepth->text().toFloat();
}

void OptionDialog::onSmall() {
	ui.lineEditBodyMargin->setText("0.3");
	ui.lineEditGap->setText("0.04");
	ui.lineEditLinkWidth->setText("1.0");
	ui.lineEditLinkDepth->setText("0.3");
	ui.lineEditHoleRadius->setText("0.26");
	ui.lineEditJointRadius->setText("0.25");
	ui.lineEditJointCapRadius1->setText("0.23");
	ui.lineEditJointCapRadius2->setText("0.28");
	ui.lineEditJointCapDepth->setText("0.15");
	ui.lineEditSliderBarWidth->setText("0.6");
	ui.lineEditSliderBarDepth->setText("0.3");
	ui.lineEditSliderWidth->setText("1.0");
	ui.lineEditSliderDepth->setText("0.5");
}

void OptionDialog::onLarge() {
	ui.lineEditBodyMargin->setText("0.3");
	ui.lineEditGap->setText("0.07");
	ui.lineEditLinkWidth->setText("2.5");
	ui.lineEditLinkDepth->setText("0.9");
	ui.lineEditHoleRadius->setText("0.6625");
	ui.lineEditJointRadius->setText("0.625");
	ui.lineEditJointCapRadius1->setText("0.6");
	ui.lineEditJointCapRadius2->setText("0.7");
	ui.lineEditJointCapDepth->setText("0.4");
	ui.lineEditSliderBarWidth->setText("1.2");
	ui.lineEditSliderBarDepth->setText("0.6");
	ui.lineEditSliderWidth->setText("2.0");
	ui.lineEditSliderDepth->setText("1.0");
}

void OptionDialog::onHalf() {
	ui.lineEditGap->setText(QString::number(ui.lineEditGap->text().toDouble() * 0.5));
	ui.lineEditLinkWidth->setText(QString::number(ui.lineEditLinkWidth->text().toDouble() * 0.5));
	ui.lineEditLinkDepth->setText(QString::number(ui.lineEditLinkDepth->text().toDouble() * 0.5));
	ui.lineEditHoleRadius->setText(QString::number(ui.lineEditHoleRadius->text().toDouble() * 0.5));
	ui.lineEditJointRadius->setText(QString::number(ui.lineEditJointRadius->text().toDouble() * 0.5));
	ui.lineEditJointCapRadius1->setText(QString::number(ui.lineEditJointCapRadius1->text().toDouble() * 0.5));
	ui.lineEditJointCapRadius2->setText(QString::number(ui.lineEditJointCapRadius2->text().toDouble() * 0.5));
	ui.lineEditJointCapDepth->setText(QString::number(ui.lineEditJointCapDepth->text().toDouble() * 0.5));
	ui.lineEditSliderBarWidth->setText(QString::number(ui.lineEditSliderBarWidth->text().toDouble() * 0.5));
	ui.lineEditSliderBarDepth->setText(QString::number(ui.lineEditSliderBarDepth->text().toDouble() * 0.5));
	ui.lineEditSliderWidth->setText(QString::number(ui.lineEditSliderWidth->text().toDouble() * 0.5));
	ui.lineEditSliderDepth->setText(QString::number(ui.lineEditSliderDepth->text().toDouble() * 0.5));
}

void OptionDialog::onTwice() {
	ui.lineEditGap->setText(QString::number(ui.lineEditGap->text().toDouble() * 2));
	ui.lineEditLinkWidth->setText(QString::number(ui.lineEditLinkWidth->text().toDouble() * 2));
	ui.lineEditLinkDepth->setText(QString::number(ui.lineEditLinkDepth->text().toDouble() * 2));
	ui.lineEditHoleRadius->setText(QString::number(ui.lineEditHoleRadius->text().toDouble() * 2));
	ui.lineEditJointRadius->setText(QString::number(ui.lineEditJointRadius->text().toDouble() * 2));
	ui.lineEditJointCapRadius1->setText(QString::number(ui.lineEditJointCapRadius1->text().toDouble() * 2));
	ui.lineEditJointCapRadius2->setText(QString::number(ui.lineEditJointCapRadius2->text().toDouble() * 2));
	ui.lineEditJointCapDepth->setText(QString::number(ui.lineEditJointCapDepth->text().toDouble() * 2));
	ui.lineEditSliderBarWidth->setText(QString::number(ui.lineEditSliderBarWidth->text().toDouble() * 2));
	ui.lineEditSliderBarDepth->setText(QString::number(ui.lineEditSliderBarDepth->text().toDouble() * 2));
	ui.lineEditSliderWidth->setText(QString::number(ui.lineEditSliderWidth->text().toDouble() * 2));
	ui.lineEditSliderDepth->setText(QString::number(ui.lineEditSliderDepth->text().toDouble() * 2));
}

void OptionDialog::onOK() {
	accept();
}

void OptionDialog::onCancel() {
	reject();
}
