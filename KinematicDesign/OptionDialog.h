#ifndef OPTIONDIALOG_H
#define OPTIONDIALOG_H

#include <QDialog>
#include "ui_OptionDialog.h"

class OptionDialog : public QDialog {
	Q_OBJECT

private:
	Ui::OptionDialog ui;

public:
	OptionDialog(QWidget *parent = 0);
	~OptionDialog();
	
	void setBodyMargin(float body_margin);
	float getBodyMargin();
	void setGap(float gap);
	float getGap();
	void setLinkWidth(float link_width);
	float getLinkWidth();
	void setLinkDepth(float link_depth);
	float getLinkDepth();
	void setHoleRadius(float hole_radius);
	float getHoleRadius();
	void setJointRaidus(float joint_radius);
	float getJointRadius();
	void setJointCapRaidus1(float joint_cap_radius1);
	float getJointCapRadius1();
	void setJointCapRaidus2(float joint_cap_radius2);
	float getJointCapRadius2();
	void setJointCapDepth(float joint_cap_depth);
	float getJointCapDepth();
	void setSliderGuideWidth(float slider_guide_width);
	float getSliderGuideWidth();
	void setSliderGuideDepth(float slider_guide_depth);
	float getSliderGuideDepth();
	void setBodyDepth(float body_depth);
	float getBodyDepth();

public slots:
	void onSmall();
	void onLarge();
	void onHalf();
	void onTwice();
	void onOK();
	void onCancel();
};

#endif // OPTIONDIALOG_H
