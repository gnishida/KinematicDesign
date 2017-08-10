#ifndef LINKAGESYNTHESISOPTIONDIALOG_H
#define LINKAGESYNTHESISOPTIONDIALOG_H

#include <QDialog>
#include "ui_LinkageSynthesisOptionDialog.h"

class LinkageSynthesisOptionDialog : public QDialog
{
	Q_OBJECT

public:
	Ui::LinkageSynthesisOptionDialog ui;

public:
	LinkageSynthesisOptionDialog(QWidget *parent = 0);
	~LinkageSynthesisOptionDialog();

public slots:
	void onOK();
	void onCancel();
};

#endif // LINKAGESYNTHESISOPTIONDIALOG_H
