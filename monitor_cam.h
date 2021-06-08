#pragma once
#include <QDockWidget>
#include "ui_monitor_cam.h"
#include "imageviewer.h"
#include "processing_thread.h"
#include <qprogressdialog.h>

class monitor_cam : public QDockWidget
{
	Q_OBJECT

public:

	QProgressDialog* progressbox;
	QImage* qColored;
	QVector<QRgb> table_8;
	processing_thread* proc_thread;
	monitor_cam(processing_thread* thread, QWidget *parent = Q_NULLPTR);
	~monitor_cam();

public slots:

	void safe_thread_close();
	void updateimage(GrabResultPtr_t ptr);
	void on_findCentroidButton_clicked();
	void updateimagesize(int width, int height);
	void on_zoomInButton_clicked();
	void on_zoomOutButton_clicked();
	void on_exposureBox_valueChanged(int);
	void on_upButton_clicked();
	void on_downButton_clicked();
	void on_leftButton_clicked();
	void on_rightButton_clicked();
	void on_resetButton_clicked();
	void on_triggerButton_toggled(bool);
	void finished_analysis();

signals:
	void write_to_log(QString q);

private:
	Ui::monitor_cam ui;
};
