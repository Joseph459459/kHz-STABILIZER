#pragma once

#include <QDockWidget>
#include "ui_camview.h"
#include "imageviewer.h"
#include "FASTSTABILIZER.h"


class camview : public QDockWidget
{
	Q_OBJECT

public:

	QProgressDialog* progressbox;
	QImage* qColored;
	QVector<QRgb> table_8;
	processing_thread* proc_thread;
	camview(processing_thread* thread, QWidget *parent = Q_NULLPTR);
	~camview();
	void safe_thread_close();

public slots:
	void updateimage(GrabResultPtr_t ptr);
	void on_findCentroidButton_clicked();
	void on_actuatorRangeButton_clicked();
	void on_transferFunctionButton_clicked();
	void updateimagesize(int width, int height);
	void on_zoomInButton_clicked();
	void on_zoomOutButton_clicked();
	void on_noiseSpectrumButton_clicked();
	void on_exposureBox_valueChanged(int);
	void on_upButton_clicked();
	void on_downButton_clicked();
	void on_leftButton_clicked();
	void on_rightButton_clicked();
	void on_resetButton_clicked();
	void finished_analysis();
	void on_triggerButton_toggled(bool);

signals:
	void write_to_log(QString q);
	void update_fft_plot();

private:
	Ui::camview ui;
};
