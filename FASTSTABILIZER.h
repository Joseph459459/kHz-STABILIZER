#pragma once

#include "ui_FASTSTABILIZER.h"
#include "feedback_cam.h"
#include "monitor_cam.h"
#include "console.h"
#include "FS_macros.h"
#include <QtWidgets/QMainWindow>
#include <QDebug>
#include <array>
#include <algorithm>
#include <cmath>
#include <QVector>
#include <QRgb>
#include <QSharedPointer>

struct myFilter {
	QCPGraph* ptr;
	int graphdatapos;
	int N;
};

class FASTSTABILIZER : public QMainWindow
{
	Q_OBJECT

public:
	FASTSTABILIZER(CDeviceInfo fb_info, QWidget *parent = Q_NULLPTR);
	FASTSTABILIZER(CDeviceInfo fb_info, CDeviceInfo m_info, QWidget* parent = Q_NULLPTR);

	~FASTSTABILIZER();
	
	feedback_cam* FC = nullptr;
	monitor_cam* MC = nullptr;

	processing_thread proc_thread;
	QVector<double> freqs;
	QVector<myFilter> x_filters;
	QVector<myFilter> y_filters;
	std::vector<QCPCurve*> hysteresis_curves;
	QTimer animateTimer;
	float phi = 0;
	int graphidx = 0;

protected:
	virtual void keyPressEvent(QKeyEvent* e);

signals:
	void send_cmd_line_data(QStringList cmd_str);

public slots:
	void update_log(QString b);
	void on_stabilizeButton_clicked();
	void on_stopButton_clicked();
	void on_learnButton_clicked();
	void create_tf_plots();
	void update_fft_plot(float rms_x, float rms_y, float peak_to_peak_x, float peak_to_peak_y);
	void update_tf_plot(QVector<QVector<double>> to_plot);
	void on_horizontalZoomButton_toggled(bool j);
	void new_filter(const QCPDataSelection& p);
	void remove_filter(QCPAbstractPlottable* p, int j, QMouseEvent* e);
	void animate_plot(bool j);
	void animate(int i);
	void create_fft_plots();
	void update_freq_label(int N, int graphdatapos);
	void update_filter_params();
	void on_cmdLineBox_returnPressed();

	QVector<double> filtersim(QVector<double>& in, int graphpos, int N, float phi);

private:
	Ui::FASTSTABILIZERClass ui;
};


inline QVector<double> FASTSTABILIZER::filtersim(QVector<double>& in, int graphpos, int N, float phi) {

	QVector<double> out(in.size());

	double omega_0 = 2 * PI * (graphpos / (_window / 2.0) * 500) / sampling_freq;


	for (int i = 0; i < in.size(); ++i) {

		double omega = 2 * PI * in[i] / sampling_freq;

		double sin_plus = sin(N * (omega + omega_0) / 2.0) / sin((omega + omega_0) / 2.0);
		double sin_minus = sin(N * (omega - omega_0) / 2.0) / sin((omega - omega_0) / 2.0);

		out[i] =
			2.0/N*sqrt(0.25 * (2 * cos((N - 1) * omega + 2 * phi) * sin_plus * sin_minus + sin_plus * sin_plus + sin_minus * sin_minus));

	}

	return out;

}
