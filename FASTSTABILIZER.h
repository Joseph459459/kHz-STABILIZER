#pragma once
#include "ui_FASTSTABILIZER.h"
#include <QtWidgets/QMainWindow>
#include <QSerialPort>
#include <QSerialPortInfo>
#include "console.h"
#include "processing_thread.h"
#include <QDebug>
#include "FS_macros.h"
#include <array>
#include <algorithm>
#include <cmath>
#include <QVector>
#include <QRgb>
#include <QtConcurrent>
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
	FASTSTABILIZER(CDeviceInfo c, QWidget *parent = Q_NULLPTR);
	~FASTSTABILIZER();
	QSerialPort teensy;
	processing_thread proc_thread;
	QVector<double> freqs;
	QVector<myFilter> xfilters;
	QVector<myFilter> yfilters;
	QTimer animateTimer;
	float phi = 0;
	int graphidx = 0;

protected:
	virtual void keyPressEvent(QKeyEvent* e);

signals:

public slots:
	void error_handling(QString b);
	void on_stabilizeButton_clicked();
	void on_stopButton_clicked();
	void on_learnButton_clicked();
	void updatefftplot();
	void on_horizontalZoomButton_toggled(bool j);
	void new_filter(const QCPDataSelection& p);
	void remove_filter(QCPAbstractPlottable* p, int j, QMouseEvent* e);
	void animate_plot(bool j);
	void animate(int i);
	void create_fft_plots();
	void update_freq_label(int N, int graphdatapos);

	QVector<double> filtersim(QVector<double>& in, int graphpos, int N, float phi);

private:
	Ui::FASTSTABILIZERClass ui;
};

inline void centroid(GrabResultPtr_t ptr, const int height, const int width, float out[],const int threshold) {

	unsigned char* p = (unsigned char*)ptr->GetBuffer();

	int sumx = 0;
	int sumy = 0;
	int sum = 0;

	int i, j;

	for (i = 0; i < height; ++i) {

		for (j = 0; j < width; ++j) {

			if (p[i * width + j] < threshold)
				p[i * width + j] = 0;

			sum += p[i * width + j];
			sumx += p[i * width + j] * j;
			sumy += p[i * width + j] * i;
		}
	}

	out[0] = sumx / (float)sum;
	out[1] = sumy / (float)sum;

}

inline std::array<float,2> centroid(GrabResultPtr_t ptr,  const int threshold) {

	const int height = ptr->GetHeight();
	const int width = ptr->GetWidth();

	std::array<float, 2> out = { 0,0 };
	unsigned char* p = (unsigned char*)ptr->GetBuffer();

	int sumx = 0;
	int sumy = 0;
	int sum = 0;

	int i, j;

	for (i = 0; i < height; ++i) {

		for (j = 0; j < width; ++j) {

			if (p[i * width + j] < threshold)
				p[i * width + j] = 0;

			sum += p[i * width + j];
			sumx += p[i * width + j] * j;
			sumy += p[i * width + j] * i;
		}
	}

	out[0] = sumx / (float)sum;
	out[1] = sumy / (float)sum;
	return out;
}

inline float mean_std(GrabResultPtr_t ptr, const int threshold) {

	const int height = ptr->GetHeight();
	const int width = ptr->GetWidth();

	unsigned char* p = (unsigned char*)ptr->GetBuffer();

	int sum = 0;

	int i, j;

	for (i = 0; i < height; ++i) {

		for (j = 0; j < width; ++j) {

			if (p[i * width + j] < threshold)
				p[i * width + j] = 0;


			sum += p[i * width + j];

		}
	}


	float out = sum / (width * height);
	return out;
}

//Output, in order { centroidx, centroidy, 2ndmomx, 2ndmomy, max, mean } 
inline std::array<double, 6> allparams(GrabResultPtr_t ptr, const int thresh) {

	const int height = ptr->GetHeight();
	const int width = ptr->GetWidth();

	unsigned int sumyy = 0;
	unsigned int sumxx = 0;
	unsigned int sum = 0;
	unsigned int sumy = 0;
	unsigned int sumx = 0;
	unsigned char max = 0;
	unsigned char f;
	unsigned char* p = (unsigned char*)ptr->GetBuffer();


	int i, j;

	for (i = 0; i < height; ++i)
	{
		for (j = 0; j < width; ++j)
		{
			f = p[i * width + j];

			if (f < thresh)
			{
				f = 0; p[i * width + j] = 0;
			}

			max = std::max(max, f);

			sum += f;
			sumx += f * j;
			sumxx += f * j * j;
			sumy += f * i;
			sumyy += f * i * i;
		}
	}

	std::array<double, 6> out{
	sumx / (double)sum,
	sumy / (double)sum,
	4 * sqrt(((double)sumxx - sumx *(double) sumx / (double)sum) / ((double)sum)),
	4 * sqrt(((double)sumyy - sumy *(double)sumy / (double)sum) / ((double)sum)),
		max,
		sum / static_cast<double>(width * height)
	};

	return out;

}

inline QVector<double> FASTSTABILIZER::filtersim(QVector<double>& in, int graphpos, int N, float phi) {

	QVector<double> out(in.size());

	double omega_0 = 2 * PI * (graphpos / (window / 2.0) * 500) / sampling_freq;


	for (int i = 0; i < in.size(); ++i) {

		double omega = 2 * PI * in[i] / sampling_freq;

		double sin_plus = sin(N * (omega + omega_0) / 2.0) / sin((omega + omega_0) / 2.0);
		double sin_minus = sin(N * (omega - omega_0) / 2.0) / sin((omega - omega_0) / 2.0);

		out[i] =
			2.0/N*sqrt(0.25 * (2 * cos((N - 1) * omega + 2 * phi) * sin_plus * sin_minus + sin_plus * sin_plus + sin_minus * sin_minus));

	}

	return out;

}
