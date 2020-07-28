#pragma once
#include "cameraselect.h"
#include "FS_macros.h"
#include <QThread>
#include <QtSerialPort/qserialport.h>
#include <QtConcurrent/qtconcurrentmap.h>
#include <QtSerialPort/qserialportinfo.h>
#include "filt.h"
#include "fftw3.h"
#include <QFuture>
#include <array>



class processing_thread : public QThread
{
	Q_OBJECT

public:
	explicit processing_thread(CDeviceInfo c, QObject* parent);
	~processing_thread();

	QVector<float> centroidx_f;
	QVector<float> centroidy_f;
	QVector<double> centroidx_d;
	QVector<double> centroidy_d;

	Camera_t camera;
	std::atomic<bool> acquiring = false;

	float fft_outx[_window];
	float fft_outy[_window];
	QVector<double> fftx;
	QVector<double> ffty;
	fftwf_plan planx;
	fftwf_plan plany;
	int plan;
	int threshold = 0;
	uint16_t yDACmax;
	uint16_t xDACmax;
	QVector<double> tf_input_arr;
	std::array<float, 3> drive_freqs;
	double* p;

public slots:
	void stabilize();
	void stream();
	void analyze_spectrum();
	void find_driving_freqs();
	void adjust_framerate();
	void find_actuator_range();
	void learn_transfer_function();

signals:
	void write_to_log(QString q);
	void sendImagePtr(GrabResultPtr_t ptr);
	void send_imgptr_blocking(GrabResultPtr_t ptr);
	void updateimagesize(int width, int height);
	void updateprogress(int i);
	void update_fft_plot();
	void update_tf_plot(std::array<float,2> tf_params, QVector<double> filteredx, QVector<double> filteredy);
	void finished_analysis();

private:
	void run() override;

};

inline void centroid(GrabResultPtr_t ptr, const int height, const int width, float out[], const int threshold) {

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

inline std::array<float, 2> centroid(GrabResultPtr_t ptr, const int threshold) {

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
	unsigned char max_val = 0;
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

			
			max_val = f > max_val ? f : max_val;

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
	4 * sqrt(((double)sumxx - sumx * (double)sumx / (double)sum) / ((double)sum)),
	4 * sqrt(((double)sumyy - sumy * (double)sumy / (double)sum) / ((double)sum)),
		max_val,
		sum / static_cast<double>(width * height)
	};

	return out;

}
