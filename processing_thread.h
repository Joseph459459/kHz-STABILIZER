#pragma once
#include "camera_select.h"
#include "kHz_macros.h"
#include <QThread>
#include <QtConcurrent>
#include <QtSerialPort>
#include "filt.h"
#include "fftw3.h"
#include <QFuture>
#include <array>


class processing_thread : public QThread
{
	Q_OBJECT
	
public:
	explicit processing_thread(CDeviceInfo fb_info, QObject* parent);
	explicit processing_thread(CDeviceInfo fb_info, CDeviceInfo m_info, QObject* parent);

	~processing_thread();

    std::array<double, 4> fit_params[2];

	Camera_t fb_cam;
	Camera_t monitor_cam;

    std::atomic_bool acquiring = false;

    QVector<double> centroids[2];
    QVector<double> fft[2];

    int run_plan;

	int threshold = 0;

    uint16_t max_DAC_val[2] = {4095,4095};

    QVector<double> system_response_input;

    std::vector<int> N[2];
    std::vector<float> tones[2];
    float centroid_set_points[2];

    std::array<float, 3> drive_freqs = {200,215,220};

    std::array<float, 2> cam_correlation_params[2];

    std::atomic_bool monitor_cam_enabled = false;

public slots:
	void stabilize();
	void stream();
	void analyze_spectrum();
	void correlate_cameras();
	void adjust_framerate();
	void find_actuator_range();
	void learn_total_system_response();
    void learn_local_system_response();
	void receive_large_serial_buffer(QSerialPort& teensy, std::vector<int>& buffer, int chunk_size);
	void receive_cmd_line_data(QStringList cmd_str);
	void test_loop_times();
    void test_loop_times_dual_cam();
    bool open_port(QSerialPort& teensy);
    void prep_cam(Camera_t* cam);
    void stabilize_dual_cam();

signals:
	void write_to_log(QString q);
	void send_feedback_ptr(GrabResultPtr_t ptr);
	void send_monitor_ptr(GrabResultPtr_t ptr);
	void send_imgptr_blocking(GrabResultPtr_t ptr);
	void update_progress(int i);
	void update_fft_plot(float rms_x, float rms_y, float peak_to_peak_x, float peak_to_peak_y);
    void update_local_sys_response_plot(QVector<QVector<double>> to_plot);
    void update_total_sys_response_plot(QVector<QVector<double>> to_plot);
	void finished_analysis();
    void update_correlation_plot(QVector<QVector<double>> fb,QVector<QVector<double>> m);

private:
	void run() override;

};
template<typename T>
inline void centroid(GrabResultPtr_t ptr, const int height, const int width, T out[], const int threshold) {

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

    out[0] = sumx / (T)sum;
    out[1] = sumy / (T)sum;

}

template <typename T>
inline std::array<T, 2> centroid(GrabResultPtr_t ptr, const int threshold) {

	const int height = ptr->GetHeight();
	const int width = ptr->GetWidth();

    std::array<T, 2> out = { 0,0 };
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

    out[0] = sumx / (T)sum;
    out[1] = sumy / (T)sum;
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
        (double) max_val,
		sum / (double) (width * height)
	};

	return out;

}

template <typename T>
double preciserms(T& s) {

    double sumx = 0;
    double sumxx = 0;
    int fails = 0;
    for (int i = 0; i < s.size(); ++i) {

        if (!isnan(s[i])) {
            sumx += s[i];
        }
        else
            fails++;
    }

    double shift = sumx / (double)s.size();

    sumx = 0;

    for (int i = 0; i < s.size(); ++i) {

        if (!isnan(s[i])) {
            sumx += s[i] - shift;
            sumxx += (s[i] - shift) * (s[i] - shift);
        }
    }

    return sqrt((sumxx - sumx * sumx / ((double)s.size() - fails)) / (s.size() - fails));

}
