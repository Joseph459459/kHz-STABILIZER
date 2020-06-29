#include "cameraselect.h"
#include "FS_macros.h"
#include <QThread>
#include <QtSerialPort/qserialport.h>
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

	float fft_outx[window];
	float fft_outy[window];
	QVector<double> LPfftx;
	QVector<double> LPffty;
	fftwf_plan planx;
	fftwf_plan plany;
	int plan;
	int threshold = 0;
	uint16_t yDACmax;
	uint16_t xDACmax;
	QVector<double> tf_input_arr;

public slots:
	//void stabilize();
	void stream();
	void analyze_spectrum();
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
	void update_tf_plot();
	void finished_analysis();

private:
	void run() override;

};
