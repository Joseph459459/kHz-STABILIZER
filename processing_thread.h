#include "cameraselect.h"
#include "FS_macros.h"
#include <QThread>
#include <QSerialPort>
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
	
	Camera_t camera;
	std::atomic<bool> acquiring = false;
	QVector<float> centroidx;
	QVector<float> centroidy;
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
	void updatefftplot();
	void finished_analysis();

private:
	void run() override;

};
