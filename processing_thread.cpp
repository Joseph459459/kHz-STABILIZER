#include "FASTSTABILIZER.h"
//#include "nlopt.hpp"
#include "camview.h"


processing_thread::processing_thread(CDeviceInfo c, QObject* parent)
	: QThread(parent), camera(CTlFactory::GetInstance().CreateDevice(c))
{
	PylonInitialize();

	CDeviceInfo info;
	info.SetDeviceClass(Camera_t::DeviceClass());


}

processing_thread::~processing_thread()
{
}

void processing_thread::adjust_framerate() {

	//Dumb hack - this sets the acquisition frame rate to the maximum allowed with given settings
	camera.AcquisitionFrameRateAbs.SetValue(100000.0);

	if (camera.ResultingFrameRateAbs() > 1000)
		camera.AcquisitionFrameRateAbs.SetValue(1000);
	else
		camera.AcquisitionFrameRateAbs.SetValue(camera.ResultingFrameRateAbs());

}

//void processing_thread::stabilize() {
//
//	float out[2] = { 0,0 };
//
//#pragma region PORT
//
//
//#pragma endregion
//
//#pragma region CAMERA
//	try {
//
//		PylonInitialize();
//
//		CDeviceInfo info;
//		info.SetDeviceClass(Camera_t::DeviceClass());
//
//		Camera_t camera(CTlFactory::GetInstance().CreateFirstDevice(info), Cleanup_Delete);
//		camera.Open();
//
//
//		camera.TriggerSelector.SetValue(TriggerSelector_FrameStart);
//		camera.TriggerMode.SetValue(TriggerMode_On);
//		camera.TriggerActivation.SetValue(TriggerActivation_RisingEdge);
//
//		//camera.TriggerSelector.SetValue(TriggerSelector_FrameStart);
//		//camera.TriggerMode.SetValue(TriggerMode_Off);
//		//camera.AcquisitionFrameRateEnable.SetValue(true);
//		//camera.AcquisitionFrameRateAbs.SetValue(500);
//
//		CGrabResultPtr ptr;
//
//#define height 50
//#define width 50
//
//		camera.Height.SetValue(height);
//		camera.Width.SetValue(width);
//
//		camera.PixelFormat.SetValue(PixelFormat_Mono8);
//		camera.MaxNumBuffer.SetValue(1);
//		camera.BlackLevelRaw.SetValue(511);
//		camera.GevSCPSPacketSize.SetValue((height * width + 14) / 4 * 4 + 4 - 4);
//		camera.ExposureTimeAbs.SetValue(22);
//
//#pragma endregion
//		teensy.write(QByteArray(1, STABILIZE));
//		float tones[6] = { 10, 20, 30, 40, 50, 60 };
//		teensy.write((const char*)tones, 24);
//		teensy.waitForBytesWritten();
//		acquiring = true;
//		camera.StartGrabbing();
//		while (acquiring) {
//
//			if (camera.RetrieveResult(2, ptr, Pylon::ETimeoutHandling::TimeoutHandling_Return)) {
//
//				centroid(ptr, height, width, out, 3);
//
//				teensy.write(QByteArray(1, SYNC_FLAG));
//				teensy.write((const char*)out);
//				teensy.waitForBytesWritten();
//
//				ptr.Release();
//			}
//		}
//
//		camera.Close();
//		camera.DestroyDevice();
//		PylonTerminate();
//
//	}
//	catch (Pylon::GenericException& e) {
//		emit write_to_log(QString(e.what()));
//	}
//	catch (std::exception& e) {
//		emit write_to_log(QString(e.what()));
//	}
//}

void processing_thread::stream() {

	adjust_framerate();

	camera.GevSCPSPacketSize.SetValue(8000);

	emit updateimagesize(camera.Width.GetValue(), camera.Height.GetValue());

	camera.MaxNumBuffer.SetValue(20);


	camera.StartGrabbing();
	GrabResultPtr_t ptr;
	acquiring = true;

	while (acquiring) {
		msleep(20);
		if (camera.RetrieveResult(30, ptr, Pylon::TimeoutHandling_Return)) {
			emit sendImagePtr(ptr);
		}
	}
	msleep(20);
	camera.StopGrabbing();
}

void processing_thread::analyze_spectrum() {

	adjust_framerate();

	emit write_to_log(QString("Beginning Noise Profiling..."));

	if (camera.ResultingFrameRateAbs.GetValue() < 1000) {
		emit write_to_log(QString("Camera cannot acquire at 1 kHz (") + QString::number(camera.ResultingFrameRateAbs()) + QString(" Hz)"));
	}

	else {

		centroidx.resize(window);
		centroidy.resize(window);

		camera.MaxNumBuffer.SetValue(window);
		camera.GevSCPSPacketSize.SetValue(((camera.Height.GetValue() * camera.Width.GetValue() + 14) / 4) * 4);

		std::vector<GrabResultPtr_t> ptrs(window);

		int missed = 0;
		int i = 0;
		camera.StartGrabbing();


		while (i < window) {
			if (camera.RetrieveResult(30, ptrs[i], Pylon::TimeoutHandling_Return)) {
				++i;
			}
			else {
				++missed;
			}

			if (i % (window / 100) == 0) {
				emit updateprogress(i);
			}
		}

		camera.StopGrabbing();

		emit updateprogress(5000);

		emit write_to_log(QString::number(i) + QString(" Shots recorded"));

		emit write_to_log(QString::number(missed) + QString(" Shots missed"));

		std::function<std::array<float, 2>(GrabResultPtr_t)> calc;

		calc = [thresh = threshold](GrabResultPtr_t pt) {
			return centroid(pt, thresh);};

		QFuture<std::array<float, 2>> centroids = QtConcurrent::mapped(ptrs.begin(), ptrs.end(), calc);

		centroids.waitForFinished();

		float sumx = 0;
		float sumy = 0;
		int nans = 0;

		for (int i = 0; i < window; ++i) {
			if (std::isnan(centroids.resultAt(i)[0]))
				++nans;

			centroidx[i] = centroids.resultAt(i)[0];
			sumx += centroidx[i];

			if (std::isnan(centroids.resultAt(i)[1]))
				++nans;

			centroidy[i] = centroids.resultAt(i)[1];
			sumy += centroidy[i];
		}

		float meanx = sumx / window;
		float meany = sumy / window;

		for (int i = 0; i < window; ++i) {
			centroidx[i] -= meanx;
			centroidy[i] -= meany;
		}

		write_to_log(QString::number(nans) + QString(" NaN centroids detected"));

		planx = fftwf_plan_r2r_1d(window, centroidx.data(), fft_outx, FFTW_R2HC, FFTW_ESTIMATE);
		plany = fftwf_plan_r2r_1d(window, centroidy.data(), fft_outy, FFTW_R2HC, FFTW_ESTIMATE);

		fftwf_execute(planx);
		fftwf_execute(plany);

		Filter LP = Filter(LPF, 60, 1, 0.1);
		if (LP.get_error_flag() != 0) {
			emit write_to_log(QString("LP Filter is broken. Good Luck..."));
		}

		LPfftx.resize(window / 2);
		LPffty.resize(window / 2);



		LPfftx[0] = sqrt(fft_outx[0] * fft_outx[0]);
		LPffty[0] = sqrt(fft_outy[0] * fft_outy[0]);


		for (int i = 1; i < (window + 1) / 2 - 1; ++i) {

			LPfftx[i] = sqrt(fft_outx[i] * fft_outx[i] +
				fft_outx[window - i] * fft_outx[window - i]);
		}

		for (int i = 1; i < (window + 1) / 2 - 1; ++i) {
			LPffty[i] = sqrt(fft_outy[i] * fft_outy[i] +
				fft_outy[window - i] * fft_outy[window - i]);
		}


		double maxX = *(std::max_element(LPfftx.begin(), LPfftx.end()));
		float maxY = *(std::max_element(LPffty.begin(), LPffty.end()));

		for (int i = 0; i < window / 2; ++i) {
			LPfftx[i] = LPfftx[i] / maxX;
			LPffty[i] = LPffty[i] / maxY;

		}

		emit updatefftplot();

		fftwf_destroy_plan(planx);
		fftwf_destroy_plan(plany);

	}


	emit finished_analysis();

}

void processing_thread::find_actuator_range() {

	adjust_framerate();

	emit write_to_log(QString("Finding Actuator Range..."));

#pragma region OPEN_PORT

	QSerialPort teensy;

	QList<QSerialPortInfo> ports = QSerialPortInfo::availablePorts();

	for (QSerialPortInfo port : ports) {

		port.vendorIdentifier();

		if (port.vendorIdentifier() == 5824) {
			teensy.setPortName(port.portName());

		}
	}

	if (!teensy.open(QIODevice::ReadWrite)) {
		emit write_to_log(QString("USB port not able to open."));
		return;
	};

	bool j = true;

	j = teensy.setBaudRate(QSerialPort::Baud115200);
	j = teensy.setDataBits(QSerialPort::Data8);
	j = teensy.setParity(QSerialPort::NoParity);
	j = teensy.setStopBits(QSerialPort::OneStop);
	j = teensy.setFlowControl(QSerialPort::NoFlowControl);

	if (!j) {
		emit write_to_log(QString("Error configuring USB connection."));
		return;
	}
	else {
		emit write_to_log(QString("Port opened and configured"));
	}



#pragma endregion

	GrabResultPtr_t ptr;

	teensy.write(QByteArray(1, FIND_RANGE));
	teensy.waitForBytesWritten(50);
	camera.StartGrabbing(Pylon::GrabStrategy_UpcomingImage);

	while (true) {

		if (camera.RetrieveResult(30, ptr, Pylon::TimeoutHandling_Return)) {

			std::array<double, 6> out = allparams(ptr, threshold);

			if (!isnan(out[3]) && out[3] < camera.Height() && (out[1] - out[3] / 2) < 0) {
				teensy.write(QByteArray(1, SYNC_FLAG));
				teensy.write(QByteArray(1, STOP_FLAG));
				if (!teensy.waitForBytesWritten(1000)) {
					emit write_to_log(QString("Synchronization error: could not write"));
					break;
				}
			}
			
			else {
				teensy.write(QByteArray(1, SYNC_FLAG));
				teensy.write(QByteArray(1, CONTINUE));
				if (!teensy.waitForBytesWritten(1000)) {
					emit write_to_log(QString("Synchronization error: could not write"));
					break;
				}
			}

			if (teensy.waitForReadyRead(1000)) {	
				if (*teensy.read(1) == STOP_FLAG) {
					yDACmax = *((uint16_t*)teensy.read(2).data());
					emit write_to_log(QString("Success: the DAC uses " + QString::number(yDACmax + 1) +
						" out of 4096 available bits"));
					break;
				}
			}

			else {
				emit write_to_log(QString("Synchronization error: could not read"));
				break;
			}
			
			emit send_imgptr_blocking(ptr);

		}

	}

	camera.StopGrabbing();

	teensy.close();

	emit finished_analysis();
}

void processing_thread::learn_transfer_function() {

	adjust_framerate();

	emit write_to_log(QString("Finding Pixel/DAC Transfer Function..."));


	if (camera.ResultingFrameRateAbs.GetValue() < 1000) {
		emit write_to_log(QString("Camera cannot acquire at 1 kHz (") + QString::number(camera.ResultingFrameRateAbs()) + QString(" Hz)"));
	}
	else {

#pragma region OPEN_PORT

		QSerialPort teensy;

		QList<QSerialPortInfo> ports = QSerialPortInfo::availablePorts();

		for (QSerialPortInfo port : ports) {

			port.vendorIdentifier();

			if (port.vendorIdentifier() == 5824) {
				teensy.setPortName(port.portName());

			}
		}

		if (!teensy.open(QIODevice::ReadWrite)) {
			emit write_to_log(QString("USB port not able to open."));
			return;
		};

		bool j = true;

		j = teensy.setBaudRate(QSerialPort::Baud115200);
		j = teensy.setDataBits(QSerialPort::Data8);
		j = teensy.setParity(QSerialPort::NoParity);
		j = teensy.setStopBits(QSerialPort::OneStop);
		j = teensy.setFlowControl(QSerialPort::NoFlowControl);

		if (!j) {
			emit write_to_log(QString("Error configuring USB connection."));
			return;
		}
		else {
			emit write_to_log(QString("Port opened and configured"));
		}



#pragma endregion

		GrabResultPtr_t ptr;

		teensy.write(QByteArray(1, LEARN_TF));
		teensy.waitForBytesWritten(50);

		int missed = 0;
		int i = 0;
		camera.StartGrabbing();


		while (i < window) {
			if (camera.RetrieveResult(30, ptr, Pylon::TimeoutHandling_Return)) {
				++i;
			}
			else {
				++missed;
			}

			if (i % (window / 100) == 0) {
				emit updateprogress(i);
			}
		}
	}

}

void processing_thread::run() {

	switch (plan) {
	case STABILIZE:
		//stabilize();
		break;
	case STREAM:
		stream();
		break;
	case SPECTRUM:
		analyze_spectrum();
		break;
	case FIND_RANGE:
		find_actuator_range();
		break;
	case LEARN_TF:
		learn_transfer_function();
	default:
		break;
	}
}
