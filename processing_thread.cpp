#include "processing_thread.h"
#include "nlopt.hpp"
#include <armadillo>
#include <gsl/gsl_vector.h>
#include <gsl/gsl_filter.h>
#include <gsl/gsl_math.h>

using namespace arma;

struct harmonics_filter {

	harmonics_filter(int freq, int half_bandwidth) {


		fundamental = new Filter(BPF, n_taps, 1000, freq - half_bandwidth, freq + half_bandwidth);
		third = new Filter(BPF, n_taps, 1000, freq + 2 * freq - half_bandwidth, freq + 2 * freq + half_bandwidth);
		fifth = new Filter(BPF, n_taps, 1000, freq + 4 * freq - half_bandwidth, freq + 4 * freq + half_bandwidth);


		//passband_ripple_compensate
		
		std::vector<int> my_freqs = { freq, freq + 2 * freq, freq + 4 * freq };
		std::vector<double> in(1500);
		std::vector<double> out(1500);

		for (int freq : my_freqs) {
		
			Filter* temp = new Filter(BPF, n_taps, 1000, freq - half_bandwidth, freq + half_bandwidth);

			for (int i = 0; i < 1500; ++i) {
				in[i] = cos(2 * PI * freq / 1000 * i);
				out[i] = temp->do_sample(in[i]);
			}

			auto gain = std::max_element(out.begin() + n_taps + 1, out.end());

			gain_factors.push_back(*gain);

			delete temp;

		}

	}

	~harmonics_filter() {

		delete fundamental;
		delete third;
		delete fifth;

	}

	void filter_vec(std::vector<double>& dest, double* in) {

		for (int i = 0; i < dest.size(); ++i) {

			dest[i] += fundamental->do_sample(in[i]) / gain_factors[0];
			dest[i] += third->do_sample(in[i]) / gain_factors[1];
			dest[i] += fifth->do_sample(in[i]) / gain_factors[2];

		}

	}

	std::vector<double> gain_factors;
	Filter* fundamental;
	Filter* third;
	Filter* fifth;
};


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

void processing_thread::stabilize() {

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

	float tones[6] = { 10, 20, 30, 40, 50, 60 };

	teensy.write((const char*)tones, 24);


}

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

		centroidx_f.resize(_window);
		centroidy_f.resize(_window);

		camera.MaxNumBuffer.SetValue(_window);
		camera.GevSCPSPacketSize.SetValue(((camera.Height.GetValue() * camera.Width.GetValue() + 14) / 4) * 4);

		std::vector<GrabResultPtr_t> ptrs(_window);

		int missed = 0;
		int i = 0;
		camera.StartGrabbing();

		while (i < _window) {
			if (camera.RetrieveResult(30, ptrs[i], Pylon::TimeoutHandling_Return)) {
				++i;
			}
			else {
				++missed;
			}

			if (i % (_window / 100) == 0) {
				emit updateprogress(i);
			}
		}

		camera.StopGrabbing();

		emit updateprogress(_window);

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

		for (int i = 0; i < _window; ++i) {
			if (std::isnan(centroids.resultAt(i)[0]))
				++nans;

			centroidx_f[i] = centroids.resultAt(i)[0];
			sumx += centroidx_f[i];

			if (std::isnan(centroids.resultAt(i)[1]))
				++nans;

			centroidy_f[i] = centroids.resultAt(i)[1];
			sumy += centroidy_f[i];
		}

		float meanx = sumx / _window;
		float meany = sumy / _window;

		for (int i = 0; i < _window; ++i) {
			centroidx_f[i] -= meanx;
			centroidy_f[i] -= meany;
		}

		write_to_log(QString::number(nans) + QString(" NaN centroids detected"));

		planx = fftwf_plan_r2r_1d(_window, centroidx_f.data(), fft_outx, FFTW_R2HC, FFTW_ESTIMATE);
		plany = fftwf_plan_r2r_1d(_window, centroidy_f.data(), fft_outy, FFTW_R2HC, FFTW_ESTIMATE);

		fftwf_execute(planx);
		fftwf_execute(plany);

		Filter LP = Filter(LPF, 60, 1, 0.1);
		if (LP.get_error_flag() != 0) {
			emit write_to_log(QString("LP Filter is broken."));
		}

		fftx.resize(_window / 2);
		ffty.resize(_window / 2);


		fftx[0] = sqrt(fft_outx[0] * fft_outx[0]);
		ffty[0] = sqrt(fft_outy[0] * fft_outy[0]);


		for (int i = 1; i < (_window + 1) / 2 - 1; ++i) {

			fftx[i] = sqrt(fft_outx[i] * fft_outx[i] +
				fft_outx[_window - i] * fft_outx[_window - i]);
		}

		for (int i = 1; i < (_window + 1) / 2 - 1; ++i) {
			ffty[i] = sqrt(fft_outy[i] * fft_outy[i] +
				fft_outy[_window - i] * fft_outy[_window - i]);
		}

		double maxX = *(std::max_element(fftx.begin(), fftx.end()));
		float maxY = *(std::max_element(ffty.begin(), ffty.end()));

		for (int i = 0; i < _window / 2; ++i) {
			fftx[i] = fftx[i] / maxX;
			ffty[i] = ffty[i] / maxY;

		}

		find_driving_freqs();
		
		emit update_fft_plot();

		fftwf_destroy_plan(planx);
		fftwf_destroy_plan(plany);

	}


	emit finished_analysis();

}

void processing_thread::find_driving_freqs() {

	gsl_vector* convx = gsl_vector_alloc(_window / 2 / 5);
	gsl_vector* convy = gsl_vector_alloc(_window / 2 / 5);

	gsl_vector* filteredy = gsl_vector_alloc(_window / 2 / 5);
	gsl_vector* filteredx = gsl_vector_alloc(_window / 2 / 5);

	gsl_filter_median_workspace* filt = gsl_filter_median_alloc(10);

	for (int i = 0; i < _window / 2 / 5; ++i) {
		gsl_vector_set(convx,i, fftx[i] + 0.2 * fftx[i + 2 * i] + 0.1 * fftx[i + 4 * i]);
		gsl_vector_set(convy,i,ffty[i] + 0.2 * ffty[i + 2 * i] + 0.1 * ffty[i + 4 * i]);
	}

	gsl_filter_median(GSL_FILTER_END_TRUNCATE, convy, filteredy, filt);
	gsl_filter_median(GSL_FILTER_END_TRUNCATE, convx, filteredx, filt);

	p = filteredy->data;

	// First third
	gsl_vector_view y1 = gsl_vector_subvector(filteredy, 5 * _window / 2 / 500, _window / 2 / 5 / 3 - 5 * _window / 2 / 500);

	// Second third
	gsl_vector_view y2 = gsl_vector_subvector(filteredy, _window / 2 / 5 / 3, _window / 2 / 5 / 3);

	// Last third
	gsl_vector_view y3 = gsl_vector_subvector(filteredy, 2 * _window / 2 / 5 / 3, _window / 2 / 5 / 3);

	drive_freqs[0] = (gsl_vector_min_index(&y1.vector) + 5 * _window / 2 / 500) / 5;
	drive_freqs[1] = (gsl_vector_min_index(&y2.vector) + _window / 2 / 5 / 3) / 5;
	drive_freqs[2] = (gsl_vector_min_index(&y3.vector) + 2 * _window / 2 / 5 / 3) / 5;

	emit write_to_log(" Drive frequencies : " + QString::number(drive_freqs[0]) + " Hz, "
		+ QString::number(drive_freqs[1]) + " Hz, "
		+ QString::number(drive_freqs[2]) + " Hz");

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

double tf_model_error(const std::vector<double >& coeffs, std::vector<double>& grad, void* f_data) {

	processing_thread* proc = (processing_thread*)f_data;
	double sumsq = 0;

	for (int i = 2; i < tf_window; ++i) {

		sumsq += (proc->centroidy_d[i] + coeffs[0] * proc->centroidy_d[i - 1] + coeffs[1] * proc->centroidy_d[i - 2]) - 
			(coeffs[2]*proc->tf_input_arr[i-1] + coeffs[3]*proc->tf_input_arr[i-2]);
	}

	return sumsq;
}

void processing_thread::learn_transfer_function() {

	emit write_to_log(QString("Finding Pixel/DAC Transfer Function..."));

	adjust_framerate();

	if (camera.ResultingFrameRateAbs.GetValue() < 1000) {
		emit write_to_log(QString("Camera cannot acquire at 1 kHz") + QString::number(camera.ResultingFrameRateAbs()) + QString(" Hz)"));
	}
	else {

		centroidx_d.resize(tf_window);
		centroidy_d.resize(tf_window);

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

		tf_input_arr.resize(tf_window);
		
		tf_input_(tf_input_arr.data(), yDACmax, drive_freqs.data());

		GrabResultPtr_t ptr;

		teensy.write(QByteArray(1, LEARN_TF));
		
		teensy.write((const char*)drive_freqs.data(), 24);

		if (!teensy.waitForBytesWritten(100)) {
			emit write_to_log(QString("Synchronization error: could not write"));
			teensy.close();
			emit finished_analysis();
			return;
		}

		if (teensy.waitForReadyRead(1000)) {
			teensy.clear();
		}
		else {
			teensy.clear();
			emit write_to_log(QString("Synchronization error: could not read"));
			teensy.close();
			emit finished_analysis();
			return;
		}

		int missed = 0;
		int i = 0;
		
		camera.StartGrabbing();

		while (i < tf_window) {

			if (camera.RetrieveResult(30, ptr, Pylon::TimeoutHandling_Return)) {
				
				std::array<float,2> out = centroid(ptr, threshold);
				
				centroidx_d[i] = ptr->GetWidth() - out[0];
				centroidy_d[i] = ptr->GetHeight() - out[1];
				
				teensy.write(QByteArray(1, SYNC_FLAG));
				if (!teensy.waitForBytesWritten(100)) {
					emit write_to_log(QString("Synchronization error: could not write"));
					break;
				}
				++i;
			}
			else {
				++missed;
			}

			if (i % (tf_window / 100) == 0) {
				emit updateprogress(i);
			}
		}

		camera.StopGrabbing();
		teensy.close();

		centroidx_d.pop_front();
		centroidy_d.pop_front();
		tf_input_arr.pop_back();
		
		emit updateprogress(tf_window);
		
		std::vector<QVector<double>*> xy = { &centroidx_d,&centroidy_d };
		
		std::vector<double> filtered_section(section_width);
		std::vector<mat> sols;

		QVector<QVector<double>> to_plot;
		
		mat A_section(section_width - 2 * (n_taps / 2) - 2, 2);

		double* tf_input_section;
		double* centroid_section;

		for (QVector<double>* centroid : xy) {

			mat A;
			vec b;

			for (int j = 0; j < 3; ++j) {

				tf_input_section = tf_input_arr.data() + j * section_width;
				centroid_section = centroid->data() + j * section_width;

				//-----------------------------------------------------

				harmonics_filter filter(drive_freqs[j], 2);

				std::fill(filtered_section.begin(), filtered_section.end(), 0);

				filter.filter_vec(filtered_section, centroid_section);

				//-----------------------------------------------------
				
				tf_input_section += n_taps/2;
				centroid_section = filtered_section.data() + 2 * (n_taps / 2);
			

				for (int i = 0; i < section_width - 2 * (n_taps / 2) - 2; ++i) {

					A_section(i, 0) = tf_input_section[i + 2] - 2 * tf_input_section[i + 1] + tf_input_section[i]
						+ 2 * centroid_section[i + 1] - centroid_section[i];
					A_section(i, 1) = tf_input_section[i + 2] - tf_input_section[i + 1];
					
				}
				
				std::vector<double> sub_section(filtered_section.begin() + 2 * (n_taps / 2) + 2, filtered_section.end());

				A = join_cols(A, A_section);
				b = join_cols(b, vec(sub_section));

			}

			to_plot.append(QVector<double>(filtered_section.begin(), filtered_section.begin() + section_width / 2));
			sols.push_back(solve(A, b));

		}

		std::array<float, 2> tf_params = { sols[1](0,0), sols[1](1,0) };
		
		emit update_tf_plot(tf_params, to_plot[0], to_plot[1]);

		emit finished_analysis();
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
