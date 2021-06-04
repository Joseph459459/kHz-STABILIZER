#include "processing_thread.h"
//#include "nlopt.hpp"
#include <armadillo>
#include <gsl/gsl_vector.h>
#include <gsl/gsl_filter.h>
#include <gsl/gsl_math.h>
#include "stabilization.h"
#include <algorithm>

#define sq(x) ((x)*(x))

using namespace arma;

#define STABILIZE_DEBUG
#undef STABILIZE_DEBUG
struct harmonics_filter {

	harmonics_filter(int freq, int filter_halfwidth) {


		fundamental = new Filter(BPF, n_taps, 1000, freq - filter_halfwidth, freq + filter_halfwidth);
		third = new Filter(BPF, n_taps, 1000, freq + 2 * freq - filter_halfwidth, freq + 2 * freq + filter_halfwidth);
		fifth = new Filter(BPF, n_taps, 1000, freq + 4 * freq - filter_halfwidth, freq + 4 * freq + filter_halfwidth);


		//passband_ripple_compensate
		
		std::vector<int> my_freqs = { freq, freq + 2 * freq, freq + 4 * freq };
		std::vector<double> in(1500);
		std::vector<double> out(1500);

		for (int freq : my_freqs) {
		
			Filter* temp = new Filter(BPF, n_taps, 1000, freq - filter_halfwidth, freq + filter_halfwidth);

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

	void filter_vec(std::vector<double>& to_filter) {

		double result;

		for (int i = 0; i < to_filter.size(); ++i) {

			result = fundamental->do_sample(to_filter[i]) / gain_factors[0];
			result += third->do_sample(to_filter[i]) / gain_factors[1];
			result += fifth->do_sample(to_filter[i]) / gain_factors[2];
			to_filter[i] = result;

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

	fit_params.reserve(2);

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

void processing_thread::identify_initial_vals() {

	GrabResultPtr_t ptr;
	const int height = camera.Height();
	const int width = camera.Width();

	camera.MaxNumBuffer.SetValue(5);
	camera.StartGrabbing();

	std::vector<double> centroid_x(2000);
	std::vector<double> centroid_y(2000);
	float new_centroid[2];

	for (int i = 0; i < 2000; ++i) {

		if (camera.RetrieveResult(10, ptr, Pylon::TimeoutHandling_Return)) {

			centroid(ptr, height, width, new_centroid, threshold);

			centroid_x[i] = width - new_centroid[0];
			centroid_y[i] = height - new_centroid[1];

			qDebug() << centroid_y[i];

		}
	}

	camera.StopGrabbing();

	mean_x = std::accumulate(centroid_x.begin(), centroid_x.end(), 0.0) / centroid_x.size();
	mean_y = std::accumulate(centroid_y.begin(), centroid_y.end(), 0.0) / centroid_y.size();

	
	qDebug() << "before stabilize mean";
	qDebug() << mean_y;
}

void processing_thread::setup_stabilize() {

	int i;

	axes[0].num_tones = x_tones.size();
	axes[1].num_tones = y_tones.size();

	for (i = 0; i < x_tones.size(); ++i) {
		axes[0].w[i] = 2 * PI * x_tones[i] / 1000;
		axes[0].N[i] = x_N[i];
	}

	for (i = 0; i < y_tones.size(); ++i) {
		axes[1].w[i] = 2 * PI * y_tones[i] / 1000;
		axes[1].N[i] = y_N[i];
	}



	for (i = 0; i < 4; ++i) {
		axes[0].tf_params[i] = fit_params[0][i];
		axes[1].tf_params[i] = fit_params[1][i];
	}

	if (x_N.size() > 0) {
		axes[0].maxN = *std::max_element(x_N.begin(), x_N.end());
		axes[0].noise = new float[axes[0].maxN + 1]();
	}

	if (y_N.size() > 0) {
		axes[1].maxN = *std::max_element(y_N.begin(), y_N.end());
		axes[1].noise = new float[axes[1].maxN + 1]();
	}
	for (int i = 0; i < 3; ++i) {

		axes[0].target[i] = mean_vals[0];
		axes[1].target[i] = mean_vals[1];

		axes[0].DAC_cmds[i] = 2048;
		axes[1].DAC_cmds[i] = 2048;

	}


	axes[0].SET_POINT = mean_vals[0];
	axes[1].SET_POINT = mean_vals[1];

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

	bool p = true;

	p = teensy.setBaudRate(QSerialPort::Baud115200);
	p = teensy.setDataBits(QSerialPort::Data8);
	p = teensy.setParity(QSerialPort::NoParity);
	p = teensy.setStopBits(QSerialPort::OneStop);
	p = teensy.setFlowControl(QSerialPort::NoFlowControl);

	if (!p) {
		emit write_to_log(QString("Error configuring USB connection."));
		return;
	}
	else {
		emit write_to_log(QString("Port opened and configured"));
	}



#pragma endregion


	adjust_framerate();

	emit write_to_log(QString("Beginning Stabilization..."));

	if (camera.ResultingFrameRateAbs.GetValue() < 1000) {

		emit write_to_log(QString("Camera cannot acquire at 1 kHz (") + QString::number(camera.ResultingFrameRateAbs()) + QString(" Hz)"));
		emit finished_analysis();
		return;
	}

	int next_commands[2];
	
	teensy.write(QByteArray(1, STABILIZE));
	teensy.waitForBytesWritten(100);

	// Sleep for Actuator initialization / placement
	msleep(80);

	identify_initial_vals();

	setup_stabilize();

#ifdef STABILIZE_DEBUG

	float new_centroid[2];
	std::vector<double> noise(2000);
	for (int i = 26; i < 2026; ++i) {
		noise[i - 26] = 4.5 * cos(2 * PI * 45 / 1000 * i);
	}

	for (int i = 0; i < 500; ++i) {

		new_centroid[1] = noise[i] + axes[1].target[0];
		
		next_commands[1] = axes[1].next_DAC(new_centroid[1]);

		qDebug() << axes[1].DAC_cmds[0];
		qDebug() << axes[1].target[0];
		qDebug() << axes[1].noise_element;
		qDebug() << new_centroid[1];

		axes[1].post_step();
	}

#else

	std::vector<double> corrected(1000);

	GrabResultPtr_t ptr;
	const int height = camera.Height();
	const int width = camera.Width();
	float new_centroid[2];

	camera.MaxNumBuffer.SetValue(5);
	camera.StartGrabbing();
	int k = 0;
	acquiring = true;

	while (acquiring) {

		if (camera.RetrieveResult(10, ptr, Pylon::TimeoutHandling_Return)) {

			centroid(ptr, height, width, new_centroid, threshold);

			next_commands[1] = axes[1].next_DAC(height - new_centroid[1]);
			
			teensy.write(QByteArray(1, SYNC_FLAG));
			teensy.write((const char*)&next_commands[1],4);
			teensy.waitForBytesWritten(10);


			qDebug() << axes[1].DAC_cmds[0];
			qDebug() << axes[1].target[0];
			qDebug() << axes[1].noise_element;
			qDebug() << height - new_centroid[1];

			axes[1].post_step();

			++k;
			if (k == 20000)
				acquiring = false;
		
		}
	}
	camera.StopGrabbing();

#endif
	teensy.close();

	emit finished_analysis();

}

void processing_thread::stream() {

	adjust_framerate();

	camera.GevSCPSPacketSize.SetValue(1500);

	emit updateimagesize(camera.Width.GetValue(), camera.Height.GetValue());

	camera.MaxNumBuffer.SetValue(20);
	camera.StartGrabbing();
	GrabResultPtr_t ptr;
	acquiring = true;
	int k = 0;

	while (acquiring) {
		msleep(20);
		if (camera.RetrieveResult(30, ptr, Pylon::TimeoutHandling_Return)) {
			emit sendImagePtr(ptr);

			std::array<float, 2> p = centroid(ptr, threshold);

			if (! (k % 100))
				qDebug() << ptr->GetHeight() - p[1];

			k++;
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
		emit finished_analysis();
		return;
	}

	
	centroidx_f.resize(_window);
	centroidy_f.resize(_window);


	std::vector<GrabResultPtr_t> ptrs(_window);

	int missed = 0;
	int i = 0;


	camera.MaxNumBuffer.SetValue(_window);
	//camera.GevSCPSPacketSize.SetValue(((camera.Height.GetValue() * camera.Width.GetValue() + 14) / 4) * 4);
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

	// First third
	gsl_vector_view y1 = gsl_vector_subvector(filteredy, 5 * _window / 2 / 500, _window / 2 / 5 / 3 - 5 * _window / 2 / 500);

	// Second third
	gsl_vector_view y2 = gsl_vector_subvector(filteredy, _window / 2 / 5 / 3, _window / 2 / 5 / 3);

	// Last third
	gsl_vector_view y3 = gsl_vector_subvector(filteredy, 2 * _window / 2 / 5 / 3, _window / 2 / 5 / 3 - 10 * _window / 2 / 500);

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

			qDebug() << ptr->GetHeight() - out[1];

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
		const int height = camera.Height();
		const int width = camera.Width();
		float new_centroid[2];

		camera.MaxNumBuffer.SetValue(tf_window);
		camera.StartGrabbing();

		while (i < tf_window) {

			if (camera.RetrieveResult(2, ptr, Pylon::TimeoutHandling_Return)) {
				
				centroid(ptr,height,width,new_centroid,threshold);
				
				centroidx_d[i] = width - new_centroid[0];
				centroidy_d[i] = height - new_centroid[1];
				
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

		}

		camera.StopGrabbing();
		teensy.close();

		centroidx_d.pop_front();
		centroidy_d.pop_front();
		tf_input_arr.pop_back();

		emit updateprogress(tf_window);
		
		std::vector<QVector<double>*> xy = { &centroidx_d,&centroidy_d };
		std::vector<mat> sols;

		QVector<QVector<double>> to_plot;
		
		vec DAC_in(section_width - 2 * (n_taps / 2));
		vec centroid_in(section_width - 2 * (n_taps / 2));
		vec DAC_diff(section_width - 2 * (n_taps / 2) - 1);
		vec centroid_diff(section_width - 2 * (n_taps / 2) - 1);

		fit_params.clear();
		DAC_centroid_linear.clear();
		std::vector<double> model_RMSE;

		for (QVector<double>* centroid : xy) {

			vec dDAC_full;
			vec dcentroid_full;
			vec DAC_full;
			vec centroid_full;

			p = centroid->data();

			double sum = std::accumulate(centroid->begin(), centroid->end(), 0.0);
			double centroid_mean = sum / centroid->size();

			qDebug() << "TF mean";
			qDebug() << centroid_mean;

			mean_vals.push_back(centroid_mean);

			for (int j = 0; j < 3; ++j) {

				// EXTRACT SECTION
				std::vector<double> tf_input_section( (double*)tf_input_arr.data() + j * section_width, (double*)tf_input_arr.data() + (j + 1) * section_width);
				std::vector<double> centroid_section( (double*)centroid->data() + j * section_width, (double*)centroid->data() + (j + 1) * section_width);

				//-----------------------------------------------------
				
				// FILTER
				harmonics_filter filter(drive_freqs[j], 2);

				filter.filter_vec(centroid_section);

				//-----------------------------------------------------
				
				// ERASE FILTER LAG INTRODUCED BY PADDING ZEROS (n_taps)
				tf_input_section.erase(tf_input_section.begin(), tf_input_section.begin() + n_taps / 2);
				tf_input_section.erase(tf_input_section.end() - n_taps / 2, tf_input_section.end());
				centroid_section.erase(centroid_section.begin(),centroid_section.begin() + 2 * (n_taps / 2));

				sum = std::accumulate(centroid_section.begin(), centroid_section.end(),0.0);
				double filtered_mean = sum / centroid_section.size();
				
				qDebug() << "filtered mean";
				qDebug() << filtered_mean;

				// FIX MEAN OFFSET INTRODUCED BY FILTER
				for (double& d : centroid_section)
					d += centroid_mean - filtered_mean;

				// CREATE ARMADILLO OBJECTS
				DAC_in = vec(tf_input_section);
				centroid_in = vec(centroid_section);

				DAC_diff = diff(vec(tf_input_section));
				centroid_diff = diff(vec(centroid_section));
				
				
				if (j == 1) {

					to_plot.push_back(QVector<double>::fromStdVector(tf_input_section));
					to_plot.push_back(QVector<double>::fromStdVector(centroid_section));

				}

				// CONCATENATE SECTIONS FOR LEAST SQUARES
				DAC_in.shed_row(0);
				centroid_in.shed_row(0);

				DAC_full = join_cols(DAC_full, DAC_in);
				centroid_full = join_cols(centroid_full, centroid_in);
			
				dDAC_full = join_cols(dDAC_full, DAC_diff);
				dcentroid_full = join_cols(dcentroid_full, centroid_diff);

			}

			mat lin_sol = solve(join_rows(DAC_full, ones<vec>(DAC_full.n_rows)), centroid_full);

			vec errors = centroid_full - (lin_sol(0) * DAC_full + lin_sol(1));

			mat correction = solve(join_rows(dDAC_full, dcentroid_full), errors);

			std::array<double,4> fit = {lin_sol(0), lin_sol(1), correction(0), correction(1)};

			fit_params.push_back(fit);

			model_RMSE.push_back(sqrt(arma::sum(square(errors - (correction(0)*dDAC_full + correction(1)*dcentroid_full)))/ dcentroid_full.n_rows));

		}

		
		emit write_to_log(" centroid_y = " + QString::number(fit_params[1][0], 'e', 2) + "x + " +
			QString::number(fit_params[1][1], 'e', 2) + " - (" +
			QString::number(fit_params[1][2], 'e', 2) + "x' + " +
			QString::number(fit_params[1][3], 'e', 2) + "centroid_y') ");
		
		emit write_to_log(" centroid_x = " + QString::number(fit_params[1][0], 'e', 2) + "x + " +
			QString::number(fit_params[1][1], 'e', 2) + " - (" +
			QString::number(fit_params[1][2], 'e', 2) + "x' + " +
			QString::number(fit_params[1][3], 'e', 2) + "centroid_x'') ");


		emit write_to_log(" x model RMSE: " + QString::number(model_RMSE[0],'g',2) + " px");
		emit write_to_log(" y model RMSE: " + QString::number(model_RMSE[1],'g',2) + " px");
	
		emit update_tf_plot(to_plot[0], to_plot[1], to_plot[2], to_plot[3]);

		emit finished_analysis();
	}

}

void processing_thread::run() {

	switch (plan) {
	case STABILIZE:
		stabilize();
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
		break;
	default:
		break;
	}
}
