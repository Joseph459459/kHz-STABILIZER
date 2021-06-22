#include "FASTSTABILIZER.h"

using namespace std::chrono;
#define T_START t1 = high_resolution_clock::now();
#define T_STOP t2 = high_resolution_clock::now(); qDebug() << duration_cast<microseconds>(t2-t1).count();
time_point<std::chrono::steady_clock> t1;
time_point<std::chrono::steady_clock> t2;
#include <iostream>
#include <fstream>


FASTSTABILIZER::FASTSTABILIZER(CDeviceInfo fb_info, QWidget* parent)
	: QMainWindow(parent), proc_thread(fb_info, this), x_filters(6), y_filters(6),animateTimer(this), FC(nullptr), MC(nullptr)
{
	ui.setupUi(this);

	connect(&proc_thread, &processing_thread::write_to_log, this, &FASTSTABILIZER::update_log);
	connect(this, &FASTSTABILIZER::send_cmd_line_data, &proc_thread, &processing_thread::receive_cmd_line_data);

	create_fft_plots();
	create_tf_plots();
		
}

FASTSTABILIZER::FASTSTABILIZER(CDeviceInfo fb_info, CDeviceInfo m_info, QWidget* parent)
	: QMainWindow(parent), proc_thread(fb_info, m_info, this), x_filters(6), y_filters(6), animateTimer(this), FC(nullptr), MC(nullptr)
{
	ui.setupUi(this);

	connect(&proc_thread, &processing_thread::write_to_log, this, &FASTSTABILIZER::update_log);
	connect(this, &FASTSTABILIZER::send_cmd_line_data, &proc_thread, &processing_thread::receive_cmd_line_data);

	create_fft_plots();
	create_tf_plots();

}

FASTSTABILIZER::~FASTSTABILIZER() {
	proc_thread.fb_cam.DestroyDevice();
	
	if (proc_thread.monitor_cam_enabled)
		proc_thread.monitor_cam.DestroyDevice();

	PylonTerminate();
}

void FASTSTABILIZER::on_stopButton_clicked() {

	proc_thread.acquiring = false;

	proc_thread.quit();

	if (!proc_thread.wait(3000)) {
		proc_thread.terminate();
		proc_thread.wait();
	}

	ui.stabilizeButton->setChecked(false);
}

void FASTSTABILIZER::on_stabilizeButton_clicked() {
	
	bool no_filters = true;
	
	for (int i = 0; i < 6; ++i) {
		no_filters = y_filters[i].ptr->data()->isEmpty();
		if (!no_filters)
			break;
		no_filters = x_filters[i].ptr->data()->isEmpty();
		if (!no_filters)
			break;
	}

	if (no_filters) {
		update_log("No filters placed.");
		return;
	}

	FC->safe_thread_close();

	FC->close();

	if (proc_thread.monitor_cam_enabled)
		MC->close();

	update_filter_params();

	proc_thread.plan = STABILIZE;
	proc_thread.start(QThread::TimeCriticalPriority);
}

void FASTSTABILIZER::update_log(QString q) {

	ui.console->appendPlainText(q);
	ui.console->appendPlainText(QString(" "));
	ui.stabilizeButton->setChecked(false);

}

void FASTSTABILIZER::on_learnButton_clicked() {

	qRegisterMetaType<GrabResultPtr_t>("GrabResultPtr_t");
	qRegisterMetaType<std::array<double, 3>>("std::array<double,3>");
	qRegisterMetaType<QVector<double>>("QVector<double>");

	proc_thread.fb_cam.Open();
	
	if (proc_thread.monitor_cam_enabled)
		proc_thread.monitor_cam.Open();

	FC = new feedback_cam(&proc_thread, this);
	connect(FC, &feedback_cam::write_to_log, this, &FASTSTABILIZER::update_log);

	qRegisterMetaType<QVector<QVector<double>>>("QVector<QVector<double>>");
	connect(&proc_thread, &processing_thread::send_feedback_ptr, FC, &feedback_cam::updateimage);
	connect(&proc_thread, &processing_thread::send_imgptr_blocking, FC, &feedback_cam::updateimage,Qt::BlockingQueuedConnection);

	connect(&proc_thread, &processing_thread::update_fft_plot, this, &FASTSTABILIZER::update_fft_plot);
	connect(&proc_thread, &processing_thread::update_tf_plot, this, &FASTSTABILIZER::update_tf_plot);

	FC->setAttribute(Qt::WA_DeleteOnClose);

	if (proc_thread.monitor_cam_enabled) {

		MC = new monitor_cam(&proc_thread, this);
		connect(MC, &monitor_cam::write_to_log, this, &FASTSTABILIZER::update_log);
		connect(&proc_thread, &processing_thread::send_monitor_ptr, MC, &monitor_cam::updateimage);
		connect(&proc_thread, &processing_thread::send_imgptr_blocking, MC, &monitor_cam::updateimage, Qt::BlockingQueuedConnection);
		connect(FC, &QDockWidget::destroyed, MC, &QDockWidget::deleteLater);
		MC->show();
	}

	FC->show();

	proc_thread.plan = STREAM;
	proc_thread.start();

}

void FASTSTABILIZER::create_tf_plots() {

	ui.tf_plot->plotLayout()->clear();
	ui.tf_plot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom);

	QCPAxisRect* tfaxes_x = new QCPAxisRect(ui.tf_plot, false);
	QCPAxisRect* tfaxes_y = new QCPAxisRect(ui.tf_plot, false);

	ui.tf_plot->plotLayout()->addElement(0, 0, tfaxes_x);
	ui.tf_plot->plotLayout()->addElement(1, 0, tfaxes_y);

	tfaxes_x->addAxes(QCPAxis::atLeft | QCPAxis::atBottom | QCPAxis::atRight);
	tfaxes_x->axis(QCPAxis::atBottom)->grid()->setVisible(true);
	tfaxes_x->axis(QCPAxis::atLeft)->grid()->setVisible(true);
	tfaxes_x->axis(QCPAxis::atRight)->grid()->setVisible(true);

	tfaxes_y->addAxes(QCPAxis::atLeft | QCPAxis::atBottom | QCPAxis::atRight);
	tfaxes_y->axis(QCPAxis::atBottom)->grid()->setVisible(true);
	tfaxes_y->axis(QCPAxis::atLeft)->grid()->setVisible(true);
	tfaxes_y->axis(QCPAxis::atRight)->grid()->setVisible(true);

	tfaxes_x->setRangeZoomAxes(tfaxes_x->axis(QCPAxis::atBottom), NULL);
	tfaxes_x->setRangeDragAxes(tfaxes_x->axis(QCPAxis::atBottom), NULL);

	tfaxes_y->setRangeZoomAxes(tfaxes_y->axis(QCPAxis::atBottom), NULL);
	tfaxes_y->setRangeDragAxes(tfaxes_y->axis(QCPAxis::atBottom), NULL);

	hysteresis_curves.resize(4);

	hysteresis_curves[0] = new QCPCurve(tfaxes_x->axis(QCPAxis::atBottom), tfaxes_x->axis(QCPAxis::atLeft));
	hysteresis_curves[1] = new QCPCurve(tfaxes_x->axis(QCPAxis::atBottom), tfaxes_x->axis(QCPAxis::atRight));
	hysteresis_curves[2] = new QCPCurve(tfaxes_y->axis(QCPAxis::atBottom), tfaxes_y->axis(QCPAxis::atLeft));
	hysteresis_curves[3] = new QCPCurve(tfaxes_y->axis(QCPAxis::atBottom), tfaxes_y->axis(QCPAxis::atRight));

	hysteresis_curves[0]->setPen(QPen(QColor(250, 130, 0, 200)));
	hysteresis_curves[1]->setPen(QPen(QColor(5, 0, 255, 200)));
	hysteresis_curves[2]->setPen(QPen(QColor(250, 130, 0, 200)));
	hysteresis_curves[3]->setPen(QPen(QColor(5, 0, 255, 200)));

	tfaxes_x->axis(QCPAxis::atLeft)->setLabel("DAC Output");
	tfaxes_x->axis(QCPAxis::atRight)->setLabel("Centroid X");

	tfaxes_y->axis(QCPAxis::atLeft)->setLabel("DAC Output");
	tfaxes_y->axis(QCPAxis::atRight)->setLabel("Centroid Y");

	ui.tf_plot->replot();

}

void FASTSTABILIZER::create_fft_plots() {

	ui.plot->plotLayout()->clear();
	QCPAxisRect* fftaxes_x = new QCPAxisRect(ui.plot, false);
	QCPAxisRect* fftaxes_y = new QCPAxisRect(ui.plot, false);

	ui.plot->plotLayout()->addElement(0, 0, fftaxes_x);
	ui.plot->plotLayout()->addElement(1, 0, fftaxes_y);

	fftaxes_x->addAxes(QCPAxis::atLeft | QCPAxis::atBottom);
	fftaxes_y->addAxes(QCPAxis::atLeft | QCPAxis::atBottom);

	fftaxes_x->axis(QCPAxis::atBottom)->grid()->setVisible(true);
	fftaxes_x->axis(QCPAxis::atLeft)->grid()->setVisible(true);

	fftaxes_y->axis(QCPAxis::atBottom)->grid()->setVisible(true);
	fftaxes_y->axis(QCPAxis::atLeft)->grid()->setVisible(true);

	fftaxes_x->setRangeZoomAxes(fftaxes_x->axis(QCPAxis::atBottom), NULL);
	fftaxes_y->setRangeDragAxes(fftaxes_x->axis(QCPAxis::atBottom), NULL);

	ui.plot->addGraph(fftaxes_x->axis(QCPAxis::atBottom), fftaxes_x->axis(QCPAxis::atLeft));
	ui.plot->addGraph(fftaxes_y->axis(QCPAxis::atBottom), fftaxes_y->axis(QCPAxis::atLeft));

	ui.plot->axisRect(0)->axis(QCPAxis::atLeft)->setLabel(QString("Centroid X"));
	ui.plot->axisRect(1)->axis(QCPAxis::atLeft)->setLabel(QString("Centroid Y"));

	ui.plot->graph(0)->setPen(QPen(QColor(5, 0, 255, 200)));
	ui.plot->graph(1)->setPen(QPen(QColor(5, 0, 255, 200)));

	ui.plot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectPlottables);

	ui.plot->graph(0)->setSelectable(QCP::stSingleData);

	ui.plot->graph(1)->setSelectable(QCP::stSingleData);

	connect(ui.plot->graph(0), qOverload<const QCPDataSelection&>(&QCPAbstractPlottable::selectionChanged),
		this, &FASTSTABILIZER::new_filter);
	connect(ui.plot->graph(1), qOverload<const QCPDataSelection&>(&QCPAbstractPlottable::selectionChanged),
		this, &FASTSTABILIZER::new_filter);

	ui.plot->replot();

	freqs.resize(_window / 2.0);

	for (int i = 0; i < _window / 2; ++i) {
		freqs[i] = (sampling_freq / 2.0) / (_window / 2.0) * i;
	}

	for (int i = 0; i < 6; ++i) {
		x_filters[i].ptr = ui.plot->addGraph(ui.plot->axisRect(0)->axis(QCPAxis::atBottom), ui.plot->axisRect(0)->axis(QCPAxis::atLeft));
		x_filters[i].ptr->setPen(QPen(QColor(250, 130, 0, 200)));
		x_filters[i].ptr->selectionDecorator()->setPen(QPen(QColor(250, 130, 0, 200)));
		x_filters[i].ptr->setBrush(QBrush(QColor(200, 130, 10, 40)));
		x_filters[i].ptr->selectionDecorator()->setBrush(QBrush(QColor(200, 130, 60, 77)));
		x_filters[i].ptr->setName(QString::number(i + 2));
		x_filters[i].ptr->data()->clear();
		connect(x_filters[i].ptr, qOverload<bool>(&QCPGraph::selectionChanged), this, &FASTSTABILIZER::animate_plot);
	}

	for (int i = 0; i < 6; ++i) {
		y_filters[i].ptr = ui.plot->addGraph(ui.plot->axisRect(1)->axis(QCPAxis::atBottom), ui.plot->axisRect(1)->axis(QCPAxis::atLeft));
		y_filters[i].ptr->setPen(QPen(QColor(250, 130, 0, 200)));
		y_filters[i].ptr->selectionDecorator()->setPen(QPen(QColor(250, 130, 0, 200)));
		y_filters[i].ptr->setBrush(QBrush(QColor(200, 130, 10, 40)));
		y_filters[i].ptr->selectionDecorator()->setBrush(QBrush(QColor(200, 130, 60, 77)));
		y_filters[i].ptr->setName(QString::number(i + 8));
		y_filters[i].ptr->data()->clear();
		connect(y_filters[i].ptr, qOverload<bool>(&QCPGraph::selectionChanged), this, &FASTSTABILIZER::animate_plot);

	}


	ui.plot->axisRect(0)->setRangeDragAxes(ui.plot->axisRect(0)->axis(QCPAxis::atBottom), NULL);
	ui.plot->axisRect(0)->setRangeZoomAxes(ui.plot->axisRect(0)->axis(QCPAxis::atBottom), NULL);
	ui.plot->axisRect(1)->setRangeDragAxes(ui.plot->axisRect(1)->axis(QCPAxis::atBottom), NULL);
	ui.plot->axisRect(1)->setRangeZoomAxes(ui.plot->axisRect(1)->axis(QCPAxis::atBottom), NULL);

	connect(ui.plot, &QCustomPlot::plottableDoubleClick, this, &FASTSTABILIZER::remove_filter);

	connect(&proc_thread, &QThread::started, &animateTimer, &QTimer::stop);


}

void FASTSTABILIZER::update_fft_plot(float rms_x, float rms_y, float peak_to_peak_x, float peak_to_peak_y) {

	ui.plot->graph(0)->setData(freqs, proc_thread.fftx);
	ui.plot->axisRect(0)->axis(QCPAxis::atLeft)->setRange(0, 1.25);
	ui.plot->graph(0)->rescaleKeyAxis();
	ui.plot->graph(1)->setData(freqs, proc_thread.ffty);
	ui.plot->axisRect(1)->axis(QCPAxis::atLeft)->setRange(0, 1.25);
	ui.plot->graph(1)->rescaleKeyAxis();
	ui.plot->replot();
	
	update_log(QString("rms x: ") + QString::number(rms_x,'f',2));
	update_log(QString("rms y: ") + QString::number(rms_y,'f',2));
	update_log(QString("peak to peak x: ") + QString::number(peak_to_peak_x,'f',2));
	update_log(QString("peak to peak y: ") + QString::number(peak_to_peak_y,'f',2));
}

void FASTSTABILIZER::update_tf_plot(QVector<QVector<double>> to_plot) {

	hysteresis_curves[1]->setData(to_plot[0],
		to_plot[1]);

	hysteresis_curves[3]->setData(to_plot[3],
		to_plot[4]);

	hysteresis_curves[0]->setData(to_plot[0],
		to_plot[2]);

	hysteresis_curves[2]->setData(to_plot[3],
		to_plot[5]);


	ui.tf_plot->rescaleAxes();
	ui.tf_plot->replot();

}

void FASTSTABILIZER::on_horizontalZoomButton_toggled(bool j) {

	if (j) {
			ui.plot->axisRect(0)->setRangeDragAxes(ui.plot->axisRect(0)->axis(QCPAxis::atBottom), NULL);
			ui.plot->axisRect(0)->setRangeZoomAxes(ui.plot->axisRect(0)->axis(QCPAxis::atBottom), NULL);
			ui.plot->axisRect(1)->setRangeDragAxes(ui.plot->axisRect(1)->axis(QCPAxis::atBottom), NULL);
			ui.plot->axisRect(1)->setRangeZoomAxes(ui.plot->axisRect(1)->axis(QCPAxis::atBottom), NULL);
			
			ui.tf_plot->axisRect(0)->setRangeDragAxes(ui.tf_plot->axisRect(0)->axis(QCPAxis::atBottom), NULL);
			ui.tf_plot->axisRect(0)->setRangeZoomAxes(ui.tf_plot->axisRect(0)->axis(QCPAxis::atBottom), NULL);
			ui.tf_plot->axisRect(1)->setRangeDragAxes(ui.tf_plot->axisRect(1)->axis(QCPAxis::atBottom), NULL);
			ui.tf_plot->axisRect(1)->setRangeZoomAxes(ui.tf_plot->axisRect(1)->axis(QCPAxis::atBottom), NULL);


	}
	else {
			ui.plot->axisRect(0)->setRangeDragAxes(NULL, ui.plot->axisRect(0)->axis(QCPAxis::atLeft));
			ui.plot->axisRect(0)->setRangeZoomAxes(NULL, ui.plot->axisRect(0)->axis(QCPAxis::atLeft));
			ui.plot->axisRect(1)->setRangeDragAxes(NULL, ui.plot->axisRect(1)->axis(QCPAxis::atLeft));
			ui.plot->axisRect(1)->setRangeZoomAxes(NULL, ui.plot->axisRect(1)->axis(QCPAxis::atLeft));

			QList<QCPAxis*> empty;
			empty.append(NULL);
			QList<QCPAxis*> axes;
			axes.append(ui.tf_plot->axisRect(0)->axis(QCPAxis::atLeft));
			axes.append(ui.tf_plot->axisRect(0)->axis(QCPAxis::atRight));

			ui.tf_plot->axisRect(0)->setRangeDragAxes(empty, axes);
			ui.tf_plot->axisRect(0)->setRangeZoomAxes(empty, axes);

			axes.clear();
			axes.append(ui.tf_plot->axisRect(1)->axis(QCPAxis::atLeft));
			axes.append(ui.tf_plot->axisRect(1)->axis(QCPAxis::atRight));

			ui.tf_plot->axisRect(1)->setRangeDragAxes(empty, axes);
			ui.tf_plot->axisRect(1)->setRangeZoomAxes(empty, axes);
		}
	
}

void FASTSTABILIZER::new_filter(const QCPDataSelection& p) {
	
	if (ui.plot->graph(0)->selected()) {
		int i;
		for (i = 0; i < x_filters.size(); ++i) {
			if (x_filters[i].ptr->data()->isEmpty())
				break;
		}

		if (i == x_filters.size())
			return;

		x_filters[i].ptr->setData(freqs, filtersim(freqs, p.dataRange().begin(), 50, 0));

		x_filters[i].N = 50;
		x_filters[i].graphdatapos = p.dataRange().begin();

		update_freq_label(50, x_filters[i].graphdatapos);

		ui.plot->replot();
	}

	else if (ui.plot->graph(1)->selected()) {
		int i;
		for (i = 0; i < y_filters.size(); ++i) {
			if (y_filters[i].ptr->data()->isEmpty())
				break;
		}

		if (i == x_filters.size())
			return;

		y_filters[i].ptr->setData(freqs, filtersim(freqs, p.dataRange().begin(), 50, 0));

		y_filters[i].N = 50;
		y_filters[i].graphdatapos = p.dataRange().begin();

		ui.plot->replot();

	}

}

void FASTSTABILIZER::keyPressEvent(QKeyEvent* e)
{
	if (e->key() == Qt::Key_Up || e->key() == Qt::Key_Down || e->key() == Qt::Key_Left || e->key() == Qt::Key_Right) {


		for (int i = 0; i < x_filters.size(); ++i) {
			
			if (x_filters[i].ptr->selected()) {

				if (e->key() == Qt::Key_Up) 
					x_filters[i].ptr->setData(freqs, filtersim(freqs, x_filters[i].graphdatapos, x_filters[i].N += 1, 0));

				else if (e->key() == Qt::Key_Down)
					x_filters[i].ptr->setData(freqs, filtersim(freqs, x_filters[i].graphdatapos, x_filters[i].N -= 1, 0));

				else if (e->key() == Qt::Key_Left) 
					x_filters[i].ptr->setData(freqs, filtersim(freqs, x_filters[i].graphdatapos -= 1, x_filters[i].N, 0));

				else if (e->key() == Qt::Key_Right) 
					x_filters[i].ptr->setData(freqs, filtersim(freqs, x_filters[i].graphdatapos += 1, x_filters[i].N, 0));

				update_freq_label(x_filters[i].N, x_filters[i].graphdatapos);
			}

		}



		for (int i = 0; i < y_filters.size(); ++i) {
			if (y_filters[i].ptr->selected()) {

				if (e->key() == Qt::Key_Up) 
					y_filters[i].ptr->setData(freqs, filtersim(freqs, y_filters[i].graphdatapos, y_filters[i].N += 1, 0));

				else if (e->key() == Qt::Key_Down) 
					y_filters[i].ptr->setData(freqs, filtersim(freqs, y_filters[i].graphdatapos, y_filters[i].N -= 1, 0));

				else if (e->key() == Qt::Key_Left) 
					y_filters[i].ptr->setData(freqs, filtersim(freqs, y_filters[i].graphdatapos -= 1, y_filters[i].N, 0));

				else if (e->key() == Qt::Key_Right) 
					y_filters[i].ptr->setData(freqs, filtersim(freqs, y_filters[i].graphdatapos += 1, y_filters[i].N, 0));

				update_freq_label(y_filters[i].N, y_filters[i].graphdatapos);

			}

		}

	}



	ui.plot->replot();

}

void FASTSTABILIZER::remove_filter(QCPAbstractPlottable* p, int j, QMouseEvent* e) {

	animateTimer.disconnect();
	animateTimer.stop();

	for (int i = 2;i < 14; ++i) {
		if (!QString::compare(p->name(), QString::number(i))) {

			ui.plot->graph(i)->data()->clear();
			ui.plotLabel->clear();
			ui.plot->replot();
			break;

		}
	}
}

void FASTSTABILIZER::animate_plot(bool j) {

	QCPGraph* graphptr = (QCPGraph*)sender();

	if (j) {

		int i = 2;
		for (; i < 14; ++i) {
			if (!QString::compare(graphptr->name(),QString::number(i)))
				break;
		}

		graphidx = i;
		animateTimer.stop();
		animateTimer.disconnect();
		connect(&animateTimer, &QTimer::timeout, this, [=](){this->animate(graphidx);});
		animateTimer.start(100);
	}
	else {
		ui.plotLabel->clear();
		animateTimer.stop();
		animateTimer.disconnect();
	}
}

void FASTSTABILIZER::animate(int i) {

	if (i < 8) {
		ui.plot->graph(i)->setData(freqs, filtersim(freqs, x_filters[i - 2].graphdatapos, x_filters[i - 2].N, phi += 0.3));
		ui.plot->replot(QCustomPlot::RefreshPriority::rpQueuedReplot);
		update_freq_label(x_filters[i-2].N, x_filters[i-2].graphdatapos);

	}
	else {
		ui.plot->graph(i)->setData(freqs, filtersim(freqs, y_filters[i - 8].graphdatapos, y_filters[i - 8].N, phi += 0.3));
		ui.plot->replot(QCustomPlot::RefreshPriority::rpQueuedReplot);
		update_freq_label(y_filters[i - 8].N, y_filters[i - 8].graphdatapos);

	}
}

void FASTSTABILIZER::update_freq_label(int N, int graphdatapos) {

	ui.plotLabel->setText("Freq: " + QString::number(graphdatapos / (_window / 2.0) * 500) +
		" Hz | Window: " + QString::number(N) + " ms ");

}

void FASTSTABILIZER::update_filter_params() {

	proc_thread.x_tones.clear();
	proc_thread.x_N.clear();

	proc_thread.y_tones.clear();
	proc_thread.y_N.clear();


	for (int i = 0; i < x_filters.size(); ++i) {

		if (!x_filters[i].ptr->data()->isEmpty()) {
			proc_thread.x_tones.push_back(x_filters[i].graphdatapos / (_window / 2.0) * (sampling_freq / 2));
			proc_thread.x_N.push_back(x_filters[i].N);
		}
	}


	for (int i = 0; i < y_filters.size(); ++i) {

		if (!y_filters[i].ptr->data()->isEmpty()) {
			proc_thread.y_tones.push_back(y_filters[i].graphdatapos / (_window / 2.0) * (sampling_freq / 2));
			proc_thread.y_N.push_back(y_filters[i].N);
		}
	}


}

void FASTSTABILIZER::on_cmdLineBox_returnPressed() {
	
	if (FC == nullptr) {
		ui.cmdLineBox->clear();
		return;
	}

	emit send_cmd_line_data(ui.cmdLineBox->text().split(','));

	ui.cmdLineBox->clear();

}

