#include "FASTSTABILIZER.h"

using namespace std::chrono;
#define T_START t1 = high_resolution_clock::now();
#define T_STOP t2 = high_resolution_clock::now(); qDebug() << duration_cast<microseconds>(t2-t1).count();
time_point<std::chrono::steady_clock> t1;
time_point<std::chrono::steady_clock> t2;


FASTSTABILIZER::FASTSTABILIZER(CDeviceInfo c, QWidget* parent)
	: QMainWindow(parent), proc_thread(c, this), x_filters(6), y_filters(6),animateTimer(this)
{
	ui.setupUi(this);

	connect(&proc_thread, &processing_thread::write_to_log, this, &FASTSTABILIZER::error_handling);

	create_fft_plots();
	create_tf_plots();
}

FASTSTABILIZER::~FASTSTABILIZER() {
	PylonTerminate();
}

void FASTSTABILIZER::on_stopButton_clicked() {

	proc_thread.acquiring = false;

	//proc_thread.quit();

	//if (!proc_thread.wait(3000)) {
	//	proc_thread.terminate();
	//	proc_thread.wait();
	//}

	ui.stabilizeButton->setChecked(false);
}

void FASTSTABILIZER::on_stabilizeButton_clicked() {
	
	CV->safe_thread_close();

	CV->close();

	update_procthread();

	proc_thread.plan = STABILIZE;
	proc_thread.start(QThread::TimeCriticalPriority);
}

void FASTSTABILIZER::error_handling(QString q) {
	ui.console->appendPlainText(q);
	ui.console->appendPlainText(QString(" "));
	ui.stabilizeButton->setChecked(false);

}

void FASTSTABILIZER::on_learnButton_clicked() {

	CV = new camview(&proc_thread, this);
	connect(&proc_thread, &processing_thread::updateimagesize, CV, &camview::updateimagesize, Qt::BlockingQueuedConnection);
	connect(CV, &camview::write_to_log, this, &FASTSTABILIZER::error_handling);
	qRegisterMetaType<GrabResultPtr_t>("GrabResultPtr_t");
	qRegisterMetaType<std::array<double,3>>("std::array<double,3>");
	connect(&proc_thread, &processing_thread::sendImagePtr, CV, &camview::updateimage);
	connect(&proc_thread, &processing_thread::send_imgptr_blocking, CV, &camview::updateimage,Qt::BlockingQueuedConnection);

	qRegisterMetaType<QVector<double>>("QVector<double>");

	connect(&proc_thread, &processing_thread::update_fft_plot, this, &FASTSTABILIZER::update_fft_plot);
	connect(&proc_thread, &processing_thread::update_tf_plot, this, &FASTSTABILIZER::update_tf_plot);

	CV->setAttribute(Qt::WA_DeleteOnClose);

	CV->show();

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

void FASTSTABILIZER::update_fft_plot() {

	ui.plot->graph(0)->setData(freqs, proc_thread.fftx);
	ui.plot->axisRect(0)->axis(QCPAxis::atLeft)->setRange(0, 1.25);
	ui.plot->graph(0)->rescaleKeyAxis();
	ui.plot->graph(1)->setData(freqs, proc_thread.ffty);
	ui.plot->axisRect(1)->axis(QCPAxis::atLeft)->setRange(0, 1.25);
	ui.plot->graph(1)->rescaleKeyAxis();
	ui.plot->replot();

}

void FASTSTABILIZER::update_tf_plot(QVector<double> DAC_x, QVector<double> filtered_x, QVector<double> DAC_y, QVector<double> filtered_y) {

	hysteresis_curves[1]->setData(DAC_x
	, filtered_x);

	hysteresis_curves[3]->setData(DAC_y
	, filtered_y);

	QVector<double> sim_centroid_y(filtered_y.size() - 2);
	QVector<double> sim_centroid_x(filtered_y.size() - 2);
	
	for (int i = 2; i < filtered_y.size(); ++i) {
		
		sim_centroid_y[i - 2] = proc_thread.sol[1] * (DAC_y[i] - 2 * DAC_y[i - 1] + DAC_y[i - 2]) + 2 * filtered_y[i - 1] - filtered_y[i - 2];

		for (int j = 0; j < 4; ++j)
			sim_centroid_y[i - 2] -= proc_thread.fit_params[1][j] * std::pow(DAC_y[i] - DAC_y[i - 1], j);


		sim_centroid_x[i - 2] = proc_thread.sol[0] * (DAC_x[i] - 2 * DAC_x[i - 1] + DAC_x[i - 2]) + 2 * filtered_x[i - 1] - filtered_x[i - 2];
		
		for (int j = 1; j < 4; ++j)
			sim_centroid_x[i - 2] -= proc_thread.fit_params[0][j] * std::pow(DAC_x[i] - DAC_x[i - 1], j);
		

	}

	DAC_y.erase(DAC_y.begin(), DAC_y.begin() + 2);
	DAC_x.erase(DAC_x.begin(), DAC_x.begin() + 2);


	hysteresis_curves[0]->setData(DAC_x
		, sim_centroid_x);

	hysteresis_curves[2]->setData( DAC_y
		, sim_centroid_y);


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

void FASTSTABILIZER::update_procthread() {

	proc_thread.x_tones.clear();
	proc_thread.x_N.clear();

	proc_thread.y_tones.clear();
	proc_thread.y_N.clear();


	for (int i = 0; i < x_filters.size(); ++i) {

		if (!x_filters[i].ptr->data()->isEmpty()) {
			proc_thread.x_tones.push_back(x_filters[i].graphdatapos * (_window / 2.0) * 500);
			proc_thread.x_N.push_back(x_filters[i].N);
		}
	}


	for (int i = 0; i < y_filters.size(); ++i) {

		if (!y_filters[i].ptr->data()->isEmpty()) {
			proc_thread.y_tones.push_back(y_filters[i].graphdatapos * (_window / 2.0) * 500);
			proc_thread.y_N.push_back(y_filters[i].N);
		}
	}


}