#include "FASTSTABILIZER.h"
#include "camview.h"

using namespace std::chrono;
#define T_START t1 = high_resolution_clock::now();
#define T_STOP t2 = high_resolution_clock::now(); qDebug() << duration_cast<microseconds>(t2-t1).count();
time_point<std::chrono::steady_clock> t1;
time_point<std::chrono::steady_clock> t2;


FASTSTABILIZER::FASTSTABILIZER(CDeviceInfo c, QWidget* parent)
	: QMainWindow(parent), proc_thread(c, this), xfilters(6), yfilters(6),animateTimer(this)
{
	ui.setupUi(this);

	create_fft_plots();
		
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
	proc_thread.plan = STABILIZE;
	proc_thread.start(QThread::TimeCriticalPriority);
}

void FASTSTABILIZER::error_handling(QString q) {
	ui.console->appendPlainText(q);
	ui.console->appendPlainText(QString(" "));
	ui.stabilizeButton->setChecked(false);

}

void FASTSTABILIZER::on_learnButton_clicked() {

	camview* CV = new camview(&proc_thread, this);
	connect(&proc_thread, &processing_thread::updateimagesize, CV, &camview::updateimagesize, Qt::BlockingQueuedConnection);
	connect(CV, &camview::write_to_log, this, &FASTSTABILIZER::error_handling);
	qRegisterMetaType<GrabResultPtr_t>("GrabResultPtr_t");
	connect(&proc_thread, &processing_thread::sendImagePtr, CV, &camview::updateimage);
	connect(&proc_thread, &processing_thread::send_imgptr_blocking, CV, &camview::updateimage,Qt::BlockingQueuedConnection);

	connect(&proc_thread, &processing_thread::updatefftplot, this, &FASTSTABILIZER::updatefftplot);

	CV->setAttribute(Qt::WA_DeleteOnClose);

	CV->show();

	proc_thread.plan = STREAM;
	proc_thread.start();

}

void FASTSTABILIZER::create_fft_plots() {
	
	freqs.resize(window / 2.0);

	connect(&proc_thread, &processing_thread::write_to_log, this, &FASTSTABILIZER::error_handling);

	ui.plot->plotLayout()->clear();
	QCPAxisRect* fftaxesX = new QCPAxisRect(ui.plot, false);
	QCPAxisRect* fftaxesY = new QCPAxisRect(ui.plot, false);

	ui.plot->plotLayout()->addElement(0, 0, fftaxesX);
	ui.plot->plotLayout()->addElement(1, 0, fftaxesY);

	fftaxesX->addAxes(QCPAxis::atLeft | QCPAxis::atBottom);
	fftaxesY->addAxes(QCPAxis::atLeft | QCPAxis::atBottom);

	fftaxesX->axis(QCPAxis::atBottom)->grid()->setVisible(true);
	fftaxesX->axis(QCPAxis::atLeft)->grid()->setVisible(true);

	fftaxesY->axis(QCPAxis::atBottom)->grid()->setVisible(true);
	fftaxesY->axis(QCPAxis::atLeft)->grid()->setVisible(true);

	fftaxesX->setRangeZoomAxes(fftaxesX->axis(QCPAxis::atBottom), NULL);
	fftaxesY->setRangeDragAxes(fftaxesX->axis(QCPAxis::atBottom), NULL);

	ui.plot->addGraph(fftaxesX->axis(QCPAxis::atBottom), fftaxesX->axis(QCPAxis::atLeft));
	ui.plot->addGraph(fftaxesY->axis(QCPAxis::atBottom), fftaxesY->axis(QCPAxis::atLeft));

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

	for (int i = 0; i < window / 2; ++i) {
		freqs[i] = (sampling_freq / 2.0) / (window / 2.0) * i;
	}

	for (int i = 0; i < 6; ++i) {
		xfilters[i].ptr = ui.plot->addGraph(ui.plot->axisRect(0)->axis(QCPAxis::atBottom), ui.plot->axisRect(0)->axis(QCPAxis::atLeft));
		xfilters[i].ptr->setPen(QPen(QColor(250, 130, 0, 200)));
		xfilters[i].ptr->selectionDecorator()->setPen(QPen(QColor(250, 130, 0, 200)));
		xfilters[i].ptr->setBrush(QBrush(QColor(200, 130, 10, 40)));
		xfilters[i].ptr->selectionDecorator()->setBrush(QBrush(QColor(200, 130, 60, 77)));
		xfilters[i].ptr->setName(QString::number(i + 2));
		xfilters[i].ptr->data()->clear();
		connect(xfilters[i].ptr, qOverload<bool>(&QCPGraph::selectionChanged), this, &FASTSTABILIZER::animate_plot);
	}

	for (int i = 0; i < 6; ++i) {
		yfilters[i].ptr = ui.plot->addGraph(ui.plot->axisRect(1)->axis(QCPAxis::atBottom), ui.plot->axisRect(1)->axis(QCPAxis::atLeft));
		yfilters[i].ptr->setPen(QPen(QColor(250, 130, 0, 200)));
		yfilters[i].ptr->selectionDecorator()->setPen(QPen(QColor(250, 130, 0, 200)));
		yfilters[i].ptr->setBrush(QBrush(QColor(200, 130, 10, 40)));
		yfilters[i].ptr->selectionDecorator()->setBrush(QBrush(QColor(200, 130, 60, 77)));
		yfilters[i].ptr->setName(QString::number(i + 8));
		yfilters[i].ptr->data()->clear();
		connect(yfilters[i].ptr, qOverload<bool>(&QCPGraph::selectionChanged), this, &FASTSTABILIZER::animate_plot);

	}


	ui.plot->axisRect(0)->setRangeDragAxes(ui.plot->axisRect(0)->axis(QCPAxis::atBottom), NULL);
	ui.plot->axisRect(0)->setRangeZoomAxes(ui.plot->axisRect(0)->axis(QCPAxis::atBottom), NULL);
	ui.plot->axisRect(1)->setRangeDragAxes(ui.plot->axisRect(1)->axis(QCPAxis::atBottom), NULL);
	ui.plot->axisRect(1)->setRangeZoomAxes(ui.plot->axisRect(1)->axis(QCPAxis::atBottom), NULL);

	connect(ui.plot, &QCustomPlot::plottableDoubleClick, this, &FASTSTABILIZER::remove_filter);

	connect(&proc_thread, &QThread::started, &animateTimer, &QTimer::stop);



}

void FASTSTABILIZER::updatefftplot() {

	ui.plot->graph(0)->setData(freqs, proc_thread.LPfftx);
	ui.plot->axisRect(0)->axis(QCPAxis::atLeft)->setRange(0, 1.25);
	ui.plot->graph(0)->rescaleKeyAxis();
	ui.plot->graph(1)->setData(freqs, proc_thread.LPffty);
	ui.plot->axisRect(1)->axis(QCPAxis::atLeft)->setRange(0, 1.25);
	ui.plot->graph(1)->rescaleKeyAxis();
	ui.plot->replot();

}

void FASTSTABILIZER::on_horizontalZoomButton_toggled(bool j) {

	if (j) {
			ui.plot->axisRect(0)->setRangeDragAxes(ui.plot->axisRect(0)->axis(QCPAxis::atBottom), NULL);
			ui.plot->axisRect(0)->setRangeZoomAxes(ui.plot->axisRect(0)->axis(QCPAxis::atBottom), NULL);
			ui.plot->axisRect(1)->setRangeDragAxes(ui.plot->axisRect(1)->axis(QCPAxis::atBottom), NULL);
			ui.plot->axisRect(1)->setRangeZoomAxes(ui.plot->axisRect(1)->axis(QCPAxis::atBottom), NULL);
	}
	else {
			ui.plot->axisRect(0)->setRangeDragAxes(NULL, ui.plot->axisRect(0)->axis(QCPAxis::atLeft));
			ui.plot->axisRect(0)->setRangeZoomAxes(NULL, ui.plot->axisRect(0)->axis(QCPAxis::atLeft));
			ui.plot->axisRect(1)->setRangeDragAxes(NULL, ui.plot->axisRect(1)->axis(QCPAxis::atLeft));
			ui.plot->axisRect(1)->setRangeZoomAxes(NULL, ui.plot->axisRect(1)->axis(QCPAxis::atLeft));

		}
	
}

void FASTSTABILIZER::new_filter(const QCPDataSelection& p) {
	
	if (ui.plot->graph(0)->selected()) {
		int i;
		for (i = 0; i < xfilters.size(); ++i) {
			if (xfilters[i].ptr->data()->isEmpty())
				break;
		}

		if (i == xfilters.size())
			return;

		xfilters[i].ptr->setData(freqs, filtersim(freqs, p.dataRange().begin(), 50, 0));

		xfilters[i].N = 50;
		xfilters[i].graphdatapos = p.dataRange().begin();

		update_freq_label(50, xfilters[i].graphdatapos);

		ui.plot->replot();
	}

	else if (ui.plot->graph(1)->selected()) {
		int i;
		for (i = 0; i < yfilters.size(); ++i) {
			if (yfilters[i].ptr->data()->isEmpty())
				break;
		}

		if (i == xfilters.size())
			return;

		yfilters[i].ptr->setData(freqs, filtersim(freqs, p.dataRange().begin(), 50, 0));

		yfilters[i].N = 50;
		yfilters[i].graphdatapos = p.dataRange().begin();

		ui.plot->replot();

	}

}

void FASTSTABILIZER::keyPressEvent(QKeyEvent* e)
{
	if (e->key() == Qt::Key_Up || e->key() == Qt::Key_Down || e->key() == Qt::Key_Left || e->key() == Qt::Key_Right) {


		for (int i = 0; i < xfilters.size(); ++i) {
			
			if (xfilters[i].ptr->selected()) {

				if (e->key() == Qt::Key_Up) 
					xfilters[i].ptr->setData(freqs, filtersim(freqs, xfilters[i].graphdatapos, xfilters[i].N += 1, 0));

				else if (e->key() == Qt::Key_Down)
					xfilters[i].ptr->setData(freqs, filtersim(freqs, xfilters[i].graphdatapos, xfilters[i].N -= 1, 0));

				else if (e->key() == Qt::Key_Left) 
					xfilters[i].ptr->setData(freqs, filtersim(freqs, xfilters[i].graphdatapos -= 1, xfilters[i].N, 0));

				else if (e->key() == Qt::Key_Right) 
					xfilters[i].ptr->setData(freqs, filtersim(freqs, xfilters[i].graphdatapos += 1, xfilters[i].N, 0));

				update_freq_label(xfilters[i].N, xfilters[i].graphdatapos);
			}

		}



		for (int i = 0; i < yfilters.size(); ++i) {
			if (yfilters[i].ptr->selected()) {

				if (e->key() == Qt::Key_Up) 
					yfilters[i].ptr->setData(freqs, filtersim(freqs, yfilters[i].graphdatapos, yfilters[i].N += 1, 0));

				else if (e->key() == Qt::Key_Down) 
					yfilters[i].ptr->setData(freqs, filtersim(freqs, yfilters[i].graphdatapos, yfilters[i].N -= 1, 0));

				else if (e->key() == Qt::Key_Left) 
					yfilters[i].ptr->setData(freqs, filtersim(freqs, yfilters[i].graphdatapos -= 1, yfilters[i].N, 0));

				else if (e->key() == Qt::Key_Right) 
					yfilters[i].ptr->setData(freqs, filtersim(freqs, yfilters[i].graphdatapos += 1, yfilters[i].N, 0));

				update_freq_label(yfilters[i].N, yfilters[i].graphdatapos);

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
		ui.plot->graph(i)->setData(freqs, filtersim(freqs, xfilters[i - 2].graphdatapos, xfilters[i - 2].N, phi += 0.3));
		ui.plot->replot(QCustomPlot::RefreshPriority::rpQueuedReplot);
		update_freq_label(xfilters[i-2].N, xfilters[i-2].graphdatapos);

	}
	else {
		ui.plot->graph(i)->setData(freqs, filtersim(freqs, yfilters[i - 8].graphdatapos, yfilters[i - 8].N, phi += 0.3));
		ui.plot->replot(QCustomPlot::RefreshPriority::rpQueuedReplot);
		update_freq_label(yfilters[i - 8].N, yfilters[i - 8].graphdatapos);

	}
}

void FASTSTABILIZER::update_freq_label(int N, int graphdatapos) {

	ui.plotLabel->setText("Freq: " + QString::number(graphdatapos / (window / 2.0) * 500) +
		" Hz | Window: " + QString::number(N) + " ms ");

}
