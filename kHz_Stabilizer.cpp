#include "kHz_Stabilizer.h"

using namespace std::chrono;
#define T_START t1 = high_resolution_clock::now();
#define T_STOP t2 = high_resolution_clock::now(); qDebug() << duration_cast<microseconds>(t2-t1).count();
time_point<std::chrono::steady_clock> t1;
time_point<std::chrono::steady_clock> t2;
#include <iostream>
#include <fstream>


kHz_Stabilizer::kHz_Stabilizer(CDeviceInfo fb_info, QWidget* parent)
    : QMainWindow(parent), proc_thread(fb_info, this), animateTimer(this), FC(nullptr), MC(nullptr)
{
	ui.setupUi(this);

    connect(&proc_thread, &processing_thread::write_to_log, this, &kHz_Stabilizer::update_log);
    connect(this, &kHz_Stabilizer::send_cmd_line_data, &proc_thread, &processing_thread::receive_cmd_line_data);

    filters[0].resize(6);
    filters[1].resize(6);

	create_fft_plots();
    create_system_response_plots();
		
}

kHz_Stabilizer::kHz_Stabilizer(CDeviceInfo fb_info, CDeviceInfo m_info, QWidget* parent)
    : QMainWindow(parent), proc_thread(fb_info, m_info, this), animateTimer(this), FC(nullptr), MC(nullptr)
{
	ui.setupUi(this);

    connect(&proc_thread, &processing_thread::write_to_log, this, &kHz_Stabilizer::update_log);
    connect(this, &kHz_Stabilizer::send_cmd_line_data, &proc_thread, &processing_thread::receive_cmd_line_data);

    filters[0].resize(6);
    filters[1].resize(6);

	create_fft_plots();
    create_system_response_plots();

}

kHz_Stabilizer::~kHz_Stabilizer() {
	proc_thread.fb_cam.DestroyDevice();
	
	if (proc_thread.monitor_cam_enabled)
		proc_thread.monitor_cam.DestroyDevice();

	PylonTerminate();
}

void kHz_Stabilizer::on_stopButton_clicked() {

	proc_thread.acquiring = false;

	proc_thread.quit();

	if (!proc_thread.wait(3000)) {
		proc_thread.terminate();
		proc_thread.wait();
	}

	ui.stabilizeButton->setChecked(false);
}

void kHz_Stabilizer::on_stabilizeButton_clicked() {
	
	bool no_filters = true;
	
	for (int i = 0; i < 6; ++i) {
        no_filters = filters[1][i].ptr->data()->isEmpty();
		if (!no_filters)
			break;
        no_filters = filters[0][i].ptr->data()->isEmpty();
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

    proc_thread.run_plan = STABILIZE;
	proc_thread.start(QThread::TimeCriticalPriority);
}

void kHz_Stabilizer::update_log(QString q) {

	ui.console->appendPlainText(q);
	ui.console->appendPlainText(QString(" "));
	ui.stabilizeButton->setChecked(false);

}

void kHz_Stabilizer::on_learnButton_clicked() {

	qRegisterMetaType<GrabResultPtr_t>("GrabResultPtr_t");
	qRegisterMetaType<std::array<double, 3>>("std::array<double,3>");
	qRegisterMetaType<QVector<double>>("QVector<double>");

	proc_thread.fb_cam.Open();
	
	if (proc_thread.monitor_cam_enabled)
		proc_thread.monitor_cam.Open();

	FC = new feedback_cam(&proc_thread, this);
    connect(FC, &feedback_cam::write_to_log, this, &kHz_Stabilizer::update_log);

	qRegisterMetaType<QVector<QVector<double>>>("QVector<QVector<double>>");
	connect(&proc_thread, &processing_thread::send_feedback_ptr, FC, &feedback_cam::updateimage);
	connect(&proc_thread, &processing_thread::send_imgptr_blocking, FC, &feedback_cam::updateimage,Qt::BlockingQueuedConnection);

    connect(&proc_thread, &processing_thread::update_fft_plot, this, &kHz_Stabilizer::update_fft_plot);
    connect(&proc_thread, &processing_thread::update_local_sys_response_plot, this, &kHz_Stabilizer::update_local_sys_response_plot);
    connect(&proc_thread, &processing_thread::update_total_sys_response_plot, this, &kHz_Stabilizer::update_total_sys_response_plot);

	FC->setAttribute(Qt::WA_DeleteOnClose);

	if (proc_thread.monitor_cam_enabled) {

		MC = new monitor_cam(&proc_thread, this);
        connect(MC, &monitor_cam::write_to_log, this, &kHz_Stabilizer::update_log);
		connect(&proc_thread, &processing_thread::send_monitor_ptr, MC, &monitor_cam::updateimage);
		connect(&proc_thread, &processing_thread::send_imgptr_blocking, MC, &monitor_cam::updateimage, Qt::BlockingQueuedConnection);
		connect(FC, &QDockWidget::destroyed, MC, &QDockWidget::deleteLater);
		MC->show();
	}

	FC->show();

    proc_thread.run_plan = STREAM;
    proc_thread.start();

}

void kHz_Stabilizer::create_system_response_plots() {

    ui.local_response_plot->plotLayout()->clear();
    ui.local_response_plot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom);

    QCPAxisRect* loc_axes_x = new QCPAxisRect(ui.local_response_plot, false);
    QCPAxisRect* loc_axes_y = new QCPAxisRect(ui.local_response_plot, false);

    ui.local_response_plot->plotLayout()->addElement(0, 0, loc_axes_x);
    ui.local_response_plot->plotLayout()->addElement(1, 0, loc_axes_y);

    loc_axes_x->addAxes(QCPAxis::atLeft | QCPAxis::atBottom | QCPAxis::atRight);
    loc_axes_x->axis(QCPAxis::atBottom)->grid()->setVisible(true);
    loc_axes_x->axis(QCPAxis::atLeft)->grid()->setVisible(true);
    loc_axes_x->axis(QCPAxis::atRight)->grid()->setVisible(true);

    loc_axes_y->addAxes(QCPAxis::atLeft | QCPAxis::atBottom | QCPAxis::atRight);
    loc_axes_y->axis(QCPAxis::atBottom)->grid()->setVisible(true);
    loc_axes_y->axis(QCPAxis::atLeft)->grid()->setVisible(true);
    loc_axes_y->axis(QCPAxis::atRight)->grid()->setVisible(true);

    loc_axes_x->setRangeZoomAxes(loc_axes_x->axis(QCPAxis::atBottom), NULL);
    loc_axes_x->setRangeDragAxes(loc_axes_x->axis(QCPAxis::atBottom), NULL);

    loc_axes_y->setRangeZoomAxes(loc_axes_y->axis(QCPAxis::atBottom), NULL);
    loc_axes_y->setRangeDragAxes(loc_axes_y->axis(QCPAxis::atBottom), NULL);

	hysteresis_curves.resize(4);

    hysteresis_curves[0] = new QCPCurve(loc_axes_x->axis(QCPAxis::atBottom), loc_axes_x->axis(QCPAxis::atLeft));
    hysteresis_curves[1] = new QCPCurve(loc_axes_x->axis(QCPAxis::atBottom), loc_axes_x->axis(QCPAxis::atRight));
    hysteresis_curves[2] = new QCPCurve(loc_axes_y->axis(QCPAxis::atBottom), loc_axes_y->axis(QCPAxis::atLeft));
    hysteresis_curves[3] = new QCPCurve(loc_axes_y->axis(QCPAxis::atBottom), loc_axes_y->axis(QCPAxis::atRight));

	hysteresis_curves[0]->setPen(QPen(QColor(250, 130, 0, 200)));
	hysteresis_curves[1]->setPen(QPen(QColor(5, 0, 255, 200)));
	hysteresis_curves[2]->setPen(QPen(QColor(250, 130, 0, 200)));
	hysteresis_curves[3]->setPen(QPen(QColor(5, 0, 255, 200)));

    loc_axes_x->axis(QCPAxis::atLeft)->setLabel("DAC Output");
    loc_axes_x->axis(QCPAxis::atRight)->setLabel("Centroid X");

    loc_axes_y->axis(QCPAxis::atLeft)->setLabel("DAC Output");
    loc_axes_y->axis(QCPAxis::atRight)->setLabel("Centroid Y");

    ui.local_response_plot->replot();

    /*-----------------------------------------------------------------*/

    ui.total_response_plot->plotLayout()->clear();
    ui.total_response_plot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom);

    QCPAxisRect* tot_axes_x = new QCPAxisRect(ui.total_response_plot, false);
    QCPAxisRect* tot_axes_y = new QCPAxisRect(ui.total_response_plot, false);

    ui.total_response_plot->plotLayout()->addElement(0, 0, tot_axes_x);
    ui.total_response_plot->plotLayout()->addElement(1, 0, tot_axes_y);

    tot_axes_x->addAxes(QCPAxis::atLeft | QCPAxis::atBottom | QCPAxis::atRight);
    tot_axes_x->axis(QCPAxis::atBottom)->grid()->setVisible(true);
    tot_axes_x->axis(QCPAxis::atLeft)->grid()->setVisible(true);
    tot_axes_x->axis(QCPAxis::atRight)->grid()->setVisible(true);

    tot_axes_y->addAxes(QCPAxis::atLeft | QCPAxis::atBottom | QCPAxis::atRight);
    tot_axes_y->axis(QCPAxis::atBottom)->grid()->setVisible(true);
    tot_axes_y->axis(QCPAxis::atLeft)->grid()->setVisible(true);
    tot_axes_y->axis(QCPAxis::atRight)->grid()->setVisible(true);

    tot_axes_x->setRangeZoomAxes(tot_axes_x->axis(QCPAxis::atBottom), NULL);
    tot_axes_x->setRangeDragAxes(tot_axes_x->axis(QCPAxis::atBottom), NULL);

    tot_axes_y->setRangeZoomAxes(tot_axes_y->axis(QCPAxis::atBottom), NULL);
    tot_axes_y->setRangeDragAxes(tot_axes_y->axis(QCPAxis::atBottom), NULL);

    tot_response_graphs.resize(4);

    tot_response_graphs[0] = new QCPGraph(tot_axes_x->axis(QCPAxis::atBottom), tot_axes_x->axis(QCPAxis::atLeft));
    tot_response_graphs[1] = new QCPGraph(tot_axes_x->axis(QCPAxis::atBottom), tot_axes_x->axis(QCPAxis::atRight));
    tot_response_graphs[2] = new QCPGraph(tot_axes_y->axis(QCPAxis::atBottom), tot_axes_y->axis(QCPAxis::atLeft));
    tot_response_graphs[3] = new QCPGraph(tot_axes_y->axis(QCPAxis::atBottom), tot_axes_y->axis(QCPAxis::atRight));

    tot_response_graphs[0]->setPen(QPen(QColor(250, 130, 0, 200)));
    tot_response_graphs[1]->setPen(QPen(QColor(5, 0, 255, 200)));
    tot_response_graphs[2]->setPen(QPen(QColor(250, 130, 0, 200)));
    tot_response_graphs[3]->setPen(QPen(QColor(5, 0, 255, 200)));

    tot_axes_x->axis(QCPAxis::atLeft)->setLabel("Magnitude X");
    tot_axes_x->axis(QCPAxis::atRight)->setLabel("Magnitude Y");

    tot_axes_y->axis(QCPAxis::atLeft)->setLabel("Phase X");
    tot_axes_y->axis(QCPAxis::atRight)->setLabel("Phase Y");

    ui.total_response_plot->replot();

}

void kHz_Stabilizer::create_fft_plots() {

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
        this, &kHz_Stabilizer::new_filter);
	connect(ui.plot->graph(1), qOverload<const QCPDataSelection&>(&QCPAbstractPlottable::selectionChanged),
        this, &kHz_Stabilizer::new_filter);

	ui.plot->replot();

	freqs.resize(fft_window / 2.0);

	for (int i = 0; i < fft_window / 2; ++i) {
		freqs[i] = (sampling_freq / 2.0) / (fft_window / 2.0) * i;
	}

	for (int i = 0; i < 6; ++i) {
        filters[0][i].ptr = ui.plot->addGraph(ui.plot->axisRect(0)->axis(QCPAxis::atBottom), ui.plot->axisRect(0)->axis(QCPAxis::atLeft));
        filters[0][i].ptr->setPen(QPen(QColor(250, 130, 0, 200)));
        filters[0][i].ptr->selectionDecorator()->setPen(QPen(QColor(250, 130, 0, 200)));
        filters[0][i].ptr->setBrush(QBrush(QColor(200, 130, 10, 40)));
        filters[0][i].ptr->selectionDecorator()->setBrush(QBrush(QColor(200, 130, 60, 77)));
        filters[0][i].ptr->setName(QString::number(i + 2));
        filters[0][i].ptr->data()->clear();
        connect(filters[0][i].ptr, qOverload<bool>(&QCPGraph::selectionChanged), this, &kHz_Stabilizer::animate_plot);
	}

	for (int i = 0; i < 6; ++i) {
        filters[1][i].ptr = ui.plot->addGraph(ui.plot->axisRect(1)->axis(QCPAxis::atBottom), ui.plot->axisRect(1)->axis(QCPAxis::atLeft));
        filters[1][i].ptr->setPen(QPen(QColor(250, 130, 0, 200)));
        filters[1][i].ptr->selectionDecorator()->setPen(QPen(QColor(250, 130, 0, 200)));
        filters[1][i].ptr->setBrush(QBrush(QColor(200, 130, 10, 40)));
        filters[1][i].ptr->selectionDecorator()->setBrush(QBrush(QColor(200, 130, 60, 77)));
        filters[1][i].ptr->setName(QString::number(i + 8));
        filters[1][i].ptr->data()->clear();
        connect(filters[1][i].ptr, qOverload<bool>(&QCPGraph::selectionChanged), this, &kHz_Stabilizer::animate_plot);

	}


	ui.plot->axisRect(0)->setRangeDragAxes(ui.plot->axisRect(0)->axis(QCPAxis::atBottom), NULL);
	ui.plot->axisRect(0)->setRangeZoomAxes(ui.plot->axisRect(0)->axis(QCPAxis::atBottom), NULL);
	ui.plot->axisRect(1)->setRangeDragAxes(ui.plot->axisRect(1)->axis(QCPAxis::atBottom), NULL);
	ui.plot->axisRect(1)->setRangeZoomAxes(ui.plot->axisRect(1)->axis(QCPAxis::atBottom), NULL);

    connect(ui.plot, &QCustomPlot::plottableDoubleClick, this, &kHz_Stabilizer::remove_filter);

	connect(&proc_thread, &QThread::started, &animateTimer, &QTimer::stop);


}

void kHz_Stabilizer::update_fft_plot(float rms_x, float rms_y, float peak_to_peak_x, float peak_to_peak_y) {

    for (int i = 0; i < 2; ++i){

    ui.plot->graph(i)->setData(freqs, QVector<double>(proc_thread.fft[i].begin(),proc_thread.fft[i].begin() + fft_window/2));
    ui.plot->graph(i)->rescaleAxes();
    ui.plot->replot();

}	
	update_log(QString("rms x: ") + QString::number(rms_x,'f',2));
	update_log(QString("rms y: ") + QString::number(rms_y,'f',2));
	update_log(QString("peak to peak x: ") + QString::number(peak_to_peak_x,'f',2));
	update_log(QString("peak to peak y: ") + QString::number(peak_to_peak_y,'f',2));
}

void kHz_Stabilizer::update_local_sys_response_plot(QVector<QVector<double>> to_plot) {

    hysteresis_curves[1]->setData(to_plot[0],
        to_plot[1]);

    hysteresis_curves[3]->setData(to_plot[3],
        to_plot[4]);

    hysteresis_curves[0]->setData(to_plot[0],
        to_plot[2]);

    hysteresis_curves[2]->setData(to_plot[3],
        to_plot[5]);


    ui.local_response_plot->rescaleAxes();
    ui.local_response_plot->replot();

}

void kHz_Stabilizer::update_total_sys_response_plot(QVector<QVector<double>> to_plot) {

    QVector<double> keys(to_plot[0].size());

    for (int i = 0; i < keys.size(); ++i)
        keys[i] =  500 * i / keys.size(); ;

    tot_response_graphs[1]->setData(keys,
        to_plot[2]);

    tot_response_graphs[3]->setData(keys,
        to_plot[3]);

    ui.total_response_plot->rescaleAxes();
    ui.total_response_plot->replot();

}

void kHz_Stabilizer::on_horizontalZoomButton_toggled(bool j) {

	if (j) {
			ui.plot->axisRect(0)->setRangeDragAxes(ui.plot->axisRect(0)->axis(QCPAxis::atBottom), NULL);
			ui.plot->axisRect(0)->setRangeZoomAxes(ui.plot->axisRect(0)->axis(QCPAxis::atBottom), NULL);
			ui.plot->axisRect(1)->setRangeDragAxes(ui.plot->axisRect(1)->axis(QCPAxis::atBottom), NULL);
			ui.plot->axisRect(1)->setRangeZoomAxes(ui.plot->axisRect(1)->axis(QCPAxis::atBottom), NULL);
			
            ui.local_response_plot->axisRect(0)->setRangeDragAxes(ui.local_response_plot->axisRect(0)->axis(QCPAxis::atBottom), NULL);
            ui.local_response_plot->axisRect(0)->setRangeZoomAxes(ui.local_response_plot->axisRect(0)->axis(QCPAxis::atBottom), NULL);
            ui.local_response_plot->axisRect(1)->setRangeDragAxes(ui.local_response_plot->axisRect(1)->axis(QCPAxis::atBottom), NULL);
            ui.local_response_plot->axisRect(1)->setRangeZoomAxes(ui.local_response_plot->axisRect(1)->axis(QCPAxis::atBottom), NULL);

            ui.total_response_plot->axisRect(0)->setRangeDragAxes(ui.total_response_plot->axisRect(0)->axis(QCPAxis::atBottom), NULL);
            ui.total_response_plot->axisRect(0)->setRangeZoomAxes(ui.total_response_plot->axisRect(0)->axis(QCPAxis::atBottom), NULL);
            ui.total_response_plot->axisRect(1)->setRangeDragAxes(ui.total_response_plot->axisRect(1)->axis(QCPAxis::atBottom), NULL);
            ui.total_response_plot->axisRect(1)->setRangeZoomAxes(ui.total_response_plot->axisRect(1)->axis(QCPAxis::atBottom), NULL);



	}
	else {
			ui.plot->axisRect(0)->setRangeDragAxes(NULL, ui.plot->axisRect(0)->axis(QCPAxis::atLeft));
			ui.plot->axisRect(0)->setRangeZoomAxes(NULL, ui.plot->axisRect(0)->axis(QCPAxis::atLeft));
			ui.plot->axisRect(1)->setRangeDragAxes(NULL, ui.plot->axisRect(1)->axis(QCPAxis::atLeft));
			ui.plot->axisRect(1)->setRangeZoomAxes(NULL, ui.plot->axisRect(1)->axis(QCPAxis::atLeft));

			QList<QCPAxis*> empty;
			empty.append(NULL);
			QList<QCPAxis*> axes;
            axes.append(ui.local_response_plot->axisRect(0)->axis(QCPAxis::atLeft));
            axes.append(ui.local_response_plot->axisRect(0)->axis(QCPAxis::atRight));

            ui.local_response_plot->axisRect(0)->setRangeDragAxes(empty, axes);
            ui.local_response_plot->axisRect(0)->setRangeZoomAxes(empty, axes);

			axes.clear();
            axes.append(ui.local_response_plot->axisRect(1)->axis(QCPAxis::atLeft));
            axes.append(ui.local_response_plot->axisRect(1)->axis(QCPAxis::atRight));

            ui.local_response_plot->axisRect(1)->setRangeDragAxes(empty, axes);
            ui.local_response_plot->axisRect(1)->setRangeZoomAxes(empty, axes);

            /*----------------------------------------------------------------------*/

            axes.clear();
            axes.append(ui.total_response_plot->axisRect(0)->axis(QCPAxis::atLeft));
            axes.append(ui.total_response_plot->axisRect(0)->axis(QCPAxis::atRight));

            ui.total_response_plot->axisRect(0)->setRangeDragAxes(empty, axes);
            ui.total_response_plot->axisRect(0)->setRangeZoomAxes(empty, axes);

            axes.clear();
            axes.append(ui.total_response_plot->axisRect(1)->axis(QCPAxis::atLeft));
            axes.append(ui.total_response_plot->axisRect(1)->axis(QCPAxis::atRight));

            ui.total_response_plot->axisRect(1)->setRangeDragAxes(empty, axes);
            ui.total_response_plot->axisRect(1)->setRangeZoomAxes(empty, axes);
		}
	
}

void kHz_Stabilizer::new_filter(const QCPDataSelection& p) {
	
    for (int i = 0; i < 2; ++i)

    if (ui.plot->graph(i)->selected()) {

        int j;
        for (j = 0; j < filters[0].size(); ++j) {
            if (filters[i][j].ptr->data()->isEmpty())
				break;
		}

        if (j == filters[i].size())
			return;

        filters[i][j].ptr->setData(freqs, filtersim(freqs, p.dataRange().begin(), 50, 0));

        filters[i][j].N = 50;
        filters[i][j].graphdatapos = p.dataRange().begin();

        update_freq_label(50, filters[i][j].graphdatapos);

		ui.plot->replot();
	}


}

void kHz_Stabilizer::keyPressEvent(QKeyEvent* e)
{
	if (e->key() == Qt::Key_Up || e->key() == Qt::Key_Down || e->key() == Qt::Key_Left || e->key() == Qt::Key_Right) {

        for (int i = 0; i < 2; ++i){

        for (int j = 0; j < filters[i].size(); ++j) {
			
            if (filters[i][j].ptr->selected()) {

				if (e->key() == Qt::Key_Up) 
                    filters[i][j].ptr->setData(freqs, filtersim(freqs, filters[i][j].graphdatapos, filters[i][j].N += 1, 0));

				else if (e->key() == Qt::Key_Down)
                    filters[i][j].ptr->setData(freqs, filtersim(freqs, filters[i][j].graphdatapos, filters[i][j].N -= 1, 0));

				else if (e->key() == Qt::Key_Left) 
                    filters[i][j].ptr->setData(freqs, filtersim(freqs, filters[i][j].graphdatapos -= 1, filters[i][j].N, 0));

				else if (e->key() == Qt::Key_Right) 
                    filters[i][j].ptr->setData(freqs, filtersim(freqs, filters[i][j].graphdatapos += 1, filters[i][j].N, 0));

                update_freq_label(filters[i][j].N, filters[i][j].graphdatapos);
                }
            }
        }
	}


	ui.plot->replot();

}

void kHz_Stabilizer::remove_filter(QCPAbstractPlottable* p, int j, QMouseEvent* e) {

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

void kHz_Stabilizer::animate_plot(bool j) {

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

void kHz_Stabilizer::animate(int i) {

	if (i < 8) {
        ui.plot->graph(i)->setData(freqs, filtersim(freqs, filters[0][i - 2].graphdatapos, filters[0][i - 2].N, phi += 0.3));
		ui.plot->replot(QCustomPlot::RefreshPriority::rpQueuedReplot);
        update_freq_label(filters[0][i-2].N, filters[0][i-2].graphdatapos);

	}
	else {
        ui.plot->graph(i)->setData(freqs, filtersim(freqs, filters[1][i - 8].graphdatapos, filters[1][i - 8].N, phi += 0.3));
		ui.plot->replot(QCustomPlot::RefreshPriority::rpQueuedReplot);
        update_freq_label(filters[1][i - 8].N, filters[1][i - 8].graphdatapos);

	}
}

void kHz_Stabilizer::update_freq_label(int N, int graphdatapos) {

	ui.plotLabel->setText("Freq: " + QString::number(graphdatapos / (fft_window / 2.0) * 500) +
		" Hz | Window: " + QString::number(N) + " ms ");

}

void kHz_Stabilizer::update_filter_params() {

    for (int i = 0; i < 2; ++i){

        proc_thread.tones[i].clear();
        proc_thread.N[i].clear();


    for (int j = 0; j < filters[i].size(); ++j) {

        if (!filters[i][j].ptr->data()->isEmpty()) {
            proc_thread.tones[i].push_back(filters[i][j].graphdatapos / (fft_window / 2.0) * (sampling_freq / 2));
            proc_thread.N[i].push_back(filters[i][j].N);
		}
	}
}


}

void kHz_Stabilizer::on_cmdLineBox_returnPressed() {
	
	if (FC == nullptr) {
		ui.cmdLineBox->clear();
		return;
	}

	emit send_cmd_line_data(ui.cmdLineBox->text().split(','));

	ui.cmdLineBox->clear();

}

