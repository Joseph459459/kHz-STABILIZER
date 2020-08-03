#include "camview.h"

camview::camview(processing_thread* thread, QWidget* parent)
	: QDockWidget(parent), proc_thread(thread),table_8(0)
{

	ui.setupUi(this);

	ui.thresholdBox->setValue(0);
	for (double i = 0; i < 256; ++i) {
		table_8.append(qRgb(
			(int)std::round(std::clamp(-4.0 * std::abs(i - 255.0 * 3.0 / 4) + 255.0 * 3.0 / 2, 0.0, 255.0)),
			(int)std::round(std::clamp(-4.0 * std::abs(i - 255.0 * 2.0 / 4) + 255 * 3.0 / 2, 0.0, 255.0)),
			(int)std::round(std::clamp(-4.0 * std::abs(i - 255.0 * 1.0 / 4) + 255.0 * 3.0 / 2, 0.0, 255.0))));

	}
	qColored = new QImage(1, 1, QImage::Format_Indexed8);

	table_8[0] = qRgb(255, 255, 255);

	connect(proc_thread, &processing_thread::finished_analysis, this, &camview::finished_analysis);

	proc_thread->camera.Open();
	ui.exposureBox->setRange(proc_thread->camera.ExposureTimeAbs.GetMin(), proc_thread->camera.ExposureTimeAbs.GetMax());
	ui.exposureBox->setValue(proc_thread->camera.ExposureTimeAbs.GetValue());

	proc_thread->camera.AcquisitionFrameRateEnable.SetValue(true);

	this->setAttribute(Qt::WA_DeleteOnClose);
}

camview::~camview()
{
	proc_thread->acquiring = false;
	while (!proc_thread->isFinished()) {
		QCoreApplication::processEvents();
	};

	proc_thread->camera.Close();
}

void camview::updateimage(GrabResultPtr_t ptr) {

	int width = ptr->GetWidth();
	int height = ptr->GetHeight();
	float out[2] = { 0 };
	centroid(ptr, height, width, out, ui.thresholdBox->value());

	QImage q = QImage((unsigned char*)ptr->GetBuffer(), width, height, QImage::Format_Grayscale8);
	
	const int bpl = q.bytesPerLine();
	const int bplDst = qColored->bytesPerLine();
	for (int y = 0; y < height; ++y) {
		const uchar* row = q.bits() + y * bpl;
		uchar* rowDst = qColored->bits() + y * bplDst;
		std::copy(row, row + width, rowDst);

	}
	
	ui.imageLabel->setPixmap(QPixmap::fromImage(*qColored));
	ptr.Release();
}

void camview::on_findCentroidButton_clicked() {
	
	proc_thread->blockSignals(true);
	proc_thread->acquiring = false;

	while (!proc_thread->isFinished()) {
		QCoreApplication::processEvents();
	};

	GrabResultPtr_t ptr;

	proc_thread->camera.OffsetX.SetValue(0);
	proc_thread->camera.OffsetY.SetValue(0);

	proc_thread->camera.Height.SetValue(proc_thread->camera.Height.GetMax());
	proc_thread->camera.Width.SetValue(proc_thread->camera.Width.GetMax());
	int trycount = 0;
tryagain:
	if (!proc_thread->camera.GrabOne(300, ptr)) {
		emit write_to_log(QString("Timeout while searching for centroid."));

	}
	else {

		std::array<double, 6> out = allparams(ptr, ui.thresholdBox->value());

		int width = round(out[2]/4)*4 + 16;
		int height = round(out[3]/2)*2 + 16;

		if (std::isnan(out[2]) || std::isnan(out[3]) || std::isnan(out[0]) || std::isnan(out[1]) || 
			width > proc_thread->camera.WidthMax() || out[2] < 2 ||
			height > proc_thread->camera.HeightMax() || out[3] < 2 )
		{
			++trycount;
			if (trycount > 7) {
				updateimagesize(proc_thread->camera.WidthMax(), proc_thread->camera.HeightMax());
				emit write_to_log("Could not find Centroid. Make sure the sensor is clear and there is a threshold.");
				goto end;
			}

			goto tryagain;

		}

		proc_thread->camera.Height.SetValue(height);
		proc_thread->camera.Width.SetValue(width);

		updateimagesize(width, height);

		int xplus =  (((int)out[0] + width / 2)/2)*2;
		int xminus = (((int)out[0] - width / 2)/2)*2;
		int yplus = (((int)out[1] + height / 2)/2)*2;
		int yminus = (((int)out[1] - height / 2)/2)*2;

		if (xminus < 0)
			proc_thread->camera.OffsetX.SetValue(0);
		else if (xplus > proc_thread->camera.WidthMax.GetValue())
			proc_thread->camera.OffsetX.SetValue(proc_thread->camera.WidthMax.GetValue() - width);
		else
			proc_thread->camera.OffsetX.SetValue(xminus);

		if (yminus < 0)
			proc_thread->camera.OffsetY.SetValue(0);
		else if (yplus > proc_thread->camera.HeightMax.GetValue())
			proc_thread->camera.OffsetY.SetValue(proc_thread->camera.HeightMax.GetValue() - height);
		else
			proc_thread->camera.OffsetY.SetValue(yminus);

	end:
		proc_thread->blockSignals(false);
		proc_thread->start();
	}
}

void camview::updateimagesize(int width, int height) {

	delete qColored;
	qColored = new QImage(width, height, QImage::Format_Indexed8);
	qColored->setColorTable(table_8);
}

void camview::on_zoomInButton_clicked() {

	proc_thread->blockSignals(true);
	proc_thread->acquiring = false;

	while (!proc_thread->isFinished()) {
		QCoreApplication::processEvents();
	};

	int curr_width = proc_thread->camera.Width.GetValue();
	int curr_height = proc_thread->camera.Height.GetValue();


	if (curr_width - 4 > 4) {
		proc_thread->camera.Width.SetValue(curr_width - 4);
		proc_thread->camera.OffsetX.SetValue(proc_thread->camera.OffsetX.GetValue() + 2);
		updateimagesize(curr_width - 4, curr_height);

	}

	if (curr_height - 4 > 4) {
		proc_thread->camera.Height.SetValue(curr_height - 4);
		proc_thread->camera.OffsetY.SetValue(proc_thread->camera.OffsetY.GetValue() + 2);
		updateimagesize(proc_thread->camera.Width.GetValue(),curr_height - 4);
	}

	proc_thread->blockSignals(false);
	proc_thread->start();

}

void camview::on_zoomOutButton_clicked() {
	
	proc_thread->blockSignals(true);
	proc_thread->acquiring = false;
	while (!proc_thread->isFinished()) {
		QCoreApplication::processEvents();
	};

	int curr_width = proc_thread->camera.Width.GetValue();
	int curr_height = proc_thread->camera.Height.GetValue();

	if ((curr_width + 2 < proc_thread->camera.Width.GetMax()) && (proc_thread->camera.OffsetX.GetValue() - 2 > 0)) {
		proc_thread->camera.OffsetX.SetValue(proc_thread->camera.OffsetX.GetValue() - 2);
		proc_thread->camera.Width.SetValue(curr_width + 4);
		updateimagesize(curr_width + 4, curr_height);
	}
	
	if ((curr_height + 2 < proc_thread->camera.Height.GetMax()) && (proc_thread->camera.OffsetY.GetValue() - 2 > 0)) {
		proc_thread->camera.OffsetY.SetValue(proc_thread->camera.OffsetY.GetValue() - 2);
		proc_thread->camera.Height.SetValue(curr_height + 4);
		updateimagesize(proc_thread->camera.Width.GetValue(), curr_height + 4);
	}
	proc_thread->blockSignals(false);
	proc_thread->start();

}

void camview::on_resetButton_clicked() {

	proc_thread->blockSignals(true);
	proc_thread->acquiring = false;
	while (!proc_thread->isFinished()) {
		QCoreApplication::processEvents();
	};

	proc_thread->camera.OffsetX.SetValue(0);
	proc_thread->camera.OffsetY.SetValue(0);
	proc_thread->camera.Width.SetValue(proc_thread->camera.WidthMax());
	proc_thread->camera.Height.SetValue(proc_thread->camera.HeightMax());

	proc_thread->blockSignals(false);
	proc_thread->start();

}

void camview::finished_analysis()
{
	this->setDisabled(false);

	safe_thread_close();


	proc_thread->plan = STREAM;
	proc_thread->start();

}

void camview::on_noiseSpectrumButton_clicked() {

	this->safe_thread_close();

	progressbox = new QProgressDialog("Recording...", "Abort", 0, _window, this);
	progressbox->setAutoClose(true);
	progressbox->setAttribute(Qt::WA_DeleteOnClose);
	connect(proc_thread, &processing_thread::updateprogress, progressbox, &QProgressDialog::setValue);
	progressbox->show();
	progressbox->raise();

	proc_thread->plan = SPECTRUM;
	this->setDisabled(true);
	progressbox->setDisabled(false);
	proc_thread->start(QThread::TimeCriticalPriority);
}

void camview::on_actuatorRangeButton_clicked() {
	
	this->safe_thread_close();

	proc_thread->plan = FIND_RANGE;

	proc_thread->start();
}

void camview::on_transferFunctionButton_clicked() {

	this->safe_thread_close();

	progressbox = new QProgressDialog("Recording...", "Abort", 0, tf_window, this);
	progressbox->setAutoClose(true);
	progressbox->setAttribute(Qt::WA_DeleteOnClose);
	connect(proc_thread, &processing_thread::updateprogress, progressbox, &QProgressDialog::setValue);
	progressbox->show();
	progressbox->raise();
	
	proc_thread->plan = LEARN_TF;
	proc_thread->start(QThread::TimeCriticalPriority);
}

void camview::on_exposureBox_valueChanged(int i) {

	proc_thread->camera.ExposureTimeAbs.SetValue(i);
	proc_thread->adjust_framerate();
}

void camview::on_upButton_clicked() {
	if (proc_thread->camera.OffsetY() - 4 > 0 )
		proc_thread->camera.OffsetY.SetValue(proc_thread->camera.OffsetY() - 4);
}

void camview::on_downButton_clicked() {
	if (proc_thread->camera.OffsetY() + proc_thread->camera.Height() + 4 < proc_thread->camera.HeightMax())
		proc_thread->camera.OffsetY.SetValue(proc_thread->camera.OffsetY() + 4);
}

void camview::on_rightButton_clicked() {
	if (proc_thread->camera.OffsetX() + proc_thread->camera.Width() + 4 < proc_thread->camera.WidthMax())
		proc_thread->camera.OffsetX.SetValue(proc_thread->camera.OffsetX() + 4);

}

void camview::on_leftButton_clicked() {
	if (proc_thread->camera.OffsetX() - 4 > 0)
		proc_thread->camera.OffsetX.SetValue(proc_thread->camera.OffsetX() - 4);

}

void camview::on_triggerButton_toggled(bool j) {

	this->safe_thread_close();

	if (j) {
		proc_thread->camera.AcquisitionFrameRateEnable.SetValue(false);
		proc_thread->camera.TriggerMode.SetValue(TriggerMode_On);
		proc_thread->camera.TriggerSource.SetValue(TriggerSourceEnums::TriggerSource_Line1);
		proc_thread->camera.TriggerSelector.SetValue(TriggerSelector_FrameStart);
		proc_thread->camera.TriggerActivation.SetValue(TriggerActivation_RisingEdge);
	}
	else {
		proc_thread->camera.TriggerMode.SetValue(TriggerMode_Off);
		proc_thread->camera.AcquisitionFrameRateEnable.SetValue(true);
		proc_thread->adjust_framerate();
	}

	proc_thread->start();
}

void camview::safe_thread_close() {

	proc_thread->blockSignals(true);
	proc_thread->acquiring = false;

	while (!proc_thread->isFinished()) {
		QCoreApplication::processEvents();
	};

	proc_thread->blockSignals(false);

}