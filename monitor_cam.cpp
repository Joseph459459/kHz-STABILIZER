#include "monitor_cam.h"

monitor_cam::monitor_cam(processing_thread* thread, QWidget* parent)
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

	connect(proc_thread, &processing_thread::finished_analysis, this, &monitor_cam::finished_analysis);
	
	ui.exposureBox->setRange(proc_thread->monitor_cam.ExposureTimeAbs.GetMin(), proc_thread->monitor_cam.ExposureTimeAbs.GetMax());
	ui.exposureBox->setValue(proc_thread->monitor_cam.ExposureTimeAbs.GetValue());
	proc_thread->monitor_cam.AcquisitionFrameRateEnable.SetValue(true);
	updateimagesize(proc_thread->monitor_cam.Width.GetValue(), proc_thread->monitor_cam.Height.GetValue());

	this->setAttribute(Qt::WA_DeleteOnClose);
}

monitor_cam::~monitor_cam()
{
	proc_thread->acquiring = false;
	while (!proc_thread->isFinished()) {
		QCoreApplication::processEvents();
	};

	proc_thread->monitor_cam.Close();
}

void monitor_cam::updateimage(GrabResultPtr_t ptr) {

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

void monitor_cam::on_findCentroidButton_clicked() {
	
	proc_thread->blockSignals(true);
	proc_thread->acquiring = false;

	while (!proc_thread->isFinished()) {
		QCoreApplication::processEvents();
	};

	GrabResultPtr_t ptr;

	proc_thread->monitor_cam.OffsetX.SetValue(0);
	proc_thread->monitor_cam.OffsetY.SetValue(0);

	proc_thread->monitor_cam.Height.SetValue(proc_thread->monitor_cam.Height.GetMax());
	proc_thread->monitor_cam.Width.SetValue(proc_thread->monitor_cam.Width.GetMax());


    for (int trycount = 0; trycount < 7; trycount++){

        if (!proc_thread->monitor_cam.GrabOne(300, ptr)) {
            emit write_to_log(QString("Timeout while searching for centroid."));
        }
        else {

            std::array<double, 6> out = allparams(ptr, ui.thresholdBox->value());

            int width = round(out[2]/4)*4 + 16;
            int height = round(out[3]/2)*2 + 16;

            if (!std::isnan(out[2]) && !std::isnan(out[3]) && !std::isnan(out[0]) && !std::isnan(out[1]) &&
                width < proc_thread->monitor_cam.WidthMax() && out[2] > 2 &&
                height < proc_thread->monitor_cam.HeightMax() && out[3] > 2 )
            {

                proc_thread->monitor_cam.Height.SetValue(height);
                proc_thread->monitor_cam.Width.SetValue(width);

                updateimagesize(width, height);

                int xplus =  (((int)out[0] + width / 2)/2)*2;
                int xminus = (((int)out[0] - width / 2)/2)*2;
                int yplus = (((int)out[1] + height / 2)/2)*2;
                int yminus = (((int)out[1] - height / 2)/2)*2;

                if (xminus < 0)
                    proc_thread->monitor_cam.OffsetX.SetValue(0);
                else if (xplus > proc_thread->monitor_cam.WidthMax.GetValue())
                    proc_thread->monitor_cam.OffsetX.SetValue(proc_thread->monitor_cam.WidthMax.GetValue() - width);
                else
                    proc_thread->monitor_cam.OffsetX.SetValue(xminus);

                if (yminus < 0)
                    proc_thread->monitor_cam.OffsetY.SetValue(0);
                else if (yplus > proc_thread->monitor_cam.HeightMax.GetValue())
                    proc_thread->monitor_cam.OffsetY.SetValue(proc_thread->monitor_cam.HeightMax.GetValue() - height);
                else
                    proc_thread->monitor_cam.OffsetY.SetValue(yminus);

                proc_thread->blockSignals(false);
                proc_thread->start();
                return;
            }

        }
    }


    updateimagesize(proc_thread->monitor_cam.WidthMax(), proc_thread->monitor_cam.HeightMax());
    emit write_to_log("Could not find Centroid. Make sure the sensor is clear and there is a threshold.");
    proc_thread->blockSignals(false);
    proc_thread->start();

}

void monitor_cam::updateimagesize(int width, int height) {

	delete qColored;
	qColored = new QImage(width, height, QImage::Format_Indexed8);
	qColored->setColorTable(table_8);
}

void monitor_cam::on_zoomInButton_clicked() {

	proc_thread->blockSignals(true);
	proc_thread->acquiring = false;

	while (!proc_thread->isFinished()) {
		QCoreApplication::processEvents();
	};

	int curr_width = proc_thread->monitor_cam.Width.GetValue();
	int curr_height = proc_thread->monitor_cam.Height.GetValue();


	if (curr_width - 4 > 4) {
		proc_thread->monitor_cam.Width.SetValue(curr_width - 4);
		proc_thread->monitor_cam.OffsetX.SetValue(proc_thread->monitor_cam.OffsetX.GetValue() + 2);
		updateimagesize(curr_width - 4, curr_height);

	}

	if (curr_height - 4 > 4) {
		proc_thread->monitor_cam.Height.SetValue(curr_height - 4);
		proc_thread->monitor_cam.OffsetY.SetValue(proc_thread->monitor_cam.OffsetY.GetValue() + 2);
		updateimagesize(proc_thread->monitor_cam.Width.GetValue(),curr_height - 4);
	}

	proc_thread->blockSignals(false);
	proc_thread->start();

}

void monitor_cam::on_zoomOutButton_clicked() {
	
	proc_thread->blockSignals(true);
	proc_thread->acquiring = false;
	while (!proc_thread->isFinished()) {
		QCoreApplication::processEvents();
	};

	int curr_width = proc_thread->monitor_cam.Width.GetValue();
	int curr_height = proc_thread->monitor_cam.Height.GetValue();

	if ((curr_width + 2 < proc_thread->monitor_cam.Width.GetMax()) && (proc_thread->monitor_cam.OffsetX.GetValue() - 2 > 0)) {
		proc_thread->monitor_cam.OffsetX.SetValue(proc_thread->monitor_cam.OffsetX.GetValue() - 2);
		proc_thread->monitor_cam.Width.SetValue(curr_width + 4);
		updateimagesize(curr_width + 4, curr_height);
	}
	
	if ((curr_height + 2 < proc_thread->monitor_cam.Height.GetMax()) && (proc_thread->monitor_cam.OffsetY.GetValue() - 2 > 0)) {
		proc_thread->monitor_cam.OffsetY.SetValue(proc_thread->monitor_cam.OffsetY.GetValue() - 2);
		proc_thread->monitor_cam.Height.SetValue(curr_height + 4);
		updateimagesize(proc_thread->monitor_cam.Width.GetValue(), curr_height + 4);
	}
	proc_thread->blockSignals(false);
	proc_thread->start();

}

void monitor_cam::on_resetButton_clicked() {

	safe_thread_close();

	proc_thread->monitor_cam.OffsetX.SetValue(0);
	proc_thread->monitor_cam.OffsetY.SetValue(0);
	proc_thread->monitor_cam.Width.SetValue(proc_thread->monitor_cam.WidthMax());
	proc_thread->monitor_cam.Height.SetValue(proc_thread->monitor_cam.HeightMax());

	updateimagesize(proc_thread->monitor_cam.Width.GetValue(), proc_thread->monitor_cam.Height.GetValue());

	proc_thread->start();

}

void monitor_cam::finished_analysis()
{
	this->setDisabled(false);
}

void monitor_cam::on_exposureBox_valueChanged(int i) {

	proc_thread->monitor_cam.ExposureTimeAbs.SetValue(i);
	proc_thread->adjust_framerate();
}

void monitor_cam::on_upButton_clicked() {
	if (proc_thread->monitor_cam.OffsetY() - 4 > 0 )
		proc_thread->monitor_cam.OffsetY.SetValue(proc_thread->monitor_cam.OffsetY() - 4);
}

void monitor_cam::on_downButton_clicked() {
	if (proc_thread->monitor_cam.OffsetY() + proc_thread->monitor_cam.Height() + 4 < proc_thread->monitor_cam.HeightMax())
		proc_thread->monitor_cam.OffsetY.SetValue(proc_thread->monitor_cam.OffsetY() + 4);
}

void monitor_cam::on_rightButton_clicked() {
	if (proc_thread->monitor_cam.OffsetX() + proc_thread->monitor_cam.Width() + 4 < proc_thread->monitor_cam.WidthMax())
		proc_thread->monitor_cam.OffsetX.SetValue(proc_thread->monitor_cam.OffsetX() + 4);

}

void monitor_cam::on_leftButton_clicked() {
	if (proc_thread->monitor_cam.OffsetX() - 4 > 0)
		proc_thread->monitor_cam.OffsetX.SetValue(proc_thread->monitor_cam.OffsetX() - 4);

}

void monitor_cam::on_triggerButton_toggled(bool j) {

	safe_thread_close();

	if (j) {
		proc_thread->monitor_cam.AcquisitionFrameRateEnable.SetValue(false);
		proc_thread->monitor_cam.TriggerMode.SetValue(TriggerMode_On);
		proc_thread->monitor_cam.TriggerSource.SetValue(TriggerSourceEnums::TriggerSource_Line1);
		proc_thread->monitor_cam.TriggerSelector.SetValue(TriggerSelector_FrameStart);
		proc_thread->monitor_cam.TriggerActivation.SetValue(TriggerActivation_RisingEdge);
	}
	else {
		proc_thread->monitor_cam.TriggerMode.SetValue(TriggerMode_Off);
		proc_thread->monitor_cam.AcquisitionFrameRateEnable.SetValue(true);
		proc_thread->adjust_framerate();
	}

	proc_thread->start();
}

void monitor_cam::safe_thread_close() {

	proc_thread->blockSignals(true);
	proc_thread->acquiring = false;

	while (!proc_thread->isFinished()) {
		QCoreApplication::processEvents();
	};

	proc_thread->blockSignals(false);

}
