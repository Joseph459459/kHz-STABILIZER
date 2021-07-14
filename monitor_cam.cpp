#include "monitor_cam.h"

monitor_cam::monitor_cam(processing_thread *thread, QWidget *parent)
    : QDockWidget(parent), proc_thread(thread), table_8(0) {

  ui.setupUi(this);

  ui.thresholdBox->setValue(0);
  for (double i = 0; i < 256; ++i) {
    table_8.append(qRgb(
        (int)std::round(
            std::clamp(-4.0 * std::abs(i - 255.0 * 3.0 / 4) + 255.0 * 3.0 / 2,
                       0.0, 255.0)),
        (int)std::round(std::clamp(
            -4.0 * std::abs(i - 255.0 * 2.0 / 4) + 255 * 3.0 / 2, 0.0, 255.0)),
        (int)std::round(
            std::clamp(-4.0 * std::abs(i - 255.0 * 1.0 / 4) + 255.0 * 3.0 / 2,
                       0.0, 255.0))));
  }
  qColored = new QImage(1, 1, QImage::Format_Indexed8);

  table_8[0] = qRgb(255, 255, 255);

  connect(proc_thread, &processing_thread::finished_analysis, this,
          &monitor_cam::finished_analysis);

  m_cam = &proc_thread->monitor_cam;

  ui.exposureBox->setRange(m_cam->ExposureTime.GetMin(),
                           m_cam->ExposureTime.GetMax());
  ui.exposureBox->setValue(m_cam->ExposureTime.GetValue());
  m_cam->AcquisitionFrameRateEnable.SetValue(true);
  updateimagesize(m_cam->Width.GetValue(), m_cam->Height.GetValue());

  height_inc = m_cam->Height.GetInc();
  width_inc = m_cam->Width.GetInc();
  max_height = m_cam->Height.GetMax();
  max_width = m_cam->Width.GetMax();
  min_height = m_cam->Height.GetMin();
  min_width = m_cam->Width.GetMin();
  min_offset_x = m_cam->OffsetX.GetMin();
  min_offset_y = m_cam->OffsetY.GetMin();
  offset_inc_x = m_cam->OffsetX.GetInc();
  offset_inc_y = m_cam->OffsetY.GetInc();

  this->setAttribute(Qt::WA_DeleteOnClose);
}

monitor_cam::~monitor_cam() {
  proc_thread->acquiring = false;
  while (!proc_thread->isFinished()) {
    QCoreApplication::processEvents();
  };

  m_cam->Close();
}

void monitor_cam::updateimage(GrabResultPtr_t ptr) {

  int width = ptr->GetWidth();
  int height = ptr->GetHeight();
  float out[2] = {0};
  centroid(ptr, height, width, out, ui.thresholdBox->value());

  QImage q = QImage((unsigned char *)ptr->GetBuffer(), width, height,
                    QImage::Format_Grayscale8);

  const int bpl = q.bytesPerLine();
  const int bplDst = qColored->bytesPerLine();
  for (int y = 0; y < height; ++y) {
    const uchar *row = q.bits() + y * bpl;
    uchar *rowDst = qColored->bits() + y * bplDst;
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

  m_cam->OffsetX.SetValue(min_offset_x);
  m_cam->OffsetY.SetValue(min_offset_y);

  m_cam->Height.SetValue(max_height);
  m_cam->Width.SetValue(max_width);

  for (int trycount = 0; trycount < 7; trycount++) {

    if (!m_cam->GrabOne(300, ptr)) {
      emit write_to_log(QString("Timeout while searching for centroid."));
    } else {

      std::array<double, 6> out = allparams(ptr, ui.thresholdBox->value());

      int width = round(out[2] / width_inc) * width_inc + 16;
      int height = round(out[3] / width_inc) * width_inc + 16;

      if (!std::isnan(out[2]) && !std::isnan(out[3]) && !std::isnan(out[0]) &&
          !std::isnan(out[1]) && width < max_width && out[2] > 2 &&
          height < max_height && out[3] > 2) {

        m_cam->Height.SetValue(height);
        m_cam->Width.SetValue(width);

        updateimagesize(width, height);

        int x_plus = (((int)out[0] + width / 2) / offset_inc_x) * offset_inc_x;
        int x_minus = (((int)out[0] - width / 2) / offset_inc_x) * offset_inc_x;

        int y_plus = (((int)out[1] + height / 2) / offset_inc_y) * offset_inc_y;
        int y_minus =
            (((int)out[1] - height / 2) / offset_inc_y) * offset_inc_y;

        if (x_minus < 0)
          m_cam->OffsetX.SetValue(min_offset_x);
        else if (x_plus > max_width)
          m_cam->OffsetX.SetValue(max_width - width);
        else
          m_cam->OffsetX.SetValue(x_minus);

        if (y_minus < 0)
          m_cam->OffsetY.SetValue(min_offset_y);
        else if (y_plus > max_height)
          m_cam->OffsetY.SetValue(max_height - height);
        else
          m_cam->OffsetY.SetValue(y_minus);

        proc_thread->blockSignals(false);
        proc_thread->start();
        return;
      }
    }
  }

  updateimagesize(max_width, max_height);
  emit write_to_log("Could not find Centroid. Make sure the sensor is clear "
                    "and there is a threshold.");
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

  int curr_width = m_cam->Width.GetValue();
  int curr_height = m_cam->Height.GetValue();

  if (curr_width - std::max(width_inc, offset_inc_x) > min_width) {
    m_cam->Width.SetValue(curr_width - std::max(width_inc, offset_inc_x));
    m_cam->OffsetX.SetValue(m_cam->OffsetX.GetValue() +
                            std::max(width_inc, offset_inc_x));
    updateimagesize(curr_width - std::max(width_inc, offset_inc_x), curr_width);
  }

  if (curr_height - std::max(height_inc, offset_inc_y) > min_height) {
    m_cam->Height.SetValue(curr_height - std::max(height_inc, offset_inc_y));
    m_cam->OffsetY.SetValue(m_cam->OffsetY.GetValue() +
                            std::max(height_inc, offset_inc_y));
    updateimagesize(m_cam->Width.GetValue(),
                    curr_height - std::max(height_inc, offset_inc_y));
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

  int curr_width = m_cam->Width.GetValue();
  int curr_height = m_cam->Height.GetValue();

  if ((curr_width + width_inc < max_width) &&
      (m_cam->OffsetX.GetValue() - offset_inc_x > min_offset_x)) {
    m_cam->OffsetX.SetValue(m_cam->OffsetX.GetValue() - offset_inc_x);
    m_cam->Width.SetValue(curr_width + width_inc);
    updateimagesize(curr_width + width_inc, curr_height);
  }

  if ((curr_height + height_inc < max_height) &&
      (m_cam->OffsetY.GetValue() - offset_inc_y > 0)) {
    m_cam->OffsetY.SetValue(m_cam->OffsetY.GetValue() - offset_inc_y);
    m_cam->Height.SetValue(curr_height + height_inc);
    updateimagesize(m_cam->Width.GetValue(), curr_height + height_inc);
  }
  proc_thread->blockSignals(false);
  proc_thread->start();
}

void monitor_cam::on_resetButton_clicked() {

  safe_thread_close();

  m_cam->OffsetX.SetValue(min_offset_x);
  m_cam->OffsetY.SetValue(min_offset_y);
  m_cam->Width.SetValue(max_width);
  m_cam->Height.SetValue(max_height);

  updateimagesize(max_width, max_height);

  proc_thread->start();
}

void monitor_cam::finished_analysis() { this->setDisabled(false); }

void monitor_cam::on_exposureBox_valueChanged(int i) {

  m_cam->ExposureTime.SetValue(i);
  proc_thread->adjust_framerate();
}

void monitor_cam::on_upButton_clicked() {
  if (m_cam->OffsetY() - offset_inc_y > min_offset_y)
    m_cam->OffsetY.SetValue(m_cam->OffsetY() - offset_inc_y);
}

void monitor_cam::on_downButton_clicked() {
  if (m_cam->OffsetY() + m_cam->Height() + offset_inc_y < max_height)
    m_cam->OffsetY.SetValue(m_cam->OffsetY() + offset_inc_y);
}

void monitor_cam::on_rightButton_clicked() {
  if (m_cam->OffsetX() + m_cam->Width() + offset_inc_x < max_width)
    m_cam->OffsetX.SetValue(m_cam->OffsetX() + offset_inc_x);
}

void monitor_cam::on_leftButton_clicked() {
  if (m_cam->OffsetX() - offset_inc_x > min_offset_x)
    m_cam->OffsetX.SetValue(m_cam->OffsetX() - offset_inc_x);
}

void monitor_cam::on_triggerButton_toggled(bool j) {

  safe_thread_close();

  if (j) {
    m_cam->AcquisitionFrameRateEnable.SetValue(false);
    m_cam->TriggerMode.SetValue(TriggerMode_On);
    m_cam->TriggerSource.SetValue(TriggerSourceEnums::TriggerSource_Line1);
    m_cam->TriggerSelector.SetValue(TriggerSelector_FrameStart);
    m_cam->TriggerActivation.SetValue(TriggerActivation_RisingEdge);
  } else {
    m_cam->TriggerMode.SetValue(TriggerMode_Off);
    m_cam->AcquisitionFrameRateEnable.SetValue(true);
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
