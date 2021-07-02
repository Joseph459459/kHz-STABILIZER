#include "processing_thread.h"
//#include "nlopt.hpp"
#include "SDTFT_algo.h"
#include <armadillo>
#include <fstream>
#include <gsl/gsl_filter.h>
#include <gsl/gsl_math.h>
#include <gsl/gsl_vector.h>
#include <iostream>
#include <queue>

#define sq(x) ((x) * (x))

using namespace arma;

#define STABILIZE_DEBUG
//#undef STABILIZE_DEBUG

const char x_y[2] = {'x', 'y'};

using namespace std::chrono;

processing_thread::processing_thread(CDeviceInfo fb_info, QObject *parent)
    : QThread(parent), fb_cam(CTlFactory::GetInstance().CreateDevice(fb_info)),
      monitor_cam_enabled(false) {}

processing_thread::processing_thread(CDeviceInfo fb_info, CDeviceInfo m_info,
                                     QObject *parent)
    : QThread(parent), fb_cam(CTlFactory::GetInstance().CreateDevice(fb_info)),
      monitor_cam(CTlFactory::GetInstance().CreateDevice(m_info)),
      monitor_cam_enabled(true) {}

processing_thread::~processing_thread() {}

void processing_thread::adjust_framerate() {

  // This sets the acquisition frame rate to the maximum allowed with given
  // settings
  fb_cam.AcquisitionFrameRateAbs.SetValue(100000.0);

  if (fb_cam.ResultingFrameRateAbs() > 1000)
    fb_cam.AcquisitionFrameRateAbs.SetValue(1000);
  else
    fb_cam.AcquisitionFrameRateAbs.SetValue(fb_cam.ResultingFrameRateAbs());

  if (monitor_cam_enabled) {

    monitor_cam.AcquisitionFrameRateAbs.SetValue(100000.0);

    if (monitor_cam.ResultingFrameRateAbs() > 1000)
      monitor_cam.AcquisitionFrameRateAbs.SetValue(1000);
    else
      monitor_cam.AcquisitionFrameRateAbs.SetValue(
          monitor_cam.ResultingFrameRateAbs());
  }
}

void processing_thread::receive_large_serial_buffer(QSerialPort &teensy,
                                                    std::vector<int> &buffer,
                                                    int chunk_size) {

  const int tot_bytes = buffer.size() * sizeof(int);
  int tot_bytes_read = 0;
  char *curr_pos = (char *)buffer.data();

  while (tot_bytes_read < tot_bytes) {

    teensy.write(QByteArray(1, CONTINUE));
    teensy.waitForBytesWritten(10);

    if (teensy.waitForReadyRead(1000)) {

      int bytes_received = 0;

      while (bytes_received != chunk_size) {

        bytes_received = teensy.read((char *)curr_pos, chunk_size);

        curr_pos += bytes_received;

        tot_bytes_read += bytes_received;
      }
    }
  }
}

void processing_thread::test_loop_times() {

  emit write_to_log(QString("Beginning Loop Time Test..."));

  adjust_framerate();

  if (fb_cam.ResultingFrameRateAbs.GetValue() < 1000) {

    emit write_to_log(QString("Camera cannot acquire at 1 kHz (") +
                      QString::number(fb_cam.ResultingFrameRateAbs()) +
                      QString(" Hz)"));
    emit finished_analysis();
    return;
  }

  if (fb_cam.TriggerMode.GetValue() != TriggerMode_On) {
    emit write_to_log(
        QString("Camera must be hooked up to trigger source in order to time"));
    return;
  }

  float new_centroid[2];

  GrabResultPtr_t ptr;

  const int height = fb_cam.Height.GetValue();
  const int width = fb_cam.Width.GetValue();

  std::vector<int> computer_loop_times(5000);
  fb_cam.GevSCPSPacketSize = 1000;
  fb_cam.MaxNumBuffer = 5;
  fb_cam.StartGrabbing();

  QSerialPort teensy;

  open_port(teensy);

  teensy.write(QByteArray(1, TEST_LOOP_TIME));
  teensy.waitForBytesWritten(100);

  int missed = 0;

  for (int i = 0; i < 5000; ++i) {

    if (fb_cam.RetrieveResult(1000, ptr, Pylon::TimeoutHandling_Return)) {
      auto start = high_resolution_clock::now();

      centroid(ptr, height, width, new_centroid, threshold);

      teensy.write(QByteArray(1, SYNC_FLAG));
      teensy.write((const char *)&i, 4);
      teensy.waitForBytesWritten(10);

      computer_loop_times[i] =
          duration_cast<microseconds>(high_resolution_clock::now() - start)
              .count();

    } else
      ++missed;
    qDebug() << missed;
  }

  qDebug() << "done grabbing, missed" << missed;

  fb_cam.StopGrabbing();

  std::vector<int> loop_times(5000);
  loop_times.reserve(5200);
  std::vector<int> shots_sent(5000);
  shots_sent.reserve(5200);

  receive_large_serial_buffer(teensy, loop_times, 128);
  receive_large_serial_buffer(teensy, shots_sent, 128);

  teensy.close();

  qDebug() << "done reading serial...";

  time_t start = time(0);
  std::string date_time = ctime(&start);
  std::string log_file("loop_tests/loop_test_");
  log_file.append(date_time);
  log_file.erase(log_file.end() - 1);
  log_file.append(".txt");
  std::replace(log_file.begin(), log_file.end(), ' ', '_');

  std::ofstream loop_tests(log_file.c_str(), std::ofstream::out);

  loop_tests << missed << std::endl;
  for (int loop_time : loop_times)
    loop_tests << loop_time << std::endl;
  ;
  for (int computer_loop_time : computer_loop_times)
    loop_tests << computer_loop_time << std::endl;
  for (int shot_sent : shots_sent)
    loop_tests << shot_sent << std::endl;

  loop_tests.close();

  emit finished_analysis();
}

void processing_thread::stabilize() {

  QSerialPort teensy;

  open_port(teensy);

  adjust_framerate();

  emit write_to_log(QString("Beginning Stabilization..."));

  if (fb_cam.ResultingFrameRateAbs.GetValue() < 1000) {

    emit write_to_log(QString("Camera cannot acquire at 1 kHz (") +
                      QString::number(fb_cam.ResultingFrameRateAbs()) +
                      QString(" Hz)"));
    emit finished_analysis();
    return;
  }

  int next_commands[2];

  teensy.write(QByteArray(1, STABILIZE));
  teensy.waitForBytesWritten(100);

  if (teensy.waitForReadyRead(1000)) {
    teensy.clear();
    emit write_to_log("Actuator Initialized...");
  } else {
    teensy.clear();
    emit write_to_log(QString("Synchronization error: could not read"));
    teensy.close();
    emit finished_analysis();
    return;
  }

  std::vector<SDTFT_algo> axes = {
      SDTFT_algo(0), SDTFT_algo(fit_params[1], tones[1], N[1], max_DAC_val[1],
                                centroid_set_points[1])};

#ifdef STABILIZE_DEBUG

  const int height = fb_cam.Height();
  const int width = fb_cam.Width();
  GrabResultPtr_t ptr;
  try {
    fb_cam.GrabOne(100, ptr);

  } catch (GenICam::GenericException &e) {
    qDebug() << e.GetDescription();
  }
  float new_centroid[2];
  centroid(ptr, height, width, new_centroid, threshold);

  std::vector<double> noise(2000);
  for (int i = 26; i < 2026; ++i) {
    noise[i - 26] = 2 * cos(2 * PI * 110 / 1000 * i);
  }

  qDebug() << "beginning stabilizer debug";

  for (int i = 0; i < 2000; ++i) {

    if (i == 0)
      new_centroid[1] = noise[i] + new_centroid[1];
    else
      new_centroid[1] = noise[i] + axes[1].target[0];

    next_commands[1] = axes[1].next_DAC_cmd(new_centroid[1]);

    qDebug() << axes[1].DAC_cmds[0];
    qDebug() << axes[1].target[0];
    qDebug() << axes[1].noise_element;
    qDebug() << height - new_centroid[1];

    axes[1].prepare_next_cmd();
  }

#else

  GrabResultPtr_t ptr;
  const int height = fb_cam.Height();
  const int width = fb_cam.Width();
  float new_centroid[2];

  fb_cam.MaxNumBuffer = 5;
  fb_cam.StartGrabbing(GrabStrategy_LatestImageOnly);
  int k = 0;
  acquiring = true;

  while (acquiring) {

    if (fb_cam.RetrieveResult(10, ptr, Pylon::TimeoutHandling_Return)) {

      centroid(ptr, height, width, new_centroid, threshold);
      ptr.Release();

      next_commands[1] = axes[1].next_DAC_cmd(height - new_centroid[1]);

      teensy.write(QByteArray(1, SYNC_FLAG));
      teensy.write((const char *)&next_commands[1], 4);
      teensy.waitForBytesWritten(10);

      qDebug() << axes[1].DAC_cmds[0];
      qDebug() << axes[1].target[0];
      qDebug() << axes[1].noise_element;
      qDebug() << height - new_centroid[1];

      axes[1].prepare_next_cmd();

      ++k;
      if (k == 4950)
        acquiring = false;
    }
  }

  fb_cam.StopGrabbing();

#endif

  teensy.close();

  emit finished_analysis();
}

void processing_thread::receive_cmd_line_data(QStringList cmd_str_list) {

  bool numcheck = false;

  if (cmd_str_list.length() == 2) {

    for (int i = 0; i < 2; ++i) {
      int DAC_range = cmd_str_list[i].toInt(&numcheck);

      if (numcheck == false || DAC_range > 4095 || DAC_range < 0) {
        write_to_log(QString(
            "Could not parse DAC range, enter a number between 0 and 4095"));
        return;
      }

      max_DAC_val[i] = DAC_range;
    }

    write_to_log(QString("DAC range in x: ") + QString::number(max_DAC_val[0]));
    write_to_log(QString("DAC range in y: ") + QString::number(max_DAC_val[1]));
  }
}

void processing_thread::stream() {

  adjust_framerate();

  const int update_period = 20; // ms

  acquiring = true;

  if (monitor_cam_enabled) {

    fb_cam.MaxNumBuffer = 5;
    monitor_cam.MaxNumBuffer = 5;

    fb_cam.StartGrabbing();
    monitor_cam.StartGrabbing();

    GrabResultPtr_t fb_ptr;
    GrabResultPtr_t m_ptr;

    while (acquiring) {

      msleep(update_period);

      if (fb_cam.RetrieveResult(5000, fb_ptr, Pylon::TimeoutHandling_Return))
        emit send_feedback_ptr(fb_ptr);

      if (monitor_cam.RetrieveResult(5000, m_ptr,
                                     Pylon::TimeoutHandling_Return))
        emit send_monitor_ptr(m_ptr);
    }

    fb_cam.StopGrabbing();
    monitor_cam.StopGrabbing();
  }

  else {

    fb_cam.MaxNumBuffer = 5;
    fb_cam.StartGrabbing();

    while (acquiring) {

      msleep(update_period);

      GrabResultPtr_t fb_ptr;

      if (fb_cam.RetrieveResult(100, fb_ptr, Pylon::TimeoutHandling_Return)) {

        emit send_feedback_ptr(fb_ptr);
      }
    }

    fb_cam.StopGrabbing();
  }
}

void processing_thread::analyze_spectrum() {

  adjust_framerate();

  emit write_to_log(QString("Beginning Noise Profiling..."));

  if (fb_cam.ResultingFrameRateAbs.GetValue() < 1000) {

    emit write_to_log(QString("Camera cannot acquire at 1 kHz (") +
                      QString::number(fb_cam.ResultingFrameRateAbs()) +
                      QString(" Hz)"));
    emit finished_analysis();
    return;
  }

  centroids[0].resize(fft_window);
  centroids[1].resize(fft_window);
  centroids[0].fill(0);
  centroids[1].fill(0);

  std::vector<GrabResultPtr_t> ptrs(fft_window);

  int missed = 0;
  int i = 0;

  fb_cam.MaxNumBuffer.SetValue(fft_window);
  fb_cam.StartGrabbing();

  while (i < fft_window) {
    if (fb_cam.RetrieveResult(30, ptrs[i], Pylon::TimeoutHandling_Return)) {
      ++i;
    } else {
      ++missed;
    }

    if (i % (fft_window / 100) == 0) {
      emit updateprogress(i);
    }
  }

  fb_cam.StopGrabbing();

  emit updateprogress(fft_window);

  emit write_to_log(QString::number(i) + QString(" Shots recorded"));

  emit write_to_log(QString::number(missed) + QString(" Shots missed"));

  std::function<std::array<double, 2>(GrabResultPtr_t)> calc;

  calc = [thresh = threshold](GrabResultPtr_t pt) {
    return centroid<double>(pt, thresh);
  };

  /* FIND CENTROIDS CONCURRENTLY -----------------------*/

  QFuture<std::array<double, 2>> centroid_results =
      QtConcurrent::mapped(ptrs.begin(), ptrs.end(), calc);

  centroid_results.waitForFinished();

  double sum = 0;
  int nans = 0;

  for (int i = 0; i < 2; ++i) {

    for (int j = 0; j < fft_window; ++j) {
      if (std::isnan(centroid_results.resultAt(j)[i]))
        ++nans;

      centroids[i][j] = centroid_results.resultAt(j)[i];
      sum += centroids[i][j];
    }

    write_to_log(QString::number(nans) + QString(" NaN centroids detected"));

    /* FAST FOURIER TRANSFORM -----------------------------*/

    fft[i].resize(fft_window);

    fftw_plan plan = fftw_plan_r2r_1d(fft_window, centroids[i].data(),
                                      fft[i].data(), FFTW_R2HC, FFTW_ESTIMATE);

    fftw_execute(plan);

    fft[i][0] = 0;
    for (int j = 1; j < (fft_window + 1) / 2 - 1; ++j) {

      fft[i][j] = sqrt(fft[i][j] * fft[i][j] +
                       fft[i][fft_window - j] * fft[i][fft_window - j]);
    }

    /* NORMALIZE ------------------------------------------*/

    for (int j = 0; j < fft_window / 2; ++j) {
      fft[i][j] = fft[i][j] * 2 / fft_window;
    }

    fftw_destroy_plan(plan);
  }


  emit update_fft_plot(
      preciserms(centroids[0]), preciserms(centroids[1]),
      *std::max_element(centroids[0].constBegin(), centroids[0].constEnd()) -
          *std::min_element(centroids[0].constBegin(), centroids[0].constEnd()),
      *std::max_element(centroids[1].constBegin(), centroids[1].constEnd()) -
          *std::min_element(centroids[1].constBegin(),
                            centroids[1].constEnd()));

  emit finished_analysis();
}

/* To determine the system response in a noisy environment, it is best to drive
 * the system at frequencies that are far from the noise.
 * This functions smooths the noise spectrum and finds the minima in three
 * regions, which are the driving frequencies */

void processing_thread::open_port(QSerialPort &teensy) {

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

  teensy.setBaudRate(QSerialPort::Baud115200);
  teensy.setDataBits(QSerialPort::Data8);
  teensy.setParity(QSerialPort::NoParity);
  teensy.setStopBits(QSerialPort::OneStop);
  j = teensy.setFlowControl(QSerialPort::NoFlowControl);

  if (!j) {
    emit write_to_log(QString("Error configuring USB connection."));
    return;
  }
  emit write_to_log(QString("Port opened and configured"));
}

void processing_thread::find_actuator_range() {

  adjust_framerate();

  emit write_to_log(QString("Finding Actuator Range..."));

  QSerialPort teensy;

  open_port(teensy);

  teensy.write(QByteArray(1, FIND_RANGE));
  teensy.waitForBytesWritten(50);

  GrabResultPtr_t ptr;

  fb_cam.StartGrabbing(Pylon::GrabStrategy_UpcomingImage);

  long AOI[2] = {fb_cam.Width(), fb_cam.Height()};

  for (int i = 0; i < 2; ++i) {

    while (true) {

      if (fb_cam.RetrieveResult(30, ptr, Pylon::TimeoutHandling_Return)) {

        std::array<double, 6> out = allparams(ptr, threshold);

        if (!isnan(out[i + 2]) && out[i + 2] < AOI[i] &&
            (out[i] - out[i + 2] / 2) < 0) {
          teensy.write(QByteArray(1, SYNC_FLAG));
          teensy.write(QByteArray(1, STOP_FLAG));
          if (!teensy.waitForBytesWritten(1000)) {
            emit write_to_log(
                QString("Synchronization error: could not write"));
            break;
          }
        }

        else {
          teensy.write(QByteArray(1, SYNC_FLAG));
          teensy.write(QByteArray(1, CONTINUE));
          if (!teensy.waitForBytesWritten(1000)) {
            emit write_to_log(
                QString("Synchronization error: could not write"));
            break;
          }
        }

        if (teensy.waitForReadyRead(1000)) {
          if (*teensy.read(1) == STOP_FLAG) {
            max_DAC_val[i] = *((uint16_t *)teensy.read(2).data());
            emit write_to_log(QString("Success: the " + QString(x_y[i]) +
                                      " axis DAC uses " +
                                      QString::number(max_DAC_val[i] + 1) +
                                      " out of 4096 available units of range"));
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
  }

  fb_cam.StopGrabbing();

  teensy.close();

  emit finished_analysis();
}

void processing_thread::learn_system_response() {

  emit write_to_log(QString("Finding Pixel/DAC output system response..."));

  adjust_framerate();

  if (fb_cam.ResultingFrameRateAbs.GetValue() < 1000) {
    emit write_to_log(QString("Camera cannot acquire at 1 kHz") +
                      QString::number(fb_cam.ResultingFrameRateAbs()) +
                      QString(" Hz)"));
  } else {

    system_response_input.resize(sys_response_window + 1);
    centroids[0].resize(sys_response_window + 1);
    centroids[1].resize(sys_response_window + 1);
    centroids[0].fill(0);
    centroids[1].fill(0);

    generate_system_response_input(system_response_input.data(), max_DAC_val[1],
                                   20, 300);

    /* SEND PLANS TO THE TEENSY ------------------------------------------*/

    QSerialPort teensy;

    open_port(teensy);

    teensy.write(QByteArray(1, LEARN_SYS_RESPONSE));
    teensy.write((const char *)&max_DAC_val[1], 2);

    if (!teensy.waitForBytesWritten(100)) {
      emit write_to_log(QString("Synchronization error: could not write"));
      teensy.close();
      emit finished_analysis();
      return;
    }

    if (teensy.waitForReadyRead(1000)) {
      teensy.clear();
    } else {
      teensy.clear();
      emit write_to_log(QString("Synchronization error: could not read"));
      teensy.close();
      emit finished_analysis();
      return;
    }

    /*-------------------------------------------------------------------*/

    int missed = 0;
    const int height = fb_cam.Height();
    const int width = fb_cam.Width();
    float new_centroid[2];
    GrabResultPtr_t ptr;

    fb_cam.MaxNumBuffer.SetValue(sys_response_window/2);
    fb_cam.StartGrabbing();

    int i = 0;
    while (i < sys_response_window+1) {

      if (fb_cam.RetrieveResult(2, ptr, Pylon::TimeoutHandling_Return)) {

        centroid(ptr, height, width, new_centroid, threshold);

        centroids[0][i] = width - new_centroid[0];
        centroids[1][i] = height - new_centroid[1];

        teensy.write(QByteArray(1, SYNC_FLAG));
        if (!teensy.waitForBytesWritten(100)) {
          emit write_to_log(QString("Synchronization error: could not write"));
          break;
        }
        ++i;
      } else {
        ++missed;
      }
    }

    fb_cam.StopGrabbing();

    teensy.close();

    qDebug() << missed;

    std::ofstream SRDEBUG("/home/joseph/Stabilizer_Tests/SRDEBUG",
                          std::ofstream::out);
    system_response_input.pop_back();
    centroids[0].pop_front();
    centroids[1].pop_front();

    for (double DAC : system_response_input)
      SRDEBUG << DAC << endl;

    QVector<QVector<double>> to_plot;
    QVector<double> driven_fft[2];
    QVector<double> input_fft(system_response_input);

    fftw_plan input_plan =
        fftw_plan_r2r_1d(sys_response_window, system_response_input.data(),
                         input_fft.data(), FFTW_R2HC, FFTW_ESTIMATE);
    fftw_execute(input_plan);

    for (int i = 0; i < 2; ++i) {

      driven_fft[i].resize(sys_response_window);

      fftw_plan driven_plan =
          fftw_plan_r2r_1d(sys_response_window, centroids[i].data(),
                           driven_fft[i].data(), FFTW_R2HC, FFTW_ESTIMATE);
      fftw_execute(driven_plan);

      QVector<double> mag(sys_response_window/2);
      QVector<double> phase(sys_response_window/2);

      for (int j = 1; j < sys_response_window/2; ++j) {

        mag[j] = sqrt(driven_fft[i][j] * driven_fft[i][j] +
                      driven_fft[i][sys_response_window - j] *
                          driven_fft[i][sys_response_window - j]) /
                 sqrt(input_fft[j] * input_fft[j] +
                      input_fft[sys_response_window - j] *
                          input_fft[sys_response_window - j]);

        phase[j] =
            atan2(driven_fft[i][sys_response_window - j], driven_fft[i][j]) -
            atan2(input_fft[sys_response_window - j], input_fft[j]);
      }

      gsl_vector* filtered_mag = gsl_vector_alloc(mag.size());

      for (int j = 0; j < mag.size(); ++j)
        gsl_vector_set(filtered_mag,j,mag[j]);

      gsl_filter_median_workspace* med_filt = gsl_filter_median_alloc(mag.size());



      to_plot.push_back(mag);
      to_plot.push_back(phase);
    }


    SRDEBUG.close();

    emit update_sys_response_plot(to_plot);

    emit finished_analysis();

    /* -----------------------------------------------------------------*/
  }
}

void processing_thread::correlate_cameras() {

  emit write_to_log(
      QString("Finding Linear Feedback/Monitor Camera Correlation..."));

  adjust_framerate();

  if (fb_cam.ResultingFrameRateAbs.GetValue() < 1000) {
    emit write_to_log(QString("feedback camera cannot acquire at 1 kHz (") +
                      QString::number(fb_cam.ResultingFrameRateAbs()) +
                      QString(" Hz)"));
    return;
  } else if (monitor_cam.ResultingFrameRateAbs.GetValue() < 1000) {
    emit write_to_log(QString("monitor camera cannot acquire at 1 kHz (") +
                      QString::number(monitor_cam.ResultingFrameRateAbs()) +
                      QString(" Hz)"));
    return;
  } else if (fb_cam.TriggerMode.GetValue() !=
                 TriggerModeEnums::TriggerMode_On ||
             monitor_cam.TriggerMode.GetValue() !=
                 TriggerModeEnums::TriggerMode_On) {
    emit write_to_log(
        QString("Both cameras must be triggered for synchronous capture"));
    return;
  } else {

    // qDebug() << fb_cam.ReadoutTimeAbs.GetValue();

    // fb_cam.GevIEEE1588.SetValue(true);
    // monitor_cam.GevIEEE1588.SetValue(true);
    // qDebug() << fb_cam.GevIEEE1588Status.ToString();
    // qDebug() << monitor_cam.GevIEEE1588Status.ToString();
    //
    // for (int k = 0; k < 30; ++k) {
    //	msleep(500);
    //	qDebug() << "monitor cam: " << monitor_cam.GevIEEE1588Status.ToString();
    //	qDebug() << "feedback cam: " << fb_cam.GevIEEE1588Status.ToString();
    //}

    IPylonDevice *fb_device = fb_cam.DetachDevice();
    IPylonDevice *monitor_device = monitor_cam.DetachDevice();

    CInstantCameraArray arr(2);

    arr[0].Attach(fb_device);
    arr[1].Attach(monitor_device);

    arr.StartGrabbing();

    CGrabResultPtr img_ptr;

    for (int i = 0; i < 30; ++i) {
      arr.RetrieveResult(300, img_ptr);
      qDebug() << img_ptr->GetCameraContext();
    }

    arr.StopGrabbing();

    fb_cam.Attach(arr[0].DetachDevice());
    monitor_cam.Attach(arr[1].DetachDevice());
  }
}

void processing_thread::run() {

  switch (run_plan) {
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
  case LEARN_SYS_RESPONSE:
    learn_system_response();
    break;
  case CORRELATE:
    correlate_cameras();
    break;
  case TEST_LOOP_TIME:
    test_loop_times();
    break;
  default:
    break;
  }
}
