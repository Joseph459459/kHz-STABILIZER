#pragma once

#include "ui_camera_select.h"
#include <pylon/PylonIncludes.h>
#include <pylon/BaslerUniversalInstantCamera.h>
#include <qlistwidget.h>

typedef Pylon::CBaslerUniversalInstantCamera Camera_t;
typedef Pylon::CBaslerUniversalGrabResultPtr GrabResultPtr_t;

using namespace Basler_UniversalCameraParams;
using namespace Pylon;

#include <QWidget>
#include "ui_camera_select.h"

class camera_select : public QWidget
{
	Q_OBJECT

public:
	DeviceInfoList_t devices;
	CDeviceInfo fb_info;
	bool selecting_monitor_cam = false;
	camera_select(QWidget *parent = Q_NULLPTR);
	~camera_select();

public slots:
	void on_refreshButton_clicked();
	void on_cameraList_itemDoubleClicked(QListWidgetItem* item);

private:
    Ui::camera_selectClass ui;
};
