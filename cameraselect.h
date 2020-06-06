#pragma once

#include <pylon/PylonIncludes.h>
#include <pylon/gige/BaslerGigECamera.h>
#include <pylon/gige/BaslerGigEInstantCamera.h>
#include <qlistwidget.h>

typedef Pylon::CBaslerGigEInstantCamera Camera_t;
typedef Pylon::CBaslerGigEImageEventHandler ImageEventHandler_t;
typedef Pylon::CBaslerGigEGrabResultPtr GrabResultPtr_t;

using namespace Basler_GigECameraParams;
using namespace Pylon;

#include <QWidget>
#include "ui_cameraselect.h"

class cameraselect : public QWidget
{
	Q_OBJECT

public:
	DeviceInfoList_t devices;
	cameraselect(QWidget *parent = Q_NULLPTR);
	~cameraselect();

public slots:
	void on_refreshButton_clicked();
	void on_cameraList_itemDoubleClicked(QListWidgetItem* item);

private:
	Ui::cameraselect ui;
};
