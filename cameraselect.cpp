#include "cameraselect.h"
#include "FASTSTABILIZER.h"

cameraselect::cameraselect(QWidget *parent)
	: QWidget(parent)
{
	PylonInitialize();
	ui.setupUi(this);
}

cameraselect::~cameraselect()
{

	PylonTerminate();


}

void cameraselect::on_refreshButton_clicked() {


		ui.cameraList->clear();
		CTlFactory& tlFactory = CTlFactory::GetInstance();
		if (tlFactory.EnumerateDevices(devices) == 0)
		{
			return;
		}

		for (int i = 0; i < devices.size(); i++) {
			ui.cameraList->addItem(devices[i].GetFriendlyName().c_str());
		}
}

void cameraselect::on_cameraList_itemDoubleClicked(QListWidgetItem* item) {

	if (selecting_monitor_cam == true) {

		CDeviceInfo m_info = devices[ui.cameraList->row(item)];
		
		if (strcmp(m_info.GetSerialNumber(), fb_info.GetSerialNumber()) != 0
			&& CTlFactory::GetInstance().IsDeviceAccessible(fb_info) 
			&& CTlFactory::GetInstance().IsDeviceAccessible(m_info)) {
			
			FASTSTABILIZER* f = new FASTSTABILIZER(fb_info,m_info, this);
			f->show();
		}

		selecting_monitor_cam = false;
		return;
	}

	QMessageBox cam_option;
	cam_option.setText("Monitor Camera?");
	cam_option.setStandardButtons(QMessageBox::Yes | QMessageBox::No | QMessageBox::Cancel);
	cam_option.setDefaultButton(QMessageBox::Yes);
	int ret = cam_option.exec();

	switch (ret) {
		case QMessageBox::Yes:
			selecting_monitor_cam = true;
			fb_info = devices[ui.cameraList->row(item)];
			return;
		case QMessageBox::No:
			fb_info = devices[ui.cameraList->row(item)];
			if (CTlFactory::GetInstance().IsDeviceAccessible(fb_info)) {
				FASTSTABILIZER* f = new FASTSTABILIZER(fb_info, this);
				f->show();
			}
		case QMessageBox::Cancel:
			selecting_monitor_cam = false;
			return;
		default:
			//should never be run
			return;

	}


}