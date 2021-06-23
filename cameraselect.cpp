#include "cameraselect.h"
#include "FASTSTABILIZER.h"

cameraselect::cameraselect(QWidget *parent)
	: QWidget(parent)
{

    system("echo Moving program to core 2 of the machine...");
    system("echo");
    system("taskset -cp 2 $(pidof kHz_Stabilizer)");
    system("echo DONE");
    system("echo Increasing maximum packet size...");
    system("echo");
    system("sudo ifconfig enp0s31f6 mtu 8000");
    system("echo DONE");
    system("echo Increasing process priorities...");
    system("echo");
    system("sudo renice -n -20 -g $(pidof kHz_Stabilizer)");
    system("echo DONE");
    system("echo Removing interrupts from core 2...");
    system("echo");
    system("irqbalance --foreground --oneshot");
    system("echo DONE");

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
