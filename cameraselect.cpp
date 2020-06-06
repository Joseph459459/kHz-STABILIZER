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
	CDeviceInfo info = devices[ui.cameraList->row(item)];


	if (CTlFactory::GetInstance().IsDeviceAccessible(info)) {
		FASTSTABILIZER* f = new FASTSTABILIZER(info, this);
		f->show();
	}
}