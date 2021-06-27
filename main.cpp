#include "camera_select.h"
#include <QtWidgets/QApplication>
#include "kHz_macros.h"
#include <QDebug>
#include <pylon/PylonIncludes.h>

int main(int argc, char *argv[])
{
    PylonInitialize();
	QApplication a(argc, argv);
	camera_select w;
	w.show();

	return a.exec();
}
