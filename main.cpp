#include "cameraselect.h"
#include <QtWidgets/QApplication>
#include "FS_macros.h"
#include <QDebug>

int main(int argc, char *argv[])
{
	QApplication a(argc, argv);
	cameraselect w;
	w.show();

	return a.exec();
}
