#include "cameraselect.h"
#include <QtWidgets/QApplication>

int main(int argc, char *argv[])
{
	QApplication a(argc, argv);
	cameraselect w;
	w.show();
	return a.exec();
}
