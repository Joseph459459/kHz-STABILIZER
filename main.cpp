#include "cameraselect.h"
#include <QtWidgets/QApplication>
#include "FS_macros.h"
#include <QDebug>

int main(int argc, char *argv[])
{
	QApplication a(argc, argv);
	cameraselect w;
	w.show();

	std::vector<double> p(6000);
	std::fill(p.begin(), p.end(), 0);

	std::vector<uint16_t> q(6000);
	std::fill(q.begin(), q.end(), 0);

	tf_input(p.data(), 0);
	tf_input(q.data(), 0);

	auto g = std::max_element(q.begin(), q.end());
	qDebug() << *g;

	return a.exec();
}
