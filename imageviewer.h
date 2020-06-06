#pragma once

#include <QWidget>
#include <QtGui>

class imageviewer : public QWidget
{
	Q_OBJECT

	QPixmap m_pixmap;
	QRectF m_rect;
	QPointF m_reference;
	QPointF m_delta;
	qreal m_scale = 1.0;
	void paintEvent(QPaintEvent*) override {
		QPainter p{ this };
		p.translate(rect().center());
		p.scale(m_scale, m_scale);
		p.translate(m_delta);
		p.drawPixmap(m_rect.topLeft(), m_pixmap);
	}

	void mousePressEvent(QMouseEvent* event) override {
		m_reference = event->pos();
		qApp->setOverrideCursor(Qt::ClosedHandCursor);
		setMouseTracking(true);
	}
	void mouseMoveEvent(QMouseEvent* event) override {
		m_delta += (event->pos() - m_reference) * 1.0 / m_scale;
		m_reference = event->pos();
		update();
	}
	void mouseReleaseEvent(QMouseEvent*) override {
		qApp->restoreOverrideCursor();
		setMouseTracking(false);
	}

	void wheelEvent(QWheelEvent* event) override {
		if (event->delta() > 0)
			scale(1.1);
		else
			scale(1.0 / 1.1);

	}
public:
	void setPixmap(const QPixmap& pix) {
		m_pixmap = pix;
		m_rect = m_pixmap.rect();
		m_rect.translate(-m_rect.center());
		update();
	}
	void scale(qreal s) {
		m_scale *= s;
		update();
	}

public:
	imageviewer(QWidget* parent = nullptr) {};
	~imageviewer() {};
};
