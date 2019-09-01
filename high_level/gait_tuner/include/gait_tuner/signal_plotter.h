/**
 * @author koseng : lintangerlangga@gmail.com
 */

#pragma once

#include <QtCharts/QChart>
#include <QTimer>
#include <QDebug>

QT_CHARTS_BEGIN_NAMESPACE
class QSplineSeries;
class QValueAxis;
QT_CHARTS_END_NAMESPACE

QT_CHARTS_USE_NAMESPACE

class SignalPlotter:public QChart{
    Q_OBJECT
public:
    SignalPlotter(QGraphicsItem* parent = 0,
                  Qt::WindowFlags w_flags = 0);
    SignalPlotter(double _abs_max_val = 1.0, QGraphicsItem* parent = 0,
                  Qt::WindowFlags w_flags = 0);
    virtual ~SignalPlotter();
public slots:
    void updateData(double _phase, double _r_y, double _l_y);
private:
    QSplineSeries* r_series_;
    QSplineSeries* l_series_;
    QStringList titles_;
    QValueAxis* r_x_axis_;
    QValueAxis* r_y_axis_;
    QValueAxis* l_x_axis_;
    QValueAxis* l_y_axis_;
    QValueAxis* phase_axis_;
    qreal r_x_;
    qreal r_y_;
    qreal l_x_;
    qreal l_y_;
    qreal phase_;
    qreal last_phase_;
};
