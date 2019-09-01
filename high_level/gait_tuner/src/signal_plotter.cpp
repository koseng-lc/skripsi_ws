#include "gait_tuner/signal_plotter.h"

#include <QtCharts/QAbstractAxis>
#include <QtCharts/QSplineSeries>
#include <QtCharts/QValueAxis>
#include <QGraphicsLayout>
#include <QtCharts/QCategoryAxis>

SignalPlotter::SignalPlotter(QGraphicsItem *parent, Qt::WindowFlags w_flags)
    : QChart(QChart::ChartTypeCartesian, parent, w_flags)
    , r_series_(0)
    , l_series_(0)
    , r_y_axis_(new QValueAxis)
    , l_y_axis_(new QValueAxis)
    , phase_axis_(new QValueAxis)
    , r_x_(0)
    , r_y_(0)
    , l_x_(0)
    , l_y_(0)
    , phase_(0)
    , last_phase_(0){

    this->layout()->setContentsMargins(0,0,0,0);
    this->setBackgroundRoundness(0);
    this->setTheme(QChart::ChartThemeDark);
    this->legend()->hide();

    phase_axis_->setTickCount(5);
    phase_axis_->setRange(-M_PI, M_PI);
    addAxis(phase_axis_, Qt::AlignBottom);

    r_y_axis_->setTickCount(5);
    r_y_axis_->setRange(-1.0, 1.0);
    addAxis(r_y_axis_, Qt::AlignLeft);

    r_series_ = new QSplineSeries(this);
    QPen blue_pen(Qt::blue);
    blue_pen.setWidth(3);
    r_series_->setPen(blue_pen);
//    r_series_->append(phase_,r_y_);
    addSeries(r_series_);

    r_series_->attachAxis(phase_axis_);
    r_series_->attachAxis(r_y_axis_);

    l_series_ = new QSplineSeries(this);
    QPen red_pen(Qt::red);
    red_pen.setWidth(3);
    l_series_->setPen(red_pen);
//    l_series_->append(phase_,l_y_);

    addSeries(l_series_);
    l_series_->attachAxis(phase_axis_);
    l_series_->attachAxis(r_y_axis_);

//    createDefaultAxes();
//    setAxisX(l_axis_, series_);
//    l_axis_->setTickCount(5);
//    axisX()->setRange(-M_PI, M_PI);
//    axisY()->setRange(-1.0, 1.0);

    /*QCategoryAxis* axis_x = new QCategoryAxis;
    QCategoryAxis* axis_y = new QCategoryAxis;

    axis_x->append(tr("-%1").arg(QString::fromUtf8("\u03c0")),-M_PI);
    axis_x->append(tr("0"), .0);
    axis_x->append(tr("%1").arg(QString::fromUtf8("\u03c0")),M_PI);
    axis_x->setRange(-M_PI, M_PI);

    axis_y->append(tr("-1.0"), -1.0);
    axis_y->append(tr("0"), .0);
    axis_y->append(tr("1.0"), 1.0);
    axis_y->setRange(-1.0, 1.0);

    addAxis(axis_x,Qt::AlignBottom);
    addAxis(axis_y,Qt::AlignLeft);

    series_->attachAxis(axis_x);
    series_->attachAxis(axis_y);*/

}

SignalPlotter::SignalPlotter(double _abs_max_val, QGraphicsItem *parent, Qt::WindowFlags w_flags)
    : QChart(QChart::ChartTypeCartesian, parent, w_flags)
    , r_series_(0)
    , l_series_(0)
    , r_y_axis_(new QValueAxis)
    , l_y_axis_(new QValueAxis)
    , phase_axis_(new QValueAxis)
    , r_x_(0)
    , r_y_(0)
    , l_x_(0)
    , l_y_(0)
    , phase_(0)
    , last_phase_(0){

    this->layout()->setContentsMargins(0,0,0,0);
    this->setBackgroundRoundness(0);
    this->setTheme(QChart::ChartThemeDark);
    this->legend()->hide();

    phase_axis_->setTickCount(5);
    phase_axis_->setRange(-M_PI, M_PI);
    addAxis(phase_axis_, Qt::AlignBottom);

    r_y_axis_->setTickCount(5);
    r_y_axis_->setRange(-_abs_max_val, _abs_max_val);
    addAxis(r_y_axis_, Qt::AlignLeft);

    r_series_ = new QSplineSeries(this);
    QPen blue_pen(Qt::blue);
    blue_pen.setWidth(3);
    r_series_->setPen(blue_pen);
//    r_series_->append(phase_,r_y_);
    addSeries(r_series_);

    r_series_->attachAxis(phase_axis_);
    r_series_->attachAxis(r_y_axis_);

    l_series_ = new QSplineSeries(this);
    QPen red_pen(Qt::red);
    red_pen.setWidth(3);
    l_series_->setPen(red_pen);
//    l_series_->append(phase_,l_y_);

    addSeries(l_series_);
    l_series_->attachAxis(phase_axis_);
    l_series_->attachAxis(r_y_axis_);

//    createDefaultAxes();
//    setAxisX(l_axis_, series_);
//    l_axis_->setTickCount(5);
//    axisX()->setRange(-M_PI, M_PI);
//    axisY()->setRange(-1.0, 1.0);

    /*QCategoryAxis* axis_x = new QCategoryAxis;
    QCategoryAxis* axis_y = new QCategoryAxis;

    axis_x->append(tr("-%1").arg(QString::fromUtf8("\u03c0")),-M_PI);
    axis_x->append(tr("0"), .0);
    axis_x->append(tr("%1").arg(QString::fromUtf8("\u03c0")),M_PI);
    axis_x->setRange(-M_PI, M_PI);

    axis_y->append(tr("-1.0"), -1.0);
    axis_y->append(tr("0"), .0);
    axis_y->append(tr("1.0"), 1.0);
    axis_y->setRange(-1.0, 1.0);

    addAxis(axis_x,Qt::AlignBottom);
    addAxis(axis_y,Qt::AlignLeft);

    series_->attachAxis(axis_x);
    series_->attachAxis(axis_y);*/

}

SignalPlotter::~SignalPlotter(){

}

void SignalPlotter::updateData(double _phase, double _r_y, double _l_y){
    phase_ = _phase;
    if(phase_ < .0 && last_phase_ >= .0){
//        qDebug() << "ABC";
        r_series_->clear();
        l_series_->clear();
    }
    r_series_->append(qreal(phase_), qreal(_r_y));
    l_series_->append(qreal(phase_), qreal(_l_y));
    last_phase_ = phase_;
//    scroll(_phase, 0);
}
