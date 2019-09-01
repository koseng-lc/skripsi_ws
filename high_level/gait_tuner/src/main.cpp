#include "gait_tuner/mainwindow.h"

#include <QApplication>
#include <QSurfaceFormat>

#include <csignal>

void sigHandler(int sig){
    (void)sig;
    QApplication::quit();
}

int main(int argc, char *argv[]){
    ros::init(argc, argv, "gait_tuner_node", ros::InitOption::NoSigintHandler);

    signal(SIGINT, sigHandler);

    QSurfaceFormat fmt;
    fmt.setVersion(3,3);
    fmt.setProfile(QSurfaceFormat::CoreProfile);
    QSurfaceFormat::setDefaultFormat(fmt);

    QApplication a(argc, argv);
    MainWindow w;
    w.show();

    return a.exec();
}
