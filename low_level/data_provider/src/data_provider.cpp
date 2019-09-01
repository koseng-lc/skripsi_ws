#include "data_provider/data_provider.h"

DataProvider::DataProvider(){
    init();
}

DataProvider::~DataProvider(){

}

void DataProvider::init(){

}

void DataProvider::routine(){
    ros::Rate loop_rate(60);
    while(ros::ok()){
        ros::spinOnce();
        loop_rate.sleep();
    }
}
