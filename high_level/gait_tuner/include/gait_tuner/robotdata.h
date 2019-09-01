/**
 * @author koseng : lintangerlangga@gmail.com
 * @brief Not used anymore
 */

#pragma once

#include <string>
#include <vector>
#include <cassert>

#include "joint_data/joint_data.h"

class RobotData{
private:
    static RobotData* instance;
    RobotData();
    class Params{
    public:
        std::string name;
        int ID;
        int child;
        int sister;
        Params():name("unnamed"),ID(-1),child(-1),sister(-1){

        }
    };
    std::vector<Params > data;

public:
    static RobotData* getInstance(){
        if(instance == nullptr)
            instance = new RobotData;
        return instance;
    }

    std::string getName(int _joint_id) const{
        assert(_joint_id >= 0 && _joint_id < JointData::NUM_OF_JOINTS);
        return data[_joint_id].name;
    }

    int getChild(int _joint_id){
        assert(_joint_id >= 0 && _joint_id < JointData::NUM_OF_JOINTS);
        return data[_joint_id].child;
    }

    int getSister(int _joint_id){
        assert(_joint_id >= 0 && _joint_id < JointData::NUM_OF_JOINTS);
        return data[_joint_id].sister;
    }

};

