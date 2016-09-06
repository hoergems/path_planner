#ifndef __PATH_HPP__
#define __PATH_HPP__
#include <frapu_core/core.hpp>

namespace frapu
{

class Trajectory
{
public: 
    Trajectory() {
	
    }
    
    std::vector<RobotStateSharedPtr> stateTrajectory;
    
    std::vector<ActionSharedPtr> actionTrajectory;
    
    std::vector<ObservationSharedPtr> observationTrajectory;
    
    std::vector<double> durations;
    
    unsigned int size() const {
	return stateTrajectory.size();
    }

};


struct VectorTrajectory {
    VectorTrajectory() {}
    
    std::vector<std::vector<double>> states;
    
    std::vector<std::vector<double>> controls;
    
    std::vector<std::vector<double>> observations;
    
    std::vector<double> durations;
};

}

#endif
