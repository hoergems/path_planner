#ifndef __DYN_OPTIONS_HPP_
#define __DYN_OPTIONS_HPP_

namespace shared {

class PathPlannerOptions {
public:
	std::string planning_algorithm;
	
	std::vector<std::vector<double>> goal_states;
	
	double goal_radius;
		
	std::string control_sampler;
		
	bool addIntermediateStates;
		
	unsigned int numControlSamples;
		
	double RRTGoalBias;
		
	std::vector<int> min_max_control_durations;
	
};



class ManipulatorPathPlannerOptions: public PathPlannerOptions {
public:
	ManipulatorPathPlannerOptions():
		shared::PathPlannerOptions()
	{
		
	}
	
	std::vector<double> ee_goal_position;
	
	
	
};


}

#endif