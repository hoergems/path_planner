#ifndef __DYN_OPTIONS_HPP_
#define __DYN_OPTIONS_HPP_

namespace shared {

struct ManipulatorPathPlannerOptions {
	std::vector<std::vector<double>> goal_states;
	
	std::vector<double> ee_goal_position;
	
	std::vector<double> goal_radius;
	
	std::string control_sampler;
	
	bool addIntermediateStates;
	
	unsigned int numControlSamples;
	
	double RRTGoalBias;
	
	std::vector<int> min_max_control_durations;
	
};


}

#endif