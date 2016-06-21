#ifndef __PATH_HPP__
#define __PATH_HPP__

namespace shared {

struct Trajectory {
	std::vector<std::vector<double>> xs;	
	std::vector<std::vector<double>> us;
	std::vector<std::vector<double>> zs;
	
	std::vector<double> control_durations;
	
};

}

#endif