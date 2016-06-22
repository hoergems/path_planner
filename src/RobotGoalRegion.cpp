#include "include/RobotGoalRegion.hpp"

#include <fstream>
#include <string>

using std::cout;
using std::endl;

namespace shared {

    RobotGoalRegion::RobotGoalRegion(const ompl::base::SpaceInformationPtr &si,
    		                                     std::shared_ptr<shared::RobotEnvironment> &robot_environment,
                                                 std::vector<std::vector<double>> &goal_states):
        shared::GoalRegion(si, robot_environment, goal_states),
        goal_threshold_(0.0)
    {   
    	std::vector<double> goal_area;
    	robot_environment->getGoalArea(goal_area);
    	setThreshold(goal_area[3]);
    	goal_threshold_ = goal_area[3];
    	//setThreshold(ee_goal_threshold_);
    }
    
    double RobotGoalRegion::euclideanDistance(const std::vector<double> &vec1, const std::vector<double> &vec2) const{
        double sum = 0.0;
        for (size_t i = 0; i < vec1.size(); i++) {
            sum += pow(vec2[i] - vec1[i], 2);
        }
        
        return sqrt(sum);
    }

    double RobotGoalRegion::distanceGoal(const ompl::base::State *st) const
    {	
        std::vector<double> v1;
        double* v = st->as<ompl::base::RealVectorStateSpace::StateType>()->values;
        for (unsigned int i = 0; i < state_space_information_->getStateDimension(); i++) {
           v1.push_back(v[i]);          
        }
        
        return robot_environment_->getRobot()->distanceGoal(v1);
        //return distance;             
    } 
    
    double RobotGoalRegion::getThreshold() const {    	
    	return goal_threshold_;
    }

    void RobotGoalRegion::sampleGoal(ompl::base::State *st) const 
    {	
    	ompl::RNG rng;    	
        int rd = rng.uniformInt(0, goal_states_.size() - 1);   
        double* v = st->as<ompl::base::RealVectorStateSpace::StateType>()->values;
        for (unsigned int i = 0; i < state_space_information_->getStateDimension(); i++) {
           v[i] = goal_states_[rd][i];
        }           
    }
    
    void RobotGoalRegion::sampleGoalVec(std::vector<double> &goal_vec) const {               
        ompl::RNG rng;
        int rd = rng.uniformInt(0, goal_states_.size() - 1);
        //int rd = 0;
        if (goal_states_.size() == 0) {cout << "wtf"<<endl;}
        for (unsigned int i = 0; i < state_space_information_->getStateDimension(); i++) {
           goal_vec.push_back(goal_states_[rd][i]);
        }
    }

    unsigned int RobotGoalRegion::maxSampleCount() const
    {
        return goal_states_.size();
    }
    
    bool RobotGoalRegion::isSatisfied(const ompl::base::State *st) const {    	
    	std::vector<double> state_vec;    	
    	for (unsigned int i = 0; i < state_space_information_->getStateDimension(); i++) {
    		state_vec.push_back(st->as<ompl::base::RealVectorStateSpace::StateType>()->values[i]);
    	}
    	
    	return robot_environment_->getRobot()->isTerminal(state_vec);
    }
}

