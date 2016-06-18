#include "include/ManipulatorGoalRegion.hpp"

#include <fstream>
#include <string>

using std::cout;
using std::endl;

namespace shared {

    ManipulatorGoalRegion::ManipulatorGoalRegion(const ompl::base::SpaceInformationPtr &si,
    		                                     std::shared_ptr<shared::RobotEnvironment> &robot_environment,
                                                 std::vector<std::vector<double>> &goal_states,
                                                 std::vector<double> &ee_goal_position,
                                                 double &ee_goal_threshold):
        shared::GoalRegion(si, robot_environment, goal_states),
        ee_goal_position_(ee_goal_position),
        ee_goal_threshold_(ee_goal_threshold)        
    {   
        setThreshold(ee_goal_threshold_);
    }
    
    double ManipulatorGoalRegion::euclideanDistance(const std::vector<double> &vec1, const std::vector<double> &vec2) const{
        double sum = 0.0;
        for (size_t i = 0; i < vec1.size(); i++) {
            sum += pow(vec2[i] - vec1[i], 2);
        }
        
        return sqrt(sum);
    }

    double ManipulatorGoalRegion::distanceGoal(const ompl::base::State *st) const
    {	
        std::vector<double> v1;
        double* v = st->as<ompl::base::RealVectorStateSpace::StateType>()->values;
        for (unsigned int i = 0; i < state_space_information_->getStateDimension(); i++) {
           v1.push_back(v[i]);          
        }
        
        std::vector<double> ee_position;        
        static_cast<shared::ManipulatorRobot *>(robot_environment_->getRobot().get())->getEndEffectorPosition(v1, ee_position);
        
        /**std::vector<double> ee_g;
        for (size_t i = 0; i < ee_goal_position_.size(); i++) {
        	ee_g.push_back(ee_goal_position_[i]);
        }*/
        
        double distance = euclideanDistance(ee_position, ee_goal_position_);        
        return distance;             
    } 
    
    double ManipulatorGoalRegion::getThreshold() const {    	
    	return ee_goal_threshold_;
    }

    void ManipulatorGoalRegion::sampleGoal(ompl::base::State *st) const 
    {   
    	ompl::RNG rng;    	
        int rd = rng.uniformInt(0, goal_states_.size() - 1);   
        double* v = st->as<ompl::base::RealVectorStateSpace::StateType>()->values;
        for (unsigned int i = 0; i < state_space_information_->getStateDimension(); i++) {
           v[i] = goal_states_[rd][i];
        }           
    }
    
    void ManipulatorGoalRegion::sampleGoalVec(std::vector<double> &goal_vec) const {               
        ompl::RNG rng;
        int rd = rng.uniformInt(0, goal_states_.size() - 1);
        //int rd = 0;
        if (goal_states_.size() == 0) {cout << "wtf"<<endl;}
        for (unsigned int i = 0; i < state_space_information_->getStateDimension(); i++) {
           goal_vec.push_back(goal_states_[rd][i]);
        }
    }

    unsigned int ManipulatorGoalRegion::maxSampleCount() const
    {
        return goal_states_.size();
    }
    
    bool ManipulatorGoalRegion::isSatisfied(const ompl::base::State *st) const {    	
    	std::vector<double> joint_angles;    	
    	for (unsigned int i = 0; i < state_space_information_->getStateDimension(); i++) {
    		joint_angles.push_back(st->as<ompl::base::RealVectorStateSpace::StateType>()->values[i]);
    	}
    	std::vector<double> ee_position;
    	static_cast<shared::ManipulatorRobot *>(robot_environment_->getRobot().get())->getEndEffectorPosition(joint_angles, ee_position);
    	double sum(0.0);
    	for (unsigned int i = 0; i < joint_angles.size(); i++) {
    		sum += pow(ee_position[i] - ee_goal_position_[i], 2);
    	}
    	
    	sum = sqrt(sum);    	
    	if (sum < ee_goal_threshold_ - 0.01) {
    		return true;
    	}
    	
    	return false;
    	
    }
}

