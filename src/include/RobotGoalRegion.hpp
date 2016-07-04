#ifndef ROBOT_GOAL_REGION_HPP_
#define ROBOT_GOAL_REGION_HPP_
#include "GoalRegion.hpp"
#include <robot_environment/robot_environment.hpp>
#include <iostream>
#include <random>
#include <boost/python.hpp>

namespace shared {    

    class RobotGoalRegion: public shared::GoalRegion {
        public:
    	    RobotGoalRegion() = default;
    	
            RobotGoalRegion(const ompl::base::SpaceInformationPtr &si,
            		              std::shared_ptr<shared::RobotEnvironment> &robot_environment,
                                  std::vector<std::vector<double>> &goal_states);
            
            RobotGoalRegion(std::shared_ptr<shared::RobotGoalRegion> &other_goal_region);
                                  
                                  
            //~RobotGoalRegion() = default;

            virtual double distanceGoal(const ompl::base::State *st) const override;

            virtual void sampleGoal(ompl::base::State *st) const override;
            
            double getThreshold() const;
            
            virtual void sampleGoalVec(std::vector<double> &goal_vec) const override;

            virtual unsigned int maxSampleCount() const override;
            
            virtual bool isSatisfied(const ompl::base::State *st) const override;    
	    

        private:
            double euclideanDistance(const std::vector<double> &vec1, const std::vector<double> &vec2) const;
            
            double goal_threshold_;
            
    };

}

#endif
