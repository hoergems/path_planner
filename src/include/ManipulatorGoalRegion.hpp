#ifndef MANIPULATOR_GOAL_REGION_HPP_
#define MANIPULATOR_GOAL_REGION_HPP_
#include "GoalRegion.hpp"
#include <robot_environment/robot_environment.hpp>
#include <iostream>
#include <random>
#include <boost/python.hpp>

namespace shared {    

    class ManipulatorGoalRegion: public shared::GoalRegion {
        public:
    	    ManipulatorGoalRegion() = default;
    	
            ManipulatorGoalRegion(const ompl::base::SpaceInformationPtr &si,
            		              std::shared_ptr<shared::RobotEnvironment> &robot_environment,
                                  std::vector<std::vector<double>> &goal_states,
                                  std::vector<double> &ee_goal_position,
                                  double &ee_goal_threshold,                                  
                                  bool dynamics);
                                  
            //~ManipulatorGoalRegion() = default;

            virtual double distanceGoal(const ompl::base::State *st) const override;

            virtual void sampleGoal(ompl::base::State *st) const override;
            
            double getThreshold() const;
            
            virtual void sampleGoalVec(std::vector<double> &goal_vec) const override;

            virtual unsigned int maxSampleCount() const override;
            
            virtual bool isSatisfied(const ompl::base::State *st) const override;

        private:
            double euclideanDistance(const std::vector<double> &vec1, const std::vector<double> &vec2) const;
            
            std::vector<double> ee_goal_position_;
            
            double ee_goal_threshold_;
            
            
    };

}

#endif
