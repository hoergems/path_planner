#ifndef ROBOT_GOAL_REGION_HPP_
#define ROBOT_GOAL_REGION_HPP_
#include "GoalRegion.hpp"
#include <iostream>
#include <random>

namespace frapu
{

class RobotGoalRegion: public GoalRegion
{
public:
    RobotGoalRegion() = default;

    RobotGoalRegion(const ompl::base::SpaceInformationPtr& si,
		    frapu::RobotSharedPtr &robot,
                    std::vector<std::vector<double>>& goal_states);

    RobotGoalRegion(std::shared_ptr<frapu::RobotGoalRegion>& other_goal_region);


    //~RobotGoalRegion() = default;

    virtual double distanceGoal(const ompl::base::State* st) const override;

    virtual void sampleGoal(ompl::base::State* st) const override;

    double getThreshold() const;

    virtual void sampleGoalVec(std::vector<double>& goal_vec) const override;

    virtual unsigned int maxSampleCount() const override;

    virtual bool isSatisfied(const ompl::base::State* st) const override;


private:
    double goal_threshold_;
    
    frapu::RobotSharedPtr robot_;

};

}

#endif
