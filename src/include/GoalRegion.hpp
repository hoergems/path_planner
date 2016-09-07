#ifndef _GOAL_REGION_HPP_
#define _GOAL_REGION_HPP_
#include <boost/python.hpp>
#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/State.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/samplers/UniformValidStateSampler.h>
#include <ompl/util/RandomNumbers.h>
#include <robot_environment/robot_environment.hpp>
#include <frapu_core/core.hpp>

namespace frapu
{

class GoalRegion: public ompl::base::GoalSampleableRegion
{
public:
    GoalRegion(const ompl::base::SpaceInformationPtr& si,               
               std::vector<std::vector<double>>& goal_states);

    virtual double distanceGoal(const ompl::base::State* st) const = 0;

    virtual void sampleGoal(ompl::base::State* st) const = 0;

    virtual void sampleGoalVec(std::vector<double>& goal_vec) const = 0;

    virtual unsigned int maxSampleCount() const = 0;

    virtual bool isSatisfied(const ompl::base::State* st) const = 0;

protected:
    ompl::base::SpaceInformationPtr state_space_information_;

    unsigned int state_dimension_;

    std::vector<std::vector<double>> goal_states_;

};

struct GoalRegionWrapper: GoalRegion, boost::python::wrapper<GoalRegion> {
public:
    GoalRegionWrapper(const ompl::base::SpaceInformationPtr& si,                      
                      std::vector<std::vector<double>>& goal_states):
        GoalRegion(si, goal_states) {
    }

    double distanceGoal(const ompl::base::State* st) const {
        this->get_override("distanceGoal")(st);
    }

    void sampleGoal(ompl::base::State* st) const {
        this->get_override("sampleGoal")(st);
    }

    void sampleGoalVec(std::vector<double>& goal_vec) const {
        this->get_override("sampleGoalVec")(goal_vec);
    }

    unsigned int maxSampleCount() const {
        return this->get_override("maxSampleCount")();
    }

    bool isSatisfied(const ompl::base::State* st) const {
        return this->isSatisfied(st);
    }
};

}

#endif
