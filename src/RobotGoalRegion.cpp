#include "include/RobotGoalRegion.hpp"

#include <fstream>
#include <string>

using std::cout;
using std::endl;

namespace frapu
{

RobotGoalRegion::RobotGoalRegion(const ompl::base::SpaceInformationPtr& si,
				 frapu::RobotSharedPtr &robot,
                                 std::vector<std::vector<double>>& goal_states):
    frapu::GoalRegion(si, goal_states),
    robot_(robot),
    goal_threshold_(0.0)
{    
    std::vector<double> goalArea;
    frapu::GoalSharedPtr goal = robot->getGoal();
    static_cast<frapu::SphereGoal *>(goal.get())->getGoalArea(goalArea);
    setThreshold(goalArea[3]);
    goal_threshold_ = goalArea[3];
}

double RobotGoalRegion::distanceGoal(const ompl::base::State* st) const
{
    std::vector<double> v1;
    double* v = st->as<ompl::base::RealVectorStateSpace::StateType>()->values;
    for (unsigned int i = 0; i < state_space_information_->getStateDimension(); i++) {
        v1.push_back(v[i]);
    }

    frapu::RobotStateSharedPtr robotState = std::make_shared<frapu::VectorState>(v1);
    return robot_->distanceGoal(robotState);
    //return distance;
}

double RobotGoalRegion::getThreshold() const
{
    return goal_threshold_;
}

void RobotGoalRegion::sampleGoal(ompl::base::State* st) const
{
    ompl::RNG rng;
    int rd = rng.uniformInt(0, goal_states_.size() - 1);
    double* v = st->as<ompl::base::RealVectorStateSpace::StateType>()->values;
    for (unsigned int i = 0; i < state_space_information_->getStateDimension(); i++) {
        v[i] = goal_states_[rd][i];
    }
}

void RobotGoalRegion::sampleGoalVec(std::vector<double>& goal_vec) const
{
    ompl::RNG rng;
    int rd = rng.uniformInt(0, goal_states_.size() - 1);
    //int rd = 0;
    if (goal_states_.size() == 0) {
        cout << "wtf" << endl;
    }
    for (unsigned int i = 0; i < state_space_information_->getStateDimension(); i++) {
        goal_vec.push_back(goal_states_[rd][i]);
    }
}

unsigned int RobotGoalRegion::maxSampleCount() const
{
    return goal_states_.size();
}

bool RobotGoalRegion::isSatisfied(const ompl::base::State* st) const
{
    std::vector<double> state_vec;
    for (unsigned int i = 0; i < state_space_information_->getStateDimension(); i++) {
        state_vec.push_back(st->as<ompl::base::RealVectorStateSpace::StateType>()->values[i]);
    }

    frapu::RobotStateSharedPtr state = std::make_shared<frapu::VectorState>(state_vec);
    return robot_->isTerminal(state);
}
}

