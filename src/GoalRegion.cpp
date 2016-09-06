#include "include/GoalRegion.hpp"

namespace frapu
{

GoalRegion::GoalRegion(const ompl::base::SpaceInformationPtr& si,
                       std::shared_ptr<frapu::RobotEnvironment>& robot_environment,
                       std::vector<std::vector<double>>& goal_states):
    ompl::base::GoalSampleableRegion(si),
    robot_environment_(robot_environment),
    state_space_information_(si),
    state_dimension_(si->getStateDimension()),
    goal_states_(goal_states)
{

}

}
