#include "include/GoalRegion.hpp"

namespace frapu
{

GoalRegion::GoalRegion(const ompl::base::SpaceInformationPtr& si,                       
                       std::vector<std::vector<double>>& goal_states):
    ompl::base::GoalSampleableRegion(si),    
    state_space_information_(si),
    state_dimension_(si->getStateDimension()),
    goal_states_(goal_states)
{

}

}
