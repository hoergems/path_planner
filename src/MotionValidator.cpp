#include "include/MotionValidator.hpp"

using std::cout;
using std::endl;

using namespace fcl;

namespace shared
{

MotionValidator::MotionValidator(const ompl::base::SpaceInformationPtr& si,
                                 bool continuous_collision,
                                 bool dynamics):
    ompl::base::MotionValidator(si),
    si_(si),
    robot_environment_(nullptr),
    continuous_collision_(continuous_collision),
    dim_(si_->getStateSpace()->getDimension())
{

}

bool MotionValidator::checkMotion(const std::vector<double>& s1,
                                  const std::vector<double>& s2,
                                  const bool& continuous_collision) const
{
    if (continuous_collision) {
        return !collidesContinuous(s1, s2);
    } else {
        return !collidesDiscrete(s2);
    }
}

/** Check if a motion between two states is valid. This assumes that state s1 is valid */
bool MotionValidator::checkMotion(const ompl::base::State* s1, const ompl::base::State* s2) const
{
    std::vector<double> state_vec1;
    std::vector<double> state_vec2;

    for (unsigned int i = 0; i < dim_; i++) {
        state_vec1.push_back(s1->as<ompl::base::RealVectorStateSpace::StateType>()->values[i]);
        state_vec2.push_back(s2->as<ompl::base::RealVectorStateSpace::StateType>()->values[i]);
    }
    if (!satisfiesConstraints(state_vec2)) {
        return false;
    }

    return checkMotion(state_vec1, state_vec2, continuous_collision_);
}

/** Check if a motion between two states is valid. This assumes that state s1 is valid */
bool MotionValidator::checkMotion(const ompl::base::State* s1,
                                  const ompl::base::State* s2,
                                  std::pair< ompl::base::State*, double >& /*lastValid*/) const
{
    return checkMotion(s1, s2);
}

bool MotionValidator::satisfiesConstraints(const std::vector<double>& s1) const
{
    std::vector<double> state_vec;
    for (size_t i = 0; i < dim_; i++) {
        state_vec.push_back(s1[i]);
    }
    std::vector<double> lower_bounds = si_->getStateSpace()->as<ompl::base::RealVectorStateSpace>()->getBounds().low;
    std::vector<double> upper_bounds = si_->getStateSpace()->as<ompl::base::RealVectorStateSpace>()->getBounds().high;
    for (size_t i = 0; i < dim_; i++) {
        if (s1[i] < lower_bounds[i]) {
            return false;
        } else if (s1[i] > upper_bounds[i]) {
            return false;
        }
    }

    return true;
}

bool MotionValidator::isValid(const ompl::base::State* state) const
{
    std::vector<double> state_vec;
    for (unsigned int i = 0; i < dim_; i++) {
        state_vec.push_back(state->as<ompl::base::RealVectorStateSpace::StateType>()->values[i]);
    }

    return isValid(state_vec);
}

bool MotionValidator::isValid(const std::vector<double>& s1, bool debug) const
{
    std::vector<double> state_vec;
    for (size_t i = 0; i < dim_; i++) {
        state_vec.push_back(s1[i]);
    }

    bool satisfies_constraints = satisfiesConstraints(state_vec);
    bool collides_discrete = collidesDiscrete(state_vec);
    if (debug) {
        cout << "satisfies constraints " << satisfies_constraints << endl;
        cout << "collides discrete " << collides_discrete << endl;
    }

    if (!satisfies_constraints || collides_discrete) {
        return false;
    }


    return true;
}

bool MotionValidator::collidesDiscrete(const std::vector<double>& state) const
{
    std::vector<double> state_vec;
    for (size_t i = 0; i < dim_; i++) {
        state_vec.push_back(state[i]);
    }

    std::vector<std::shared_ptr<fcl::CollisionObject>> collision_objects;
    robot_environment_->getRobot()->createRobotCollisionObjects(state_vec, collision_objects);
    std::vector<std::shared_ptr<shared::Obstacle>> obstacles;
    robot_environment_->getObstacles(obstacles);
    for (size_t i = 0; i < obstacles.size(); i++) {
        if (!obstacles[i]->getTerrain()->isTraversable()) {
            if (obstacles[i]->in_collision(collision_objects)) {
                return true;
            }
        }
    }

    //return inSelfCollision(collision_objects);
    return false;
}

bool MotionValidator::collidesContinuous(const std::vector<double>& state1,
        const std::vector<double>& state2) const
{
    std::vector<std::shared_ptr<fcl::CollisionObject>> collision_objects_start;
    robot_environment_->getRobot()->createRobotCollisionObjects(state1, collision_objects_start);
    std::vector<std::shared_ptr<fcl::CollisionObject>> collision_objects_goal;
    robot_environment_->getRobot()->createRobotCollisionObjects(state2, collision_objects_goal);
    std::vector<std::shared_ptr<shared::Obstacle>> obstacles;
    robot_environment_->getObstacles(obstacles);
    for (size_t i = 0; i < obstacles.size(); i++) {
        if (!obstacles[i]->isTraversable()) {
            for (size_t j = 0; j < collision_objects_start.size(); j++) {
                if (obstacles[i]->in_collision(collision_objects_start[j], collision_objects_goal[j])) {
                    return true;
                }
            }
        }
    }

    return inSelfCollision(collision_objects_goal);
    //return false;
}

bool MotionValidator::inSelfCollision(const std::vector<double>& state) const
{
    return robot_environment_->getRobot()->checkSelfCollision(state);
}

void MotionValidator::setRobotEnvironment(std::shared_ptr<shared::RobotEnvironment>& robot_environment)
{
    robot_environment_ = robot_environment;
}

bool MotionValidator::inSelfCollision(std::vector<std::shared_ptr<fcl::CollisionObject>>& robot_collision_objects) const
{
    bool in_self_collision = robot_environment_->getRobot()->checkSelfCollision(robot_collision_objects);
    return in_self_collision;
}

void MotionValidator::setContinuousCollisionCheck(bool continuous_collision_check)
{
    continuous_collision_ = continuous_collision_check;
}

}
