#include "include/MotionValidator.hpp"

using std::cout;
using std::endl;

using namespace fcl;

namespace frapu
{

MotionValidator::MotionValidator(const ompl::base::SpaceInformationPtr& si,
                                 bool continuous_collision,
                                 bool dynamics):
    ompl::base::MotionValidator(si),
    si_(si),
    scene_(nullptr),
    robot_(nullptr),
    continuous_collision_(continuous_collision),
    ignore_unobservable_obstacles_(true),
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

bool MotionValidator::isValid(const frapu::RobotStateSharedPtr &state) const {
    std::vector<double> stateVec = static_cast<frapu::VectorState *>(state.get())->asVector();
    return isValid(stateVec, false);
}

bool MotionValidator::collidesDiscrete(const std::vector<double>& state) const
{
    std::vector<double> state_vec;
    for (size_t i = 0; i < dim_; i++) {
        state_vec.push_back(state[i]);
    }

    frapu::RobotStateSharedPtr robotState = std::make_shared<frapu::VectorState>(state_vec);
    std::vector<frapu::CollisionObjectSharedPtr> collision_objects;
    robot_->createRobotCollisionObjects(robotState, collision_objects);
    std::vector<frapu::ObstacleSharedPtr> obstacles;
    if (ignore_unobservable_obstacles_) {
        scene_->getObservableObstacles(obstacles);

    } else {
        scene_->getObstacles(obstacles);
    }

    for (size_t i = 0; i < obstacles.size(); i++) {
        if (!obstacles[i]->getTerrain()->isTraversable()) {
            if (obstacles[i]->inCollision(collision_objects)) {
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
    frapu::RobotStateSharedPtr robotState1 = std::make_shared<frapu::VectorState>(state2);
    frapu::RobotStateSharedPtr robotState2 = std::make_shared<frapu::VectorState>(state2);
    std::vector<frapu::CollisionObjectSharedPtr> collision_objects_start;
    robot_->createRobotCollisionObjects(robotState1, collision_objects_start);
    std::vector<frapu::CollisionObjectSharedPtr> collision_objects_goal;
    robot_->createRobotCollisionObjects(robotState2, collision_objects_goal);
    std::vector<frapu::ObstacleSharedPtr> obstacles;
    if (ignore_unobservable_obstacles_) {
        scene_->getObservableObstacles(obstacles);
    } else {
        scene_->getObstacles(obstacles);
    }
    for (size_t i = 0; i < obstacles.size(); i++) {
        if (!obstacles[i]->getTerrain()->isTraversable()) {
            for (size_t j = 0; j < collision_objects_start.size(); j++) {
                if (obstacles[i]->inCollisionContinuous(collision_objects_start[j], collision_objects_goal[j])) {
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
    frapu::RobotStateSharedPtr robotState = std::make_shared<frapu::VectorState>(state);
    return robot_->checkSelfCollision(robotState);
}

void MotionValidator::setScene(frapu::SceneSharedPtr &scene) {
    scene_ = scene;
}
    
void MotionValidator::setRobot(frapu::RobotSharedPtr &robot) {
    robot_ = robot;
}

bool MotionValidator::inSelfCollision(std::vector<frapu::CollisionObjectSharedPtr>& robot_collision_objects) const
{
    bool in_self_collision = robot_->checkSelfCollision(robot_collision_objects);
    return in_self_collision;
}

void MotionValidator::setContinuousCollisionCheck(bool continuous_collision_check)
{
    continuous_collision_ = continuous_collision_check;
}

void MotionValidator::setIgnoreUnobservableObstacles(bool ignore_unobservable_obstacles)
{
    ignore_unobservable_obstacles_ = ignore_unobservable_obstacles;
}

void MotionValidator::makeCollisionReport(std::shared_ptr<frapu::CollisionReport>& collisionReport) const
{
    std::vector<frapu::CollisionObjectSharedPtr> collision_objects_goal;
    robot_->createRobotCollisionObjects(collisionReport->state2, collision_objects_goal);
    std::vector<frapu::ObstacleSharedPtr> obstacles;
    if (collisionReport->ignoreUnobservableObstacles) {
        scene_->getObservableObstacles(obstacles);
    } else {
        scene_->getObstacles(obstacles);
    }

    collisionReport->collides = false;
    unsigned int collidingObstacleIndex = 0;
    if (collisionReport->continuousCollisionCheck) {
        std::vector<frapu::CollisionObjectSharedPtr> collision_objects_start;
        robot_->createRobotCollisionObjects(collisionReport->state1, collision_objects_start);
        for (size_t i = 0; i < obstacles.size(); i++) {
            for (size_t j = 0; j < collision_objects_start.size(); j++) {
                if (obstacles[i]->inCollisionContinuous(collision_objects_start[j], collision_objects_goal[j])) {
                    collisionReport->collides = true;
                    collidingObstacleIndex = i;
                    break;
                }
            }
        }
    } else {
        for (size_t i = 0; i < obstacles.size(); i++) {
            if (obstacles[i]->inCollision(collision_objects_goal)) {
                collisionReport->collides = true;
                collidingObstacleIndex = i;
                break;
            }
        }
    }

    if (collisionReport->collides) {
        collisionReport->collidingObstacle = obstacles[collidingObstacleIndex]->getName();
        collisionReport->obstacleTraversable = obstacles[collidingObstacleIndex]->getTerrain()->isTraversable();
    }

}

}
