#ifndef MAN_MOTION_VALIDATOR_HPP_
#define MAN_MOTION_VALIDATOR_HPP_

#include <ompl/base/MotionValidator.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/State.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <robot_environment/robot_environment.hpp>
#include <robot_environment/Obstacle.hpp>
#include <iostream>
#include <mutex>

namespace shared
{
    
class CollisionReport {
public:
    CollisionReport() {
	
    }
    
    // Continuous collision check
    bool continuousCollisionCheck = false;
    
    // Ignore unobservable obstacles when doing collision checking
    bool ignoreUnobservableObstacles = false;
    
    // Did we collide?
    bool collides = false;
    
    // The previous state
    frapu::RobotStateSharedPtr state1;

    // The next state
    frapu::RobotStateSharedPtr state2;

    // The obstacle we collided with
    std::string collidingObstacle;

    // Did we collide with an traversable obstacle?
    bool obstacleTraversable = false;    
};

class MotionValidator: public ompl::base::MotionValidator
{
public:
    MotionValidator(const ompl::base::SpaceInformationPtr& si,
                    bool continuous_collision,
                    bool dynamics);
    ~MotionValidator() = default;

    /** Check if a motion between two states is valid. This assumes that state s1 is valid */
    bool checkMotion(const ompl::base::State* s1, const ompl::base::State* s2) const;

    bool checkMotion(const std::vector<double>& s1,
                     const std::vector<double>& s2,
                     const bool& continuous_collision) const;

    /** Check if a motion between two states is valid. This assumes that state s1 is valid */
    bool checkMotion(const ompl::base::State* s1,
                     const ompl::base::State* s2,
                     std::pair< ompl::base::State*, double >& /*lastValid*/) const;

    bool satisfiesConstraints(const std::vector<double>& s1) const;

    bool isValid(const ompl::base::State* state) const;

    bool isValid(const std::vector<double>& s1, bool debug = false) const;

    void setContinuousCollisionCheck(bool continuous_collision_check);
    
    void setIgnoreUnobservableObstacles(bool ignore_unobservable_obstacles);

    bool collidesDiscrete(const std::vector<double>& state) const;

    bool collidesContinuous(const std::vector<double>& state1,
                            const std::vector<double>& state2) const;

    bool inSelfCollision(const std::vector<double>& state) const;

    bool inSelfCollision(std::vector<frapu::CollisionObjectSharedPtr>& robot_collision_objects) const;

    void setRobotEnvironment(std::shared_ptr<shared::RobotEnvironment>& robot_environment);
    
    void makeCollisionReport(std::shared_ptr<shared::CollisionReport> &collisionReport);

private:
    const ompl::base::SpaceInformationPtr si_;

    std::shared_ptr<shared::RobotEnvironment> robot_environment_;

    std::mutex mtx;

    bool continuous_collision_;
    
    bool ignore_unobservable_obstacles_;

    unsigned int dim_;

    //void set_link_aabbs() const;
};
}

#endif
