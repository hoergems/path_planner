#ifndef PATH_PLANNER_HPP_
#define PATH_PLANNER_HPP_
#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/RealVectorBounds.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/Goal.h>
#include "RobotGoalRegion.hpp"
#include <boost/make_shared.hpp>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/sbl/SBL.h>
#include <ompl/geometric/planners/kpiece/BKPIECE1.h>
#include <ompl/geometric/planners/pdst/PDST.h>
#include <ompl/geometric/planners/stride/STRIDE.h>
#include <ompl/base/Path.h>
#include <ompl/base/PlannerTerminationCondition.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/base/Planner.h>
#include <ompl/util/Console.h>
#include <ompl/base/MotionValidator.h>
#include "MotionValidator.hpp"
#include <robot_headers/robot.hpp>
#include <boost/timer.hpp>
#include <frapu_core/core.hpp>
//#include "trajectory.hpp"

#include <iostream>


namespace frapu
{

class StandardPathPlanner: public frapu::PathPlanner
{
public:
    StandardPathPlanner() = default;

    StandardPathPlanner(double delta_t,
                        bool continuous_collision,
                        double max_joint_velocity,
                        double stretching_factor,
                        bool check_linear_path,
                        bool verbose);

    ~StandardPathPlanner() {
        clearAll();
    }

    /** Checks of a state is valid */
    bool isValid(const ompl::base::State* state);

    bool isValidPy(std::vector<double>& state);

    virtual TrajectorySharedPtr solve(const RobotStateSharedPtr& robotState,
                                      double timeout) override;

    void setPlanningDimensions(int& dimensions);

    void setupPlanner(std::string planner_string);

    /** Setup OMPL. This need to be called before solving the motion planning problem*/
    void setup(frapu::SceneSharedPtr &scene, frapu::RobotSharedPtr &robot);

    void setGoal(ompl::base::GoalPtr& goalRegion);

    ompl::base::MotionValidatorPtr getMotionValidator();

    ompl::base::SpaceInformationPtr getSpaceInformation();

private:
    VectorTrajectory solve(const std::vector<double>& start_state_vec, double timeout);

    ompl::base::GoalPtr goal_region_;

    /** The dimension of the space we're planning in */
    int dim_;
    
    unsigned int stateSpaceDimension_;

    /** The maximum allowed euclidean distance between two connected nodes */
    double delta_t_;

    bool continuous_collision_;

    double max_joint_velocity_;

    double stretching_factor_;

    double planning_range_;

    bool check_linear_path_;

    /** The space we're planning in */
    ompl::base::StateSpacePtr space_;

    /** A SpaceInformation pointer */
    ompl::base::SpaceInformationPtr si_;

    /** The definition of the path planning problem */
    ompl::base::ProblemDefinitionPtr problem_definition_;

    /** A pointer to the path planner */
    ompl::base::PlannerPtr planner_;

    /** A simplifier which can be used to simplify (shorten an smoothen) paths */
    //ompl::geometric::PathSimplifier simplifier_;

    ompl::base::MotionValidatorPtr motionValidator_;

    std::string planner_str_;

    bool verbose_;

    std::vector<std::vector<double>> goal_states_;

    std::vector<double> ee_goal_position_;

    double ee_goal_threshold_;

    /** Clean memory from unused states */
    void clear();

    void clearAll();

    bool solve_(double time_limit);

    /** Generates a linear path from point p1 to point p2 */
    std::vector<std::vector<double> > genLinearPath(std::vector<double>& p1, std::vector<double>& p2);

    VectorTrajectory augmentPath_(std::vector<std::vector<double>>& solution_path);
};
}

#endif



