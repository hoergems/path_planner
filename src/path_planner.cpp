#include "include/path_planner.hpp"

using std::cout;
using std::endl;

namespace frapu
{

StandardPathPlanner::StandardPathPlanner(double delta_t,
        bool continuous_collision,
        double max_joint_velocity,
        double stretching_factor,
        bool check_linear_path,
        bool verbose):
    PathPlanner(),
    goal_region_(nullptr),
    dim_(0),
    delta_t_(delta_t),
    continuous_collision_(continuous_collision),
    max_joint_velocity_(max_joint_velocity),
    stretching_factor_(stretching_factor),
    planning_range_(delta_t_ * max_joint_velocity_),
    check_linear_path_(check_linear_path),
    space_(nullptr),
    si_(nullptr),
    problem_definition_(nullptr),
    planner_(nullptr),
    motionValidator_(nullptr),
    verbose_(verbose),
    planner_str_("")
{
    if (!verbose_) {
        ompl::msg::noOutputHandler();
    }
}

void StandardPathPlanner::setPlanningDimensions(int& dimensions)
{
    dim_ = dimensions;
}

void StandardPathPlanner::setupPlanner(std::string planner_str)
{
    planner_str_ = planner_str;
    if (planner_str == "RRTConnect") {
        planner_ = ompl::base::PlannerPtr(new ompl::geometric::RRTConnect(si_));
        static_cast<ompl::geometric::RRTConnect*>(planner_.get())->setRange(planning_range_);
    } else if (planner_str == "RRT") {
        planner_ = ompl::base::PlannerPtr(new ompl::geometric::RRT(si_));
        static_cast<ompl::geometric::RRT*>(planner_.get())->setRange(planning_range_);
        static_cast<ompl::geometric::RRT*>(planner_.get())->setGoalBias(0.1);
    } else if (planner_str == "SBL") {
        planner_ = ompl::base::PlannerPtr(new ompl::geometric::SBL(si_));
        static_cast<ompl::geometric::SBL*>(planner_.get())->setRange(planning_range_);
    } else if (planner_str == "BKPIECE1") {
        planner_ = ompl::base::PlannerPtr(new ompl::geometric::BKPIECE1(si_));
        static_cast<ompl::geometric::BKPIECE1*>(planner_.get())->setRange(planning_range_);
    } else if (planner_str == "PDST") {
        planner_ = ompl::base::PlannerPtr(new ompl::geometric::PDST(si_));
        static_cast<ompl::geometric::PDST*>(planner_.get())->setGoalBias(0.1);
    } else if (planner_str == "STRIDE") {
        planner_ = ompl::base::PlannerPtr(new ompl::geometric::STRIDE(si_));
        static_cast<ompl::geometric::STRIDE*>(planner_.get())->setRange(planning_range_);
    }
    planner_->setProblemDefinition(problem_definition_);
}

void StandardPathPlanner::setup(std::shared_ptr<frapu::RobotEnvironment>& robot_environment)
{
    dim_ = robot_environment->getRobot()->getDOF();
    space_ = ompl::base::StateSpacePtr(new ompl::base::RealVectorStateSpace(dim_));
    si_ = ompl::base::SpaceInformationPtr(new ompl::base::SpaceInformation(space_));

    problem_definition_ = ompl::base::ProblemDefinitionPtr(new ompl::base::ProblemDefinition(si_));
    motionValidator_ = std::make_shared<frapu::MotionValidator>(si_, continuous_collision_, false);    
    //motionValidator_ = boost::make_shared<shared::MotionValidator>(si_, continuous_collision_, false);
    //problem_definition_(new ompl::base::ProblemDefinition(si_))
    /**motionValidator_(new MotionValidator(si_,
                                             continuous_collision,
                                             false))*/
    static_cast<frapu::MotionValidator*>(motionValidator_.get())->setRobotEnvironment(robot_environment);
    std::vector<double> lowerStateLimits;
    std::vector<double> upperStateLimits;
    ompl::base::RealVectorBounds bounds(dim_);
    frapu::StateLimitsSharedPtr stateLimits =
        robot_environment->getRobot()->getStateSpace()->getStateLimits();
    static_cast<frapu::VectorStateLimits*>(stateLimits.get())->getVectorLimits(lowerStateLimits, upperStateLimits);

    if (robot_environment->getRobot()->constraintsEnforced()) {
        for (size_t i = 0; i < dim_; i++) {
            bounds.setLow(i, lowerStateLimits[i]);
            bounds.setHigh(i, upperStateLimits[i]);
        }
    } else {
        for (size_t i = 0; i < dim_; i++) {
            bounds.setLow(i, -100000);
            bounds.setHigh(i, 100000);
        }
    }

    /** Apply the bounds to the space */
    space_->as<ompl::base::RealVectorStateSpace>()->setBounds(bounds);

    /** Set the StateValidityChecker */
    si_->setStateValidityChecker(boost::bind(&StandardPathPlanner::isValid, this, _1));

    /** Set the MotionValidation */
    si_->setMotionValidator(motionValidator_);
}

ompl::base::MotionValidatorPtr StandardPathPlanner::getMotionValidator()
{
    assert(motionValidator_ && "MotionValidator in path planner is null!!!");
    return motionValidator_;
}

bool StandardPathPlanner::isValidPy(std::vector<double>& state)
{
    bool valid = static_cast<MotionValidator&>(*motionValidator_).isValid(state);
    return valid;
}

bool StandardPathPlanner::isValid(const ompl::base::State* state)
{
    std::vector<double> state_vec;
    for (unsigned int i = 0; i < si_->getStateSpace()->getDimension(); i++) {
        state_vec.push_back(state->as<ompl::base::RealVectorStateSpace::StateType>()->values[i]);
    }
    return static_cast<MotionValidator&>(*motionValidator_).isValid(state_vec);
}

void StandardPathPlanner::clear()
{
    planner_->clear();
    if (problem_definition_->hasSolution()) {
        problem_definition_->clearSolutionPaths();
    }

    if (problem_definition_->getStartStateCount() > 0) {
        problem_definition_->clearStartStates();
    }
}

void StandardPathPlanner::clearAll()
{
    clear();
    if (problem_definition_->getGoal()) {
        problem_definition_->clearGoal();
    }

}

/** Generates a linear path from point p1 to point p2 */
std::vector<std::vector<double> > StandardPathPlanner::genLinearPath(std::vector<double>& p1, std::vector<double>& p2)
{
    std::vector<double> vec;
    std::vector<double> vec_res;
    std::vector<std::vector<double> > solution_vector;
    for (size_t i = 0; i < p1.size(); i++) {
        vec.push_back(p2[i] - p1[i]);
        vec_res.push_back(p1[i]);
    }

    double length(0.0);
    for (size_t j = 0; j < vec.size(); j++) {
        length += std::pow(vec[j], 2);
    }
    length = std::sqrt(length);
    int num_points = (int)(length / planning_range_);
    std::vector<double> vec_norm;
    for (size_t j = 0; j < vec.size(); j++) {
        vec_norm.push_back(vec[j] / length);
    }

    solution_vector.push_back(p1);
    for (int j = 0; j < num_points; j++) {
        for (size_t k = 0; k < vec_res.size(); k++) {
            vec_res[k] += planning_range_ * vec_norm[k];
        }
        std::vector<double> vec_append(vec_res);
        solution_vector.push_back(vec_append);
    }

    solution_vector.push_back(p2);
    return solution_vector;
}

void StandardPathPlanner::setGoal(ompl::base::GoalPtr& goalRegion)
{
    goal_region_ = goalRegion;
}

ompl::base::SpaceInformationPtr StandardPathPlanner::getSpaceInformation()
{
    return si_;
}

/**void PathPlanner::setGoalStates(std::vector<std::vector<double>> &goal_states,
                                std::vector<double> &ee_goal_position,
                                double ee_goal_threshold) {
    for (size_t i = 0; i < goal_states.size(); i++) {
        goal_states_.push_back(goal_states[i]);
    }

    ee_goal_position_.clear();
    for (size_t i = 0; i < ee_goal_position.size(); i++) {
      ee_goal_position_.push_back(ee_goal_position[i]);
    }

    ee_goal_threshold_ = ee_goal_threshold;
}*/

bool StandardPathPlanner::solve_(double time_limit)
{
    bool solved = false;
    bool hasExactSolution = false;

    solved = planner_->solve(time_limit);

    // Get all the solutions
    std::vector<ompl::base::PlannerSolution> solutions = problem_definition_->getSolutions();
    for (size_t i = 0; i < solutions.size(); i++) {
        if (!solutions[i].approximate_) {
            return true;

        }
    }
    // Check if there's an exact solution

    return false;
}

TrajectorySharedPtr StandardPathPlanner::solve(const RobotStateSharedPtr& robotState,
        double timeout)
{
    TrajectorySharedPtr trajectory;
    std::vector<double> startStateVec = static_cast<VectorState*>(robotState.get())->asVector();
    VectorTrajectory vectorTrajectory = solve(startStateVec, timeout);
    if (vectorTrajectory.states.size() == 0) {
        return nullptr;
    }
    trajectory->stateTrajectory = std::vector<RobotStateSharedPtr>(vectorTrajectory.states.size());
    trajectory->observationTrajectory = std::vector<ObservationSharedPtr>(vectorTrajectory.observations.size());
    trajectory->actionTrajectory = std::vector<ActionSharedPtr>(vectorTrajectory.controls.size());
    for (size_t i = 0; i < vectorTrajectory.states.size(); i++) {
        trajectory->stateTrajectory[i] = std::make_shared<VectorState>(vectorTrajectory.states[i]);
        trajectory->observationTrajectory[i] = std::make_shared<VectorObservation>(vectorTrajectory.observations[i]);
        trajectory->actionTrajectory[i] = std::make_shared<VectorAction>(vectorTrajectory.controls[i]);
    }

    return trajectory;
}

/** Solves the motion planning problem */

VectorTrajectory StandardPathPlanner::solve(const std::vector<double>& start_state_vec, double timeout)
{

    std::vector<double> ss_vec;
    ompl::base::ScopedState<> start_state(space_);
    for (size_t i = 0; i < dim_; i++) {
        ss_vec.push_back(start_state_vec[i]);
        start_state[i] = ss_vec[i];
    }

    std::vector<std::vector<double> > solution_vector;
    MotionValidator* mv = static_cast<MotionValidator*>(si_->getMotionValidator().get());

    // Add the start state to the problem definition
    if (verbose_) {
        cout << "Adding start state: ";
        for (size_t i = 0; i < dim_; i++) {
            cout << start_state[i] << ", ";
        }
        cout << endl;
    }

    problem_definition_->addStartState(start_state);
    if (!goal_region_) {
        cout << "PathPlanner: Error: no goal region defined!" << endl;
    }
    problem_definition_->setGoal(std::static_pointer_cast<ompl::base::Goal>(goal_region_));
    //problem_definition_->setGoal(boost::static_pointer_cast<ompl::base::Goal>(goal_region_));

    if (check_linear_path_) {
        bool collides = false;
        std::vector<double> goal_state_vec;
        static_cast<frapu::GoalRegion*>(goal_region_.get())->sampleGoalVec(goal_state_vec);
        std::vector<std::vector<double> > linear_path(genLinearPath(ss_vec, goal_state_vec));

        for (size_t i = 1; i < linear_path.size(); i++) {
            if (!(mv->checkMotion(linear_path[i - 1], linear_path[i], continuous_collision_))) {
                collides = true;
                break;
            }
        }

        if (!collides) {
            clear();
            if (verbose_) {
                cout << "Linear path is a valid solution. Returning linear path of length " << linear_path.size() << endl;
            }

            return augmentPath_(linear_path);
        }

    }

    // Solve the planning problem with a maximum of *timeout* seconds per attempt
    bool solved = false;
    boost::timer t;
    bool approximate_solution = true;
    solved = solve_(timeout);
    if (!solved) {
        VectorTrajectory v;
        return v;
    }


    ompl::geometric::PathGeometric* solution_path =
        static_cast<ompl::geometric::PathGeometric*>(problem_definition_->getSolutionPath().get());

    solution_vector.push_back(ss_vec);
    // We found a solution, so get the solution path

    if (verbose_) {
        cout << "Solution found in " << t.elapsed() << " seconds." << endl;
        cout << "Solution path has length " << solution_path->getStates().size() << endl;
    }

    std::vector<double> vals;
    const bool cont_check = true;
    for (size_t i = 1; i < solution_path->getStates().size(); i++) {
        vals.clear();
        for (unsigned int j = 0; j < dim_; j++) {
            vals.push_back(solution_path->getState(i)->as<ompl::base::RealVectorStateSpace::StateType>()->values[j]);
        }

        solution_vector.push_back(vals);
    }
    clear();
    return augmentPath_(solution_vector);
}

VectorTrajectory StandardPathPlanner::augmentPath_(std::vector<std::vector<double>>& solution_path)
{
    VectorTrajectory vectorTrajectory;
    for (size_t i = 0; i < solution_path.size(); i++) {
        std::vector<double> path_element;
        std::vector<double> solution_element;
        std::vector<double> next_solution_element;
        std::vector<double> control;
        std::vector<double> observation;
        for (size_t j = 0; j < dim_; j++) {
            solution_element.push_back(solution_path[i][j]);
            if (i != solution_path.size() - 1) {
                next_solution_element.push_back(solution_path[i + 1][j]);
                control.push_back((next_solution_element[j] - solution_element[j]) / delta_t_);
            } else {
                control.push_back(0.0);
            }
            observation.push_back(solution_element[j]);

        }
        /**for (size_t j = 0; j < dim_; j++) {
          control.push_back(0.0);
        }*/

        for (size_t j = 0; j < dim_; j++) {
            solution_element.push_back(0.0);
            observation.push_back(0.0);
        }

        vectorTrajectory.states.push_back(solution_element);
        vectorTrajectory.observations.push_back(observation);
        vectorTrajectory.controls.push_back(control);

        if (i != solution_path.size() - 1) {
            vectorTrajectory.durations.push_back(delta_t_);
        } else {
            vectorTrajectory.durations.push_back(0.0);
        }
    }

    return vectorTrajectory;
}



}

