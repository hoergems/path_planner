#include "include/dynamic_path_planner.hpp"
#include <random>
#include <fstream>
#include <iterator>
#include <stdio.h>

using std::cout;
using std::endl;

namespace frapu
{

ompl::base::GoalPtr makeRobotGoalRegion(const ompl::base::SpaceInformationPtr si,
                                        std::shared_ptr<frapu::RobotEnvironment>& robot_environment,
                                        std::vector<std::vector<double>> goal_states)
{
    if (goal_states.size() == 0) {
        cout << "No goal states provided. In order to generate a goal area, at least one goal state is required." << endl;
        return nullptr;
    }
    ompl::base::GoalPtr robot_goal_region(new RobotGoalRegion(si, robot_environment, goal_states));

    if (!robot_goal_region) {
        cout << "NULLLLPTR!!!!!!!!" << endl;
    }
    return robot_goal_region;
}


DynamicPathPlanner::DynamicPathPlanner(bool verbose):
    goal_region_(nullptr),
    state_space_dimension_(0),
    control_space_dimension_(0),
    state_space_(nullptr),
    state_space_bounds_(1),
    control_space_(nullptr),
    space_information_(nullptr),
    problem_definition_(nullptr),
    planner_(nullptr),
    planner_str_("RRT"),
    state_propagator_(nullptr),
    motionValidator_(nullptr),
    verbose_(verbose),
    all_states_(),
    num_control_samples_(1),
    control_sampler_("discrete"),
    rrt_goal_bias_(0.05),
    min_max_control_duration_(std::vector<int>( {1, 4})),
                          add_intermediate_states_(true)
{

}

DynamicPathPlanner::DynamicPathPlanner(std::shared_ptr<DynamicPathPlanner>& dynamic_path_planner,
                                       std::shared_ptr<frapu::RobotEnvironment>& robot_environment):
    goal_region_(nullptr),
    state_space_dimension_(0),
    control_space_dimension_(0),
    state_space_(nullptr),
    state_space_bounds_(1),
    control_space_(nullptr),
    space_information_(nullptr),
    problem_definition_(nullptr),
    planner_(nullptr),
    planner_str_(dynamic_path_planner->planner_str_),
    state_propagator_(nullptr),
    motionValidator_(nullptr),
    verbose_(dynamic_path_planner->verbose_),
    all_states_(),
    num_control_samples_(1),
    control_sampler_("discrete"),
    rrt_goal_bias_(0.05),
    min_max_control_duration_(std::vector<int>( {1, 4})),
                          add_intermediate_states_(true)
{
    setup(robot_environment, planner_str_);
    ompl::base::GoalPtr goal_region = dynamic_path_planner->getGoalRegion();
    setGoal(goal_region);
    setNumControlSamples(dynamic_path_planner->num_control_samples_);
    setControlSampler(dynamic_path_planner->control_sampler_);
    setRRTGoalBias(dynamic_path_planner->rrt_goal_bias_);
    setMinMaxControlDuration(dynamic_path_planner->min_max_control_duration_);
    addIntermediateStates(dynamic_path_planner->add_intermediate_states_);
}

ompl::base::GoalPtr DynamicPathPlanner::getGoalRegion() const
{
    return goal_region_;
}

ompl::base::MotionValidatorPtr DynamicPathPlanner::getMotionValidator() const
{
    return motionValidator_;
}

ompl::base::SpaceInformationPtr DynamicPathPlanner::getSpaceInformation()
{
    assert(space_information_ && "DynamicPathPlanner: Fatal error: space_information not initialized. Did you call setup()?");
    return space_information_;
}

void DynamicPathPlanner::setRRTGoalBias(double goal_bias)
{
    rrt_goal_bias_ = goal_bias;
    if (planner_str_ == "EST") {
        static_cast<ESTControl*>(planner_.get())->setGoalBias(goal_bias);
    } else {
        static_cast<RRTControl*>(planner_.get())->setGoalBias(goal_bias);
    }

}

void DynamicPathPlanner::log_(std::string msg, bool warn = false)
{
    if (warn) {
        cout << "DynamicPathPlanner: " << msg << endl;
    } else if (verbose_) {
        cout << "DynamicPathPlanner: " << msg << endl;
    }
}

bool DynamicPathPlanner::setup(std::shared_ptr<frapu::RobotEnvironment>& robot_environment,
                               std::string planner)
{
    state_space_dimension_ = robot_environment->getRobot()->getStateSpace()->getNumDimensions();
    control_space_dimension_ = robot_environment->getRobot()->getActionSpace()->getNumDimensions();
    state_space_ = ompl::base::StateSpacePtr(new ompl::base::RealVectorStateSpace(state_space_dimension_));
    control_space_ = ompl::control::ControlSpacePtr(new frapu::ControlSpace(state_space_, control_space_dimension_));
    space_information_ = ompl::control::SpaceInformationPtr(new ompl::control::SpaceInformation(state_space_, control_space_));

    planner_str_ = planner;
    /***** Setup OMPL *****/
    log_("Setting up OMPL");
    setup_ompl_(robot_environment, verbose_);
    log_("OMPL setup");
    log_("Setup complete");
    return true;
}

ompl::control::ControlSamplerPtr DynamicPathPlanner::allocUniformControlSampler_(const ompl::control::ControlSpace* control_space)
{
    return nullptr;
    return ompl::control::ControlSamplerPtr(new UniformControlSampler(control_space));
}

void DynamicPathPlanner::setNumControlSamples(unsigned int num_control_samples)
{
    num_control_samples_ = num_control_samples;
    static_cast<frapu::PlanningSpaceInformation*>(space_information_.get())->setNumControlSamples(num_control_samples);
}

void DynamicPathPlanner::setControlSampler(std::string control_sampler)
{
    control_sampler_ = control_sampler;
    static_cast<frapu::ControlSpace*>(control_space_.get())->setControlSampler(control_sampler);
}

void DynamicPathPlanner::setMinMaxControlDuration(std::vector<int>& min_max_control_duration)
{
    min_max_control_duration_ = min_max_control_duration;
    unsigned int min = (unsigned int)min_max_control_duration[0];
    unsigned int max = (unsigned int)min_max_control_duration[1];
    space_information_->setMinMaxControlDuration(min, max);
}

void DynamicPathPlanner::addIntermediateStates(bool add_intermediate_states)
{
    add_intermediate_states_ = add_intermediate_states;
    if (planner_str_ == "RRT") {
        static_cast<RRTControl*>(planner_.get())->setIntermediateStates(add_intermediate_states);
    } else if (planner_str_ == "EST") {
        static_cast<ESTControl*>(planner_.get())->setIntermediateStates(true);
        //boost::static_pointer_cast<ESTControl>(planner_)->setIntermediateStates(true);
    }
}

void DynamicPathPlanner::setContinuousCollisionCheck(bool continuous_collision)
{
    static_cast<MotionValidator&>(*motionValidator_).setContinuousCollisionCheck(continuous_collision);
}

bool DynamicPathPlanner::setup_ompl_(std::shared_ptr<frapu::RobotEnvironment>& robot_environment,
                                     bool& verbose)
{
    if (!verbose_) {
        ompl::msg::noOutputHandler();
    }
    bool continuous_collision = true;
    motionValidator_ = ompl::base::MotionValidatorPtr(new frapu::MotionValidator(space_information_,
                       continuous_collision,
                       true));

    static_cast<frapu::MotionValidator*>(motionValidator_.get())->setRobotEnvironment(robot_environment);

    state_space_bounds_ = ompl::base::RealVectorBounds(state_space_dimension_);
    space_information_->setMotionValidator(motionValidator_);
    space_information_->setMinMaxControlDuration(1, 1);
    space_information_->setPropagationStepSize(robot_environment->getControlDuration());

    problem_definition_ = ompl::base::ProblemDefinitionPtr(new ompl::base::ProblemDefinition(space_information_));
    if (planner_str_ == "EST") {
        planner_ = ompl::base::PlannerPtr(new frapu::ESTControl(space_information_));
    } else {
        planner_ = ompl::base::PlannerPtr(new frapu::RRTControl(space_information_));
    }

    planner_->setProblemDefinition(problem_definition_);

    state_propagator_ = ompl::control::StatePropagatorPtr(new frapu::StatePropagator(space_information_,
                        robot_environment,
                        verbose));
    space_information_->setStatePropagator(state_propagator_);

    // Set the bounds
    ompl::base::RealVectorBounds control_bounds(control_space_dimension_);

    std::vector<double> lowerStateLimits;
    std::vector<double> upperStateLimits;
    std::vector<double> lowerControlLimits;
    std::vector<double> upperControlLimits;

    std::shared_ptr<frapu::Robot> robot = robot_environment->getRobot();

    frapu::StateLimitsSharedPtr stateLimits = robot->getStateSpace()->getStateLimits();
    static_cast<frapu::VectorStateLimits*>(stateLimits.get())->getVectorLimits(lowerStateLimits, upperStateLimits);

    frapu::ActionLimitsSharedPtr actionLimits =
        robot->getActionSpace()->getActionLimits();
    static_cast<frapu::VectorActionLimits*>(actionLimits.get())->getRawLimits(lowerControlLimits, upperControlLimits);

    for (size_t i = 0; i < lowerStateLimits.size(); i++) {
        state_space_bounds_.setLow(i, lowerStateLimits[i]);
        state_space_bounds_.setHigh(i, upperStateLimits[i]);
    }

    for (size_t i = 0; i < lowerControlLimits.size(); i++) {
        control_bounds.setLow(i, lowerControlLimits[i]);
        control_bounds.setHigh(i, upperControlLimits[i]);
    }

    state_space_->as<ompl::base::RealVectorStateSpace>()->setBounds(state_space_bounds_);
    control_space_->as<ompl::control::RealVectorControlSpace>()->setBounds(control_bounds);
    return true;
}

bool DynamicPathPlanner::isValid(const ompl::base::State* state)
{
    std::vector<double> state_vec;
    for (unsigned int i = 0; i < space_information_->getStateSpace()->getDimension(); i++) {
        state_vec.push_back(state->as<ompl::base::RealVectorStateSpace::StateType>()->values[i]);
    }
    all_states_.push_back(state_vec);
    if (static_cast<MotionValidator&>(*motionValidator_).isValid(state_vec)) {
        accepted_ = accepted_ + 1.0;
        return true;
    } else {
        rejected_ = rejected_ + 1.0;
        return false;
    }
}

void DynamicPathPlanner::getAllStates(std::vector<std::vector<double>>& all_states)
{
    for (auto & k : all_states_) {
        all_states.push_back(k);
    }
}

bool DynamicPathPlanner::isValidPy(std::vector<double>& state)
{
    bool valid = static_cast<MotionValidator&>(*motionValidator_).isValid(state);
    return valid;
}

bool DynamicPathPlanner::solve_(double time_limit)
{
    bool solved = false;
    bool hasExactSolution = false;
    bool timeoutReached = false;
    boost::timer t0;
    while (!solved && !hasExactSolution && !timeoutReached) {
        solved = planner_->solve(time_limit);
        // Get all the solutions
        std::vector<ompl::base::PlannerSolution> solutions = problem_definition_->getSolutions();
        for (size_t i = 0; i < solutions.size(); i++) {
            if (!solutions[i].approximate_) {
                hasExactSolution = true;
                break;
            }
        }
        if (t0.elapsed() > time_limit) {
            return false;
        }
        // Check if there's an exact solution
    }
    return hasExactSolution;
}

/**void DynamicPathPlanner::setGoalStates(std::vector<std::vector<double>> &goal_states,
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

void DynamicPathPlanner::setGoal(ompl::base::GoalPtr& goal_region)
{
    goal_region_ = goal_region;
}

TrajectorySharedPtr DynamicPathPlanner::solve(const RobotStateSharedPtr& robotState,
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

VectorTrajectory DynamicPathPlanner::solve(const std::vector<double>& start_state_vec,
        double timeout)
{
    // Set the start and goal state
    ompl::base::ScopedState<> start_state(state_space_);
    VectorTrajectory vectorTrajectory;
    for (unsigned int i = 0; i < state_space_dimension_; i++) {
        start_state[i] = start_state_vec[i];
    }

    if (!static_cast<MotionValidator&>(*motionValidator_).isValid(start_state_vec)) {
        cout << "DynamicPathPlanner: ERROR: Start state is not valid!" << endl;
        return vectorTrajectory;
    }

    problem_definition_->addStartState(start_state);
    problem_definition_->setGoal(goal_region_);
    planner_->setup();
    bool solved = false;
    boost::timer t;
    solved = solve_(timeout);
    if (solved) {
        ompl::base::PlannerSolution planner_solution(problem_definition_->getSolutionPath());
        ompl::base::PathPtr solution_path_ = planner_solution.path_;	
        std::vector<ompl::base::State*> solution_states_(static_cast<ompl::control::PathControl*>(solution_path_.get())->getStates());
        std::vector<ompl::control::Control*> solution_controls_(static_cast<ompl::control::PathControl*>(solution_path_.get())->getControls());
        std::vector<double> control_durations(static_cast<ompl::control::PathControl*>(solution_path_.get())->getControlDurations());        
        for (size_t i = 0; i < solution_states_.size(); i++) {            
            std::vector<double> state;
            std::vector<double> control;	   
            std::vector<double> observation;	    
            for (size_t j = 0; j < state_space_dimension_; j++) {		
                state.push_back(solution_states_[i]->as<ompl::base::RealVectorStateSpace::StateType>()->values[j]);
		observation.push_back(solution_states_[i]->as<ompl::base::RealVectorStateSpace::StateType>()->values[j]);
            }
            
            vectorTrajectory.states.push_back(state);
	    vectorTrajectory.observations.push_back(observation);	    
            
            for (size_t j = 0; j < state_space_dimension_ / 2; j++) {
                if (i < solution_states_.size() - 1) {
		    control.push_back(solution_controls_[i]->as<ompl::control::RealVectorControlSpace::ControlType>()->values[j]);
                } else {
                    control.push_back(0.0);
                }
            } 
            
            vectorTrajectory.controls.push_back(control);
            if (i < solution_states_.size() - 1) {
		vectorTrajectory.durations.push_back(control_durations[i]);                
            } else {
                vectorTrajectory.durations.push_back(0.0);
            }
        }        
    }

    return vectorTrajectory;
}

}
