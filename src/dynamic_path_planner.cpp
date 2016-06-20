#include "include/dynamic_path_planner.hpp"
#include <random>
#include <fstream>
#include <iterator>
#include <stdio.h>

using std::cout;
using std::endl;

namespace shared {

ompl::base::GoalPtr makeManipulatorGoalRegion(const ompl::base::SpaceInformationPtr si,
		                                                        std::shared_ptr<shared::RobotEnvironment> &robot_environment,
		                                                        std::vector<std::vector<double>> goal_states,
		                                                        std::vector<double> ee_goal_position,
		                                                        double ee_goal_threshold) {
	if (goal_states.size() == 0) {
		cout << "No goal states provided. In order to generate a Manipulator goal area, at least one goal state is required." << endl;
		return nullptr;				
	}
	ompl::base::GoalPtr manip_goal_region(new ManipulatorGoalRegion(si, robot_environment, goal_states, ee_goal_position, ee_goal_threshold));
			
	if (!manip_goal_region) {
		cout << "NULLLLPTR!!!!!!!!" << endl;
	}
	return manip_goal_region;
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
	min_max_control_duration_(std::vector<int>({1, 4})),
	add_intermediate_states_(true)
{
    
}

DynamicPathPlanner::DynamicPathPlanner(std::shared_ptr<DynamicPathPlanner> &dynamic_path_planner,
		                               std::shared_ptr<shared::RobotEnvironment> &robot_environment):
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
    min_max_control_duration_(std::vector<int>({1, 4})),
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

ompl::base::GoalPtr DynamicPathPlanner::getGoalRegion() const {
	return goal_region_;
}

ompl::base::SpaceInformationPtr DynamicPathPlanner::getSpaceInformation() {
	assert(space_information_ && "DynamicPathPlanner: Fatal error: space_information not initialized. Did you call setup()?");
	return space_information_;
}

/**void DynamicPathPlanner::setupMotionValidator(std::shared_ptr<shared::RobotEnvironment> &robot_environment,
		                                      bool continuous_collision) {	
	motionValidator_ = boost::make_shared<MotionValidator>(space_information_,			                                               
														   continuous_collision,
														   true);
	static_cast<shared::MotionValidator *>(motionValidator_.get())->setRobotEnvironment(robot_environment);
}*/

void DynamicPathPlanner::setRRTGoalBias(double goal_bias) {
	rrt_goal_bias_ = goal_bias;
	if (planner_str_ == "EST") {
		static_cast<ESTControl *>(planner_.get())->setGoalBias(goal_bias);
	}
	else {
		static_cast<RRTControl *>(planner_.get())->setGoalBias(goal_bias);
	}
	
}

void DynamicPathPlanner::log_(std::string msg, bool warn=false) {
	if (warn) {
		cout << "DynamicPathPlanner: " << msg << endl;
	}
	else if (verbose_) {
		cout << "DynamicPathPlanner: " << msg << endl;
	}
}

bool DynamicPathPlanner::setup(std::shared_ptr<shared::RobotEnvironment> &robot_environment,
							   std::string planner) {
	state_space_dimension_ = robot_environment->getRobot()->getStateSpaceDimension();
	control_space_dimension_ = robot_environment->getRobot()->getControlSpaceDimension();
	
	state_space_ = ompl::base::StateSpacePtr(new ompl::base::RealVectorStateSpace(state_space_dimension_));
	//state_space_ = boost::make_shared<ompl::base::RealVectorStateSpace>(state_space_dimension_);
	//state_space_(new ompl::base::RealVectorStateSpace(state_space_dimension_))
	control_space_ = ompl::control::ControlSpacePtr(new shared::ControlSpace(state_space_, control_space_dimension_));
	//control_space_ = boost::make_shared<ControlSpace>(state_space_, control_space_dimension_);
	//control_space_(new ControlSpace(state_space_, control_space_dimension_))
	space_information_ = ompl::control::SpaceInformationPtr(new ompl::control::SpaceInformation(state_space_, control_space_));
	//space_information_ = boost::make_shared<ompl::control::SpaceInformation>(state_space_, control_space_);
	//space_information_(new PlanningSpaceInformation(state_space_, control_space_))
	
	planner_str_ = planner;
	/***** Setup OMPL *****/
	log_("Setting up OMPL");
	setup_ompl_(robot_environment, verbose_);
	log_("OMPL setup");
	log_("Setup complete");
	return true;
}

ompl::control::ControlSamplerPtr DynamicPathPlanner::allocUniformControlSampler_(const ompl::control::ControlSpace *control_space) {	
	return nullptr;
    return ompl::control::ControlSamplerPtr(new UniformControlSampler(control_space));
}

void DynamicPathPlanner::setNumControlSamples(unsigned int num_control_samples) {	
	num_control_samples_ = num_control_samples;
	static_cast<shared::PlanningSpaceInformation *>(space_information_.get())->setNumControlSamples(num_control_samples);
	//boost::static_pointer_cast<PlanningSpaceInformation>(space_information_)->setNumControlSamples(num_control_samples);	
}

void DynamicPathPlanner::setControlSampler(std::string control_sampler) {
	control_sampler_ = control_sampler;
	static_cast<shared::ControlSpace *>(control_space_.get())->setControlSampler(control_sampler);
	//boost::static_pointer_cast<shared::ControlSpace>(control_space_)->setControlSampler(control_sampler);
	//control_space_->setControlSampler(control_sampler);
}

void DynamicPathPlanner::setMinMaxControlDuration(std::vector<int> &min_max_control_duration) {
	min_max_control_duration_ = min_max_control_duration;
	unsigned int min = (unsigned int)min_max_control_duration[0];
	unsigned int max = (unsigned int)min_max_control_duration[1];
	space_information_->setMinMaxControlDuration(min, max);	
}

void DynamicPathPlanner::addIntermediateStates(bool add_intermediate_states) {
	add_intermediate_states_ = add_intermediate_states;
	if (planner_str_ == "RRT") {
		static_cast<RRTControl *>(planner_.get())->setIntermediateStates(add_intermediate_states);
	}	
	else if (planner_str_ == "EST") {
		static_cast<ESTControl *>(planner_.get())->setIntermediateStates(true);
	    //boost::static_pointer_cast<ESTControl>(planner_)->setIntermediateStates(true);
	}
}

bool DynamicPathPlanner::setup_ompl_(std::shared_ptr<shared::RobotEnvironment> &robot_environment,		                             
		                             bool &verbose) {
	if (!verbose_) {        
	    ompl::msg::noOutputHandler();
	}
	bool continuous_collision = true;
	motionValidator_ = ompl::base::MotionValidatorPtr(new shared::MotionValidator(space_information_,			                                               
			                                                                      continuous_collision,
			                                                                      true));
	
	static_cast<shared::MotionValidator *>(motionValidator_.get())->setRobotEnvironment(robot_environment);
	
    state_space_bounds_ = ompl::base::RealVectorBounds(state_space_dimension_);    
    //space_information_->setStateValidityChecker(boost::bind(&DynamicPathPlanner::isValid, this, _1));
    space_information_->setMotionValidator(motionValidator_);
    space_information_->setMinMaxControlDuration(1, 1);
    space_information_->setPropagationStepSize(robot_environment->getControlDuration());
     
    problem_definition_ = ompl::base::ProblemDefinitionPtr(new ompl::base::ProblemDefinition(space_information_));
    
    //planner_ = boost::make_shared<ompl::control::RRT>(space_information_);
    //
    if (planner_str_ == "EST") {
    	planner_ = ompl::base::PlannerPtr(new shared::ESTControl(space_information_));    	
    }
    else {
    	planner_ = ompl::base::PlannerPtr(new shared::RRTControl(space_information_));    	
    }
    
    planner_->setProblemDefinition(problem_definition_);    
    
    state_propagator_ = ompl::control::StatePropagatorPtr(new shared::StatePropagator(space_information_,
                                                                                      robot_environment,                                                            
                                                                                      verbose));
    /**state_propagator_ = boost::make_shared<StatePropagator>(space_information_,
    		                                                robot_environment,                                                            
                                                            verbose);*/    
    space_information_->setStatePropagator(state_propagator_);
    
    // Set the bounds    
    ompl::base::RealVectorBounds control_bounds(control_space_dimension_);
    
    std::vector<double> lowerStateLimits;
    std::vector<double> upperStateLimits;
    std::vector<double> lowerControlLimits;
    std::vector<double> upperControlLimits;
    
    robot_environment->getRobot()->getStateLimits(lowerStateLimits, upperStateLimits);
    robot_environment->getRobot()->getControlLimits(lowerControlLimits, upperControlLimits);
    
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

bool DynamicPathPlanner::isValid(const ompl::base::State *state) {	
	std::vector<double> state_vec;
	for (unsigned int i = 0; i < space_information_->getStateSpace()->getDimension(); i++) {
	    state_vec.push_back(state->as<ompl::base::RealVectorStateSpace::StateType>()->values[i]);        
	} 
	all_states_.push_back(state_vec);
    if (static_cast<MotionValidator &>(*motionValidator_).isValid(state_vec)) {
    	accepted_ = accepted_ + 1.0;
    	return true;
    }
    else {
    	rejected_ = rejected_ + 1.0;
    	return false;
    }
    /**if (valid) {
    	accepted_ = accepted_ + 1.0;
    }
    else {
    	cout << "not valid: ";
    	for (unsigned int i = 0; i < space_information_->getStateSpace()->getDimension(); i++) {
    		cout << state->as<ompl::base::RealVectorStateSpace::StateType>()->values[i] << ", ";
    	}
    	cout << endl;
    	rejected_ = rejected_ + 1.0;
    	return false;
    }
    
    all_states_.push_back(state_vec);
    return static_cast<MotionValidator &>(*motionValidator_).isValid(state_vec); */   
}

void DynamicPathPlanner::getAllStates(std::vector<std::vector<double>> &all_states) {	
	for (auto &k: all_states_) {
		all_states.push_back(k);		
	}
}

bool DynamicPathPlanner::isValidPy(std::vector<double> &state) {
    bool valid = static_cast<MotionValidator &>(*motionValidator_).isValid(state);
    return valid;    
}

bool DynamicPathPlanner::solve_(double time_limit) {
    bool solved = false;
    bool hasExactSolution = false;    
    while (!solved && !hasExactSolution) {
        solved = planner_->solve(time_limit);
        
        // Get all the solutions
        std::vector<ompl::base::PlannerSolution> solutions = problem_definition_->getSolutions();
        for (size_t i = 0; i < solutions.size(); i++) {
            if (!solutions[i].approximate_) {
                hasExactSolution = true;                
                break;
            }
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

void DynamicPathPlanner::setGoal(ompl::base::GoalPtr &goal_region) {
	goal_region_ = goal_region;
}

std::vector<std::vector<double>> DynamicPathPlanner::solve(const std::vector<double> &start_state_vec,
		                                                   double timeout) {	
    // Set the start and goal state	
    ompl::base::ScopedState<> start_state(state_space_);
    std::vector<std::vector<double>> solution_vector; 
    for (unsigned int i = 0; i < state_space_dimension_; i++) {
        start_state[i] = start_state_vec[i];        
    }
    
    if (!static_cast<MotionValidator &>(*motionValidator_).isValid(start_state_vec)) {
    	cout << "DynamicPathPlanner: ERROR: Start state is not valid!" << endl;
    	return solution_vector;
    }

    /**ompl::base::GoalPtr gp(new ManipulatorGoalRegion(space_information_,
    		                                         robot_,
    		                                         goal_states_, 
    		                                         ee_goal_position_, 
    		                                         ee_goal_threshold_,
    		                                         true));
    boost::static_pointer_cast<ManipulatorGoalRegion>(gp)->setThreshold(ee_goal_threshold_);*/
    
    problem_definition_->addStartState(start_state);    
    problem_definition_->setGoal(goal_region_);
    
    //planner_->setGoalBias(0.1);
    planner_->setup();
    bool solved = false;
    
    boost::timer t;
    solved = solve_(timeout);    
    
    if (solved) {
        ompl::base::PlannerSolution planner_solution(problem_definition_->getSolutionPath());                
        ompl::base::PathPtr solution_path_ = planner_solution.path_;               
        //cout << "Length of solution path " << solution_path_->length() << endl << endl;
        std::vector<ompl::base::State*> solution_states_(static_cast<ompl::control::PathControl *>(solution_path_.get())->getStates());
        std::vector<ompl::control::Control*> solution_controls_(static_cast<ompl::control::PathControl *>(solution_path_.get())->getControls());
        std::vector<double> control_durations(static_cast<ompl::control::PathControl *>(solution_path_.get())->getControlDurations());
        /**cout << "durations: ";
        for (auto &k: control_durations) {
        	cout << k << ", ";
        }
        cout << endl;*/
        for (size_t i = 0; i < solution_states_.size(); i++) {
            //cout << "State: ";
            std::vector<double> solution_state;
            for (size_t j = 0; j < state_space_dimension_; j++) {
            	solution_state.push_back(solution_states_[i]->as<ompl::base::RealVectorStateSpace::StateType>()->values[j]);
                //cout << solution_states_[i]->as<ompl::base::RealVectorStateSpace::StateType>()->values[j] << ", ";
            }
            //cout << endl;
            
            
            //cout << "Control: ";
            for (size_t j = 0; j < state_space_dimension_ / 2; j++) {
            	if (i < solution_states_.size() - 1) {
                    solution_state.push_back(solution_controls_[i]->as<ompl::control::RealVectorControlSpace::ControlType>()->values[j]);
                    //cout << solution_controls_[i]->as<ompl::control::RealVectorControlSpace::ControlType>()->values[j] << ", ";
            	}
            	else {            		
            		solution_state.push_back(0.0);            		
            	}
            }
            
            //cout << "Duration: " << control_durations[i] << endl;
            /**for (size_t j = 0; j < state_space_dimension_ / 2; j++) { 
            	solution_state.push_back(0.0);
            }*/
            
            //cout << endl;
            
            for (size_t j = 0; j < state_space_dimension_; j++) {
                solution_state.push_back(solution_states_[i]->as<ompl::base::RealVectorStateSpace::StateType>()->values[j]);                
            }
            
            if (i < solution_states_.size() - 1) { 
            	solution_state.push_back(control_durations[i]);
            }
            else {
            	solution_state.push_back(0.0);
            }
            
            solution_vector.push_back(solution_state);          
        }
        //cout << "Solution found in " << t.elapsed() << "seconds" << endl;
        //cout << "accepted " << accepted_ << endl;
        //cout << "rejected " << rejected_ << endl;        
        //return solution_path_; 
    }
    
    return solution_vector;
}
 
}
