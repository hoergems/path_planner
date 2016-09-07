#include "include/state_propagator.hpp"

using std::cout;
using std::endl;

namespace frapu
{

StatePropagator::StatePropagator(const ompl::control::SpaceInformationPtr& si,
                                 frapu::SceneSharedPtr& scene,
                                 frapu::RobotSharedPtr& robot,
                                 bool& verbose):
    ompl::control::StatePropagator(si),
    space_information_(si),
    model_setup_(false),
    scene_(scene),
    robot_(robot),
    simulation_step_size_(0.0),
    verbose_(verbose)
{

}

void StatePropagator::propagate(const ompl::base::State* state,
                                const ompl::control::Control* control,
                                const double duration,
                                ompl::base::State* result) const
{
    unsigned int dim = space_information_->getStateSpace()->getDimension();
    unsigned int control_dim = robot_->getActionSpace()->getNumDimensions();
    std::vector<double> current_vel;
    if (verbose_) {
        cout << "State: ";
        for (unsigned int i = 0; i < dim; i++) {
            cout << " " << state->as<ompl::base::RealVectorStateSpace::StateType>()->values[i];
        }
        cout << endl;

        cout << "Torques: ";
        for (unsigned int i = 0; i < control_dim; i++) {
            cout << " " << control->as<ompl::control::RealVectorControlSpace::ControlType>()->values[i];
        }
        cout << endl;
    }

    std::vector<double> currentStateVec;
    std::vector<double> controlErrorVec;
    std::vector<double> actionVec;
    for (unsigned int i = 0; i < dim; i++) {
        currentStateVec.push_back(state->as<ompl::base::RealVectorStateSpace::StateType>()->values[i]);
    }

    for (unsigned int i = 0; i < control_dim; i++) {
        actionVec.push_back(control->as<ompl::control::RealVectorControlSpace::ControlType>()->values[i]);
        controlErrorVec.push_back(0.0);
    }

    frapu::RobotStateSharedPtr currentState = std::make_shared<frapu::VectorState>(currentStateVec);
    frapu::ActionSharedPtr action = std::make_shared<frapu::VectorAction>(actionVec);
    frapu::RobotStateSharedPtr resultingState;

    double dur = duration;
    double sss = simulation_step_size_;

    robot_->propagateState(currentState,
                           action,
                           controlErrorVec,
                           dur,
                           sss,
                           resultingState);

    std::vector<double> resultVec = static_cast<frapu::VectorState*>(resultingState.get())->asVector();

    if (verbose_) {
        cout << "Propagation result: ";
        for (size_t i = 0; i < resultVec.size(); i++) {
            cout << resultVec[i] << ", ";
        }
        cout << endl << endl;
        sleep(1);
    }

    for (unsigned int i = 0; i < dim; i++) {
        result->as<ompl::base::RealVectorStateSpace::StateType>()->values[i] = resultVec[i];
    }
}

void StatePropagator::setSimulationStepSize(double &simulationStepSize) {
    simulation_step_size_ = simulationStepSize;
}

bool StatePropagator::canPropagateBackward() const
{
    return false;
}

bool StatePropagator::steer(const ompl::base::State* /*from*/,
                            const ompl::base::State* /*to*/,
                            ompl::control::Control* /*result*/,
                            double& /*duration*/) const
{
    return false;
}

bool StatePropagator::canSteer() const
{
    return false;
}

}
