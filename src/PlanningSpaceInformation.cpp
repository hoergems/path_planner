#include "include/PlanningSpaceInformation.hpp"

using std::cout;
using std::endl;

namespace frapu
{

PlanningSpaceInformation::PlanningSpaceInformation(const ompl::base::StateSpacePtr& stateSpace,
        const ompl::control::ControlSpacePtr& controlSpace):
    ompl::control::SpaceInformation(stateSpace, controlSpace),
    num_control_samples_(1)
{

}

unsigned int PlanningSpaceInformation::propagateWhileValid(const ompl::base::State* state,
        const ompl::control::Control* control,
        int steps,
        std::vector<ompl::base::State*>& result,
        bool alloc) const
{
    double signedStepSize = steps > 0 ? stepSize_ : -stepSize_;
    steps = abs(steps);

    if (alloc)
        result.resize(steps);
    else {
        if (result.empty())
            return 0;
        steps = std::min(steps, (int)result.size());
    }

    int st = 0;

    if (st < steps) {
        if (alloc)
            result[st] = allocState();
        statePropagator_->propagate(state, control, signedStepSize, result[st]);
        if (checkMotion(state, result[st])) {
            ++st;
            while (st < steps) {
                if (alloc)
                    result[st] = allocState();
                statePropagator_->propagate(result[st - 1], control, signedStepSize, result[st]);
                if (!checkMotion(result[st - 1], result[st])) {
                    if (alloc) {
                        freeState(result[st]);
                        result.resize(st);
                    }
                    break;
                } else
                    ++st;
            }
        } else {
            if (alloc) {
                freeState(result[st]);
                result.resize(st);
            }
        }
    }

    return st;
}

ompl::control::DirectedControlSamplerPtr PlanningSpaceInformation::allocDirectedControlSampler() const
{
    ompl::control::DirectedControlSamplerPtr ptr(new DirectedControlSampler(this, num_control_samples_));
    static_cast<DirectedControlSampler*>(ptr.get())->setNumControlSamples(num_control_samples_);
    return ptr;
}

void PlanningSpaceInformation::setNumControlSamples(unsigned int& num_control_samples)
{
    num_control_samples_ = num_control_samples;
}

}
