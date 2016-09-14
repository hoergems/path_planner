#ifndef UNIFORM_CONTROL_SAMPLER_TEST_HPP_
#define UNIFORM_CONTROL_SAMPLER_TEST_HPP_
#include <ompl/control/ControlSampler.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/control/Control.h>

#include <ompl/base/State.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <iostream>
#include <robot_headers/ActionSpace.hpp>

namespace frapu
{

class UniformControlSampler : public ompl::control::ControlSampler
{
public:
    UniformControlSampler(const ompl::control::ControlSpace* space,
	                  const frapu::ActionSpaceSharedPtr &actionSpace);

    void sample(ompl::control::Control* control) override;

    void sample(ompl::control::Control* control, const ompl::base::State* state) override;

    void sampleNext(ompl::control::Control* control, const ompl::control::Control* previous, const ompl::base::State* state);

private:
    const ompl::control::ControlSpace* space_;
    
    frapu::ActionSpaceSharedPtr actionSpace_;

    std::vector<double> low_;

    std::vector<double> high_;

};

class DiscreteControlSampler: public ompl::control::ControlSampler
{
public:
    DiscreteControlSampler(const ompl::control::ControlSpace* space, 
			   const frapu::ActionSpaceSharedPtr &discreteActionSpace);

    void sample(ompl::control::Control* control) override;

    void sample(ompl::control::Control* control, const ompl::base::State* state) override;

    void sampleNext(ompl::control::Control* control, const ompl::control::Control* previous, const ompl::base::State* state);

private:
    const ompl::control::ControlSpace* space_;
    
    frapu::DiscreteVectorActionSpace *discreteActionSpace_;
    
    unsigned int numActions_;

    std::vector<double> low_;

    std::vector<double> high_;

};
}

#endif
