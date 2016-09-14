#ifndef M_CONTROL_SPACE_HPP_
#define M_CONTROL_SPACE_HPP_
#include <iostream>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/control/ControlSampler.h>
#include <ompl/base/StateSpace.h>
#include <ompl/util/RandomNumbers.h>
#include "control_sampler.hpp"

namespace frapu
{

class ControlSpace : public ompl::control::RealVectorControlSpace
{
public:
    ControlSpace(const ompl::base::StateSpacePtr& stateSpace,
		 frapu::ActionSpaceSharedPtr &actionSpace,
		 unsigned int dim);

    ompl::control::ControlSamplerPtr allocDefaultControlSampler() const;

    void setControlSampler(std::string control_sampler);

private:
    frapu::ActionSpaceSharedPtr actionSpace_;
    
    unsigned int dim_;

    ompl::RNG rng_;

    std::string control_sampler_;
};

}

#endif
