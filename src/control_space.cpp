#include "include/control_space.hpp"

using std::cout;
using std::endl;

namespace frapu
{

ControlSpace::ControlSpace(const ompl::base::StateSpacePtr& stateSpace,
			   frapu::ActionSpaceSharedPtr &actionSpace,
			   unsigned int dim):
    ompl::control::RealVectorControlSpace(stateSpace, dim),
    actionSpace_(actionSpace),
    control_sampler_("continuous")
{

}

void ControlSpace::setControlSampler(std::string control_sampler)
{
    control_sampler_ = control_sampler;
}

ompl::control::ControlSamplerPtr ControlSpace::allocDefaultControlSampler() const
{
    if (control_sampler_ == "discrete") {	
        return ompl::control::ControlSamplerPtr(new DiscreteControlSampler(this, actionSpace_));
    }

    return ompl::control::ControlSamplerPtr(new UniformControlSampler(this, actionSpace_));
}

}
