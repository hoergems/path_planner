#ifndef _DIRECTED_CONTROL_SAMPLER_HPP_
#define _DIRECTED_CONTROL_SAMPLER_HPP_
#include <ompl/control/SimpleDirectedControlSampler.h>
#include <ompl/control/ControlSampler.h>
#include <ompl/control/SpaceInformation.h>

namespace frapu {
    class DirectedControlSampler: public ompl::control::SimpleDirectedControlSampler {
    	public:
    	    DirectedControlSampler(const ompl::control::SpaceInformation *si, unsigned int k);
    	    
    	    unsigned int sampleTo(ompl::control::Control *control,
    	    		              const ompl::base::State *source,
    	    		              ompl::base::State *dest);
    	    
    	    unsigned int sampleTo(ompl::control::Control *control,
    	    		              const ompl::control::Control *previous,
    	    		              const ompl::base::State *source,
    	    		              ompl::base::State *dest);
    	    
    	private:
    	    ompl::control::ControlSamplerPtr cs_;
    	    
    	    unsigned int numControlSamples_;
    
    };
}

#endif