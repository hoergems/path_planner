#include "include/path_planner.hpp"
#include "include/dynamic_path_planner.hpp"

namespace shared {

BOOST_PYTHON_MODULE(libpath_planner) {
    using namespace boost::python;  
    
    
    
    class_<PathPlanner>("PathPlanner", init<double,
                                            bool,                                            
                                            double,
                                            double,
                                            bool,                                            
                                            bool>())
                        .def("solve", &PathPlanner::solve)                        
                        .def("getSpaceInformation", &PathPlanner::getSpaceInformation)
                        .def("setGoal", &PathPlanner::setGoal)
                        .def("isValid", &PathPlanner::isValidPy) 
                        .def("setPlanningDimensions", &PathPlanner::setPlanningDimensions)
                        .def("setup", &PathPlanner::setup)
                        .def("setupPlanner", &PathPlanner::setupPlanner)
    ;
    
    class_<ompl::base::SpaceInformation, ompl::base::SpaceInformationPtr, boost::noncopyable>("SpaceInformation", no_init);    
    //register_ptr_to_python<ompl::base::SpaceInformationPtr>();
    class_<GoalRegion, std::shared_ptr<GoalRegion>, boost::noncopyable>("GoalRegion", no_init);
    
    class_<GoalRegionWrapper, boost::noncopyable>("GoalRegion", init<const ompl::base::SpaceInformationPtr &,
    		                                                         std::shared_ptr<shared::RobotEnvironment>&,
    		                                                         std::vector<std::vector<double>>&>())        		
    ;
        
    class_<shared::RobotGoalRegion, boost::shared_ptr<RobotGoalRegion>, boost::noncopyable>("RobotGoalRegion", 
    		init<const ompl::base::SpaceInformationPtr&,    		                                                                     
    		     std::shared_ptr<shared::RobotEnvironment>&,
    		     std::vector<std::vector<double>> &>())
                                             						
    ;
    
    
       class_<std::vector<int> > ("v_int")
                .def(vector_indexing_suite<std::vector<int> >());
        
       
        class_<DynamicPathPlanner>("DynamicPathPlanner", init<bool>())
    							   .def("solve", &DynamicPathPlanner::solve)		                       
    							   .def("setGoal", &DynamicPathPlanner::setGoal)
    							   .def("isValid", &DynamicPathPlanner::isValidPy)
    							   .def("setup", &DynamicPathPlanner::setup)
    							   //.def("setupMotionValidator", &DynamicPathPlanner::setupMotionValidator)
    							   .def("getAllStates", &DynamicPathPlanner::getAllStates)
    							   .def("setNumControlSamples", &DynamicPathPlanner::setNumControlSamples)
    							   .def("setMinMaxControlDuration", &DynamicPathPlanner::setMinMaxControlDuration)
    							   .def("addIntermediateStates", &DynamicPathPlanner::addIntermediateStates)
    							   .def("setRRTGoalBias", &DynamicPathPlanner::setRRTGoalBias)
    							   .def("setControlSampler", &DynamicPathPlanner::setControlSampler)
    							   .def("getSpaceInformation", &DynamicPathPlanner::getSpaceInformation)
        ;
        
        def("makeRobotGoalRegion", &makeRobotGoalRegion);
    
}

}