/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2010, Rice University
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Rice University nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Justin Kottinger */

#include "Robot.h"
#include "Obstacle.h"
#include <ompl/multirobot/control/SpaceInformation.h>
#include <unordered_map>

namespace omr = ompl::multirobot;
namespace omrb = ompl::multirobot::base;
namespace omrc = ompl::multirobot::control;


class homogeneous2ndOrderCarSystemMerger: public omrc::SystemMerger
{
public:
	homogeneous2ndOrderCarSystemMerger(const omrc::SpaceInformationPtr &si, 
		const omrb::ProblemDefinitionPtr pdef, std::unordered_map<std::string, Robot*> robots, std::set<Obstacle*> obstacles, const unsigned int space_bound,
		const std::map<std::string, std::pair<double, double>> start_map, const std::map<std::string, std::pair<double, double>> goal_map): 
			omrc::SystemMerger(si, pdef), robots_(robots), obstacles_(obstacles), bound_(space_bound), starts_(start_map), goals_(goal_map) {};

	std::pair<const omrc::SpaceInformationPtr, const omrb::ProblemDefinitionPtr> merge(const int index1, const int index2) const override
	{
		/* 	we assume that all robots are 2nd order cars -- need to combine them
		*	we also assume that we will place the meta-robot (from composing index1 and index2) at the end of the individual's list
		*	this is certainly not required, but it helps with book-keeping 
		*/

		// start with creating a new spaceInformation and ProblemDefinition -- these will be returned to the planner
		auto merged_si(std::make_shared<omrc::SpaceInformation>());
    	auto merged_pdef(std::make_shared<omrb::ProblemDefinition>(merged_si));

    	// get the names of index1 and index2
    	std::string robot1_name = si_->getIndividual(index1)->getStateSpace()->getName();
    	std::string robot2_name = si_->getIndividual(index2)->getStateSpace()->getName();

        // grab the individuals and create new name (used below)
        std::string new_name = robot1_name + " and " + robot2_name;
        Robot* robot1 = robots_.at(robot1_name);
        Robot* robot2 = robots_.at(robot2_name);

    	// fill the new robot map
        std::unordered_map<std::string, Robot*> new_robot_map;
        for (auto itr = robots_.begin(); itr != robots_.end(); itr++)
        {
            if ((itr->first != robot1_name) && (itr->first != robot2_name))
                new_robot_map[itr->first] = itr->second;
        }
        Robot* new_robot = new CompoundRobot(new_name, robot1, robot2);
        new_robot_map[new_name] = new_robot;

    	// lets start with the easy thing -- fill merged_si and merged_pdef with the individuals that remain the same
    	for (unsigned int idx = 0; idx < si_->getIndividualCount(); idx++)
    	{
    		if ((idx != index1) && (idx != index2))
    		{
    			// clear the constraints and existing path
    			si_->getIndividual(idx)->clearDynamicObstacles();
    			pdef_->getIndividual(idx)->clearSolutionPaths();
                // set the new state validity checker object with new robot set
                si_->getIndividual(idx)->setStateValidityChecker(std::make_shared<homogeneous2ndOrderCarSystemSVC>(si_->getIndividual(idx), new_robot_map, obstacles_));
    			// add individual back to the new definitions
    			merged_si->addIndividual(si_->getIndividual(idx));
        		merged_pdef->addIndividual(pdef_->getIndividual(idx));
    		}
    	}

    	// if we already composed index1 or index2, we will not merge
    	// theoretically one could do this, but this implementation does not currently support it
    	if ((si_->getIndividual(index1)->getStateSpace()->getDimension() != 5) || (si_->getIndividual(index2)->getStateSpace()->getDimension() != 5))
    		return {nullptr, nullptr};

    	/* 
    	*	now, we are ready to compose index1 and index2!
    	*/
    	// construct the state space we are planning in
        auto space = createBoundedTwo2ndOrderCarStateSpace(bound_, bound_);
        space->setName(new_name);

        // create a control space
        auto cspace = createUniform4DRealVectorControlSpace(space);

        // construct an instance of  space information from this control space
        auto si(std::make_shared<oc::SpaceInformation>(space, cspace));

        // set state validity checking for this space
        si->setStateValidityChecker(std::make_shared<homogeneousTwo2ndOrderCarSystemSVC>(si, robot1, robot2, robots_, obstacles_));

        // set the state propagation routine
        auto sp = std::make_shared<TwoSecondOrderCarStatePropagator>(si, goals_.at(robot1_name).first, goals_.at(robot1_name).second,
        															     goals_.at(robot2_name).first, goals_.at(robot2_name).second);
        si->setStatePropagator(sp);

        // set the propagation step size
        si->setPropagationStepSize(0.1);

        // set this to remove the warning
        si->setMinMaxControlDuration(1, 10);

        // create a start state
        ob::ScopedState<> start(space);
        start[0] = starts_.at(robot1_name).first;
        start[1] = starts_.at(robot1_name).second;
        start[2] = 0.0;
        start[3] = 0.0;
        start[4] = 0.0;
        start[5] = starts_.at(robot2_name).first;
        start[6] = starts_.at(robot2_name).second;
        start[7] = 0.0;
        start[8] = 0.0;
        start[9] = 0.0;

        // create a problem instance
        auto pdef(std::make_shared<ob::ProblemDefinition>(si));

        // set the start and goal states
        pdef->addStartState(start);
        const ob::GoalPtr two_robot_goal = std::make_shared<GoalRegionTwo2ndOrderCars>(si, goals_.at(robot1_name).first, goals_.at(robot1_name).second,
        																	  			   goals_.at(robot2_name).first, goals_.at(robot2_name).second);
        pdef->setGoal(two_robot_goal);

        // add the individual information to the multi-robot SpaceInformation and ProblemDefinition
        merged_si->addIndividual(si);
        merged_pdef->addIndividual(pdef);

        // done adding individuals, go ahead and lock
        merged_si->lock();
        merged_pdef->lock();

        // set the planner allocator for the multi-agent planner
    	ompl::base::PlannerAllocator allocator = &allocateControlRRT;
    	merged_si->setPlannerAllocator(allocator);

    	for (unsigned int idx = 0; idx < merged_si->getIndividualCount(); idx++)
    	{
    		std::cout << merged_si->getIndividual(idx)->getStateSpace()->getName() << std::endl;
    	}

		return {merged_si, merged_pdef};
	}

private:
	std::unordered_map<std::string, Robot*> robots_;
	std::set<Obstacle*> obstacles_;
	const unsigned int bound_;
	const std::map<std::string, std::pair<double, double>> starts_;
	const std::map<std::string, std::pair<double, double>> goals_;
};

