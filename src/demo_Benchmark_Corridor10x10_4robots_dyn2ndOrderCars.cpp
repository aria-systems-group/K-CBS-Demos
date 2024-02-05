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
#include "StateSpaceDatabase.h"
#include "ControlSpaceDatabase.h"
#include "StateValidityCheckerDatabase.h"
#include "StatePropagatorDatabase.h"
#include "GoalRegionDatabase.h"
#include "PlannerAllocatorDatabase.h"
#include "SystemMergerDatabase.h"
#include "Benchmark.h"

#include <ompl/multirobot/control/planners/kcbs/KCBS.h>
#include <ompl/multirobot/control/planners/pp/PP.h>
#include <map>
#include <math.h>
#include <tuple>

namespace omrb = ompl::multirobot::base;
namespace omrc = ompl::multirobot::control;
namespace ob = ompl::base;
namespace oc = ompl::control;

void benchmark(const std::string plannerName)
{
    // provide start and goals for every robot
    const std::map<std::string, std::pair<double, double>> start_map{       {"Robot 1", {1.0, 0.5}}, 
                                                                            {"Robot 2", {1.0, 3.5}},
                                                                            {"Robot 3", {9.0, 0.5}},
                                                                            {"Robot 4", {9.0, 3.5}},
                                                                        };

    const std::map<std::string, std::pair<double, double>> goal_map{        {"Robot 1", {9.0, 0.5}}, 
                                                                            {"Robot 2", {9.0, 9.0}},
                                                                            {"Robot 3", {1.0, 0.5}}, 
                                                                            {"Robot 4", {1.0, 9.0}},
                                                                        };

    // provide obstacles
    std::set<Obstacle*> obs_set;
    auto obs1 = new RectangularObstacle(0.5, 2.0, 9.0, 0.5); // lower_left_x. lower_left_y, length, width
    auto obs2 = new RectangularObstacle(4.0, 5.0, 2.0, 2.0);
    obs_set.insert(obs1);
    obs_set.insert(obs2);

    // construct all of the robots
    std::unordered_map<std::string, Robot*> robot_map;
    for (auto itr = start_map.begin(); itr != start_map.end(); itr++)
    {
        Robot* robot = new RectangularRobot(itr->first, 1.0, 1.0);
        robot_map[itr->first] = robot;
    }

    // construct an instance of multi-robot space information
    auto ma_si(std::make_shared<omrc::SpaceInformation>());
    auto ma_pdef(std::make_shared<omrb::ProblemDefinition>(ma_si));

    // construct four individuals that operate in SE3
    for (auto itr = start_map.begin(); itr != start_map.end(); itr++) 
    {
        // construct the state space we are planning in
        auto space = createBounded2ndOrderCarStateSpace(10, 10);

        // name the state space parameter
        space->setName(itr->first);

        // create a control space
        auto cspace = createUniform2DRealVectorControlSpace(space);

        // construct an instance of  space information from this control space
        auto si(std::make_shared<oc::SpaceInformation>(space, cspace));

        // // set state validity checking for this space
        si->setStateValidityChecker(std::make_shared<homogeneous2ndOrderCarSystemSVC>(si, robot_map, obs_set));

        // set the state propagation routine
        auto odeSolver(std::make_shared<oc::ODEBasicSolver<>>(si, &SecondOrderCarODE));
        si->setStatePropagator(oc::ODESolver::getStatePropagator(odeSolver, &SecondOrderCarODEPostIntegration));

        // set the propagation step size
        si->setPropagationStepSize(0.1);

        // set this to remove the warning
        si->setMinMaxControlDuration(1, 10);

        // create a start state
        ob::ScopedState<> start(space);
        start[0] = start_map.at(itr->first).first;
        start[1] = start_map.at(itr->first).second;
        start[2] = 0.0;
        start[3] = 0.0;
        start[4] = 0.0;

        // // create a problem instance
        auto pdef(std::make_shared<ob::ProblemDefinition>(si));

        // set the start and goal states
        pdef->addStartState(start);
        pdef->setGoal(std::make_shared<GoalRegion2ndOrderCar>(si, goal_map.at(itr->first).first, goal_map.at(itr->first).second));

        // add the individual information to the multi-robot SpaceInformation and ProblemDefinition
        ma_si->addIndividual(si);
        ma_pdef->addIndividual(pdef);
    }

    // set the planner allocator for the multi-agent planner
    ompl::base::PlannerAllocator allocator = &allocateControlRRT;
    ma_si->setPlannerAllocator(allocator);

    // set the system merger
    omrc::SystemMergerPtr merger = std::make_shared<homogeneous2ndOrderCarSystemMerger>(ma_si, ma_pdef, robot_map, obs_set, 10, start_map, goal_map);
    ma_si->setSystemMerger(merger);

    // lock the multi-robot SpaceInformation and ProblemDefinitions when done adding individuals
    ma_si->lock();
    ma_pdef->lock();

    // instantiate the benchmark class and set the problem setting
    auto b = std::make_shared<Benchmark>();
    b->setSpaceInformation(ma_si);
    b->setPoroblemDefinition(ma_pdef);

    // set optional params
    b->setSolveTime(180); // optional -- default is 300 seconds
    b->setKCBSMergeBound(10); // optional -- default is std::max

    /* For benchmarking KCBS*/
    std::string results_string;
    if (plannerName == "K-CBS")
        results_string = "KCBS-benchmark-Corridor10x10-4robots";
    else
        results_string = "PP-benchmark-Corridor10x10-4robots";
    
    b->setFileName(results_string); // optional -- default is "Results"

    /* Perform a single planner run and save relevant data */
    if (plannerName == "K-CBS")
        b->runKCBS();
    else
        b->runPP();
}

int main(int argc, char ** argv)
{
    std::string plannerName = "K-CBS";
    // std::string plannerName = "PP";
    std::cout << "Planning for 4 2nd order cars inside a Corridor 10x10 workspace with " << plannerName << "." << std::endl;
    benchmark(plannerName);
}