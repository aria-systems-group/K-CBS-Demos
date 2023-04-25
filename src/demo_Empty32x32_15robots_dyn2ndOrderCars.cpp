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

#include <ompl/multirobot/control/planners/kcbs/KCBS.h>
#include <unordered_map>
#include <math.h>

namespace omrb = ompl::multirobot::base;
namespace omrc = ompl::multirobot::control;
namespace ob = ompl::base;
namespace oc = ompl::control;

void plan()
{
    // provide start and goals for every robot
    const std::unordered_map<std::string, std::pair<int, int>> start_map{   {"Robot 0", {11, 6}}, 
                                                                            {"Robot 1", {29, 9}}, 
                                                                            {"Robot 2", {9, 1}},
                                                                            {"Robot 3", {11, 16}},
                                                                            {"Robot 4", {3, 26}},
                                                                            {"Robot 5", {30, 28}},
                                                                            {"Robot 6", {2, 12}},
                                                                            {"Robot 7", {18, 12}},
                                                                            {"Robot 8", {19, 21}},
                                                                            {"Robot 9", {10, 22}},
                                                                            {"Robot 10", {28, 20}},
                                                                            {"Robot 11", {24, 12}},
                                                                            {"Robot 12", {17, 30}},
                                                                            {"Robot 13", {14, 2}},
                                                                            {"Robot 14", {4, 30}},
                                                                        };

    const std::unordered_map<std::string, std::pair<int, int>> goal_map{    {"Robot 0", {7, 18}}, 
                                                                            {"Robot 1", {3, 5}}, 
                                                                            {"Robot 2", {13, 21}},
                                                                            {"Robot 3", {26, 15}},
                                                                            {"Robot 4", {24, 26}},
                                                                            {"Robot 5", {22, 18}},
                                                                            {"Robot 6", {14, 30}},
                                                                            {"Robot 7", {27, 3}},
                                                                            {"Robot 8", {30, 22}},
                                                                            {"Robot 9", {18, 4}},
                                                                            {"Robot 10", {28, 8}},
                                                                            {"Robot 11", {4, 12}},
                                                                            {"Robot 12", {31, 26}},
                                                                            {"Robot 13", {31, 5}},
                                                                            {"Robot 14", {2, 20}},
                                                                        };

    // construct all of the robots
    std::unordered_map<std::string, Robot*> robot_map;
    for (auto itr = start_map.begin(); itr != start_map.end(); itr++)
    {
        Robot* robot = new RectangularRobot(itr->first, 0.7, 0.5);
        robot_map[itr->first] = robot;
    }

    // construct an instance of multi-robot space information
    auto ma_si(std::make_shared<omrc::SpaceInformation>());
    auto ma_pdef(std::make_shared<omrb::ProblemDefinition>(ma_si));

    // construct four individuals that operate in SE3
    for (auto itr = start_map.begin(); itr != start_map.end(); itr++) 
    {
        // construct the state space we are planning in
        auto space = createBounded2ndOrderCarStateSpace(32, 32);

        // name the state space parameter
        space->setName(itr->first);

        // create a control space
        auto cspace = createUniform2DRealVectorControlSpace(space);

        // construct an instance of  space information from this control space
        auto si(std::make_shared<oc::SpaceInformation>(space, cspace));

        // // set state validity checking for this space
        si->setStateValidityChecker(std::make_shared<homogeneous2ndOrderCarSystemSVC>(si, robot_map));

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
        pdef->setGoal(std::make_shared<GoalRegion2ndOrderCar>(si, goal_map.at(itr->first)));

        // add the individual information to the multi-robot SpaceInformation and ProblemDefinition
        ma_si->addIndividual(si);
        ma_pdef->addIndividual(pdef);
    }

    // lock the multi-robot SpaceInformation and ProblemDefinitions when done adding individuals
    ma_si->lock();
    ma_pdef->lock();

    // set the planner allocator for the multi-agent planner
    ompl::base::PlannerAllocator allocator = &allocateControlRRT;
    ma_si->setPlannerAllocator(allocator);

    // plan with K-CBS
    // plan using Kinodynamic Conflict Based Search
    auto planner = std::make_shared<omrc::KCBS>(ma_si);
    planner->setProblemDefinition(ma_pdef); // be sure to set the problem definition
    planner->setLowLevelSolveTime(5.);

    bool solved = planner->as<omrb::Planner>()->solve(240.0);
    if (solved)
    {
        std::cout << "Found solution!" << std::endl;
        omrb::PlanPtr solution = ma_pdef->getSolutionPlan();
        std::ofstream MyFile("plan.txt");
        solution->as<omrc::PlanControl>()->printAsMatrix(MyFile, "Robot");
        std::ofstream MyFile2("tree.txt");
        planner->printConstraintTree(MyFile2);
    }
}

int main(int argc, char ** argv)
{
    std::cout << "Planning for 10 2nd order cars inside an Empty 32x32 workspace with K-CBS." << std::endl;
    plan();
}
