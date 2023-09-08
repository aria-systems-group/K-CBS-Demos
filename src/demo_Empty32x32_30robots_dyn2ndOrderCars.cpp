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
#include <ompl/multirobot/control/planners/pp/PP.h>
#include <unordered_map>
#include <math.h>
#include <chrono>
#include <thread>

namespace omrb = ompl::multirobot::base;
namespace omrc = ompl::multirobot::control;
namespace ob = ompl::base;
namespace oc = ompl::control;

void plan(const std::string plannerName)
{
    // provide start and goals for every robot
    const std::unordered_map<std::string, std::pair<int, int>> start_map{   {"Robot 0",     {11, 25}}, 
                                                                            {"Robot 1",     {14, 31}}, 
                                                                            {"Robot 2",     {5,  23}},
                                                                            {"Robot 3",     {13, 13}},
                                                                            {"Robot 4",     {3,  27}},
                                                                            {"Robot 5",     {16, 3}},
                                                                            {"Robot 6",     {24, 25}},
                                                                            {"Robot 7",     {18, 22}},
                                                                            {"Robot 8",     {28, 8}},
                                                                            {"Robot 9",     {7,  3}},
                                                                            {"Robot 10",    {5,  0}},
                                                                            {"Robot 11",    {14, 7}},
                                                                            {"Robot 12",    {23, 8}},
                                                                            {"Robot 13",    {9,  20}},
                                                                            {"Robot 14",    {26, 14}},
                                                                            {"Robot 15",    {19, 10}},
                                                                            {"Robot 16",    {9,  25}},
                                                                            {"Robot 17",    {22, 1}},
                                                                            {"Robot 18",    {18, 24}},
                                                                            {"Robot 19",    {3,  4}},
                                                                            {"Robot 20",    {1,  26}},
                                                                            {"Robot 21",    {11, 27}},
                                                                            {"Robot 22",    {31, 10}},
                                                                            {"Robot 23",    {12, 16}},
                                                                            {"Robot 24",    {8,  9}},
                                                                            {"Robot 25",    {13, 14}},
                                                                            {"Robot 26",    {0,  3}},
                                                                            {"Robot 27",    {31, 8}},
                                                                            {"Robot 28",    {30, 17}},
                                                                            {"Robot 29",    {6,  17}},
                                                                        };

    const std::unordered_map<std::string, std::pair<int, int>> goal_map{    {"Robot 0",     {20, 30}}, 
                                                                            {"Robot 1",     {23, 16}}, 
                                                                            {"Robot 2",     {5,  11}},
                                                                            {"Robot 3",     {28, 27}},
                                                                            {"Robot 4",     {13, 2}},
                                                                            {"Robot 5",     {0,  22}},
                                                                            {"Robot 6",     {22, 23}},
                                                                            {"Robot 7",     {27, 2}},
                                                                            {"Robot 8",     {11, 7}},
                                                                            {"Robot 9",     {12, 28}},
                                                                            {"Robot 10",    {2,  10}},
                                                                            {"Robot 11",    {26, 28}},
                                                                            {"Robot 12",    {24, 22}},
                                                                            {"Robot 13",    {20, 20}},
                                                                            {"Robot 14",    {31, 17}},
                                                                            {"Robot 15",    {28, 28}},
                                                                            {"Robot 16",    {26, 2}},
                                                                            {"Robot 17",    {0,  19}},
                                                                            {"Robot 18",    {10, 14}},
                                                                            {"Robot 19",    {11, 30}},
                                                                            {"Robot 20",    {30, 18}},
                                                                            {"Robot 21",    {30, 30}},
                                                                            {"Robot 22",    {6,  6}},
                                                                            {"Robot 23",    {21, 9}},
                                                                            {"Robot 24",    {26, 13}},
                                                                            {"Robot 25",    {17, 12}},
                                                                            {"Robot 26",    {28, 5}},
                                                                            {"Robot 27",    {31, 18}},
                                                                            {"Robot 28",    {28, 21}},
                                                                            {"Robot 29",    {0,  29}},
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
        pdef->setGoal(std::make_shared<GoalRegion2ndOrderCar>(si, goal_map.at(itr->first).first, goal_map.at(itr->first).second));

        // add the individual information to the multi-robot SpaceInformation and ProblemDefinition
        ma_si->addIndividual(si);
        ma_pdef->addIndividual(pdef);
    }

    // set the planner allocator for the multi-agent planner
    ompl::base::PlannerAllocator allocator = &allocateControlRRT;
    ma_si->setPlannerAllocator(allocator);

    // lock the multi-robot SpaceInformation and ProblemDefinitions when done adding individuals
    ma_si->lock();
    ma_pdef->lock();

    omrb::PlannerPtr planner = nullptr;
    if (plannerName == "K-CBS")
    {
        // plan using Kinodynamic Conflict Based Search
        planner = std::make_shared<omrc::KCBS>(ma_si);
        planner->as<omrc::KCBS>()->setLowLevelSolveTime(5.);
        planner->as<omrc::KCBS>()->setNumThreads(std::thread::hardware_concurrency());
    }
    else
    {
        // plan using Prioritized Planner
        planner = std::make_shared<omrc::PP>(ma_si);
    }

    planner->setProblemDefinition(ma_pdef); // be sure to set the problem definition

    auto start = std::chrono::high_resolution_clock::now();
    bool solved = planner->as<omrb::Planner>()->solve(180.0);
    auto end = std::chrono::high_resolution_clock::now();
    auto duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    double duration_s = (duration_ms.count() * 0.001);

    if (solved)
    {
        printf("Found Solution in %0.2f seconds!\n", duration_s);
        omrb::PlanPtr solution = ma_pdef->getSolutionPlan();
        std::ofstream MyFile("plan.txt");
        solution->as<omrc::PlanControl>()->printAsMatrix(MyFile, "Robot");
    }

    if (plannerName == "K-CBS")
    {
        std::ofstream MyFile2("tree.txt");
        planner->as<omrc::KCBS>()->printConstraintTree(MyFile2);
    }
}

int main(int argc, char ** argv)
{
    // std::string plannerName = "K-CBS";
    std::string plannerName = "PP";
    std::cout << "Planning for 30 2nd order cars inside an Empty 32x32 workspace with " << plannerName << "." << std::endl;
    plan(plannerName);
}