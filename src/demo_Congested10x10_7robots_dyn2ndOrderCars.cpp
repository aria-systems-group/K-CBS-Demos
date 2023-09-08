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
#include "StateSpaceDatabase.h"
#include "ControlSpaceDatabase.h"
#include "StateValidityCheckerDatabase.h"
#include "StatePropagatorDatabase.h"
#include "GoalRegionDatabase.h"
#include "PlannerAllocatorDatabase.h"
#include "Benchmark.h"

#include <ompl/multirobot/control/planners/kcbs/KCBS.h>
#include <ompl/multirobot/control/planners/pp/PP.h>
#include <map>
#include <set>
#include <math.h>

namespace omrb = ompl::multirobot::base;
namespace omrc = ompl::multirobot::control;
namespace ob = ompl::base;
namespace oc = ompl::control;

// std::pair<omrc::SpaceInformationPtr, omrb::ProblemDefinitionPtr> setScene()
// {
    
// }

void plan(const std::string plannerName)
{
    // provide start and goals for every robot
    const std::map<std::string, std::pair<double, double>> start_map{       {"Robot 1", {3.0, 0.5}}, 
                                                                            {"Robot 2", {8.5, 0.5}},
                                                                            {"Robot 3", {9.0, 7.0}},
                                                                            {"Robot 4", {1.0, 8.0}},
                                                                            {"Robot 5", {1.5, 4.5}},
                                                                            {"Robot 6", {9.0, 3.0}},
                                                                            {"Robot 7", {1.0, 2.0}},
                                                                        };

    const std::map<std::string, std::pair<double, double>> goal_map{        {"Robot 1", {7.0, 0.5}}, 
                                                                            {"Robot 2", {5.0, 2.5}},
                                                                            {"Robot 3", {6.0, 7.0}}, 
                                                                            {"Robot 4", {9.0, 3.0}},
                                                                            {"Robot 5", {1.0, 8.0}},
                                                                            {"Robot 6", {9.0, 6.0}},
                                                                            {"Robot 7", {1.0, 5.0}},
                                                                        };

    // provide obstacles
    std::set<Obstacle*> obs_set;
    // auto obs1 = new RectangularObstacle(2, 1.25, 2, 1);
    auto obs1 = new RectangularObstacle(6.0, 1.25, 4.5, 1);
    auto obs2 = new RectangularObstacle(5.5, 1.25, 4.5, 1);
    auto obs3 = new RectangularObstacle(2.0, 8.0, 1, 1);
    auto obs4 = new RectangularObstacle(7.0, 6.0, 1, 2);
    auto obs5 = new RectangularObstacle(9.0, 9.0, 1, 1);
    auto obs6 = new RectangularObstacle(4.0, 4.0, 1, 1);
    auto obs7 = new RectangularObstacle(0.0, 0.0, 1, 1);
    auto obs8 = new RectangularObstacle(0.0, 5.5, 2, 1);
    auto obs9 = new RectangularObstacle(7.0, 2.25, 1, 1);
    obs_set.insert(obs1);
    obs_set.insert(obs2);
    obs_set.insert(obs3);
    obs_set.insert(obs4);
    obs_set.insert(obs5);
    obs_set.insert(obs6);
    obs_set.insert(obs7);
    obs_set.insert(obs8);
    obs_set.insert(obs9);

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
    std::string plannerName = "K-CBS";
    // std::string plannerName = "PP";
    std::cout << "Planning for 7 2nd order cars inside a Congested 10x10 workspace with " << plannerName << "." << std::endl;
    plan(plannerName);
}