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
#include <map>
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
    const std::map<std::string, std::pair<int, int>> start_map{             {"Robot 0", {11, 6}}, 
                                                                            {"Robot 1", {29, 13}}, 
                                                                            {"Robot 2", {9,  0}},
                                                                            {"Robot 3", {11, 16}},
                                                                            {"Robot 4", {3,  26}},
                                                                            {"Robot 5", {19, 21}},
                                                                            {"Robot 6", {23, 0}},
                                                                            {"Robot 7", {29, 10}},
                                                                            {"Robot 8", {1,  12}},
                                                                            {"Robot 9", {30, 30}},
                                                                        };

    const std::map<std::string, std::pair<int, int>> goal_map{              {"Robot 0", {7,  18}}, 
                                                                            {"Robot 1", {1,  16}}, 
                                                                            {"Robot 2", {13, 21}},
                                                                            {"Robot 3", {18, 18}},
                                                                            {"Robot 4", {7,  15}},
                                                                            {"Robot 5", {27, 4}},
                                                                            {"Robot 6", {0,  29}},
                                                                            {"Robot 7", {25, 9}},
                                                                            {"Robot 8", {10, 22}},
                                                                            {"Robot 9", {15, 19}},
                                                                        };


    std::set<Obstacle*> obs_set;
    auto obs1 = new RectangularObstacle(21 - 0.5, 1 - 0.5, 1, 1);
    auto obs2 = new RectangularObstacle(25 - 0.5, 1 - 0.5, 1, 1);
    auto obs3 = new RectangularObstacle(8 - 0.5,  2 - 0.5, 1, 1);
    auto obs4 = new RectangularObstacle(19 - 0.5, 2 - 0.5, 1, 1);
    auto obs5 = new RectangularObstacle(26 - 0.5, 2 - 0.5, 1, 1);
    auto obs6 = new RectangularObstacle(15 - 0.5, 3 - 0.5, 1, 1);
    auto obs7 = new RectangularObstacle(29 - 0.5, 3 - 0.5, 1, 1);
    auto obs8 = new RectangularObstacle(24 - 0.5, 4 - 0.5, 1, 1);
    auto obs9 = new RectangularObstacle(9 - 0.5,  5 - 0.5, 1, 1);
    auto obs10 = new RectangularObstacle(20 - 0.5, 5 - 0.5, 1, 1);
    auto obs11 = new RectangularObstacle(4 - 0.5,  6 - 0.5, 1, 1);
    auto obs12 = new RectangularObstacle(8 - 0.5,  7 - 0.5, 1, 1);
    auto obs13 = new RectangularObstacle(24 - 0.5, 7 - 0.5, 1, 1);
    auto obs14 = new RectangularObstacle(2 - 0.5,  8 - 0.5, 1, 1);
    auto obs15 = new RectangularObstacle(19 - 0.5, 8 - 0.5, 1, 1);
    auto obs16 = new RectangularObstacle(8 - 0.5,  9 - 0.5, 1, 1);
    auto obs17 = new RectangularObstacle(26 - 0.5, 9 - 0.5, 1, 1);
    auto obs18 = new RectangularObstacle(17 - 0.5, 10 - 0.5, 1, 1);
    auto obs19 = new RectangularObstacle(20 - 0.5, 11 - 0.5, 1, 1);
    auto obs20 = new RectangularObstacle(5 - 0.5,  12 - 0.5, 1, 1);
    auto obs21 = new RectangularObstacle(11 - 0.5, 12 - 0.5, 1, 1);
    auto obs22 = new RectangularObstacle(27 - 0.5, 12 - 0.5, 1, 1);
    auto obs23 = new RectangularObstacle(4 - 0.5,  14 - 0.5, 1, 1);
    auto obs24 = new RectangularObstacle(20 - 0.5, 15 - 0.5, 1, 1);
    auto obs25 = new RectangularObstacle(18 - 0.5, 16 - 0.5, 1, 1);
    auto obs26 = new RectangularObstacle(28 - 0.5, 17 - 0.5, 1, 1);
    auto obs27 = new RectangularObstacle(3 - 0.5,  19 - 0.5, 1, 1);
    auto obs28 = new RectangularObstacle(26 - 0.5, 19 - 0.5, 1, 1);
    auto obs29 = new RectangularObstacle(4 - 0.5,  21 - 0.5, 1, 1);
    auto obs30 = new RectangularObstacle(30 - 0.5, 21 - 0.5, 1, 1);
    auto obs31 = new RectangularObstacle(20 - 0.5, 23 - 0.5, 1, 1);
    auto obs32 = new RectangularObstacle(25 - 0.5, 23 - 0.5, 1, 1);
    auto obs33 = new RectangularObstacle(28 - 0.5, 23 - 0.5, 1, 1);
    auto obs34 = new RectangularObstacle(5 - 0.5,  24 - 0.5, 1, 1);
    auto obs35 = new RectangularObstacle(1 - 0.5,  26 - 0.5, 1, 1);
    auto obs36 = new RectangularObstacle(14 - 0.5, 26 - 0.5, 1, 1);
    auto obs37 = new RectangularObstacle(21 - 0.5, 26 - 0.5, 1, 1);
    auto obs38 = new RectangularObstacle(28 - 0.5, 26 - 0.5, 1, 1);
    auto obs39 = new RectangularObstacle(7 - 0.5,  27 - 0.5, 1, 1);
    auto obs40 = new RectangularObstacle(22 - 0.5, 28 - 0.5, 1, 1);
    auto obs41 = new RectangularObstacle(9 - 0.5,  29 - 0.5, 1, 1);
    auto obs42 = new RectangularObstacle(4 - 0.5,  30 - 0.5, 1, 1);
    auto obs43 = new RectangularObstacle(15 - 0.5, 30 - 0.5, 1, 1);

    obs_set.insert(obs1); 
    obs_set.insert(obs2); 
    obs_set.insert(obs3); 
    obs_set.insert(obs4); 
    obs_set.insert(obs5); 
    obs_set.insert(obs6); 
    obs_set.insert(obs7); 
    obs_set.insert(obs8); 
    obs_set.insert(obs9); 
    obs_set.insert(obs10); 
    obs_set.insert(obs11); 
    obs_set.insert(obs12); 
    obs_set.insert(obs13); 
    obs_set.insert(obs14); 
    obs_set.insert(obs15); 
    obs_set.insert(obs16); 
    obs_set.insert(obs17); 
    obs_set.insert(obs18); 
    obs_set.insert(obs19); 
    obs_set.insert(obs20); 
    obs_set.insert(obs21); 
    obs_set.insert(obs22); 
    obs_set.insert(obs23); 
    obs_set.insert(obs24); 
    obs_set.insert(obs25); 
    obs_set.insert(obs26); 
    obs_set.insert(obs27); 
    obs_set.insert(obs28); 
    obs_set.insert(obs29); 
    obs_set.insert(obs30); 
    obs_set.insert(obs31); 
    obs_set.insert(obs32); 
    obs_set.insert(obs33); 
    obs_set.insert(obs34); 
    obs_set.insert(obs35); 
    obs_set.insert(obs36); 
    obs_set.insert(obs37); 
    obs_set.insert(obs38); 
    obs_set.insert(obs39); 
    obs_set.insert(obs40); 
    obs_set.insert(obs41); 
    obs_set.insert(obs42); 
    obs_set.insert(obs43);

    // construct all of the robots (assume square robots with unit length)
    std::unordered_map<std::string, Robot*> robot_map;
    for (auto itr = start_map.begin(); itr != start_map.end(); itr++)
    {
        Robot* robot = new RectangularRobot(itr->first, 0.25, 0.25);
        robot_map[itr->first] = robot;
    }

    // construct an instance of multi-robot space information
    auto ma_si(std::make_shared<omrc::SpaceInformation>());
    auto ma_pdef(std::make_shared<omrb::ProblemDefinition>(ma_si));

    // construct four individuals that operate in SE3
    for (auto itr = start_map.begin(); itr != start_map.end(); itr++) 
    {
        // construct the state space we are planning in
        auto space = createBoundedLinearizedUnicycleStateSpace(32, 32);

        // name the state space parameter
        space->setName(itr->first);

        // create a control space
        auto cspace = createLinearizedUnicycleControlSpace(space, 32, 32);

        // construct an instance of  space information from this control space
        auto si(std::make_shared<oc::SpaceInformation>(space, cspace));

        // // set state validity checking for this space
        si->setStateValidityChecker(std::make_shared<homogeneousLinearizedUnicycleStateValidityChecker>(si, robot_map, obs_set));

        // set the state propagation routine
        si->setStatePropagator(oc::StatePropagatorPtr(new LinearizedUnicycleDynamics(si)));

        // set the propagation step size
        si->setPropagationStepSize(0.15);

        // set this to remove the warning
        si->setMinMaxControlDuration(1, 10);

        // create a start state
        ob::ScopedState<> start(space);
        start[0] = start_map.at(itr->first).first;
        start[1] = start_map.at(itr->first).second;
        start[2] = 0.0;
        start[3] = 0.1;

        // // create a problem instance
        auto pdef(std::make_shared<ob::ProblemDefinition>(si));

        // set the start and goal states
        pdef->addStartState(start);
        pdef->setGoal(std::make_shared<LinearizedUnicycleGoalRegion>(si, goal_map.at(itr->first).first, goal_map.at(itr->first).second));

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
    std::cout << "Planning for 10 2nd order cars inside an Empty 32x32 workspace with " << plannerName << "." << std::endl;
    plan(plannerName);
}
