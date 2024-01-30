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
#include <stdexcept>


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
                                                                            {"Robot 10",    {28, 20}},
                                                                            {"Robot 11",    {24, 12}},
                                                                            {"Robot 12",    {17, 30}},
                                                                            {"Robot 13",    {14, 2}},
                                                                            {"Robot 14",    {4, 30}},
                                                                            {"Robot 15",    {8, 30}},
                                                                            {"Robot 16",    {2, 8}},
                                                                            {"Robot 17",    {18, 18}},
                                                                            {"Robot 18",    {30, 24}},
                                                                            // {"Robot 19",    {12, 24}},
                                                                        };

    const std::unordered_map<std::string, std::pair<int, int>> goal_map {   {"Robot 0",     {20, 30}}, 
                                                                            {"Robot 1",     {23, 16}}, 
                                                                            {"Robot 2",     {5,  11}},
                                                                            {"Robot 3",     {28, 27}},
                                                                            {"Robot 4",     {13, 2}},
                                                                            {"Robot 5",     {0,  22}},
                                                                            {"Robot 6",     {22, 23}},
                                                                            {"Robot 7",     {27, 2}},
                                                                            {"Robot 8",     {11, 7}},
                                                                            {"Robot 9",     {12, 28}},
                                                                            {"Robot 10",    {28, 8}},
                                                                            {"Robot 11",    {4, 12}},
                                                                            {"Robot 12",    {31, 26}},
                                                                            {"Robot 13",    {31, 5}},
                                                                            {"Robot 14",    {2, 20}},
                                                                            {"Robot 15",    {27, 31}},
                                                                            {"Robot 16",    {11, 3}},
                                                                            {"Robot 17",    {16, 28}},
                                                                            {"Robot 18",    {22, 10}},
                                                                            // {"Robot 19",    {22, 14}},
                                                                        };

    const std::unordered_map<std::string, std::string> dynamics_map     {   {"Robot 0",     "Drone"}, 
                                                                            {"Robot 1",     "Car"}, 
                                                                            {"Robot 2",     "Car"},
                                                                            {"Robot 3",     "Drone"},
                                                                            {"Robot 4",     "Drone"},
                                                                            {"Robot 5",     "Car"},
                                                                            {"Robot 6",     "Car"},
                                                                            {"Robot 7",     "Drone"},
                                                                            {"Robot 8",     "Car"},
                                                                            {"Robot 9",     "Car"},
                                                                            {"Robot 10",     "Car"},
                                                                            {"Robot 11",     "Car"},
                                                                            {"Robot 12",     "Car"},
                                                                            {"Robot 13",     "Car"},
                                                                            {"Robot 14",     "Car"},
                                                                            {"Robot 15",     "Car"},
                                                                            {"Robot 16",     "Car"},
                                                                            {"Robot 17",     "Car"},
                                                                            {"Robot 18",     "Car"},
                                                                            // {"Robot 19",     "Car"},
                                                                        };
    // provide obstacles
    std::set<Obstacle*> obs_set;
    auto obs1 = new RectangularObstacle3D(20, 25, 2, 2, 1); // lower_left_x. lower_left_y, length, width, height
    auto obs2 = new RectangularObstacle3D(5, 2, 1, 1, 2);
    auto obs3 = new RectangularObstacle3D(4, 14, 1, 1, 2);
    auto obs4 = new RectangularObstacle3D(19, 5, 1, 1, 2);
    auto obs5 = new RectangularObstacle3D(29, 0, 1, 2, 2);
    // auto obs6 = new RectangularObstacle3D(30, 31, 2, 2, 1);
    // auto obs7 = new RectangularObstacle3D(7, 14, 2, 2, 2);
    // auto obs8 = new RectangularObstacle3D(15, 31, 2, 1, 1);
    // auto obs9 = new RectangularObstacle3D(12, 25, 2, 1, 1);
    // auto obs10 = new RectangularObstacle3D(17, 0, 1, 2, 2);
    // auto obs11 = new RectangularObstacle3D(8, 22, 1, 2, 1);
    obs_set.insert(obs1);
    obs_set.insert(obs2);
    obs_set.insert(obs3);
    obs_set.insert(obs4);
    obs_set.insert(obs5);
    // obs_set.insert(obs6);
    // obs_set.insert(obs7);
    // obs_set.insert(obs8);
    // obs_set.insert(obs9);
    // obs_set.insert(obs10);
    // obs_set.insert(obs11);

    // construct all of the robots
    std::unordered_map<std::string, Robot*> robot_map;
    for (auto itr = start_map.begin(); itr != start_map.end(); itr++)
    {
        Robot* robot = new RectangularRobot3D(itr->first, 1.0, 1.0, 0.5);
        robot->setDynamics(dynamics_map.at(itr->first));
        robot_map[itr->first] = robot;
    }

    // construct an instance of multi-robot space information
    auto ma_si(std::make_shared<omrc::SpaceInformation>());
    auto ma_pdef(std::make_shared<omrb::ProblemDefinition>(ma_si));

    // construct four individuals that operate in SE3
    const int x_lim = 32;
    const int y_lim = 32;
    const int z_lim = 5;
    for (auto itr = start_map.begin(); itr != start_map.end(); itr++) 
    {
        std::string dynamics;
        try {
        	dynamics = dynamics_map.at(itr->first);
        }
        catch (std::out_of_range const& ex){
        	throw std::out_of_range("Key: `" + itr->first + "` does not exist in dynamics map.");
        }

        // construct the state space we are planning in
        oc::SpaceInformationPtr si = nullptr;
        ob::ProblemDefinitionPtr pdef = nullptr;
        if (dynamics == "Drone") {
            // set up everything needed for drone planning
            auto space = createBoundedDroneStateSpace(x_lim, y_lim, z_lim);
            auto cspace = createDroneControlSpace(space);
            si = std::make_shared<oc::SpaceInformation>(space, cspace);
            auto odeSolver(std::make_shared<oc::ODEBasicSolver<>>(si, &SimplifiedDroneODE));
            si->setStatePropagator(oc::ODESolver::getStatePropagator(odeSolver, &SecondOrderCarODEPostIntegration));
            space->setName(itr->first);
            pdef = std::make_shared<ob::ProblemDefinition>(si);
            
            ob::ScopedState<> start(space);
            start[0] = start_map.at(itr->first).first;
            start[1] = start_map.at(itr->first).second;
            start[2] = 0.0;
            start[3] = 0.0;
            start[4] = 0.0;
            start[5] = 0.0;
            pdef->addStartState(start);
            pdef->setGoal(std::make_shared<GoalRegionSimplifiedDrone>(si, goal_map.at(itr->first).first, goal_map.at(itr->first).second, 0.0));
        }
        else if (dynamics == "Car") {
            // set up everything needed for car planning
            auto space = createBounded2ndOrderCarStateSpace(x_lim, y_lim);
            auto cspace = createUniform2DRealVectorControlSpace(space);
            si = std::make_shared<oc::SpaceInformation>(space, cspace);
            auto odeSolver(std::make_shared<oc::ODEBasicSolver<>>(si, &SecondOrderCarODE));
            si->setStatePropagator(oc::ODESolver::getStatePropagator(odeSolver, &SecondOrderCarODEPostIntegration));
            space->setName(itr->first);
            pdef = std::make_shared<ob::ProblemDefinition>(si);

            ob::ScopedState<> start(space);
            start[0] = start_map.at(itr->first).first;
            start[1] = start_map.at(itr->first).second;
            start[2] = 0.0;
            start[3] = 0.0;
            start[4] = 0.0;
            pdef->addStartState(start);
            pdef->setGoal(std::make_shared<GoalRegion2ndOrderCar>(si, goal_map.at(itr->first).first, goal_map.at(itr->first).second));
        }
        else {
            throw std::logic_error("Planning for vehicles with `" + dynamics + 
                "` dynamics are not yet supported. Please choose another dynamics function.");
        }

        // set state validity checking for this space
        si->setStateValidityChecker(std::make_shared<heterogeneousSystemIn3DStateValidityChecker>(si, robot_map, obs_set));

        // set the propagation step size
        si->setPropagationStepSize(0.1);

        // set this to remove the warning
        si->setMinMaxControlDuration(1, 10);

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
    std::cout << "Planning for 1 drone inside an Empty 32x32x10 workspace with " << plannerName << "." << std::endl;
    plan(plannerName);
}