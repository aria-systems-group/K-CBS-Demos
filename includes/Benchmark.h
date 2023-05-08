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

#include <ompl/multirobot/control/SpaceInformation.h>
#include <ompl/multirobot/base/ProblemDefinition.h>
#include <ompl/multirobot/base/Planner.h>
#include <ompl/multirobot/control/planners/kcbs/KCBS.h>
#include <map>
#include <iostream>
#include <fstream>

namespace omrb = ompl::multirobot::base;
namespace omrc = ompl::multirobot::control;

class Benchmark
{
public:
    Benchmark():si_(nullptr), pdef_(nullptr), solveTime_(300), numRuns_(100), filename_("Results")
    {
    }

    ~Benchmark()
    {
        results_.clear();
        si_.reset();
        pdef_.reset();
        for (auto &p: planners_)
            p.reset();
    }

    void setSpaceInformation(omrc::SpaceInformationPtr &si)
    {
        si_ = si;
    }

    void setPoroblemDefinition(omrb::ProblemDefinitionPtr &pdef)
    {
        pdef_ = pdef;
    }

    void addPlanner(omrb::PlannerPtr &p)
    {
        planners_.push_back(p);
    }

    void setSolveTime(const double t)
    {
        solveTime_ = t;
    }

    void setNumberOfRuns(const unsigned int n)
    {
        numRuns_ = n;
    }

    void setFileName(const std::string name)
    {
        filename_ = name;
    }

    void run()
    {
        // set up all of the planners and saveable parameters
        for (auto &p: planners_)
        {
            results_[p->getName() + " Success (Bool)"] = {};
            results_[p->getName() + " Computation Times (seconds)"] = {};
            if (p->getName() == "K-CBS")
            {
                results_[p->getName() + " Number of Expanded Nodes"] = {};
                results_[p->getName() + " Number Approximate Solutions"] = {};
                results_[p->getName() + " Root Node Solve Time"] = {};
            }
        }

        std::cout << "Set-Up Complete: Benchmarking..." << std::endl;

        for (unsigned int n = 0; n < numRuns_; n++)
        {
            std::cout << "Run " << n << std::endl;
            for (auto &p: planners_)
            {
                // time the planner's solve sequence
                auto start = std::chrono::high_resolution_clock::now();
                ob::PlannerStatus solved = p->solve(solveTime_);
                auto end = std::chrono::high_resolution_clock::now();
                auto duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
                double duration_s = (duration_ms.count() * 0.001);

                // save the relevant data
                results_[p->getName() + " Success (Bool)"].push_back(std::to_string(solved == ob::PlannerStatus::EXACT_SOLUTION));
                results_[p->getName() + " Computation Times (seconds)"].push_back(std::to_string(duration_s));
                if (p->getName() == "K-CBS")
                {
                    results_[p->getName() + " Number of Expanded Nodes"].push_back(std::to_string(p->as<omrc::KCBS>()->getNumberOfNodesExpanded()));
                    results_[p->getName() + " Number Approximate Solutions"].push_back(std::to_string(p->as<omrc::KCBS>()->getNumberOfApproximateSolutions()));
                    results_[p->getName() + " Root Node Solve Time"].push_back(std::to_string(p->as<omrc::KCBS>()->getRootSolveTime()));
                }

                // clear the planner data
                p->clear();

                // reset the problem definition
                pdef_->clearSolutionPaths();
            }
        }
    }

    void writeCSV()
{
    std::string name = filename_ + ".csv";

    std::ifstream infile(name);
    bool exist = infile.good();
    infile.close();
    if (!exist)
    {
        std::ofstream addHeads(name);
        for(auto itr = results_.begin(); itr != results_.end(); ++itr)
            addHeads << itr->first << ",";
        addHeads << std::endl;
        addHeads.close();
    }
    std::ofstream stats(name, std::ios::app);
    for (unsigned int idx = 0; idx != numRuns_; idx++)
    {
        for(auto itr = results_.begin(); itr != results_.end(); ++itr)
        {
            stats << itr->second[idx] << ",";
        }
        stats << std::endl;
    }
    stats.close();
}

private:
    omrc::SpaceInformationPtr si_;
    omrb::ProblemDefinitionPtr pdef_;
    std::vector<omrb::PlannerPtr> planners_;
    double solveTime_;
    unsigned int numRuns_;
    std::string filename_;
    std::map<std::string, std::vector<std::string>> results_;
};
