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

#include <iterator>
#include <ompl/multirobot/control/SpaceInformation.h>
#include <ompl/multirobot/base/ProblemDefinition.h>
#include <ompl/multirobot/base/Planner.h>
#include <ompl/multirobot/control/planners/kcbs/KCBS.h>
#include <map>
#include <iostream>
#include <fstream>

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
#include <ompl/multirobot/control/planners/pp/PP.h>
#include <map>
#include <iostream>
#include <fstream>
#include <string>
#include <thread>

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
        for (auto &p: planners_) {
            std::cout << p.use_count() << std::endl;
            p.reset();
        }
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

    void setKCBSMergeBound(int value)
    {
        merge_bound_ = value;
    }

    void setNumberOfRuns(const unsigned int n)
    {
        numRuns_ = n;
    }

    void setFileName(const std::string name)
    {
        filename_ = name;
    }

    void runKCBS()
    {
        omrc::KCBS p(si_);
        p.setProblemDefinition(pdef_);
        p.setLowLevelSolveTime(5.);
        p.setMergeBound(merge_bound_);

        // time the planner's solve sequence
        auto start = std::chrono::high_resolution_clock::now();
        ob::PlannerStatus solved = p.as<omrb::Planner>()->solve(solveTime_);
        auto end = std::chrono::high_resolution_clock::now();
        auto duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        double duration_s = (duration_ms.count() * 0.001);

        // save the relevant data
        results_.insert({"K-CBS Success (Bool)", std::to_string(solved == ob::PlannerStatus::EXACT_SOLUTION)});
        results_.insert({"K-CBS Computation Times (s)", std::to_string(duration_s)});
        results_.insert({"K-CBS Root Node Solve Time (s)", std::to_string(p.getRootSolveTime())});
        results_.insert({"K-CBS Number of Expanded Nodes", std::to_string(p.getNumberOfNodesExpanded())});
        results_.insert({"K-CBS Number Approximate Solutions", std::to_string(p.getNumberOfApproximateSolutions())});

        // reset the problem definition
        pdef_->clearSolutionPaths();

        p.clear();
        writeCSV();
    }

    void runPP()
    {
        omrc::PP p(si_);
        p.setProblemDefinition(pdef_);

        // time the planner's solve sequence
        auto start = std::chrono::high_resolution_clock::now();
        ob::PlannerStatus solved = p.as<omrb::Planner>()->solve(solveTime_);
        auto end = std::chrono::high_resolution_clock::now();
        auto duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        double duration_s = (duration_ms.count() * 0.001);

        // save the relevant data
        results_.insert({"PP Success (Bool)", std::to_string(solved == ob::PlannerStatus::EXACT_SOLUTION)});
        results_.insert({"PP Computation Times (s)", std::to_string(duration_s)});

        // reset the problem definition
        pdef_->clearSolutionPaths();

        p.clear();
        writeCSV();
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
        for(auto itr = results_.begin(); itr != results_.end(); ++itr)
            stats << itr->second << ",";
        stats << std::endl;
        stats.close();
    }

private:
    omrc::SpaceInformationPtr si_;
    omrb::ProblemDefinitionPtr pdef_;
    std::vector<omrb::PlannerPtr> planners_;
    double solveTime_;
    unsigned int numRuns_;
    int merge_bound_ = std::numeric_limits<int>::max();
    std::string filename_;
    std::map<std::string, std::string> results_;
};
