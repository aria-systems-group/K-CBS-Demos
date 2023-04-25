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
#include <ompl/control/SpaceInformation.h>

namespace ob = ompl::base;

class homogeneous2ndOrderCarSystemSVC: public ob::StateValidityChecker
{
public:
    homogeneous2ndOrderCarSystemSVC(const ob::SpaceInformationPtr &si, std::unordered_map<std::string, Robot*> robots): 
        robot1_name_(si->getStateSpace()->getName()), robot1_(robots.at(robot1_name_)), rad1_(robot1_->getBoundingRadius()), robots_(robots), ob::StateValidityChecker(si)
    {
    }

    // Answers the question: is the robot described by `si_` at `state` valid?
    bool isValid(const ompl::base::State *state) const override
    {
        if (!si_->satisfiesBounds(state))
            return false;
        return true;
    }

    // Answers the question: does the robot described by `si_` at `state1` avoid collision with some other robot described by a different `si` located at `state2`?
    bool areStatesValid(const ompl::base::State* state1, const std::pair<const ompl::base::SpaceInformationPtr,const ompl::base::State*> state2) const override
    {
        // get the correct other robots information
        const std::string robot2_name = state2.first->getStateSpace()->getName();
        Robot* other_robot = robots_.at(robot2_name);
        const double rad2 = other_robot->getBoundingRadius();

        // get the positions of the robots
        const double* this_pos = state1->as<ob::CompoundStateSpace::StateType>()->as<ob::RealVectorStateSpace::StateType>(0)->values;
        const double* other_pos = state2.second->as<ob::CompoundStateSpace::StateType>()->as<ob::RealVectorStateSpace::StateType>(0)->values;

        // perform quick collision check
        if (!performQuickCollisionCheck(this_pos, other_pos, rad2))
        {
            // get the positions of the robots
            const double this_rot = state1->as<ob::CompoundStateSpace::StateType>()->as<ob::SO2StateSpace::StateType>(1)->value;
            const double other_rot = state2.second->as<ob::CompoundStateSpace::StateType>()->as<ob::SO2StateSpace::StateType>(1)->value;
            // collision possible, must check exact
            if (!performExactCollisionCheck(this_pos, this_rot, other_pos, other_rot, other_robot))
                return false;
        }
        return true;
    }
private:
    // returns true if no collision is possible (i.e. robots are too far away to collide)
    bool performQuickCollisionCheck(const double* this_pos, const double* other_pos, const double other_rad) const
    {
        const double distance = sqrt(pow(this_pos[0] - other_pos[0], 2) + pow(this_pos[1] - other_pos[1], 2));
        if (distance > (rad1_ + other_rad))
            return true;
        else
            return false;
    }

    // returns true if no collision
    bool performExactCollisionCheck(const double* this_pos, const double this_rot, const double* other_pos, const double other_rot, const Robot* other_robot) const
    {
        BoostPolygon poly1 = robot1_->getShape();
        // use boost transform to create polygon at true robots location
        BoostPolygon tmp1;
        BoostPolygon result1;
        boost::geometry::correct(tmp1);
        boost::geometry::assign(tmp1, poly1);
        boost::geometry::strategy::transform::matrix_transformer<double, 2, 2> xfrm1(
                 cos(this_rot), sin(this_rot), this_pos[0],
                -sin(this_rot), cos(this_rot), this_pos[1],
                          0,          0,  1);
        boost::geometry::transform(tmp1, result1, xfrm1);
        boost::geometry::correct(result1);

        BoostPolygon poly2 = other_robot->getShape();
        // use boost transform to create polygon at true robots location
        BoostPolygon tmp2;
        BoostPolygon result2;
        boost::geometry::correct(tmp2);
        boost::geometry::assign(tmp2, poly2);
        boost::geometry::strategy::transform::matrix_transformer<double, 2, 2> xfrm2(
                 cos(other_rot), sin(other_rot), other_pos[0],
                -sin(other_rot), cos(other_rot), other_pos[1],
                          0,          0,  1);
        boost::geometry::transform(tmp2, result2, xfrm2);
        boost::geometry::correct(result2);

        // check if resulting polygons are in collision
        if (!boost::geometry::disjoint(result1, result2))
            return false;
        else
            return true;
    }

    const std::string robot1_name_;
    const Robot* robot1_;
    const double rad1_;
    std::unordered_map<std::string, Robot*> robots_;
};
