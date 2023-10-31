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

#include <ompl/base/goals/GoalRegion.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <utility>

namespace ob = ompl::base;

class GoalRegion2ndOrderCar: public ob::GoalRegion
{
public:
    GoalRegion2ndOrderCar(const ob::SpaceInformationPtr &si, double gx, double gy): 
        ob::GoalRegion(si), gx_(gx), gy_(gy)
    {
        threshold_ = 0.5;
    }
    
    double distanceGoal(const ob::State *st) const override
    {
        const double* robot_pos = st->as<ob::CompoundStateSpace::StateType>()->as<ob::RealVectorStateSpace::StateType>(0)->values;
        return sqrt(pow(robot_pos[0] - gx_, 2) + pow(robot_pos[1] - gy_, 2));
    }
private:
    const double gx_;
    const double gy_;
};

class GoalRegionTwo2ndOrderCars: public ob::Goal
{
public:
    GoalRegionTwo2ndOrderCars(const ob::SpaceInformationPtr &si, double gx1, double gy1, double gx2, double gy2): 
        ob::Goal(si), gx1_(gx1), gy1_(gy1), gx2_(gx2), gy2_(gy2)
    {
    }

    bool isSatisfied(const ob::State *st) const override
    {
    	// decompose the state
    	const double* r1_pos = st->as<ob::CompoundStateSpace::StateType>()->as<ob::RealVectorStateSpace::StateType>(0)->values;
    	const double* r2_pos = st->as<ob::CompoundStateSpace::StateType>()->as<ob::RealVectorStateSpace::StateType>(2)->values;
    	// check if both are in goal
    	if (getDistance(r1_pos[0], r1_pos[1], gx1_, gy1_) < threshold_)
    	{
    		if (getDistance(r2_pos[0], r2_pos[1], gx2_, gy2_) < threshold_)
    			return true;
    	}
    	return false;
    }

    bool isSatisfied(const ob::State *st, double *distance) const override
    {
        *distance = 0;
        // decompose the state
        const double* r1_pos = st->as<ob::CompoundStateSpace::StateType>()->as<ob::RealVectorStateSpace::StateType>(0)->values;
        const double* r2_pos = st->as<ob::CompoundStateSpace::StateType>()->as<ob::RealVectorStateSpace::StateType>(2)->values;
        double d1 = getDistance(r1_pos[0], r1_pos[1], gx1_, gy1_);
        double d2 = getDistance(r2_pos[0], r2_pos[1], gx2_, gy2_);
        // check if both are in goal while updating distance
        if (d1 > threshold_)
            *distance += d1;
        if (d2 > threshold_)
            *distance += d2;
        if (*distance == 0)
            return true;
        return false;
    }
 
private:
	const double getDistance(const double x, const double y, const double gx, const double gy) const
	{
		return sqrt(pow(x - gx, 2) + pow(y - gy, 2));
	}
    const double gx1_;
    const double gy1_;
    const double gx2_;
    const double gy2_;
    const double threshold_ = 1.5;
};

class LinearizedUnicycleGoalRegion: public ob::GoalRegion
{
public:
    LinearizedUnicycleGoalRegion(const ob::SpaceInformationPtr &si, double gx, double gy): 
        ob::GoalRegion(si), gx_(gx), gy_(gy)
    {
        threshold_ = 2.0;
    }
    
    double distanceGoal(const ob::State *st) const override
    {
        const double* robot_pos = st->as<ob::RealVectorStateSpace::StateType>()->values;
        return sqrt(pow(robot_pos[0] - gx_, 2) + pow(robot_pos[1] - gy_, 2));
    }
private:
    const double gx_;
    const double gy_;
};
