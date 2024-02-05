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

#include <ompl/control/ODESolver.h>
#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>

namespace ob = ompl::base;
namespace oc = ompl::control;

void SecondOrderCarODE (const oc::ODESolver::StateType& q, const oc::Control* control, oc::ODESolver::StateType& qdot)
{
    // q = x, y, v, phi, theta
    // c = a, phidot
    const double *u = control->as<oc::RealVectorControlSpace::ControlType>()->values;
    // state params
    const double v = q[2];
    const double phi = q[3];
    const double theta = q[4];
    const double carLength = 0.5;

    // Zero out qdot
    qdot.resize (q.size (), 0);
 
    // vehicle model
    qdot[0] = v * cos(theta);
    qdot[1] = v * sin(theta);
    qdot[2] = u[0];
    qdot[3] = u[1];
    qdot[4] = (v / carLength) * tan(phi);
}

// callback for putting angle [0, 2pi]
void SecondOrderCarODEPostIntegration (const ob::State* /*state*/, const oc::Control* /*control*/, const double /*duration*/, ob::State *result)
{
    // wrap the angle
    ob::CompoundState* cs = result->as<ob::CompoundState>();
    ob::SO2StateSpace::StateType* angleState1 = cs->as<ob::SO2StateSpace::StateType>(1);
    ob::SO2StateSpace SO2;
    SO2.enforceBounds(angleState1);
}

// Not used directly -- see TwoSecondOrderCarStatePropagator class
void TwoSecondOrderCarsODE (const oc::ODESolver::StateType& q, const oc::Control* control, oc::ODESolver::StateType& qdot)
{
    // q = x1, y1, v1, phi1, theta1, x2, y2, v2, phi2, theta2 
    // c = a1, phidot1, a2, phidot2
    const double *u = control->as<oc::RealVectorControlSpace::ControlType>()->values;
    
    // Zero out qdot
    qdot.resize (q.size (), 0);
    
    // iterate over two robots and propagate them seperately
    for (unsigned int idx = 0; idx < 2; idx++)
    {
        // state params
        const double v_i = q[5 * idx + 2];
        const double phi_i = q[5 * idx + 3];
        const double theta_i = q[5 * idx + 4];
        const double carLength_i = 0.5;

        // vehicle model
        qdot[5 * idx + 0] = v_i * cos(theta_i);
        qdot[5 * idx + 1] = v_i * sin(theta_i);
        qdot[5 * idx + 2] = u[2 * idx];
        qdot[5 * idx + 3] = u[2 * idx + 1];
        qdot[5 * idx + 4] = (v_i / carLength_i) * tan(phi_i);
    }
}

// Not used directly -- see TwoSecondOrderCarStatePropagator class
// callback for putting angle [0, 2pi]
void TwoSecondOrderCarODEPostIntegration (const ob::State* /*state*/, const oc::Control* /*control*/, const double /*duration*/, ob::State *result)
{
    ob::CompoundState* cs = result->as<ob::CompoundState>();
    // iterate over two robots and wrap their angles seperately
    for (unsigned int idx = 0; idx < 2; idx++)
    {
        // wrap the angle
        ob::SO2StateSpace::StateType* angleState1 = cs->as<ob::SO2StateSpace::StateType>(2 * idx + 1);
        ob::SO2StateSpace SO2;
        SO2.enforceBounds(angleState1);
    }
}

class TwoSecondOrderCarStatePropagator: public oc::StatePropagator
{
public:
    /** \brief Constructor */
    TwoSecondOrderCarStatePropagator(const oc::SpaceInformationPtr &si, double gx1, double gy1, double gx2, double gy2): 
        oc::StatePropagator(si.get()), gx1_(gx1), gy1_(gy1), gx2_(gx2), gy2_(gy2)
    {
        auto odeSolver(std::make_shared<oc::ODEBasicSolver<>>(si, &TwoSecondOrderCarsODE));
        systemSP_ = oc::ODESolver::getStatePropagator(odeSolver, &TwoSecondOrderCarODEPostIntegration);
    }

    void propagate(const ob::State *state, const oc::Control *control, double duration, ob::State *result) const override
    {
        // first, propagate the system as normal
        systemSP_->propagate(state, control, duration, result);
        // next, we override the result when a robot is in goal (prior to propagation)
        double* r1_pos = state->as<ob::CompoundStateSpace::StateType>()->as<ob::RealVectorStateSpace::StateType>(0)->values;
        double* r2_pos = state->as<ob::CompoundStateSpace::StateType>()->as<ob::RealVectorStateSpace::StateType>(2)->values;
        if (getDistance(r1_pos[0], r1_pos[1], gx1_, gy1_) < threshold_)
        {
            // override result to state for r1 elems
            for (unsigned int idx = 0; idx < 5; idx++)
                result->as<ob::CompoundStateSpace::StateType>()->as<ob::RealVectorStateSpace::StateType>(0)->values[idx] = r1_pos[idx];
            result->as<ob::CompoundStateSpace::StateType>()->as<ob::SO2StateSpace::StateType>(1)->value = 
                state->as<ob::CompoundStateSpace::StateType>()->as<ob::SO2StateSpace::StateType>(1)->value;
        }
        if (getDistance(r2_pos[0], r2_pos[1], gx2_, gy2_) < threshold_)
        {
            // override result to state for r2 elems
            for (unsigned int idx = 0; idx < 5; idx++)
                result->as<ob::CompoundStateSpace::StateType>()->as<ob::RealVectorStateSpace::StateType>(2)->values[idx] = r2_pos[idx];
            result->as<ob::CompoundStateSpace::StateType>()->as<ob::SO2StateSpace::StateType>(3)->value = 
                state->as<ob::CompoundStateSpace::StateType>()->as<ob::SO2StateSpace::StateType>(3)->value;
        }
    }

    // returns true if system propagates for negative time durations
    bool canPropagateBackward() const override
    {
        return false;
    }
private:
    oc::StatePropagatorPtr systemSP_ = nullptr;
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
