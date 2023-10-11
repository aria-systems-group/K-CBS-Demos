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
#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>


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

class LinearizedUnicycleDynamics : public oc::StatePropagator 
{
    public:
        LinearizedUnicycleDynamics(const oc::SpaceInformationPtr &si) : oc::StatePropagator(si)
        {
            // discrete step size
            duration_ = si->getPropagationStepSize();
            
            // forward accel bounds
            forward_acceleration_bounds_.push_back(-0.5);
            forward_acceleration_bounds_.push_back(0.5);
            
            // turning rate bounds
            turning_rate_bounds_.push_back(-0.5);
            turning_rate_bounds_.push_back(0.5);
            
            // control params
            controller_parameters_.push_back(1.0);
            controller_parameters_.push_back(1.0);
            controller_parameters_.push_back(0.0);
            controller_parameters_.push_back(0.0);
            controller_parameters_.push_back(0.0);
            controller_parameters_.push_back(0.0);
            controller_parameters_.push_back(1.0);
            controller_parameters_.push_back(1.0);

            // fill gain matrix
            K_(0, 0) = 2.5857;
            K_(0, 1) = 3.4434;
            K_(0, 2) = 0.0;
            K_(0, 3) = 0.0;
            K_(1, 0) = 0.0;
            K_(1, 1) = 0.0;
            K_(1, 2) = 2.5857;
            K_(1, 3) = 3.4434;

            // define continuous time open-loop dynamics
            A_ol_ <<    0.0, 1.0, 0.0, 0.0,
                        0.0, 0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0, 1.0,
                        0.0, 0.0, 0.0, 0.0;
            B_ol_ <<    0.0, 0.0,
                        1.0, 0.0,
                        0.0, 0.0,
                        0.0, 1.0;

            // convert into discrete-time closed loop dynamics
            Eigen::Matrix4d A_ol_d_dt = A_ol_ * duration_;
            Eigen::Matrix4d A_ol_d = A_ol_d_dt.exp();

            Eigen::MatrixXd AB(A_ol_.rows(), A_ol_.cols()+B_ol_.cols());
            Eigen::MatrixXd zeroMat = Eigen::MatrixXd::Zero(A_ol_.cols()+B_ol_.cols()-A_ol_.rows(), A_ol_.cols()+B_ol_.cols());
            AB.leftCols(A_ol_.cols()) = A_ol_;
            AB.rightCols(B_ol_.cols()) = B_ol_;
            Eigen::MatrixXd AB0(AB.rows()+zeroMat.rows(), AB.cols());
            AB0.topRows(AB.rows()) = AB;
            AB0.bottomRows(zeroMat.rows()) = zeroMat;
            Eigen::MatrixXd AB0dt = duration_*AB0;
            Eigen::MatrixXd AB0exp = AB0dt.exp();
            Eigen::MatrixXd B_ol_d = AB0exp.rightCols(2).topRows(4);

            A_cl_d_ = A_ol_d - B_ol_d * K_;

            F_ = A_ol_d;

            // define the noise matrices
            double processNoise = 0.1; //process noise is 0.0 for scenario
            Q = pow(processNoise, 2) * Eigen::Matrix4d::Identity();
            double measurementNoise = 0.1;
            R = pow(measurementNoise, 2) * Eigen::Matrix4d::Identity();

        }

        virtual ~LinearizedUnicycleDynamics(void)
        {
            return;
        }

        virtual void propagate(const ob::State *start, const oc::Control* control, const double duration, ob::State *result) const
        {
            // deconstruct current state
            double x_pose, y_pose, yaw, surge;
            x_pose = start->as<ob::RealVectorStateSpace::StateType>()->values[0];
            y_pose = start->as<ob::RealVectorStateSpace::StateType>()->values[1];
            yaw = start->as<ob::RealVectorStateSpace::StateType>()->values[2];
            surge = start->as<ob::RealVectorStateSpace::StateType>()->values[3];
            
            // deconstruct the control (reference state)
            double x_pose_reference, y_pose_reference, yaw_reference, surge_reference;
            x_pose_reference = control->as<oc::RealVectorControlSpace::ControlType>()->values[0];
            y_pose_reference = control->as<oc::RealVectorControlSpace::ControlType>()->values[1];
            yaw_reference = control->as<oc::RealVectorControlSpace::ControlType>()->values[2];
            surge_reference = control->as<oc::RealVectorControlSpace::ControlType>()->values[3];

            // compute the control input
            double cos_y = cos(yaw);
            double sin_y = sin(yaw);
            double u_0 = controller_parameters_[0] * (x_pose_reference - x_pose) + controller_parameters_[1] * (surge_reference * cos(yaw_reference) - surge * cos_y);
            double u_1 = controller_parameters_[6] * (y_pose_reference - y_pose) + controller_parameters_[7] * (surge_reference * sin(yaw_reference) - surge * sin_y);

            double u_bar_0 = cos_y * u_0 + sin_y * u_1;
            double u_bar_1 = (-sin_y * u_0 + cos_y * u_1) / surge;

            // saturate the control inputs
            saturate(u_bar_0, forward_acceleration_bounds_[0], forward_acceleration_bounds_[1]);
            saturate(u_bar_1, turning_rate_bounds_[0], turning_rate_bounds_[1]);

            // compute resulting state
            double surge_final = surge + duration * u_bar_0;
            result->as<ob::RealVectorStateSpace::StateType>()->values[0] = x_pose + duration * cos_y * surge_final;
            result->as<ob::RealVectorStateSpace::StateType>()->values[1] = y_pose + duration * sin_y * surge_final;
            result->as<ob::RealVectorStateSpace::StateType>()->values[2] = wrap(yaw + duration * u_bar_1);
            result->as<ob::RealVectorStateSpace::StateType>()->values[3] = surge_final;
            
            return;
        }

        bool canPropagateBackward(void) const override
        {
            return false;
        }

    private:

        void saturate(double &value, const double min_value, const double max_value) const
        {
            if(value < min_value) value = min_value;
            if(value > max_value) value = max_value;
        }

        double wrap(double angle) const
        {
            double k = std::floor((M_PI - angle) / (2*M_PI));
            double alpha = angle + (2 * k * M_PI);
            return alpha;
        }

        // constants
        double duration_;
        std::vector<double> controller_parameters_, forward_acceleration_bounds_, turning_rate_bounds_;
        Eigen::Matrix4d A_ol_;
        Eigen::Matrix<double, 4, 2> B_ol_;
        Eigen::Matrix<double, 2, 4> K_;
        Eigen::Matrix4d A_cl_d_;

        Eigen::Matrix4d I_ = Eigen::Matrix4d::Identity();
        Eigen::Matrix4d H_ = Eigen::Matrix4d::Identity();
        Eigen::Matrix4d F_;
        

        Eigen::Matrix4d Q;
        Eigen::Matrix4d R;
};

