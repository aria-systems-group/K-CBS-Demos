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

#pragma once
#include <boost/geometry/geometries/geometries.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/algorithms/correct.hpp>
#include <boost/geometry/strategies/transform/matrix_transformers.hpp>
#include <stdio.h>

typedef boost::geometry::model::d2::point_xy<double> BoostPoint;
typedef boost::geometry::model::polygon<BoostPoint, false, true> BoostPolygon;

// A Robot has a name and shape
class Robot
{
public:
    Robot(std::string name): name_(name) {};
    const std::string getName() const {return name_;};
    const BoostPolygon& getShape() const {return shape_;};
    const double getBoundingRadius() const {return bounding_radius_;};
    void printShape() const
    {
        auto exterior_points = boost::geometry::exterior_ring(shape_);
        printf("Printing shape for %s\n", name_.c_str());
        for (int i=0; i<exterior_points.size(); i++) 
        {
            printf("\t- Point(%0.2f, %0.2f)\n", boost::geometry::get<0>(exterior_points[i]), boost::geometry::get<1>(exterior_points[i]));
        }
    }
protected:
    const std::string name_;
    BoostPolygon shape_; // assumed to be centered at origin
    double bounding_radius_;
};

// a rectangular robot class
class RectangularRobot: public Robot 
{
public:
    RectangularRobot(std::string name, const double length, const double width):
        length_(length), width_(width), Robot(name) 
    {
        BoostPoint bott_left    ((-length_ / 2),   (-width_ / 2));
        BoostPoint bott_right   ((length_ / 2),    (-width_ / 2));
        BoostPoint top_left     ((-length_ / 2),   (width_ / 2));
        BoostPoint top_right    ((length_ / 2),    (width_ / 2));

        boost::geometry::append(shape_.outer(), bott_left);
        boost::geometry::append(shape_.outer(), bott_right);
        boost::geometry::append(shape_.outer(), top_right);
        boost::geometry::append(shape_.outer(), top_left);
        boost::geometry::append(shape_.outer(), bott_left);
        // assures that the polygon (1) has counter-clockwise points, and (2) is closed
        boost::geometry::correct(shape_);

        // set bounding radius of a RectangularRobot
        bounding_radius_ = sqrt( pow((length_ / 2), 2) + pow((width_ / 2), 2));
    };
private:
    const double length_;
    const double width_;
};


