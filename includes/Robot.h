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
#include <boost/geometry/geometries/point_xyz.hpp>
#include <boost/geometry/algorithms/correct.hpp>
#include <boost/geometry/strategies/transform/matrix_transformers.hpp>
#include <stdio.h>

typedef boost::geometry::model::d2::point_xy<double> BoostPoint;
typedef boost::geometry::model::polygon<BoostPoint, false, true> BoostPolygon;
typedef boost::geometry::model::d3::point_xyz<double> BoostPoint3D;
typedef boost::geometry::model::polygon<BoostPoint3D, false, true> BoostPolygon3D;

/*
* ***************************************************************
* ********************* 2D Robot Definitions ********************
* ***************************************************************
*/

// A Robot has a name and shape
class Robot
{
public:
    Robot(std::string name): name_(name) {};
    /** \brief Cast this instance to a desired type. */
    template <class T>
    T *as()
    {
        /** \brief Make sure the type we are casting to is indeed a robot */
        BOOST_CONCEPT_ASSERT((boost::Convertible<T *, Robot *>));
        return static_cast<T *>(this);
    }
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
    void setDynamics(std::string s) {dynamics_ = s;}
    std::string getDynamics() const {return dynamics_;};
protected:
    const std::string name_;
    BoostPolygon shape_; // assumed to be centered at origin
    double bounding_radius_;
    std::string dynamics_;
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

class CompoundRobot: public Robot
{
public:
    CompoundRobot(std::string name, Robot* robot1, Robot* robot2):
        robot1_(robot1), robot2_(robot2), Robot(name) {}
    Robot* getRobot(unsigned int idx)
    {
        if (idx == 0)
            return robot1_;
        else if (idx == 1)
            return robot2_;
        return nullptr;
    }
private:
    Robot* robot1_;
    Robot* robot2_;
};

/*
* ***************************************************************
* ********************* 3D Robot Definitions ********************
* ***************************************************************
*/

// a rectangular robot class in 3D
/*
// IMPORTANT NOTE: Apparently, Boost does not support intersection / disjoint operations
	with 3d points (e.g. https://stackoverflow.com/questions/49006155/how-to-intersection-to-3d-polygons-by-boost-c-library).
	Therefore, this class still contains a 2D polygon, but the robot has a height h>0 that is used in collision checking
	rather than simply checking the intersection of two 3D polygons. height is defined as the "max vertical distance from center
    of the robot." (e.g. a 1x1x1 robot would have height == 0.5)
*/
class RectangularRobot3D: public RectangularRobot 
{
public:
    RectangularRobot3D(std::string name, const double length, const double width, const double height):
        height_(height), RectangularRobot(name, length, width)
    {
        
    };
    const double getHeight() const {return height_;};
private:
    const double height_;
    BoostPolygon3D shape3D_;
};



