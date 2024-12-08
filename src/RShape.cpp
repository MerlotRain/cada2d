/**
 * Copyright (c) 2011-2018 by Andrew Mustun. All rights reserved.
 * 
 * This file is part of the QCAD project.
 *
 * QCAD is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * QCAD is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with QCAD.
 */
#include <cmath>
#include <typeinfo>

#include <cada2d/RArc.h>
#include <cada2d/RCircle.h>
#include <cada2d/REllipse.h>
#include <cada2d/RLine.h>
#include <cada2d/RMath.h>
#include <cada2d/RPolyline.h>
#include <cada2d/RRay.h>
#include <cada2d/RShape.h>
#include <cada2d/RSpline.h>
#include <cada2d/RXLine.h>

double RShape::getDistanceTo(const RVector& point, bool limited, double strictRange) const {
    RVector v = getVectorTo(point, limited, strictRange);
    if (v.isValid()) {
        return v.getMagnitude();
    }
    return RNANDOUBLE;
}

/**
 * \return Maximum shortest distance from this shape to any of the given points.
 */
double RShape::getMaxDistanceTo(const std::vector<RVector>& points, bool limited, double strictRange) const {
    double ret = 0.0;
    for (int i=0; i<points.size(); i++) {
        double d = getDistanceTo(points[i], limited, strictRange);
        ret = std::max(ret, d);
    }
    return ret;
}

/**
 * \return true if the given point is on this shape.
 */
bool RShape::isOnShape(const RVector& point, bool limited, double tolerance) const {
//    potential performance gain or loss:
//    if (limited) {
//        // point is well outside bounding box of shape:
//        RBox bbox = getBoundingBox();
//        bbox.grow(tolerance);
//        if (!bbox.contains(point)) {
//            return false;
//        }
//    }

    double d = getDistanceTo(point, limited);
    if (RMath::isNaN(d)) {
        return false;
    }
    // much more tolerance here (e.g. for ellipses):
    return d < tolerance;
}

/**
 * \return List of those points in pointList which are on this shape.
 */
std::vector<RVector> RShape::filterOnShape(const std::vector<RVector>& pointList, bool limited, double tolerance) const {
    std::vector<RVector> ret;
    for (int i=0; i<pointList.size(); i++) {
        if (isOnShape(pointList[i], limited, tolerance)) {
            ret.push_back(pointList[i]);
        }
    }
    return ret;
}

/**
 * \return Shortest vector from any end point of this shape
 *      to the given point.
 */
RVector RShape::getVectorFromEndpointTo(const RVector& point) const {
    std::vector<RVector> endPoints = getEndPoints();
    RVector closest = point.getClosest(endPoints);
    return point - closest;
}

/**
 * \return Point on this shape that is closest to p. Based on getVectorTo.
 */
RVector RShape::getClosestPointOnShape(const RVector& p, bool limited, double strictRange) const {
    RVector dv = getVectorTo(p, limited, strictRange);
    if (!dv.isValid()) {
        return RVector::invalid;
    }
    return p - dv;
}

bool RShape::equals(const RShape& other, double tolerance) const {
    if (getShapeType()!=other.getShapeType()) {
        return false;
    }

    return true;
}

RVector RShape::getPointOnShape() const {
    std::vector<RVector> midPoints = getMiddlePoints();
    if (midPoints.size()>0) {
        return midPoints[0];
    }

    std::vector<RVector> endPoints = getEndPoints();
    if (endPoints.size()>0) {
        return endPoints[0];
    }

    return getClosestPointOnShape(RVector(0.0,0.0));
}

/**
 * \return Point at given percentile.
 */
RVector RShape::getPointAtPercent(double p) const {
    double length = getLength();
    double distance = p * length;
    std::vector<RVector> candidates = getPointsWithDistanceToEnd(distance, RS::FromStart);
    if (candidates.size()!=1) {
        return RVector::invalid;
    }
    return candidates.at(0);
}

/**
 * \return Angle at given percentile.
 */
double RShape::getAngleAtPercent(double p) const {
    double length = getLength();
    double distance = p * length;
    return getAngleAt(distance);
}

/**
 * \return True if this shape intersects with the given shape.
 */
bool RShape::intersectsWith(const RShape& other, bool limited) const {
    return !getIntersectionPoints(other, limited).empty();
}

/**
 * \return The intersection point(s) between this shape and the given
 *      other shape.
 */
std::vector<RVector> RShape::getIntersectionPoints(const RShape& other,
        bool limited, bool same, bool force) const {
    return getIntersectionPoints(*this, other, limited, same, force);
}

std::vector<RVector> RShape::getIntersectionPoints(const RShape& shape1,
        const RShape& shape2, bool limited, bool same, bool force)
        {
            return std::vector<RVector>();
        }

bool RShape::flipHorizontal() {
    return mirror(RLine(RVector(0,0), RVector(0,1)));
}

bool RShape::flipVertical() {
    return mirror(RLine(RVector(0,0), RVector(1,0)));
}

bool RShape::stretch(const RBox& area, const RVector& offset) {
    return stretch(area.getPolyline(), offset);
}

bool RShape::stretch(const RPolyline& area, const RVector& offset) {
    bool ret = false;

    if (area.containsShape(*this)) {
        // whole shape inside stretch area:
        return move(offset);
    }

    return ret;
}

bool RShape::scale(double scaleFactor, const RVector& center) {
    return scale(RVector(scaleFactor, scaleFactor, scaleFactor), center);
}

std::vector<std::shared_ptr<RShape> > RShape::getOffsetShapes(double distance, int number, RS::Side side, const RVector& position) {
    return std::vector<std::shared_ptr<RShape> >();
}

std::vector<std::shared_ptr<RShape> > RShape::splitAt(const std::vector<RVector>& points) const {
    std::vector<std::shared_ptr<RShape> > ret;
    return ret;
}