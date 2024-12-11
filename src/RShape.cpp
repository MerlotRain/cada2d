/**
 * Copyright (c) 2024-present Merlot.Rain
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to
 * deal in the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
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

double RShape::getMaxDistanceTo(const std::vector<RVector>& points, bool limited, double strictRange) const {
    double ret = 0.0;
    for (int i=0; i<points.size(); i++) {
        double d = getDistanceTo(points[i], limited, strictRange);
        ret = std::max(ret, d);
    }
    return ret;
}

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

std::vector<RVector> RShape::filterOnShape(const std::vector<RVector>& pointList, bool limited, double tolerance) const {
    std::vector<RVector> ret;
    for (int i=0; i<pointList.size(); i++) {
        if (isOnShape(pointList[i], limited, tolerance)) {
            ret.push_back(pointList[i]);
        }
    }
    return ret;
}

RVector RShape::getVectorFromEndpointTo(const RVector& point) const {
    std::vector<RVector> endPoints = getEndPoints();
    RVector closest = point.getClosest(endPoints);
    return point - closest;
}

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

RVector RShape::getPointAtPercent(double p) const {
    double length = getLength();
    double distance = p * length;
    std::vector<RVector> candidates = getPointsWithDistanceToEnd(distance, RS::FromStart);
    if (candidates.size()!=1) {
        return RVector::invalid;
    }
    return candidates.at(0);
}

double RShape::getAngleAtPercent(double p) const {
    double length = getLength();
    double distance = p * length;
    return getAngleAt(distance);
}

bool RShape::intersectsWith(const RShape& other, bool limited) const {
    return !getIntersectionPoints(other, limited).empty();
}

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