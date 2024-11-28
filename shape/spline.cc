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

#include "cada_shape.h"

#include <cmath>
#include <sstream>
#include <iomanip>

namespace cada {
namespace shape {

Spline::Spline()
    : mDegree(3), mPeriodic(false), mDirty(true), mUpdateInProgress(false),
      mLength(std::numeric_limits<double>::quiet_NaN())
{
}

Spline::Spline(const std::vector<Vec2d> &controlPoints, int degree)
    : mControlPoints(controlPoints), mDegree(degree), mPeriodic(false),
      mDirty(true), mUpdateInProgress(false),
      mLength(std::numeric_limits<double>::quiet_NaN())
{
}

void Spline::appendControlPoint(const Vec2d &point)
{
    mControlPoints.push_back(point);
    update();
}

void Spline::appendControlPoints(const std::vector<Vec2d> &points)
{
    mControlPoints.insert(mControlPoints.end(), points.begin(), points.end());
    update();
}

void Spline::removeLastControlPoint()
{
    mControlPoints.pop_back();
    update();
}

void Spline::setControlPoints(const std::vector<Vec2d> &controlPoints)
{
    this->mControlPoints = controlPoints;
    update();
}

std::vector<Vec2d> Spline::getControlPoints() const
{
    return mControlPoints;
}

std::vector<Vec2d> Spline::getControlPointsWrapped() const
{
    std::vector<Vec2d> ret;

    updateInternal();

    return ret;
}

int Spline::countControlPoints() const
{
    return mControlPoints.size();
}

Vec2d Spline::getControlPointAt(int i) const
{
    if (i >= 0 && i < mControlPoints.size()) {
        return mControlPoints.at(i);
    }
    return Vec2d::invalid;
}

void Spline::appendFitPoint(const Vec2d &point)
{
    mFitPoints.push_back(point);
    update();
}

void Spline::prependFitPoint(const Vec2d &point)
{
}

void Spline::insertFitPointAt(const Vec2d &point)
{
    Vec2d p = getClosestPointOnShape(point);

    // find out T at the point closest to point:
    double t = getTAtPoint(p);

    insertFitPointAt(t, p);
}

void Spline::insertFitPointAt(double t, const Vec2d &p)
{
}

void Spline::removeFitPointAt(const Vec2d &point)
{
}

void Spline::removeLastFitPoint()
{
}

void Spline::removeFirstFitPoint()
{
}

void Spline::setFitPoints(const std::vector<Vec2d> &fitPoints)
{
    this->mFitPoints = mFitPoints;
    update();
}

std::vector<Vec2d> Spline::getFitPoints() const
{
    return mFitPoints;
}

int Spline::countFitPoints() const
{
    return mFitPoints.size();
}

bool Spline::hasFitPoints() const
{
    return !mFitPoints.empty();
}

Vec2d Spline::getFitPointAt(int i) const
{
    if (i >= 0 && i < mFitPoints.size()) {
        return mFitPoints.at(i);
    }
    return Vec2d::invalid;
}

std::vector<double> Spline::getKnotVector() const
{
    return mKnotVector;
}

std::vector<double> Spline::getActualKnotVector() const
{
    return std::vector<double>();
}

void Spline::setKnotVector(const std::vector<double> &knots)
{
    mKnotVector = knots;
    update();
}

void Spline::appendKnot(double k)
{
    mKnotVector.push_back(k);
    update();
}

std::vector<double> Spline::getWeights() const
{
    return mWeights;
}

void Spline::setWeights(std::vector<double> &w)
{
    mWeights = w;
}

void Spline::setDegree(int d)
{
    mDegree = d;
    update();
}

int Spline::getDegree() const
{
    return mDegree;
}

int Spline::getOrder() const
{
    return mDegree + 1;
}

void Spline::setPeriodic(bool on)
{
    mPeriodic = on;
    update();
}

bool Spline::isClosed() const
{
    return mPeriodic;
}

bool Spline::isGeometricallyClosed(double tolerance) const
{
    return isClosed() ||
           getStartPoint().getDistanceTo(getEndPoint()) < tolerance;
}

bool Spline::isPeriodic() const
{
    return mPeriodic;
}

Vec2d Spline::getStartPoint() const
{
    return getPointAt(getTMin());
}

void Spline::setStartPoint(const Vec2d &v)
{
    mControlPoints[0] = v;
    update();
}

Vec2d Spline::getEndPoint() const
{
    return getPointAt(getTMax());
}

void Spline::setEndPoint(const Vec2d &v)
{
    mControlPoints[mControlPoints.size() - 1] = v;
    update();
}

void Spline::setTangents(const Vec2d &start, const Vec2d &end)
{
    mTangentStart = start;
    mTangentEnd = end;
    update();
}

void Spline::setTangentAtStart(const Vec2d &t)
{
    mTangentStart = t;
    update();
}

Vec2d Spline::getTangentAtStart() const
{
    return mTangentStart;
}

void Spline::setTangentAtEnd(const Vec2d &t)
{
    mTangentEnd = t;
    update();
}

Vec2d Spline::getTangentAtEnd() const
{
    return mTangentEnd;
}

void Spline::unsetTangentAtStart()
{
    setTangentAtStart(Vec2d::invalid);
}

void Spline::unsetTangentAtEnd()
{
    setTangentAtEnd(Vec2d::invalid);
}

void Spline::unsetTangents()
{
    setTangents(Vec2d::invalid, Vec2d::invalid);
}

void Spline::updateTangentsPeriodic()
{
    unsetTangents();

    double tangent1 = getDirection1();
    double tangent2 = Math::getNormalizedAngle(getDirection2() + M_PI);
    Vec2d v1 = Vec2d::createPolar(1.0, tangent1);
    Vec2d v2 = Vec2d::createPolar(1.0, tangent2);
    Vec2d t = (v1 + v2).getNormalized();
    setTangents(t, t);
}

std::unique_ptr<Polyline> Spline::approximateWithArcs(double tolerance,
                                                      double radiusLimit) const
{
    return std::unique_ptr<Polyline>();
}

std::unique_ptr<Polyline> Spline::toPolyline(int segments) const
{
    return std::unique_ptr<Polyline>();
}

std::vector<std::unique_ptr<Shape>>
Spline::getExplodedBezier(int segments) const
{
    std::vector<std::unique_ptr<Shape>> ret;
    return ret;
}

void Spline::appendToExploded(const Line *line) const
{
}

std::vector<std::unique_ptr<Shape>>
Spline::getExplodedWithSegmentLength(double segmentLength) const
{
    std::vector<std::unique_ptr<Shape>> ret;
    return ret;
}

Vec2d Spline::getPointAt(double t) const
{
    updateInternal();
    return Vec2d::invalid;
}

Vec2d Spline::getPointAtDistance(double distance) const
{
    double t = getTAtDistance(distance);
    return getPointAt(t);
}

std::vector<Vec2d> Spline::getEndPoints() const
{
    std::vector<Vec2d> ret;

    ret.push_back(getStartPoint());
    ret.push_back(getEndPoint());

    return ret;
}

Vec2d Spline::getMiddlePoint() const
{
    return getPointAt(getTMin() + (getTDelta() / 2.0));
}

std::vector<Vec2d> Spline::getMiddlePoints() const
{
    std::vector<Vec2d> ret;

    ret.push_back(getMiddlePoint());

    return ret;
}

std::vector<Vec2d> Spline::getCenterPoints() const
{
    return std::vector<Vec2d>();
}

std::vector<std::unique_ptr<Spline>> Spline::createSplineFromArc(const Arc *arc)
{
    return std::vector<std::unique_ptr<Spline>>();
}

bool Spline::isValid() const
{
    return false;
}

NS::ShapeType Spline::getShapeType() const
{
    return NS::Spline;
}

Spline *Spline::cloneImpl() const
{
    return nullptr;
}

double Spline::getTDelta() const
{
    return getTMax() - getTMin();
}

double Spline::getTMin() const
{
    updateInternal();
    return 0.0;
}

double Spline::getTMax() const
{
    updateInternal();
    return 0.0;
}

double Spline::getTAtPoint(const Vec2d &point) const
{
    return 0.0;
}

double Spline::getTAtDistance(double distance) const
{
    return 0.0;
}

double Spline::getDistanceAtT(double t) const
{
    return 0.0;
}

std::vector<Spline> Spline::getSegments(const std::vector<Vec2d> &points) const
{
    return std::vector<Spline>();
}

std::vector<Vec2d> Spline::getDiscontinuities() const
{
    updateInternal();

    std::vector<Vec2d> ret;
    return ret;
}

Spline *Spline::simplify(double tolerance)
{
    return nullptr;
}

void Spline::invalidate() const
{
    mExploded.clear();
    mLength = std::numeric_limits<double>::quiet_NaN();
}

void Spline::updateInternal() const
{
    if (!mDirty || mUpdateInProgress) {
        return;
    }

    mDirty = false;
    mUpdateInProgress = true;

    if (mDegree < 1) {
        invalidate();
        mUpdateInProgress = false;
        return;
    }

    mExploded.clear();
    mLength = std::numeric_limits<double>::quiet_NaN();

    if (mFitPoints.size() == 0) {
        updateFromControlPoints();
    }
    else {
        updateFromFitPoints();
    }

    mBoundingBox = BBox();
    mUpdateInProgress = false;
}

void Spline::updateFromControlPoints() const
{
}

void Spline::updateFromFitPoints() const
{
}

void Spline::updateBoundingBox() const
{
}

std::vector<Spline> Spline::getBezierSegments(const BBox &queryBox) const
{
    std::vector<Spline> ret;
    return ret;
}

void Spline::update() const
{
    mDirty = true;
    mBoundingBox = BBox();
    mExploded.clear();
}

bool Spline::isDirty() const
{
    return false;
}

std::string Spline::to_string() const
{
    std::stringstream ss;
    ss << std::fixed << std::setprecision(6);
    ss << "Spline: ";
    ss << "degree: " << mDegree << ", ";
    ss << "controlPoints: " << mControlPoints.size() << ", ";
    for (auto &cp : mControlPoints) {
        ss << cp.to_string() << ", ";
    }
    ss << "weights: " << mWeights.size() << ", ";
    for (auto &w : mWeights) {
        ss << w << ", ";
    }
    return ss.str();
}

} // namespace shape
} // namespace cada