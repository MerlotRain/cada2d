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

BSpline::BSpline()
    : mDegree(3), mPeriodic(false), mDirty(true), mUpdateInProgress(false),
      mLength(std::numeric_limits<double>::quiet_NaN())
{
}

BSpline::BSpline(const std::vector<Vec2d> &controlPoints, int degree)
    : mControlPoints(controlPoints), mDegree(degree), mPeriodic(false),
      mDirty(true), mUpdateInProgress(false),
      mLength(std::numeric_limits<double>::quiet_NaN())
{
}

void BSpline::appendControlPoint(const Vec2d &point)
{
    mControlPoints.push_back(point);
    update();
}

void BSpline::appendControlPoints(const std::vector<Vec2d> &points)
{
    mControlPoints.insert(mControlPoints.end(), points.begin(), points.end());
    update();
}

void BSpline::removeLastControlPoint()
{
    mControlPoints.pop_back();
    update();
}

void BSpline::setControlPoints(const std::vector<Vec2d> &controlPoints)
{
    this->mControlPoints = controlPoints;
    update();
}

std::vector<Vec2d> BSpline::getControlPoints() const
{
    return mControlPoints;
}

std::vector<Vec2d> BSpline::getControlPointsWrapped() const
{
    std::vector<Vec2d> ret;

    updateInternal();

    return ret;
}

int BSpline::countControlPoints() const
{
    return mControlPoints.size();
}

Vec2d BSpline::getControlPointAt(int i) const
{
    if (i >= 0 && i < mControlPoints.size()) {
        return mControlPoints.at(i);
    }
    return Vec2d::invalid;
}

void BSpline::appendFitPoint(const Vec2d &point)
{
    mFitPoints.push_back(point);
    update();
}

void BSpline::prependFitPoint(const Vec2d &point)
{
}

void BSpline::insertFitPointAt(const Vec2d &point)
{
    Vec2d p = getClosestPointOnShape(point);

    // find out T at the point closest to point:
    double t = getTAtPoint(p);

    insertFitPointAt(t, p);
}

void BSpline::insertFitPointAt(double t, const Vec2d &p)
{
}

void BSpline::removeFitPointAt(const Vec2d &point)
{
}

void BSpline::removeLastFitPoint()
{
}

void BSpline::removeFirstFitPoint()
{
}

void BSpline::setFitPoints(const std::vector<Vec2d> &fitPoints)
{
    this->mFitPoints = mFitPoints;
    update();
}

std::vector<Vec2d> BSpline::getFitPoints() const
{
    return mFitPoints;
}

int BSpline::countFitPoints() const
{
    return mFitPoints.size();
}

bool BSpline::hasFitPoints() const
{
    return !mFitPoints.empty();
}

Vec2d BSpline::getFitPointAt(int i) const
{
    if (i >= 0 && i < mFitPoints.size()) {
        return mFitPoints.at(i);
    }
    return Vec2d::invalid;
}

std::vector<double> BSpline::getKnotVector() const
{
    return mKnotVector;
}

std::vector<double> BSpline::getActualKnotVector() const
{
    return std::vector<double>();
}

void BSpline::setKnotVector(const std::vector<double> &knots)
{
    mKnotVector = knots;
    update();
}

void BSpline::appendKnot(double k)
{
    mKnotVector.push_back(k);
    update();
}

std::vector<double> BSpline::getWeights() const
{
    return mWeights;
}

void BSpline::setWeights(std::vector<double> &w)
{
    mWeights = w;
}

void BSpline::setDegree(int d)
{
    mDegree = d;
    update();
}

int BSpline::getDegree() const
{
    return mDegree;
}

int BSpline::getOrder() const
{
    return mDegree + 1;
}

void BSpline::setPeriodic(bool on)
{
    mPeriodic = on;
    update();
}

bool BSpline::isClosed() const
{
    return mPeriodic;
}

bool BSpline::isGeometricallyClosed(double tolerance) const
{
    return isClosed() ||
           getStartPoint().getDistanceTo(getEndPoint()) < tolerance;
}

bool BSpline::isPeriodic() const
{
    return mPeriodic;
}

Vec2d BSpline::getStartPoint() const
{
    return getPointAt(getTMin());
}

void BSpline::setStartPoint(const Vec2d &v)
{
    mControlPoints[0] = v;
    update();
}

Vec2d BSpline::getEndPoint() const
{
    return getPointAt(getTMax());
}

void BSpline::setEndPoint(const Vec2d &v)
{
    mControlPoints[mControlPoints.size() - 1] = v;
    update();
}

void BSpline::setTangents(const Vec2d &start, const Vec2d &end)
{
    mTangentStart = start;
    mTangentEnd = end;
    update();
}

void BSpline::setTangentAtStart(const Vec2d &t)
{
    mTangentStart = t;
    update();
}

Vec2d BSpline::getTangentAtStart() const
{
    return mTangentStart;
}

void BSpline::setTangentAtEnd(const Vec2d &t)
{
    mTangentEnd = t;
    update();
}

Vec2d BSpline::getTangentAtEnd() const
{
    return mTangentEnd;
}

void BSpline::unsetTangentAtStart()
{
    setTangentAtStart(Vec2d::invalid);
}

void BSpline::unsetTangentAtEnd()
{
    setTangentAtEnd(Vec2d::invalid);
}

void BSpline::unsetTangents()
{
    setTangents(Vec2d::invalid, Vec2d::invalid);
}

void BSpline::updateTangentsPeriodic()
{
    unsetTangents();

    double tangent1 = getDirection1();
    double tangent2 = Math::getNormalizedAngle(getDirection2() + M_PI);
    Vec2d v1 = Vec2d::createPolar(1.0, tangent1);
    Vec2d v2 = Vec2d::createPolar(1.0, tangent2);
    Vec2d t = (v1 + v2).getNormalized();
    setTangents(t, t);
}

std::unique_ptr<Polyline> BSpline::approximateWithArcs(double tolerance,
                                                       double radiusLimit) const
{
    return std::unique_ptr<Polyline>();
}

std::unique_ptr<Polyline> BSpline::toPolyline(int segments) const
{
    return std::unique_ptr<Polyline>();
}

std::vector<std::unique_ptr<Shape>>
BSpline::getExplodedBezier(int segments) const
{
    std::vector<std::unique_ptr<Shape>> ret;
    return ret;
}

void BSpline::appendToExploded(const Line *line) const
{
}

std::vector<std::unique_ptr<Shape>>
BSpline::getExplodedWithSegmentLength(double segmentLength) const
{
    std::vector<std::unique_ptr<Shape>> ret;
    return ret;
}

Vec2d BSpline::getPointAt(double t) const
{
    updateInternal();
    return Vec2d::invalid;
}

Vec2d BSpline::getPointAtDistance(double distance) const
{
    double t = getTAtDistance(distance);
    return getPointAt(t);
}

std::vector<Vec2d> BSpline::getEndPoints() const
{
    std::vector<Vec2d> ret;

    ret.push_back(getStartPoint());
    ret.push_back(getEndPoint());

    return ret;
}

Vec2d BSpline::getMiddlePoint() const
{
    return getPointAt(getTMin() + (getTDelta() / 2.0));
}

std::vector<Vec2d> BSpline::getMiddlePoints() const
{
    std::vector<Vec2d> ret;

    ret.push_back(getMiddlePoint());

    return ret;
}

std::vector<Vec2d> BSpline::getCenterPoints() const
{
    return std::vector<Vec2d>();
}

bool BSpline::isValid() const
{
    return false;
}

NS::ShapeType BSpline::getShapeType() const
{
    return NS::BSpline;
}

BSpline *BSpline::cloneImpl() const
{
    return nullptr;
}

double BSpline::getTDelta() const
{
    return getTMax() - getTMin();
}

double BSpline::getTMin() const
{
    updateInternal();
    return 0.0;
}

double BSpline::getTMax() const
{
    updateInternal();
    return 0.0;
}

double BSpline::getTAtPoint(const Vec2d &point) const
{
    return 0.0;
}

double BSpline::getTAtDistance(double distance) const
{
    return 0.0;
}

double BSpline::getDistanceAtT(double t) const
{
    return 0.0;
}

std::vector<BSpline>
BSpline::getSegments(const std::vector<Vec2d> &points) const
{
    return std::vector<BSpline>();
}

std::vector<Vec2d> BSpline::getDiscontinuities() const
{
    updateInternal();

    std::vector<Vec2d> ret;
    return ret;
}

BSpline *BSpline::simplify(double tolerance)
{
    return nullptr;
}

void BSpline::invalidate() const
{
    mExploded.clear();
    mLength = std::numeric_limits<double>::quiet_NaN();
}

void BSpline::updateInternal() const
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

void BSpline::updateFromControlPoints() const
{
}

void BSpline::updateFromFitPoints() const
{
}

void BSpline::updateBoundingBox() const
{
}

std::vector<BSpline> BSpline::getBezierSegments(const BBox &queryBox) const
{
    std::vector<BSpline> ret;
    return ret;
}

void BSpline::update() const
{
    mDirty = true;
    mBoundingBox = BBox();
    mExploded.clear();
}

bool BSpline::isDirty() const
{
    return false;
}

std::string BSpline::to_string() const
{
    std::stringstream ss;
    ss << std::fixed << std::setprecision(6);
    ss << "BSpline: ";
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