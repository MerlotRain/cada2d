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

#include <cada_shape.h>
#include <cmath>

namespace cada {
namespace shape {

Arc::Arc()
    : mCenter(Vec2d::invalid), mRadius(0.0), mStartAngle(0.0), mEndAngle(0.0),
      mReversed(false)
{
}

Arc::Arc(double cx, double cy, double radius, double startAngle,
         double endAngle, bool reversed)
    : mCenter(cx, cy), mRadius(radius), mStartAngle(startAngle),
      mEndAngle(endAngle), mReversed(reversed)
{
}

Arc::Arc(const Vec2d &center, double radius, double startAngle, double endAngle,
         bool reversed)
    : mCenter(center), mRadius(radius), mStartAngle(startAngle),
      mEndAngle(endAngle), mReversed(reversed)
{
}

bool Arc::isValid() const
{
    return mCenter.isValid() && mRadius > 0.0;
}

NS::ShapeType Arc::getShapeType() const
{
    return NS::Arc;
}

Arc *Arc::cloneImpl() const
{
    Arc *pClone = new Arc();
    pClone->mCenter = mCenter;
    pClone->mRadius = mRadius;
    pClone->mStartAngle = mStartAngle;
    pClone->mEndAngle = mEndAngle;
    pClone->mReversed = mReversed;
    return pClone;
}

bool Arc::isFullCircle(double tolerance) const
{
    return fabs(Math::getAngleDifference180(
               Math::getNormalizedAngle(mStartAngle),
               Math::getNormalizedAngle(mEndAngle))) < tolerance;
}

bool Arc::isAngleWithinArc(double a) const
{
    return Math::isAngleBetween(a, mStartAngle, mEndAngle, mReversed);
}

double Arc::getBulge() const
{
    double bulge = tan(fabs(getSweep()) / 4.0);
    if (isReversed()) {
        bulge *= -1;
    }
    return bulge;
}

double Arc::getDiameter() const
{
    return 2 * mRadius;
}

void Arc::setDiameter(double d)
{
    mRadius = d / 2.0;
}

void Arc::setLength(double l)
{
    double sweep = l / mRadius;
    if (sweep > 2 * M_PI) {
        sweep = 2 * M_PI;
    }
    if (mReversed) {
        sweep *= -1;
    }

    mEndAngle = mStartAngle + sweep;
}

double Arc::getArea() const
{
    return (mRadius * mRadius * getAngleLength(false)) / 2.0;
}

void Arc::setArea(double a)
{
    double sweep = (a * 2.0) / (mRadius * mRadius);
    if (mReversed) {
        mEndAngle = Math::getNormalizedAngle(mStartAngle - sweep);
    }
    else {
        mEndAngle = Math::getNormalizedAngle(mStartAngle + sweep);
    }
}

double Arc::getChordArea() const
{
    double sectorArea = 0.0;
    double angleLength = getAngleLength(false);
    double sweep = getSweep();
    if (sweep < M_PI) {
        sectorArea =
            ((mRadius * mRadius) * (angleLength - sin(angleLength))) / 2.0;
    }
    else if (sweep == M_PI) {
        sectorArea = 0.5 * (M_PI * mRadius * mRadius);
    }
    else {
        double remainAngle = (M_PI * 2) - sweep;
        double remainSliceArea = (mRadius * mRadius * remainAngle) / 2.0;
        double remainSectorArea =
            (mRadius * mRadius * (remainAngle - sin(remainAngle))) / 2.0;
        sectorArea = getArea() + (remainSliceArea - remainSectorArea);
    }

    return sectorArea;
}

double Arc::getAngleLength(bool allowForZeroLength) const
{
    double ret = fabs(getSweep());

    // full circle or zero length arc:
    if (!allowForZeroLength) {
        if (ret < NS::PointTolerance) {
            ret = 2 * M_PI;
        }
    }
    else {
        if (ret > 2 * M_PI - NS::PointTolerance) {
            ret = 0.0;
        }
    }

    return ret;
}

double Arc::getSweep() const
{
    double ret = 0.0;

    if (mReversed) {
        if (mStartAngle <= mEndAngle) {
            ret = -(mStartAngle + 2 * M_PI - mEndAngle);
        }
        else {
            ret = -(mStartAngle - mEndAngle);
        }
    }
    else {
        if (mEndAngle <= mStartAngle) {
            ret = mEndAngle + 2 * M_PI - mStartAngle;
        }
        else {
            ret = mEndAngle - mStartAngle;
        }
    }

    return ret;
}

void Arc::setSweep(double s)
{
    mEndAngle = mStartAngle + s;
    mReversed = (s < 0.0);
}

Vec2d Arc::getCenter() const
{
    return mCenter;
}

void Arc::setCenter(const Vec2d &vector)
{
    mCenter = vector;
}

double Arc::getRadius() const
{
    return mRadius;
}

void Arc::setRadius(double r)
{
    mRadius = r;
}

double Arc::getStartAngle() const
{
    return mStartAngle;
}

void Arc::setStartAngle(double a)
{
    mStartAngle = Math::getNormalizedAngle(a);
}

double Arc::getEndAngle() const
{
    return mEndAngle;
}

void Arc::setEndAngle(double a)
{
    mEndAngle = Math::getNormalizedAngle(a);
}

Vec2d Arc::getMiddlePoint() const
{
    double a;
    a = mStartAngle + getSweep() / 2.0;
    Vec2d v = Vec2d::createPolar(mRadius, a);
    v += mCenter;
    return v;
}

Vec2d Arc::getStartPoint() const
{
    return getPointAtAngle(mStartAngle);
}

Vec2d Arc::getEndPoint() const
{
    return getPointAtAngle(mEndAngle);
}

Vec2d Arc::getPointAtAngle(double a) const
{
    return Vec2d(mCenter.x + cos(a) * mRadius, mCenter.y + sin(a) * mRadius);
}

bool Arc::isReversed() const
{
    return mReversed;
}

void Arc::setReversed(bool r)
{
    mReversed = r;
}

std::vector<Vec2d> Arc::getEndPoints() const
{
    std::vector<Vec2d> ret;
    ret.push_back(getStartPoint());
    ret.push_back(getEndPoint());
    return ret;
}

std::vector<Vec2d> Arc::getMiddlePoints() const
{
    std::vector<Vec2d> ret;
    ret.push_back(getMiddlePoint());
    return ret;
}

std::vector<Vec2d> Arc::getCenterPoints() const
{
    std::vector<Vec2d> ret;
    ret.push_back(getCenter());
    return ret;
}

std::vector<Vec2d> Arc::getArcRefPoints() const
{
    std::vector<Vec2d> ret;

    std::vector<Vec2d> p;
    p.push_back(mCenter + Vec2d(mRadius, 0));
    p.push_back(mCenter + Vec2d(0, mRadius));
    p.push_back(mCenter - Vec2d(mRadius, 0));
    p.push_back(mCenter - Vec2d(0, mRadius));

    for (size_t i = 0; i < p.size(); i++) {
        if (Math::isAngleBetween(mCenter.getAngleTo(p[i]), mStartAngle,
                                 mEndAngle, mReversed)) {
            ret.push_back(p[i]);
        }
    }

    return ret;
}

std::unique_ptr<Polyline> Arc::approximateWithLines(double segmentLength,
                                                    double angle) const
{
    std::unique_ptr<Polyline> polyline =
        ShapeFactory::instance()->createPolyline();

    double aStep;
    if (segmentLength < NS::PointTolerance && angle > NS::PointTolerance) {
        aStep = angle;
    }
    else {
        // avoid a segment length of 0:
        if (segmentLength > 0.0 && segmentLength < 1.0e-6) {
            segmentLength = 1.0e-6;
        }
        if (segmentLength > 0.0) {
            aStep = segmentLength / mRadius;
        }
        else {
            // negative segment length: auto:
            aStep = 1.0;
        }
    }

    double a1 = getStartAngle();
    double a2 = getEndAngle();
    double a, cix, ciy;

    polyline->appendVertex(getStartPoint());
    if (!mReversed) {
        // Arc Counterclockwise:
        if (a1 > a2 - 1.0e-10) {
            a2 += 2 * M_PI;
        }
        for (a = a1 + aStep; a <= a2; a += aStep) {
            cix = mCenter.x + cos(a) * mRadius;
            ciy = mCenter.y + sin(a) * mRadius;
            polyline->appendVertex(Vec2d(cix, ciy));
        }
    }
    else {
        // Arc Clockwise:
        if (a1 < a2 + 1.0e-10) {
            a2 -= 2 * M_PI;
        }
        for (a = a1 - aStep; a >= a2; a -= aStep) {
            cix = mCenter.x + cos(a) * mRadius;
            ciy = mCenter.y + sin(a) * mRadius;
            polyline->appendVertex(Vec2d(cix, ciy));
        }
    }
    polyline->appendVertex(getEndPoint());

    return polyline;
}

std::unique_ptr<Polyline> Arc::approximateWithLinesTan(double segmentLength,
                                                       double angle) const
{
    std::unique_ptr<Polyline> polyline =
        ShapeFactory::instance()->createPolyline();

    double aStep;
    if (segmentLength < NS::PointTolerance && angle > NS::PointTolerance) {
        aStep = angle;
        double sw = fabs(getSweep());
        if (aStep > sw) {
            // make sure aStep is not too large for arc:
            aStep = sw / 2;
        }
    }
    else {
        // avoid a segment length of 0:
        if (segmentLength < 1.0e-6) {
            segmentLength = 1.0e-6;
        }

        // ideal angle step to satisfy segmentLength:
        aStep = segmentLength / mRadius;

        int steps = ceil(fabs(getSweep()) / aStep);
        // real angle step:
        aStep = fabs(getSweep()) / steps;
        if (fabs(cos(aStep / 2)) < NS::PointTolerance) {
            polyline->appendVertex(getStartPoint());
            polyline->appendVertex(getEndPoint());
            return polyline;
        }
    }

    double r2 = mRadius / cos(aStep / 2);

    double a1 = getStartAngle();
    double a2 = getEndAngle();

    double a, cix, ciy;

    polyline->appendVertex(getStartPoint());
    if (!mReversed) {
        // Arc Counterclockwise:
        if (a1 > a2 - 1.0e-10) {
            a2 += 2 * M_PI;
        }
        for (a = a1 + aStep / 2; a < a2; a += aStep) {
            cix = mCenter.x + cos(a) * r2;
            ciy = mCenter.y + sin(a) * r2;
            polyline->appendVertex(Vec2d(cix, ciy));
        }
    }
    else {
        // Arc Clockwise:
        if (a1 < a2 + 1.0e-10) {
            a2 -= 2 * M_PI;
        }
        for (a = a1 - aStep / 2; a > a2; a -= aStep) {
            cix = mCenter.x + cos(a) * r2;
            ciy = mCenter.y + sin(a) * r2;
            polyline->appendVertex(Vec2d(cix, ciy));
        }
    }

    if (polyline->countVertices() == 1) {
        // only got start point, add point in the middle:
        a = getAngleAtPercent(0.5);
        cix = mCenter.x + cos(a) * r2;
        ciy = mCenter.y + sin(a) * r2;
        polyline->appendVertex(Vec2d(cix, ciy));
    }

    polyline->appendVertex(getEndPoint());

    return polyline;
}

std::vector<std::unique_ptr<Line>> Arc::getTangents(const Vec2d &point) const
{
    std::unique_ptr<Circle> circle =
        ShapeFactory::instance()->createCircle(mCenter.x, mCenter.y, mRadius);
    return circle->getTangents(point);
}

std::vector<std::unique_ptr<Arc>> Arc::splitAtQuadrantLines() const
{
    std::vector<double> angles;
    angles.push_back(0.0);
    angles.push_back(M_PI / 2);
    angles.push_back(M_PI);
    angles.push_back(M_PI / 2 * 3);

    std::vector<Vec2d> points;
    for (size_t i = 0; i < angles.size(); i++) {
        if (isAngleWithinArc(angles[i])) {
            points.push_back(mCenter + Vec2d::createPolar(mRadius, angles[i]));
        }
    }

    std::vector<std::unique_ptr<Shape>> segments = splitAt(points);
    std::vector<std::unique_ptr<Arc>> ret;
    for (auto &seg : segments) {
        Arc *arc = dynamic_cast<Arc *>(seg.get());
        ret.emplace_back(arc);
    }
    return ret;
}

void Arc::moveStartPoint(const Vec2d &pos, bool keepRadius)
{
    if (!keepRadius) {
        auto a = ShapeFactory::instance()->createArcFrom3Point(
            pos, getMiddlePoint(), getEndPoint());
        if (a->isReversed() != isReversed()) {
            a->reverse();
        }
        mCenter = a->getCenter();
        mRadius = a->getRadius();
        mStartAngle = a->getStartAngle();
        mEndAngle = a->getEndAngle();
        mReversed = a->isReversed();
    }
    else {
        double bulge = getBulge();

        // full circle: trim instead of move:
        if (bulge < 1.0e-6 || bulge > 1.0e6) {
            mStartAngle = mCenter.getAngleTo(pos);
        }
        else {
            auto a = ShapeFactory::instance()->createArcFrom2PBulge(
                pos, getEndPoint(), bulge);
            mCenter = a->getCenter();
            mRadius = a->getRadius();
            mStartAngle = a->getStartAngle();
            mEndAngle = a->getEndAngle();
            mReversed = a->isReversed();
        }
    }
}

void Arc::moveEndPoint(const Vec2d &pos, bool keepRadius)
{
    if (!keepRadius) {
        auto a = ShapeFactory::instance()->createArcFrom3Point(
            pos, getMiddlePoint(), getStartPoint());
        if (a->isReversed() != isReversed()) {
            a->reverse();
        }
        mCenter = a->getCenter();
        mRadius = a->getRadius();
        mStartAngle = a->getStartAngle();
        mEndAngle = a->getEndAngle();
        mReversed = a->isReversed();
    }
    else {
        double bulge = getBulge();

        // full circle: trim instead of move:
        if (bulge < 1.0e-6 || bulge > 1.0e6) {
            mEndAngle = mCenter.getAngleTo(pos);
        }
        else {
            auto a = ShapeFactory::instance()->createArcFrom2PBulge(
                getStartPoint(), pos, bulge);
            mCenter = a->getCenter();
            mRadius = a->getRadius();
            mStartAngle = a->getStartAngle();
            mEndAngle = a->getEndAngle();
            mReversed = a->isReversed();
        }
    }
}

void Arc::moveMiddlePoint(const Vec2d &pos)
{
    auto a = ShapeFactory::instance()->createArcFrom3Point(getStartPoint(), pos,
                                                           getEndPoint());
    mCenter = a->getCenter();
    mRadius = a->getRadius();
    mStartAngle = a->getStartAngle();
    mEndAngle = a->getEndAngle();
    mReversed = a->isReversed();
}

} // namespace shape
} // namespace cada