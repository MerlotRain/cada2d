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

#include <QMutex>

#include "Arc.h"
#include "RDebug.h"
#include "Line.h"
#include "BSpline.h"
#include "RPainterPath.h"
#include "Polyline.h"

// BSpline::UpdateFromFitPointsFunction BSpline::updateFromFitPointsFunction =
// NULL; BSpline::SplitFunction BSpline::splitFunction = NULL;
RSplineProxy *BSpline::splineProxy = NULL;

/**
 * Creates a spline object without controlPoints.
 */
BSpline::BSpline()
    : degree(3), periodic(false), dirty(true), updateInProgress(false),
      length(std::numeric_limits<double>::quiet_NaN())
{
}

BSpline::BSpline(const BSpline &other)
{
    *this = other;
}

/**
 * Creates a spline object with the given control points and degree.
 */
BSpline::BSpline(const std::vector<Vec3d> &controlPoints, int degree)
    : controlPoints(controlPoints), degree(degree), periodic(false),
      dirty(true), updateInProgress(false),
      length(std::numeric_limits<double>::quiet_NaN())
{

    // updateInternal();
}

// BSpline::~BSpline() {
// invalidate();
//}

BSpline &BSpline::operator=(const BSpline &other)
{
    controlPoints = other.controlPoints;
    knotVector = other.knotVector;
    weights = other.weights;
    fitPoints = other.fitPoints;
    degree = other.degree;
    tangentStart = other.tangentStart;
    tangentEnd = other.tangentEnd;
    periodic = other.periodic;
    dirty = other.dirty;
    updateInProgress = other.updateInProgress;
#ifndef R_NO_OPENNURBS
    if (other.curve.IsValid()) {
        curve = other.curve;
    }
#endif
    boundingBox = other.boundingBox;
    exploded = other.exploded;
    length = other.length;

    return *this;
}

void BSpline::copySpline(const BSpline &other)
{
    this->degree = other.degree;
    this->periodic = other.periodic;
    this->controlPoints = other.controlPoints;
    this->fitPoints = other.fitPoints;
    this->knotVector = other.knotVector;
    this->weights = other.weights;
    this->tangentStart = other.tangentStart;
    this->tangentEnd = other.tangentEnd;
    this->boundingBox = other.boundingBox;
    this->length = other.length;
    bool d = other.dirty;

    // do NOT copy curve member (copying of ON_NurbsCurve is broken, segfaults).
    dirty = true;
    updateInternal();

    // if original was not dirty, copy cached exploded representation:
    if (!d) {
        this->exploded = other.exploded;
    }
}

/**
 *  \return List of splines which approximate the given arc.
 */
std::vector<BSpline> BSpline::createSplinesFromArc(const Arc &arc)
{
    Arc a = arc;
    bool reversed = false;
    if (a.isReversed()) {
        reversed = true;
        a.reverse();
    }

    double startAngle = Math::getNormalizedAngle(a.getStartAngle());
    double endAngle = Math::getNormalizedAngle(a.getEndAngle());
    if (a.isFullCircle()) {
        startAngle = 0.0;
        endAngle = 2 * M_PI;
    }

    // normalize startAngle, endAngle to [-2PI, 2PI]
    double twoPI = M_PI * 2;
    // double startAngle = Math::getNormalizedAngle(a.getStartAngle());
    // double endAngle = Math::getNormalizedAngle(a.getEndAngle());
    if (startAngle > endAngle) {
        startAngle -= 2 * M_PI;
    }
    double radius = a.getRadius();
    double EPSILON = 0.00001;

    // Compute the sequence of arc curves, up to PI/2 at a time.  Total arc
    // angle is less than 2PI.

    std::vector<BSpline> curves;

    double piOverTwo = M_PI_2;
    double segmentationAngle = piOverTwo / 8;
    // double segmentationAngle = M_PI/8;
    double sgn = (startAngle < endAngle) ? +1 : -1;

    double a1 = startAngle;
    for (double totalAngle = qMin(twoPI, qAbs(endAngle - startAngle));
         totalAngle > EPSILON;) {
        double a2 = a1 + sgn * qMin(totalAngle, segmentationAngle);
        BSpline sp = BSpline::createBezierFromSmallArc(radius, a1, a2);
        sp.move(a.getCenter());
        if (reversed) {
            sp.reverse();
            curves.prepend(sp);
        }
        else {
            curves.push_back(sp);
        }
        totalAngle -= qAbs(a2 - a1);
        a1 = a2;
    }

    return curves;
}

/**
 *  Cubic bezier approximation of a circular arc centered at the origin,
 *  from (radians) a1 to a2, where a2-a1 < pi/2.  The arc's radius is r.
 *
 *  Returns an spline approximation.
 *
 *  This algorithm is based on the approach described in:
 *  A. Riškus, "Approximation of a Cubic Bezier Curve by Circular Arcs and Vice
 * Versa," Information Technology and Control, 35(4), 2006 pp. 371-378.
 */
BSpline BSpline::createBezierFromSmallArc(double r, double a1, double a2)
{
    // Compute all four points for an arc that subtends the same total angle
    // but is centered on the X-axis

    double a = (a2 - a1) / 2.0; //

    double x4 = r * cos(a);
    double y4 = r * sin(a);
    double x1 = x4;
    double y1 = -y4;

    // double k = 0.552284749831;
    // double k = 4.0/3.0*(sqrt(2.0)-1.0);
    // double f = k * tan(a);

    double q1 = x1 * x1 + y1 * y1;
    double q2 = q1 + x1 * x4 + y1 * y4;
    double k2 = 4 / 3 * (sqrt(2 * q1 * q2) - q2) / (x1 * y4 - y1 * x4);

    double x2 = x1 - k2 * y1;
    double y2 = y1 + k2 * x1;
    // double x2 = x1 + f * y4;
    // double y2 = y1 + f * x4;
    double x3 = x2;
    double y3 = -y2;

    // Find the arc points actual locations by computing x1,y1 and x4,y4
    // and rotating the control points by a + a1
    double ar = a + a1;
    double cos_ar = cos(ar);
    double sin_ar = sin(ar);

    std::vector<Vec3d> ctrlPts;
    ctrlPts << Vec3d(r * cos(a1), r * sin(a1))
            << Vec3d(x2 * cos_ar - y2 * sin_ar, x2 * sin_ar + y2 * cos_ar)
            << Vec3d(x3 * cos_ar - y3 * sin_ar, x3 * sin_ar + y3 * cos_ar)
            << Vec3d(r * cos(a2), r * sin(a2));

    //    qDebug() << "ctrlPts: " << ctrlPts[0];
    //    qDebug() << "ctrlPts: " << ctrlPts[1];
    //    qDebug() << "ctrlPts: " << ctrlPts[2];
    //    qDebug() << "ctrlPts: " << ctrlPts[3];

    // this should be cubic but appears to be far off if cubic
    return BSpline(ctrlPts, 2);
}

void BSpline::appendControlPoint(const Vec3d &point)
{
    controlPoints.push_back(point);
    update();
}

/**
 * Appends the given control points.
 */
void BSpline::appendControlPoints(const std::vector<Vec3d> &points)
{
    controlPoints.push_back(points);
    update();
}

/**
 * Removes the last control point.
 *
 * \param upd Update internal spline representation.
 */
void BSpline::removeLastControlPoint()
{
    controlPoints.removeLast();
    update();
}

/**
 * Sets the control points of this spline.
 */
void BSpline::setControlPoints(const std::vector<Vec3d> &controlPoints)
{
    this->controlPoints = controlPoints;
    update();
}

/**
 * \return Control points.
 */
std::vector<Vec3d> BSpline::getControlPoints() const
{
    return controlPoints;
}

/**
 * \return Control points of internal spline representation (may be closed).
 */
std::vector<Vec3d> BSpline::getControlPointsWrapped() const
{
    std::vector<Vec3d> ret;

    updateInternal();

#ifndef R_NO_OPENNURBS
    ON_3dPoint onp;
    for (int i = 0; i < curve.CVCount(); ++i) {
        curve.GetCV(i, onp);
        ret.push_back(Vec3d(onp.x, onp.y));
    }
#endif

    return ret;
}

/**
 * \return Number of control points.
 */
int BSpline::countControlPoints() const
{
    return controlPoints.size();
}

Vec3d BSpline::getControlPointAt(int i) const
{
    if (i >= 0 && i < controlPoints.size()) {
        return controlPoints.at(i);
    }
    return Vec3d::invalid;
}

/**
 * Appends a fit point.
 */
void BSpline::appendFitPoint(const Vec3d &point)
{
    fitPoints.push_back(point);
    update();
}

/**
 * Prepends a fit point.
 */
void BSpline::prependFitPoint(const Vec3d &point)
{
    fitPoints.prepend(point);
    update();
}

/**
 * Inserts a fit point at the point on the spline closest to the given position.
 */
void BSpline::insertFitPointAt(const Vec3d &point)
{
    Vec3d p = getClosestPointOnShape(point);

    // find out T at the point closest to point:
    double t = getTAtPoint(p);

    insertFitPointAt(t, p);
}

void BSpline::insertFitPointAt(double t, const Vec3d &p)
{
    // find out index of fit point before t:
    int index = -1;
    for (int i = 0; i < fitPoints.size(); i++) {
        double tc = getTAtPoint(fitPoints[i]);
        if (i == 0 &&
            (isClosed() || getStartPoint().equalsFuzzy(getEndPoint()))) {
            // closed spline: two t's for first fit point:
            tc = 0.0;
        }
        // qWarning() << "tc: " << tc;
        if (tc < t) {
            index = i + 1;
        }
        else {
            break;
        }
    }

    // point not on spline:
    if (index < 0 || index >= fitPoints.size()) {
        if (isClosed()) {
            index = 0;
        }
        else {
            qWarning() << "no point on spline found. t: " << t
                       << ", index: " << index;
            return;
        }
    }

    fitPoints.insert(index, p);
    update();
}

void BSpline::removeFitPointAt(const Vec3d &point)
{
    double minDist = RMAXDOUBLE;
    int index = -1;
    for (int i = 0; i < fitPoints.size(); i++) {
        double dist = point.getDistanceTo(fitPoints[i]);
        if (dist < minDist) {
            minDist = dist;
            index = i;
        }
    }

    if (index < 0 || index >= fitPoints.size()) {
        return;
    }

    fitPoints.removeAt(index);
    update();
}

/**
 * Removes the last fit point.
 */
void BSpline::removeLastFitPoint()
{
    fitPoints.removeLast();
    update();
}

/**
 * Removes the first fit point.
 */
void BSpline::removeFirstFitPoint()
{
    fitPoints.removeFirst();
    update();
}

/**
 * Sets the fit points.
 */
void BSpline::setFitPoints(const std::vector<Vec3d> &fitPoints)
{
    this->fitPoints = fitPoints;
    update();
}

/**
 * \return Fit points.
 */
std::vector<Vec3d> BSpline::getFitPoints() const
{
    return fitPoints;
}

/**
 * \return Number of fit points.
 */
int BSpline::countFitPoints() const
{
    return fitPoints.size();
}

/**
 * \return True if this spline has fit points and is therefore defined
 *      by its fit points, false otherwise.
 */
bool BSpline::hasFitPoints() const
{
    return !fitPoints.isEmpty();
}

Vec3d BSpline::getFitPointAt(int i) const
{
    if (i >= 0 && i < fitPoints.size()) {
        return fitPoints.at(i);
    }
    return Vec3d::invalid;
}

/**
 * \return Knot vector, internally calculated and updated.
 */
std::vector<double> BSpline::getKnotVector() const
{
    return knotVector;
}

std::vector<double> BSpline::getActualKnotVector() const
{
#ifndef R_NO_OPENNURBS
    updateInternal();
    std::vector<double> ret;
    for (int i = 0; i < curve.KnotCount(); ++i) {
        ret.push_back(curve.Knot(i));
    }
    return ret;
#else
    return std::vector<double>();
#endif
}

/**
 * Sets the knot vector manually. Mainly for importing ready data.
 */
void BSpline::setKnotVector(const std::vector<double> &knots)
{
    knotVector = knots;
    update();
}

void BSpline::appendKnot(double k)
{
    knotVector.push_back(k);
    update();
}

/**
 * \return Knot weights, internally calculated and updated.
 */
std::vector<double> BSpline::getWeights() const
{
    return weights;

    /*
    std::vector<double> ret;

    for (int i=0; i<curve.CVCount(); ++i) {
        ret.push_back(curve.Weight(i));
    }
    return ret;
    */
}

void BSpline::setWeights(std::vector<double> &w)
{
    weights = w;
}

/**
 * Sets the degree of this spline (2 or 3 for control point defined spline,
 * 3 for fit point defined spline).
 */
void BSpline::setDegree(int d)
{
    degree = d;
    update();
}

/**
 * \return Degree of this spline.
 */
int BSpline::getDegree() const
{
    return degree;
}

/**
 * \return Order of this spline (=degree+1).
 */
int BSpline::getOrder() const
{
    return degree + 1;
}

void BSpline::setPeriodic(bool on)
{
    periodic = on;

    // TODO: tangent support:
    //    tangentStart.valid = false;
    //    tangentEnd.valid = false;
    //    if (on) {
    //        if (!fitPoints.isEmpty()) {
    //            if (fitPoints.first().equalsFuzzy(fitPoints.last())) {
    //                fitPoints.removeLast();
    //            }
    //        }
    //    }

    update();
}

/**
 * \return True if this spline is closed, i.e. start point and end point
 *      are very close to each other.
 */
bool BSpline::isClosed() const
{
    return periodic;

    // return curve.IsClosed();

    /*
    if (hasFitPoints()) {
        return fitPoints.first().getDistanceTo(fitPoints.last()) <
    NS::PointTolerance;
    }
    else {
        //return controlPoints.first().getDistanceTo(controlPoints.last()) <
    NS::PointTolerance; if (controlPoints.count()<degree) { return false;
        }

        bool ret = true;
        for (int i=0; i<degree; i++) {
            if
    (controlPoints.at(i).getDistanceTo(controlPoints.at(controlPoints.count()-degree+i))
    > NS::PointTolerance) { ret = false;
            }
        }
        return ret;

        //return
    controlPoints.at(0).getDistanceTo(controlPoints.at(controlPoints.count()-degree))
    < NS::PointTolerance &&
        //
    controlPoints.at(1).getDistanceTo(controlPoints.at(controlPoints.count()-1))
    < NS::PointTolerance
    }
    */
}

bool BSpline::isGeometricallyClosed(double tolerance) const
{
    return isClosed() ||
           getStartPoint().getDistanceTo(getEndPoint()) < tolerance;
}

/**
 * \return True if this spline is periodic, i.e. closed and 'smooth'
 *      where start and end connect. The tangents at the start point and
 *      end point are nearly identical.
 */
bool BSpline::isPeriodic() const
{
    return periodic;
    /*
    int c = curve.CVCount();
    if (c<=degree) {
        return false;
    }

    if (hasFitPoints()) {
        return periodic;
    }
    else {
        // check if first N control points match with N last control points:
 #ifndef R_NO_OPENNURBS
        for (int i=0; i<degree; i++) {
            ON_3dPoint onp1;
            curve.GetCV(i, onp1);
            Vec3d p1(onp1.x, onp1.y);

            ON_3dPoint onp2;
            curve.GetCV(curve.CVCount()-degree+i, onp2);
            Vec3d p2(onp2.x, onp2.y);
            if (p1.getDistanceTo(p2) > NS::PointTolerance) {
                return false;
            }
        }
 #endif
    }

    return true;
    */

    //    return periodic;

    // return curve.IsPeriodic();
    /*
    if (!isClosed()) {
        return false;
    }

    double ad = Math::getAngleDifference180(getDirection1(),
    getDirection2()+M_PI); if (fabs(ad) < NS::AngleTolerance) { return true;
    }

    return false;
    */
}

/**
 * \return Tangent angle of spline at start point.
 */
double BSpline::getDirection1() const
{
    if (!isValid()) {
        return 0.0;
    }
    updateInternal();

#ifndef R_NO_OPENNURBS
    ON_3dVector ontan = curve.TangentAt(getTMin());
    Vec3d rtan(ontan.x, ontan.y);
    return rtan.getAngle();
#else
    return 0.0;
#endif
}

/**
 * \return Tangent angle of spline at end point.
 */
double BSpline::getDirection2() const
{
    if (!isValid()) {
        return 0.0;
    }
    updateInternal();

#ifndef R_NO_OPENNURBS
    ON_3dVector ontan = curve.TangentAt(getTMax());
    Vec3d rtan(ontan.x, ontan.y);
    return Math::getNormalizedAngle(rtan.getAngle() + M_PI);
#else
    return 0.0;
#endif
}

NS::Side BSpline::getSideOfPoint(const Vec3d &point) const
{
    Polyline pl = toPolyline(16);
    return pl.getSideOfPoint(point);
}

Vec3d BSpline::getStartPoint() const
{
    //    if (!isClosed()) {
    //        if (hasFitPoints()) {
    //            return fitPoints.first();
    //        }
    //        else if (!controlPoints.isEmpty()) {
    //            return controlPoints.first();
    //        }
    //    }
    return getPointAt(getTMin());
}

void BSpline::setStartPoint(const Vec3d &v)
{
    // TODO: handle fit points
    controlPoints[0] = v;
    update();
}

Vec3d BSpline::getEndPoint() const
{
    //    if (!isClosed()) {
    //        if (hasFitPoints()) {
    //            return fitPoints.last();
    //        }
    //        else if (!controlPoints.isEmpty()) {
    //            return controlPoints.last();
    //        }
    //    }
    return getPointAt(getTMax());
}

void BSpline::setEndPoint(const Vec3d &v)
{
    // TODO: handle fit points
    controlPoints[controlPoints.size() - 1] = v;
    update();
}

/**
 * Sets the start and end tangents.
 */
void BSpline::setTangents(const Vec3d &start, const Vec3d &end)
{
    tangentStart = start;
    tangentEnd = end;
    update();
}

/**
 * Sets the start tangent.
 */
void BSpline::setTangentAtStart(const Vec3d &t)
{
    tangentStart = t;
    update();
}

/**
 * \return The start tangent.
 */
Vec3d BSpline::getTangentAtStart() const
{
    return tangentStart;
}

/**
 * Sets the end tangent.
 */
void BSpline::setTangentAtEnd(const Vec3d &t)
{
    tangentEnd = t;
    update();
}

/**
 * \return The end tangent.
 */
Vec3d BSpline::getTangentAtEnd() const
{
    return tangentEnd;
}

/**
 * Clears the valud of the start tangent. The start tangent is calculated
 *      and updated internally if not set manually.
 */
void BSpline::unsetTangentAtStart()
{
    setTangentAtStart(Vec3d::invalid);
}

/**
 * Clears the valud of the end tangent. The end tangent is calculated
 *      and updated internally if not set manually.
 */
void BSpline::unsetTangentAtEnd()
{
    setTangentAtEnd(Vec3d::invalid);
}

/**
 * Clears the valud of the start and end tangents. The tangents are calculated
 *      and updated internally if not set manually.
 */
void BSpline::unsetTangents()
{
    setTangents(Vec3d::invalid, Vec3d::invalid);
}

/**
 * Updates the tangents at the start and end to make the spline periodic.
 */
void BSpline::updateTangentsPeriodic()
{
    if (!isValid() || !isClosed()) {
        qWarning() << "BSpline::updateTangentsPeriodic(): "
                      "spline not valid or not closed";
    }

    // TODO: tangent support:
    //    Vec3d lStartTangent = tangentStart;
    //    Vec3d lEndTangent = tangentEnd;

    unsetTangents();

    double tangent1 = getDirection1();
    double tangent2 = Math::getNormalizedAngle(getDirection2() + M_PI);
    Vec3d v1 = Vec3d::createPolar(1.0, tangent1);
    Vec3d v2 = Vec3d::createPolar(1.0, tangent2);
    Vec3d t = (v1 + v2).getNormalized();

    // TODO: tangent support:
    //    Vec3d t1 = t;
    //    Vec3d t2 = t;
    //    t1.valid = lStartTangent.valid;
    //    t2.valid = lEndTangent.valid;

    //    if (!lStartTangent.isValid()) {
    //        setTangentAtStart(t1);
    //    }
    //    else {
    //        setTangentAtStart(lStartTangent);
    //    }
    //    if (!lEndTangent.isValid()) {
    //        setTangentAtEnd(t2);
    //    }
    //    else {
    //        setTangentAtEnd(lEndTangent);
    //    }
    setTangents(t, t);
}

Polyline BSpline::approximateWithArcs(double tolerance,
                                      double radiusLimit) const
{
    if (hasProxy()) {
        return getSplineProxy()->approximateWithArcs(*this, tolerance,
                                                     radiusLimit);
    }
    return Polyline();
}

Polyline BSpline::toPolyline(int segments) const
{
    Polyline ret;

    std::vector<std::shared_ptr<Shape>> lineSegments =
        getExplodedBezier(segments);
    for (int k = 0; k < lineSegments.size(); k++) {
        std::shared_ptr<Shape> shape = lineSegments[k];
        if (shape.isNull() || !shape->isDirected()) {
            continue;
        }
        if (k == 0) {
            ret.appendVertex(shape->getStartPoint());
        }
        ret.appendVertex(shape->getEndPoint());
    }
    if (isClosed()) {
        ret.setClosed(true);
    }

    return ret;
}

/**
 * \return List of RLines describing this spline.
 */
std::vector<std::shared_ptr<Shape>> BSpline::getExploded(int segments) const
{
    if (!exploded.isEmpty() && segments == -1) {
        return exploded;
    }

    // qDebug() << "BSpline::getExploded: segments: " << segments;
    // RDebug::printBacktrace("getExploded:    ");

    // ##boundingBox = BBox();

    updateInternal();

    exploded.clear();

    if (!isValid()) {
        // qWarning() << "BSpline::getExploded: invalid spline";
        return exploded;
    }

    if (segments == -1) {
        segments = 8;
    }

    double tMin = getTMin();
    double tMax = getTMax();

    double step = getTDelta() / (controlPoints.size() * segments);

    Vec3d p1;
    Vec3d prev = Vec3d::invalid;
    for (double t = tMin; t < tMax + (step / 2.0); t += step) {
        double tc = qMin(t, tMax);
        p1 = getPointAt(tc);

        if (Math::isNaN(p1.x) || Math::isNaN(p1.y)) {
            continue;
        }

        if (prev.isValid()) {
            appendToExploded(Line(prev, p1));
            //            Line* line = new Line(prev, p1);
            //            exploded.push_back(std::shared_ptr<Shape>(line));
        }
        prev = p1;

        // ##boundingBox.growToInclude(p1);
    }

    p1 = getEndPoint();
    if (!Math::isNaN(p1.x) && !Math::isNaN(p1.y)) {
        if (prev.isValid()) {
            appendToExploded(Line(prev, p1));
            //            Line* line = new Line(prev, p1);
            // prevent zero length line at the end:
            //            if (line->getLength()>1.0e-4) {
            //                exploded.push_back(std::shared_ptr<Shape>(line));
            //            }
        }
    }

    return exploded;
}

/**
 * \return exploded spline, treated as one spline segment, typically only
 * used for bezier spline segments (degree+1 control points).
 */
std::vector<std::shared_ptr<Shape>>
BSpline::getExplodedBezier(int segments) const
{
    std::vector<std::shared_ptr<Shape>> ret;
    std::vector<BSpline> bezierSegments = getBezierSegments();
    for (int i = 0; i < bezierSegments.size(); i++) {
        ret.push_back(bezierSegments[i].getExploded(segments));
    }
    return ret;
}

void BSpline::appendToExploded(const Line &line) const
{
    if (line.getLength() < 1.0e-6) {
        return;
    }

    static QMutex m;
    QMutexLocker ml(&m);

    if (!exploded.isEmpty()) {
        // compare angle of this sement with last segment and
        // modify last segment if angle is the same (straight line):
        std::shared_ptr<Line> prev = exploded.last().dynamicCast<Line>();
        if (!prev.isNull()) {
            if (Math::fuzzyCompare(
                    prev->getAngle(),
                    prev->getStartPoint().getAngleTo(line.getEndPoint()))) {
                prev->setEndPoint(line.getEndPoint());
                return;
            }
        }
    }

    exploded.push_back(std::shared_ptr<Shape>(new Line(line)));
}

std::vector<std::shared_ptr<Shape>>
BSpline::getExplodedWithSegmentLength(double segmentLength) const
{
    std::vector<std::shared_ptr<Shape>> ret;
    std::vector<BSpline> bezierSegments = getBezierSegments();
    for (int i = 0; i < bezierSegments.size(); i++) {
        double len = bezierSegments[i].getLength();
        int seg = static_cast<int>(ceil(len / segmentLength));
        ret.push_back(bezierSegments[i].getExploded(seg));
    }
    return ret;
}

BBox BSpline::getBoundingBox() const
{
    if (!isValid()) {
        return BBox();
    }

    if (!boundingBox.isValid()) {
        updateBoundingBox();
    }

    return boundingBox;

    // ON_3dPoint onmin;
    // ON_3dPoint onmax;
    // curve.GetBoundingBox(onmin, onmax);

    //    double min[3];
    //    double max[3];
    //    curve.GetBBox(min, max);

    // ON_BoundingBox bb;
    // curve.GetTightBoundingBox(bb);

    // return BBox(Vec3d(bb.Min().x, bb.Min().y), Vec3d(bb.Max().x,
    // bb.Max().y)); return BBox(Vec3d(min[0], min[1]), Vec3d(max[0],
    // max[1]));
}

double BSpline::getLength() const
{
    if (!isValid()) {
        return 0.0;
    }
    if (!dirty && !Math::isNaN(length)) {
        return length;
    }

    if (hasProxy()) {
        length = splineProxy->getLength(*this);
    }
    else {
        length = 0.0;
        std::vector<std::shared_ptr<Shape>> shapes = getExploded();
        for (int i = 0; i < shapes.size(); i++) {
            std::shared_ptr<Shape> shape = shapes[i];
            length += shape->getLength();
        }
    }

    return length;

    // seems to only work in the context of another product which uses
    // OpenNURBS: curve.GetLength(&length);
}

/**
 * \return Point on spline at given position t (0..1).
 */
Vec3d BSpline::getPointAt(double t) const
{
    updateInternal();
#ifndef R_NO_OPENNURBS
    ON_3dPoint p = curve.PointAt(t);
    if (p.IsUnsetPoint()) {
        return Vec3d::invalid;
    }
    return Vec3d(p.x, p.y);
#else
    return Vec3d::invalid;
#endif
}

Vec3d BSpline::getPointAtDistance(double distance) const
{
    double t = getTAtDistance(distance);
    return getPointAt(t);
}

double BSpline::getAngleAt(double distance, NS::From from) const
{
    std::vector<Vec3d> points = getPointsWithDistanceToEnd(distance, from);
    if (points.size() != 1) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    double t = getTAtPoint(points[0]);
    ON_3dVector v = curve.DerivativeAt(t);
    return Vec3d(v.x, v.y).getAngle();
}

std::vector<Vec3d> BSpline::getEndPoints() const
{
    std::vector<Vec3d> ret;

    ret.push_back(getStartPoint());
    ret.push_back(getEndPoint());

    return ret;
}

Vec3d BSpline::getMiddlePoint() const
{
    return getPointAt(getTMin() + (getTDelta() / 2.0));
}

std::vector<Vec3d> BSpline::getMiddlePoints() const
{
    std::vector<Vec3d> ret;

    ret.push_back(getMiddlePoint());

    return ret;
}

std::vector<Vec3d> BSpline::getCenterPoints() const
{
    return std::vector<Vec3d>();
}

std::vector<Vec3d> BSpline::getPointsWithDistanceToEnd(double distance,
                                                       int from) const
{
    std::vector<Vec3d> ret;

    if (hasProxy()) {
        double t;
        if (from & NS::FromStart) {
            t = splineProxy->getTAtDistance(*this, distance);
            ret << getPointAt(t);
        }
        if (from & NS::FromEnd) {
            t = splineProxy->getTAtDistance(*this, getLength() - distance);
            ret << getPointAt(t);
        }
    }
    else {
        // no spline proxy (not precise, but better than nothing in some cases):
        double length = getLength();
        if (length <= NS::PointTolerance) {
            return ret;
        }

        if (from & NS::FromStart) {
            Vec3d p = getPointAt(getTMin() + (distance / length * getTDelta()));
            ret.push_back(p);
        }

        if (from & NS::FromEnd) {
            Vec3d p = getPointAt(getTMin() +
                                 ((length - distance) / length * getTDelta()));
            ret.push_back(p);
        }
    }

    return ret;
}

std::vector<Vec3d> BSpline::getPointCloud(double segmentLength) const
{
    Polyline pl = approximateWithArcs(0.01);
    return pl.getPointCloud(segmentLength);
}

Vec3d BSpline::getVectorTo(const Vec3d &point, bool limited,
                           double strictRange) const
{
    if (hasProxy()) {
        return splineProxy->getVectorTo(*this, point, limited, strictRange);
    }
    else {
        Vec3d ret = Vec3d::invalid;

        std::vector<std::shared_ptr<Shape>> sub = getExploded();
        std::vector<std::shared_ptr<Shape>>::iterator it;
        for (it = sub.begin(); it != sub.end(); ++it) {
            Vec3d v = (*it)->getVectorTo(point, limited, strictRange);
            if (v.isValid() &&
                (!ret.isValid() || v.getMagnitude() < ret.getMagnitude())) {
                ret = v;
            }
        }

        return ret;
    }
}

bool BSpline::isOnShape(const Vec3d &point, bool limited,
                        double tolerance) const
{
    if (hasProxy()) {
        double t = getTAtPoint(point);
        Vec3d p = getPointAt(t);
        return point.getDistanceTo(p) < tolerance;
    }
    else {
        return Shape::isOnShape(point, limited, tolerance);
    }
}

bool BSpline::move(const Vec3d &offset)
{
    for (int i = 0; i < controlPoints.size(); i++) {
        controlPoints[i].move(offset);
    }
    for (int i = 0; i < fitPoints.size(); i++) {
        fitPoints[i].move(offset);
    }
    update();
    return true;
}

bool BSpline::rotate(double rotation, const Vec3d &center)
{
    if (fabs(rotation) < NS::AngleTolerance) {
        return false;
    }
    for (int i = 0; i < controlPoints.size(); i++) {
        controlPoints[i].rotate(rotation, center);
    }
    for (int i = 0; i < fitPoints.size(); i++) {
        fitPoints[i].rotate(rotation, center);
    }
    tangentStart.rotate(rotation);
    tangentEnd.rotate(rotation);
    update();
    return true;
}

bool BSpline::scale(const Vec3d &scaleFactors, const Vec3d &center)
{
    for (int i = 0; i < controlPoints.size(); i++) {
        controlPoints[i].scale(scaleFactors, center);
    }
    for (int i = 0; i < fitPoints.size(); i++) {
        fitPoints[i].scale(scaleFactors, center);
    }
    update();
    return true;
}

bool BSpline::mirror(const Line &axis)
{
    Vec3d sp = getStartPoint();
    Vec3d ep = getEndPoint();

    for (int i = 0; i < controlPoints.size(); i++) {
        controlPoints[i].mirror(axis);
    }
    for (int i = 0; i < fitPoints.size(); i++) {
        fitPoints[i].mirror(axis);
    }

    Vec3d absTan = sp + tangentStart;
    absTan.mirror(axis);
    sp.mirror(axis);
    tangentStart = absTan - sp;

    absTan = ep + tangentEnd;
    absTan.mirror(axis);
    ep.mirror(axis);
    tangentEnd = absTan - ep;

    update();

    return true;
}

bool BSpline::flipHorizontal()
{
    for (int i = 0; i < controlPoints.size(); i++) {
        controlPoints[i].flipHorizontal();
    }
    for (int i = 0; i < fitPoints.size(); i++) {
        fitPoints[i].flipHorizontal();
    }
    tangentStart.flipHorizontal();
    tangentEnd.flipHorizontal();
    update();
    return true;
}

bool BSpline::flipVertical()
{
    for (int i = 0; i < controlPoints.size(); i++) {
        controlPoints[i].flipVertical();
    }
    for (int i = 0; i < fitPoints.size(); i++) {
        fitPoints[i].flipVertical();
    }
    tangentStart.flipVertical();
    tangentEnd.flipVertical();
    update();
    return true;
}

bool BSpline::reverse()
{
    int k;
    if (!isClosed()) {
        for (k = 0; k < controlPoints.size() / 2; k++) {
#if QT_VERSION >= 0x060000
            controlPoints.swapItemsAt(k, controlPoints.size() - (1 + k));
#else
            controlPoints.swap(k, controlPoints.size() - (1 + k));
#endif
        }
        for (k = 0; k < fitPoints.size() / 2; k++) {
#if QT_VERSION >= 0x060000
            fitPoints.swapItemsAt(k, fitPoints.size() - (1 + k));
#else
            fitPoints.swap(k, fitPoints.size() - (1 + k));
#endif
        }
        double t;
        int i, j;
        for (i = 0, j = knotVector.size() - 1; i <= j; i++, j--) {
            t = knotVector[i];
            knotVector[i] = -knotVector[j];
            knotVector[j] = -t;
        }
        Vec3d ts = tangentStart;
        tangentStart = tangentEnd.getNegated();
        tangentEnd = ts.getNegated();
    }
    else {
        if (hasFitPoints()) {
            for (k = 0; k < (int)floor(fitPoints.size() / 2.0); k++) {
#if QT_VERSION >= 0x060000
                fitPoints.swapItemsAt(k, fitPoints.size() - (1 + k));
#else
                fitPoints.swap(k, fitPoints.size() - (1 + k));
#endif
            }
            // keep start node the same:
            fitPoints.prepend(fitPoints.takeLast());
        }
        else {
            for (k = 0; k < controlPoints.size() / 2; k++) {
#if QT_VERSION >= 0x060000
                controlPoints.swapItemsAt(k, controlPoints.size() - (1 + k));
#else
                controlPoints.swap(k, controlPoints.size() - (1 + k));
#endif
            }
        }
        updateTangentsPeriodic();
    }
    update();
    return true;
}

bool BSpline::stretch(const Polyline &area, const Vec3d &offset)
{
    if (!fitPoints.isEmpty()) {
        for (int i = 0; i < fitPoints.size(); i++) {
            fitPoints[i].stretch(area, offset);
        }
        update();
        return true;
    }
    return false;
}

bool BSpline::isValid() const
{
    if (!dirty) {
#ifndef R_NO_OPENNURBS
        //        enable for opennurbs debugging:
        //        ON_wString s;
        //        ON_TextLog log(s);
        //        if (!curve.IsValid(&log)) {
        //            qDebug() << "BSpline::isValid: spline curve is not
        //            valid:"; QString qs; for (int i=0; i<s.size(); i++) {
        //                qs.push_back(QChar(s.GetAt(i)));
        //            }
        //            qDebug() << qs;
        //        }
        return curve.IsValid();
#endif
    }

    if (degree < 1) {
        qDebug() << "BSpline::isValid: spline not valid: degree: " << degree;
        return false;
    }
    if (hasFitPoints()) {
        // spline with two fit points is line:
        if (fitPoints.count() < 2) {
            // qDebug() << "BSpline::isValid: spline not valid: less than 2 fit
            // points";
            return false;
        }
        return true;
    }
    else {
        if (controlPoints.count() < degree + 1) {
            // qDebug() << "BSpline::isValid: spline not valid: less than " <<
            // degree+1 << " control points";
            return false;
        }
        return true;
    }
    /*
    bool ret = curve.ctrlPnts().size() > degree &&
           curve.ctrlPnts().size() + degree + 1 == curve.knot().size();
    if (!ret) {
        qWarning() << "RSpine::isValid: false";
    }
    return ret;
    */
    // return true;
}

double BSpline::getTDelta() const
{
    return getTMax() - getTMin();
}

double BSpline::getTMin() const
{
    updateInternal();

    if (isValid()) {
#ifndef R_NO_OPENNURBS
        return curve.Domain().Min();
#endif
    }

    return 0.0;
}

double BSpline::getTMax() const
{
    updateInternal();

    if (isValid()) {
#ifndef R_NO_OPENNURBS
        return curve.Domain().Max();
#endif
    }

    return 0.0;
}

double BSpline::getTAtPoint(const Vec3d &point) const
{
    if (hasProxy()) {
        // TODO: fails for splines with clamped control points in the middle
        // (multiple control points at the same location):
        return splineProxy->getTAtPoint(*this, point);
    }

    return 0.0;
}

double BSpline::getTAtDistance(double distance) const
{
    if (hasProxy()) {
        return splineProxy->getTAtDistance(*this, distance);
    }
    return 0.0;
}

double BSpline::getDistanceAtT(double t) const
{
    if (hasProxy()) {
        return splineProxy->getDistanceAtT(*this, t);
    }
    return 0.0;
}

std::vector<BSpline>
BSpline::getSegments(const std::vector<Vec3d> &points) const
{
    return splitAtPoints(points);
}

std::vector<Vec3d> BSpline::getDiscontinuities() const
{
    updateInternal();

    std::vector<Vec3d> ret;

#ifndef R_NO_OPENNURBS
    if (isValid()) {
        for (int c = 0; c <= 11; c++) {
            double t0 = getTMin();
            double t1 = getTMax();
            bool found;
            do {
                double t;
                found =
                    curve.GetNextDiscontinuity((ON::continuity)c, t0, t1, &t);
                if (found) {
                    ret.push_back(getPointAt(t));
                    t0 = t;
                }
            } while (found);
        }
    }
#endif

    return ret;
}

BSpline BSpline::simplify(double tolerance)
{
    if (hasProxy()) {
        return splineProxy->simplify(*this, tolerance);
    }
    return *this;
}

void BSpline::invalidate() const
{
#ifndef R_NO_OPENNURBS
    curve.Destroy();
    // curve.Initialize();
#endif
    exploded.clear();
    length = std::numeric_limits<double>::quiet_NaN();
}

void BSpline::updateInternal() const
{
    if (!dirty || updateInProgress) {
        return;
    }

    dirty = false;
    updateInProgress = true;

    if (degree < 1) {
        invalidate();
        qWarning() << "BSpline::updateInternal: invalid degree: " << degree;
        updateInProgress = false;
        return;
    }

    exploded.clear();
    length = std::numeric_limits<double>::quiet_NaN();

    // if fit points are known, update from fit points, otherwise from
    // control points:
    // TODO: use fitpoints from DXF/DWG file if possible (fit points might not
    // correspond to control points):
    if (fitPoints.size() == 0) {
        updateFromControlPoints();
    }
    else {
        updateFromFitPoints();
    }

    // updateBoundingBox();
    boundingBox = BBox();
    // getExploded();

    updateInProgress = false;
}

void BSpline::updateFromControlPoints() const
{
#ifndef R_NO_OPENNURBS
    if (controlPoints.size() < degree + 1) {
        invalidate();
        qWarning()
            << "BSpline::updateFromControlPoints: not enough control points: "
            << controlPoints.size();
        return;
    }

    // periodic:
    if (periodic && !hasFitPoints()) {
        ON_3dPoint *points = new ON_3dPoint[controlPoints.size()];
        for (int i = 0; i < controlPoints.size(); ++i) {
            Vec3d cp = controlPoints.at(i);
            points[i] = ON_3dPoint(cp.x, cp.y, cp.z);
        }
        curve.CreatePeriodicUniformNurbs(3, getOrder(), controlPoints.size(),
                                         points);
        delete[] points;
    }

    // open or from fit points:
    else {
        curve.Create(3, false, getOrder(), controlPoints.size());
        // curve.Create(3, true, getOrder(), controlPoints.size());

        // setting control points:
        for (int i = 0; i < controlPoints.size(); ++i) {
            Vec3d cp = controlPoints.at(i);
            ON_3dPoint onp(cp.x, cp.y, cp.z);
            // ON_4dPoint onp(cp.x, cp.y, cp.z, weights.size()>i ?
            // weights.at(i) : 1.0);
            curve.SetCV(i, onp);

            //            if (i<weights.size()) {
            //                double w = weights.at(i);
            //                curve.SetWeight(i, w);
            //            }
            // qDebug() << "BSpline: controlPoints[" << i << "]: " << cp;
        }

        bool knotCondition =
            (knotVector.size() == getOrder() + controlPoints.size() - 2);
        // knotCondition = true;

        // genetate knot vector automatically:
        if (knotVector.isEmpty() || !knotCondition) {
            //            if (!knotVector.isEmpty()) {
            //                qDebug() << "BSpline: knotVector ignored";
            //                qDebug() << "BSpline:   knots: " <<
            //                knotVector.size(); qDebug() << "BSpline:   order:
            //                " << getOrder(); qDebug() << "BSpline:
            //                controlPoints: " << controlPoints.size();
            //            }

            int si = ON_KnotCount(getOrder(), controlPoints.size());
            double *knot = new double[si];
            // ON_MakePeriodicUniformKnotVector(getOrder(),
            // controlPoints.size(), knot);
            ON_MakeClampedUniformKnotVector(getOrder(), controlPoints.size(),
                                            knot);
            for (int i = 0; i < si; ++i) {
                //                qDebug() << "BSpline: knot[" << i << "]: " <<
                //                knot[i];
                curve.SetKnot(i, knot[i]);
            }
            delete[] knot;
        }
        else {
            int k = 0;
            for (int i = 0; i < knotVector.count(); ++i) {
                // qDebug() << "BSpline: knot[" << i << "]: " <<
                // knotVector.at(i);
                bool ok = curve.SetKnot(k++, knotVector.at(i));
                if (!ok) {
                    // qDebug() << "BSpline: knot[" << i << "]: NOT set";
                }
            }
        }
    }

    // set weights:
    //    qDebug() << "rat:" << curve.m_is_rat;
    //    //curve.m_is_rat = 1;
    //    for (int i=0; i<weights.size(); ++i) {
    //        if (i<curve.CVCount()) {
    //            double w = weights.at(i);
    //            curve.SetWeight(i, w);
    //            qDebug() << "set weight of internal curve:" << w;
    //        }
    //        //qDebug() << "BSpline: controlPoints[" << i << "]: " << cp;
    //    }

    //    for (int i=0; i<curve.CVCount(); ++i) {
    //        qDebug() << "weight internal:" << curve.Weight(i);
    //    }

    // ##getExploded();
#endif
}

/**
 * Closes this spline and makes it periodic if it isn't already.
 */
/*void BSpline::close() {
    if (hasFitPoints()) {
        if (!isValid()) {
            return;
        }

        if (isClosed() && isPeriodic()) {
            return;
        }

        Vec3d fp0 = getFitPoints().at(0);
        appendFitPoint(fp0);
        updateTangentsPeriodic();
    }
    else {
        ON_3dPoint* points = new ON_3dPoint[controlPoints.size()];
        for (int i=0; i<controlPoints.size(); ++i) {
            Vec3d cp = controlPoints.at(i);
            points[i] = ON_3dPoint(cp.x, cp.y, cp.z);
        }
        curve.CreatePeriodicUniformNurbs(3, getOrder(), controlPoints.size(),
points); delete[] points;

        / *
        int si = controlPoints.size() + degree + 1;
        PlVector_double knot(si-1);
        double v=0.0;
        for (int i=0; i<si-1; ++i) {
            knot[i] = v;
            //qDebug() << "knot[<< " << i << "]: " << knot[i];
            v+=1.0/(si-2);
        }

        Vector_HPoint2Dd pointsWrapped;
        PlVector_double knotWrapped;
        PLib::wrapPointVectorH(curve.ctrlPnts(),degree,pointsWrapped);
        PLib::knotAveragingClosed(knot,degree,knotWrapped);

        v = 0.0;
        for (int i=0; i<knotWrapped.n(); ++i) {
            knotWrapped[i] = v;
            //qDebug() << "knotWrapped[<< " << i << "]: " << knotWrapped[i];
            v+=1.0/(knotWrapped.n()-1);
        }

        curve.reset(pointsWrapped, knotWrapped, degree);

        controlPoints.clear();
        Vector_HPoint2Dd ctrlPts = curve.ctrlPnts();
        Vector_HPoint2Dd::iterator it;
        for (it=ctrlPts.begin(); it!=ctrlPts.end(); ++it) {
            PLib::HPoint2Dd p = *it;
            controlPoints.push_back(Vec3d(p.x(), p.y()));
        }

        knotVector.clear();
        PlVector_double knotV = curve.knot();
        PlVector_double::iterator it2;
        for (it2=knotV.begin(); it2!=knotV.end(); it2++) {
            double v = *it2;
            knotVector.push_back(v);
        }

        updateBoundingBox();
        * /
    }
}
*/

/**
 * Updates the internal spline data from \c fitPoints.
 * Degree is always corrected to 3rd degree.
 */
void BSpline::updateFromFitPoints() const
{
    // spline with two fit points is line:
    if (fitPoints.size() < 2) {
        invalidate();
        return;
    }

    // call into plugin
    if (hasProxy()) {
        BSpline spline = splineProxy->updateFromFitPoints(*this);
        //        qDebug() << "tan start before:" << this->tangentStart;
        //        qDebug() << "tan start:" << spline.tangentStart;
        this->degree = spline.degree;
        this->periodic = spline.periodic;
        this->controlPoints = spline.controlPoints;
        this->knotVector = spline.knotVector;
        this->weights = spline.weights;
        this->tangentStart = spline.tangentStart;
        this->tangentEnd = spline.tangentEnd;
        this->curve = spline.curve;
        this->dirty = false;
    }
    else {
        invalidate();
        return;
    }
}

/**
 * Updates the internal bounding box.
 */
void BSpline::updateBoundingBox() const
{
    // getExploded();
    RPainterPath pp;
    pp.addSpline(*this);
    boundingBox = pp.getBoundingBox();
}

/**
 * \return List of bezier spline segments which together represent this curve.
 */
std::vector<BSpline> BSpline::getBezierSegments(const BBox &queryBox) const
{
    int ctrlCount = countControlPoints();

    // spline is a single bezier segment:
    if (ctrlCount == getDegree() + 1) {
        return std::vector<BSpline>() << *this;
    }

    updateInternal();

    std::vector<BSpline> ret;
#ifndef R_NO_OPENNURBS
    if (ctrlCount > 0) {
        ON_NurbsCurve *dup =
            dynamic_cast<ON_NurbsCurve *>(curve.DuplicateCurve());
        if (dup == NULL) {
            return ret;
        }

        dup->MakePiecewiseBezier();
        for (int i = 0; i <= dup->CVCount() - dup->Order(); ++i) {
            ON_BezierCurve bc;
            if (!dup->ConvertSpanToBezier(i, bc)) {
                continue;
            }

            std::vector<Vec3d> ctrlPts;
            for (int cpi = 0; cpi < bc.CVCount(); cpi++) {
                ON_3dPoint onp;
                bc.GetCV(cpi, onp);
                ctrlPts.push_back(Vec3d(onp.x, onp.y, onp.z));
            }
            BSpline bezierSegment(ctrlPts, degree);

            if (!queryBox.isValid() ||
                queryBox.intersects(bezierSegment.getBoundingBox())) {
                ret.push_back(bezierSegment);
            }
        }
        delete dup;
    }
#endif

    return ret;
}

NS::Ending BSpline::getTrimEnd(const Vec3d &trimPoint, const Vec3d &clickPoint)
{
    double tAtClickPoint = getTAtPoint(clickPoint);
    double tAtTrimPoint = getTAtPoint(trimPoint);

    if (tAtTrimPoint < tAtClickPoint) {
        return NS::EndingStart;
    }
    else {
        return NS::EndingEnd;
    }
}

bool BSpline::trimStartPoint(const Vec3d &trimPoint, const Vec3d &clickPoint,
                             bool extend)
{
    // Q_UNUSED(clickPoint)
    // Q_UNUSED(extend)
    if (!isValid()) {
        return false;
    }
    if (trimPoint.equalsFuzzy(getStartPoint())) {
        return true;
    }
    if (trimPoint.equalsFuzzy(getEndPoint())) {
        this->invalidate();
        return true;
    }

    std::vector<BSpline> splines =
        splitAtPoints(std::vector<Vec3d>() << trimPoint);
    if (splines.size() > 1) {
        copySpline(splines[1]);
    }
    update();
    return true;
}

bool BSpline::trimEndPoint(const Vec3d &trimPoint, const Vec3d &clickPoint,
                           bool extend)
{
    // Q_UNUSED(clickPoint)
    // Q_UNUSED(extend)
    if (!isValid()) {
        return false;
    }
    if (trimPoint.equalsFuzzy(getStartPoint())) {
        this->invalidate();
        return true;
    }
    if (trimPoint.equalsFuzzy(getEndPoint())) {
        return true;
    }

    std::vector<BSpline> splines =
        splitAtPoints(std::vector<Vec3d>() << trimPoint);
    if (splines.size() > 0) {
        copySpline(splines[0]);
    }
    update();
    return true;
}

double BSpline::getDistanceFromStart(const Vec3d &p) const
{
    double t = getTAtPoint(p);
    return getDistanceAtT(t);
}

std::vector<BSpline>
BSpline::splitAtPoints(const std::vector<Vec3d> &points) const
{
    std::vector<double> params;
    for (int i = 0; i < points.size(); i++) {
        params.push_back(getTAtPoint(points[i]));
    }
    return splitAtParams(params);
}

std::vector<BSpline>
BSpline::splitAtParams(const std::vector<double> &params) const
{
    if (hasProxy()) {
        return splineProxy->split(*this, params);
    }
    return std::vector<BSpline>();
}

void BSpline::update() const
{
    dirty = true;
    boundingBox = BBox();
    exploded.clear();
}

/**
 * \return New spline that covers this spline from d1 to d2, where
 *      d1 and d2 are distances from the start point of this spline.
 */
// BSpline BSpline::getSubSpline(double d1, double d2) const {
/*
double u1 = d1 / getLength() * getTMax();
//double u2 = d2 / getLength();
PLib::NurbsCurve_2Dd curve1;
PLib::NurbsCurve_2Dd curve2;
curve.splitAt(u1, curve1, curve2);
double u2 = (d2 - d1) / curve2.size() *
curve2.knot(curve2.knot().n()-1-degree); PLib::NurbsCurve_2Dd curve3;
PLib::NurbsCurve_2Dd curve4;
curve2.splitAt(u2, curve3, curve4);

return BSpline::createFrom(curve3);
*/
// return BSpline();
//}

/**
 * \internal
 */
/*
BSpline BSpline::createFrom(PLib::NurbsCurve_2Dd& sp) {
    std::vector<Vec3d> ctrlPts;

    Vector_HPoint2Dd cps = sp.ctrlPnts();
    Vector_HPoint2Dd::iterator it;
    for (it=cps.begin(); it!=cps.end(); it++) {
        PLib::HPoint2Dd p = *it;
        ctrlPts.push_back(Vec3d(p.x(), p.y()));
    }

    return BSpline(ctrlPts, sp.degree());
}
*/

std::vector<std::shared_ptr<Shape>>
BSpline::splitAt(const std::vector<Vec3d> &points) const
{
    if (points.size() == 0 || !BSpline::hasProxy()) {
        return Shape::splitAt(points);
    }

    std::vector<std::shared_ptr<Shape>> ret;

    QMultiMap<double, Vec3d> sortable;
    for (int i = 0; i < points.size(); i++) {
        double t = getTAtPoint(points[i]);
        sortable.insert(t, points[i]);
    }

    std::vector<double> keys = sortable.keys();
#if QT_VERSION >= 0x060000
    std::sort(keys.begin(), keys.end());
#else
    qSort(keys);
#endif

    std::vector<Vec3d> sortedPoints;
    for (int i = 0; i < keys.size(); i++) {
        std::vector<Vec3d> values = sortable.values(keys[i]);
        for (int k = 0; k < values.size(); k++) {
            sortedPoints.push_back(values[k]);
        }
    }

    std::vector<BSpline> subSplines = splitAtPoints(sortedPoints);
    for (int i = 0; i < subSplines.size(); i++) {
        ret.push_back(std::shared_ptr<Shape>(subSplines[i].clone()));
    }
    return ret;
}

/**
 * Finds _some_ self intersection points of splines. Used for snapping to those
 * intersections. Note that this does not find all intersections and might also
 * return non-intersections. Most notably, self-intersections of bezier segments
 * are not detected. This is not suitable to reliably detect the existence of
 * self intersections.
 */
std::vector<Vec3d> BSpline::getSelfIntersectionPoints(double tolerance) const
{
    return getIntersectionPointsSS(*this, *this, true, true, tolerance);
}
