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
#include <vector>
#include <map>
#include <algorithm>

#include <cada2d/RArc.h>
#include <cada2d/RLine.h>
#include <cada2d/RSpline.h>
#include <cada2d/RPolyline.h>

RSpline::RSpline()
    : m_degree(3), m_periodic(false), m_dirty(true), m_updateInProgress(false),
      m_length(RNANDOUBLE)
{
}

RSpline::RSpline(const RSpline &other)
{
    *this = other;
}

RSpline::RSpline(const std::vector<RVector> &controlPoints, int degree)
    : m_controlPoints(controlPoints), m_degree(degree), m_periodic(false),
      m_dirty(true), m_updateInProgress(false), m_length(RNANDOUBLE)
{

    updateInternal();
}

void RSpline::copySpline(const RSpline &other)
{
}

RS::ShapeType RSpline::getShapeType() const
{
    return RS::Spline;
}

bool RSpline::isDirected() const
{
    return false;
}

RSpline *RSpline::clone() const
{
    return nullptr;
}

RSpline::~RSpline()
{
}
std::vector<RSpline> RSpline::createSplinesFromArc(const RArc &arc)
{
    RArc a = arc;
    bool reversed = false;
    if (a.isReversed()) {
        reversed = true;
        a.reverse();
    }

    double startAngle = RMath::getNormalizedAngle(a.getStartAngle());
    double endAngle = RMath::getNormalizedAngle(a.getEndAngle());
    if (a.isFullCircle()) {
        startAngle = 0.0;
        endAngle = 2 * M_PI;
    }

    // normalize startAngle, endAngle to [-2PI, 2PI]
    double twoPI = M_PI * 2;
    // double startAngle = RMath::getNormalizedAngle(a.getStartAngle());
    // double endAngle = RMath::getNormalizedAngle(a.getEndAngle());
    if (startAngle > endAngle) {
        startAngle -= 2 * M_PI;
    }
    double radius = a.getRadius();
    double EPSILON = 0.00001;

    // Compute the sequence of arc curves, up to PI/2 at a time.  Total arc
    // angle is less than 2PI.

    std::vector<RSpline> curves;

    double piOverTwo = M_PI_2;
    double segmentationAngle = piOverTwo / 8;
    // double segmentationAngle = M_PI/8;
    double sgn = (startAngle < endAngle) ? +1 : -1;

    double a1 = startAngle;
    for (double totalAngle = qMin(twoPI, std::fabs(endAngle - startAngle));
         totalAngle > EPSILON;) {
        double a2 = a1 + sgn * qMin(totalAngle, segmentationAngle);
        RSpline sp = RSpline::createBezierFromSmallArc(radius, a1, a2);
        sp.move(a.getCenter());
        if (reversed) {
            sp.reverse();
            curves.insert(curves.begin(), sp);
        }
        else {
            curves.push_back(sp);
        }
        totalAngle -= std::fabs(a2 - a1);
        a1 = a2;
    }

    return curves;
}

RSpline RSpline::createBezierFromSmallArc(double r, double a1, double a2)
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

    std::vector<RVector> ctrlPts = {
        RVector(r * cos(a1), r * sin(a1)),
        RVector(x2 * cos_ar - y2 * sin_ar, x2 * sin_ar + y2 * cos_ar),
        RVector(x3 * cos_ar - y3 * sin_ar, x3 * sin_ar + y3 * cos_ar),
        RVector(r * cos(a2), r * sin(a2))};

    // this should be cubic but appears to be far off if cubic
    return RSpline(ctrlPts, 2);
}

bool RSpline::isInterpolated() const
{
    return false;
}

void RSpline::appendControlPoint(const RVector &point)
{
    m_controlPoints.push_back(point);
    update();
}

void RSpline::appendControlPoints(const std::vector<RVector> &points)
{
    m_controlPoints.insert(m_controlPoints.end(), points.begin(), points.end());
    update();
}

void RSpline::removeLastControlPoint()
{
    m_controlPoints.pop_back();
    update();
}

void RSpline::setControlPoints(const std::vector<RVector> &controlPoints)
{
    this->m_controlPoints = controlPoints;
    update();
}

std::vector<RVector> RSpline::getControlPoints() const
{
    return m_controlPoints;
}

std::vector<RVector> RSpline::getControlPointsWrapped() const
{
    std::vector<RVector> ret;

    updateInternal();

#ifndef R_NO_OPENNURBS
    ON_3dPoint onp;
    for (int i = 0; i < m_curve.CVCount(); ++i) {
        m_curve.GetCV(i, onp);
        ret.push_back(RVector(onp.x, onp.y));
    }
#endif

    return ret;
}

int RSpline::countControlPoints() const
{
    return m_controlPoints.size();
}

RVector RSpline::getControlPointAt(int i) const
{
    if (i >= 0 && i < m_controlPoints.size()) {
        return m_controlPoints.at(i);
    }
    return RVector::invalid;
}

void RSpline::appendFitPoint(const RVector &point)
{
    m_fitPoints.push_back(point);
    update();
}

void RSpline::prependFitPoint(const RVector &point)
{
    m_fitPoints.insert(m_fitPoints.begin(), point);
    update();
}

void RSpline::insertFitPointAt(const RVector &point)
{
    RVector p = getClosestPointOnShape(point);

    // find out T at the point closest to point:
    double t = getTAtPoint(p);

    insertFitPointAt(t, p);
}

void RSpline::insertFitPointAt(double t, const RVector &p)
{
    // find out index of fit point before t:
    int index = -1;
    for (int i = 0; i < m_fitPoints.size(); i++) {
        double tc = getTAtPoint(m_fitPoints[i]);
        if (i == 0 &&
            (isClosed() || getStartPoint().equalsFuzzy(getEndPoint()))) {
            // closed spline: two t's for first fit point:
            tc = 0.0;
        }
        if (tc < t) {
            index = i + 1;
        }
        else {
            break;
        }
    }

    // point not on spline:
    if (index < 0 || index >= m_fitPoints.size()) {
        if (isClosed()) {
            index = 0;
        }
        else {
            return;
        }
    }

    m_fitPoints.insert(m_fitPoints.begin() + index, p);
    update();
}

void RSpline::removeFitPointAt(const RVector &point)
{
    double minDist = RMAXDOUBLE;
    int index = -1;
    for (int i = 0; i < m_fitPoints.size(); i++) {
        double dist = point.getDistanceTo(m_fitPoints[i]);
        if (dist < minDist) {
            minDist = dist;
            index = i;
        }
    }

    if (index < 0 || index >= m_fitPoints.size()) {
        return;
    }

    m_fitPoints.erase(m_fitPoints.begin() + index);
    update();
}

void RSpline::removeLastFitPoint()
{
    m_fitPoints.pop_back();
    update();
}

void RSpline::removeFirstFitPoint()
{
    m_fitPoints.erase(m_fitPoints.begin());
    update();
}

void RSpline::setFitPoints(const std::vector<RVector> &fitPoints)
{
    this->m_fitPoints = fitPoints;
    update();
}

std::vector<RVector> RSpline::getFitPoints() const
{
    return m_fitPoints;
}

int RSpline::countFitPoints() const
{
    return m_fitPoints.size();
}

bool RSpline::hasFitPoints() const
{
    return !m_fitPoints.empty();
}

RVector RSpline::getFitPointAt(int i) const
{
    if (i >= 0 && i < m_fitPoints.size()) {
        return m_fitPoints.at(i);
    }
    return RVector::invalid;
}

std::vector<double> RSpline::getKnotVector() const
{
    return m_knotVector;
}

std::vector<double> RSpline::getActualKnotVector() const
{
    updateInternal();
    std::vector<double> ret;
    for (int i = 0; i < m_curve.KnotCount(); ++i) {
        ret.push_back(m_curve.Knot(i));
    }
    return ret;
}

void RSpline::setKnotVector(const std::vector<double> &knots)
{
    m_knotVector = knots;
    update();
}

void RSpline::appendKnot(double k)
{
    m_knotVector.push_back(k);
    update();
}

std::vector<double> RSpline::getWeights() const
{
    return m_weights;
}

void RSpline::setWeights(std::vector<double> &w)
{
    m_weights = w;
}

void RSpline::setDegree(int d)
{
    m_degree = d;
    update();
}

int RSpline::getDegree() const
{
    return m_degree;
}

int RSpline::getOrder() const
{
    return m_degree + 1;
}

void RSpline::setPeriodic(bool on)
{
    m_periodic = on;
    update();
}

bool RSpline::isClosed() const
{
    return m_periodic;
}

bool RSpline::isGeometricallyClosed(double tolerance) const
{
    return isClosed() ||
           getStartPoint().getDistanceTo(getEndPoint()) < tolerance;
}

bool RSpline::isPeriodic() const
{
    return m_periodic;
}

double RSpline::getDirection1() const
{
    if (!isValid()) {
        return 0.0;
    }
    updateInternal();

    ON_3dVector ontan = m_curve.TangentAt(getTMin());
    RVector rtan(ontan.x, ontan.y);
    return rtan.getAngle();
}

double RSpline::getDirection2() const
{
    if (!isValid()) {
        return 0.0;
    }
    updateInternal();

    ON_3dVector ontan = m_curve.TangentAt(getTMax());
    RVector rtan(ontan.x, ontan.y);
    return RMath::getNormalizedAngle(rtan.getAngle() + M_PI);
}

RS::Side RSpline::getSideOfPoint(const RVector &point) const
{
    RPolyline pl = toPolyline(16);
    return pl.getSideOfPoint(point);
}

RVector RSpline::getStartPoint() const
{
    return getPointAt(getTMin());
}

void RSpline::setStartPoint(const RVector &v)
{
    m_controlPoints[0] = v;
    update();
}

RVector RSpline::getEndPoint() const
{
    return getPointAt(getTMax());
}

void RSpline::setEndPoint(const RVector &v)
{
    // TODO: handle fit points
    m_controlPoints[m_controlPoints.size() - 1] = v;
    update();
}

void RSpline::setTangents(const RVector &start, const RVector &end)
{
    m_tangentStart = start;
    m_tangentEnd = end;
    update();
}

void RSpline::setTangentAtStart(const RVector &t)
{
    m_tangentStart = t;
    update();
}

RVector RSpline::getTangentAtStart() const
{
    return m_tangentStart;
}

void RSpline::setTangentAtEnd(const RVector &t)
{
    m_tangentEnd = t;
    update();
}

RVector RSpline::getTangentAtEnd() const
{
    return m_tangentEnd;
}

void RSpline::unsetTangentAtStart()
{
    setTangentAtStart(RVector::invalid);
}

void RSpline::unsetTangentAtEnd()
{
    setTangentAtEnd(RVector::invalid);
}

void RSpline::unsetTangents()
{
    setTangents(RVector::invalid, RVector::invalid);
}

void RSpline::updateTangentsPeriodic()
{
    unsetTangents();

    double tangent1 = getDirection1();
    double tangent2 = RMath::getNormalizedAngle(getDirection2() + M_PI);
    RVector v1 = RVector::createPolar(1.0, tangent1);
    RVector v2 = RVector::createPolar(1.0, tangent2);
    RVector t = (v1 + v2).getNormalized();

    setTangents(t, t);
}

RPolyline RSpline::approximateWithArcs(double tolerance,
                                       double radiusLimit) const
{
    return RPolyline();
}

RPolyline RSpline::toPolyline(int segments) const
{
    RPolyline ret;

    std::vector<std::shared_ptr<RShape>> lineSegments =
        getExplodedBezier(segments);
    for (int k = 0; k < lineSegments.size(); k++) {
        std::shared_ptr<RShape> shape = lineSegments[k];
        if (!shape || !shape->isDirected()) {
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

std::vector<std::shared_ptr<RShape>> RSpline::getExploded(int segments) const
{
    if (!m_exploded.empty() && segments == -1) {
        return m_exploded;
    }

    // ##boundingBox = RBox();

    updateInternal();

    m_exploded.clear();

    if (!isValid()) {
        return m_exploded;
    }

    if (segments == -1) {
        segments = 8;
    }

    double tMin = getTMin();
    double tMax = getTMax();

    double step = getTDelta() / (m_controlPoints.size() * segments);

    RVector p1;
    RVector prev = RVector::invalid;
    for (double t = tMin; t < tMax + (step / 2.0); t += step) {
        double tc = qMin(t, tMax);
        p1 = getPointAt(tc);

        if (RMath::isNaN(p1.x) || RMath::isNaN(p1.y)) {
            continue;
        }

        if (prev.isValid()) {
            appendToExploded(RLine(prev, p1));
        }
        prev = p1;

        // ##boundingBox.growToInclude(p1);
    }

    p1 = getEndPoint();
    if (!RMath::isNaN(p1.x) && !RMath::isNaN(p1.y)) {
        if (prev.isValid()) {
            appendToExploded(RLine(prev, p1));
        }
    }

    return m_exploded;
}

std::vector<std::shared_ptr<RShape>>
RSpline::getExplodedBezier(int segments) const
{
    std::vector<std::shared_ptr<RShape>> ret;
    std::vector<RSpline> bezierSegments = getBezierSegments();
    for (int i = 0; i < bezierSegments.size(); i++) {
        auto tls = bezierSegments[i].getExploded(segments);
        ret.insert(ret.end(), tls.begin(), tls.end());
    }
    return ret;
}

void RSpline::appendToExploded(const RLine &line) const
{
    if (line.getLength() < 1.0e-6) {
        return;
    }

    if (!m_exploded.empty()) {
        // compare angle of this sement with last segment and
        // modify last segment if angle is the same (straight line):
        std::shared_ptr<RLine> prev =
            std::dynamic_pointer_cast<RLine>(m_exploded.back());
        if (prev) {
            if (RMath::fuzzyCompare(
                    prev->getAngle(),
                    prev->getStartPoint().getAngleTo(line.getEndPoint()))) {
                prev->setEndPoint(line.getEndPoint());
                return;
            }
        }
    }

    m_exploded.push_back(std::shared_ptr<RShape>(new RLine(line)));
}

std::vector<std::shared_ptr<RShape>>
RSpline::getExplodedWithSegmentLength(double segmentLength) const
{
    std::vector<std::shared_ptr<RShape>> ret;
    std::vector<RSpline> bezierSegments = getBezierSegments();
    for (int i = 0; i < bezierSegments.size(); i++) {
        double len = bezierSegments[i].getLength();
        int seg = static_cast<int>(ceil(len / segmentLength));
        auto tls = bezierSegments[i].getExploded(seg);
        ret.insert(ret.end(), tls.begin(), tls.end());
    }
    return ret;
}

RBox RSpline::getBoundingBox() const
{
    if (!isValid()) {
        return RBox();
    }

    if (!m_boundingBox.isValid()) {
        updateBoundingBox();
    }

    return m_boundingBox;
}

double RSpline::getLength() const
{
    if (!isValid()) {
        return 0.0;
    }
    if (!m_dirty && !RMath::isNaN(m_length)) {
        return m_length;
    }

    m_length = 0.0;
    std::vector<std::shared_ptr<RShape>> shapes = getExploded();
    for (int i = 0; i < shapes.size(); i++) {
        std::shared_ptr<RShape> shape = shapes[i];
        m_length += shape->getLength();
    }

    return m_length;

    // seems to only work in the context of another product which uses
    // OpenNURBS: curve.GetLength(&length);
}

/**
 * \return Point on spline at given position t (0..1).
 */
RVector RSpline::getPointAt(double t) const
{
    updateInternal();
#ifndef R_NO_OPENNURBS
    ON_3dPoint p = m_curve.PointAt(t);
    if (p.IsUnset()) {
        return RVector::invalid;
    }
    return RVector(p.x, p.y);
#else
    return RVector::invalid;
#endif
}

RVector RSpline::getPointAtDistance(double distance) const
{
    double t = getTAtDistance(distance);
    return getPointAt(t);
}

double RSpline::getAngleAt(double distance, RS::From from) const
{
    std::vector<RVector> points = getPointsWithDistanceToEnd(distance, from);
    if (points.size() != 1) {
        return RNANDOUBLE;
    }
    double t = getTAtPoint(points[0]);
    ON_3dVector v = m_curve.DerivativeAt(t);
    return RVector(v.x, v.y).getAngle();
}

std::vector<RVector> RSpline::getEndPoints() const
{
    std::vector<RVector> ret;

    ret.push_back(getStartPoint());
    ret.push_back(getEndPoint());

    return ret;
}

RVector RSpline::getMiddlePoint() const
{
    return getPointAt(getTMin() + (getTDelta() / 2.0));
}

std::vector<RVector> RSpline::getMiddlePoints() const
{
    std::vector<RVector> ret;

    ret.push_back(getMiddlePoint());

    return ret;
}

std::vector<RVector> RSpline::getCenterPoints() const
{
    return std::vector<RVector>();
}

std::vector<RVector> RSpline::getPointsWithDistanceToEnd(double distance,
                                                         int from) const
{
    std::vector<RVector> ret;

    // no spline proxy (not precise, but better than nothing in some cases):
    double length = getLength();
    if (length <= RS::PointTolerance) {
        return ret;
    }

    if (from & RS::FromStart) {
        RVector p = getPointAt(getTMin() + (distance / length * getTDelta()));
        ret.push_back(p);
    }

    if (from & RS::FromEnd) {
        RVector p = getPointAt(getTMin() +
                               ((length - distance) / length * getTDelta()));
        ret.push_back(p);
    }

    return ret;
}

std::vector<RVector> RSpline::getPointCloud(double segmentLength) const
{
    RPolyline pl = approximateWithArcs(0.01);
    return pl.getPointCloud(segmentLength);
}

RVector RSpline::getVectorTo(const RVector &point, bool limited,
                             double strictRange) const
{
    RVector ret = RVector::invalid;

    std::vector<std::shared_ptr<RShape>> sub = getExploded();
    std::vector<std::shared_ptr<RShape>>::iterator it;
    for (it = sub.begin(); it != sub.end(); ++it) {
        RVector v = (*it)->getVectorTo(point, limited, strictRange);
        if (v.isValid() &&
            (!ret.isValid() || v.getMagnitude() < ret.getMagnitude())) {
            ret = v;
        }
    }

    return ret;
}

bool RSpline::isOnShape(const RVector &point, bool limited,
                        double tolerance) const
{
    double t = getTAtPoint(point);
    RVector p = getPointAt(t);
    return point.getDistanceTo(p) < tolerance;
}

bool RSpline::move(const RVector &offset)
{
    for (int i = 0; i < m_controlPoints.size(); i++) {
        m_controlPoints[i].move(offset);
    }
    for (int i = 0; i < m_fitPoints.size(); i++) {
        m_fitPoints[i].move(offset);
    }
    update();
    return true;
}

bool RSpline::rotate(double rotation, const RVector &center)
{
    if (fabs(rotation) < RS::AngleTolerance) {
        return false;
    }
    for (int i = 0; i < m_controlPoints.size(); i++) {
        m_controlPoints[i].rotate(rotation, center);
    }
    for (int i = 0; i < m_fitPoints.size(); i++) {
        m_fitPoints[i].rotate(rotation, center);
    }
    m_tangentStart.rotate(rotation);
    m_tangentEnd.rotate(rotation);
    update();
    return true;
}

bool RSpline::scale(const RVector &scaleFactors, const RVector &center)
{
    for (int i = 0; i < m_controlPoints.size(); i++) {
        m_controlPoints[i].scale(scaleFactors, center);
    }
    for (int i = 0; i < m_fitPoints.size(); i++) {
        m_fitPoints[i].scale(scaleFactors, center);
    }
    update();
    return true;
}

bool RSpline::mirror(const RLine &axis)
{
    RVector sp = getStartPoint();
    RVector ep = getEndPoint();

    for (int i = 0; i < m_controlPoints.size(); i++) {
        m_controlPoints[i].mirror(axis.getStartPoint(), axis.getEndPoint());
    }
    for (int i = 0; i < m_fitPoints.size(); i++) {
        m_fitPoints[i].mirror(axis.getStartPoint(), axis.getEndPoint());
    }

    RVector absTan = sp + m_tangentStart;
    absTan.mirror(axis.getStartPoint(), axis.getEndPoint());
    sp.mirror(axis.getStartPoint(), axis.getEndPoint());
    m_tangentStart = absTan - sp;

    absTan = ep + m_tangentEnd;
    absTan.mirror(axis.getStartPoint(), axis.getEndPoint());
    ep.mirror(axis.getStartPoint(), axis.getEndPoint());
    m_tangentEnd = absTan - ep;

    update();

    return true;
}

bool RSpline::flipHorizontal()
{
    for (int i = 0; i < m_controlPoints.size(); i++) {
        m_controlPoints[i].flipHorizontal();
    }
    for (int i = 0; i < m_fitPoints.size(); i++) {
        m_fitPoints[i].flipHorizontal();
    }
    m_tangentStart.flipHorizontal();
    m_tangentEnd.flipHorizontal();
    update();
    return true;
}

bool RSpline::flipVertical()
{
    for (int i = 0; i < m_controlPoints.size(); i++) {
        m_controlPoints[i].flipVertical();
    }
    for (int i = 0; i < m_fitPoints.size(); i++) {
        m_fitPoints[i].flipVertical();
    }
    m_tangentStart.flipVertical();
    m_tangentEnd.flipVertical();
    update();
    return true;
}

bool RSpline::reverse()
{
    int k;
    if (!isClosed()) {
        for (k = 0; k < m_controlPoints.size() / 2; k++) {
            std::swap(m_controlPoints[k],
                      m_controlPoints[m_controlPoints.size() - (1 + k)]);
        }
        for (k = 0; k < m_fitPoints.size() / 2; k++) {
            std::swap(m_fitPoints[k],
                      m_fitPoints[m_fitPoints.size() - (1 + k)]);
        }
        double t;
        int i, j;
        for (i = 0, j = m_knotVector.size() - 1; i <= j; i++, j--) {
            t = m_knotVector[i];
            m_knotVector[i] = -m_knotVector[j];
            m_knotVector[j] = -t;
        }
        RVector ts = m_tangentStart;
        m_tangentStart = m_tangentEnd.getNegated();
        m_tangentEnd = ts.getNegated();
    }
    else {
        if (hasFitPoints()) {
            for (k = 0; k < (int)floor(m_fitPoints.size() / 2.0); k++) {
                std::swap(m_fitPoints[k],
                          m_fitPoints[m_fitPoints.size() - (1 + k)]);
            }
            // keep start node the same:
            auto &&e = m_fitPoints.back();
            m_fitPoints.pop_back();
            m_fitPoints.insert(m_fitPoints.begin(), e);
        }
        else {
            for (k = 0; k < m_controlPoints.size() / 2; k++) {
                std::swap(m_controlPoints[k],
                          m_controlPoints[m_controlPoints.size() - (1 + k)]);
            }
        }
        updateTangentsPeriodic();
    }
    update();
    return true;
}

bool RSpline::stretch(const RPolyline &area, const RVector &offset)
{
    if (!m_fitPoints.empty()) {
        for (int i = 0; i < m_fitPoints.size(); i++) {
            m_fitPoints[i].stretch(area, offset);
        }
        update();
        return true;
    }
    return false;
}

bool RSpline::isValid() const
{
    if (!m_dirty) {
        return m_curve.IsValid();
    }

    if (m_degree < 1) {
        return false;
    }
    if (hasFitPoints()) {
        // spline with two fit points is line:
        if (m_fitPoints.size() < 2) {
            return false;
        }
        return true;
    }
    else {
        if (m_controlPoints.size() < m_degree + 1) {
            return false;
        }
        return true;
    }
}

double RSpline::getTDelta() const
{
    return getTMax() - getTMin();
}

double RSpline::getTMin() const
{
    updateInternal();

    if (isValid()) {
        return m_curve.Domain().Min();
    }

    return 0.0;
}

double RSpline::getTMax() const
{
    updateInternal();

    if (isValid()) {
        return m_curve.Domain().Max();
    }

    return 0.0;
}

double RSpline::getTAtPoint(const RVector &point) const
{

    return 0.0;
}

double RSpline::getTAtDistance(double distance) const
{

    return 0.0;
}

double RSpline::getDistanceAtT(double t) const
{

    return 0.0;
}

std::vector<RSpline>
RSpline::getSegments(const std::vector<RVector> &points) const
{
    return splitAtPoints(points);
}

std::vector<RVector> RSpline::getDiscontinuities() const
{
    updateInternal();

    std::vector<RVector> ret;

    if (isValid()) {
        for (int c = 0; c <= 11; c++) {
            double t0 = getTMin();
            double t1 = getTMax();
            bool found;
            do {
                double t;
                found =
                    m_curve.GetNextDiscontinuity((ON::continuity)c, t0, t1, &t);
                if (found) {
                    ret.push_back(getPointAt(t));
                    t0 = t;
                }
            } while (found);
        }
    }

    return ret;
}

RSpline RSpline::simplify(double tolerance)
{
    return *this;
}

void RSpline::invalidate() const
{
    m_curve.Destroy();

    m_exploded.clear();
    m_length = RNANDOUBLE;
}

void RSpline::updateInternal() const
{
    if (!m_dirty || m_updateInProgress) {
        return;
    }

    m_dirty = false;
    m_updateInProgress = true;

    if (m_degree < 1) {
        invalidate();
        m_updateInProgress = false;
        return;
    }

    m_exploded.clear();
    m_length = RNANDOUBLE;

    // if fit points are known, update from fit points, otherwise from
    // control points:
    // TODO: use fitpoints from DXF/DWG file if possible (fit points might not
    // correspond to control points):
    if (m_fitPoints.size() == 0) {
        updateFromControlPoints();
    }
    else {
        updateFromFitPoints();
    }

    // updateBoundingBox();
    m_boundingBox = RBox();
    // getExploded();

    m_updateInProgress = false;
}

void RSpline::updateFromControlPoints() const
{
    if (m_controlPoints.size() < m_degree + 1) {
        invalidate();
        return;
    }

    // periodic:
    if (m_periodic && !hasFitPoints()) {
        ON_3dPoint *points = new ON_3dPoint[m_controlPoints.size()];
        for (int i = 0; i < m_controlPoints.size(); ++i) {
            RVector cp = m_controlPoints.at(i);
            points[i] = ON_3dPoint(cp.x, cp.y, 0);
        }
        m_curve.CreatePeriodicUniformNurbs(3, getOrder(),
                                           m_controlPoints.size(), points);
        delete[] points;
    }

    // open or from fit points:
    else {
        m_curve.Create(3, false, getOrder(), m_controlPoints.size());
        // curve.Create(3, true, getOrder(), controlPoints.size());

        // setting control points:
        for (int i = 0; i < m_controlPoints.size(); ++i) {
            RVector cp = m_controlPoints.at(i);
            ON_3dPoint onp(cp.x, cp.y, 0);
            // ON_4dPoint onp(cp.x, cp.y, cp.z, weights.length()>i ?
            // weights.at(i) : 1.0);
            m_curve.SetCV(i, onp);
        }

        bool knotCondition =
            (m_knotVector.size() == getOrder() + m_controlPoints.size() - 2);
        // knotCondition = true;

        // genetate knot vector automatically:
        if (m_knotVector.empty() || !knotCondition) {
            int si = ON_KnotCount(getOrder(), m_controlPoints.size());
            double *knot = new double[si];
            // ON_MakePeriodicUniformKnotVector(getOrder(),
            // controlPoints.size(), knot);
            ON_MakeClampedUniformKnotVector(getOrder(), m_controlPoints.size(),
                                            knot);
            for (int i = 0; i < si; ++i) {
                m_curve.SetKnot(i, knot[i]);
            }
            delete[] knot;
        }
        else {
            int k = 0;
            for (int i = 0; i < m_knotVector.size(); ++i) {
                bool ok = m_curve.SetKnot(k++, m_knotVector.at(i));
            }
        }
    }
}

void RSpline::updateFromFitPoints() const
{
    // spline with two fit points is line:
    if (m_fitPoints.size() < 2) {
        invalidate();
        return;
    }
    invalidate();
    return;
}

void RSpline::updateBoundingBox() const
{
    // getExploded();
}

std::vector<RSpline> RSpline::getBezierSegments(const RBox &queryBox) const
{
    int ctrlCount = countControlPoints();

    // spline is a single bezier segment:
    if (ctrlCount == getDegree() + 1) {
        return {*this};
    }

    updateInternal();

    std::vector<RSpline> ret;
    if (ctrlCount > 0) {
        ON_NurbsCurve *dup =
            dynamic_cast<ON_NurbsCurve *>(m_curve.DuplicateCurve());
        if (dup == NULL) {
            return ret;
        }

        dup->MakePiecewiseBezier();
        for (int i = 0; i <= dup->CVCount() - dup->Order(); ++i) {
            ON_BezierCurve bc;
            if (!dup->ConvertSpanToBezier(i, bc)) {
                continue;
            }

            std::vector<RVector> ctrlPts;
            for (int cpi = 0; cpi < bc.CVCount(); cpi++) {
                ON_3dPoint onp;
                bc.GetCV(cpi, onp);
                ctrlPts.push_back(RVector(onp.x, onp.y, 0));
            }
            RSpline bezierSegment(ctrlPts, m_degree);

            if (!queryBox.isValid() ||
                queryBox.intersects(bezierSegment.getBoundingBox())) {
                ret.push_back(bezierSegment);
            }
        }
        delete dup;
    }

    return ret;
}

RS::Ending RSpline::getTrimEnd(const RVector &trimPoint,
                               const RVector &clickPoint)
{
    double tAtClickPoint = getTAtPoint(clickPoint);
    double tAtTrimPoint = getTAtPoint(trimPoint);

    if (tAtTrimPoint < tAtClickPoint) {
        return RS::EndingStart;
    }
    else {
        return RS::EndingEnd;
    }
}

bool RSpline::trimStartPoint(const RVector &trimPoint,
                             const RVector &clickPoint, bool extend)
{
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

    std::vector<RSpline> splines = splitAtPoints({trimPoint});
    if (splines.size() > 1) {
        copySpline(splines[1]);
    }
    update();
    return true;
}

bool RSpline::trimEndPoint(const RVector &trimPoint, const RVector &clickPoint,
                           bool extend)
{
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

    std::vector<RSpline> splines = splitAtPoints({trimPoint});
    if (splines.size() > 0) {
        copySpline(splines[0]);
    }
    update();
    return true;
}

bool RSpline::trimStartPoint(double trimDist)
{
    return false;
}

bool RSpline::trimEndPoint(double trimDist)
{
    return false;
}

double RSpline::getDistanceFromStart(const RVector &p) const
{
    double t = getTAtPoint(p);
    return getDistanceAtT(t);
}

std::vector<RSpline>
RSpline::splitAtPoints(const std::vector<RVector> &points) const
{
    std::vector<double> params;
    for (int i = 0; i < points.size(); i++) {
        params.push_back(getTAtPoint(points[i]));
    }
    return splitAtParams(params);
}

std::vector<RSpline>
RSpline::splitAtParams(const std::vector<double> &params) const
{
    return std::vector<RSpline>();
}

void RSpline::update() const
{
    m_dirty = true;
    m_boundingBox = RBox();
    m_exploded.clear();
}

std::vector<std::shared_ptr<RShape>>
RSpline::splitAt(const std::vector<RVector> &points) const
{
    if (points.size() == 0) {
        return RShape::splitAt(points);
    }

    std::vector<std::shared_ptr<RShape>> ret;

    std::multimap<double, RVector> sortable;
    for (int i = 0; i < points.size(); i++) {
        double t = getTAtPoint(points[i]);
        sortable.insert({t, points[i]});
    }

    std::vector<double> keys(sortable.size());
    std::transform(
        sortable.begin(), sortable.end(), keys.begin(),
        [](const std::pair<double, RVector> &pair) { return pair.first; });
    std::sort(keys.begin(), keys.end());

    std::vector<RVector> sortedPoints;
    for (int i = 0; i < keys.size(); i++) {
        std::vector<RVector> values(sortable.size());
        std::transform(
            sortable.begin(), sortable.end(), values.begin(),
            [](const std::pair<double, RVector> &pair) { return pair.second; });
        for (int k = 0; k < values.size(); k++) {
            sortedPoints.push_back(values[k]);
        }
    }

    std::vector<RSpline> subSplines = splitAtPoints(sortedPoints);
    for (int i = 0; i < subSplines.size(); i++) {
        ret.push_back(std::shared_ptr<RShape>(subSplines[i].clone()));
    }
    return ret;
}

std::vector<RVector> RSpline::getSelfIntersectionPoints(double tolerance) const
{
    return std::vector<RVector>();
}
