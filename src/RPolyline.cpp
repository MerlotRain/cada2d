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

#include <assert.h>
#include <cmath>

#include <cada2d/RArc.h>
#include <cada2d/RBox.h>
#include <cada2d/RCircle.h>
#include <cada2d/REllipse.h>
#include <cada2d/RLine.h>
#include <cada2d/RPolyline.h>
#include <cada2d/RSpline.h>
#include <cada2d/private/RShapePrivate.h>

RPolyline::RPolyline() : m_closed(false) {}

RPolyline::RPolyline(const std::vector<RVector> &vertices, bool closed)
    : m_closed(closed)
{

    setVertices(vertices);
}

RPolyline::RPolyline(const std::vector<std::shared_ptr<RShape>> &segments)
    : m_closed(false)
{

    std::vector<std::shared_ptr<RShape>>::const_iterator it;
    for (it = segments.begin(); it != segments.end(); ++it)
    {
        std::shared_ptr<RShape> segment = *it;

        if (segment->isDirected())
        {
            if (m_vertices.size() == 0)
            {
                appendVertex(segment->getStartPoint(), 0.0);
            }
            appendVertex(segment->getEndPoint(), 0.0);
        }

        std::shared_ptr<RArc> arc = std::dynamic_pointer_cast<RArc>(segment);
        if (arc)
        {
            if (m_bulges.size() > 1)
            {
                m_bulges[m_bulges.size() - 2] = arc->getBulge();
            }
        }
    }

    assert(m_vertices.size() == m_bulges.size());
    assert(m_vertices.size() == m_startWidths.size());
    assert(m_vertices.size() == m_endWidths.size());

    autoClose();
}

RPolyline::~RPolyline() {}

RS::ShapeType RPolyline::getShapeType() const { return RS::Polyline; }

std::shared_ptr<RShape> RPolyline::clone() const
{
    return std::shared_ptr<RShape>(new RPolyline(*this));
}

bool RPolyline::isDirected() const { return true; }

void RPolyline::clear()
{
    m_vertices.clear();
    m_bulges.clear();
    m_startWidths.clear();
    m_endWidths.clear();

    assert(m_vertices.size() == m_bulges.size());
    assert(m_vertices.size() == m_startWidths.size());
    assert(m_vertices.size() == m_endWidths.size());
}

void RPolyline::normalize(double tolerance)
{
    std::vector<RVector> newVertices;
    std::vector<double> newBulges;
    std::vector<double> newStartWidths;
    std::vector<double> newEndWidths;

    RVector vPrev;
    int newIndex = 0;

    for (int i = 0; i < m_vertices.size(); i++)
    {
        RVector v = m_vertices[i];
        double b = m_bulges[i];
        double s = m_startWidths[i];
        double e = m_endWidths[i];

        if (i == 0 || !v.equalsFuzzy(vPrev, tolerance))
        {
            newVertices.push_back(v);
            newBulges.push_back(b);
            newStartWidths.push_back(s);
            newEndWidths.push_back(e);
            newIndex = newIndex + 1;
            vPrev = v;
        }
        else if (i > 0)
        {
            newBulges[newIndex - 1] = b;
            newStartWidths[newIndex - 1] = s;
            newEndWidths[newIndex - 1] = e;
        }
    }

    // remove duplicate last vertex of closed polyline:
    if (m_closed)
    {
        if (newVertices.front().equalsFuzzy(newVertices.back(), tolerance))
        {
            newVertices.pop_back();
            newBulges.pop_back();
            newStartWidths.pop_back();
            newEndWidths.pop_back();
        }
    }

    m_vertices = newVertices;
    m_bulges = newBulges;
    m_startWidths = newStartWidths;
    m_endWidths = newEndWidths;

    assert(m_vertices.size() == m_bulges.size());
    assert(m_vertices.size() == m_startWidths.size());
    assert(m_vertices.size() == m_endWidths.size());
}

bool RPolyline::prependShape(const RShape &shape)
{
    return appendShape(shape, true);
}

bool RPolyline::appendShape(const RShape &shape, bool prepend)
{
    bool ret = true;

    // append spline as polyline approximation:
    if (shape.getShapeType() == RS::Spline)
    {
        const RSpline *spl = dynamic_cast<const RSpline *>(&shape);
        if (spl != NULL)
        {
            RPolyline pl = spl->approximateWithArcs(0.01);
            return appendShape(pl, prepend);
        }
    }

    // append ellipse as polyline approximation:
    else if (shape.getShapeType() == RS::Ellipse)
    {
        const REllipse *elp = dynamic_cast<const REllipse *>(&shape);
        if (elp != NULL)
        {
            RPolyline pl = elp->approximateWithArcs(32);
            return appendShape(pl, prepend);
        }
    }

    // append circle as polyline to empty polyline:
    else if (shape.getShapeType() == RS::Circle && isEmpty())
    {
        const RCircle *circle = dynamic_cast<const RCircle *>(&shape);
        if (circle != NULL)
        {
            appendShape(RArc(circle->getCenter(), circle->getRadius(), 0.0,
                             M_PI, false));
            appendShape(RArc(circle->getCenter(), circle->getRadius(), M_PI,
                             2 * M_PI, false));
            return true;
        }
    }

    // append full circle arc as circle (two arc segments) to empty polyline:
    else if (shape.getShapeType() == RS::Arc)
    {
        const RArc *arc = dynamic_cast<const RArc *>(&shape);
        if (arc != NULL && arc->isFullCircle())
        {
            appendShape(RCircle(arc->getCenter(), arc->getRadius()));
            return true;
        }
    }

    // append polyline:
    else if (shape.getShapeType() == RS::Polyline)
    {
        const RPolyline *pl = dynamic_cast<const RPolyline *>(&shape);
        if (pl != NULL)
        {
            if (prepend)
            {
                for (int i = pl->countSegments() - 1; i >= 0; --i)
                {
                    std::shared_ptr<RShape> s = pl->getSegmentAt(i);
                    if (!s) { continue; }
                    ret = prependShape(*s) && ret;
                    setStartWidthAt(0, pl->getStartWidthAt(i));
                    setEndWidthAt(0, pl->getEndWidthAt(i));
                }
            }
            else
            {
                for (int i = 0; i < pl->countSegments(); ++i)
                {
                    std::shared_ptr<RShape> s = pl->getSegmentAt(i);
                    if (!s) { continue; }
                    setStartWidthAt(m_vertices.size() - 1,
                                    pl->getStartWidthAt(i));
                    setEndWidthAt(m_vertices.size() - 1, pl->getEndWidthAt(i));
                    ret = appendShape(*s) && ret;
                }
            }
            return ret;
        }
    }

    double bulge = 0.0;

    const RArc *arc = dynamic_cast<const RArc *>(&shape);
    if (arc != NULL) { bulge = arc->getBulge(); }

    if (!shape.isDirected()) { return false; }

    RVector connectionPoint;
    RVector nextPoint;
    double gap;
    if (prepend)
    {
        // prepend:
        connectionPoint = shape.getEndPoint();
        nextPoint = shape.getStartPoint();
        if (m_vertices.size() == 0)
        {
            // first point:
            appendVertex(connectionPoint);
        }
        gap = m_vertices.front().getDistanceTo(connectionPoint);
    }
    else
    {
        // append:
        connectionPoint = shape.getStartPoint();
        nextPoint = shape.getEndPoint();
        if (m_vertices.size() == 0)
        {
            // first point:
            appendVertex(connectionPoint);
        }
        gap = m_vertices.back().getDistanceTo(connectionPoint);
    }

    if (!RMath::fuzzyCompare(gap, 0.0, 1.0e-3)) { ret = false; }

    if (prepend)
    {
        prependVertex(nextPoint);
        setBulgeAt(0, bulge);
    }
    else
    {
        appendVertex(nextPoint);
        setBulgeAt(m_bulges.size() - 2, bulge);
    }

    assert(m_vertices.size() == m_bulges.size());
    assert(m_vertices.size() == m_startWidths.size());
    assert(m_vertices.size() == m_endWidths.size());

    return ret;
}

bool RPolyline::appendShapeAuto(const RShape &shape)
{
    if (!shape.isDirected()) { return false; }

    if (countVertices() > 0 && getEndPoint().equalsFuzzy(shape.getEndPoint()))
    {
        std::shared_ptr<RShape> rev = std::shared_ptr<RShape>(shape.clone());
        rev->reverse();
        return appendShape(*rev);
    }

    return appendShape(shape);
}

bool RPolyline::appendShapeTrim(const RShape &shape)
{
    if (!shape.isDirected()) { return false; }

    if (countVertices() > 0)
    {
        if (getEndPoint().equalsFuzzy(shape.getStartPoint()))
        {
            return appendShape(shape);
        }
        if (getEndPoint().equalsFuzzy(shape.getEndPoint()))
        {
            std::shared_ptr<RShape> rev =
                    std::shared_ptr<RShape>(shape.clone());
            rev->reverse();
            return appendShape(*rev);
        }

        if (shape.getShapeType() == RS::Line)
        {
            std::shared_ptr<RShape> lastSegment = getLastSegment();
            std::vector<RVector> ips =
                    lastSegment->getIntersectionPoints(shape, false);
            if (ips.size() == 1)
            {
                RVector ip = ips[0];
                moveEndPoint(ip);
                std::shared_ptr<RShape> trimmed =
                        std::shared_ptr<RShape>(shape.clone());
                trimmed->trimStartPoint(ip);
                return appendShape(*trimmed);
            }
        }
    }

    return appendShape(shape);
}

bool RPolyline::closeTrim()
{
    if (isGeometricallyClosed()) { return true; }

    if (countSegments() > 1)
    {
        std::shared_ptr<RShape> firstSegment = getFirstSegment();
        std::shared_ptr<RShape> lastSegment = getLastSegment();

        if (!firstSegment || !lastSegment) { return false; }

        if (firstSegment->getShapeType() == RS::Line &&
            lastSegment->getShapeType() == RS::Line)
        {
            std::vector<RVector> ips =
                    lastSegment->getIntersectionPoints(*firstSegment, false);
            if (ips.size() == 1)
            {
                RVector ip = ips[0];
                moveStartPoint(ip);
                moveEndPoint(ip);
                return true;
            }
        }
    }

    return false;
}

void RPolyline::appendVertex(const RVector &vertex, double bulge, double w1,
                             double w2)
{
    m_vertices.push_back(vertex);
    m_bulges.push_back(bulge);
    m_startWidths.push_back(w1);
    m_endWidths.push_back(w2);

    assert(m_vertices.size() == m_bulges.size());
    assert(m_vertices.size() == m_startWidths.size());
    assert(m_vertices.size() == m_endWidths.size());
}

void RPolyline::appendVertex(double x, double y, double bulge, double w1,
                             double w2)
{
    appendVertex(RVector(x, y), bulge, w1, w2);
}

void RPolyline::prependVertex(const RVector &vertex, double bulge, double w1,
                              double w2)
{

    m_vertices.insert(m_vertices.begin(), vertex);
    m_bulges.insert(m_bulges.begin(), bulge);
    m_startWidths.insert(m_startWidths.begin(), w1);
    m_endWidths.insert(m_endWidths.begin(), w2);

    assert(m_vertices.size() == m_bulges.size());
    assert(m_vertices.size() == m_startWidths.size());
    assert(m_vertices.size() == m_endWidths.size());
}

void RPolyline::insertVertex(int index, const RVector &vertex,
                             double bulgeBefore, double bulgeAfter)
{

    m_vertices.insert(m_vertices.begin() + index, vertex);
    if (index > 0) { m_bulges[index - 1] = bulgeBefore; }
    m_bulges.insert(m_bulges.begin() + index, bulgeAfter);
    m_startWidths.insert(m_startWidths.begin() + index, 0.0);
    m_endWidths.insert(m_endWidths.begin() + index, 0.0);

    assert(m_vertices.size() == m_bulges.size());
    assert(m_vertices.size() == m_startWidths.size());
    assert(m_vertices.size() == m_endWidths.size());
}

void RPolyline::insertVertexAt(const RVector &point)
{
    int index = getClosestSegment(point);
    if (index < 0) { return; }

    std::shared_ptr<RShape> seg1 = getSegmentAt(index);
    if (!seg1) { return; }

    RVector p = seg1->getClosestPointOnShape(point, false);

    std::shared_ptr<RShape> seg2 = std::shared_ptr<RShape>(seg1->clone());

    if (!seg1->isDirected() || !seg2->isDirected()) { return; }

    seg1->trimEndPoint(p);
    seg2->trimStartPoint(p);

    insertVertex(index + 1, p);

    std::shared_ptr<RArc> arc1 = std::dynamic_pointer_cast<RArc>(seg1);
    std::shared_ptr<RArc> arc2 = std::dynamic_pointer_cast<RArc>(seg2);
    if (!arc1) { setBulgeAt(index, 0.0); }
    else { setBulgeAt(index, arc1->getBulge()); }

    if (!arc2) { setBulgeAt(index + 1, 0.0); }
    else { setBulgeAt(index + 1, arc2->getBulge()); }

    assert(m_vertices.size() == m_bulges.size());
    assert(m_vertices.size() == m_startWidths.size());
    assert(m_vertices.size() == m_endWidths.size());
}

RVector RPolyline::insertVertexAtDistance(double dist)
{
    return RVector::invalid;
}

void RPolyline::removeFirstVertex()
{
    if (m_vertices.empty()) { return; }

    m_vertices.erase(m_vertices.begin());
    m_bulges.erase(m_bulges.begin());
    m_startWidths.erase(m_startWidths.begin());
    m_endWidths.erase(m_endWidths.begin());

    assert(m_vertices.size() == m_bulges.size());
    assert(m_vertices.size() == m_startWidths.size());
    assert(m_vertices.size() == m_endWidths.size());
}

void RPolyline::removeLastVertex()
{
    if (m_vertices.empty()) { return; }

    m_vertices.pop_back();
    m_bulges.pop_back();
    m_startWidths.pop_back();
    m_endWidths.pop_back();

    assert(m_vertices.size() == m_bulges.size());
    assert(m_vertices.size() == m_startWidths.size());
    assert(m_vertices.size() == m_endWidths.size());
}

void RPolyline::removeVertex(int index)
{

    m_vertices.erase(m_vertices.begin() + index);
    m_bulges.erase(m_bulges.begin() + index);
    m_startWidths.erase(m_startWidths.begin() + index);
    m_endWidths.erase(m_endWidths.begin() + index);

    assert(m_vertices.size() == m_bulges.size());
    assert(m_vertices.size() == m_startWidths.size());
    assert(m_vertices.size() == m_endWidths.size());
}

void RPolyline::removeVerticesAfter(int index)
{
    m_vertices = {m_vertices.begin(), m_vertices.begin() + index + 1};
    m_bulges = {m_bulges.begin(), m_bulges.begin() + index + 1};
    m_startWidths = {m_startWidths.begin(), m_startWidths.begin() + index + 1};
    m_endWidths = {m_endWidths.begin(), m_endWidths.begin() + index + 1};

    assert(m_vertices.size() == m_bulges.size());
    assert(m_vertices.size() == m_startWidths.size());
    assert(m_vertices.size() == m_endWidths.size());
}

void RPolyline::removeVerticesBefore(int index)
{
    m_vertices = {m_vertices.begin() + index, m_vertices.end()};
    m_bulges = {m_bulges.begin() + index, m_bulges.end()};
    m_startWidths = {m_startWidths.begin() + index, m_startWidths.end()};
    m_endWidths = {m_endWidths.begin() + index, m_endWidths.end()};

    assert(m_vertices.size() == m_bulges.size());
    assert(m_vertices.size() == m_startWidths.size());
    assert(m_vertices.size() == m_endWidths.size());
}

void RPolyline::setVertices(const std::vector<RVector> &vertices)
{
    this->m_vertices = vertices;

    m_bulges.clear();
    m_startWidths.clear();
    m_endWidths.clear();
    for (int i = 0; i < vertices.size(); ++i)
    {
        m_bulges.push_back(0.0);
        m_startWidths.push_back(0.0);
        m_endWidths.push_back(0.0);
    }

    assert(m_vertices.size() == m_bulges.size());
    assert(m_vertices.size() == m_startWidths.size());
    assert(m_vertices.size() == m_endWidths.size());
}

std::vector<RVector> RPolyline::getVertices() const { return m_vertices; }

RVector RPolyline::getVertexAt(int i) const
{
    if (i < 0 || i >= m_vertices.size())
    {
        assert(false);
        return RVector::invalid;
    }

    return m_vertices.at(i);
}

int RPolyline::getVertexIndex(const RVector &v, double tolerance) const
{
    for (int i = 0; i < m_vertices.size(); i++)
    {
        if (m_vertices[i].equalsFuzzy(v, tolerance)) { return i; }
    }

    return -1;
}

RVector RPolyline::getLastVertex() const
{
    if (m_vertices.size() == 0) { return RVector::invalid; }

    return m_vertices.at(m_vertices.size() - 1);
}

void RPolyline::setVertexAt(int i, const RVector &v)
{
    if (i < 0 || i >= m_vertices.size())
    {
        assert(false);
        return;
    }

    m_vertices[i] = v;
}

void RPolyline::moveVertexAt(int i, const RVector &offset)
{
    if (i < 0 || i >= m_vertices.size())
    {
        assert(false);
        return;
    }

    m_vertices[i] += offset;
}

int RPolyline::countVertices() const { return m_vertices.size(); }

void RPolyline::setBulges(const std::vector<double> &b) { m_bulges = b; }

std::vector<double> RPolyline::getBulges() const { return m_bulges; }

double RPolyline::getBulgeAt(int i) const
{
    if (i < 0 || i >= m_bulges.size()) { return RNANDOUBLE; }

    return m_bulges.at(i);
}

void RPolyline::setBulgeAt(int i, double b)
{
    if (i < 0 || i >= m_bulges.size()) { return; }

    m_bulges[i] = b;
}

bool RPolyline::hasArcSegments() const
{
    for (int i = 0; i < m_bulges.size(); i++)
    {
        if (!isStraight(m_bulges[i])) { return true; }
    }

    return false;
}

std::vector<double> RPolyline::getVertexAngles() const
{
    RS::Orientation orientation = getOrientation(true);
    std::vector<double> ret;
    for (int i = 0; i < countVertices(); i++)
    {
        ret.push_back(getVertexAngle(i, orientation));
    }
    return ret;
}

double RPolyline::getVertexAngle(int i, RS::Orientation orientation) const
{
    if (!isGeometricallyClosed() && (i == 0 || i == countVertices() - 1))
    {
        // angles at first / last vertex for open polyline:
        return 0.0;
    }

    if (countSegments() == 0) { return 0.0; }

    std::shared_ptr<RShape> prevSegment =
            getSegmentAt(RMath::absmod(i - 1, countSegments()));
    std::shared_ptr<RShape> nextSegment = getSegmentAt(i % countSegments());

    // angle from vertex to next segment:
    double aNext = nextSegment->getDirection1();
    // angle from vertex to previous segment:
    double aPrev = prevSegment->getDirection2();

    if (orientation == RS::UnknownOrientation)
    {
        orientation = getOrientation(true);
    }
    if (orientation == RS::CW)
    {
        return RMath::getAngleDifference(aPrev, aNext);
    }
    else { return RMath::getAngleDifference(aNext, aPrev); }
}

void RPolyline::setGlobalWidth(double w)
{
    for (int i = 0; i < m_startWidths.size(); i++) { m_startWidths[i] = w; }
    for (int i = 0; i < m_endWidths.size(); i++) { m_endWidths[i] = w; }
}

void RPolyline::setStartWidthAt(int i, double w)
{
    if (i < 0 || i >= m_startWidths.size()) { return; }
    m_startWidths[i] = w;
}

double RPolyline::getStartWidthAt(int i) const
{
    if (i < 0 || i >= m_startWidths.size()) { return -1.0; }

    return m_startWidths.at(i);
}

void RPolyline::setEndWidthAt(int i, double w)
{
    if (i < 0 || i >= m_endWidths.size()) { return; }
    m_endWidths[i] = w;
}

double RPolyline::getEndWidthAt(int i) const
{
    if (i < 0 || i >= m_endWidths.size()) { return -1.0; }

    return m_endWidths.at(i);
}

bool RPolyline::hasWidths() const
{
    for (int i = 0; i < m_startWidths.size() && i < m_endWidths.size(); i++)
    {
        if (!RMath::isNaN(m_startWidths[i]) && m_startWidths[i] > 0.0)
        {
            // widths in last vertex only count if closed:
            if (i != m_startWidths.size() - 1 || isClosed()) { return true; }
        }
        if (!RMath::isNaN(m_endWidths[i]) && m_endWidths[i] > 0.0)
        {
            if (i != m_startWidths.size() - 1 || isClosed()) { return true; }
        }
    }

    return false;
}

void RPolyline::setStartWidths(const std::vector<double> &sw)
{
    m_startWidths = sw;
}

std::vector<double> RPolyline::getStartWidths() const { return m_startWidths; }

void RPolyline::setEndWidths(const std::vector<double> &ew)
{
    m_endWidths = ew;
}

std::vector<double> RPolyline::getEndWidths() const { return m_endWidths; }

void RPolyline::setClosed(bool on) { m_closed = on; }

bool RPolyline::isClosed() const { return m_closed; }

bool RPolyline::isGeometricallyClosed(double tolerance) const
{
    return isClosed() ||
           getStartPoint().getDistanceTo(getEndPoint()) < tolerance;
}

bool RPolyline::toLogicallyClosed(double tolerance)
{
    if (isClosed()) { return false; }

    if (!isGeometricallyClosed(tolerance)) { return false; }

    removeLastVertex();
    setClosed(true);

    assert(m_vertices.size() == m_bulges.size());
    assert(m_vertices.size() == m_startWidths.size());
    assert(m_vertices.size() == m_endWidths.size());

    return true;
}

bool RPolyline::toLogicallyOpen()
{
    if (!isClosed()) { return false; }

    appendVertex(getEndPoint(), getBulgeAt(m_vertices.size() - 1));
    setClosed(false);

    assert(m_vertices.size() == m_bulges.size());
    assert(m_vertices.size() == m_startWidths.size());
    assert(m_vertices.size() == m_endWidths.size());

    return true;
}

std::vector<RVector>
RPolyline::getSelfIntersectionPoints(double tolerance) const
{
    std::vector<RVector> ret;

    bool cl = isGeometricallyClosed();

    std::vector<std::shared_ptr<RShape>> segments = getExploded();
    for (int i = 0; i < segments.size(); i++)
    {
        std::shared_ptr<RShape> segment = getSegmentAt(i);

        for (int k = i + 1; k < segments.size(); k++)
        {
            std::shared_ptr<RShape> otherSegment = getSegmentAt(k);

            std::vector<RVector> ips =
                    segment->getIntersectionPoints(*otherSegment);
            for (int n = 0; n < ips.size(); n++)
            {
                RVector ip = ips[n];
                if (k == i + 1 &&
                    ip.equalsFuzzy(segment->getEndPoint(), tolerance))
                {
                    // ignore intersection at vertex between two consecutive
                    // segments:
                    continue;
                }

                if (cl)
                {
                    if (i == 0 && k == segments.size() - 1 &&
                        ip.equalsFuzzy(segment->getStartPoint(), tolerance))
                    {
                        continue;
                    }
                }

                ret.push_back(ip);
            }
        }
    }

    return ret;
}

RS::Orientation RPolyline::getOrientation(bool implicitelyClosed) const
{
    if (!implicitelyClosed && !isGeometricallyClosed(0.00001))
    {
        return RS::Any;
    }

    if (countSegments() < 1) { return RS::Any; }

    if (hasArcSegments())
    {
        RPolyline plSegmented = convertArcToLineSegments(16);
        return plSegmented.getOrientation(implicitelyClosed);
    }

    RVector minV = RVector::invalid;
    std::shared_ptr<RShape> shapeBefore;
    std::shared_ptr<RShape> shapeAfter;
    std::shared_ptr<RShape> shape;
    std::shared_ptr<RShape> previousShape = getSegmentAt(countSegments() - 1);

    // find minimum vertex (lower left corner):
    std::vector<std::shared_ptr<RShape>> segments = getExploded();
    for (int i = 0; i < segments.size(); i++)
    {
        shape = getSegmentAt(i);
        if (!shape) { continue; }

        if (shape->getLength() < 0.001) { continue; }

        RVector v = shape->getStartPoint();
        if (!minV.isValid() || v.x < minV.x || (v.x == minV.x && v.y < minV.y))
        {
            minV = v;
            shapeBefore = previousShape;
            shapeAfter = shape;
        }

        previousShape = shape;
    }

    // TODO: fails for large arc (>180d) at bottom left corner, creating round
    // bottom left shape:
    //    double l;
    //    RVector p;
    //    std::vector<RVector> list;
    //    std::shared_ptr<RArc> arcBefore = shapeBefore.dynamicCast<RArc>();
    //    if (!arcBefore.isNull()) {
    //        l = arcBefore->getLength();
    //        list = arcBefore->getPointsWithDistanceToEnd(l/10, RS::FromEnd);
    //        if (!list.isEmpty()) {
    //            p = list[0];
    //            shapeBefore = std::shared_ptr<RLine>(new RLine(p,
    //            arcBefore->getEndPoint()));
    //        }
    //    }

    //    std::shared_ptr<RArc> arcAfter = shapeAfter.dynamicCast<RArc>();
    //    if (!arcAfter.isNull()) {
    //        l = arcAfter->getLength();
    //        list = arcAfter->getPointsWithDistanceToEnd(l/10, RS::FromStart);
    //        if (!list.isEmpty()) {
    //            p = list[0];
    //            shapeAfter = std::shared_ptr<RLine>(new
    //            RLine(arcAfter->getStartPoint(), p));
    //        }
    //    }

    if (!shapeBefore || !shapeAfter) { return RS::Any; }

    double xa = shapeBefore->getStartPoint().x;
    double ya = shapeBefore->getStartPoint().y;
    double xb = shapeAfter->getStartPoint().x;
    double yb = shapeAfter->getStartPoint().y;
    double xc = shapeAfter->getEndPoint().x;
    double yc = shapeAfter->getEndPoint().y;

    double det = (xb - xa) * (yc - ya) - (xc - xa) * (yb - ya);

    if (det < 0.0)
    {
        // clockwise:
        return RS::CW;
    }
    else
    {
        // counter-clockwise:
        return RS::CCW;
    }
}

bool RPolyline::setOrientation(RS::Orientation orientation)
{
    if (getOrientation(true) != orientation) { return reverse(); }
    return false;
}

RPolyline RPolyline::convertArcToLineSegments(int segments) const
{
    RPolyline ret;

    std::vector<std::shared_ptr<RShape>> segs = getExploded();
    for (int i = 0; i < segs.size(); i++)
    {
        std::shared_ptr<RShape> seg = segs[i];
        if (seg->getShapeType() == RS::Arc)
        {
            std::shared_ptr<RArc> arc = std::dynamic_pointer_cast<RArc>(seg);
            RPolyline pl =
                    arc->approximateWithLinesTan(arc->getLength() / segments);
            ret.appendShape(pl);
        }
        else { ret.appendShape(*seg); }
    }

    ret.autoClose();
    return ret;
}

RPolyline RPolyline::convertArcToLineSegmentsLength(double segmentLength) const
{
    RPolyline ret;

    std::vector<std::shared_ptr<RShape>> segs = getExploded();
    for (int i = 0; i < segs.size(); i++)
    {
        std::shared_ptr<RShape> seg = segs[i];
        if (seg->getShapeType() == RS::Arc)
        {
            std::shared_ptr<RArc> arc = std::dynamic_pointer_cast<RArc>(seg);
            RPolyline pl = arc->approximateWithLinesTan(segmentLength);
            ret.appendShape(pl);
        }
        else { ret.appendShape(*seg); }
    }

    ret.autoClose();
    return ret;
}

void RPolyline::stripWidths()
{
    for (int i = 0; i < m_startWidths.size(); i++) { m_startWidths[i] = 0.0; }
    for (int i = 0; i < m_endWidths.size(); i++) { m_endWidths[i] = 0.0; }
}

void RPolyline::setMinimumWidth(double w)
{
    for (int i = 0; i < m_startWidths.size(); i++)
    {
        if (m_startWidths[i] > RS::PointTolerance)
        {
            m_startWidths[i] = qMax(m_startWidths[i], w);
        }
    }
    for (int i = 0; i < m_endWidths.size(); i++)
    {
        if (m_endWidths[i] > RS::PointTolerance)
        {
            m_endWidths[i] = qMax(m_endWidths[i], w);
        }
    }
}

int RPolyline::getSegmentAtDist(double dist) { return -1; }

bool RPolyline::relocateStartPoint(const RVector &p) { return false; }

bool RPolyline::relocateStartPoint(double dist) { return false; }

bool RPolyline::convertToClosed()
{
    if (isClosed()) { return true; }

    if (!isGeometricallyClosed()) { return false; }

    removeLastVertex();
    setClosed(true);
    return true;
}

bool RPolyline::convertToOpen()
{
    if (!isClosed()) { return true; }

    std::shared_ptr<RShape> last = getLastSegment();
    setClosed(false);
    appendShape(*last);
    return true;
}

bool RPolyline::isLineSegment(int i) const
{
    if (i < 0 || i > m_bulges.size()) { return true; }

    return RPolyline::isStraight(m_bulges.at(i));
}

bool RPolyline::isStraight(double bulge) { return fabs(bulge) < 1.0e-6; }

std::vector<std::shared_ptr<RShape>> RPolyline::getExploded(int segments) const
{

    std::vector<std::shared_ptr<RShape>> ret;

    if (m_vertices.size() <= 1) { return ret; }

    for (int i = 0; i < m_vertices.size(); i++)
    {
        if (!m_closed && i == m_vertices.size() - 1) { break; }

        std::shared_ptr<RShape> subShape = getSegmentAt(i);
        if (!subShape) { continue; }

        ret.push_back(subShape);
    }

    return ret;
}

std::vector<std::pair<RPolyline, RPolyline>>
RPolyline::getLeftRightOutline() const
{
    return std::vector<std::pair<RPolyline, RPolyline>>();
}

std::vector<RPolyline> RPolyline::getOutline() const
{
    return std::vector<RPolyline>();
}

int RPolyline::countSegments() const
{
    int ret = countVertices();
    if (!m_closed) { ret -= 1; }
    if (ret < 0) { ret = 0; }
    return ret;
}

std::shared_ptr<RShape> RPolyline::getSegmentAt(int i) const
{
    if (i < 0 || i >= m_vertices.size() || i >= m_bulges.size())
    {
        return std::shared_ptr<RShape>();
    }

    RVector p1 = m_vertices.at(i);
    RVector p2 = m_vertices.at((i + 1) % m_vertices.size());

    if (RPolyline::isStraight(m_bulges.at(i)))
    {
        return std::shared_ptr<RShape>(new RLine(p1, p2));
    }

    else
    {
        double bulge = m_bulges.at(i);
        bool reversed = bulge < 0.0;
        double alpha = atan(bulge) * 4.0;

        if (fabs(alpha) > 2 * M_PI - RS::PointTolerance)
        {
            return std::shared_ptr<RShape>(new RLine(p1, p2));
            // return std::shared_ptr<RShape>();
        }

        double radius;
        RVector center;
        RVector middle;
        double dist;
        double angle;

        middle = (p1 + p2) / 2.0;
        dist = p1.getDistanceTo(p2) / 2.0;
        angle = p1.getAngleTo(p2);

        // alpha can't be 0.0 at this point
        radius = fabs(dist / sin(alpha / 2.0));

        double rootTerm = fabs(radius * radius - dist * dist);
        double h = sqrt(rootTerm);

        if (bulge > 0.0) { angle += M_PI / 2.0; }
        else { angle -= M_PI / 2.0; }

        if (fabs(alpha) > M_PI) { h *= -1.0; }

        center.setPolar(h, angle);
        center += middle;

        double a1;
        double a2;

        a1 = center.getAngleTo(p1);
        a2 = center.getAngleTo(p2);

        return std::shared_ptr<RShape>(
                new RArc(center, radius, a1, a2, reversed));
    }
}

bool RPolyline::isArcSegmentAt(int i) const
{
    if (i < 0 || i >= m_bulges.size()) { return false; }
    return !RPolyline::isStraight(m_bulges[i]);
}

std::shared_ptr<RShape> RPolyline::getLastSegment() const
{
    if (countSegments() == 0) { return std::shared_ptr<RShape>(); }
    return getSegmentAt(countSegments() - 1);
}

std::shared_ptr<RShape> RPolyline::getFirstSegment() const
{
    if (countSegments() == 0) { return std::shared_ptr<RShape>(); }
    return getSegmentAt(0);
}

bool RPolyline::contains(const RVector &point, bool borderIsInside,
                         double tolerance) const
{
    if (!isGeometricallyClosed(tolerance)) { return false; }

    // check if point is on polyline:
    if (isOnShape(point, true, tolerance)) { return borderIsInside; }

    if (hasArcSegments())
    {
        // TODO: not always reliable:
        // TODO
    }

    int nvert = m_vertices.size();
    int i, j;
    bool c = false;
    for (i = 0, j = nvert - 1; i < nvert; j = i++)
    {
        if (((m_vertices[i].y > point.y) != (m_vertices[j].y > point.y)) &&
            (point.x < (m_vertices[j].x - m_vertices[i].x) *
                                       (point.y - m_vertices[i].y) /
                                       (m_vertices[j].y - m_vertices[i].y) +
                               m_vertices[i].x))
        {
            c = !c;
        }
    }
    return c;
}

bool RPolyline::containsShape(const RShape &shape) const
{
    // check if the shape intersects with any of the polygon edges:
    bool gotIntersection = false;
    if (shape.intersectsWith(*this)) { gotIntersection = true; }

    if (gotIntersection)
    {
        // normal selection:
        // entity does not match if there is an intersection:
        return false;
    }

    if (shape.getShapeType() == RS::Polyline)
    {
        const RPolyline &pl = dynamic_cast<const RPolyline &>(shape);

        RBox bbOuter = pl.getBoundingBox();
        RBox bbInner = shape.getBoundingBox();

        if (!bbOuter.contains(bbInner)) { return false; }

        for (int i = 0; i < pl.countVertices() && i < 5; i++)
        {
            if (!contains(pl.getVertexAt(i))) { return false; }
        }
        return true;
    }

    // check if the shape is completely inside the polygon.
    // this is the case if one point on the entity is inside the polygon
    // and the entity does not intersect with the polygon.
    else if (shape.isDirected())
    {
        return contains(shape.getStartPoint()) && contains(shape.getEndPoint());
    }
    else
    {
        // circle:
        if (shape.getShapeType() == RS::Circle)
        {
            const RCircle &circle = dynamic_cast<const RCircle &>(shape);
            RVector p1 = circle.getCenter() + RVector(circle.getRadius(), 0);
            RVector p2 = circle.getCenter() + RVector(-circle.getRadius(), 0);
            if (contains(p1) || contains(p2)) { return true; }
            return false;
        }
        else
        {
            // other shapes:
            RVector pointOnShape = shape.getPointOnShape();
            if (contains(pointOnShape, true)) { return true; }
            return false;
        }
    }

    // unsupported shape:
    assert(false);
    return false;
}

RVector RPolyline::getPointInside() const { return RVector::invalid; }

RVector RPolyline::getStartPoint() const
{
    if (m_vertices.size() == 0) { return RVector::invalid; }

    return m_vertices.front();
}

RVector RPolyline::getEndPoint() const
{
    if (m_vertices.size() == 0) { return RVector::invalid; }

    if (isClosed()) { return m_vertices.front(); }

    return m_vertices.back();
}

RVector RPolyline::getMiddlePoint() const
{
    std::vector<RVector> pts =
            getPointsWithDistanceToEnd(getLength() / 2, RS::FromStart);
    if (pts.size() == 1) { return pts[0]; }
    return RVector::invalid;
}

void RPolyline::moveStartPoint(const RVector &pos)
{
    if (m_vertices.empty()) { return; }
    m_vertices.front() = pos;
}

void RPolyline::moveEndPoint(const RVector &pos)
{
    if (m_vertices.empty()) { return; }
    m_vertices.back() = pos;
}

void RPolyline::moveSegmentAt(int i, const RVector &offset)
{
    moveVertexAt(i, offset);
    if (i + 1 < countVertices()) { moveVertexAt(i + 1, offset); }
    else
    {
        if (m_closed) { moveVertexAt(0, offset); }
    }
}

double RPolyline::getDirection1() const
{
    if (m_vertices.size() == 0) { return RNANDOUBLE; }

    std::shared_ptr<RShape> shape = getSegmentAt(0);
    return shape->getDirection1();
}

double RPolyline::getDirection2() const
{
    if (m_vertices.size() == 0) { return RNANDOUBLE; }

    int i = m_vertices.size() - 2;
    if (isClosed()) { i++; }

    std::shared_ptr<RShape> shape = getSegmentAt(i);
    if (!shape) { return RNANDOUBLE; }
    return shape->getDirection2();
}

RS::Side RPolyline::getSideOfPoint(const RVector &point) const
{
    int i = getClosestSegment(point);
    if (i < 0 || i >= countSegments()) { return RS::NoSide; }

    std::shared_ptr<RShape> segment = getSegmentAt(i);
    if (!segment) { return RS::NoSide; }
    return segment->getSideOfPoint(point);
}

RBox RPolyline::getBoundingBox() const
{
    RBox ret;

    if (hasWidths())
    {
        std::vector<RPolyline> outline = getOutline();
        for (int i = 0; i < outline.size(); i++)
        {
            assert(!outline[i].hasWidths());
            RBox bb = outline[i].getBoundingBox();
            ret.growToInclude(bb);
        }
        return ret;
    }

    if (countVertices() == 1)
    {
        ret = RBox(m_vertices.at(0), m_vertices.at(0));
    }

    std::vector<std::shared_ptr<RShape>> sub = getExploded();
    std::vector<std::shared_ptr<RShape>>::iterator it;
    for (it = sub.begin(); it != sub.end(); ++it)
    {
        RBox bb = (*it)->getBoundingBox();
        ret.growToInclude(bb);
    }

    return ret;
}

double RPolyline::getArea() const
{
    double ret = 0.0;

    return ret;
}

double RPolyline::getLength() const
{
    double ret = 0.0;

    std::vector<std::shared_ptr<RShape>> sub = getExploded();
    std::vector<std::shared_ptr<RShape>>::iterator it;
    for (it = sub.begin(); it != sub.end(); ++it)
    {
        double l = (*it)->getLength();
        if (RMath::isNormal(l)) { ret += l; }
    }

    return ret;
}

double RPolyline::getLengthTo(const RVector &p, bool limited) const
{
    double ret = 0.0;

    if (p.equalsFuzzy(getStartPoint())) { return 0.0; }

    int segIdx = getClosestSegment(p);
    if (segIdx < 0) { return -1.0; }

    for (int i = 0; i < segIdx; i++)
    {
        double l = getSegmentAt(i)->getLength();
        if (RMath::isNormal(l)) { ret += l; }
    }

    std::shared_ptr<RShape> seg = getSegmentAt(segIdx);
    bool lim = limited;
    if (segIdx != 0 && segIdx != countSegments() - 1) { lim = true; }
    RVector p2 = seg->getClosestPointOnShape(p, lim);
    seg->trimEndPoint(p2);
    ret += seg->getLength();

    return ret;
}

double RPolyline::getSegmentsLength(int fromIndex, int toIndex) const
{
    double len = 0.0;
    for (int i = fromIndex; i < toIndex; i++)
    {
        std::shared_ptr<RShape> segment = getSegmentAt(i);
        len += segment->getLength();
    }
    return len;
}

double RPolyline::getDistanceFromStart(const RVector &p) const { return 0.0; }

std::vector<double> RPolyline::getDistancesFromStart(const RVector &p) const
{
    std::vector<double> ret;

    // any segment might contain point (self intersection):
    double len = 0.0;
    for (int i = 0; i < countSegments(); i++)
    {
        std::shared_ptr<RShape> segment = getSegmentAt(i);
        if (segment->getDistanceTo(p) < 0.0001)
        {
            ret.push_back(len + segment->getDistanceFromStart(p));
        }
        len += segment->getLength();
    }

    // point is not on polyline, return distance to point closest to position:
    if (ret.empty()) { ret.push_back(getLengthTo(p, true)); }

    return ret;
}

std::vector<RVector> RPolyline::getEndPoints() const { return m_vertices; }

std::vector<RVector> RPolyline::getMiddlePoints() const
{
    std::vector<RVector> ret;

    std::vector<std::shared_ptr<RShape>> sub = getExploded();
    std::vector<std::shared_ptr<RShape>>::iterator it;
    for (it = sub.begin(); it != sub.end(); ++it)
    {
        auto pps = (*it)->getMiddlePoints();
        ret.insert(ret.end(), pps.begin(), pps.end());
    }

    return ret;
}

std::vector<RVector> RPolyline::getCenterPoints() const
{
    std::vector<RVector> ret;

    std::vector<std::shared_ptr<RShape>> sub = getExploded();
    std::vector<std::shared_ptr<RShape>>::iterator it;
    for (it = sub.begin(); it != sub.end(); ++it)
    {
        auto pps = (*it)->getCenterPoints();
        ret.insert(ret.end(), pps.begin(), pps.end());
    }

    return ret;
}

RVector RPolyline::getPointAtPercent(double p) const
{
    double length = getLength();
    double distance = p * length;
    std::vector<RVector> candidates = getPointsWithDistanceToEnd(
            distance, RS::FromStart | RS::AlongPolyline);
    if (candidates.size() != 1) { return RVector::invalid; }
    return candidates.at(0);
}

std::vector<RVector> RPolyline::getPointsWithDistanceToEnd(double distance,
                                                           int from) const
{
    std::vector<RVector> ret;

    std::vector<std::shared_ptr<RShape>> sub = getExploded();

    if (sub.empty()) { return ret; }

    if (from & RS::AlongPolyline)
    {
        double remainingDist;
        double len;

        if (from & RS::FromStart)
        {
            if (distance < 0.0)
            {
                // extend at start:
                auto pps = sub.front()->getPointsWithDistanceToEnd(
                        distance, RS::FromStart);
                ret.insert(ret.end(), pps.begin(), pps.end());
            }
            else
            {
                remainingDist = distance;
                for (int i = 0; i < sub.size(); i++)
                {
                    len = sub[i]->getLength();
                    if (remainingDist > len) { remainingDist -= len; }
                    else
                    {
                        auto pps = sub[i]->getPointsWithDistanceToEnd(
                                remainingDist, RS::FromStart);
                        ret.insert(ret.end(), pps.begin(), pps.end());
                        break;
                    }
                }
            }
        }

        if (from & RS::FromEnd)
        {
            if (distance < 0.0)
            {
                // extend at end:
                auto pps = sub.back()->getPointsWithDistanceToEnd(distance,
                                                                  RS::FromEnd);
                ret.insert(ret.end(), pps.begin(), pps.end());
            }
            else
            {
                remainingDist = distance;
                for (int i = sub.size() - 1; i >= 0; i--)
                {
                    len = sub[i]->getLength();
                    if (remainingDist > len) { remainingDist -= len; }
                    else
                    {
                        auto pps = sub[i]->getPointsWithDistanceToEnd(
                                remainingDist, RS::FromEnd);
                        ret.insert(ret.end(), pps.begin(), pps.end());
                        break;
                    }
                }
            }
        }
    }
    else
    {
        std::vector<std::shared_ptr<RShape>>::iterator it;
        for (it = sub.begin(); it != sub.end(); ++it)
        {
            auto pps = (*it)->getPointsWithDistanceToEnd(distance, from);
            ret.insert(ret.end(), pps.begin(), pps.end());
        }
    }

    return ret;
}

std::vector<RVector> RPolyline::getPointCloud(double segmentLength) const
{
    std::vector<RVector> ret;
    for (int i = 0; i < countSegments(); i++)
    {
        std::shared_ptr<RShape> seg = getSegmentAt(i);
        if (!seg) { continue; }
        auto pps = seg->getPointCloud(segmentLength);
        ret.insert(ret.end(), pps.begin(), pps.end());
    }
    return ret;
}

double RPolyline::getAngleAt(double distance, RS::From from) const
{
    std::vector<std::shared_ptr<RShape>> sub = getExploded();

    if (from & RS::AlongPolyline)
    {
        double remainingDist;
        double len;

        if (from & RS::FromStart)
        {
            remainingDist = distance;
            for (int i = 0; i < sub.size(); i++)
            {
                len = sub[i]->getLength();
                if (remainingDist > len) { remainingDist -= len; }
                else
                {
                    return sub[i]->getAngleAt(remainingDist, RS::FromStart);
                }
            }
        }

        if (from & RS::FromEnd)
        {
            remainingDist = distance;
            for (int i = sub.size() - 1; i >= 0; i--)
            {
                len = sub[i]->getLength();
                if (remainingDist > len) { remainingDist -= len; }
                else { return sub[i]->getAngleAt(remainingDist, RS::FromEnd); }
            }
        }
    }
    // else {
    //  not implemented / never used
    //    assert(false);
    //}

    return RNANDOUBLE;
}

RVector RPolyline::getVectorTo(const RVector &point, bool limited,
                               double strictRange) const
{
    RVector ret = RVector::invalid;

    std::vector<std::shared_ptr<RShape>> sub = getExploded();
    for (int i = 0; i < sub.size(); i++)
    {
        std::shared_ptr<RShape> shape = sub.at(i);
        bool lim = limited;
        if (i != 0 && i != sub.size() - 1)
        {
            // segments in the middle: always limited:
            lim = true;
        }
        RVector v = shape->getVectorTo(point, lim, strictRange);
        if (v.isValid() &&
            (!ret.isValid() || v.getMagnitude() < ret.getMagnitude()))
        {
            ret = v;
        }
    }

    return ret;
}

double RPolyline::getDistanceTo(const RVector &point, bool limited,
                                double strictRange) const
{
    if (!hasWidths())
    {
        return RShape::getDistanceTo(point, limited, strictRange);
    }

    if (!getBoundingBox().grow(strictRange).contains(point))
    {
        return RNANDOUBLE;
    }

    double ret = RNANDOUBLE;

    std::vector<RPolyline> outline = getOutline();
    for (int i = 0; i < outline.size(); i++)
    {
        assert(!outline[i].hasWidths());
        double d = outline[i].getDistanceTo(point);
        if (RMath::isNaN(ret) || d < ret) { ret = d; }

        if (outline[i].isGeometricallyClosed())
        {
            if (outline[i].contains(point))
            {
                if (RMath::isNaN(ret) || strictRange < ret)
                {
                    ret = strictRange;
                }
            }
        }
    }

    return ret;
}

int RPolyline::getClosestSegment(const RVector &point) const
{
    int ret = -1;
    double minDist = -1;

    for (int i = 0; i < countSegments(); i++)
    {
        std::shared_ptr<RShape> segment = getSegmentAt(i);
        if (!segment) { break; }
        double dist = segment->getDistanceTo(point, true);
        if (!RMath::isNormal(dist)) { continue; }
        if (minDist < 0 || dist < minDist)
        {
            minDist = dist;
            ret = i;
        }
    }

    return ret;
}

int RPolyline::getClosestVertex(const RVector &point) const
{
    return point.getClosestIndex(getVertices());
}

bool RPolyline::move(const RVector &offset)
{
    for (int i = 0; i < m_vertices.size(); i++) { m_vertices[i].move(offset); }
    return true;
}

bool RPolyline::rotate(double rotation, const RVector &center)
{
    if (fabs(rotation) < RS::AngleTolerance) { return false; }
    for (int i = 0; i < m_vertices.size(); i++)
    {
        m_vertices[i].rotate(rotation, center);
    }
    return true;
}

bool RPolyline::scale(double scaleFactor, const RVector &center)
{
    return RShape::scale(scaleFactor, center);
}

bool RPolyline::scale(const RVector &scaleFactors, const RVector &center)
{
    if (hasArcSegments() &&
        !RMath::fuzzyCompare(scaleFactors.x, scaleFactors.y))
    {
        // non-uniform scaling of polyline with arcs:
        RPolyline pl;
        for (int i = 0; i < countSegments(); i++)
        {
            std::shared_ptr<RShape> seg = getSegmentAt(i);
            if (!seg) { continue; }

            std::shared_ptr<RShape> newSeg;
            if (seg->getShapeType() == RS::Line)
            {
                newSeg = seg;
                newSeg->scale(scaleFactors, center);
            }
            else
            {
                newSeg = RShapePrivate::scaleArc(*seg, scaleFactors, center);
            }

            if (newSeg) { pl.appendShape(*newSeg); }
        }

        *this = pl;
        return true;
    }

    for (int i = 0; i < m_vertices.size(); i++)
    {
        m_vertices[i].scale(scaleFactors, center);
    }
    for (int i = 0; i < m_startWidths.size(); i++)
    {
        if (m_startWidths[i] > 0.0)
        {
            m_startWidths[i] *= fabs(scaleFactors.x);
        }
    }
    for (int i = 0; i < m_endWidths.size(); i++)
    {
        if (m_endWidths[i] > 0.0) { m_endWidths[i] *= fabs(scaleFactors.x); }
    }
    // factor in x or in y is negative -> mirror:
    if ((scaleFactors.x < 0) != (scaleFactors.y < 0))
    {
        for (int i = 0; i < m_bulges.size(); i++) { m_bulges[i] *= -1; }
    }
    return true;
}

bool RPolyline::mirror(const RLine &axis)
{
    int i;
    for (i = 0; i < m_vertices.size(); i++)
    {
        m_vertices[i].mirror(axis.getStartPoint(), axis.getEndPoint());
    }
    for (i = 0; i < m_bulges.size(); i++) { m_bulges[i] *= -1; }
    return true;
}

bool RPolyline::reverse()
{
    std::vector<RVector> vs = m_vertices;
    if (m_closed) { vs.push_back(vs.front()); }

    RPolyline nPolyline;

    for (int i = vs.size() - 1, k = 0; i >= 0; i--, k++)
    {
        nPolyline.appendVertex(vs[i]);
        if (i > 0)
        {
            nPolyline.setBulgeAt(k, -m_bulges[i - 1]);

            nPolyline.setStartWidthAt(k, m_endWidths[i - 1]);
            nPolyline.setEndWidthAt(k, m_startWidths[i - 1]);
        }
    }
    if (m_closed) { nPolyline.convertToClosed(); }

    *this = nPolyline;

    assert(m_vertices.size() == m_bulges.size());
    assert(m_vertices.size() == m_startWidths.size());
    assert(m_vertices.size() == m_endWidths.size());

    return true;
}

RPolyline RPolyline::getReversed() const
{
    RPolyline ret = *this;
    ret.reverse();
    return ret;
}

bool RPolyline::stretch(const RPolyline &area, const RVector &offset)
{
    for (int i = 0; i < m_vertices.size(); i++)
    {
        m_vertices[i].stretch(area, offset);
    }
    return true;
}

RS::Ending RPolyline::getTrimEnd(const RVector &trimPoint,
                                 const RVector &clickPoint)
{
    return RS::EndingNone;
}

bool RPolyline::trimStartPoint(const RVector &trimPoint,
                               const RVector &clickPoint, bool extend)
{
    return false;
}

bool RPolyline::trimEndPoint(const RVector &trimPoint,
                             const RVector &clickPoint, bool extend)
{
    return false;
}

bool RPolyline::trimStartPoint(double trimDist) { return false; }

bool RPolyline::trimEndPoint(double trimDist) { return false; }

bool RPolyline::simplify(double tolerance) { return false; }

std::vector<RVector> RPolyline::verifyTangency(double toleranceMin,
                                               double toleranceMax)
{
    return std::vector<RVector>();
}

RPolyline RPolyline::modifyPolylineCorner(const RShape &trimmedShape1,
                                          RS::Ending ending1, int segmentIndex1,
                                          const RShape &trimmedShape2,
                                          RS::Ending ending2, int segmentIndex2,
                                          const RShape *cornerShape) const
{
    std::shared_ptr<RShape> segment;

    RPolyline pl;

    if (segmentIndex1 < segmentIndex2 && ending1 == RS::EndingEnd &&
        ending2 == RS::EndingStart)
    {
        for (int i = 0; i < segmentIndex1; i++)
        {
            segment = getSegmentAt(i);
            pl.appendShape(*segment);
            pl.setStartWidthAt(pl.m_startWidths.size() - 2, getStartWidthAt(i));
            pl.setEndWidthAt(pl.m_endWidths.size() - 2, getEndWidthAt(i));
        }

        pl.appendShapeAuto(trimmedShape1);
        if (cornerShape != NULL) { pl.appendShapeAuto(*cornerShape); }
        pl.appendShapeAuto(trimmedShape2);

        for (int i = segmentIndex2 + 1; i < countSegments(); i++)
        {
            segment = getSegmentAt(i);
            pl.appendShape(*segment);
            pl.setStartWidthAt(pl.m_startWidths.size() - 2, getStartWidthAt(i));
            pl.setEndWidthAt(pl.m_endWidths.size() - 2, getEndWidthAt(i));
        }
    }
    else if (segmentIndex1 > segmentIndex2 && ending1 == RS::EndingStart &&
             ending2 == RS::EndingEnd)
    {
        for (int i = 0; i < segmentIndex2; i++)
        {
            segment = getSegmentAt(i);
            pl.appendShape(*segment);
            pl.setStartWidthAt(pl.m_startWidths.size() - 2, getStartWidthAt(i));
            pl.setEndWidthAt(pl.m_endWidths.size() - 2, getEndWidthAt(i));
        }

        pl.appendShapeAuto(trimmedShape2);
        if (cornerShape != NULL) { pl.appendShapeAuto(*cornerShape); }
        pl.appendShapeAuto(trimmedShape1);

        for (int i = segmentIndex1 + 1; i < countSegments(); i++)
        {
            segment = getSegmentAt(i);
            pl.appendShape(*segment);
            pl.setStartWidthAt(pl.m_startWidths.size() - 2, getStartWidthAt(i));
            pl.setEndWidthAt(pl.m_endWidths.size() - 2, getEndWidthAt(i));
        }
    }
    else if (segmentIndex1 < segmentIndex2 && ending1 == RS::EndingStart &&
             ending2 == RS::EndingEnd)
    {
        pl.appendShapeAuto(trimmedShape1);
        for (int i = segmentIndex1 + 1; i < segmentIndex2; i++)
        {
            segment = getSegmentAt(i);
            pl.appendShape(*segment);
            pl.setStartWidthAt(pl.m_startWidths.size() - 2, getStartWidthAt(i));
            pl.setEndWidthAt(pl.m_endWidths.size() - 2, getEndWidthAt(i));
        }
        pl.appendShapeAuto(trimmedShape2);
        if (cornerShape != NULL) { pl.appendShapeAuto(*cornerShape); }
    }
    else if (segmentIndex1 > segmentIndex2 && ending1 == RS::EndingEnd &&
             ending2 == RS::EndingStart)
    {
        pl.appendShapeAuto(trimmedShape2);
        for (int i = segmentIndex2 + 1; i < segmentIndex1; i++)
        {
            segment = getSegmentAt(i);
            pl.appendShape(*segment);
            pl.setStartWidthAt(pl.m_startWidths.size() - 2, getStartWidthAt(i));
            pl.setEndWidthAt(pl.m_endWidths.size() - 2, getEndWidthAt(i));
        }
        pl.appendShapeAuto(trimmedShape1);
        if (cornerShape != NULL) { pl.appendShapeAuto(*cornerShape); }
    }

    return pl;
}

bool RPolyline::isConcave() const { return !getConcaveVertices().empty(); }

std::vector<RVector> RPolyline::getConvexVertices(bool convex) const
{
    if (!isGeometricallyClosed()) { return std::vector<RVector>(); }

    RPolyline pl = *this;
    pl.autoClose();

    RS::Orientation ori = pl.getOrientation();

    std::vector<RVector> ret;

    for (int i = 0; i < pl.m_vertices.size(); i++)
    {
        int iPrev = RMath::absmod(i - 1, pl.m_vertices.size());
        std::shared_ptr<RShape> segmentPrev = pl.getSegmentAt(iPrev);
        std::shared_ptr<RShape> segmentNext = pl.getSegmentAt(i);

        double aPrev = segmentPrev->getDirection2() + M_PI;
        double aNext = segmentNext->getDirection1();

        RVector pPrev = RVector::createPolar(1.0, aPrev);
        RVector pNext = RVector::createPolar(1.0, aNext);

        double cp = RVector::getCrossProduct(pPrev, pNext);

        if (convex)
        {
            if (ori == RS::CW && cp < 0.0 || ori == RS::CCW && cp > 0.0)
            {
                ret.push_back(pl.m_vertices[i]);
            }
        }
        else
        {
            if (ori == RS::CCW && cp < 0.0 || ori == RS::CW && cp > 0.0)
            {
                ret.push_back(pl.m_vertices[i]);
            }
        }
    }

    return ret;
}

std::vector<RVector> RPolyline::getConcaveVertices() const
{
    return getConvexVertices(false);
}

RVector RPolyline::getCentroid() const
{
    if (hasArcSegments()) { return RVector::invalid; }

    double xSum = 0;
    double ySum = 0;
    double signedArea = 0;
    int n = m_vertices.size();

    for (int i = 0; i < n; i++)
    {
        double x0 = m_vertices[i].x;
        double y0 = m_vertices[i].y;
        double x1 = m_vertices[(i + 1) % n].x;
        double y1 = m_vertices[(i + 1) % n].y;

        // calculate the cross product of the edges
        double crossProduct = x0 * y1 - x1 * y0;
        signedArea += crossProduct;
        xSum += (x0 + x1) * crossProduct;
        ySum += (y0 + y1) * crossProduct;
    }

    signedArea *= 0.5;
    double centroidX = xSum / (6.0 * signedArea);
    double centroidY = ySum / (6.0 * signedArea);

    return RVector(centroidX, centroidY);
}

std::vector<RPolyline> RPolyline::splitAtDiscontinuities(double tolerance) const
{
    return {*this};
}

std::vector<RPolyline> RPolyline::splitAtSegmentTypeChange() const
{
    return {*this};
}

double RPolyline::getBaseAngle() const { return 0.0; }

double RPolyline::getWidth() const { return 0.0; }

bool RPolyline::setWidth(double v) { return false; }

double RPolyline::getHeight() const { return 0.0; }

bool RPolyline::setHeight(double v) { return false; }

RPolyline RPolyline::roundAllCorners(double radius) const { return *this; }

RPolyline RPolyline::getPolygon(double segmentLength) const { return *this; }

RPolyline RPolyline::getPolygonHull(double angle, double tolerance,
                                    bool inner) const
{
    return *this;
}
