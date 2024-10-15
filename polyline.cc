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

using namespace cada;

/**
 * Creates a polyline object without points.
 */
Polyline::Polyline() : closed(false)
{
}

/**
 * Creates a polyline object with the given points.
 */
Polyline::Polyline(const std::vector<Vec3d> &vertices, bool closed)
    : closed(closed)
{

    setVertices(vertices);
}

/**
 * Creates a polyline from segments (lines or arcs).
 */
Polyline::Polyline(const std::vector<std::shared_ptr<Shape>> &segments)
    : closed(false)
{

    std::vector<std::shared_ptr<Shape>>::const_iterator it;
    for (it = segments.begin(); it != segments.end(); ++it) {
        std::shared_ptr<Shape> segment = *it;

        if (segment->isDirected()) {
            if (vertices.size() == 0) {
                appendVertex(segment->getStartPoint(), 0.0);
            }
            appendVertex(segment->getEndPoint(), 0.0);
        }

        std::shared_ptr<Arc> arc = segment.dynamicCast<Arc>();
        if (!arc.isNull()) {
            if (bulges.size() > 1) {
                bulges[bulges.size() - 2] = arc->getBulge();
            }
        }
    }

    Q_ASSERT(vertices.size() == bulges.size());
    Q_ASSERT(vertices.size() == startWidths.size());
    Q_ASSERT(vertices.size() == endWidths.size());

    autoClose();
}

Polyline::~Polyline()
{
}

bool Polyline::isFlat() const
{
    double z = std::numeric_limits<double>::quiet_NaN();
    for (int i = 0; i < vertices.size(); i++) {
        if (i == 0) {
            z = vertices[i].z;
            continue;
        }

        if (!Math::fuzzyCompare(z, vertices[i].z)) {
            return false;
        }
    }

    return true;
}

void Polyline::clear()
{
    vertices.clear();
    bulges.clear();
    startWidths.clear();
    endWidths.clear();

    Q_ASSERT(vertices.size() == bulges.size());
    Q_ASSERT(vertices.size() == startWidths.size());
    Q_ASSERT(vertices.size() == endWidths.size());
}

/**
 * Removes duplicate vertices.
 */
void Polyline::normalize(double tolerance)
{
    std::vector<Vec3d> newVertices;
    std::vector<double> newBulges;
    std::vector<double> newStartWidths;
    std::vector<double> newEndWidths;

    Vec3d vPrev;
    int newIndex = 0;

    for (int i = 0; i < vertices.size(); i++) {
        Vec3d v = vertices[i];
        double b = bulges[i];
        double s = startWidths[i];
        double e = endWidths[i];

        if (i == 0 || !v.equalsFuzzy(vPrev, tolerance)) {
            newVertices.push_back(v);
            newBulges.push_back(b);
            newStartWidths.push_back(s);
            newEndWidths.push_back(e);
            newIndex = newIndex + 1;
            vPrev = v;
        }
        else if (i > 0) {
            newBulges[newIndex - 1] = b;
            newStartWidths[newIndex - 1] = s;
            newEndWidths[newIndex - 1] = e;
        }
    }

    // remove duplicate last vertex of closed polyline:
    if (closed) {
        if (newVertices.first().equalsFuzzy(newVertices.last(), tolerance)) {
            newVertices.removeLast();
            newBulges.removeLast();
            newStartWidths.removeLast();
            newEndWidths.removeLast();
        }
    }

    vertices = newVertices;
    bulges = newBulges;
    startWidths = newStartWidths;
    endWidths = newEndWidths;

    Q_ASSERT(vertices.size() == bulges.size());
    Q_ASSERT(vertices.size() == startWidths.size());
    Q_ASSERT(vertices.size() == endWidths.size());
}

bool Polyline::prependShape(const Shape &shape)
{
    return appendShape(shape, true);
}

bool Polyline::appendShape(const Shape &shape, bool prepend)
{
    bool ret = true;

    // append spline as polyline approximation:
    if (shape.getShapeType() == Shape::Spline) {
        const BSpline *spl = dynamic_cast<const BSpline *>(&shape);
        if (spl != NULL) {
            double tol =
                RSettings::getDoubleValue("Explode/SplineTolerance", 0.01);
            Polyline pl = spl->approximateWithArcs(tol);
            return appendShape(pl, prepend);
        }
    }

    // append ellipse as polyline approximation:
    else if (shape.getShapeType() == Shape::Ellipse) {
        const Ellipse *elp = dynamic_cast<const Ellipse *>(&shape);
        if (elp != NULL) {
            double seg =
                RSettings::getDoubleValue("Explode/EllipseSegments", 32);
            Polyline pl = elp->approximateWithArcs(seg);
            return appendShape(pl, prepend);
        }
    }

    // append circle as polyline to empty polyline:
    else if (shape.getShapeType() == Shape::Circle && isEmpty()) {
        const Circle *circle = dynamic_cast<const Circle *>(&shape);
        if (circle != NULL) {
            appendShape(Arc(circle->getCenter(), circle->getRadius(), 0.0, M_PI,
                            false));
            appendShape(Arc(circle->getCenter(), circle->getRadius(), M_PI,
                            2 * M_PI, false));
            return true;
        }
    }

    // append full circle arc as circle (two arc segments) to empty polyline:
    else if (shape.getShapeType() == Shape::Arc) {
        const Arc *arc = dynamic_cast<const Arc *>(&shape);
        if (arc != NULL && arc->isFullCircle()) {
            appendShape(Circle(arc->getCenter(), arc->getRadius()));
            return true;
        }
    }

    // append polyline:
    else if (shape.getShapeType() == Shape::Polyline) {
        const Polyline *pl = dynamic_cast<const Polyline *>(&shape);
        if (pl != NULL) {
            if (prepend) {
                for (int i = pl->countSegments() - 1; i >= 0; --i) {
                    std::shared_ptr<Shape> s = pl->getSegmentAt(i);
                    if (s.isNull()) {
                        continue;
                    }
                    ret = prependShape(*s) && ret;
                    setStartWidthAt(0, pl->getStartWidthAt(i));
                    setEndWidthAt(0, pl->getEndWidthAt(i));
                }
            }
            else {
                for (int i = 0; i < pl->countSegments(); ++i) {
                    std::shared_ptr<Shape> s = pl->getSegmentAt(i);
                    if (s.isNull()) {
                        continue;
                    }
                    setStartWidthAt(vertices.size() - 1,
                                    pl->getStartWidthAt(i));
                    setEndWidthAt(vertices.size() - 1, pl->getEndWidthAt(i));
                    ret = appendShape(*s) && ret;
                }
            }
            return ret;
        }
    }

    double bulge = 0.0;

    const Arc *arc = dynamic_cast<const Arc *>(&shape);
    if (arc != NULL) {
        bulge = arc->getBulge();
    }

    if (!shape.isDirected()) {
        qWarning()
            << "Polyline::appendShape: shape is not a line, arc or polyline: "
            << shape;
        return false;
    }

    Vec3d connectionPoint;
    Vec3d nextPoint;
    double gap;
    if (prepend) {
        // prepend:
        connectionPoint = shape.getEndPoint();
        nextPoint = shape.getStartPoint();
        if (vertices.size() == 0) {
            // first point:
            appendVertex(connectionPoint);
        }
        gap = vertices.first().getDistanceTo(connectionPoint);
    }
    else {
        // append:
        connectionPoint = shape.getStartPoint();
        nextPoint = shape.getEndPoint();
        if (vertices.size() == 0) {
            // first point:
            appendVertex(connectionPoint);
        }
        gap = vertices.last().getDistanceTo(connectionPoint);
    }

    if (!Math::fuzzyCompare(gap, 0.0, 1.0e-3)) {
        qWarning() << "Polyline::appendShape: "
                   << "arc or line not connected to polyline at "
                   << connectionPoint << ":"
                   << "\nshape:" << shape << "\ngap: " << gap;
        ret = false;
    }

    if (prepend) {
        prependVertex(nextPoint);
        setBulgeAt(0, bulge);
    }
    else {
        appendVertex(nextPoint);
        setBulgeAt(bulges.size() - 2, bulge);
    }

    Q_ASSERT(vertices.size() == bulges.size());
    Q_ASSERT(vertices.size() == startWidths.size());
    Q_ASSERT(vertices.size() == endWidths.size());

    return ret;
}

/**
 * Appends the given shape to this polyline. The shape is reversed if necessary.
 */
bool Polyline::appendShapeAuto(const Shape &shape)
{
    if (!shape.isDirected()) {
        return false;
    }

    if (countVertices() > 0 && getEndPoint().equalsFuzzy(shape.getEndPoint())) {
        std::shared_ptr<Shape> rev = std::shared_ptr<Shape>(shape.clone());
        rev->reverse();
        return appendShape(*rev);
    }

    return appendShape(shape);
}

bool Polyline::appendShapeTrim(const Shape &shape)
{
    if (!shape.isDirected()) {
        return false;
    }

    if (countVertices() > 0) {
        if (getEndPoint().equalsFuzzy(shape.getStartPoint())) {
            return appendShape(shape);
        }
        if (getEndPoint().equalsFuzzy(shape.getEndPoint())) {
            std::shared_ptr<Shape> rev = std::shared_ptr<Shape>(shape.clone());
            rev->reverse();
            return appendShape(*rev);
        }

        if (shape.getShapeType() == Shape::Line) {
            std::shared_ptr<Shape> lastSegment = getLastSegment();
            std::vector<Vec3d> ips =
                lastSegment->getIntersectionPoints(shape, false);
            if (ips.size() == 1) {
                Vec3d ip = ips[0];
                moveEndPoint(ip);
                std::shared_ptr<Shape> trimmed =
                    std::shared_ptr<Shape>(shape.clone());
                trimmed->trimStartPoint(ip);
                return appendShape(*trimmed);
            }
        }
    }

    return appendShape(shape);
}

bool Polyline::closeTrim()
{
    if (isGeometricallyClosed()) {
        return true;
    }

    if (countSegments() > 1) {
        std::shared_ptr<Shape> firstSegment = getFirstSegment();
        std::shared_ptr<Shape> lastSegment = getLastSegment();

        if (firstSegment.isNull() || lastSegment.isNull()) {
            return false;
        }

        if (firstSegment->getShapeType() == Shape::Line &&
            lastSegment->getShapeType() == Shape::Line) {
            std::vector<Vec3d> ips =
                lastSegment->getIntersectionPoints(*firstSegment, false);
            if (ips.size() == 1) {
                Vec3d ip = ips[0];
                moveStartPoint(ip);
                moveEndPoint(ip);
                return true;
            }
        }
    }

    return false;
}

void Polyline::appendVertex(const Vec3d &vertex, double bulge, double w1,
                            double w2)
{
    vertices.push_back(vertex);
    bulges.push_back(bulge);
    startWidths.push_back(w1);
    endWidths.push_back(w2);

    Q_ASSERT(vertices.size() == bulges.size());
    Q_ASSERT(vertices.size() == startWidths.size());
    Q_ASSERT(vertices.size() == endWidths.size());
}

void Polyline::appendVertex(double x, double y, double bulge, double w1,
                            double w2)
{
    appendVertex(Vec3d(x, y), bulge, w1, w2);
}

void Polyline::prependVertex(const Vec3d &vertex, double bulge, double w1,
                             double w2)
{
    vertices.prepend(vertex);
    bulges.prepend(bulge);
    startWidths.prepend(w1);
    endWidths.prepend(w2);

    Q_ASSERT(vertices.size() == bulges.size());
    Q_ASSERT(vertices.size() == startWidths.size());
    Q_ASSERT(vertices.size() == endWidths.size());
}

void Polyline::insertVertex(int index, const Vec3d &vertex, double bulgeBefore,
                            double bulgeAfter)
{
    vertices.insert(index, vertex);
    if (index > 0) {
        bulges[index - 1] = bulgeBefore;
    }
    bulges.insert(index, bulgeAfter);
    startWidths.insert(index, 0.0);
    endWidths.insert(index, 0.0);

    Q_ASSERT(vertices.size() == bulges.size());
    Q_ASSERT(vertices.size() == startWidths.size());
    Q_ASSERT(vertices.size() == endWidths.size());
}

/**
 * Inserts a vertex at the point on the polyline closest to the given position.
 */
void Polyline::insertVertexAt(const Vec3d &point)
{
    int index = getClosestSegment(point);
    if (index < 0) {
        return;
    }

    std::shared_ptr<Shape> seg1 = getSegmentAt(index);
    if (seg1.isNull()) {
        return;
    }

    Vec3d p = seg1->getClosestPointOnShape(point, false);

    std::shared_ptr<Shape> seg2 = std::shared_ptr<Shape>(seg1->clone());

    if (!seg1->isDirected() || !seg2->isDirected()) {
        return;
    }

    seg1->trimEndPoint(p);
    seg2->trimStartPoint(p);

    insertVertex(index + 1, p);

    std::shared_ptr<Arc> arc1 = seg1.dynamicCast<Arc>();
    std::shared_ptr<Arc> arc2 = seg2.dynamicCast<Arc>();
    if (arc1.isNull()) {
        setBulgeAt(index, 0.0);
    }
    else {
        setBulgeAt(index, arc1->getBulge());
    }

    if (arc2.isNull()) {
        setBulgeAt(index + 1, 0.0);
    }
    else {
        setBulgeAt(index + 1, arc2->getBulge());
    }

    Q_ASSERT(vertices.size() == bulges.size());
    Q_ASSERT(vertices.size() == startWidths.size());
    Q_ASSERT(vertices.size() == endWidths.size());
}

Vec3d Polyline::insertVertexAtDistance(double dist)
{
    if (polylineProxy != NULL) {
        return polylineProxy->insertVertexAtDistance(*this, dist);
    }
    return Vec3d::invalid;
}

void Polyline::removeFirstVertex()
{
    if (vertices.isEmpty()) {
        return;
    }

    vertices.removeFirst();
    bulges.removeFirst();
    startWidths.removeFirst();
    endWidths.removeFirst();

    Q_ASSERT(vertices.size() == bulges.size());
    Q_ASSERT(vertices.size() == startWidths.size());
    Q_ASSERT(vertices.size() == endWidths.size());
}

void Polyline::removeLastVertex()
{
    if (vertices.isEmpty()) {
        return;
    }

    vertices.removeLast();
    bulges.removeLast();
    startWidths.removeLast();
    endWidths.removeLast();

    Q_ASSERT(vertices.size() == bulges.size());
    Q_ASSERT(vertices.size() == startWidths.size());
    Q_ASSERT(vertices.size() == endWidths.size());
}

void Polyline::removeVertex(int index)
{
    vertices.removeAt(index);
    bulges.removeAt(index);
    startWidths.removeAt(index);
    endWidths.removeAt(index);

    Q_ASSERT(vertices.size() == bulges.size());
    Q_ASSERT(vertices.size() == startWidths.size());
    Q_ASSERT(vertices.size() == endWidths.size());
}

void Polyline::removeVerticesAfter(int index)
{
    vertices = vertices.mid(0, index + 1);
    bulges = bulges.mid(0, index + 1);
    startWidths = startWidths.mid(0, index + 1);
    endWidths = endWidths.mid(0, index + 1);

    Q_ASSERT(vertices.size() == bulges.size());
    Q_ASSERT(vertices.size() == startWidths.size());
    Q_ASSERT(vertices.size() == endWidths.size());
}

void Polyline::removeVerticesBefore(int index)
{
    vertices = vertices.mid(index);
    bulges = bulges.mid(index);
    startWidths = startWidths.mid(index);
    endWidths = endWidths.mid(index);

    Q_ASSERT(vertices.size() == bulges.size());
    Q_ASSERT(vertices.size() == startWidths.size());
    Q_ASSERT(vertices.size() == endWidths.size());
}

void Polyline::setVertices(const std::vector<Vec3d> &vertices)
{
    this->vertices = vertices;

    bulges.clear();
    startWidths.clear();
    endWidths.clear();
    for (int i = 0; i < vertices.size(); ++i) {
        bulges.push_back(0.0);
        startWidths.push_back(0.0);
        endWidths.push_back(0.0);
    }

    Q_ASSERT(vertices.size() == bulges.size());
    Q_ASSERT(vertices.size() == startWidths.size());
    Q_ASSERT(vertices.size() == endWidths.size());
}

std::vector<Vec3d> Polyline::getVertices() const
{
    return vertices;
}

Vec3d Polyline::getVertexAt(int i) const
{
    if (i < 0 || i >= vertices.size()) {
        Q_ASSERT(false);
        return Vec3d::invalid;
    }

    return vertices.at(i);
}

int Polyline::getVertexIndex(const Vec3d &v, double tolerance) const
{
    for (int i = 0; i < vertices.size(); i++) {
        if (vertices[i].equalsFuzzy(v, tolerance)) {
            return i;
        }

        if (vertices[i].equalsFuzzy(v, 0.01)) {
            qDebug() << "almost match: " << vertices[i].getDistanceTo(v);
        }
    }

    return -1;
}

Vec3d Polyline::getLastVertex() const
{
    if (vertices.size() == 0) {
        return Vec3d::invalid;
    }

    return vertices.at(vertices.size() - 1);
}

void Polyline::setVertexAt(int i, const Vec3d &v)
{
    if (i < 0 || i >= vertices.size()) {
        Q_ASSERT(false);
        return;
    }

    vertices[i] = v;
}

void Polyline::moveVertexAt(int i, const Vec3d &offset)
{
    if (i < 0 || i >= vertices.size()) {
        Q_ASSERT(false);
        return;
    }

    vertices[i] += offset;
}

int Polyline::countVertices() const
{
    return vertices.size();
}

void Polyline::setBulges(const std::vector<double> &b)
{
    bulges = b;
}

std::vector<double> Polyline::getBulges() const
{
    return bulges;
}

double Polyline::getBulgeAt(int i) const
{
    if (i < 0 || i >= bulges.size()) {
        return std::numeric_limits<double>::quiet_NaN();
    }

    return bulges.at(i);
}

void Polyline::setBulgeAt(int i, double b)
{
    if (i < 0 || i >= bulges.size()) {
        return;
    }

    bulges[i] = b;
}

bool Polyline::hasArcSegments() const
{
    for (int i = 0; i < bulges.size(); i++) {
        if (!isStraight(bulges[i])) {
            return true;
        }
    }

    return false;
}

std::vector<double> Polyline::getVertexAngles() const
{
    NS::Orientation orientation = getOrientation(true);
    std::vector<double> ret;
    for (int i = 0; i < countVertices(); i++) {
        ret.push_back(getVertexAngle(i, orientation));
    }
    return ret;
}

double Polyline::getVertexAngle(int i, NS::Orientation orientation) const
{
    if (!isGeometricallyClosed() && (i == 0 || i == countVertices() - 1)) {
        // angles at first / last vertex for open polyline:
        return 0.0;
    }

    if (countSegments() == 0) {
        return 0.0;
    }

    std::shared_ptr<Shape> prevSegment =
        getSegmentAt(Math::absmod(i - 1, countSegments()));
    std::shared_ptr<Shape> nextSegment = getSegmentAt(i % countSegments());

    // angle from vertex to next segment:
    double aNext = nextSegment->getDirection1();
    // angle from vertex to previous segment:
    double aPrev = prevSegment->getDirection2();

    if (orientation == NS::UnknownOrientation) {
        orientation = getOrientation(true);
    }
    if (orientation == NS::CW) {
        return Math::getAngleDifference(aPrev, aNext);
    }
    else {
        return Math::getAngleDifference(aNext, aPrev);
    }
}

void Polyline::setGlobalWidth(double w)
{
    for (int i = 0; i < startWidths.size(); i++) {
        startWidths[i] = w;
    }
    for (int i = 0; i < endWidths.size(); i++) {
        endWidths[i] = w;
    }
}

void Polyline::setStartWidthAt(int i, double w)
{
    if (i < 0 || i >= startWidths.size()) {
        return;
    }
    startWidths[i] = w;
}

double Polyline::getStartWidthAt(int i) const
{
    if (i < 0 || i >= startWidths.size()) {
        return -1.0;
    }

    return startWidths.at(i);
}

void Polyline::setEndWidthAt(int i, double w)
{
    if (i < 0 || i >= endWidths.size()) {
        return;
    }
    endWidths[i] = w;
}

double Polyline::getEndWidthAt(int i) const
{
    if (i < 0 || i >= endWidths.size()) {
        return -1.0;
    }

    return endWidths.at(i);
}

bool Polyline::hasWidths() const
{
    for (int i = 0; i < startWidths.size() && i < endWidths.size(); i++) {
        if (!Math::isNaN(startWidths[i]) && startWidths[i] > 0.0) {
            // widths in last vertex only count if closed:
            if (i != startWidths.size() - 1 || isClosed()) {
                return true;
            }
        }
        if (!Math::isNaN(endWidths[i]) && endWidths[i] > 0.0) {
            if (i != startWidths.size() - 1 || isClosed()) {
                return true;
            }
        }
    }

    return false;
}

void Polyline::setStartWidths(const std::vector<double> &sw)
{
    startWidths = sw;
}

std::vector<double> Polyline::getStartWidths() const
{
    return startWidths;
}

void Polyline::setEndWidths(const std::vector<double> &ew)
{
    endWidths = ew;
}

std::vector<double> Polyline::getEndWidths() const
{
    return endWidths;
}

/**
 * Marks the poyline as logically (explicitly) closed.
 * The first and last node are usually not identical. Logically
 * closed polylines have an additional segment from start to end point.
 */
void Polyline::setClosed(bool on)
{
    closed = on;
}

/**
 * \return True if this polyline is logically marked as closed.
 */
bool Polyline::isClosed() const
{
    return closed;
}

/**
 * \return True is this polyline is geometrically closed. If this polyline is
 * logically closed it is implicitly also geometrically closed.
 */
bool Polyline::isGeometricallyClosed(double tolerance) const
{
    return isClosed() ||
           getStartPoint().getDistanceTo(getEndPoint()) < tolerance;
}

/**
 * Converts this geometrically closed polyline (start == end) to a
 * locically closed polyline.
 *
 * \return True on success, false if this polyline is already
 * locically closed or is not geometrically closed.
 */
bool Polyline::toLogicallyClosed(double tolerance)
{
    if (isClosed()) {
        return false;
    }

    if (!isGeometricallyClosed(tolerance)) {
        return false;
    }

    removeLastVertex();
    setClosed(true);

    Q_ASSERT(vertices.size() == bulges.size());
    Q_ASSERT(vertices.size() == startWidths.size());
    Q_ASSERT(vertices.size() == endWidths.size());

    return true;
}

/**
 * Converts this logically closed polyline to a locically open,
 * geometrically closed polyline. An additional node is inserted
 * to make sure start == end.
 *
 * \return True on success, false if this polyline is not
 * locically closed.
 */
bool Polyline::toLogicallyOpen()
{
    if (!isClosed()) {
        return false;
    }

    appendVertex(getEndPoint(), getBulgeAt(vertices.size() - 1));
    setClosed(false);

    Q_ASSERT(vertices.size() == bulges.size());
    Q_ASSERT(vertices.size() == startWidths.size());
    Q_ASSERT(vertices.size() == endWidths.size());

    return true;
}

std::vector<Vec3d> Polyline::getSelfIntersectionPoints(double tolerance) const
{
    std::vector<Vec3d> ret;

    bool cl = isGeometricallyClosed();

    std::vector<std::shared_ptr<Shape>> segments = getExploded();
    for (int i = 0; i < segments.size(); i++) {
        std::shared_ptr<Shape> segment = getSegmentAt(i);

        for (int k = i + 1; k < segments.size(); k++) {
            std::shared_ptr<Shape> otherSegment = getSegmentAt(k);

            std::vector<Vec3d> ips =
                segment->getIntersectionPoints(*otherSegment);
            for (int n = 0; n < ips.size(); n++) {
                Vec3d ip = ips[n];
                if (k == i + 1 &&
                    ip.equalsFuzzy(segment->getEndPoint(), tolerance)) {
                    // ignore intersection at vertex between two consecutive
                    // segments:
                    continue;
                }

                if (cl) {
                    if (i == 0 && k == segments.size() - 1 &&
                        ip.equalsFuzzy(segment->getStartPoint(), tolerance)) {
                        continue;
                    }
                }

                ret.push_back(ip);
            }
        }
    }

    return ret;
}

NS::Orientation Polyline::getOrientation(bool implicitelyClosed) const
{
    if (!implicitelyClosed && !isGeometricallyClosed(0.00001)) {
        return NS::Any;
    }

    if (countSegments() < 1) {
        return NS::Any;
    }

    if (hasArcSegments()) {
        Polyline plSegmented = convertArcToLineSegments(16);
        return plSegmented.getOrientation(implicitelyClosed);
    }

    Vec3d minV = Vec3d::invalid;
    std::shared_ptr<Shape> shapeBefore;
    std::shared_ptr<Shape> shapeAfter;
    std::shared_ptr<Shape> shape;
    std::shared_ptr<Shape> previousShape = getSegmentAt(countSegments() - 1);

    // find minimum vertex (lower left corner):
    std::vector<std::shared_ptr<Shape>> segments = getExploded();
    for (int i = 0; i < segments.size(); i++) {
        shape = getSegmentAt(i);
        if (shape.isNull()) {
            continue;
        }

        if (shape->getLength() < 0.001) {
            continue;
        }

        Vec3d v = shape->getStartPoint();
        if (!minV.isValid() || v.x < minV.x ||
            (v.x == minV.x && v.y < minV.y)) {
            minV = v;
            shapeBefore = previousShape;
            shapeAfter = shape;
        }

        previousShape = shape;
    }

    // TODO: fails for large arc (>180d) at bottom left corner, creating round
    // bottom left shape:
    //    double l;
    //    Vec3d p;
    //    std::vector<Vec3d> list;
    //    std::shared_ptr<Arc> arcBefore = shapeBefore.dynamicCast<Arc>();
    //    if (!arcBefore.isNull()) {
    //        l = arcBefore->getLength();
    //        list = arcBefore->getPointsWithDistanceToEnd(l/10, NS::FromEnd);
    //        if (!list.isEmpty()) {
    //            p = list[0];
    //            shapeBefore = std::shared_ptr<Line>(new Line(p,
    //            arcBefore->getEndPoint()));
    //        }
    //    }

    //    std::shared_ptr<Arc> arcAfter = shapeAfter.dynamicCast<Arc>();
    //    if (!arcAfter.isNull()) {
    //        l = arcAfter->getLength();
    //        list = arcAfter->getPointsWithDistanceToEnd(l/10, NS::FromStart);
    //        if (!list.isEmpty()) {
    //            p = list[0];
    //            shapeAfter = std::shared_ptr<Line>(new
    //            Line(arcAfter->getStartPoint(), p));
    //        }
    //    }

    if (shapeBefore.isNull() || shapeAfter.isNull()) {
        return NS::Any;
    }

    double xa = shapeBefore->getStartPoint().x;
    double ya = shapeBefore->getStartPoint().y;
    double xb = shapeAfter->getStartPoint().x;
    double yb = shapeAfter->getStartPoint().y;
    double xc = shapeAfter->getEndPoint().x;
    double yc = shapeAfter->getEndPoint().y;

    double det = (xb - xa) * (yc - ya) - (xc - xa) * (yb - ya);

    if (det < 0.0) {
        // clockwise:
        return NS::CW;
    }
    else {
        // counter-clockwise:
        return NS::CCW;
    }
}

bool Polyline::setOrientation(NS::Orientation orientation)
{
    if (getOrientation(true) != orientation) {
        return reverse();
    }
    return false;
}

Polyline Polyline::convertArcToLineSegments(int segments) const
{
    Polyline ret;

    std::vector<std::shared_ptr<Shape>> segs = getExploded();
    for (int i = 0; i < segs.size(); i++) {
        std::shared_ptr<Shape> seg = segs[i];
        if (seg->getShapeType() == Shape::Arc) {
            std::shared_ptr<Arc> arc = seg.dynamicCast<Arc>();
            Polyline pl =
                arc->approximateWithLinesTan(arc->getLength() / segments);
            ret.appendShape(pl);
        }
        else {
            ret.appendShape(*seg);
        }
    }

    ret.autoClose();
    return ret;
}

Polyline Polyline::convertArcToLineSegmentsLength(double segmentLength) const
{
    Polyline ret;

    std::vector<std::shared_ptr<Shape>> segs = getExploded();
    for (int i = 0; i < segs.size(); i++) {
        std::shared_ptr<Shape> seg = segs[i];
        if (seg->getShapeType() == Shape::Arc) {
            std::shared_ptr<Arc> arc = seg.dynamicCast<Arc>();
            Polyline pl = arc->approximateWithLinesTan(segmentLength);
            ret.appendShape(pl);
        }
        else {
            ret.appendShape(*seg);
        }
    }

    ret.autoClose();
    return ret;
}

/**
 * \return A QPainterPath object that represents this polyline.
 */
RPainterPath Polyline::toPainterPath(bool addOriginalShapes) const
{
    RPainterPath ret;

    if (vertices.size() <= 1) {
        return ret;
    }

    ret.moveTo(vertices.at(0));

    for (int i = 0; i < vertices.size(); i++) {
        if (!closed && i == vertices.size() - 1) {
            break;
        }
        std::shared_ptr<Shape> shape = getSegmentAt(i);
        ret.addShape(shape);
        if (addOriginalShapes) {
            ret.addOriginalShape(shape);
        }
    }

    return ret;
}

void Polyline::stripWidths()
{
    for (int i = 0; i < startWidths.size(); i++) {
        startWidths[i] = 0.0;
    }
    for (int i = 0; i < endWidths.size(); i++) {
        endWidths[i] = 0.0;
    }
}

void Polyline::setMinimumWidth(double w)
{
    for (int i = 0; i < startWidths.size(); i++) {
        if (startWidths[i] > NS::PointTolerance) {
            startWidths[i] = qMax(startWidths[i], w);
        }
    }
    for (int i = 0; i < endWidths.size(); i++) {
        if (endWidths[i] > NS::PointTolerance) {
            endWidths[i] = qMax(endWidths[i], w);
        }
    }
}

int Polyline::getSegmentAtDist(double dist)
{
    if (polylineProxy != NULL) {
        return polylineProxy->getSegmentAtDist(*this, dist);
    }
    return -1;
}

/**
 * Relocates the start point of this closed polyline to the given point.
 * The visual appearance of the polyline does not change.
 */
bool Polyline::relocateStartPoint(const Vec3d &p)
{
    if (polylineProxy != NULL) {
        return polylineProxy->relocateStartPoint(*this, p);
    }
    return false;
}

bool Polyline::relocateStartPoint(double dist)
{
    if (polylineProxy != NULL) {
        return polylineProxy->relocateStartPoint(*this, dist);
    }
    return false;
}

bool Polyline::convertToClosed()
{
    if (isClosed()) {
        // nothing to do: polyline is already logically closed:
        return true;
    }

    if (!isGeometricallyClosed()) {
        // cannot convert geometrically open polyline to closed (use setClosed
        // instead):
        return false;
    }

    removeLastVertex();
    setClosed(true);
    return true;
}

bool Polyline::convertToOpen()
{
    if (!isClosed()) {
        // nothing to do: polyline is already logically or geometrically open:
        return true;
    }

    std::shared_ptr<Shape> last = getLastSegment();
    setClosed(false);
    appendShape(*last);
    return true;
}

/**
 * \return True if the segment at the given position is a line.
 */
bool Polyline::isLineSegment(int i) const
{
    if (i < 0 || i > bulges.size()) {
        return true;
    }

    return Polyline::isStraight(bulges.at(i));
}

/**
 * \return True if the given bulge indicates a straight line segment (i.e. is
 * 0.0).
 */
bool Polyline::isStraight(double bulge)
{
    return fabs(bulge) < 1.0e-6;
}

/**
 * \return List of RLines and RArcs describing this polyline.
 */
std::vector<std::shared_ptr<Shape>> Polyline::getExploded(int segments) const
{
    // Q_UNUSED(segments);

    std::vector<std::shared_ptr<Shape>> ret;

    if (vertices.size() <= 1) {
        return ret;
    }

    for (int i = 0; i < vertices.size(); i++) {
        if (!closed && i == vertices.size() - 1) {
            break;
        }

        std::shared_ptr<Shape> subShape = getSegmentAt(i);
        if (subShape.isNull()) {
            continue;
        }

        ret.push_back(subShape);
    }

    return ret;
}

std::vector<QPair<Polyline, Polyline>> Polyline::getLeftRightOutline() const
{
    if (Polyline::hasProxy()) {
        return Polyline::getPolylineProxy()->getLeftRightOutline(*this);
    }
    else {
        return std::vector<QPair<Polyline, Polyline>>();
    }
}

std::vector<Polyline> Polyline::getOutline() const
{
    if (Polyline::hasProxy()) {
        return Polyline::getPolylineProxy()->renderThickPolyline(*this);
    }
    else {
        return std::vector<Polyline>();
    }
}

/**
 * \return Number of segments. The number of segments equals the
 *      number of vertices for a closed polyline and one less for
 *      an open polyline.
 */
int Polyline::countSegments() const
{
    int ret = countVertices();
    if (!closed) {
        ret -= 1;
    }
    if (ret < 0) {
        ret = 0;
    }
    return ret;
}

/**
 * \return Shape of segment at given position.
 */
std::shared_ptr<Shape> Polyline::getSegmentAt(int i) const
{
    if (i < 0 || i >= vertices.size() || i >= bulges.size()) {
        qWarning() << "Polyline::getSegmentAt(" << i << "): i out of range";
        return std::shared_ptr<Shape>();
    }

    Vec3d p1 = vertices.at(i);
    Vec3d p2 = vertices.at((i + 1) % vertices.size());

    if (Polyline::isStraight(bulges.at(i))) {
        return std::shared_ptr<Shape>(new Line(p1, p2));
    }

    else {
        double bulge = bulges.at(i);
        bool reversed = bulge < 0.0;
        double alpha = atan(bulge) * 4.0;

        if (fabs(alpha) > 2 * M_PI - NS::PointTolerance) {
            return std::shared_ptr<Shape>(new Line(p1, p2));
            // return std::shared_ptr<Shape>();
        }

        double radius;
        Vec3d center;
        Vec3d middle;
        double dist;
        double angle;

        middle = (p1 + p2) / 2.0;
        dist = p1.getDistanceTo(p2) / 2.0;
        angle = p1.getAngleTo(p2);

        // alpha can't be 0.0 at this point
        radius = fabs(dist / sin(alpha / 2.0));

        double rootTerm = fabs(radius * radius - dist * dist);
        double h = sqrt(rootTerm);

        if (bulge > 0.0) {
            angle += M_PI / 2.0;
        }
        else {
            angle -= M_PI / 2.0;
        }

        if (fabs(alpha) > M_PI) {
            h *= -1.0;
        }

        center.setPolar(h, angle);
        center += middle;

        double a1;
        double a2;

        a1 = center.getAngleTo(p1);
        a2 = center.getAngleTo(p2);

        return std::shared_ptr<Shape>(
            new Arc(center, radius, a1, a2, reversed));
    }
}

bool Polyline::isArcSegmentAt(int i) const
{
    if (i < 0 || i >= bulges.size()) {
        return false;
    }
    return !Polyline::isStraight(bulges[i]);
}

std::shared_ptr<Shape> Polyline::getLastSegment() const
{
    if (countSegments() == 0) {
        return std::shared_ptr<Shape>();
    }
    return getSegmentAt(countSegments() - 1);
}

std::shared_ptr<Shape> Polyline::getFirstSegment() const
{
    if (countSegments() == 0) {
        return std::shared_ptr<Shape>();
    }
    return getSegmentAt(0);
}

/**
 * Checks if the given point is inside this closed polygon. If this
 * polyline is not closed, false is returned.
 *
 * \see setClosed
 *
 * \param borderIsInside True if a position on the polyline counts as inside.
 * \param tolerance Tolerance used to check if point is on polyline.
 */
bool Polyline::contains(const Vec3d &point, bool borderIsInside,
                        double tolerance) const
{
    if (!isGeometricallyClosed(tolerance)) {
        return false;
    }

    // check if point is on polyline:
    if (isOnShape(point, true, tolerance)) {
        return borderIsInside;
    }

    if (hasArcSegments()) {
        // TODO: not always reliable:
        QPainterPath pp = toPainterPath();
        return pp.contains(QPointF(point.x, point.y));
    }

    int nvert = vertices.size();
    int i, j;
    bool c = false;
    for (i = 0, j = nvert - 1; i < nvert; j = i++) {
        if (((vertices[i].y > point.y) != (vertices[j].y > point.y)) &&
            (point.x < (vertices[j].x - vertices[i].x) *
                               (point.y - vertices[i].y) /
                               (vertices[j].y - vertices[i].y) +
                           vertices[i].x)) {
            c = !c;
        }
    }
    return c;
}

/**
 * Checks if the given shape is completely inside this closed polygon. If this
 * polyline is not closed, false is returned.
 *
 * \see setClosed
 *
 * If the shape touches the polyline, false is returned.
 *
 * \param shape The shape to check.
 */
bool Polyline::containsShape(const Shape &shape) const
{
    // check if the shape intersects with any of the polygon edges:
    bool gotIntersection = false;
    if (shape.intersectsWith(*this)) {
        gotIntersection = true;
    }

    if (gotIntersection) {
        // normal selection:
        // entity does not match if there is an intersection:
        return false;
    }

    if (Shape::isPolylineShape(shape)) {
        const Polyline &pl = dynamic_cast<const Polyline &>(shape);

        BBox bbOuter = pl.getBoundingBox();
        BBox bbInner = shape.getBoundingBox();

        if (!bbOuter.contains(bbInner)) {
            return false;
        }

        for (int i = 0; i < pl.countVertices() && i < 5; i++) {
            if (!contains(pl.getVertexAt(i))) {
                return false;
            }
        }
        return true;
    }

    // check if the shape is completely inside the polygon.
    // this is the case if one point on the entity is inside the polygon
    // and the entity does not intersect with the polygon.
    else if (shape.isDirected()) {
        return contains(shape.getStartPoint()) && contains(shape.getEndPoint());
    }
    else {
        // circle:
        if (Shape::isCircleShape(shape)) {
            const Circle &circle = dynamic_cast<const Circle &>(shape);
            Vec3d p1 = circle.getCenter() + Vec3d(circle.getRadius(), 0);
            Vec3d p2 = circle.getCenter() + Vec3d(-circle.getRadius(), 0);
            if (contains(p1) || contains(p2)) {
                return true;
            }
            return false;
        }
        else {
            // other shapes:
            Vec3d pointOnShape = shape.getPointOnShape();
            if (contains(pointOnShape, true)) {
                return true;
            }
            return false;
        }
    }

    // unsupported shape:
    Q_ASSERT(false);
    return false;
}

/**
 * \return Any point that is inside this polyline.
 */
Vec3d Polyline::getPointInside() const
{
    if (polylineProxy != NULL) {
        return polylineProxy->getPointInside(*this);
    }
    return Vec3d::invalid;
}

Vec3d Polyline::getStartPoint() const
{
    if (vertices.size() == 0) {
        return Vec3d::invalid;
    }

    return vertices.first();
}

Vec3d Polyline::getEndPoint() const
{
    if (vertices.size() == 0) {
        return Vec3d::invalid;
    }

    if (isClosed()) {
        return vertices.first();
    }

    return vertices.last();
}

Vec3d Polyline::getMiddlePoint() const
{
    std::vector<Vec3d> pts =
        getPointsWithDistanceToEnd(getLength() / 2, NS::FromStart);
    if (pts.size() == 1) {
        return pts[0];
    }
    return Vec3d::invalid;
}

void Polyline::moveStartPoint(const Vec3d &pos)
{
    if (vertices.isEmpty()) {
        return;
    }
    vertices.first() = pos;
}

void Polyline::moveEndPoint(const Vec3d &pos)
{
    if (vertices.isEmpty()) {
        return;
    }
    vertices.last() = pos;
}

void Polyline::moveSegmentAt(int i, const Vec3d &offset)
{
    moveVertexAt(i, offset);
    if (i + 1 < countVertices()) {
        moveVertexAt(i + 1, offset);
    }
    else {
        if (closed) {
            moveVertexAt(0, offset);
        }
    }
}

double Polyline::getDirection1() const
{
    if (vertices.size() == 0) {
        return std::numeric_limits<double>::quiet_NaN();
    }

    std::shared_ptr<Shape> shape = getSegmentAt(0);
    return shape->getDirection1();
}

double Polyline::getDirection2() const
{
    if (vertices.size() == 0) {
        return std::numeric_limits<double>::quiet_NaN();
    }

    int i = vertices.size() - 2;
    if (isClosed()) {
        i++;
    }

    std::shared_ptr<Shape> shape = getSegmentAt(i);
    if (shape.isNull()) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    return shape->getDirection2();
}

NS::Side Polyline::getSideOfPoint(const Vec3d &point) const
{
    int i = getClosestSegment(point);
    if (i < 0 || i >= countSegments()) {
        return NS::NoSide;
    }

    std::shared_ptr<Shape> segment = getSegmentAt(i);
    if (segment.isNull()) {
        return NS::NoSide;
    }
    return segment->getSideOfPoint(point);
}

BBox Polyline::getBoundingBox() const
{
    BBox ret;

    if (hasWidths()) {
        std::vector<Polyline> outline = getOutline();
        for (int i = 0; i < outline.size(); i++) {
            Q_ASSERT(!outline[i].hasWidths());
            BBox bb = outline[i].getBoundingBox();
            ret.growToInclude(bb);
        }
        return ret;
    }

    if (countVertices() == 1) {
        ret = BBox(vertices.at(0), vertices.at(0));
    }

    std::vector<std::shared_ptr<Shape>> sub = getExploded();
    std::vector<std::shared_ptr<Shape>>::iterator it;
    for (it = sub.begin(); it != sub.end(); ++it) {
        BBox bb = (*it)->getBoundingBox();
        ret.growToInclude(bb);
    }

    return ret;
}

/**
 * \return Area of (implicitly closed) polyline.
 */
double Polyline::getArea() const
{
    double ret = 0.0;
    if (polylineProxy != NULL) {
        ret = polylineProxy->getArea(*this, 0.01);
    }
    return ret;
}

double Polyline::getLength() const
{
    double ret = 0.0;

    std::vector<std::shared_ptr<Shape>> sub = getExploded();
    std::vector<std::shared_ptr<Shape>>::iterator it;
    for (it = sub.begin(); it != sub.end(); ++it) {
        double l = (*it)->getLength();
        if (Math::isNormal(l)) {
            ret += l;
        }
    }

    return ret;
}

/**
 * \return Length along polyline from start point to given point p or
 * the closest point to p on the polyline.
 */
double Polyline::getLengthTo(const Vec3d &p, bool limited) const
{
    double ret = 0.0;

    if (p.equalsFuzzy(getStartPoint())) {
        return 0.0;
    }

    int segIdx = getClosestSegment(p);
    if (segIdx < 0) {
        return -1.0;
    }

    for (int i = 0; i < segIdx; i++) {
        double l = getSegmentAt(i)->getLength();
        if (Math::isNormal(l)) {
            ret += l;
        }
    }

    std::shared_ptr<Shape> seg = getSegmentAt(segIdx);
    bool lim = limited;
    if (segIdx != 0 && segIdx != countSegments() - 1) {
        lim = true;
    }
    Vec3d p2 = seg->getClosestPointOnShape(p, lim);
    seg->trimEndPoint(p2);
    ret += seg->getLength();

    return ret;
}

/**
 * \return Length of all segments from first index (fromIndex) to last index
 * (toIndex), excluding toIndex.
 */
double Polyline::getSegmentsLength(int fromIndex, int toIndex) const
{
    double len = 0.0;
    for (int i = fromIndex; i < toIndex; i++) {
        std::shared_ptr<Shape> segment = getSegmentAt(i);
        len += segment->getLength();
    }
    return len;
}

std::vector<double> Polyline::getDistancesFromStart(const Vec3d &p) const
{
    std::vector<double> ret;

    // any segment might contain point (self intersection):
    double len = 0.0;
    for (int i = 0; i < countSegments(); i++) {
        std::shared_ptr<Shape> segment = getSegmentAt(i);
        if (segment->getDistanceTo(p) < 0.0001) {
            ret.push_back(len + segment->getDistanceFromStart(p));
        }
        len += segment->getLength();
    }

    // point is not on polyline, return distance to point closest to position:
    if (ret.isEmpty()) {
        ret.push_back(getLengthTo(p, true));
    }

    return ret;
}

std::vector<Vec3d> Polyline::getEndPoints() const
{
    return vertices;
}

/*
// maybe:
void Polyline::forEachSubShape(Polyline* instance, void
(Polyline::*callBack)()) const{ std::vector<std::shared_ptr<Shape> > sub =
getExploded(); std::vector<std::shared_ptr<Shape> >::iterator it; for
(it=sub.begin(); it!=sub.end(); ++it) { (instance->*callBack)(*it);
    }
}
*/

std::vector<Vec3d> Polyline::getMiddlePoints() const
{
    std::vector<Vec3d> ret;

    std::vector<std::shared_ptr<Shape>> sub = getExploded();
    std::vector<std::shared_ptr<Shape>>::iterator it;
    for (it = sub.begin(); it != sub.end(); ++it) {
        ret.push_back((*it)->getMiddlePoints());
    }

    return ret;
}

std::vector<Vec3d> Polyline::getCenterPoints() const
{
    std::vector<Vec3d> ret;

    std::vector<std::shared_ptr<Shape>> sub = getExploded();
    std::vector<std::shared_ptr<Shape>>::iterator it;
    for (it = sub.begin(); it != sub.end(); ++it) {
        ret.push_back((*it)->getCenterPoints());
    }

    return ret;
}

Vec3d Polyline::getPointAtPercent(double p) const
{
    double length = getLength();
    double distance = p * length;
    std::vector<Vec3d> candidates =
        getPointsWithDistanceToEnd(distance, NS::FromStart | NS::AlongPolyline);
    if (candidates.size() != 1) {
        return Vec3d::invalid;
    }
    return candidates.at(0);
}

std::vector<Vec3d> Polyline::getPointsWithDistanceToEnd(double distance,
                                                        int from) const
{
    std::vector<Vec3d> ret;

    std::vector<std::shared_ptr<Shape>> sub = getExploded();

    if (sub.isEmpty()) {
        return ret;
    }

    if (from & NS::AlongPolyline) {
        double remainingDist;
        double len;

        if (from & NS::FromStart) {
            if (distance < 0.0) {
                // extend at start:
                ret.push_back(sub.first()->getPointsWithDistanceToEnd(
                    distance, NS::FromStart));
            }
            else {
                remainingDist = distance;
                for (int i = 0; i < sub.size(); i++) {
                    len = sub[i]->getLength();
                    if (remainingDist > len) {
                        remainingDist -= len;
                    }
                    else {
                        ret.push_back(sub[i]->getPointsWithDistanceToEnd(
                            remainingDist, NS::FromStart));
                        break;
                    }
                }
            }
        }

        if (from & NS::FromEnd) {
            if (distance < 0.0) {
                // extend at end:
                ret.push_back(sub.last()->getPointsWithDistanceToEnd(
                    distance, NS::FromEnd));
            }
            else {
                remainingDist = distance;
                for (int i = sub.size() - 1; i >= 0; i--) {
                    len = sub[i]->getLength();
                    if (remainingDist > len) {
                        remainingDist -= len;
                    }
                    else {
                        ret.push_back(sub[i]->getPointsWithDistanceToEnd(
                            remainingDist, NS::FromEnd));
                        break;
                    }
                }
            }
        }
    }
    else {
        std::vector<std::shared_ptr<Shape>>::iterator it;
        for (it = sub.begin(); it != sub.end(); ++it) {
            ret.push_back((*it)->getPointsWithDistanceToEnd(distance, from));
        }
    }

    return ret;
}

std::vector<Vec3d> Polyline::getPointCloud(double segmentLength) const
{
    std::vector<Vec3d> ret;
    for (int i = 0; i < countSegments(); i++) {
        std::shared_ptr<Shape> seg = getSegmentAt(i);
        if (seg.isNull()) {
            continue;
        }
        ret.push_back(seg->getPointCloud(segmentLength));
    }
    return ret;
}

double Polyline::getAngleAt(double distance, NS::From from) const
{
    std::vector<std::shared_ptr<Shape>> sub = getExploded();

    if (from & NS::AlongPolyline) {
        double remainingDist;
        double len;

        if (from & NS::FromStart) {
            remainingDist = distance;
            for (int i = 0; i < sub.size(); i++) {
                len = sub[i]->getLength();
                if (remainingDist > len) {
                    remainingDist -= len;
                }
                else {
                    return sub[i]->getAngleAt(remainingDist, NS::FromStart);
                }
            }
        }

        if (from & NS::FromEnd) {
            remainingDist = distance;
            for (int i = sub.size() - 1; i >= 0; i--) {
                len = sub[i]->getLength();
                if (remainingDist > len) {
                    remainingDist -= len;
                }
                else {
                    return sub[i]->getAngleAt(remainingDist, NS::FromEnd);
                }
            }
        }
    }
    // else {
    //  not implemented / never used
    //    Q_ASSERT(false);
    //}

    return std::numeric_limits<double>::quiet_NaN();
}

Vec3d Polyline::getVectorTo(const Vec3d &point, bool limited,
                            double strictRange) const
{
    Vec3d ret = Vec3d::invalid;

    std::vector<std::shared_ptr<Shape>> sub = getExploded();
    for (int i = 0; i < sub.size(); i++) {
        std::shared_ptr<Shape> shape = sub.at(i);
        bool lim = limited;
        if (i != 0 && i != sub.size() - 1) {
            // segments in the middle: always limited:
            lim = true;
        }
        Vec3d v = shape->getVectorTo(point, lim, strictRange);
        if (v.isValid() &&
            (!ret.isValid() || v.getMagnitude() < ret.getMagnitude())) {
            ret = v;
        }
    }

    return ret;
}

double Polyline::getDistanceTo(const Vec3d &point, bool limited,
                               double strictRange) const
{
    if (!hasWidths()) {
        return Shape::getDistanceTo(point, limited, strictRange);
    }

    // Q_UNUSED(limited)

    if (!getBoundingBox().grow(strictRange).contains(point)) {
        return std::numeric_limits<double>::quiet_NaN();
    }

    double ret = std::numeric_limits<double>::quiet_NaN();

    std::vector<Polyline> outline = getOutline();
    for (int i = 0; i < outline.size(); i++) {
        Q_ASSERT(!outline[i].hasWidths());
        double d = outline[i].getDistanceTo(point);
        if (Math::isNaN(ret) || d < ret) {
            ret = d;
        }

        if (outline[i].isGeometricallyClosed()) {
            if (outline[i].contains(point)) {
                if (Math::isNaN(ret) || strictRange < ret) {
                    ret = strictRange;
                }
            }
        }
    }

    return ret;
}

int Polyline::getClosestSegment(const Vec3d &point) const
{
    int ret = -1;
    double minDist = -1;

    for (int i = 0; i < countSegments(); i++) {
        std::shared_ptr<Shape> segment = getSegmentAt(i);
        if (segment.isNull()) {
            break;
        }
        double dist = segment->getDistanceTo(point, true);
        if (!Math::isNormal(dist)) {
            continue;
        }
        if (minDist < 0 || dist < minDist) {
            minDist = dist;
            ret = i;
        }
    }

    return ret;
}

int Polyline::getClosestVertex(const Vec3d &point) const
{
    return point.getClosestIndex(getVertices());
}

bool Polyline::move(const Vec3d &offset)
{
    for (int i = 0; i < vertices.size(); i++) {
        vertices[i].move(offset);
    }
    return true;
}

bool Polyline::rotate(double rotation, const Vec3d &center)
{
    if (fabs(rotation) < NS::AngleTolerance) {
        return false;
    }
    for (int i = 0; i < vertices.size(); i++) {
        vertices[i].rotate(rotation, center);
    }
    return true;
}

bool Polyline::scale(double scaleFactor, const Vec3d &center)
{
    return Shape::scale(scaleFactor, center);
}

bool Polyline::scale(const Vec3d &scaleFactors, const Vec3d &center)
{
    if (hasArcSegments() &&
        !Math::fuzzyCompare(scaleFactors.x, scaleFactors.y)) {
        // non-uniform scaling of polyline with arcs:
        Polyline pl;
        for (int i = 0; i < countSegments(); i++) {
            std::shared_ptr<Shape> seg = getSegmentAt(i);
            if (seg.isNull()) {
                continue;
            }

            // TODO: apply widths to new segments:
            // double w1 = getStartWidthAt(i);
            // double w2 = getStartWidthAt((i+1)%countVertices());

            std::shared_ptr<Shape> newSeg;
            if (Shape::isLineShape(*seg)) {
                newSeg = seg;
                newSeg->scale(scaleFactors, center);
            }
            else {
                newSeg = Shape::scaleArc(*seg, scaleFactors, center);
            }

            if (!newSeg.isNull()) {
                pl.appendShape(*newSeg);
            }
        }
        // new polyline with tangentially connected small arc segments for
        // original arc segments:
        *this = pl;
        return true;
    }

    for (int i = 0; i < vertices.size(); i++) {
        vertices[i].scale(scaleFactors, center);
    }
    for (int i = 0; i < startWidths.size(); i++) {
        if (startWidths[i] > 0.0) {
            startWidths[i] *= fabs(scaleFactors.x);
        }
    }
    for (int i = 0; i < endWidths.size(); i++) {
        if (endWidths[i] > 0.0) {
            endWidths[i] *= fabs(scaleFactors.x);
        }
    }
    // factor in x or in y is negative -> mirror:
    if ((scaleFactors.x < 0) != (scaleFactors.y < 0)) {
        for (int i = 0; i < bulges.size(); i++) {
            bulges[i] *= -1;
        }
    }
    return true;
}

bool Polyline::mirror(const Line &axis)
{
    int i;
    for (i = 0; i < vertices.size(); i++) {
        vertices[i].mirror(axis);
    }
    for (i = 0; i < bulges.size(); i++) {
        bulges[i] *= -1;
    }
    return true;
}

bool Polyline::reverse()
{
    std::vector<Vec3d> vs = vertices;
    if (closed) {
        vs.push_back(vs.first());
    }

    Polyline nPolyline;

    for (int i = vs.count() - 1, k = 0; i >= 0; i--, k++) {
        nPolyline.appendVertex(vs[i]);
        if (i > 0) {
            nPolyline.setBulgeAt(k, -bulges[i - 1]);

            nPolyline.setStartWidthAt(k, endWidths[i - 1]);
            nPolyline.setEndWidthAt(k, startWidths[i - 1]);
        }
    }
    if (closed) {
        nPolyline.convertToClosed();
    }

    *this = nPolyline;

    Q_ASSERT(vertices.size() == bulges.size());
    Q_ASSERT(vertices.size() == startWidths.size());
    Q_ASSERT(vertices.size() == endWidths.size());

    return true;
}

Polyline Polyline::getReversed() const
{
    Polyline ret = *this;
    ret.reverse();
    return ret;
}

bool Polyline::stretch(const Polyline &area, const Vec3d &offset)
{
    for (int i = 0; i < vertices.size(); i++) {
        vertices[i].stretch(area, offset);
    }
    return true;
}

NS::Ending Polyline::getTrimEnd(const Vec3d &trimPoint, const Vec3d &clickPoint)
{
    if (polylineProxy != NULL) {
        return polylineProxy->getTrimEnd(*this, trimPoint, clickPoint);
    }
    return NS::EndingNone;
}

bool Polyline::trimStartPoint(const Vec3d &trimPoint, const Vec3d &clickPoint,
                              bool extend)
{
    if (polylineProxy != NULL) {
        return polylineProxy->trimStartPoint(*this, trimPoint, clickPoint,
                                             extend);
    }
    return false;
}

bool Polyline::trimEndPoint(const Vec3d &trimPoint, const Vec3d &clickPoint,
                            bool extend)
{
    if (polylineProxy != NULL) {
        return polylineProxy->trimEndPoint(*this, trimPoint, clickPoint,
                                           extend);
    }
    return false;
}

bool Polyline::trimStartPoint(double trimDist)
{
    if (polylineProxy != NULL) {
        return polylineProxy->trimStartPoint(*this, trimDist);
    }
    return false;
}

bool Polyline::trimEndPoint(double trimDist)
{
    if (polylineProxy != NULL) {
        return polylineProxy->trimEndPoint(*this, trimDist);
    }
    return false;
}

/**
 * Simplify by attempting to skip nodes within given tolerance.
 * \return True if nodes have been skipped.
 */
bool Polyline::simplify(double tolerance)
{
    if (Polyline::hasProxy()) {
        return Polyline::getPolylineProxy()->simplify(*this, tolerance);
    }
    else {
        return false;
    }
}

/**
 * Verifies the tangency of this polyline.
 */
std::vector<Vec3d> Polyline::verifyTangency(double toleranceMin,
                                            double toleranceMax)
{
    if (Polyline::hasProxy()) {
        return Polyline::getPolylineProxy()->verifyTangency(*this, toleranceMin,
                                                            toleranceMax);
    }
    else {
        return std::vector<Vec3d>();
    }
}

/**
 * Modifies (bevels, rounds, trims) the corner of this polyline between
 * segmentIndex1 and segmentIndex2 at the given segment endings. The given
 * cornerShape (bevel, rounding) is inserted between.
 */
Polyline Polyline::modifyPolylineCorner(const Shape &trimmedShape1,
                                        NS::Ending ending1, int segmentIndex1,
                                        const Shape &trimmedShape2,
                                        NS::Ending ending2, int segmentIndex2,
                                        const Shape *cornerShape) const
{
    std::shared_ptr<Shape> segment;

    Polyline pl;

    if (segmentIndex1 < segmentIndex2 && ending1 == NS::EndingEnd &&
        ending2 == NS::EndingStart) {
        for (int i = 0; i < segmentIndex1; i++) {
            segment = getSegmentAt(i);
            pl.appendShape(*segment);
            pl.setStartWidthAt(pl.startWidths.size() - 2, getStartWidthAt(i));
            pl.setEndWidthAt(pl.endWidths.size() - 2, getEndWidthAt(i));
        }

        pl.appendShapeAuto(trimmedShape1);
        if (cornerShape != NULL) {
            pl.appendShapeAuto(*cornerShape);
        }
        pl.appendShapeAuto(trimmedShape2);

        for (int i = segmentIndex2 + 1; i < countSegments(); i++) {
            segment = getSegmentAt(i);
            pl.appendShape(*segment);
            pl.setStartWidthAt(pl.startWidths.size() - 2, getStartWidthAt(i));
            pl.setEndWidthAt(pl.endWidths.size() - 2, getEndWidthAt(i));
        }
    }
    else if (segmentIndex1 > segmentIndex2 && ending1 == NS::EndingStart &&
             ending2 == NS::EndingEnd) {
        for (int i = 0; i < segmentIndex2; i++) {
            segment = getSegmentAt(i);
            pl.appendShape(*segment);
            pl.setStartWidthAt(pl.startWidths.size() - 2, getStartWidthAt(i));
            pl.setEndWidthAt(pl.endWidths.size() - 2, getEndWidthAt(i));
        }

        pl.appendShapeAuto(trimmedShape2);
        if (cornerShape != NULL) {
            pl.appendShapeAuto(*cornerShape);
        }
        pl.appendShapeAuto(trimmedShape1);

        for (int i = segmentIndex1 + 1; i < countSegments(); i++) {
            segment = getSegmentAt(i);
            pl.appendShape(*segment);
            pl.setStartWidthAt(pl.startWidths.size() - 2, getStartWidthAt(i));
            pl.setEndWidthAt(pl.endWidths.size() - 2, getEndWidthAt(i));
        }
    }
    else if (segmentIndex1 < segmentIndex2 && ending1 == NS::EndingStart &&
             ending2 == NS::EndingEnd) {
        pl.appendShapeAuto(trimmedShape1);
        for (int i = segmentIndex1 + 1; i < segmentIndex2; i++) {
            segment = getSegmentAt(i);
            pl.appendShape(*segment);
            pl.setStartWidthAt(pl.startWidths.size() - 2, getStartWidthAt(i));
            pl.setEndWidthAt(pl.endWidths.size() - 2, getEndWidthAt(i));
        }
        pl.appendShapeAuto(trimmedShape2);
        if (cornerShape != NULL) {
            pl.appendShapeAuto(*cornerShape);
        }
    }
    else if (segmentIndex1 > segmentIndex2 && ending1 == NS::EndingEnd &&
             ending2 == NS::EndingStart) {
        pl.appendShapeAuto(trimmedShape2);
        for (int i = segmentIndex2 + 1; i < segmentIndex1; i++) {
            segment = getSegmentAt(i);
            pl.appendShape(*segment);
            pl.setStartWidthAt(pl.startWidths.size() - 2, getStartWidthAt(i));
            pl.setEndWidthAt(pl.endWidths.size() - 2, getEndWidthAt(i));
        }
        pl.appendShapeAuto(trimmedShape1);
        if (cornerShape != NULL) {
            pl.appendShapeAuto(*cornerShape);
        }
    }

    return pl;
}

bool Polyline::isConcave() const
{
    return !getConcaveVertices().isEmpty();
}

std::vector<Vec3d> Polyline::getConvexVertices(bool convex) const
{
    if (!isGeometricallyClosed()) {
        return std::vector<Vec3d>();
    }

    Polyline pl = *this;
    pl.autoClose();

    NS::Orientation ori = pl.getOrientation();

    std::vector<Vec3d> ret;

    for (int i = 0; i < pl.vertices.count(); i++) {
        int iPrev = Math::absmod(i - 1, pl.vertices.count());
        std::shared_ptr<Shape> segmentPrev = pl.getSegmentAt(iPrev);
        std::shared_ptr<Shape> segmentNext = pl.getSegmentAt(i);

        double aPrev = segmentPrev->getDirection2() + M_PI;
        double aNext = segmentNext->getDirection1();

        Vec3d pPrev = Vec3d::createPolar(1.0, aPrev);
        Vec3d pNext = Vec3d::createPolar(1.0, aNext);

        Vec3d cp = Vec3d::getCrossProduct(pPrev, pNext);

        if (convex) {
            if (ori == NS::CW && cp.z < 0.0 || ori == NS::CCW && cp.z > 0.0) {
                ret.push_back(pl.vertices[i]);
            }
        }
        else {
            if (ori == NS::CCW && cp.z < 0.0 || ori == NS::CW && cp.z > 0.0) {
                ret.push_back(pl.vertices[i]);
            }
        }
    }

    return ret;
}

std::vector<Vec3d> Polyline::getConcaveVertices() const
{
    return getConvexVertices(false);
}

/**
 * \return Centroid of this polygon or invalid if polyline contains arc
 * segments. The polyline is implicitely considered to be closed.
 */
Vec3d Polyline::getCentroid() const
{
    if (hasArcSegments()) {
        return Vec3d::invalid;
    }

    double xSum = 0;
    double ySum = 0;
    double signedArea = 0;
    int n = vertices.size();

    for (int i = 0; i < n; i++) {
        double x0 = vertices[i].x;
        double y0 = vertices[i].y;
        double x1 = vertices[(i + 1) % n].x;
        double y1 = vertices[(i + 1) % n].y;

        // calculate the cross product of the edges
        double crossProduct = x0 * y1 - x1 * y0;
        signedArea += crossProduct;
        xSum += (x0 + x1) * crossProduct;
        ySum += (y0 + y1) * crossProduct;
    }

    signedArea *= 0.5;
    double centroidX = xSum / (6.0 * signedArea);
    double centroidY = ySum / (6.0 * signedArea);

    return Vec3d(centroidX, centroidY);
}

std::vector<Polyline> Polyline::splitAtDiscontinuities(double tolerance) const
{
    if (polylineProxy != NULL) {
        return polylineProxy->splitAtDiscontinuities(*this, tolerance);
    }
    return std::vector<Polyline>() << *this;
}

/**
 * Splits the polyline into polylines with exclusively line or arc segments.
 */
std::vector<Polyline> Polyline::splitAtSegmentTypeChange() const
{
    if (polylineProxy != NULL) {
        return polylineProxy->splitAtSegmentTypeChange(*this);
    }
    return std::vector<Polyline>() << *this;
}

double Polyline::getBaseAngle() const
{
    if (polylineProxy != NULL) {
        return polylineProxy->getBaseAngle(*this);
    }
    return 0.0;
}

double Polyline::getWidth() const
{
    if (polylineProxy != NULL) {
        return polylineProxy->getWidth(*this);
    }
    return 0.0;
}

bool Polyline::setWidth(double v)
{
    if (polylineProxy != NULL) {
        return polylineProxy->setWidth(*this, v);
    }
    return false;
}

double Polyline::getHeight() const
{
    if (polylineProxy != NULL) {
        return polylineProxy->getHeight(*this);
    }
    return 0.0;
}

bool Polyline::setHeight(double v)
{
    if (polylineProxy != NULL) {
        return polylineProxy->setHeight(*this, v);
    }
    return false;
}

std::vector<Polyline> Polyline::morph(const Polyline &target, int steps,
                                      NS::Easing easing, bool zLinear,
                                      double customFactor) const
{
    if (polylineProxy != NULL) {
        return polylineProxy->morph(*this, target, steps, easing, zLinear,
                                    customFactor);
    }
    return std::vector<Polyline>();
}

Polyline Polyline::roundAllCorners(double radius) const
{
    if (polylineProxy != NULL) {
        return polylineProxy->roundAllCorners(*this, radius);
    }
    return *this;
}

Polyline Polyline::getPolygon(double segmentLength) const
{
    if (polylineProxy != NULL) {
        return polylineProxy->getPolygon(*this, segmentLength);
    }
    return *this;
}

Polyline Polyline::getPolygonHull(double angle, double tolerance,
                                  bool inner) const
{
    if (polylineProxy != NULL) {
        return polylineProxy->getPolygonHull(*this, angle, tolerance, inner);
    }
    return *this;
}
