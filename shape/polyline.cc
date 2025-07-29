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
#include "shape_algorithm_extern.inl"

#include <sstream>
#include <iomanip>
#include <assert.h>

namespace cada {
namespace shape {

Polyline::Polyline() : mClosed(false)
{
}

Polyline::Polyline(const std::vector<Vec2d> &vertices, bool closed)
    : mClosed(closed)
{
    setVertices(vertices);
}

bool Polyline::isValid() const
{
    return true;
}

NS::ShapeType Polyline::getShapeType() const
{
    return NS::Polyline;
}

Polyline *Polyline::cloneImpl() const
{
    Polyline *pClone = new Polyline();
    pClone->mVertices = mVertices;
    pClone->mBulges = mBulges;
    pClone->mClosed = mClosed;
    return pClone;
}

void Polyline::clear()
{
    mVertices.clear();
    mBulges.clear();

    assert(mVertices.size() == mBulges.size());
}

void Polyline::normalize(double tolerance)
{
    std::vector<Vec2d> newVertices;
    std::vector<double> newBulges;

    Vec2d vPrev;
    int newIndex = 0;

    for (size_t i = 0; i < mVertices.size(); i++) {
        Vec2d v = mVertices[i];
        double b = mBulges[i];

        if (i == 0 || !v.equalsFuzzy(vPrev, tolerance)) {
            newVertices.push_back(v);
            newBulges.push_back(b);
            newIndex = newIndex + 1;
            vPrev = v;
        }
        else if (i > 0) {
            newBulges[newIndex - 1] = b;
        }
    }

    // remove duplicate last vertex of closed polyline:
    if (mClosed) {
        if (newVertices.front().equalsFuzzy(newVertices.back(), tolerance)) {
            newVertices.pop_back();
            newBulges.pop_back();
        }
    }

    mVertices = newVertices;
    mBulges = newBulges;

    assert(mVertices.size() == mBulges.size());
}

bool Polyline::prependShape(const Shape *shape)
{
    return appendShape(shape, true);
}

bool Polyline::appendShape(const Shape *shape, bool prepend)
{
    bool ret = true;

    // append spline as polyline approximation:
    if (shape->getShapeType() == NS::Spline) {
        const Spline *spl = dynamic_cast<const Spline *>(shape);
        if (spl != NULL) {
            double tol = 0.01;
            auto pl = spl->approximateWithArcs(tol);
            return appendShape(pl.get(), prepend);
        }
    }

    // append ellipse as polyline approximation:
    else if (shape->getShapeType() == NS::Ellipse) {
        const Ellipse *elp = dynamic_cast<const Ellipse *>(shape);
        if (elp != NULL) {
            double seg = 32;
            auto pl = elp->approximateWithArcs(seg);
            return appendShape(pl.get(), prepend);
        }
    }

    // append circle as polyline to empty polyline:
    else if (shape->getShapeType() == NS::Circle && isEmpty()) {
        const Circle *circle = dynamic_cast<const Circle *>(shape);
        if (circle != NULL) {
            appendShape(ShapeFactory::instance()
                            ->createArc(circle->getCenter(),
                                        circle->getRadius(), 0.0, M_PI, false)
                            .get());
            appendShape(ShapeFactory::instance()
                            ->createArc(circle->getCenter(),
                                        circle->getRadius(), M_PI, 2 * M_PI,
                                        false)
                            .get());
            return true;
        }
    }

    // append full circle arc as circle (two arc segments) to empty polyline:
    else if (shape->getShapeType() == NS::Arc) {
        const Arc *arc = dynamic_cast<const Arc *>(shape);
        if (arc != NULL && arc->isFullCircle()) {
            appendShape(ShapeFactory::instance()
                            ->createCircle(arc->getCenter(), arc->getRadius())
                            .get());
            return true;
        }
    }

    // append polyline:
    else if (shape->getShapeType() == NS::Polyline) {
        const Polyline *pl = dynamic_cast<const Polyline *>(shape);
        if (pl != NULL) {
            if (prepend) {
                for (size_t i = pl->countSegments() - 1; i >= 0; --i) {
                    auto s = pl->getSegmentAt(i);
                    if (!s) {
                        continue;
                    }
                    ret = prependShape(s.get()) && ret;
                }
            }
            else {
                for (size_t i = 0; i < pl->countSegments(); ++i) {
                    auto s = pl->getSegmentAt(i);
                    if (!s) {
                        continue;
                    }
                    ret = appendShape(s.get()) && ret;
                }
            }
            return ret;
        }
    }

    double bulge = 0.0;

    const Arc *arc = dynamic_cast<const Arc *>(shape);
    if (arc != NULL) {
        bulge = arc->getBulge();
    }

    if (!shape->isDirected()) {
        return false;
    }

    Vec2d connectionPoint;
    Vec2d nextPoint;
    double gap;
    if (prepend) {
        // prepend:
        connectionPoint = shape->getEndPoint();
        nextPoint = shape->getStartPoint();
        if (mVertices.size() == 0) {
            // first point:
            appendVertex(connectionPoint);
        }
        gap = mVertices.front().getDistanceTo(connectionPoint);
    }
    else {
        // append:
        connectionPoint = shape->getStartPoint();
        nextPoint = shape->getEndPoint();
        if (mVertices.size() == 0) {
            // first point:
            appendVertex(connectionPoint);
        }
        gap = mVertices.back().getDistanceTo(connectionPoint);
    }

    if (!Math::fuzzyCompare(gap, 0.0, 1.0e-3)) {
        ret = false;
    }

    if (prepend) {
        prependVertex(nextPoint);
        setBulgeAt(0, bulge);
    }
    else {
        appendVertex(nextPoint);
        setBulgeAt(mBulges.size() - 2, bulge);
    }

    assert(mVertices.size() == mBulges.size());

    return ret;
}

bool Polyline::appendShapeAuto(const Shape *shape)
{
    if (!shape->isDirected()) {
        return false;
    }

    if (countVertices() > 0 &&
        getEndPoint().equalsFuzzy(shape->getEndPoint())) {
        std::unique_ptr<Shape> rev = shape->clone();
        rev->reverse();
        return appendShape(rev.get());
    }

    return appendShape(shape);
}

bool Polyline::appendShapeTrim(const Shape *shape)
{
    if (!shape->isDirected()) {
        return false;
    }

    if (countVertices() > 0) {
        if (getEndPoint().equalsFuzzy(shape->getStartPoint())) {
            return appendShape(shape);
        }
        if (getEndPoint().equalsFuzzy(shape->getEndPoint())) {
            std::unique_ptr<Shape> rev = shape->clone();
            rev->reverse();
            return appendShape(rev.get());
        }

        if (shape->getShapeType() == NS::Line) {
            auto lastSegment = getLastSegment();
            std::vector<Vec2d> ips =
                lastSegment->getIntersectionPoints(shape, false);
            if (ips.size() == 1) {
                Vec2d ip = ips[0];
                moveEndPoint(ip);
                std::unique_ptr<Shape> trimmed = shape->clone();
                trimmed->trimStartPoint(ip);
                return appendShape(trimmed.get());
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
        auto firstSegment = getFirstSegment();
        auto lastSegment = getLastSegment();

        if (!firstSegment || !lastSegment) {
            return false;
        }

        if (firstSegment->getShapeType() == NS::Line &&
            lastSegment->getShapeType() == NS::Line) {
            std::vector<Vec2d> ips =
                lastSegment->getIntersectionPoints(firstSegment.get(), false);
            if (ips.size() == 1) {
                Vec2d ip = ips[0];
                moveStartPoint(ip);
                moveEndPoint(ip);
                return true;
            }
        }
    }

    return false;
}

void Polyline::appendVertex(const Vec2d &vertex, double bulge)
{
    mVertices.push_back(vertex);
    mBulges.push_back(bulge);

    assert(mVertices.size() == mBulges.size());
}

void Polyline::appendVertex(double x, double y, double bulge)
{
    appendVertex(Vec2d(x, y), bulge);
}

void Polyline::prependVertex(const Vec2d &vertex, double bulge)
{
    mVertices.insert(mVertices.begin(), vertex);
    mBulges.insert(mBulges.begin(), bulge);

    assert(mVertices.size() == mBulges.size());
}

void Polyline::insertVertex(int index, const Vec2d &vertex, double bulgeBefore,
                            double bulgeAfter)
{
    mVertices.insert(mVertices.begin() + index, vertex);
    if (index > 0) {
        mBulges[index - 1] = bulgeBefore;
    }
    mBulges.insert(mBulges.begin() + index, bulgeAfter);

    assert(mVertices.size() == mBulges.size());
}

void Polyline::insertVertexAt(const Vec2d &point)
{
    int index = getClosestSegment(point);
    if (index < 0) {
        return;
    }

    auto seg1 = getSegmentAt(index);
    if (!seg1) {
        return;
    }

    Vec2d p = seg1->getClosestPointOnShape(point, false);

    std::unique_ptr<Shape> seg2 = seg1->clone();

    if (!seg1->isDirected() || !seg2->isDirected()) {
        return;
    }

    seg1->trimEndPoint(p);
    seg2->trimStartPoint(p);

    insertVertex(index + 1, p);

    Arc *arc1 = dynamic_cast<Arc *>(seg1.get());
    Arc *arc2 = dynamic_cast<Arc *>(seg2.get());
    if (!arc1) {
        setBulgeAt(index, 0.0);
    }
    else {
        setBulgeAt(index, arc1->getBulge());
    }

    if (!arc2) {
        setBulgeAt(index + 1, 0.0);
    }
    else {
        setBulgeAt(index + 1, arc2->getBulge());
    }

    assert(mVertices.size() == mBulges.size());
}

Vec2d Polyline::insertVertexAtDistance(double dist)
{
    return Vec2d::invalid;
}

void Polyline::removeFirstVertex()
{
    if (mVertices.empty()) {
        return;
    }

    mVertices.erase(mVertices.begin());
    mBulges.erase(mBulges.begin());

    assert(mVertices.size() == mBulges.size());
}

void Polyline::removeLastVertex()
{
    if (mVertices.empty()) {
        return;
    }

    mVertices.pop_back();
    mBulges.pop_back();

    assert(mVertices.size() == mBulges.size());
}

void Polyline::removeVertex(int index)
{
    mVertices.erase(mVertices.begin() + index);
    mBulges.erase(mBulges.begin() + index);

    assert(mVertices.size() == mBulges.size());
}

void Polyline::removeVerticesAfter(int index)
{
    mVertices = {mVertices.begin(), mVertices.begin() + index + 1};
    mBulges = {mBulges.begin(), mBulges.begin() + index + 1};

    assert(mVertices.size() == mBulges.size());
}

void Polyline::removeVerticesBefore(int index)
{
    mVertices = {mVertices.begin() + index, mVertices.end()};
    mBulges = {mBulges.begin() + index, mBulges.end()};

    assert(mVertices.size() == mBulges.size());
}

bool Polyline::isEmpty() const
{
    return false;
}

void Polyline::setVertices(const std::vector<Vec2d> &vertices)
{
    mVertices = vertices;

    mBulges.clear();
    for (size_t i = 0; i < mVertices.size(); ++i) {
        mBulges.push_back(0.0);
    }

    assert(mVertices.size() == mBulges.size());
}

std::vector<Vec2d> Polyline::getVertices() const
{
    return mVertices;
}

std::vector<Vec2d> &Polyline::getVertices()
{
    return mVertices;
}

Vec2d Polyline::getVertexAt(int i) const
{
    if (i < 0 || i >= mVertices.size()) {
        assert(false);
        return Vec2d::invalid;
    }

    return mVertices.at(i);
}

int Polyline::getVertexIndex(const Vec2d &v, double tolerance) const
{
    for (size_t i = 0; i < mVertices.size(); i++) {
        if (mVertices[i].equalsFuzzy(v, tolerance)) {
            return i;
        }
    }

    return -1;
}

Vec2d Polyline::getLastVertex() const
{
    if (mVertices.size() == 0) {
        return Vec2d::invalid;
    }

    return mVertices.at(mVertices.size() - 1);
}

void Polyline::setVertexAt(int i, const Vec2d &v)
{
    if (i < 0 || i >= mVertices.size()) {
        assert(false);
        return;
    }

    mVertices[i] = v;
}

void Polyline::moveVertexAt(int i, const Vec2d &offset)
{
    if (i < 0 || i >= mVertices.size()) {
        assert(false);
        return;
    }

    mVertices[i] += offset;
}

int Polyline::countVertices() const
{
    return mVertices.size();
}

void Polyline::setBulges(const std::vector<double> &b)
{
    mBulges = b;
}

std::vector<double> Polyline::getBulges() const
{
    return mBulges;
}

std::vector<double> &Polyline::getBulges()
{
    return mBulges;
}

double Polyline::getBulgeAt(int i) const
{
    if (i < 0 || i >= mBulges.size()) {
        return std::numeric_limits<double>::quiet_NaN();
    }

    return mBulges.at(i);
}

void Polyline::setBulgeAt(int i, double b)
{
    if (i < 0 || i >= mBulges.size()) {
        return;
    }

    mBulges[i] = b;
}

bool Polyline::hasArcSegments() const
{
    for (size_t i = 0; i < mBulges.size(); i++) {
        if (!isStraight(mBulges[i])) {
            return true;
        }
    }

    return false;
}

std::vector<double> Polyline::getVertexAngles() const
{
    NS::Orientation orientation = getOrientation(true);
    std::vector<double> ret;
    for (size_t i = 0; i < countVertices(); i++) {
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

    auto prevSegment = getSegmentAt(Math::absmod(i - 1, countSegments()));
    auto nextSegment = getSegmentAt(i % countSegments());

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

void Polyline::setClosed(bool on)
{
    mClosed = on;
}

bool Polyline::isClosed() const
{
    return mClosed;
}

bool Polyline::isGeometricallyClosed(double tolerance) const
{
    return isClosed() ||
           getStartPoint().getDistanceTo(getEndPoint()) < tolerance;
}

bool Polyline::autoClose(double tolerance)
{
    return false;
}

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

    assert(mVertices.size() == mBulges.size());

    return true;
}

bool Polyline::toLogicallyOpen()
{
    if (!isClosed()) {
        return false;
    }

    appendVertex(getEndPoint(), getBulgeAt(mVertices.size() - 1));
    setClosed(false);

    assert(mVertices.size() == mBulges.size());

    return true;
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
        auto plSegmented = convertArcToLineSegments(16);
        return plSegmented->getOrientation(implicitelyClosed);
    }

    Vec2d minV = Vec2d::invalid;
    std::unique_ptr<Shape> shapeBefore;
    std::unique_ptr<Shape> shapeAfter;
    std::unique_ptr<Shape> shape;
    std::unique_ptr<Shape> previousShape = getSegmentAt(countSegments() - 1);

    // find minimum vertex (lower left corner):
    std::vector<std::unique_ptr<Shape>> segments = getExploded();
    for (size_t i = 0; i < segments.size(); i++) {
        shape = getSegmentAt(i);
        if (!shape) {
            continue;
        }

        if (shape->getLength() < 0.001) {
            continue;
        }

        Vec2d v = shape->getStartPoint();
        if (!minV.isValid() || v.x < minV.x ||
            (v.x == minV.x && v.y < minV.y)) {
            minV = v;
            shapeBefore = std::move(previousShape);
            shapeAfter = std::move(shape);
        }

        previousShape = std::move(shape);
    }

    if (!shapeBefore || !shapeAfter) {
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

std::unique_ptr<Polyline> Polyline::convertArcToLineSegments(int segments) const
{
    std::unique_ptr<Polyline> ret = ShapeFactory::instance()->createPolyline();

    std::vector<std::unique_ptr<Shape>> segs = getExploded();
    for (auto &&seg : segs) {
        if (seg->getShapeType() == NS::Arc) {
            Arc *arc = dynamic_cast<Arc *>(seg.get());
            auto pl = arc->approximateWithLinesTan(arc->getLength() / segments);
            ret->appendShape(pl.get());
        }
        else {
            ret->appendShape(seg.get());
        }
    }

    ret->autoClose();
    return ret;
}

std::unique_ptr<Polyline>
Polyline::convertArcToLineSegmentsLength(double segmentLength) const
{
    std::unique_ptr<Polyline> ret = ShapeFactory::instance()->createPolyline();

    std::vector<std::unique_ptr<Shape>> segs = getExploded();
    for (auto &&seg : segs) {
        if (seg->getShapeType() == NS::Arc) {
            Arc *arc = dynamic_cast<Arc *>(seg.get());
            auto pl = arc->approximateWithLinesTan(segmentLength);
            ret->appendShape(pl.get());
        }
        else {
            ret->appendShape(seg.get());
        }
    }

    ret->autoClose();
    return ret;
}

int Polyline::getSegmentAtDist(double dist)
{
    double total = getLength();
    auto &&segments = getExploded();

    if (segments.empty()) {
        return -1;
    }

    if (dist < 0) {
        dist = std::fmod(dist, total) + total;
    }

    if (isClosed()) {
        dist = std::fmod(dist, total);
        if (dist < 0) {
            dist += total;
        }
    }
    else {
        if (dist < 0 || dist > total) {
            return -1;
        }
    }

    double accumulatedLength = 0.0;
    for (size_t i = 0; i < segments.size(); ++i) {
        accumulatedLength += segments[i]->getLength();
        if (accumulatedLength >= dist) {
            return static_cast<int>(i);
        }
    }
    return -1;
}

bool Polyline::relocateStartPoint(const Vec2d &p)
{
    if (!isGeometricallyClosed()) {
        return false;
    }

    // convert closed to open polyline with start in p:
    // find closest segment of polyline:
    int segmentIndex = getClosestSegment(p);
    if (segmentIndex < 0) {
        return false;
    }

    std::unique_ptr<Polyline> newShape =
        ShapeFactory::instance()->createPolyline();

    auto &&firstSegment = getSegmentAt(segmentIndex);
    auto &&lastSegment = getSegmentAt(segmentIndex);
    if (!firstSegment || !lastSegment)
        return false;

    // trim segment start to p
    firstSegment->trimStartPoint(p);

    // start polyline with second part of split segment:
    newShape->appendShape(firstSegment.get());

    // append rest of polyline:
    for (int i = segmentIndex + 1; i < countSegments(); i++) {
        newShape->appendShape(getSegmentAt(i).get());
    }
    for (int i = 0; i < segmentIndex; i++) {
        newShape->appendShape(getSegmentAt(i).get());
    }

    // trim segment end to p
    lastSegment->trimEndPoint(p);

    // end polyline with second part of split segment:
    newShape->appendShape(lastSegment.get());
    newShape->setClosed(false);

    mVertices = newShape->getVertices();
    mBulges = newShape->getBulges();
    mClosed = newShape->isClosed();
    return true;
}

bool Polyline::relocateStartPoint(double dist)
{
    Vec2d v = getPointAtPercent(dist);
    return relocateStartPoint(v);
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

    auto last = getLastSegment();
    setClosed(false);
    appendShape(last.get());
    return true;
}

bool Polyline::isLineSegment(int i) const
{
    if (i < 0 || i > mBulges.size()) {
        return true;
    }

    return Polyline::isStraight(mBulges.at(i));
}

bool Polyline::isStraight(double bulge) const
{
    return fabs(bulge) < 1.0e-6;
}

std::vector<std::unique_ptr<Shape>> Polyline::getExploded() const
{
    std::vector<std::unique_ptr<Shape>> ret;

    if (mVertices.size() <= 1) {
        return ret;
    }

    for (size_t i = 0; i < mVertices.size(); i++) {
        if (!mClosed && i == mVertices.size() - 1) {
            break;
        }

        auto subShape = getSegmentAt(i);
        if (!subShape) {
            continue;
        }
        ret.emplace_back(subShape.release());
    }

    return ret;
}

bool Polyline::contains(const Vec2d &point, bool borderIsInside,
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
        return algorithm::cada_polylineContains(this, point);
    }

    int nvert = mVertices.size();
    int i, j;
    bool c = false;
    for (i = 0, j = nvert - 1; i < nvert; j = i++) {
        if (((mVertices[i].y > point.y) != (mVertices[j].y > point.y)) &&
            (point.x < (mVertices[j].x - mVertices[i].x) *
                               (point.y - mVertices[i].y) /
                               (mVertices[j].y - mVertices[i].y) +
                           mVertices[i].x)) {
            c = !c;
        }
    }
    return c;
}

int Polyline::countSegments() const
{
    int ret = countVertices();
    if (!mClosed) {
        ret -= 1;
    }
    if (ret < 0) {
        ret = 0;
    }
    return ret;
}

std::unique_ptr<Shape> Polyline::getSegmentAt(int i) const
{
    if (i < 0 || i >= mVertices.size() || i >= mBulges.size()) {
        return nullptr;
    }

    Vec2d p1 = mVertices.at(i);
    Vec2d p2 = mVertices.at((i + 1) % mVertices.size());

    if (Polyline::isStraight(mBulges.at(i))) {
        return ShapeFactory::instance()->createLine(p1, p2);
    }

    else {
        double bulge = mBulges.at(i);
        bool reversed = bulge < 0.0;
        double alpha = atan(bulge) * 4.0;

        if (fabs(alpha) > 2 * M_PI - NS::PointTolerance) {
            return ShapeFactory::instance()->createLine(p1, p2);
        }

        double radius;
        Vec2d center;
        Vec2d middle;
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

        return ShapeFactory::instance()->createArc(center, radius, a1, a2,
                                                   reversed);
    }
}

bool Polyline::isArcSegmentAt(int i) const
{
    if (i < 0 || i >= mBulges.size()) {
        return false;
    }
    return !Polyline::isStraight(mBulges[i]);
}

std::unique_ptr<Shape> Polyline::getLastSegment() const
{
    if (countSegments() == 0) {
        return nullptr;
    }
    return getSegmentAt(countSegments() - 1);
}

std::unique_ptr<Shape> Polyline::getFirstSegment() const
{
    if (countSegments() == 0) {
        return nullptr;
    }
    return getSegmentAt(0);
}

bool Polyline::containsShape(const Shape *shape) const
{
    // check if the shape intersects with any of the polygon edges:
    bool gotIntersection = false;
    if (shape->intersectsWith(this)) {
        gotIntersection = true;
    }

    if (gotIntersection) {
        // normal selection:
        // entity does not match if there is an intersection:
        return false;
    }

    if (shape->getShapeType() == NS::Polyline) {
        const Polyline *pl = dynamic_cast<const Polyline *>(shape);

        BBox bbOuter = pl->getBoundingBox();
        BBox bbInner = shape->getBoundingBox();

        if (!bbOuter.contains(bbInner)) {
            return false;
        }

        for (size_t i = 0; i < pl->countVertices() && i < 5; i++) {
            if (!contains(pl->getVertexAt(i))) {
                return false;
            }
        }
        return true;
    }

    // check if the shape is completely inside the polygon.
    // this is the case if one point on the entity is inside the polygon
    // and the entity does not intersect with the polygon.
    else if (shape->isDirected()) {
        return contains(shape->getStartPoint()) &&
               contains(shape->getEndPoint());
    }
    else {
        // circle:
        if (shape->getShapeType() == NS::Circle) {
            const Circle *circle = dynamic_cast<const Circle *>(shape);
            Vec2d p1 = circle->getCenter() + Vec2d(circle->getRadius(), 0);
            Vec2d p2 = circle->getCenter() + Vec2d(-circle->getRadius(), 0);
            if (contains(p1) || contains(p2)) {
                return true;
            }
            return false;
        }
        else {
            // other shapes:
            Vec2d pointOnShape = shape->getPointOnShape();
            if (contains(pointOnShape, true)) {
                return true;
            }
            return false;
        }
    }

    return false;
}

Vec2d Polyline::getPointInside() const
{
    return Vec2d::invalid;
}

Vec2d Polyline::getStartPoint() const
{
    if (mVertices.size() == 0) {
        return Vec2d::invalid;
    }

    return mVertices.front();
}

Vec2d Polyline::getEndPoint() const
{
    if (mVertices.size() == 0) {
        return Vec2d::invalid;
    }

    if (isClosed()) {
        return mVertices.front();
    }

    return mVertices.back();
}

Vec2d Polyline::getMiddlePoint() const
{
    std::vector<Vec2d> pts =
        getPointsWithDistanceToEnd(getLength() / 2, NS::FromStart);
    if (pts.size() == 1) {
        return pts[0];
    }
    return Vec2d::invalid;
}

void Polyline::moveStartPoint(const Vec2d &pos)
{
    if (mVertices.empty()) {
        return;
    }
    mVertices.front() = pos;
}

void Polyline::moveEndPoint(const Vec2d &pos)
{
    if (mVertices.empty()) {
        return;
    }
    mVertices.back() = pos;
}

void Polyline::moveSegmentAt(int i, const Vec2d &offset)
{
    moveVertexAt(i, offset);
    if (i + 1 < countVertices()) {
        moveVertexAt(i + 1, offset);
    }
    else {
        if (mClosed) {
            moveVertexAt(0, offset);
        }
    }
}

double Polyline::getArea() const
{
    auto &&closedCopy = clone();

    if (!closedCopy->isGeometricallyClosed()) {
        closedCopy->autoClose();
    }

    // polygonal area (all segments treated as lines):
    std::vector<Vec2d> pts = closedCopy->getVertices();
    double area = 0;
    int nPts = closedCopy->countVertices();
    int j = nPts - 1;
    Vec2d p1;
    Vec2d p2;

    for (int i = 0; i < nPts; j = i++) {
        p1 = pts[i];
        p2 = pts[j];
        area += p1.x * p2.y;
        area -= p1.y * p2.x;
    }
    area /= 2;
    area = fabs(area);

    // add / subtract arc segment sector area:
    if (closedCopy->hasArcSegments()) {
        bool plReversed = (closedCopy->getOrientation() == NS::CW);
        for (int i = 0; i < closedCopy->countSegments(); i++) {
            if (closedCopy->isArcSegmentAt(i)) {
                auto &&shape = closedCopy->getSegmentAt(i);
                if (shape->getShapeType() == NS::Arc) {
                    auto &&arc = dynamic_cast<Arc *>(shape.get());
                    double chordArea = arc->getChordArea();

                    if (arc->isReversed() == plReversed) {
                        // arc has same orientation as polyline: add
                        area = area + chordArea;
                    }
                    else {
                        // arc has opposite orientation of polyline: subtract
                        area = area - chordArea;
                    }
                }
            }
        }
    }

    area = fabs(area);
    return area;
}

double Polyline::getLengthTo(const Vec2d &p, bool limited) const
{
    double ret = 0.0;

    if (p.equalsFuzzy(getStartPoint())) {
        return 0.0;
    }

    int segIdx = getClosestSegment(p);
    if (segIdx < 0) {
        return -1.0;
    }

    for (size_t i = 0; i < segIdx; i++) {
        double l = getSegmentAt(i)->getLength();
        if (Math::isNormal(l)) {
            ret += l;
        }
    }

    auto seg = getSegmentAt(segIdx);
    bool lim = limited;
    if (segIdx != 0 && segIdx != countSegments() - 1) {
        lim = true;
    }
    Vec2d p2 = seg->getClosestPointOnShape(p, lim);
    seg->trimEndPoint(p2);
    ret += seg->getLength();

    return ret;
}

double Polyline::getSegmentsLength(int fromIndex, int toIndex) const
{
    double len = 0.0;
    for (size_t i = fromIndex; i < toIndex; i++) {
        auto segment = getSegmentAt(i);
        len += segment->getLength();
    }
    return len;
}

std::vector<Vec2d> Polyline::getEndPoints() const
{
    return mVertices;
}

std::vector<Vec2d> Polyline::getMiddlePoints() const
{
    std::vector<Vec2d> ret;

    std::vector<std::unique_ptr<Shape>> sub = getExploded();
    std::vector<std::unique_ptr<Shape>>::iterator it;
    for (it = sub.begin(); it != sub.end(); ++it) {
        auto vt = (*it)->getMiddlePoints();
        ret.insert(ret.begin(), vt.begin(), vt.end());
    }

    return ret;
}

std::vector<Vec2d> Polyline::getCenterPoints() const
{
    std::vector<Vec2d> ret;

    std::vector<std::unique_ptr<Shape>> sub = getExploded();
    std::vector<std::unique_ptr<Shape>>::iterator it;
    for (it = sub.begin(); it != sub.end(); ++it) {
        auto vt = (*it)->getCenterPoints();
        ret.insert(ret.begin(), vt.begin(), vt.end());
    }

    return ret;
}

int Polyline::getClosestSegment(const Vec2d &point) const
{
    int ret = -1;
    double minDist = -1;

    for (size_t i = 0; i < countSegments(); i++) {
        auto segment = getSegmentAt(i);
        if (!segment) {
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

int Polyline::getClosestVertex(const Vec2d &point) const
{
    return point.getClosestIndex(getVertices());
}

std::unique_ptr<Polyline>
Polyline::modifyPolylineCorner(const Shape *trimmedShape1, NS::Ending ending1,
                               int segmentIndex1, const Shape *trimmedShape2,
                               NS::Ending ending2, int segmentIndex2,
                               const Shape *cornerShape) const
{
    std::unique_ptr<Shape> segment;

    std::unique_ptr<Polyline> pl = ShapeFactory::instance()->createPolyline();

    if (segmentIndex1 < segmentIndex2 && ending1 == NS::EndingEnd &&
        ending2 == NS::EndingStart) {
        for (size_t i = 0; i < segmentIndex1; i++) {
            segment = getSegmentAt(i);
            pl->appendShape(segment.get());
        }

        pl->appendShapeAuto(trimmedShape1);
        if (cornerShape != NULL) {
            pl->appendShapeAuto(cornerShape);
        }
        pl->appendShapeAuto(trimmedShape2);

        for (size_t i = segmentIndex2 + 1; i < countSegments(); i++) {
            segment = getSegmentAt(i);
            pl->appendShape(segment.get());
        }
    }
    else if (segmentIndex1 > segmentIndex2 && ending1 == NS::EndingStart &&
             ending2 == NS::EndingEnd) {
        for (size_t i = 0; i < segmentIndex2; i++) {
            segment = getSegmentAt(i);
            pl->appendShape(segment.get());
        }

        pl->appendShapeAuto(trimmedShape2);
        if (cornerShape != NULL) {
            pl->appendShapeAuto(cornerShape);
        }
        pl->appendShapeAuto(trimmedShape1);

        for (size_t i = segmentIndex1 + 1; i < countSegments(); i++) {
            segment = getSegmentAt(i);
            pl->appendShape(segment.get());
        }
    }
    else if (segmentIndex1 < segmentIndex2 && ending1 == NS::EndingStart &&
             ending2 == NS::EndingEnd) {
        pl->appendShapeAuto(trimmedShape1);
        for (size_t i = segmentIndex1 + 1; i < segmentIndex2; i++) {
            segment = getSegmentAt(i);
            pl->appendShape(segment.get());
        }
        pl->appendShapeAuto(trimmedShape2);
        if (cornerShape != NULL) {
            pl->appendShapeAuto(cornerShape);
        }
    }
    else if (segmentIndex1 > segmentIndex2 && ending1 == NS::EndingEnd &&
             ending2 == NS::EndingStart) {
        pl->appendShapeAuto(trimmedShape2);
        for (size_t i = segmentIndex2 + 1; i < segmentIndex1; i++) {
            segment = getSegmentAt(i);
            pl->appendShape(segment.get());
        }
        pl->appendShapeAuto(trimmedShape1);
        if (cornerShape != NULL) {
            pl->appendShapeAuto(cornerShape);
        }
    }

    return pl;
}

bool Polyline::isConcave() const
{
    return !getConcaveVertices().empty();
}

std::vector<Vec2d> Polyline::getConvexVertices(bool convex) const
{
    if (!isGeometricallyClosed()) {
        return std::vector<Vec2d>();
    }

    std::unique_ptr<Polyline> pl = clone();
    pl->autoClose();

    NS::Orientation ori = pl->getOrientation();

    std::vector<Vec2d> ret;

    for (size_t i = 0; i < pl->mVertices.size(); i++) {
        int iPrev = Math::absmod(i - 1, pl->mVertices.size());
        std::unique_ptr<Shape> segmentPrev = pl->getSegmentAt(iPrev);
        std::unique_ptr<Shape> segmentNext = pl->getSegmentAt(i);

        double aPrev = segmentPrev->getDirection2() + M_PI;
        double aNext = segmentNext->getDirection1();

        Vec2d pPrev = Vec2d::createPolar(1.0, aPrev);
        Vec2d pNext = Vec2d::createPolar(1.0, aNext);

        double cp = Vec2d::getCrossProduct(pPrev, pNext);

        if (convex) {
            if (ori == NS::CW && cp < 0.0 || ori == NS::CCW && cp > 0.0) {
                ret.push_back(pl->mVertices[i]);
            }
        }
        else {
            if (ori == NS::CCW && cp < 0.0 || ori == NS::CW && cp > 0.0) {
                ret.push_back(pl->mVertices[i]);
            }
        }
    }

    return ret;
}

std::vector<Vec2d> Polyline::getConcaveVertices() const
{
    return getConvexVertices(false);
}

Vec2d Polyline::getCentroid() const
{
    if (hasArcSegments()) {
        return Vec2d::invalid;
    }

    double xSum = 0;
    double ySum = 0;
    double signedArea = 0;
    int n = mVertices.size();

    for (size_t i = 0; i < n; i++) {
        double x0 = mVertices[i].x;
        double y0 = mVertices[i].y;
        double x1 = mVertices[(i + 1) % n].x;
        double y1 = mVertices[(i + 1) % n].y;

        // calculate the cross product of the edges
        double crossProduct = x0 * y1 - x1 * y0;
        signedArea += crossProduct;
        xSum += (x0 + x1) * crossProduct;
        ySum += (y0 + y1) * crossProduct;
    }

    signedArea *= 0.5;
    double centroidX = xSum / (6.0 * signedArea);
    double centroidY = ySum / (6.0 * signedArea);

    return Vec2d(centroidX, centroidY);
}

bool Polyline::simplify(double angleTolerance)
{
    bool ret = false;
    std::unique_ptr<Polyline> newPolyline =
        ShapeFactory::instance()->createPolyline();

    NS::ShapeType type = NS::Unkonwn;
    double angle = std::numeric_limits<double>::max();
    double radius = std::numeric_limits<double>::max();
    Vec2d center = Vec2d::invalid;

    for (int i = 0; i < countSegments(); i++) {
        auto &&seg = getSegmentAt(i);

        if (seg->getShapeType() == NS::Line) {
            auto &&line = dynamic_cast<Line *>(seg.get());
            if (line->getLength() < NS::PointTolerance) {
                ret = true;
            }
            else {
                double angleDiff = std::fabs(
                    Math::getAngleDifference180(line->getAngle(), angle));
                if (type == NS::Line && angleDiff < angleTolerance) {
                    ret = true;
                }
                else {
                    newPolyline->appendVertex(line->getStartPoint());
                    angle = line->getAngle();
                    type = NS::Line;
                }
            }
            radius = std::numeric_limits<double>::max();
            center = Vec2d::invalid;
        }

        if (seg->getShapeType() == NS::Arc) {
            auto &&arc = dynamic_cast<Arc *>(seg.get());
            // simplify consecutive arcs:
            if (arc->getCenter().equalsFuzzy(center, 0.001) &&
                Math::fuzzyCompare(arc->getRadius(), radius, 0.001)) {
                arc->setStartAngle(
                    arc->getCenter().getAngleTo(newPolyline->getEndPoint()));
                newPolyline->removeLastVertex();
            }
            newPolyline->appendVertex(arc->getStartPoint(), arc->getBulge());
            angle = std::numeric_limits<double>::max();
            radius = arc->getRadius();
            center = arc->getCenter();
        }
    }

    if (isClosed()) {
        newPolyline->setClosed(true);
    }
    else {
        newPolyline->appendVertex(getEndPoint());
    }

    mVertices = newPolyline->getVertices();
    mBulges = newPolyline->getBulges();
    mClosed = newPolyline->isClosed();

    return ret;
}

std::string Polyline::to_string() const
{
    std::stringstream ss;
    ss << std::fixed << std::setprecision(6);
    ss << "Polyline: ";
    ss << "vertices: " << mVertices.size() << ", ";
    ss << "closed: " << mClosed;
    return ss.str();
}

} // namespace shape
} // namespace cada