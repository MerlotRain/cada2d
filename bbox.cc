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

BBox::BBox() : c1(Vec3d::invalid), c2(Vec3d::invalid)
{
}

BBox::BBox(const Vec3d &c1, const Vec3d &c2) : c1(c1), c2(c2)
{
}

BBox::BBox(double x1, double y1, double x2, double y2) : c1(x1, y1), c2(x2, y2)
{
}

BBox::BBox(const Vec3d &center, double range)
{
    c1 = Vec3d(center.getX() - range, center.getY() - range);
    c2 = Vec3d(center.getX() + range, center.getY() + range);
}

BBox::BBox(const Vec3d &center, double width, double height)
{
    c1 = center - Vec3d(width, height) / 2;
    c2 = center + Vec3d(width, height) / 2;
}

bool BBox::isValid() const
{
    return (c1.isValid() && c2.isValid());
}

bool BBox::isSane() const
{
    return (c1.isSane() && c2.isSane());
}

bool BBox::equalsFuzzy(const BBox &b, double tol) const
{
    return c1.equalsFuzzy(b.c1, tol) && c2.equalsFuzzy(b.c2, tol);
}

bool BBox::equalsFuzzy2D(const BBox &b, double tol) const
{
    return c1.equalsFuzzy2D(b.c1, tol) && c2.equalsFuzzy2D(b.c2, tol);
}

BBox &BBox::grow(double offset)
{
    Vec3d min = getMinimum();
    Vec3d max = getMaximum();
    min -= Vec3d(offset, offset, offset);
    max += Vec3d(offset, offset, offset);
    c1 = min;
    c2 = max;
    return *this;
}

BBox &BBox::growXY(double offset)
{
    Vec3d min = getMinimum();
    Vec3d max = getMaximum();
    min -= Vec3d(offset, offset);
    max += Vec3d(offset, offset);
    c1 = min;
    c2 = max;
    return *this;
}

BBox &BBox::growXY(double offsetX, double offsetY)
{
    Vec3d min = getMinimum();
    Vec3d max = getMaximum();
    min -= Vec3d(offsetX, offsetY);
    max += Vec3d(offsetX, offsetY);
    c1 = min;
    c2 = max;
    return *this;
}

void BBox::move(const Vec3d &offset)
{
    c1.move(offset);
    c2.move(offset);
}

bool BBox::scaleByReference(const Vec3d &referencePoint,
                            const Vec3d &targetPoint, bool keepAspectRatio,
                            bool fromCenter)
{
    Vec3d oriSize = getSize().getAbsolute();

    // prevent division by 0:
    if (Math::fuzzyCompare(oriSize.getX(), 0.0))
        oriSize.setX(1);
    if (Math::fuzzyCompare(oriSize.getY(), 0.0))
        oriSize.setY(1);
    if (Math::fuzzyCompare(oriSize.getZ(), 0.0))
        oriSize.setZ(1);

    int match = -1;
    if (referencePoint.equalsFuzzy(c1))
        match = 1;
    if (referencePoint.equalsFuzzy(c2))
        match = 2;

    Vec3d c3 = Vec3d(c2.getX(), c1.getY());
    Vec3d c4 = Vec3d(c1.getX(), c2.getY());
    if (referencePoint.equalsFuzzy(c3))
        match = 3;
    if (referencePoint.equalsFuzzy(c4))
        match = 4;

    if (match == -1) {
        return false;
    }

    Vec3d center = getCenter();

    // vector of translation of corner:
    Vec3d vf;
    switch (match) {
    case 1:
        vf = (c2 - targetPoint);
        break;
    case 2:
        vf = (targetPoint - c1);
        break;
    case 3:
        vf = (targetPoint - c4);
        vf.getY() *= -1;
        break;
    case 4:
        vf = (c3 - targetPoint);
        vf.getY() *= -1;
        break;
    }
    vf = vf.getDividedComponents(oriSize);

    if (keepAspectRatio) {
        if (std::fabs(vf.getX()) > std::fabs(vf.getY())) {
            if (vf.getX() * vf.getY() >= 0.0) {
                vf.getY() = vf.getX();
            }
            else {
                vf.getY() = -vf.getX();
            }
        }
        else {
            if (vf.getX() * vf.getY() >= 0.0) {
                vf.getX() = vf.getY();
            }
            else {
                vf.getX() = -vf.getY();
            }
        }
        // vf.getX() = vf.getY() = qMax(vf.getX(), vf.getY());
    }

    switch (match) {
    case 1:
        c1.scale(vf, c2);
        break;
    case 2:
        c2.scale(vf, c1);
        break;
    case 3:
        c3.scale(vf, c4);
        break;
    case 4:
        c4.scale(vf, c3);
        break;
    }

    if (match == 3 || match == 4) {
        c1 = Vec3d(c4.getX(), c3.getY());
        c2 = Vec3d(c3.getX(), c4.getY());
    }

    return true;
}

double BBox::getWidth() const
{
    return std::fabs(c2.getX() - c1.getX());
}

double BBox::getHeight() const
{
    return std::fabs(c2.getY() - c1.getY());
}

Vec3d BBox::getSize() const
{
    return c2 - c1;
}

double BBox::getArea() const
{
    return getWidth() * getHeight();
}

Vec3d BBox::getCenter() const
{
    return (c1 + c2) / 2.0;
}

Vec3d BBox::getMinimum() const
{
    return Vec3d::getMinimum(c1, c2);
}

Vec3d BBox::getMaximum() const
{
    return Vec3d::getMaximum(c1, c2);
}

Vec3d BBox::getCorner1() const
{
    return c1;
}

void BBox::setCorner1(const Vec3d &v)
{
    c1 = v;
}

Vec3d BBox::getCorner2() const
{
    return c2;
}

void BBox::setCorner2(const Vec3d &v)
{
    c2 = v;
}

bool BBox::isOutside(const BBox &other) const
{
    Vec3d maximum = getMaximum();
    Vec3d minimum = getMinimum();
    Vec3d otherMaximum = other.getMaximum();
    Vec3d otherMinimum = other.getMinimum();

    return (minimum.getX() > otherMaximum.getX() ||
            minimum.getY() > otherMaximum.getY() ||
            minimum.getZ() > otherMaximum.getZ() ||
            maximum.getX() < otherMinimum.getX() ||
            maximum.getY() < otherMinimum.getY() ||
            maximum.getZ() < otherMinimum.getZ());
}

bool BBox::isOutsideXY(const BBox &other) const
{
    Vec3d maximum = getMaximum();
    Vec3d minimum = getMinimum();
    Vec3d otherMaximum = other.getMaximum();
    Vec3d otherMinimum = other.getMinimum();

    return (minimum.getX() > otherMaximum.getX() ||
            minimum.getY() > otherMaximum.getY() ||
            maximum.getX() < otherMinimum.getX() ||
            maximum.getY() < otherMinimum.getY());
}

bool BBox::contains(const BBox &other) const
{
    return other.c1.isInside(*this) && other.c2.isInside(*this);
}

bool BBox::contains(const Vec3d &v) const
{
    return v.isInside(*this);
}

bool BBox::intersects(const BBox &other) const
{
    Vec3d maximum = getMaximum();
    Vec3d minimum = getMinimum();
    Vec3d otherMaximum = other.getMaximum();
    Vec3d otherMinimum = other.getMinimum();

    if (minimum.getX() > otherMaximum.getX() ||
        minimum.getY() > otherMaximum.getY() ||
        minimum.getZ() > otherMaximum.getZ()) {
        return false;
    }
    if (maximum.getX() < otherMinimum.getX() ||
        maximum.getY() < otherMinimum.getY() ||
        maximum.getZ() < otherMinimum.getZ()) {
        return false;
    }

    return true;
}

bool BBox::intersectsWith(const RShape &shape, bool limited) const
{
    if (limited && !intersects(shape.getBoundingBox())) {
        return false;
    }

    std::vector<Line> boxEdges = getLines2d();
    for (int i = 0; i < boxEdges.size(); i++) {
        if (boxEdges[i].intersectsWith(shape, limited)) {
            return true;
        }
    }

    return false;
}

void BBox::growToIncludeBoxes(const std::vector<BBox> &others)
{
    for (int i = 0; i < others.size(); i++) {
        growToInclude(others[i]);
    }
}

void BBox::growToInclude(const BBox &other)
{
    if (!other.isSane()) {
        return;
    }

    if (!isValid()) {
        *this = other;
        return;
    }

    Vec3d min = getMinimum();
    Vec3d max = getMaximum();
    Vec3d omin = other.getMinimum();
    Vec3d omax = other.getMaximum();

    c1 = Vec3d::getMinimum(min, omin);
    c2 = Vec3d::getMaximum(max, omax);
}

void BBox::growToInclude(const Vec3d &v)
{
    if (!isValid()) {
        c1 = c2 = v;
        return;
    }

    Vec3d min = Vec3d::getMinimum(getMinimum(), v);
    Vec3d max = Vec3d::getMaximum(getMaximum(), v);
    c1 = min;
    c2 = max;
}

std::vector<Vec3d> BBox::getCorners() const
{
    std::vector<Vec3d> ret;

    ret.push_back(Vec3d(c1.getX(), c1.getY(), c1.getZ()));
    ret.push_back(Vec3d(c2.getX(), c1.getY(), c1.getZ()));
    ret.push_back(Vec3d(c2.getX(), c2.getY(), c1.getZ()));
    ret.push_back(Vec3d(c1.getX(), c2.getY(), c1.getZ()));
    ret.push_back(Vec3d(c1.getX(), c1.getY(), c2.getZ()));
    ret.push_back(Vec3d(c2.getX(), c1.getY(), c2.getZ()));
    ret.push_back(Vec3d(c2.getX(), c2.getY(), c2.getZ()));
    ret.push_back(Vec3d(c1.getX(), c2.getY(), c2.getZ()));

    return ret;
}

std::vector<Vec3d> BBox::getCorners2d() const
{
    std::vector<Vec3d> ret;

    ret.push_back(Vec3d(c1.getX(), c1.getY()));
    ret.push_back(Vec3d(c2.getX(), c1.getY()));
    ret.push_back(Vec3d(c2.getX(), c2.getY()));
    ret.push_back(Vec3d(c1.getX(), c2.getY()));

    return ret;
}

std::vector<Line> BBox::getLines2d() const
{
    std::vector<Line> ret;

    ret.push_back(
        Line(Vec3d(c1.getX(), c1.getY()), Vec3d(c2.getX(), c1.getY())));
    ret.push_back(
        Line(Vec3d(c2.getX(), c1.getY()), Vec3d(c2.getX(), c2.getY())));
    ret.push_back(
        Line(Vec3d(c2.getX(), c2.getY()), Vec3d(c1.getX(), c2.getY())));
    ret.push_back(
        Line(Vec3d(c1.getX(), c2.getY()), Vec3d(c1.getX(), c1.getY())));

    return ret;
}

Polyline BBox::getPolyline2d() const
{
    Polyline ret;
    ret.appendVertex(Vec3d(c1.getX(), c1.getY()));
    ret.appendVertex(Vec3d(c2.getX(), c1.getY()));
    ret.appendVertex(Vec3d(c2.getX(), c2.getY()));
    ret.appendVertex(Vec3d(c1.getX(), c2.getY()));
    ret.setClosed(true);
    return ret;
}

std::vector<Triangle> BBox::getTriangles() const
{
    std::vector<Triangle> ret;
    std::vector<Vec3d> corners = getCorners();

    // front:
    ret.push_back(Triangle(corners[0], corners[1], corners[5]));
    ret.push_back(Triangle(corners[0], corners[5], corners[4]));

    // right:
    ret.push_back(Triangle(corners[1], corners[2], corners[6]));
    ret.push_back(Triangle(corners[1], corners[6], corners[5]));

    // back:
    ret.push_back(Triangle(corners[2], corners[3], corners[7]));
    ret.push_back(Triangle(corners[2], corners[7], corners[6]));

    // left
    ret.push_back(Triangle(corners[3], corners[0], corners[4]));
    ret.push_back(Triangle(corners[3], corners[4], corners[7]));

    // bottom:
    ret.push_back(Triangle(corners[0], corners[2], corners[1]));
    ret.push_back(Triangle(corners[0], corners[3], corners[2]));

    // top:
    ret.push_back(Triangle(corners[4], corners[5], corners[7]));
    ret.push_back(Triangle(corners[5], corners[6], corners[7]));

    return ret;
}

bool BBox::operator==(const BBox &other) const
{
    return c1 == other.c1 && c2 == other.c2;
}
