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

namespace cada {
namespace shape {

BBox::BBox() : c1(Vec2d::invalid), c2(Vec2d::invalid)
{
}

BBox::BBox(const Vec2d &c1, const Vec2d &c2) : c1(c1), c2(c2)
{
}

BBox::BBox(double x1, double y1, double x2, double y2) : c1(x1, y1), c2(x2, y2)
{
}

BBox::BBox(const Vec2d &center, double range)
{
    c1 = Vec2d(center.x - range, center.y - range);
    c2 = Vec2d(center.x + range, center.y + range);
}

BBox::BBox(const Vec2d &center, double width, double height)
{
    c1 = center - Vec2d(width, height) / 2;
    c2 = center + Vec2d(width, height) / 2;
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

BBox &BBox::grow(double offset)
{
    Vec2d min = getMinimum();
    Vec2d max = getMaximum();
    min -= Vec2d(offset, offset);
    max += Vec2d(offset, offset);
    c1 = min;
    c2 = max;
    return *this;
}

BBox &BBox::grow(double offsetX, double offsetY)
{
    Vec2d min = getMinimum();
    Vec2d max = getMaximum();
    min -= Vec2d(offsetX, offsetY);
    max += Vec2d(offsetX, offsetY);
    c1 = min;
    c2 = max;
    return *this;
}

void BBox::move(const Vec2d &offset)
{
    c1.move(offset);
    c2.move(offset);
}

double BBox::getWidth() const
{
    return std::fabs(c2.x - c1.x);
}

double BBox::getHeight() const
{
    return std::fabs(c2.y - c1.y);
}

Vec2d BBox::getSize() const
{
    return c2 - c1;
}

double BBox::getArea() const
{
    return getWidth() * getHeight();
}

Vec2d BBox::getCenter() const
{
    return (c1 + c2) / 2.0;
}

Vec2d BBox::getMinimum() const
{
    return Vec2d::getMinimum(c1, c2);
}

Vec2d BBox::getMaximum() const
{
    return Vec2d::getMaximum(c1, c2);
}

Vec2d BBox::getCorner1() const
{
    return c1;
}

void BBox::setCorner1(const Vec2d &v)
{
    c1 = v;
}

Vec2d BBox::getCorner2() const
{
    return c2;
}

void BBox::setCorner2(const Vec2d &v)
{
    c2 = v;
}

bool BBox::isOutside(const BBox &other) const
{
    Vec2d maximum = getMaximum();
    Vec2d minimum = getMinimum();
    Vec2d otherMaximum = other.getMaximum();
    Vec2d otherMinimum = other.getMinimum();

    return (minimum.x > otherMaximum.x || minimum.y > otherMaximum.y ||
            maximum.x < otherMinimum.x || maximum.y < otherMinimum.y);
}

bool BBox::contains(const BBox &other) const
{
    return contains(other.c1) && contains(other.c2);
}

bool BBox::contains(const Vec2d &v) const
{
    Vec2d bMin = getMinimum();
    Vec2d bMax = getMaximum();

    return (v.x >= bMin.x && v.x <= bMax.x && v.y >= bMin.y && v.y <= bMax.y);
}

bool BBox::intersects(const BBox &other) const
{
    Vec2d maximum = getMaximum();
    Vec2d minimum = getMinimum();
    Vec2d otherMaximum = other.getMaximum();
    Vec2d otherMinimum = other.getMinimum();

    if (minimum.x > otherMaximum.x || minimum.y > otherMaximum.y) {
        return false;
    }
    if (maximum.x < otherMinimum.x || maximum.y < otherMinimum.y) {
        return false;
    }

    return true;
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

    Vec2d min = getMinimum();
    Vec2d max = getMaximum();
    Vec2d omin = other.getMinimum();
    Vec2d omax = other.getMaximum();

    c1 = Vec2d::getMinimum(min, omin);
    c2 = Vec2d::getMaximum(max, omax);
}

void BBox::growToInclude(const Vec2d &v)
{
    if (!isValid()) {
        c1 = c2 = v;
        return;
    }

    Vec2d min = Vec2d::getMinimum(getMinimum(), v);
    Vec2d max = Vec2d::getMaximum(getMaximum(), v);
    c1 = min;
    c2 = max;
}

std::vector<Vec2d> BBox::getCorners() const
{
    std::vector<Vec2d> ret;

    ret.push_back(Vec2d(c1.x, c1.y));
    ret.push_back(Vec2d(c2.x, c1.y));
    ret.push_back(Vec2d(c2.x, c2.y));
    ret.push_back(Vec2d(c1.x, c2.y));
    ret.push_back(Vec2d(c1.x, c1.y));
    ret.push_back(Vec2d(c2.x, c1.y));
    ret.push_back(Vec2d(c2.x, c2.y));
    ret.push_back(Vec2d(c1.x, c2.y));

    return ret;
}

bool BBox::operator==(const BBox &other) const
{
    return c1 == other.c1 && c2 == other.c2;
}

} // namespace shape
} // namespace cada