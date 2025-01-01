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

#include <cada2d/RBox.h>
#include <cada2d/RLine.h>
#include <cada2d/RPolyline.h>

RBox::RBox() : c1(RVector::invalid), c2(RVector::invalid) {}

RBox::RBox(const RVector &c1, const RVector &c2) : c1(c1), c2(c2) {}

RBox::RBox(double x1, double y1, double x2, double y2) : c1(x1, y1), c2(x2, y2)
{
}

RBox::RBox(const RVector &center, double range)
{
    c1 = RVector(center.x - range, center.y - range);
    c2 = RVector(center.x + range, center.y + range);
}

RBox::RBox(const RVector &center, double width, double height)
{
    c1 = center - RVector(width, height) / 2;
    c2 = center + RVector(width, height) / 2;
}

bool RBox::isValid() const { return (c1.isValid() && c2.isValid()); }

bool RBox::isSane() const { return (c1.isSane() && c2.isSane()); }

bool RBox::equalsFuzzy(const RBox &b, double tol) const
{
    return c1.equalsFuzzy(b.c1, tol) && c2.equalsFuzzy(b.c2, tol);
}

RBox &RBox::grow(double offset)
{
    RVector min = getMinimum();
    RVector max = getMaximum();
    min -= RVector(offset, offset);
    max += RVector(offset, offset);
    c1 = min;
    c2 = max;
    return *this;
}

RBox &RBox::growXY(double offset)
{
    RVector min = getMinimum();
    RVector max = getMaximum();
    min -= RVector(offset, offset);
    max += RVector(offset, offset);
    c1 = min;
    c2 = max;
    return *this;
}

RBox &RBox::growXY(double offsetX, double offsetY)
{
    RVector min = getMinimum();
    RVector max = getMaximum();
    min -= RVector(offsetX, offsetY);
    max += RVector(offsetX, offsetY);
    c1 = min;
    c2 = max;
    return *this;
}

void RBox::move(const RVector &offset)
{
    c1.move(offset);
    c2.move(offset);
}

bool RBox::scaleByReference(const RVector &referencePoint,
                            const RVector &targetPoint, bool keepAspectRatio,
                            bool fromCenter)
{
    RVector oriSize = getSize().getAbsolute();

    // prevent division by 0:
    if (RMath::fuzzyCompare(oriSize.x, 0.0)) oriSize.x = 1;
    if (RMath::fuzzyCompare(oriSize.y, 0.0)) oriSize.y = 1;

    int match = -1;
    if (referencePoint.equalsFuzzy(c1)) match = 1;
    if (referencePoint.equalsFuzzy(c2)) match = 2;

    RVector c3 = RVector(c2.x, c1.y);
    RVector c4 = RVector(c1.x, c2.y);
    if (referencePoint.equalsFuzzy(c3)) match = 3;
    if (referencePoint.equalsFuzzy(c4)) match = 4;

    if (match == -1) { return false; }

    RVector center = getCenter();

    // vector of translation of corner:
    RVector vf;
    switch (match)
    {
        case 1:
            vf = (c2 - targetPoint);
            break;
        case 2:
            vf = (targetPoint - c1);
            break;
        case 3:
            vf = (targetPoint - c4);
            vf.y *= -1;
            break;
        case 4:
            vf = (c3 - targetPoint);
            vf.y *= -1;
            break;
    }
    vf = vf.getDividedComponents(oriSize);

    if (keepAspectRatio)
    {
        if (std::fabs(vf.x) > std::fabs(vf.y))
        {
            if (vf.x * vf.y >= 0.0) { vf.y = vf.x; }
            else { vf.y = -vf.x; }
        }
        else
        {
            if (vf.x * vf.y >= 0.0) { vf.x = vf.y; }
            else { vf.x = -vf.y; }
        }
        // vf.x = vf.y = qMax(vf.x, vf.y);
    }

    switch (match)
    {
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

    if (match == 3 || match == 4)
    {
        c1 = RVector(c4.x, c3.y);
        c2 = RVector(c3.x, c4.y);
    }

    return true;
}

double RBox::getWidth() const { return std::fabs(c2.x - c1.x); }

double RBox::getHeight() const { return std::fabs(c2.y - c1.y); }

RVector RBox::getSize() const { return c2 - c1; }

double RBox::getArea() const { return getWidth() * getHeight(); }

RVector RBox::getCenter() const { return (c1 + c2) / 2.0; }

RVector RBox::getMinimum() const { return RVector::getMinimum(c1, c2); }

RVector RBox::getMaximum() const { return RVector::getMaximum(c1, c2); }

RVector RBox::getCorner1() const { return c1; }

void RBox::setCorner1(const RVector &v) { c1 = v; }

RVector RBox::getCorner2() const { return c2; }

void RBox::setCorner2(const RVector &v) { c2 = v; }

bool RBox::isOutside(const RBox &other) const
{
    RVector maximum = getMaximum();
    RVector minimum = getMinimum();
    RVector otherMaximum = other.getMaximum();
    RVector otherMinimum = other.getMinimum();

    return (minimum.x > otherMaximum.x || minimum.y > otherMaximum.y ||
            maximum.x < otherMinimum.x || maximum.y < otherMinimum.y);
}

bool RBox::isOutsideXY(const RBox &other) const
{
    RVector maximum = getMaximum();
    RVector minimum = getMinimum();
    RVector otherMaximum = other.getMaximum();
    RVector otherMinimum = other.getMinimum();

    return (minimum.x > otherMaximum.x || minimum.y > otherMaximum.y ||
            maximum.x < otherMinimum.x || maximum.y < otherMinimum.y);
}

bool RBox::contains(const RBox &other) const
{
    return other.c1.isInside(*this) && other.c2.isInside(*this);
}

bool RBox::contains(const RVector &v) const { return v.isInside(*this); }

bool RBox::intersects(const RBox &other) const
{
    RVector maximum = getMaximum();
    RVector minimum = getMinimum();
    RVector otherMaximum = other.getMaximum();
    RVector otherMinimum = other.getMinimum();

    if (minimum.x > otherMaximum.x || minimum.y > otherMaximum.y)
    {
        return false;
    }
    if (maximum.x < otherMinimum.x || maximum.y < otherMinimum.y)
    {
        return false;
    }

    return true;
}

void RBox::growToIncludeBoxes(const std::vector<RBox> &others)
{
    for (int i = 0; i < others.size(); i++) { growToInclude(others[i]); }
}

void RBox::growToIncludeBox(const RBox &other) { growToInclude(other); }

void RBox::growToInclude(const RBox &other)
{
    if (!other.isSane()) { return; }

    if (!isValid())
    {
        *this = other;
        return;
    }

    RVector min = getMinimum();
    RVector max = getMaximum();
    RVector omin = other.getMinimum();
    RVector omax = other.getMaximum();

    c1 = RVector::getMinimum(min, omin);
    c2 = RVector::getMaximum(max, omax);
}

void RBox::growToInclude(const RVector &v)
{
    if (!isValid())
    {
        c1 = c2 = v;
        return;
    }

    RVector min = RVector::getMinimum(getMinimum(), v);
    RVector max = RVector::getMaximum(getMaximum(), v);
    c1 = min;
    c2 = max;
}

void RBox::growToIncludePoint(const RVector &v) { growToInclude(v); }

std::vector<RVector> RBox::getCorners() const
{
    std::vector<RVector> ret;

    ret.push_back(RVector(c1.x, c1.y));
    ret.push_back(RVector(c2.x, c1.y));
    ret.push_back(RVector(c2.x, c2.y));
    ret.push_back(RVector(c1.x, c2.y));

    return ret;
}

std::vector<RLine> RBox::getLines() const
{
    std::vector<RLine> ret;

    ret.push_back(RLine(RVector(c1.x, c1.y), RVector(c2.x, c1.y)));
    ret.push_back(RLine(RVector(c2.x, c1.y), RVector(c2.x, c2.y)));
    ret.push_back(RLine(RVector(c2.x, c2.y), RVector(c1.x, c2.y)));
    ret.push_back(RLine(RVector(c1.x, c2.y), RVector(c1.x, c1.y)));

    return ret;
}

RPolyline RBox::getPolyline() const
{
    RPolyline ret;
    ret.appendVertex(RVector(c1.x, c1.y));
    ret.appendVertex(RVector(c2.x, c1.y));
    ret.appendVertex(RVector(c2.x, c2.y));
    ret.appendVertex(RVector(c1.x, c2.y));
    ret.setClosed(true);
    return ret;
}

bool RBox::operator==(const RBox &other) const
{
    return c1 == other.c1 && c2 == other.c2;
}