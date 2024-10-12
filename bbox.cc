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

BBox::BBox() :c1(Vec2d::invalid), c2(Vec2d::invalid) {}

BBox::BBox(double x1, double y1, double x2, double y2) : c1(x1, y1), c2(x2, y2)
{
}

BBox::BBox(const Vec2d &c1, const Vec2d &c2) : c1(c1), c2(c2) {}

BBox::BBox(const Vec2d &center, double range) 
{
    c1 = Vec2d(center.x() - range, center.y() - range);
    c2 = Vec2d(center.x() + range, center.y() + range);
}

BBox::BBox(const Vec2d &center, double width, double height) 
{
    c1 = center - Vec2d(width, height) / 2;
    c2 = center + Vec2d(width, height) / 2;
}

bool BBox::isValid() const { return c1.isValid() && c2.isValid(); }

bool BBox::isSane() const { return c1.isSane() && c2.isSane(); }

bool BBox::equalsFuzzy(const BBox &b, double tol) 
{
    return c1.equalsFuzzy(b.c1, tol) && c2.equalsFuzzy(b.c2, tol);
}

double BBox::width() const { return fabs(c2.x() - c1.x()); }

double BBox::height() const { return fabs(c2.y() - c1.y()); }

double BBox::area() const { return width() * height(); }

Vec2d BBox::size() const { return c2 - c1; }

Vec2d BBox::center() const { return (c1 + c2) / 2.0; }

Vec2d BBox::minimum() const { return Vec2d::getMinimum(c1, c2); }

Vec2d BBox::maximum() const { return Vec2d::getMaximum(c1, c2); }