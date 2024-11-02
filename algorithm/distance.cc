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

#include "distance.h"

#include <cada_shape.h>
#include <numeric>
#include <limits>
#include <cassert>

using namespace cada::shape;

namespace cada {
namespace algorithm {

DistanceTo::DistanceTo(shape::Shape *shape, bool limited, double strictRange)
    : mShape(shape), mLimited(limited), mStrictRange(strictRange)
{
}

double DistanceTo::operator()(const shape::Vec2d &point) const
{
    assert(mShape);
    if (mShape->getShapeType() == NS::BSpline) {
        return distance(point);
    }
    else {
        Vec2d v = mShape->getVectorTo(point, mLimited, mStrictRange);
        if (v.isValid()) {
            return v.getMagnitude();
        }
        return std::numeric_limits<double>::quiet_NaN();
    }
}

double DistanceTo::distance(const shape::Vec2d &point) const
{
    return 0.0;
}

} // namespace algorithm
} // namespace cada
