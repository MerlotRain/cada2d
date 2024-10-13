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

#include "intcurvecurve.h"

namespace cada {

IntCurveCurve::IntCurveCurve(Shape *shp1, Shape *shp2)
    : mShape1(shp1), mShape2(shp2)
{
    if (!mShape1 || !mShape2)
        throw std::bad_alloc(
            "The input shape paramaters can not be nullptr_t.");
}

std::vector<Vec3d> IntCurveCurve::intersectPoints(bool limited, bool same,
                                                  bool force) const
{
    bool gotInfiniteShape = false;
    if (mShape1->shapeType() == NS::XLine ||
        mShape2->shapeType() == NS::XLine || mShape1->shapeType() == NS::Ray ||
        mShape2->shapeType() == NS::Ray) {
        gotInfiniteShape = true;
    }

    if (limited && !gotInfiniteShape) {
        BBox bb1 = mShape1->getBoundingBox().growXY(1e-2);
        BBox bb2 = mShape2->getBoundingBox().growXY(1e-2);
        if (!bb1.intersects(bb2))
            return std::vector<Vec3d>();
    }

    // case shape1 is line
    {
        const Line *line1 = dynamic_cast<const Line *>(mShape1);
        if (!line1) {
            if (same) {
                return std::vector<Vec3d>();
            }

            const Line *line2 = dynamic_cast<const Line *>(mShape2);
            if (line2) {
                return getIntersectionPointsLL(line1, line2, limited);
            }

            const Arc *arc2 = dynamic_cast<const Arc *>(mShape2);
            if (arc2) {
                return getIntersectionPointsLA(line1, arc2, limited);
            }

            const Circle *circle2 = dynamic_cast<const Circle *>(mShape2);
            if (circle2) {
                return getIntersectionPointsLC(line1, circle2, limited);
            }

            const Ellipse *ellipse2 = dynamic_cast<const Ellipse *>(mShape2);
            if (ellipse2) {
                return getIntersectionPointsLE(line1, ellipse2, limited);
            }

            const Triangle *triangle2 = dynamic_cast<const Triangle *>(mShape2);
            if (triangle2) {
                return getIntersectionPointsLT(line1, triangle2, limited);
            }

            const BSpline *bspline2 = dynamic_cast<const BSpline *>(mShape2);
            if (bspline2) {
                return getIntersectionPointsLS(line1, bspline2, limited);
            }
        }
    }
}

} // namespace cada
