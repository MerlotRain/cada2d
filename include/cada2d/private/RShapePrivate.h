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

#ifndef CAD2D_PRIVATE_RSHAPEPRIVATE_H
#define CAD2D_PRIVATE_RSHAPEPRIVATE_H

#include <cada2d/exports.h>
#include <cada2d/RShape.h>
#include <vector>
#include <memory>

class CADA_API RShapePrivate {
public:
    static std::vector<std::shared_ptr<RShape>>
    getReversedShapeList(const std::vector<std::shared_ptr<RShape>> &shapes);

    static std::shared_ptr<RShape>
    scaleArc(const RShape &shape, const RVector &scaleFactors,
             const RVector &center = RDEFAULT_RVECTOR);

    static std::vector<RVector> getIntersectionPoints(const RShape &shape1,
                                                      const RShape &shape2,
                                                      bool limited = true,
                                                      bool same = false,
                                                      bool force = false);

    static std::vector<std::shared_ptr<RShape>>
    getOffsetArcs(const RShape &shape, double distance, int number,
                  RS::Side side, const RVector &position = RVector::invalid);

    static std::vector<std::shared_ptr<RShape>> 
    getOffsetLines(const RShape& shape, double distance, int number, 
                   RS::Side side, const RVector& position = RVector::invalid);
};

#endif // CAD2D_PRIVATE_RSHAPEPRIVATE_H
