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

#ifndef CADA_RS_H
#define CADA_RS_H

#include <cada2d/exports.h>

class CADA_API RS {
public:
    static constexpr auto AngleTolerance = 1.0e-9;
    static constexpr auto PointTolerance = 1.0e-9;

    enum Side {
        NoSide,
        LeftHand,
        RightHand,
        BothSides,
    };

    enum Ending {
        EndingStart,
        EndingEnd,
        EndingNone,
    };

    enum From {
        FromStart = 0x01,
        FromEnd = 0x02,
        FromAny = FromStart | FromEnd,
        AlongPolyline = 0x04,
    };

    enum Orientation {
        UnknownOrientation = -1,
        Any = 0,
        CW,
        CCW,
    };

    enum ShapeType {
        Unkonwn,
        Point,
        Line,
        Arc,
        Circle,
        Ellipse,
        XLine,
        Ray,
        Polyline,
        Spline
    };

    enum PolygonOption {
        WithCenterCorner,
        With2PointsOfSide,
        WithCenterSide,
        WithSideSide,
    };

    enum JoinType { JoinBevel, JoinRound, JoinMiter };
};

#endif // CADA_RS_H