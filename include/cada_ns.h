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

#ifndef CADA_H
#define CADA_H

namespace cada {

class NS {
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

    enum TextFlag {
        NoFlags = 0x000,
        Bold = 0x001,
        Italic = 0x002,
        Simple = 0x004,
        DimensionLabel = 0x008,
        Highlighted = 0x010,
        Backward = 0x020,
        UpsideDown = 0x040
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
        BSpline
    };

    enum DimensionType {
        Angular2L,
        Angular3P,
        ArcLength,
        Diametric,
        Aligned,
        Rotated,
        Ordinate,
        Radial
    };

    enum Easing {
        Linear,
        InQuad,
        OutQuad,
        InOutQuad,
        OutInQuad,
        InCubic,
        OutCubic,
        InOutCubic,
        OutInCubic,
        InQuart,
        OutQuart,
        InOutQuart,
        OutInQuart,
        InQuint,
        OutQuint,
        InOutQuint,
        OutInQuint,
        InSine,
        OutSine,
        InOutSine,
        OutInSine,
        InExpo,
        OutExpo,
        InOutExpo,
        OutInExpo,
        InCirc,
        OutCirc,
        InOutCirc,
        OutInCirc,
        InElastic,
        OutElastic,
        InOutElastic,
        OutInElastic,
        InBack,
        OutBack,
        InOutBack,
        OutInBack,
        InBounce,
        OutBounce,
        InOutBounce,
        OutInBounce
    };

    enum RegularPolygonOption
    {
        InscribedCircle,
        CircumscribedCircle
    };
};

} // namespace cada

#endif