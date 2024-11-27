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

#ifndef CADA_C_H
#define CADA_C_H

typedef void *CadaShapeH;

typedef enum {
    CADA_POINT,
    CADA_LINE,
    CADA_ARC,
    CADA_CIRCLE,
    CADA_ELLIPSE,
    CADA_POLYLINE,
    CADA_XLINE,
    CADA_RAY,
    CADA_SPLINE,
} CadaShapeType;

typedef struct {
    double x;
    double y;
} CadaRawPoint;

CadaShapeType GetShapeType(CadaShapeH shape);

CadaShapeH CadaShape_CreatePoint_r();
CadaShapeH CadaShape_CreatePointFromXY_r(double x, double y);
CadaShapeH CadaShape_CreateLine_r();
CadaShapeH CadaShape_CreateLineFrom2Point_r(double x1, double y1, double x2,
                                            double y2);
CadaShapeH CadaShape_CreateLineFromPointAngleDistance_r(double x, double y,
                                                        double angle,
                                                        double distance);
CadaShapeH CadaShape_CreateArc_r();
CadaShapeH CadaShape_CreateArcFromAttribute_r(double x, double y, double radius,
                                              double startAngle,
                                              double endAngle, char reversed);
CadaShapeH CadaShape_CreateArcFrom3Points_r(double x1, double y1, double x2,
                                            double y2, double x3, double y3);

#endif
