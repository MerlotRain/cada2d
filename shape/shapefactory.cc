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

namespace cada {
namespace shape {

ShapeFactory *g_shapefactory = nullptr;

const ShapeFactory *ShapeFactory::instance()
{
    static std::once_flag flag;
    std::call_once(flag, [&]() { g_shapefactory = new ShapeFactory(); });
    return g_shapefactory;
}

std::unique_ptr<Point> ShapeFactory::createPoint() const
{
    return nullptr;
}

std::unique_ptr<Point> ShapeFactory::createPoint(double x, double y) const
{
    return nullptr;
}

std::unique_ptr<Point> ShapeFactory::createPoint(const Vec2d &point) const
{
    return nullptr;
}

std::unique_ptr<Line> ShapeFactory::createLine() const
{
    return nullptr;
}

std::unique_ptr<Line> ShapeFactory::createLine(double x1, double y1, double x2,
                                               double y2) const
{
    return nullptr;
}

std::unique_ptr<Line> ShapeFactory::createLine(const Vec2d &startPoint,
                                               const Vec2d &endPoint) const
{
    return nullptr;
}

std::unique_ptr<Line> ShapeFactory::createLine(const Vec2d &startPoint,
                                               double angle,
                                               double ditance) const
{
    return nullptr;
}

std::unique_ptr<Polyline> ShapeFactory::createPolyline() const
{
    return nullptr;
}

std::unique_ptr<Polyline> ShapeFactory::createPolyline(
    std::vector<Vec2d> &&vertrices, bool closed, std::vector<double> &&bulges,
    std::vector<double> &&endWidths, std::vector<double> &&startWidths) const
{
    return nullptr;
}

std::unique_ptr<Arc> ShapeFactory::createArc() const
{
    return nullptr;
}

std::unique_ptr<Arc> ShapeFactory::createArc(const Vec2d &center, double radius,
                                             double startAngle, double endAngle,
                                             bool reversed) const
{
    return nullptr;
}

std::unique_ptr<Arc> ShapeFactory::createArc(double cx, double xy,
                                             double radius, double startAngle,
                                             double endAngle,
                                             bool reversed) const
{
    return nullptr;
}

std::unique_ptr<Arc> ShapeFactory::createArcFrom3Point(
    const Vec2d &startPoint, const Vec2d &midPoint, const Vec2d &endPoint) const
{
    return nullptr;
}

std::unique_ptr<Arc> ShapeFactory::createArcFrom2PBulgs(const Vec2d &startPoint,
                                                        const Vec2d &endPoint,
                                                        double bulge) const
{
    return nullptr;
}

std::unique_ptr<Arc>
ShapeFactory::createArcFromTangential(const Vec2d &startPoint, const Vec2d &pos,
                                      double direction, double radius) const
{
    return nullptr;
}

std::unique_ptr<Arc> ShapeFactory::createArcFromBiarc(const Vec2d &startPoint,
                                                      double startDirection,
                                                      const Vec2d &endPoint,
                                                      double endDirection,
                                                      bool secondTry) const
{
    return nullptr;
}

std::unique_ptr<Circle> ShapeFactory::createCircle() const
{
    return nullptr;
}

std::unique_ptr<Circle> ShapeFactory::createCircle(const Vec2d &center,
                                                   double radius) const
{
    return nullptr;
}

std::unique_ptr<Circle> ShapeFactory::createCircle(double cx, double cy,
                                                   double radius) const
{
    return nullptr;
}

std::unique_ptr<Circle>
ShapeFactory::createCircleFrom2Points(const Vec2d &p1, const Vec2d &p2) const
{
    return nullptr;
}

std::unique_ptr<Circle>
ShapeFactory::createCircleFrom3Points(const Vec2d &p1, const Vec2d &p2,
                                      const Vec2d &p3) const
{
    return nullptr;
}

std::unique_ptr<Ellipse> ShapeFactory::createEllipse() const
{
    return nullptr;
}

std::unique_ptr<Ellipse>
ShapeFactory::createEllipse(const Vec2d &center, const Vec2d &majorPoint,
                            double ratio, double startParam, double endParam,
                            bool reversed) const
{
    return nullptr;
}

std::unique_ptr<Ellipse>
ShapeFactory::createEllipseFromInscribed(const Vec2d &p1, const Vec2d &p2,
                                         const Vec2d &p3, const Vec2d &p4,
                                         const Vec2d &centerHint) const
{
    return nullptr;
}

std::unique_ptr<Ellipse>
ShapeFactory::createEllipseFrom4Points(const Vec2d &p1, const Vec2d &p2,
                                       const Vec2d &p3, const Vec2d &p4) const
{
    return nullptr;
}

std::unique_ptr<XLine> ShapeFactory::createXLine() const
{
    return nullptr;
}

std::unique_ptr<XLine>
ShapeFactory::createXLine(const Vec2d &basePoint,
                          const Vec2d &directionVector) const
{
    return nullptr;
}

std::unique_ptr<XLine> ShapeFactory::createXLine(const Vec2d &basePoint,
                                                 double angle,
                                                 double distance) const
{
    return nullptr;
}

std::unique_ptr<Ray> ShapeFactory::createRay() const
{
    return nullptr;
}

std::unique_ptr<Ray> ShapeFactory::createRay(const Vec2d &basePoint,
                                             const Vec2d &directionVector) const
{
    return nullptr;
}

std::unique_ptr<Ray> ShapeFactory::createRay(const Vec2d &basePoint,
                                             double angle,
                                             double distance) const
{
    return nullptr;
}

std::unique_ptr<BSpline> ShapeFactory::createBSpline() const
{
    return nullptr;
}

std::unique_ptr<BSpline>
ShapeFactory::createBSpline(std::vector<Vec2d> &&controlPoints,
                            int degree) const
{
    return nullptr;
}

} // namespace shape
} // namespace cada
