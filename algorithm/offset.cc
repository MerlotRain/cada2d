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

#include <cada_shape.h>
#include <assert.h>

using namespace cada::shape;

namespace cada {
namespace algorithm {

std::vector<std::unique_ptr<shape::Shape>>
cada_offset_arc(const shape::Arc *arc, double distance, int number,
                NS::Side side, const shape::Vec2d &position)
{
    assert(arc);
    std::vector<std::unique_ptr<shape::Shape>> ret;

    Vec2d center = arc->getCenter();
    double radius = arc->getRadius();
    bool reversed = arc->isReversed();

    std::vector<bool> insides;
    if (position.isValid()) {
        insides.push_back(center.getDistanceTo(position) < radius);
    }
    else {
        if (side == NS::BothSides) {
            insides.push_back(true);
            insides.push_back(false);
        }
        else {
            if (!reversed) {
                if (side == NS::LeftHand) {
                    insides.push_back(true);
                }
                else {
                    insides.push_back(false);
                }
            }
            else {
                if (side == NS::RightHand) {
                    insides.push_back(true);
                }
                else {
                    insides.push_back(false);
                }
            }
        }
    }

    for (size_t i = 0; i < insides.size(); ++i) {
        bool inside = insides[i];
        double d = distance;

        if (inside) {
            d *= -1.0;
        }

        for (size_t n = 1; n <= number; ++n) {
            std::unique_ptr<Arc> concentric = arc->clone();
            concentric->setRadius(concentric->getRadius() + d * n);
            if (concentric->getRadius() < 0.0) {
                break;
            }
            ret.emplace_back(std::move(concentric));
        }
    }
    return ret;
}

std::vector<std::unique_ptr<shape::Shape>>
cada_offset_circle(const shape::Circle *circle, double distance, int number,
                   NS::Side side, const shape::Vec2d &position)
{
    assert(circle);
    std::vector<std::unique_ptr<shape::Shape>> ret;

    Vec2d center = circle->getCenter();
    double radius = circle->getRadius();
    bool reversed = false;

    std::vector<bool> insides;
    if (position.isValid()) {
        insides.push_back(center.getDistanceTo(position) < radius);
    }
    else {
        if (side == NS::BothSides) {
            insides.push_back(true);
            insides.push_back(false);
        }
        else {
            if (!reversed) {
                if (side == NS::LeftHand) {
                    insides.push_back(true);
                }
                else {
                    insides.push_back(false);
                }
            }
            else {
                if (side == NS::RightHand) {
                    insides.push_back(true);
                }
                else {
                    insides.push_back(false);
                }
            }
        }
    }

    for (size_t i = 0; i < insides.size(); ++i) {
        bool inside = insides[i];
        double d = distance;

        if (inside) {
            d *= -1.0;
        }

        for (size_t n = 1; n <= number; ++n) {
            std::unique_ptr<Circle> concentric = circle->clone();
            concentric->setRadius(concentric->getRadius() + d * n);
            if (concentric->getRadius() < 0.0) {
                break;
            }
            ret.emplace_back(std::move(concentric));
        }
    }
    return ret;
}

std::vector<std::unique_ptr<shape::Shape>>
cada_offset_ellipse(const shape::Ellipse *e, double distance, int number,
                    NS::Side side, const shape::Vec2d &position)
{
    assert(e);
    std::vector<std::unique_ptr<shape::Shape>> ret;

    auto ellipse = e->clone();
    if (!ellipse)
        return ret;

    Vec2d center = ellipse->getCenter();
    if (ellipse->isReversed()) {
        ellipse->reverse();
    }

    std::vector<bool> insides;
    if (position.isValid()) {
        double ang = center.getAngleTo(position) - ellipse->getAngle();
        double t = ellipse->angleToParam(ang);
        Vec2d p = ellipse->getPointAt(t);
        insides.push_back(center.getDistanceTo(position) <
                          center.getDistanceTo(p));
    }
    else {
        if (side == NS::BothSides) {
            insides.push_back(true);
            insides.push_back(false);
        }
        else {
            if (side == NS::LeftHand) {
                insides.push_back(true);
            }
            else {
                insides.push_back(false);
            }
        }
    }

    double a = ellipse->getMajorRadius();
    double b = ellipse->getMinorRadius();

    for (size_t i = 0; i < insides.size(); ++i) {
        bool inside = insides[i];
        double d = distance;
        if (inside) {
            d *= -1;
        }

        for (size_t n = 1; n <= number; ++n) {
            auto pl = ShapeFactory::instance()->createPolyline();

            double endParam = ellipse->getEndParam();
            double startParam = ellipse->getStartParam();
            if (Math::fuzzyCompare(endParam, 0.0)) {
                endParam = 2 * M_PI;
            }

            if (endParam < startParam) {
                endParam += 2 * M_PI;
            }

            double k = d * n;
            double tMax = endParam + 0.1;
            if (ellipse->isFullEllipse()) {
                tMax = endParam;
            }

            for (double t = startParam; t < tMax; t += 0.1) {
                if (t > endParam) {
                    t = endParam;
                }

                double root =
                    sqrt(a * a * pow(sin(t), 2) + b * b * pow(cos(t), 2));
                double x = (a + (b * k) / root) * cos(t);
                double y = (b + (a * k) / root) * sin(t);
                Vec2d v(x, y);
                v.rotate(ellipse->getAngle());
                v.move(center);

                pl->appendVertex(v);
            }

            if (ellipse->isFullEllipse()) {
                pl->setClosed(true);
            }

            ret.emplace_back(std::move(pl));
        }
    }
    return ret;
}

std::vector<std::unique_ptr<shape::Shape>>
cada_offset_lines(const shape::Shape *l, double distance, int number,
                  NS::Side side, const shape::Vec2d &position)
{
    assert(l);
    std::vector<std::unique_ptr<shape::Shape>> ret;

    if (!l->isDirected()) {
        return ret;
    }

    std::vector<NS::Side> sides;
    if (position.isValid()) {
        sides.push_back(l->getSideOfPoint(position));
    }
    else {
        if (side == NS::BothSides) {
            sides.push_back(NS::LeftHand);
            sides.push_back(NS::RightHand);
        }
        else {
            sides.push_back(side);
        }
    }

    for (size_t i = 0; i < sides.size(); ++i) {
        NS::Side side_ = sides[i];
        double a = 0.0;
        if (side_ == NS::LeftHand) {
            a = l->getDirection1() + M_PI / 2.0;
        }
        else {
            a = l->getDirection2() + M_PI / 2.0;
        }

        Vec2d distanceV;
        for (size_t n = 1; n <= number; ++n) {
            distanceV.setPolar(distance * n, a);
            auto parallel = l->clone();
            parallel->move(distanceV);
            ret.emplace_back(std::move(parallel));
        }
    }
    return ret;
}

std::vector<std::unique_ptr<shape::Shape>>
cada_offset_polyline(const shape::Polyline *pline, double distance, int number,
                     NS::Side side, const shape::Vec2d &position)
{
    return std::vector<std::unique_ptr<shape::Shape>>();
}

std::vector<std::unique_ptr<shape::Shape>>
cada_getOffsetShapes(const shape::Shape *shape, double distance, int number,
                     NS::Side side, const shape::Vec2d &position)
{
    assert(shape);
    switch (shape->getShapeType()) {
    case NS::Point:
    case NS::Arc: {
        auto a = dynamic_cast<const shape::Arc *>(shape);
        return cada_offset_arc(a, distance, number, side, position);
    }
    case NS::Circle: {
        auto c = dynamic_cast<const shape::Circle *>(shape);
        return cada_offset_circle(c, distance, number, side, position);
    }
    case NS::Ellipse: {
        auto e = dynamic_cast<const shape::Ellipse *>(shape);
        return cada_offset_ellipse(e, distance, number, side, position);
    }
    case NS::Line:
    case NS::XLine:
    case NS::Ray: {
        return cada_offset_lines(shape, distance, number, side, position);
    }
    case NS::Polyline: {
        auto pline = dynamic_cast<const shape::Polyline *>(shape);
        return cada_offset_polyline(pline, distance, number, side, position);
    }
    case NS::Spline:
        break;
    default:
        break;
    };
    return std::vector<std::unique_ptr<shape::Shape>>();
}

} // namespace algorithm
} // namespace cada
