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

#include "cada_algorithm.h"
#include <assert.h>

using namespace cada::shape;

namespace cada {
namespace algorithm {

inline bool cada_is_point_shape(const Shape *shape)
{
    assert(shape);
    return (shape->getShapeType() == NS::Point) ? true : false;
}

inline bool cada_is_circle_shape(const Shape *shape)
{
    assert(shape);
    if (shape->getShapeType() == NS::Circle ||
        shape->getShapeType() == NS::Arc) {
        return true;
    }
    return false;
}

inline bool cada_is_line_shape(const Shape *shape)
{
    assert(shape);
    if (shape->getShapeType() == NS::Line || shape->getShapeType() == NS::Ray ||
        shape->getShapeType() == NS::XLine) {
        return true;
    }
    return false;
}

static std::unique_ptr<shape::Point>
make_apollonius_point(const shape::Shape *shp)
{
    assert(shp);
    if (shp->getShapeType() == NS::Point) {
        auto p = dynamic_cast<const Point *>(shp);
        return p->clone();
    }
}

static std::unique_ptr<shape::Line>
make_apollonius_line(const shape::Shape *shp)
{
    assert(shp);
    if (shp->getShapeType() == NS::Line) {
        auto l = dynamic_cast<const Line *>(shp);
        return l->clone();
    }
    else if (shp->getShapeType() == NS::Ray ||
             shp->getShapeType() == NS::XLine) {
        auto l = dynamic_cast<const XLine *>(shp);
        return l->getLineShape();
    }
    else {
        return nullptr;
    }
}

static std::unique_ptr<shape::Circle>
make_apollonius_circle(const shape::Shape *shp)
{
    assert(shp);
    if (shp->getShapeType() == NS::Circle) {
        auto c = dynamic_cast<const Circle *>(shp);
        return c->clone();
    }
    else if (shp->getShapeType() == NS::Arc) {
        auto a = dynamic_cast<const Arc *>(shp);
        return ShapeFactory::instance()->createCircle(a->getCenter(),
                                                      a->getRadius());
    }
    else {
        return nullptr;
    }
}

/* ---------------------------- static functions ---------------------------- */

std::vector<std::unique_ptr<Circle>>
cada_apollonius_solution_from_PPP(const shape::Point *point1,
                                  const shape::Point *point2,
                                  const shape::Point *point3);

std::vector<std::unique_ptr<Circle>>
cada_apollonius_solution_from_PPC(const shape::Point *point1,
                                  const shape::Point *point2,
                                  const shape::Circle *circle);

std::vector<std::unique_ptr<Circle>>
cada_apollonius_solution_from_PPL(const shape::Point *point1,
                                  const shape::Point *point2,
                                  const shape::Line *line);

std::vector<std::unique_ptr<Circle>>
cada_apollonius_solution_from_PCC(const shape::Point *point,
                                  const shape::Circle *circle1,
                                  const shape::Circle *circle2);

std::vector<std::unique_ptr<Circle>>
cada_apollonius_solution_from_PLL(const shape::Point *point,
                                  const shape::Line *line1,
                                  const shape::Line *line2);

std::vector<std::unique_ptr<Circle>>
cada_apollonius_solution_from_PLC(const shape::Point *point,
                                  const shape::Line *line,
                                  const shape::Circle *circle);

std::vector<std::unique_ptr<Circle>>
cada_apollonius_solution_from_LLL(const shape::Line *line1,
                                  const shape::Line *line2,
                                  const shape::Line *line3);

std::vector<std::unique_ptr<Circle>>
cada_apollonius_solution_from_LLC(const shape::Line *line1,
                                  const shape::Line *line2,
                                  const shape::Circle *circle);

std::vector<std::unique_ptr<Circle>>
cada_apollonius_solution_from_LCC(const shape::Line *line,
                                  const shape::Circle *circle2,
                                  const shape::Circle *circle3);

std::vector<std::unique_ptr<Circle>>
cada_apollonius_solution_from_CCC(const shape::Circle *circle1,
                                  const shape::Circle *circle2,
                                  const shape::Circle *circle3);

/* ---------------------------------- impls --------------------------------- */

std::vector<std::unique_ptr<Circle>> apollonius_solutions(const Shape *shape1,
                                                          const Shape *shape2,
                                                          const Shape *shape3)
{
    std::vector<std::unique_ptr<Point>> points;
    std::vector<std::unique_ptr<Line>> lines;
    std::vector<std::unique_ptr<Circle>> circles;

    std::vector<const Shape *> shapes;
    shapes.push_back(shape1);
    shapes.push_back(shape2);
    shapes.push_back(shape3);

    for (size_t i = 0; i < shapes.size(); i++) {
        auto &&s = shapes[i];

        if (cada_is_point_shape(s)) {
            points.push_back(make_apollonius_point(s));
            continue;
        }
        if (cada_is_line_shape(s)) {
            lines.push_back(make_apollonius_line(s));
            continue;
        }
        if (cada_is_circle_shape(s)) {
            circles.push_back(make_apollonius_circle(s));
            continue;
        }
    }

    if (points.size() == 3) {
        return cada_apollonius_solution_from_PPP(
            points[0].release(), points[1].release(), points[2].release());
    }

    else if (points.size() == 2) {
        if (circles.size() == 1) {
            return cada_apollonius_solution_from_PPC(
                points[0].release(), points[1].release(), circles[0].release());
        }
        else if (lines.size() == 1) {
            return cada_apollonius_solution_from_PPL(
                points[0].release(), points[1].release(), lines[0].release());
        }
    }

    else if (points.size() == 1) {
        if (circles.size() == 2) {
            return cada_apollonius_solution_from_PCC(points[0].release(),
                                                     circles[0].release(),
                                                     circles[1].release());
        }
        else if (lines.size() == 2) {
            return cada_apollonius_solution_from_PLL(
                points[0].release(), lines[0].release(), lines[1].release());
        }
        else if (circles.size() == 1 && lines.size() == 1) {
            return cada_apollonius_solution_from_PLC(
                points[0].release(), lines[0].release(), circles[0].release());
        }
    }

    else if (points.size() == 0) {
        if (lines.size() == 3) {
            return cada_apollonius_solution_from_LLL(
                lines[0].release(), lines[1].release(), lines[2].release());
        }
        else if (lines.size() == 2 && circles.size() == 1) {
            return cada_apollonius_solution_from_LLC(
                lines[0].release(), lines[1].release(), circles[0].release());
        }
        else if (lines.size() == 1 && circles.size() == 2) {
            return cada_apollonius_solution_from_LCC(
                lines[0].release(), circles[0].release(), circles[1].release());
        }
        else if (circles.size() == 3) {
            return cada_apollonius_solution_from_CCC(circles[0].release(),
                                                     circles[1].release(),
                                                     circles[2].release());
        }
    }

    return std::vector<std::unique_ptr<Circle>>();
}

/* -------------------------- static function impls ------------------------- */

std::vector<std::unique_ptr<Circle>>
cada_apollonius_solution_from_PPP(const shape::Point *point1,
                                  const shape::Point *point2,
                                  const shape::Point *point3)
{
    assert(point1);
    assert(point2);
    assert(point3);

    std::vector<std::unique_ptr<Circle>> res;
    auto c = ShapeFactory::instance()->createCircleFrom3Points(
        point1->getPosition(), point2->getPosition(), point3->getPosition());
    res.emplace_back(c);
    return res;
}

std::vector<std::unique_ptr<Circle>>
cada_apollonius_solution_from_PPC(const shape::Point *point1,
                                  const shape::Point *point2,
                                  const shape::Circle *circle)
{
    return std::vector<std::unique_ptr<Circle>>();
}

std::vector<std::unique_ptr<Circle>>
cada_apollonius_solution_from_PPL(const shape::Point *point1,
                                  const shape::Point *point2,
                                  const shape::Line *line)
{
    return std::vector<std::unique_ptr<Circle>>();
}

std::vector<std::unique_ptr<Circle>>
cada_apollonius_solution_from_PCC(const shape::Point *point,
                                  const shape::Circle *circle1,
                                  const shape::Circle *circle2)
{
    return std::vector<std::unique_ptr<Circle>>();
}

std::vector<std::unique_ptr<Circle>>
cada_apollonius_solution_from_PLL(const shape::Point *point,
                                  const shape::Line *line1,
                                  const shape::Line *line2)
{
    return std::vector<std::unique_ptr<Circle>>();
}

std::vector<std::unique_ptr<Circle>>
cada_apollonius_solution_from_PLC(const shape::Point *point,
                                  const shape::Line *line,
                                  const shape::Circle *circle)
{
    return std::vector<std::unique_ptr<Circle>>();
}

std::vector<std::unique_ptr<Circle>>
cada_apollonius_solution_from_LLL(const shape::Line *line1,
                                  const shape::Line *line2,
                                  const shape::Line *line3)
{
    return std::vector<std::unique_ptr<Circle>>();
}

std::vector<std::unique_ptr<Circle>>
cada_apollonius_solution_from_LLC(const shape::Line *line1,
                                  const shape::Line *line2,
                                  const shape::Circle *circle)
{
    return std::vector<std::unique_ptr<Circle>>();
}

std::vector<std::unique_ptr<Circle>>
cada_apollonius_solution_from_LCC(const shape::Line *line,
                                  const shape::Circle *circle2,
                                  const shape::Circle *circle3)
{
    return std::vector<std::unique_ptr<Circle>>();
}

std::vector<std::unique_ptr<Circle>>
cada_apollonius_solution_from_CCC(const shape::Circle *circle1,
                                  const shape::Circle *circle2,
                                  const shape::Circle *circle3)
{
    return std::vector<std::unique_ptr<Circle>>();
}

} // namespace algorithm
} // namespace cada