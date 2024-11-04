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

/* ---------------------------- static functions ---------------------------- */

std::vector<std::unique_ptr<Circle>>
cada_apollonius_solution_from_PPP(const Shape *point1, const Shape *point2,
                                  const Shape *point3);

std::vector<std::unique_ptr<Circle>>
cada_apollonius_solution_from_PPC(const Shape *point1, const Shape *point2,
                                  const Shape *circle);

std::vector<std::unique_ptr<Circle>>
cada_apollonius_solution_from_PPL(const Shape *point1, const Shape *point2,
                                  const Shape *line);

std::vector<std::unique_ptr<Circle>>
cada_apollonius_solution_from_PCC(const Shape *point, const Shape *circle1,
                                  const Shape *circle2);

std::vector<std::unique_ptr<Circle>>
cada_apollonius_solution_from_PLL(const Shape *point, const Shape *line1,
                                  const Shape *line2);

std::vector<std::unique_ptr<Circle>>
cada_apollonius_solution_from_PLC(const Shape *point, const Shape *line,
                                  const Shape *circle);

std::vector<std::unique_ptr<Circle>>
cada_apollonius_solution_from_LLL(const Shape *line1, const Shape *line2,
                                  const Shape *line3);

std::vector<std::unique_ptr<Circle>>
cada_apollonius_solution_from_LLC(const Shape *line1, const Shape *line2,
                                  const Shape *circle);

std::vector<std::unique_ptr<Circle>>
cada_apollonius_solution_from_LCC(const Shape *line, const Shape *circle2,
                                  const Shape *circle3);

std::vector<std::unique_ptr<Circle>>
cada_apollonius_solution_from_CCC(const Shape *circle1, const Shape *circle2,
                                  const Shape *circle3);

/* ---------------------------------- impls --------------------------------- */

std::vector<std::unique_ptr<Circle>> apollonius_solutions(const Shape *shape1,
                                                          const Shape *shape2,
                                                          const Shape *shape3)
{
    std::vector<const Shape *> points;
    std::vector<const Shape *> lines;
    std::vector<const Shape *> circles;

    std::vector<const Shape *> shapes;
    shapes.push_back(shape1);
    shapes.push_back(shape2);
    shapes.push_back(shape3);

    for (size_t i = 0; i < shapes.size(); i++) {
        auto &&s = shapes[i];

        if (cada_is_point_shape(s)) {
            points.push_back(s);
            continue;
        }
        if (cada_is_line_shape(s)) {
            lines.push_back(s);
            continue;
        }
        if (s->getShapeType() == NS::Arc) {
            auto arc = dynamic_cast<const Arc *>(s);
            circles.push_back(
                ShapeFactory::instance()
                    ->createCircle(arc->getCenter(), arc->getRadius())
                    .release());
            continue;
        }
        if (cada_is_circle_shape(s)) {
            circles.push_back(s);
            continue;
        }
    }

    if (points.size() == 3) {
        return cada_apollonius_solution_from_PPP(points[0], points[1],
                                                 points[2]);
    }

    else if (points.size() == 2) {
        if (circles.size() == 1) {
            return cada_apollonius_solution_from_PPC(points[0], points[1],
                                                     circles[0]);
        }
        else if (lines.size() == 1) {
            return cada_apollonius_solution_from_PPL(points[0], points[1],
                                                     lines[0]);
        }
    }

    else if (points.size() == 1) {
        if (circles.size() == 2) {
            return cada_apollonius_solution_from_PCC(points[0], circles[0],
                                                     circles[1]);
        }
        else if (lines.size() == 2) {
            return cada_apollonius_solution_from_PLL(points[0], lines[0],
                                                     lines[1]);
        }
        else if (circles.size() == 1 && lines.size() == 1) {
            return cada_apollonius_solution_from_PLC(points[0], lines[0],
                                                     circles[0]);
        }
    }

    else if (points.size() == 0) {
        if (lines.size() == 3) {
            return cada_apollonius_solution_from_LLL(lines[0], lines[1],
                                                     lines[2]);
        }
        else if (lines.size() == 2 && circles.size() == 1) {
            return cada_apollonius_solution_from_LLC(lines[0], lines[1],
                                                     circles[0]);
        }
        else if (lines.size() == 1 && circles.size() == 2) {
            return cada_apollonius_solution_from_LCC(lines[0], circles[0],
                                                     circles[1]);
        }
        else if (circles.size() == 3) {
            return cada_apollonius_solution_from_CCC(circles[0], circles[1],
                                                     circles[2]);
        }
    }

    return std::vector<std::unique_ptr<Circle>>();
}

/* -------------------------- static function impls ------------------------- */

std::vector<std::unique_ptr<Circle>>
cada_apollonius_solution_from_PPP(const Shape *point1, const Shape *point2,
                                  const Shape *point3)
{
    return std::vector<std::unique_ptr<Circle>>();
}

std::vector<std::unique_ptr<Circle>>
cada_apollonius_solution_from_PPC(const Shape *point1, const Shape *point2,
                                  const Shape *circle)
{
    return std::vector<std::unique_ptr<Circle>>();
}

std::vector<std::unique_ptr<Circle>>
cada_apollonius_solution_from_PPL(const Shape *point1, const Shape *point2,
                                  const Shape *line)
{
    return std::vector<std::unique_ptr<Circle>>();
}

std::vector<std::unique_ptr<Circle>>
cada_apollonius_solution_from_PCC(const Shape *point, const Shape *circle1,
                                  const Shape *circle2)
{
    return std::vector<std::unique_ptr<Circle>>();
}

std::vector<std::unique_ptr<Circle>>
cada_apollonius_solution_from_PLL(const Shape *point, const Shape *line1,
                                  const Shape *line2)
{
    return std::vector<std::unique_ptr<Circle>>();
}

std::vector<std::unique_ptr<Circle>>
cada_apollonius_solution_from_PLC(const Shape *point, const Shape *line,
                                  const Shape *circle)
{
    return std::vector<std::unique_ptr<Circle>>();
}

std::vector<std::unique_ptr<Circle>>
cada_apollonius_solution_from_LLL(const Shape *line1, const Shape *line2,
                                  const Shape *line3)
{
    return std::vector<std::unique_ptr<Circle>>();
}

std::vector<std::unique_ptr<Circle>>
cada_apollonius_solution_from_LLC(const Shape *line1, const Shape *line2,
                                  const Shape *circle)
{
    return std::vector<std::unique_ptr<Circle>>();
}

std::vector<std::unique_ptr<Circle>>
cada_apollonius_solution_from_LCC(const Shape *line, const Shape *circle2,
                                  const Shape *circle3)
{
    return std::vector<std::unique_ptr<Circle>>();
}

std::vector<std::unique_ptr<Circle>>
cada_apollonius_solution_from_CCC(const Shape *circle1, const Shape *circle2,
                                  const Shape *circle3)
{
    return std::vector<std::unique_ptr<Circle>>();
}

} // namespace algorithm
} // namespace cada