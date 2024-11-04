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

#ifndef CADA_ALGORITHM_H
#define CADA_ALGORITHM_H

#include "cada_shape.h"

namespace cada {
namespace algorithm {

extern std::vector<shape::Vec2d>
calculate_equidistant_points_on_line(const shape::Vec2d &v1,
                                     const shape::Vec2d &v2, size_t n);

extern std::vector<shape::Vec2d>
calculate_equidistant_distribution_points_on_surface(const shape::Vec2d &v1,
                                                     const shape::Vec2d &v2,
                                                     const shape::Vec2d &v3,
                                                     const shape::Vec2d &v4,
                                                     size_t col, size_t row);

extern std::vector<std::unique_ptr<shape::Line>>
calculate_angle_bisector_of_two_line_segments(
    const shape::Line *l1, const shape::Line *l2, const shape::Vec2d &pos1,
    const shape::Vec2d &pos2, double line_length, int line_number);

extern std::vector<std::unique_ptr<shape::Line>>
calculate_common_tangent_between_two_circles(const shape::Circle *c1,
                                             const shape::Circle *c2);

extern std::vector<shape::Line>
calculate_orthogonal_tangent_between_shape_and_line(shape::Line *line,
                                                    shape::Shape *shape);

extern std::vector<std::unique_ptr<shape::Circle>>
apollonius_solutions(const shape::Shape *shape1, const shape::Shape *shape2,
                     const shape::Shape *shape3);

} // namespace algorithm
} // namespace cada

#endif