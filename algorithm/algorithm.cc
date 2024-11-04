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

std::vector<shape::Vec2d>
calculate_equidistant_points_on_line(const shape::Vec2d &v1,
                                     const shape::Vec2d &v2, size_t n)
{
    std::vector<shape::Vec2d> rets;
    assert(n > 2);

    double dx = v2.x - v1.x;
    double dy = v2.y - v1.y;

    double step_x = dx / (n - 1);
    double step_y = dy / (n - 1);

    for (size_t i = 0; i < n; ++i) {
        rets.emplace_back(Vec2d(v1.x + step_x * i, v1.y + step_y * i));
    }
    return rets;
}

std::vector<shape::Vec2d> calculate_equidistant_distribution_points_on_surface(
    const shape::Vec2d &v1, const shape::Vec2d &v2, const shape::Vec2d &v3,
    const shape::Vec2d &v4, size_t col, size_t row)
{
    assert(col > 2);
    assert(row > 2);

    std::vector<Vec2d> line1_pts =
        calculate_equidistant_points_on_line(v1, v4, row);
    std::vector<Vec2d> line2_pts =
        calculate_equidistant_points_on_line(v2, v3, row);

    assert(line1_pts.size() != 0);
    assert(line2_pts.size() != 0);
    assert(line1_pts.size() == line2_pts.size());

    std::vector<Vec2d> rets;
    for (size_t i = 0; i < line1_pts.size(); ++i) {
        auto &p1 = line1_pts.at(i);
        auto &p2 = line2_pts.at(i);
        auto &&pps = calculate_equidistant_points_on_line(p1, p2, col);
        rets.insert(rets.end(), pps.begin(), pps.end());
    }
    return rets;
}

std::vector<std::unique_ptr<shape::Line>>
calculate_angle_bisector_of_two_line_segments(
    const shape::Line *l1, const shape::Line *l2, const shape::Vec2d &pos1,
    const shape::Vec2d &pos2, double line_length, int line_number)
{
    assert(l1);
    assert(l2);
    assert(line_number > 0);
    assert(line_length > 0);

    auto ips = l1->getIntersectionPoints(l2, false);
    if (ips.empty()) {
        return std::vector<std::unique_ptr<shape::Line>>();
    }

    std::vector<std::unique_ptr<shape::Line>> rets;

    Vec2d ip = ips[0];

    double angle1 = ip.getAngleTo(l1->getClosestPointOnShape(pos1));
    double angle2 = ip.getAngleTo(l2->getClosestPointOnShape(pos2));
    double angleDiff = Math::getAngleDifference(angle1, angle2);
    if (angleDiff > M_PI) {
        angleDiff = angleDiff - 2 * M_PI;
    }

    for (int i = 0; i < line_number; ++i) {
        double angle = angle1 + (angleDiff / (line_number + 1) * i);
        Vec2d vec;
        vec.setPolar(line_length, angle);
        rets.emplace_back(ShapeFactory::instance()->createLine(ip, ip + vec));
    }
    return rets;
}

std::vector<std::unique_ptr<shape::Line>>
calculate_common_tangent_between_two_circles(const shape::Circle *c1,
                                             const shape::Circle *c2)
{
    assert(c1);
    assert(c2);

    Vec2d offs1, offs2;

    Vec2d cc1 = c1->getCenter();
    Vec2d cc2 = c2->getCenter();
    double cr1 = c1->getRadius();
    double cr2 = c2->getRadius();

    double angle1 = cc1.getAngleTo(cc2);
    double dist1 = cc1.getDistanceTo(cc2);
    if (dist1 < 1.0e-6) {
        return std::vector<std::unique_ptr<shape::Line>>();
    }

    std::vector<std::unique_ptr<shape::Line>> tangents;

    // outer tangents:
    double dist2 = cr2 - cr1;
    if (dist1 > dist2) {
        double angle2 = asin(dist2 / dist1);
        double angt1 = angle1 + angle2 + M_PI / 2.0;
        double angt2 = angle1 - angle2 - M_PI / 2.0;
        offs1 = Vec2d();
        offs2 = Vec2d();

        offs1.setPolar(cr1, angt1);
        offs2.setPolar(cr2, angt1);

        tangents.emplace_back(
            ShapeFactory::instance()->createLine(cc1 + offs1, cc2 + offs2));

        offs1.setPolar(cr1, angt2);
        offs2.setPolar(cr2, angt2);

        tangents.emplace_back(
            ShapeFactory::instance()->createLine(cc1 - offs1, cc2 + offs2));
    }

    // inner tangents:
    double dist3 = cr2 + cr1;
    if (dist1 > dist3) {
        double angle3 = asin(dist3 / dist1);
        double angt3 = angle1 + angle3 + M_PI / 2.0;
        double angt4 = angle1 - angle3 - M_PI / 2.0;
        offs1 = Vec2d();
        offs2 = Vec2d();

        offs1.setPolar(cr1, angt3);
        offs2.setPolar(cr2, angt3);

        tangents.emplace_back(
            ShapeFactory::instance()->createLine(cc1 - offs1, cc2 + offs2));

        offs1.setPolar(cr1, angt4);
        offs2.setPolar(cr2, angt4);

        tangents.emplace_back(
            ShapeFactory::instance()->createLine(cc1 - offs1, cc2 + offs2));
    }

    return tangents;
}

} // namespace algorithm
} // namespace cada