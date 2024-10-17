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
#include <assert.h>
#include <cmath>

namespace cada {
namespace shape {

static Vec2d projectVertex(const Vec2d &v, double distance, double azimuth,
                           double inclination = 90.0)
{
    const double radsXy = azimuth * M_PI / 180.0;
    double dx = 0.0, dy = 0.0;

    inclination = std::fmod(inclination, 360.0);

    if (inclination == 90.0) {
        dx = distance * std::sin(radsXy);
        dy = distance * std::cos(radsXy);
    }
    else {
        const double radsZ = inclination * M_PI / 180.0;
        dx = distance * std::sin(radsZ) * std::sin(radsXy);
        dy = distance * std::sin(radsZ) * std::cos(radsXy);
    }

    return Vec2d(v.x + dx, v.y + dy);
}

static double azimuthVertex(const Vec2d &v1, const Vec2d &v2)
{
    const double dx = v2.x - v1.x;
    const double dy = v2.y - v1.y;
    return (std::atan2(dx, dy) * 180.0 / M_PI);
}

RegularPolygon::RegularPolygon() : mNumberSides(0), mRadius(0.0)
{
}

RegularPolygon::RegularPolygon(const Vec2d &center, double radius,
                               double azimuth, unsigned int numberSides,
                               NS::RegularPolygonOption option)
    : mCenter(center)
{
    assert(numberSides >= 3);
    mNumberSides = numberSides;

    switch (option) {
    case NS::InscribedCircle: {
        mRadius = std::fabs(radius);
        mFirstVertex = projectVertex(mCenter, mRadius, azimuth);
        break;
    }
    case NS::CircumscribedCircle: {
        mRadius = apothemToRadius(std::fabs(radius), numberSides);
        mFirstVertex = projectVertex(mCenter, mRadius,
                                     azimuth - centralAngle(numberSides) / 2);
    }
    }
}

RegularPolygon::RegularPolygon(const Vec2d &center, const Vec2d &pt1,
                               unsigned int numberSides,
                               NS::RegularPolygonOption option)
    : mCenter(center)
{
    assert(numberSides >= 3);
    switch (option) {
    case NS::InscribedCircle: {
        mFirstVertex = pt1;
        mRadius = center.getDistanceTo(pt1);
        break;
    }
    case NS::CircumscribedCircle: {
        mRadius = apothemToRadius(center.getDistanceTo(pt1), numberSides);
        const double azimuth = azimuthVertex(center, pt1);
        mFirstVertex = projectVertex(mCenter, mRadius,
                                     azimuth - centralAngle(mNumberSides) / 2);
        break;
    }
    }
}

RegularPolygon::RegularPolygon(const Vec2d &pt1, const Vec2d &pt2,
                               unsigned int numSides)
{
    assert(numSides >= 3);
    mNumberSides = numSides;

    double azimuth = azimuthVertex(pt1, pt2);
    Vec2d pm = Vec2d::getAverage(pt1, pt2);
    double length = pt1.getDistanceTo(pm);

    const double angle = (180 - (360 / numSides)) / 2.0;
    const double hypothenuse = length / std::cos(angle * M_PI / 180);

    mCenter = projectVertex(pt1, hypothenuse, azimuth + angle);
    mFirstVertex = pt1;
    mRadius = std::fabs(hypothenuse);
}

Vec2d RegularPolygon::center() const
{
    return mCenter;
}

double RegularPolygon::radius() const
{
    return mRadius;
}

Vec2d RegularPolygon::firstVertex() const
{
    return mFirstVertex;
}

double RegularPolygon::apothem() const
{
    return mRadius * std::cos(M_PI / mNumberSides);
}

unsigned int RegularPolygon::numberSides() const
{
    return mNumberSides;
}

void RegularPolygon::setCenter(const Vec2d &center)
{
    const double azimuth =
        mFirstVertex.isNaN() ? 0 : azimuthVertex(mCenter, mFirstVertex);
    mCenter = center;
    mFirstVertex = projectVertex(center, mRadius, azimuth);
}

void RegularPolygon::setRadius(double radius)
{
    mRadius = std::fabs(radius);
    const double azimuth =
        mFirstVertex.isNaN() ? 0 : azimuthVertex(mCenter, mFirstVertex);
    mFirstVertex = projectVertex(mCenter, mRadius, azimuth);
}

void RegularPolygon::setFirstVertex(const Vec2d &firstVertex)
{
    const double azimuth = azimuthVertex(mCenter, mFirstVertex);
    mFirstVertex = firstVertex;
    mCenter = projectVertex(mFirstVertex, mRadius, azimuth);
}

void RegularPolygon::setNumberSides(unsigned int numberSides)
{
    mNumberSides = numberSides;
}

std::vector<Vec2d> RegularPolygon::points() const
{
    std::vector<Vec2d> pts;
    double azimuth = azimuthVertex(mCenter, mFirstVertex);
    const double azimuth_add = centralAngle();

    unsigned int n = 1;
    while (n <= mNumberSides) {
        pts.push_back(projectVertex(mCenter, mRadius, azimuth));
        azimuth += azimuth_add;
        if ((azimuth_add > 0) && (azimuth > 180.0)) {
            azimuth -= 360.0;
        }

        n++;
    }

    return pts;
}

std::vector<Line *> RegularPolygon::toLines() const
{
    return std::vector<Line *>();
}

Polyline *RegularPolygon::toPolyline() const
{
    return nullptr;
}

Circle RegularPolygon::inscribedCircle() const
{
    return Circle(mCenter, apothem());
}

Circle RegularPolygon::circumscribedCircle() const
{
    return Circle(mCenter, mRadius);
}

double RegularPolygon::interiorAngle() const
{
    return interiorAngle(mNumberSides);
}

double RegularPolygon::centralAngle() const
{
    return centralAngle(mNumberSides);
}

double RegularPolygon::area() const
{
    return (mRadius * mRadius * mNumberSides *
            std::sin(centralAngle() * M_PI / 180.0)) /
           2;
}

double RegularPolygon::perimeter() const
{
    return length() * mNumberSides;
}

double RegularPolygon::length() const
{
    return mRadius * 2 * std::sin(M_PI / mNumberSides);
}

double RegularPolygon::apothemToRadius(double apothem,
                                       unsigned int numberSides) const
{
    return apothem / std::cos(M_PI / numberSides);
}

double RegularPolygon::interiorAngle(unsigned int nbSides) const
{
    return (nbSides - 2) * 180 / nbSides;
}

double RegularPolygon::centralAngle(unsigned int nbSides) const
{
    return 360.0 / nbSides;
}

} // namespace shape
} // namespace cada