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

namespace cartesian_internal {

void solve(const double &a1, const double &a2, const double &a3,
           const double &b1, const double &b2, const double &b3,
           const double &c1, const double &c2, const double &c3,
           const double &d1, const double &d2, const double &d3, double &x,
           double &y, double &z, double &denom)
{
    double ab23 = a3 * b2 - a2 * b3;
    double ab13 = a3 * b1 - a1 * b3;
    double ab12 = a2 * b1 - a1 * b2;

    denom = ab23 * c1 - ab13 * c2 + ab12 * c3;

    double cd23 = c3 * d2 - c2 * d3;
    double cd13 = c3 * d1 - c1 * d3;
    double cd12 = c2 * d1 - c1 * d2;

    x = (b3 * cd12 - b2 * cd13 + b1 * cd23);

    y = (a2 * cd13 - cd12 * a3 - cd23 * a1);

    z = (ab23 * d1 + ab12 * d3 - ab13 * d2);
    if (denom < 0) {
        denom = -denom;
        x = -x;
        y = -y;
        z = -z;
    }
}

void solve(const double &a1, const double &a2, const double &a3,
           const double &b1, const double &b2, const double &b3,
           const double &c1, const double &c2, const double &c3,
           const double &d1, const double &d2, const double &d3, double &x,
           double &y, double &z)
{
    double ab23 = a3 * b2 - a2 * b3;
    double ab13 = a3 * b1 - a1 * b3;
    double ab12 = a2 * b1 - a1 * b2;

    double denom = ab23 * c1 - ab13 * c2 + ab12 * c3;

    double cd23 = c3 * d2 - c2 * d3;
    double cd13 = c3 * d1 - c1 * d3;
    double cd12 = c2 * d1 - c1 * d2;

    x = (b3 * cd12 - b2 * cd13 + b1 * cd23) / denom;

    y = (a2 * cd13 - cd12 * a3 - cd23 * a1) / denom;

    z = (ab23 * d1 + ab12 * d3 - ab13 * d2) / denom;
}

} // namespace cartesian_internal

bool plane_from_points(const Vec3d &p, const Vec3d &q, const Vec3d &r,
                       double &a, double &b, double &c, double &d)
{
    double rpx = p.x - r.x;
    double rpy = p.y - r.y;
    double rpz = p.z - r.z;
    double rqx = q.x - r.x;
    double rqy = q.y - r.y;
    double rqz = q.z - r.z;
    a = rpy * rqz - rqy * rpz;
    b = rpz * rqx - rqz * rpx;
    c = rpx * rqy - rqx * rpy;
    d = -a * r.x - b * r.y - c * r.z;
    return true;
}

Plane::Plane(const Vec3d &p1, const Vec3d &p2, const Vec3d &p3)
{
    plane_from_points(p1, p2, p3, mRep[0], mRep[1], mRep[2], mRep[3]);
}

Plane::Plane(const double &a, const double &b, const double &c, const double &d)
    : mRep({a, b, c, d})
{
}

double Plane::a() const
{
    return mRep[0];
}

double Plane::b() const
{
    return mRep[1];
}

double Plane::c() const
{
    return mRep[2];
}

double Plane::d() const
{
    return mRep[3];
}

Vec3d Plane::point() const
{
    double x = 0;
    double y = 0;
    double z = 0;

    double abs_pa = fabs(mRep[0]);
    double abs_pb = fabs(mRep[1]);
    double abs_pc = fabs(mRep[2]);
    if (abs_pa >= abs_pb && abs_pa >= abs_pc)
        x = -mRep[3] / mRep[0];
    else if (abs_pb >= abs_pa && abs_pb >= abs_pc)
        y = -mRep[3] / mRep[1];
    else
        z = -mRep[3] / mRep[2];
    return Vec3d(x, y, z);
}

Vec3d Plane::base1() const
{
    if (mRep[0] == 0) {
        return Vec3d(1, 0, 0);
    }
    if (mRep[1] == 0) {
        return Vec3d(0, 1, 0);
    }
    if (mRep[2] == 0) {
        return Vec3d(0, 0, 1);
    }

    double a = fabs(mRep[0]);
    double b = fabs(mRep[1]);
    double c = fabs(mRep[2]);

    if (a <= b && a <= c)
        return Vec3d(0, -mRep[2], mRep[1]);

    if (b <= a && b <= c)
        return Vec3d(-mRep[2], 0, mRep[0]);

    return Vec3d(-mRep[1], mRep[0], 0);
}

Vec3d Plane::base2() const
{
    Vec3d v = orthogonalVector();
    Vec3d w = base1();
    return Vec3d(v.y * w.z - v.z * w.y, v.z * w.x - v.x * w.z,
                 v.x * w.y - v.y * w.x);
}
Vec3d Plane::orthogonalVector() const
{
    return Vec3d(mRep[0], mRep[1], mRep[2]);
}

bool Plane::on(const Vec3d &p) const
{
    double a = mRep[0];
    double b = mRep[1];
    double c = mRep[2];
    double d = mRep[3];
    double s = a * p.x + b * p.y + c * p.z + d;
    return (s == 0);
}
Vec3d Plane::projection(const Vec3d &p) const
{
    double pa = mRep[0];
    double pb = mRep[1];
    double pc = mRep[2];
    double pd = mRep[3];
    double num = pa * p.x + pb * p.y + pc * p.z + pd;
    double den = pa * pa + pb * pb + pc * pc;
    double lambda = num / den;

    double x = p.x - lambda * pa;
    double y = p.y - lambda * pb;
    double z = p.z - lambda * pc;
    return Vec3d(x, y, z);
}

Vec2d Plane::to2d(const Vec3d &p) const
{
    double alpha, beta, gamma;
    Vec3d b1 = base1();
    Vec3d b2 = base2();
    Vec3d ov = orthogonalVector();
    Vec3d d = p - point();

    cartesian_internal::solve(b1.x, b1.y, b1.z, b2.x, b2.y, b2.z, ov.x, ov.y,
                              ov.z, d.x, d.y, d.z, alpha, beta, gamma);

    return Vec2d(alpha, beta);
}

Vec3d Plane::to3d(const Vec3d &p) const
{
    Vec3d sb1 = base1() * p.x;
    Vec3d sb2 = base2() * p.y;
    return point() + sb1 + sb2;
}

} // namespace shape
} // namespace cada