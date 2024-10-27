#include "distance.h"

#include <cada_shape.h>
#include <numeric>
#include <limits>
#include <cassert>

using namespace cada::shape;

namespace cada {
namespace algorithm {

DistanceTo::DistanceTo(shape::Shape *shape, bool limited, double strictRange)
    : mShape(shape), mLimited(limited), mStrictRange(strictRange)
{
}

double DistanceTo::operator()(const shape::Vec2d &point) const
{
    assert(mShape);
    if (mShape->getShapeType() == NS::BSpline) {
        return distance(point);
    }
    else {
        Vec2d v = mShape->getVectorTo(point, mLimited, mStrictRange);
        if (v.isValid()) {
            return v.getMagnitude();
        }
        return std::numeric_limits<double>::quiet_NaN();
    }
}

double DistanceTo::distance(const shape::Vec2d &point) const
{
    return 0.0;
}

} // namespace algorithm
} // namespace cada
