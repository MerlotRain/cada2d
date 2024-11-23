#include "shape_algorithm.h"
#include <assert.h>

using namespace cada::shape;
namespace cada {
namespace shapeAlgorithm {

/* -------------------------- foundation functions -------------------------- */
bool cada__is_circle_shape(const Shape *shape)
{
    assert(shape);
    return shape->getShapeType() == NS::Circle;
}

bool cada__is_ellipse_shape(const Shape *shape)
{
    assert(shape);
    return shape->getShapeType() == NS::Ellipse;
}

bool cada__is_full_ellipse(const Shape *shape)
{
    assert(shape);
    return shape->getShapeType() == NS::Ellipse &&
           dynamic_cast<const Ellipse *>(shape)->isFullEllipse();
}

bool cada__is_xline_shape(const Shape *shape)
{
    assert(shape);
    return shape->getShapeType() == NS::XLine;
}

bool cada__is_line_shape(const Shape *shape)
{
    assert(shape);
    return shape->getShapeType() == NS::Line;
}

bool cada__is_polyline_shape(const Shape *shape)
{
    assert(shape);
    return shape->getShapeType() == NS::Polyline;
}

bool cada__is_arc_shape(const Shape *shape)
{
    assert(shape);
    return shape->getShapeType() == NS::Arc;
}

bool cada__is_geometrically_closed(const Shape *shape)
{
    assert(shape);

    return (shape->getShapeType() == NS::Polyline &&
            dynamic_cast<const Polyline *>(shape)->isGeometricallyClosed()) ||
           (shape->getShapeType() == NS::BSpline &&
            dynamic_cast<const BSpline *>(shape)->isGeometricallyClosed());
}

bool cada__is_spline_shape(const Shape *shape)
{
    assert(shape);
    return shape->getShapeType() == NS::BSpline;
}

bool cada__is_closed(const Shape *shape)
{
    assert(shape);
    return (shape->getShapeType() == NS::Polyline &&
            dynamic_cast<const Polyline *>(shape)->isClosed()) ||
           (shape->getShapeType() == NS::BSpline &&
            dynamic_cast<const BSpline *>(shape)->isClosed());
}

bool cada__is_ray_shape(const Shape *shape)
{
    assert(shape);
    return shape->getShapeType() == NS::Ray;
}

std::vector<std::unique_ptr<Shape>>
breve(const shape::Shape *shp1, const shape::Vec2d &clickPos1,
      const shape::Shape *shp2, const shape::Vec2d &clickPos2, bool trim,
      bool samePolyline, double distance1, double distance2)
{

    std::vector<std::unique_ptr<Shape>> results;

    std::unique_ptr<Shape> shape1(shp1->clone());
    std::unique_ptr<Shape> shape2(shp2->clone());

    // convert circles to arcs:
    if (isCircleShape(shape1.get())) {
        auto &&circle = dynamic_cast<Circle *>(shape1.release());
        shape1 = std::move(circle->toArc());
    }
    if (isCircleShape(shape2.get())) {
        auto &&circle = dynamic_cast<Circle *>(shape2.release());
        shape2 = std::move(circle->toArc());
    }

    std::unique_ptr<Shape> simpleShape1, simpleShape2;
    size_t i1, i2;

    if (isPolylineShape(shape1.get())) {
        auto &&pline = dynamic_cast<Polyline *>(shape1.release());
        i1 = pline->getClosestSegment(clickPos1);
        if (i1 == -1) {
            return results;
        }
        simpleShape1 = pline->getSegmentAt(i1);
    }
    else {
        simpleShape1 = std::move(shape1);
    }

    if (isPolylineShape(shape2.get())) {
        auto &&pline = dynamic_cast<Polyline *>(shape2.release());
        i2 = pline->getClosestSegment(clickPos2);
        if (i2 == -1) {
            return results;
        }
        simpleShape2 = pline->getSegmentAt(i2);
    }
    else {
        simpleShape2 = std::move(shape2);
    }

    // get intersection point(s) between two shapes:
    auto &&sol = simpleShape1->getIntersectionPoints(simpleShape2.get(), false);

    if (sol.size() == 0) {
        return results;
    }

    // var trimmed1 = shape1.clone();
    // var trimmed2 = shape2.clone();
    std::unique_ptr<Shape> trimmed1, trimmed2;
    if (samePolyline) {
        trimmed1 = simpleShape1->clone();
        trimmed2 = simpleShape2->clone();
    }
    else {
        // maybe
        trimmed1 = shape1->clone();
        trimmed2 = shape2->clone();
    }

    // trim shapes to intersection:
    auto start1 = NS::FromAny;
    auto is = clickPos1.getClosest(sol);
    auto ending1 = NS::EndingNone;

    ending1 = trimmed1->getTrimEnd(is, clickPos1);
    switch (ending1) {
    case NS::EndingStart:
        trimmed1 = TrimStartPoint(trimmed1, is, clickPos1);
        start1 = NS::FromStart;
        break;
    case NS::EndingEnd:
        trimmed1 = TrimEndPoint(trimmed1, is, clickPos1);
        if (isRayShape(trimmed1.get())) {
            start1 = NS::FromStart;
            ending1 = NS::EndingStart;
        }
        else {
            start1 = NS::FromEnd;
        }
        break;
    default:
        break;
    }

    auto start2 = NS::FromAny;
    is = clickPos2.getClosest(sol);
    auto ending2 = NS::EndingNone;

    ending2 = trimmed2->getTrimEnd(is, clickPos2);
    switch (ending2) {
    case NS::EndingStart:
        trimmed2 = TrimStartPoint(trimmed2, is, clickPos2);
        start2 = NS::FromStart;
        break;
    case NS::EndingEnd:
        trimmed2 = TrimEndPoint(trimmed2, is, clickPos2);
        if (isRayShape(trimmed2.get())) {
            start2 = NS::FromStart;
            ending2 = NS::EndingStart;
        }
        else {
            start2 = NS::FromEnd;
        }

        break;
    default:
        break;
    }

    // find definitive bevel points:
    auto t1 = trimmed1;
    if (isCircleShape(trimmed1.get())) {
        t1 = ShapeAlgorithms.circleToArc(trimmed1,
                                         t1.getCenter().getAngleTo(is));
        start1 = NS::FromAny;
    }
    auto t2 = trimmed2;
    if (isCircleShape(trimmed2.get())) {
        t2 = ShapeAlgorithms.circleToArc(trimmed2,
                                         t2.getCenter().getAngleTo(is));
        start2 = NS::FromAny;
    }

    auto bp1 = t1->getPointsWithDistanceToEnd(distance1, start1);
    if (bp1.size() == 0) {
        return results;
    }
    bp1 = clickPos2.getClosest(bp1);

    auto bp2 = t2->getPointsWithDistanceToEnd(distance2, start2);
    if (bp2.size() == 0) {
        return results;
    }
    bp2 = clickPos2.getClosest(bp2);

    // final trim:
    if (trim || samePolyline) {
        switch (ending1) {
        case NS::EndingStart:
            trimmed1->trimStartPoint(bp1);
            break;
        case NS::EndingEnd:
            trimmed1->trimEndPoint(bp1);
            break;
        default:
            break;
        }

        switch (ending2) {
        case NS::EndingStart:
            trimmed2->trimStartPoint(bp2);
            break;
        case NS::EndingEnd:
            trimmed2->trimEndPoint(bp2);
            break;
        default:
            break;
        }
    }

    var bevel = new RLine(bp1, bp2);

    if (samePolyline) {
        var pl = shape1.modifyPolylineCorner(trimmed1, ending1, i1, trimmed2,
                                             ending2, i2, bevel);
        return [pl];
    }

    return [ trimmed1, bevel, trimmed2 ];
}

} // namespace shapeAlgorithm
} // namespace cada