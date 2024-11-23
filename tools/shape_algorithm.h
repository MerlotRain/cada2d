#ifndef SHAPE_ALGORITHM_H
#define SHAPE_ALGORITHM_H

#include <cada_shape.h>
#include <vector>

namespace cada {
namespace shapeAlgorithm {

/* -------------------------- foundation functions -------------------------- */

bool cada__is_circle_shape(const shape::Shape *shape);
bool cada__is_ellipse_shape(const shape::Shape *shape);
bool cada__is_full_ellipse(const shape::Shape *shape);
bool cada__is_xline_shape(const shape::Shape *shape);
bool cada__is_line_shape(const shape::Shape *shape);
bool cada__is_polyline_shape(const shape::Shape *shape);
bool cada__is_arc_shape(const shape::Shape *shape);
bool cada__is_geometrically_closed(const shape::Shape *shape);
bool cada__is_spline_shape(const shape::Shape *shape);
bool cada__is_closed(const shape::Shape *shape);
bool cada__is_ray_shape(const shape::Shape *shape);

#define isCircleShape(shp)              cada__is_circle_shape(shp)
#define isEllipseShape(shp)             cada__is_ellipse_shape(shp)
#define isFullEllipseShape(shp)         cada__is_full_ellipse(shp)
#define isXLineShape(shp)               cada__is_xline_shape(shp)
#define isLineShape(shp)                cada__is_line_shape(shp)
#define isPolylineShape(shp)            cada__is_polyline_shape(shp)
#define isArcShape2(shp)                cada__is_arc_shape(shape)
#define shapeIsGeometricallyClosed(shp) cada__is_geometrically_closed(shp)
#define isSplineShape(shp)              cada__is_spline_shape(shp)
#define shapeIsClosed(shp)              cada__is_closed(shp)
#define isRayShape(shp)                 cada__is_ray_shape(shp)

std::vector<std::unique_ptr<shape::Shape>>
breve(const shape::Shape *shape1, const shape::Vec2d &clickPos1,
      const shape::Shape *shape2, const shape::Vec2d &clickPos2, bool trim,
      bool samePolyline, double distance1, double distance2);

} // namespace shapeAlgorithm
} // namespace cada

#endif