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

#ifndef CADA2D_RSHAPEALGORITHM_H
#define CADA2D_RSHAPEALGORITHM_H

#include <vector>
#include <cada2d/RShape.h>
#include <cada2d/exports.h>

class CADA_API RShapeAlgorithm {

public:
    /**
     * @brief Calculate equidistant points on a line segment
     *
     * @param v1 The start point of the line segment, of type Vec2d
     * @param v2 The end point of the line segment, of type Vec2d
     * @param n The number of equidistant points to calculate
     * @return std::vector<RVector> The set of calculated equidistant
     * points
     */
    static std::vector<RVector>
    calculateEquidistantPointsOnLine(const RVector &v1, const RVector &v2,
                                     size_t n);

    /**
     * @brief Calculate equidistant distribution points on a surface
     *
     * @param v1 The first corner point of the surface, of type Vec2d
     * @param v2 The second corner point of the surface, of type Vec2d
     * @param v3 The third corner point of the surface, of type Vec2d
     * @param v4 The fourth corner point of the surface, of type Vec2d
     * @param col The number of columns of equidistant points to calculate
     * @param row The number of rows of equidistant points to calculate
     * @return std::vector<RVector> The set of calculated equidistant
     * distribution points
     */
    static std::vector<RVector> calculateEquidistantDistributionPointsOnSurface(
        const RVector &v1, const RVector &v2, const RVector &v3,
        const RVector &v4, size_t col, size_t row);

    /**
     * @brief Calculate the angle bisector of two line segments
     *
     * @param v1 The first corner point of the first line segment, of type Vec2d
     * @param v2 The second corner point of the first line segment, of type
     * Vec2d
     * @param v3 The first corner point of the second line segment, of type
     * Vec2d
     * @param v4 The second corner point of the second line segment, of type
     * Vec2d
     * @return std::vector<RVector> The set of calculated angle bisector
     * points
     */
    static std::vector<std::shared_ptr<RLine>>
    calculateAngleBisectorOf2Lines(const RLine &l1, const RLine &l2,
                                   const RVector &pos1, const RVector &pos2,
                                   double line_length, int line_number);

    /**
     * @brief Calculate the common tangent between two circles
     *
     * @param c1 The first circle, of type Circle
     * @param c2 The second circle, of type Circle
     * @return std::vector<std::shared_ptr<RLine>> The set of calculated
     * common tangent lines
     */
    static std::vector<std::shared_ptr<RLine>>
    calculateCommonTangentBetween2Circles(const RCircle &c1, const RCircle &c2);

    /**
     * @brief Calculate the orthogonal tangent between a shape and a line
     *
     * @param line The line, of type Line
     * @param shape The shape, of type Shape
     * @return std::vector<std::shared_ptr<RLine>> The set of calculated
     * orthogonal tangent lines
     */
    static std::vector<std::shared_ptr<RLine>>
    calculateOrthogonalTangentBetweenShapeAndLine(const RLine &line,
                                                  const RShape &shape);

    /**
     * @brief Automatically split a shape at a given position
     *
     * @param pos The position at which to split the shape, of type Vec2d
     * @param shape The shape to be split, of type Shape
     * @param intersecting_shapes The shapes that intersect with the shape to be
     * split, of type Shape*
     * @param extend A boolean flag indicating whether to extend the shape to
     * the intersection point
     * @return std::vector<std::shared_ptr<RShape>> The set of split
     * shapes
     */
    static std::vector<std::shared_ptr<RShape>>
    autoSplit(const RVector &pos, const RShape &shape,
              const std::vector<std::shared_ptr<RShape>> &intersecting_shapes,
              bool extend);

    /**
     * @brief Automatically split a shape at a given position manually
     *
     * @param shp The shape to be split, of type Shape
     * @param cutDist1 The distance from the start of the shape to the first
     * cutting position
     * @param cutDist2 The distance from the start of the shape to the second
     * cutting position
     * @param cutPos1 The first cutting position, of type Vec2d
     * @param cutPos2 The second cutting position, of type Vec2d
     * @param position The position at which to split the shape, of type Vec2d
     * @param extend A boolean flag indicating whether to extend the shape to
     * the intersection point
     * @return std::vector<std::shared_ptr<RShape>> The set of split
     * shapes
     */
    static std::vector<std::shared_ptr<RShape>>
    autoSplitManual(const RShape &shp, double cutDist1, double cutDist2,
                    RVector cutPos1, RVector cutPos2, const RVector &position,
                    bool extend);

    /**
     * @brief Break out a gap at a given position
     *
     * @param pos The position at which to break out the gap, of type Vec2d
     * @param shape The shape to be broken out, of type Shape
     * @param additional A vector to store additional shapes created during the
     * breaking out process
     * @return bool A boolean flag indicating whether the breaking out process
     * was successful
     */
    static bool breakOutGap(const RVector &pos, const RShape &shape,
                            std::vector<std::shared_ptr<RShape>> &additional);

    /**
     * @brief Calculate the bevel shapes of two shapes.
     *
     * This function takes two shapes (shp1 and shp2) and their click positions
     * (clickPos1 and clickPos2), as well as some additional parameters (trim,
     * samePolyline, distance1 and distance2).
     *
     * The function returns a std::vector<std::shared_ptr<RShape>>, which
     * contains the calculated bevel shapes.
     *
     * @param shp1 The first shape.
     * @param clickPos1 The click position of the first shape.
     * @param shp2 The second shape.
     * @param clickPos2 The click position of the second shape.
     * @param trim A boolean indicating whether to trim the shapes.
     * @param samePolyline A boolean indicating whether the shapes are the same
     * polyline.
     * @param distance1 The distance of the first shape.
     * @param distance2 The distance of the second shape.
     * @return std::vector<std::shared_ptr<RShape>> The calculated bevel
     * shapes.
     */
    static std::vector<std::shared_ptr<RShape>>
    bevelShapes(const RShape &shp1, const RVector &clickPos1,
                const RShape &shp2, const RVector &clickPos2, bool trim,
                bool samePolyline, double distance1, double distance2);

    /**
     * @brief Round the corners of two shapes at a given position
     *
     * @param shp1 The first shape, of type Shape
     * @param pos1 The position of the first shape, of type Vec2d
     * @param shp2 The second shape, of type Shape
     * @param pos2 The position of the second shape, of type Vec2d
     * @param trim A boolean flag indicating whether to trim the shapes
     * @param samepolyline A boolean flag indicating whether the shapes are the
     * same polyline
     * @param radius The radius of the rounding, of type double
     * @param solutionPos The position at which to round the corners, of type
     * Vec2d
     * @return std::vector<std::shared_ptr<RShape>> The set of rounded
     * shapes
     */
    static std::vector<std::shared_ptr<RShape>>
    roundRhapes(const RShape &shp1, const RVector &pos1, const RShape &shp2,
                const RVector &pos2, bool trim, bool samepolyline,
                double radius, const RVector &solutionPos);

    /**
     * @brief Lengthen a shape at a given position
     *
     * @param shape The shape to be lengthened, of type Shape
     * @param position The position at which to lengthen the shape, of type
     * Vec2d
     * @param trim_start A boolean flag indicating whether to trim the start of
     * the shape
     * @param amount The amount to lengthen the shape, of type double
     * @return bool A boolean flag indicating whether the lengthening process
     * was successful
     */
    static bool lengthen(RShape &shape, const RVector &position,
                         bool trim_start, double amount);
    /**
     * @brief Calculate the Apollonius solutions for three given shapes
     *
     * @param shape1 The first shape, of type Shape
     * @param shape2 The second shape, of type Shape
     * @param shape3 The third shape, of type Shape
     * @return std::vector<std::unique_ptr<shape::Circle>> The set of calculated
     * Apollonius solutions
     */
    static std::vector<RCircle> apolloniusSolutions(const RShape &shape1,
                                                    const RShape &shape2,
                                                    const RShape &shape3);
};

#endif /* CADA2D_RSHAPEALGORITHM_H */