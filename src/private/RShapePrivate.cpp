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

#include <cada2d/private/RShapePrivate.h>
#include <cada2d/RArc.h>
#include <cada2d/RCircle.h>

std::vector<std::shared_ptr<RShape>> RShapePrivate::getReversedShapeList(
    const std::vector<std::shared_ptr<RShape>> &shapes)
{
    return std::vector<std::shared_ptr<RShape>>();
}

std::shared_ptr<RShape> RShapePrivate::scaleArc(const RShape &shape,
                                                const RVector &scaleFactors,
                                                const RVector &center)
{
    return std::shared_ptr<RShape>();
}

std::vector<RVector> RShapePrivate::getIntersectionPoints(const RShape &shape1,
                                                          const RShape &shape2,
                                                          bool limited,
                                                          bool same, bool force)
{
    return std::vector<RVector>();
}

std::vector<std::shared_ptr<RShape>>
RShapePrivate::getOffsetArcs(const RShape &shape, double distance, int number,
                             RS::Side side, const RVector &position)
{
    std::vector<std::shared_ptr<RShape>> ret;

    const RArc *arc = dynamic_cast<const RArc *>(&shape);
    const RCircle *circle = dynamic_cast<const RCircle *>(&shape);

    RVector center;
    double radius;
    bool reversed;
    if (arc != NULL) {
        center = arc->getCenter();
        radius = arc->getRadius();
        reversed = arc->isReversed();
    }
    else if (circle != NULL) {
        center = circle->getCenter();
        radius = circle->getRadius();
        reversed = false;
    }
    else {
        return ret;
    }

    std::vector<bool> insides;
    if (position.isValid()) {
        insides.push_back(center.getDistanceTo(position) < radius);
    }
    else {
        if (side == RS::BothSides) {
            insides.push_back(true);
            insides.push_back(false);
        }
        else {
            if (!reversed) {
                if (side == RS::LeftHand) {
                    insides.push_back(true);
                }
                else {
                    insides.push_back(false);
                }
            }
            else {
                if (side == RS::RightHand) {
                    insides.push_back(true);
                }
                else {
                    insides.push_back(false);
                }
            }
        }
    }

    for (size_t i = 0; i < insides.size(); i++) {
        bool inside = insides[i];
        double d = distance;

        if (inside) {
            d *= -1;
        }

        for (int n = 1; n <= number; ++n) {
            std::shared_ptr<RShape> s = std::shared_ptr<RShape>(shape.clone());

            if (arc != NULL) {
                auto &&concentric = std::dynamic_pointer_cast<RArc>(s);
                concentric->setRadius(concentric->getRadius() + d * n);
                if (concentric->getRadius() < 0.0) {
                    break;
                }
            }
            if (circle != NULL) {
                auto &&concentric = std::dynamic_pointer_cast<RCircle>(s);
                concentric->setRadius(concentric->getRadius() + d * n);
                if (concentric->getRadius() < 0.0) {
                    break;
                }
            }
            ret.push_back(s);
        }
    }

    return ret;
}

std::vector<std::shared_ptr<RShape>>
RShapePrivate::getOffsetLines(const RShape &shape, double distance, int number,
                              RS::Side side, const RVector &position)
{
    std::vector<std::shared_ptr<RShape>> ret;

    if (!shape.isDirected()) {
        return ret;
    }

    std::vector<RS::Side> sides;
    if (position.isValid()) {
        sides.push_back(shape.getSideOfPoint(position));
    }
    else {
        if (side == RS::BothSides) {
            sides.push_back(RS::LeftHand);
            sides.push_back(RS::RightHand);
        }
        else {
            sides.push_back(side);
        }
    }

    for (size_t i = 0; i < sides.size(); i++) {
        RS::Side side = sides[i];

        double a;
        if (side == RS::LeftHand) {
            a = shape.getDirection1() + M_PI / 2.0;
        }
        else {
            a = shape.getDirection1() - M_PI / 2.0;
        }

        RVector distanceV;
        for (int n = 1; n <= number; ++n) {
            distanceV.setPolar(distance * n, a);
            RShape *parallel = shape.clone();
            parallel->move(distanceV);
            ret.push_back(std::shared_ptr<RShape>(parallel));
        }
    }
    return ret;
}